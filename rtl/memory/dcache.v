//==============================================================================
// Module Name: dcache
// Description: 1KB Direct-Mapped L1 Data Cache (D-Cache) with 16-Byte Lines,
//              Byte-Masked Writes, Multi-Cycle Burst Refill & Writeback for RV32I / RV32IM System-on-Chip.
//
// Cache Specifications:
//   - Total Capacity : 1024 Bytes (1 KB)
//   - Line Size      : 16 Bytes (4 x 32-bit words per line)
//   - Total Lines    : 64 Lines (1024 / 16)
//   - Offset         : 4 bits [3:0] ([3:2] word select, [1:0] byte offset)
//   - Index Width    : 6 bits [9:4] (Selects 1 of 64 lines)
//   - Tag Width      : 22 bits [31:10]
//
// Cache Policies:
//   - Write-Back     : Stores update the cache line and mark it dirty. Main
//                      memory is written via 4-word burst when dirty line evicted.
//   - Write-Allocate : On store miss, the 16-byte line is refilled into cache,
//                      then updated with store data (masked by cpu_wstrb) and marked dirty.
//   - Sub-Word Writes: cpu_wstrb[3:0] enables individual byte writes for SB, SH, SW.
//   - MMIO Bypass    : Memory-mapped I/O (UART, CLINT) bypasses the cache directly.
//
// State Machine:
//   - STATE_IDLE      (2'b00): 0-stall hit latency for reads and writes.
//   - STATE_WRITEBACK (2'b01): 4-cycle burst writing dirty line to main memory.
//   - STATE_REFILL    (2'b10): 4-cycle burst reading missing line from main memory.
//   - STATE_DONE      (2'b11): Latches newly allocated word and unfreezes CPU.
//==============================================================================

`timescale 1ns / 1ps

module dcache #(
    parameter CACHE_LINES = 64
)(
    input  wire        clk,
    input  wire        rst_n,

    // CPU Memory Interface (MEM Stage of pipeline_cpu)
    input  wire [31:0] cpu_addr,
    input  wire [31:0] cpu_wdata,
    input  wire [3:0]  cpu_wstrb,
    input  wire        cpu_mem_write,
    input  wire        cpu_mem_read,
    output wire [31:0] cpu_rdata,
    output wire        cpu_stall,
    output wire        cache_hit,

    // Main Memory / MMIO Bus Interface
    output reg  [31:0] mem_addr,
    output reg  [31:0] mem_wdata,
    output reg  [3:0]  mem_wstrb,
    output reg         mem_write,
    output reg         mem_read,
    input  wire [31:0] mem_rdata,

    // Performance & Diagnostic Counters
    output reg  [31:0] hit_count,
    output reg  [31:0] miss_count,
    output reg  [31:0] evict_count
);

    // FSM States
    localparam [1:0] STATE_IDLE      = 2'b00;
    localparam [1:0] STATE_WRITEBACK = 2'b01;
    localparam [1:0] STATE_REFILL    = 2'b10;
    localparam [1:0] STATE_DONE      = 2'b11;

    reg [1:0] state;
    reg [1:0] burst_idx;

    // Cache Storage Arrays (64 Lines x 16 Bytes)
    reg [63:0] valid_array;
    reg [63:0] dirty_array;
    reg [21:0] tag_array   [0:63];
    reg [31:0] data_w0     [0:63];
    reg [31:0] data_w1     [0:63];
    reg [31:0] data_w2     [0:63];
    reg [31:0] data_w3     [0:63];

    // Address Breakdown for 16-byte lines
    wire [1:0]  cpu_word_sel = cpu_addr[3:2];
    wire [5:0]  cpu_index    = cpu_addr[9:4];
    wire [21:0] cpu_tag      = cpu_addr[31:10];

    // Non-cacheable MMIO detection:
    //   UART:  0x80000000
    //   CLINT: 0x02004000 / 0x80000010
    wire is_mmio = (cpu_addr[31:12] == 20'h80000) || (cpu_addr[31:24] == 8'h02);

    // Active CPU memory operation request
    wire cpu_req = cpu_mem_read || cpu_mem_write;

    // Cache Hit Evaluation
    wire line_valid = valid_array[cpu_index];
    wire line_dirty = dirty_array[cpu_index];
    wire tag_match  = (tag_array[cpu_index] == cpu_tag);
    wire is_hit     = cpu_req && !is_mmio && line_valid && tag_match;

    assign cache_hit = is_hit;

    // Selected Word on Hit
    wire [31:0] hit_data = (cpu_word_sel == 2'b00) ? data_w0[cpu_index] :
                           (cpu_word_sel == 2'b01) ? data_w1[cpu_index] :
                           (cpu_word_sel == 2'b10) ? data_w2[cpu_index] :
                                                     data_w3[cpu_index];

    // Latched request registers for multi-cycle miss servicing
    reg [31:0] req_addr;
    reg [31:0] req_wdata;
    reg [3:0]  req_wstrb;
    reg        req_is_write;
    reg [1:0]  req_word_sel;
    reg [5:0]  req_index;
    reg [21:0] req_tag;

    reg [21:0] evict_tag;

    // Combinational CPU Stall
    assign cpu_stall = (state == STATE_WRITEBACK) ||
                       (state == STATE_REFILL)    ||
                       ((state == STATE_IDLE) && cpu_req && !is_mmio && !is_hit);

    // Synchronous Cache Controller & Memory FSM
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= STATE_IDLE;
            burst_idx    <= 2'b00;
            valid_array  <= 64'd0;
            dirty_array  <= 64'd0;
            hit_count    <= 32'd0;
            miss_count   <= 32'd0;
            evict_count  <= 32'd0;
            req_addr     <= 32'd0;
            req_wdata    <= 32'd0;
            req_wstrb    <= 4'd0;
            req_is_write <= 1'b0;
            req_word_sel <= 2'b00;
            req_index    <= 6'd0;
            req_tag      <= 22'd0;
            evict_tag    <= 22'd0;
            for (i = 0; i < 64; i = i + 1) begin
                tag_array[i] <= 22'd0;
                data_w0[i]   <= 32'd0;
                data_w1[i]   <= 32'd0;
                data_w2[i]   <= 32'd0;
                data_w3[i]   <= 32'd0;
            end
        end else begin
            case (state)
                STATE_IDLE: begin
                    if (cpu_req && !is_mmio) begin
                        if (is_hit) begin
                            // --- Single-Cycle Cache Hit (0 Stall Cycles) ---
                            hit_count <= hit_count + 32'd1;
                            if (cpu_mem_write) begin
                                case (cpu_word_sel)
                                    2'b00: begin
                                        if (cpu_wstrb[0]) data_w0[cpu_index][7:0]   <= cpu_wdata[7:0];
                                        if (cpu_wstrb[1]) data_w0[cpu_index][15:8]  <= cpu_wdata[15:8];
                                        if (cpu_wstrb[2]) data_w0[cpu_index][23:16] <= cpu_wdata[23:16];
                                        if (cpu_wstrb[3]) data_w0[cpu_index][31:24] <= cpu_wdata[31:24];
                                    end
                                    2'b01: begin
                                        if (cpu_wstrb[0]) data_w1[cpu_index][7:0]   <= cpu_wdata[7:0];
                                        if (cpu_wstrb[1]) data_w1[cpu_index][15:8]  <= cpu_wdata[15:8];
                                        if (cpu_wstrb[2]) data_w1[cpu_index][23:16] <= cpu_wdata[23:16];
                                        if (cpu_wstrb[3]) data_w1[cpu_index][31:24] <= cpu_wdata[31:24];
                                    end
                                    2'b10: begin
                                        if (cpu_wstrb[0]) data_w2[cpu_index][7:0]   <= cpu_wdata[7:0];
                                        if (cpu_wstrb[1]) data_w2[cpu_index][15:8]  <= cpu_wdata[15:8];
                                        if (cpu_wstrb[2]) data_w2[cpu_index][23:16] <= cpu_wdata[23:16];
                                        if (cpu_wstrb[3]) data_w2[cpu_index][31:24] <= cpu_wdata[31:24];
                                    end
                                    2'b11: begin
                                        if (cpu_wstrb[0]) data_w3[cpu_index][7:0]   <= cpu_wdata[7:0];
                                        if (cpu_wstrb[1]) data_w3[cpu_index][15:8]  <= cpu_wdata[15:8];
                                        if (cpu_wstrb[2]) data_w3[cpu_index][23:16] <= cpu_wdata[23:16];
                                        if (cpu_wstrb[3]) data_w3[cpu_index][31:24] <= cpu_wdata[31:24];
                                    end
                                endcase
                                dirty_array[cpu_index] <= 1'b1;
                            end
                        end else begin
                            // --- Cache Miss Initiated ---
                            miss_count   <= miss_count + 32'd1;
                            req_addr     <= cpu_addr;
                            req_wdata    <= cpu_wdata;
                            req_wstrb    <= cpu_wstrb;
                            req_is_write <= cpu_mem_write;
                            req_word_sel <= cpu_word_sel;
                            req_index    <= cpu_index;
                            req_tag      <= cpu_tag;
                            burst_idx    <= 2'b00;

                            if (line_valid && line_dirty) begin
                                // Existing line in slot is dirty: must evict all 4 words first
                                evict_tag <= tag_array[cpu_index];
                                state     <= STATE_WRITEBACK;
                            end else begin
                                state     <= STATE_REFILL;
                            end
                        end
                    end
                end

                STATE_WRITEBACK: begin
                    // Write back 4 words to main memory sequentially
                    case (burst_idx)
                        2'b00: burst_idx <= 2'b01;
                        2'b01: burst_idx <= 2'b10;
                        2'b10: burst_idx <= 2'b11;
                        2'b11: begin
                            burst_idx              <= 2'b00;
                            evict_count            <= evict_count + 32'd1;
                            dirty_array[req_index] <= 1'b0;
                            state                  <= STATE_REFILL;
                        end
                    endcase
                end

                STATE_REFILL: begin
                    // Fetch 4 words from main memory into cache line
                    case (burst_idx)
                        2'b00: begin
                            data_w0[req_index] <= mem_rdata;
                            burst_idx          <= 2'b01;
                        end
                        2'b01: begin
                            data_w1[req_index] <= mem_rdata;
                            burst_idx          <= 2'b10;
                        end
                        2'b10: begin
                            data_w2[req_index] <= mem_rdata;
                            burst_idx          <= 2'b11;
                        end
                        2'b11: begin
                            burst_idx              <= 2'b00;
                            valid_array[req_index] <= 1'b1;
                            tag_array[req_index]   <= req_tag;

                            if (req_is_write) begin
                                // Write-Allocate: update target word with CPU store data according to byte enables
                                case (req_word_sel)
                                    2'b00: begin
                                        if (req_wstrb[0]) data_w0[req_index][7:0]   <= req_wdata[7:0];
                                        if (req_wstrb[1]) data_w0[req_index][15:8]  <= req_wdata[15:8];
                                        if (req_wstrb[2]) data_w0[req_index][23:16] <= req_wdata[23:16];
                                        if (req_wstrb[3]) data_w0[req_index][31:24] <= req_wdata[31:24];
                                        data_w3[req_index] <= mem_rdata;
                                    end
                                    2'b01: begin
                                        if (req_wstrb[0]) data_w1[req_index][7:0]   <= req_wdata[7:0];
                                        if (req_wstrb[1]) data_w1[req_index][15:8]  <= req_wdata[15:8];
                                        if (req_wstrb[2]) data_w1[req_index][23:16] <= req_wdata[23:16];
                                        if (req_wstrb[3]) data_w1[req_index][31:24] <= req_wdata[31:24];
                                        data_w3[req_index] <= mem_rdata;
                                    end
                                    2'b10: begin
                                        if (req_wstrb[0]) data_w2[req_index][7:0]   <= req_wdata[7:0];
                                        if (req_wstrb[1]) data_w2[req_index][15:8]  <= req_wdata[15:8];
                                        if (req_wstrb[2]) data_w2[req_index][23:16] <= req_wdata[23:16];
                                        if (req_wstrb[3]) data_w2[req_index][31:24] <= req_wdata[31:24];
                                        data_w3[req_index] <= mem_rdata;
                                    end
                                    2'b11: begin
                                        data_w3[req_index][7:0]   <= req_wstrb[0] ? req_wdata[7:0]   : mem_rdata[7:0];
                                        data_w3[req_index][15:8]  <= req_wstrb[1] ? req_wdata[15:8]  : mem_rdata[15:8];
                                        data_w3[req_index][23:16] <= req_wstrb[2] ? req_wdata[23:16] : mem_rdata[23:16];
                                        data_w3[req_index][31:24] <= req_wstrb[3] ? req_wdata[31:24] : mem_rdata[31:24];
                                    end
                                endcase
                                dirty_array[req_index] <= 1'b1;
                            end else begin
                                data_w3[req_index]     <= mem_rdata;
                                dirty_array[req_index] <= 1'b0;
                            end

                            state <= STATE_DONE;
                        end
                    endcase
                end

                STATE_DONE: begin
                    // Unfreezes CPU and returns to idle
                    state <= STATE_IDLE;
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

    // Combinational Memory Bus Master Request Signals
    always @(*) begin
        case (state)
            STATE_WRITEBACK: begin
                mem_addr  = {evict_tag, req_index, burst_idx, 2'b00};
                mem_wstrb = 4'b1111;
                mem_write = 1'b1;
                mem_read  = 1'b0;
                case (burst_idx)
                    2'b00: mem_wdata = data_w0[req_index];
                    2'b01: mem_wdata = data_w1[req_index];
                    2'b10: mem_wdata = data_w2[req_index];
                    2'b11: mem_wdata = data_w3[req_index];
                endcase
            end

            STATE_REFILL: begin
                mem_addr  = {req_tag, req_index, burst_idx, 2'b00};
                mem_wdata = 32'h0000_0000;
                mem_wstrb = 4'b0000;
                mem_write = 1'b0;
                mem_read  = 1'b1;
            end

            STATE_IDLE: begin
                if (is_mmio) begin
                    // Direct non-cacheable MMIO passthrough
                    mem_addr  = cpu_addr;
                    mem_wdata = cpu_wdata;
                    mem_wstrb = cpu_wstrb;
                    mem_write = cpu_mem_write;
                    mem_read  = cpu_mem_read;
                end else begin
                    mem_addr  = 32'h0000_0000;
                    mem_wdata = 32'h0000_0000;
                    mem_wstrb = 4'b0000;
                    mem_write = 1'b0;
                    mem_read  = 1'b0;
                end
            end

            default: begin
                mem_addr  = 32'h0000_0000;
                mem_wdata = 32'h0000_0000;
                mem_wstrb = 4'b0000;
                mem_write = 1'b0;
                mem_read  = 1'b0;
            end
        endcase
    end

    // Refilled Target Word for STATE_DONE
    wire [31:0] refilled_word = (req_word_sel == 2'b00) ? data_w0[req_index] :
                                (req_word_sel == 2'b01) ? data_w1[req_index] :
                                (req_word_sel == 2'b10) ? data_w2[req_index] :
                                                          data_w3[req_index];

    // Combinational CPU Read Data Multiplexer
    assign cpu_rdata = (state == STATE_DONE) ? refilled_word :
                       (is_mmio)             ? mem_rdata :
                       (is_hit)              ? hit_data :
                       32'h0000_0000;

endmodule
