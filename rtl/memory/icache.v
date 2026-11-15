//==============================================================================
// Module Name: icache
// Description: 1KB Direct-Mapped L1 Instruction Cache with 16-Byte Lines
//              and Multi-Cycle Burst Refill for RV32I / RV32IM System-on-Chip.
//
// Cache Geometry:
//   - Total Capacity: 1024 Bytes (1 KB)
//   - Line Size     : 16 Bytes (4 x 32-bit instruction words per line)
//   - Total Lines   : 64 Lines (1024 / 16)
//   - Offset        : 4 bits [3:0] ([3:2] word select, [1:0] byte offset)
//   - Index Width   : 6 bits [9:4] (Selects 1 of 64 lines)
//   - Tag Width     : 22 bits [31:10]
//
// Features:
//   - 0-stall single-cycle hit latency on cache hits.
//   - 4-cycle burst refilling on cache miss to fetch the full 16-byte block.
//   - Spatial locality acceleration: 1 miss loads 4 consecutive instructions.
//   - Integrated hit/miss performance counters for profiling.
//==============================================================================

`timescale 1ns / 1ps

module icache #(
    parameter ENABLE_MISS_STALL = 1 // 1: multi-cycle burst refill stall, 0: transparent fill
)(
    input  wire        clk,
    input  wire        rst_n,

    // CPU Instruction Fetch Interface
    input  wire [31:0] cpu_pc,          // Program Counter from CPU
    input  wire        cpu_read_en,     // Instruction fetch enable
    output wire [31:0] cpu_instruction, // Fetched instruction delivered to CPU
    output wire        cache_hit,       // 1 if current PC hits in cache
    output wire        cache_stall,     // 1 if CPU must freeze due to cache miss refill

    // Instruction ROM / Main Memory Interface
    output reg  [31:0] rom_addr,        // Address to external ROM
    input  wire [31:0] rom_instruction, // Instruction read from external ROM

    // Profiling Diagnostics
    output reg  [31:0] hit_count,
    output reg  [31:0] miss_count
);

    // Cache Storage Arrays (64 Lines x 16 Bytes)
    reg [63:0] valid_array;             // 64 valid bits
    reg [21:0] tag_array   [0:63];      // 64 x 22-bit tags
    reg [31:0] data_w0     [0:63];      // Word 0 of line
    reg [31:0] data_w1     [0:63];      // Word 1 of line
    reg [31:0] data_w2     [0:63];      // Word 2 of line
    reg [31:0] data_w3     [0:63];      // Word 3 of line

    // Address Breakdown for 16-byte lines
    wire [1:0]  cpu_word_sel = cpu_pc[3:2];
    wire [5:0]  cpu_index    = cpu_pc[9:4];
    wire [21:0] cpu_tag      = cpu_pc[31:10];

    // Tag comparison & Hit determination
    wire line_valid = valid_array[cpu_index];
    wire tag_match  = (tag_array[cpu_index] == cpu_tag);
    assign cache_hit = ENABLE_MISS_STALL ? (line_valid && tag_match) : 1'b0;

    // Multi-cycle Burst Refill Controller
    localparam [1:0] I_IDLE   = 2'b00;
    localparam [1:0] I_REFILL = 2'b01;
    localparam [1:0] I_DONE   = 2'b10;

    reg [1:0] state;
    reg [1:0] burst_idx;
    reg [5:0] req_index;
    reg [21:0] req_tag;
    reg [1:0] req_word_sel;

    // Cache Hit Word Selection
    wire [31:0] hit_instruction = (cpu_word_sel == 2'b00) ? data_w0[cpu_index] :
                                  (cpu_word_sel == 2'b01) ? data_w1[cpu_index] :
                                  (cpu_word_sel == 2'b10) ? data_w2[cpu_index] :
                                                            data_w3[cpu_index];

    // ROM Address Generation: during refill, burst across words 0..3 of the 16-byte line
    always @(*) begin
        if (state == I_REFILL) begin
            rom_addr = {req_tag, req_index, burst_idx, 2'b00};
        end else begin
            rom_addr = cpu_pc;
        end
    end

    // Synchronous FSM & Storage Management
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= I_IDLE;
            burst_idx    <= 2'b00;
            valid_array  <= 64'd0;
            hit_count    <= 32'd0;
            miss_count   <= 32'd0;
            req_index    <= 6'd0;
            req_tag      <= 22'd0;
            req_word_sel <= 2'b00;
            for (i = 0; i < 64; i = i + 1) begin
                tag_array[i] <= 22'd0;
                data_w0[i]   <= 32'd0;
                data_w1[i]   <= 32'd0;
                data_w2[i]   <= 32'd0;
                data_w3[i]   <= 32'd0;
            end
        end else if (cpu_read_en) begin
            case (state)
                I_IDLE: begin
                    if (cache_hit) begin
                        hit_count <= hit_count + 32'd1;
                    end else begin
                        // Miss detected: record miss and start 4-word burst refill
                        miss_count   <= miss_count + 32'd1;
                        req_index    <= cpu_index;
                        req_tag      <= cpu_tag;
                        req_word_sel <= cpu_word_sel;
                        burst_idx    <= 2'b00;
                        if (ENABLE_MISS_STALL) begin
                            state <= I_REFILL;
                        end else begin
                            // Fast transparent simulation bypass: do not populate uninitialized cache lines
                        end
                    end
                end

                I_REFILL: begin
                    // Latch word corresponding to current burst_idx
                    case (burst_idx)
                        2'b00: begin
                            data_w0[req_index] <= rom_instruction;
                            burst_idx          <= 2'b01;
                        end
                        2'b01: begin
                            data_w1[req_index] <= rom_instruction;
                            burst_idx          <= 2'b10;
                        end
                        2'b10: begin
                            data_w2[req_index] <= rom_instruction;
                            burst_idx          <= 2'b11;
                        end
                        2'b11: begin
                            data_w3[req_index]     <= rom_instruction;
                            valid_array[req_index] <= 1'b1;
                            tag_array[req_index]   <= req_tag;
                            burst_idx              <= 2'b00;
                            state                  <= I_DONE;
                        end
                    endcase
                end

                I_DONE: begin
                    state <= I_IDLE;
                end

                default: state <= I_IDLE;
            endcase
        end
    end

    // Stall CPU during burst refill on a miss
    assign cache_stall = (state == I_REFILL) || ((state == I_IDLE) && cpu_read_en && !cache_hit && ENABLE_MISS_STALL);

    // MUX Output to CPU
    assign cpu_instruction = cache_hit ? hit_instruction : rom_instruction;

endmodule
