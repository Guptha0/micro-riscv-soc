//==============================================================================
// Module Name: atomics
// Description: RV32A Atomic Memory Extension Controller supporting:
//   - LR.W (Load-Reserved Word)
//   - SC.W (Store-Conditional Word)
//   - Hardware Reservation Register with Bus-Snooping Invalidation
//
// RISC-V Specification:
//   - LR.W: Loads word from rs1, registers address reservation in hardware.
//   - SC.W: Conditionally writes rs2 to rs1 if reservation is valid and matches.
//           Returns 0 in rd on success, non-zero (1) in rd on failure.
//           Clears reservation unconditionally.
//   - Bus Snooping: External writes to reserved address invalidate reservation.
//==============================================================================

`timescale 1ns / 1ps

module atomics (
    input  wire        clk,
    input  wire        rst_n,

    // Core Pipeline Memory Stage Signals
    input  wire        is_lr,           // Asserted when MEM instruction is LR.W
    input  wire        is_sc,           // Asserted when MEM instruction is SC.W
    input  wire        is_store,        // Normal store instruction
    input  wire [31:0] mem_addr,        // Memory access address
    input  wire [31:0] mem_wdata,       // Store data

    // Filtered Memory Control Outputs (to D-Cache)
    output wire        effective_mem_we,
    output wire        effective_mem_re,
    output wire        sc_success,
    output wire [31:0] sc_result,       // 0 = Success, 1 = Failure (written to rd)

    // Bus-Snooping Invalidation Interface
    input  wire        snoop_we,        // External bus or DMA write strobe
    input  wire [31:0] snoop_addr,      // External bus write address

    // Telemetry & Diagnostics
    output wire        res_valid_out,
    output wire [29:0] res_addr_out,
    output reg  [31:0] lr_count,
    output reg  [31:0] sc_success_count,
    output reg  [31:0] sc_fail_count
);

    // Reservation Register Structure (Word-aligned: [31:2])
    reg        reservation_valid;
    reg [29:0] reservation_addr;

    wire [29:0] cpu_word_addr   = mem_addr[31:2];
    wire [29:0] snoop_word_addr = snoop_addr[31:2];

    // Reservation Match Check
    wire reservation_matches = reservation_valid && (reservation_addr == cpu_word_addr);
    wire snoop_matches       = reservation_valid && snoop_we && (reservation_addr == snoop_word_addr);

    // Output assignments
    assign sc_success = is_sc && reservation_matches;
    assign sc_result  = sc_success ? 32'd0 : 32'd1;

    // SC writes to memory ONLY if reservation is valid and matches
    assign effective_mem_we = is_store || sc_success;
    assign effective_mem_re = is_lr;

    assign res_valid_out = reservation_valid;
    assign res_addr_out  = reservation_addr;

    // Synchronous Reservation Management
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reservation_valid <= 1'b0;
            reservation_addr  <= 30'd0;
            lr_count          <= 32'd0;
            sc_success_count  <= 32'd0;
            sc_fail_count     <= 32'd0;
        end else begin
            // 1. Bus-Snooping Invalidation has highest priority
            if (snoop_matches) begin
                reservation_valid <= 1'b0;
            end

            // 2. CPU Atomic Operations
            if (is_lr) begin
                // Register new reservation
                reservation_valid <= 1'b1;
                reservation_addr  <= cpu_word_addr;
                lr_count          <= lr_count + 32'd1;
            end else if (is_sc) begin
                // SC clears reservation unconditionally
                reservation_valid <= 1'b0;
                if (reservation_matches) begin
                    sc_success_count <= sc_success_count + 32'd1;
                end else begin
                    sc_fail_count <= sc_fail_count + 32'd1;
                end
            end else if (is_store && (cpu_word_addr == reservation_addr)) begin
                // Regular store to reserved address invalidates reservation
                reservation_valid <= 1'b0;
            end
        end
    end

endmodule
