//==============================================================================
// Module Name: pc_fetch
// Description: Production-grade 32-bit RISC-V (RV32I) Program Counter & 
//              Instruction Memory (ROM) Unit.
//
// Specifications:
//   - Synchronous 32-bit Program Counter (PC) register:
//       * Increments by 4 (pc <= pc + 4) on each posedge clk during normal execution.
//       * Supports branch and jump target override: when branch_taken is asserted,
//         pc <= next_pc.
//   - Asynchronous active-low reset (rst_n) initializing PC to 32'h00000000.
//   - Embedded Instruction Memory (ROM) with pre-loaded RV32I test program:
//       * Exercises add, sub, and, or, slt, and branch instructions.
//       * Pre-loaded with synthesizable defaults and supports optional $readmemh.
//   - Combinational instruction read:
//       * Output instruction updates combinationally according to pc[31:2].
//==============================================================================

`timescale 1ns / 1ps

module pc_fetch #(
    parameter MEM_DEPTH = 64,       // 64 instruction words (256 bytes)
    parameter HEX_FILE  = ""        // Optional hex file path for custom programs
)(
    input  wire        clk,          // System clock
    input  wire        rst_n,        // Asynchronous active-low reset
    input  wire        branch_taken, // Branch/Jump override enable
    input  wire [31:0] next_pc,      // Target address for branch/jump
    output reg  [31:0] pc,           // Current Program Counter value
    output wire [31:0] instruction   // 32-bit fetched instruction word
);

    //--------------------------------------------------------------------------
    // Synchronous Program Counter Register with Asynchronous Reset
    //--------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc <= 32'h0000_0000;
        end else begin
            if (branch_taken) begin
                pc <= next_pc;
            end else begin
                pc <= pc + 32'd4;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Embedded Instruction Memory (ROM)
    //--------------------------------------------------------------------------
    reg [31:0] imem [0:MEM_DEPTH-1];

    // Word address extraction (bits [31:2] of byte address)
    wire [$clog2(MEM_DEPTH)-1:0] word_addr;
    assign word_addr = pc[$clog2(MEM_DEPTH)+1:2];

    // Combinational instruction readout
    // Returns NOP (addi x0, x0, 0 -> 32'h00000013) if PC exceeds address space
    assign instruction = (pc[31:2] < MEM_DEPTH) ? imem[word_addr] : 32'h0000_0013;

    //--------------------------------------------------------------------------
    // Instruction Memory Pre-load (RV32I Machine Code)
    //--------------------------------------------------------------------------
    integer i;

    initial begin
        // 1. Initialize all memory words to NOP (addi x0, x0, 0)
        for (i = 0; i < MEM_DEPTH; i = i + 1) begin
            imem[i] = 32'h0000_0013;
        end

        // 2. Pre-load embedded RV32I Test Program:
        // PC = 0x00: addi x1, x0, 10         (x1 = 10)
        imem[0]  = 32'h00A00093;

        // PC = 0x04: addi x2, x0, 25         (x2 = 25)
        imem[1]  = 32'h01900113;

        // PC = 0x08: add  x3, x1, x2         (x3 = 10 + 25 = 35)
        imem[2]  = 32'h002081B3;

        // PC = 0x0C: sub  x4, x2, x1         (x4 = 25 - 10 = 15)
        imem[3]  = 32'h40110233;

        // PC = 0x10: and  x5, x1, x2         (x5 = 10 & 25 = 8)
        imem[4]  = 32'h0020F2B3;

        // PC = 0x14: or   x6, x1, x2         (x6 = 10 | 25 = 27)
        imem[5]  = 32'h0020E333;

        // PC = 0x18: slt  x7, x1, x2         (x7 = (10 < 25) ? 1 : 0 = 1)
        imem[6]  = 32'h0020A3B3;

        // PC = 0x1C: beq  x1, x2, 16         (10 == 25 false -> not taken)
        imem[7]  = 32'h00208863;

        // PC = 0x20: bne  x1, x2, 12         (10 != 25 true  -> target 0x2C)
        imem[8]  = 32'h00209663;

        // PC = 0x24: add  x8, x3, x4         (x8 = 35 + 15 = 50, bypassed if branch taken)
        imem[9]  = 32'h00418433;

        // PC = 0x28: sub  x9, x3, x4         (x9 = 35 - 15 = 20, bypassed if branch taken)
        imem[10] = 32'h404184B3;

        // PC = 0x2C: addi x10, x0, 100       (Branch target: x10 = 100)
        imem[11] = 32'h06400513;

        // PC = 0x30: jal  x0, 0              (Infinite loop / halt)
        imem[12] = 32'h0000006F;

        // Optional external hex file load
        if (HEX_FILE != "") begin
            $readmemh(HEX_FILE, imem);
        end
    end

endmodule
