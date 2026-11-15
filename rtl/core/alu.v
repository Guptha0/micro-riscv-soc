//==============================================================================
// Module Name: alu
// Description: 32-bit Arithmetic Logic Unit (ALU) for RV32I RISC-V Processor Core
// Supported Operations:
//   - ADD : 32-bit Addition (with signed overflow detection)
//   - SUB : 32-bit Subtraction (with signed overflow detection)
//   - AND : 32-bit Bitwise Logical AND
//   - OR  : 32-bit Bitwise Logical OR
//   - SLT : Set Less Than (Signed comparison)
// Flags:
//   - zero    : Asserted when result is 32'h0000_0000
//   - overflow: Asserted on signed two's complement overflow during ADD / SUB
//==============================================================================

`timescale 1ns / 1ps

module alu #(
    // Standard RISC-V RV32I ALU operation encodings {funct7[5], funct3}
    parameter [3:0] ALU_ADD  = 4'b0000, // funct7[5]=0, funct3=000
    parameter [3:0] ALU_SUB  = 4'b1000, // funct7[5]=1, funct3=000
    parameter [3:0] ALU_SLT  = 4'b0010, // funct7[5]=0, funct3=010
    parameter [3:0] ALU_OR   = 4'b0110, // funct7[5]=0, funct3=110
    parameter [3:0] ALU_AND  = 4'b0111  // funct7[5]=0, funct3=111
)(
    input  wire [31:0] a,        // Operand A
    input  wire [31:0] b,        // Operand B
    input  wire [3:0]  alu_ctrl, // ALU operation control code
    output reg  [31:0] result,   // 32-bit ALU operation result
    output wire        zero,     // Zero flag: high when result == 0
    output reg         overflow  // Signed overflow flag for arithmetic ops
);

    //--------------------------------------------------------------------------
    // Internal Arithmetic Computations
    //--------------------------------------------------------------------------
    wire [31:0] add_res;
    wire [31:0] sub_res;
    wire        add_ovf;
    wire        sub_ovf;
    wire        slt_val;

    assign add_res = a + b;
    assign sub_res = a - b;

    // Signed overflow occurs when:
    // ADD: Two positive operands yield a negative result, or two negative operands yield a positive result.
    //      Condition: (a[31] == b[31]) && (result[31] != a[31])
    assign add_ovf = (~(a[31] ^ b[31])) & (a[31] ^ add_res[31]);

    // SUB: Subtracting a negative from a positive yields a negative result,
    //      or subtracting a positive from a negative yields a positive result.
    //      Condition: (a[31] != b[31]) && (result[31] != a[31])
    assign sub_ovf = (a[31] ^ b[31]) & (a[31] ^ sub_res[31]);

    // SLT (Set Less Than - Signed):
    // Hardware implementation reuses the subtraction result and subtraction overflow:
    // If no overflow, a < b iff sub_res[31] == 1.
    // If overflow occurred, the sign bit is inverted: slt_val = sub_res[31] ^ sub_ovf.
    assign slt_val = sub_res[31] ^ sub_ovf;

    // Zero flag: Asserted if all bits of result are zero (synthesizes to NOR gate tree)
    assign zero = (result == 32'h0000_0000);

    //--------------------------------------------------------------------------
    // Combinational Output Multiplexer
    //--------------------------------------------------------------------------
    always @(*) begin
        // Default values to prevent inferred latches
        result   = 32'h0000_0000;
        overflow = 1'b0;

        case (alu_ctrl)
            ALU_ADD: begin
                result   = add_res;
                overflow = add_ovf;
            end

            ALU_SUB, 4'b0001: begin // Standard RISC-V 4'b1000 and alias 4'b0001
                result   = sub_res;
                overflow = sub_ovf;
            end

            ALU_AND: begin
                result   = a & b;
                overflow = 1'b0;
            end

            ALU_OR: begin
                result   = a | b;
                overflow = 1'b0;
            end

            ALU_SLT: begin
                result   = {31'b0, slt_val};
                overflow = 1'b0;
            end

            default: begin
                result   = 32'h0000_0000;
                overflow = 1'b0;
            end
        endcase
    end

endmodule
