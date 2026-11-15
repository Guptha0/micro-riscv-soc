//==============================================================================
// Module Name: multiplier
// Description: Multi-cycle Hardware Multiplier supporting RV32M Extension:
//   - MUL    : rd = (rs1 * rs2)[31:0]
//   - MULH   : rd = (rs1 * rs2)[63:32] (signed x signed)
//   - MULHSU : rd = (rs1 * rs2)[63:32] (signed x unsigned)
//   - MULHU  : rd = (rs1 * rs2)[63:32] (unsigned x unsigned)
//==============================================================================

`timescale 1ns / 1ps

module multiplier (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,     // Trigger pulse from EX stage
    input  wire [2:0]  funct3,    // 000: MUL, 001: MULH, 010: MULHSU, 011: MULHU
    input  wire [31:0] a,         // Operand A
    input  wire [31:0] b,         // Operand B
    output reg  [31:0] result,    // 32-bit selected product
    output reg         ready,     // Operation complete
    output reg         busy       // Multiplier active
);

    // Multiplier State Encoding
    localparam STATE_IDLE = 1'b0;
    localparam STATE_DONE = 1'b1;

    reg state;

    // Signed / Unsigned Operand Extensions
    wire signed [32:0] op_a_signed = {a[31], a};
    wire signed [32:0] op_b_signed = {b[31], b};
    wire signed [32:0] op_a_su     = {a[31], a};
    wire signed [32:0] op_b_su     = {1'b0, b};

    // Full 64-bit Products
    wire signed [65:0] prod_signed = op_a_signed * op_b_signed;
    wire signed [65:0] prod_su     = op_a_su * op_b_su;
    wire        [63:0] prod_unsigned = {32'b0, a} * {32'b0, b};

    // Output Selection based on funct3
    reg [31:0] next_result;
    always @(*) begin
        case (funct3)
            3'b000:  next_result = prod_unsigned[31:0]; // MUL
            3'b001:  next_result = prod_signed[63:32];   // MULH
            3'b010:  next_result = prod_su[63:32];       // MULHSU
            3'b011:  next_result = prod_unsigned[63:32]; // MULHU
            default: next_result = prod_unsigned[31:0];
        endcase
    end

    // Multi-Cycle State Machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state  <= STATE_IDLE;
            result <= 32'h0000_0000;
            ready  <= 1'b0;
            busy   <= 1'b0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    ready <= 1'b0;
                    if (start) begin
                        busy   <= 1'b1;
                        result <= next_result;
                        state  <= STATE_DONE;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                STATE_DONE: begin
                    ready <= 1'b1;
                    busy  <= 1'b0;
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

endmodule
