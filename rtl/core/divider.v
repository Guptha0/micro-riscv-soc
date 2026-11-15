//==============================================================================
// Module Name: divider
// Description: Multi-cycle Hardware Restoring Divider supporting RV32M Extension:
//   - DIV  : Signed Quotient
//   - DIVU : Unsigned Quotient
//   - REM  : Signed Remainder
//   - REMU : Unsigned Remainder
//
// RISC-V Compliance Edge Cases:
//   - Division by zero:
//       * DIV/DIVU : Quotient = 32'hFFFF_FFFF (-1)
//       * REM/REMU : Remainder = Dividend (a)
//   - Overflow (MIN_INT / -1 -> 0x80000000 / -1):
//       * DIV : Quotient = 0x80000000
//       * REM : Remainder = 0x00000000
//==============================================================================

`timescale 1ns / 1ps

module divider (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,     // Trigger pulse from EX stage
    input  wire [2:0]  funct3,    // 100: DIV, 101: DIVU, 110: REM, 111: REMU
    input  wire [31:0] a,         // Dividend
    input  wire [31:0] b,         // Divisor
    output reg  [31:0] result,    // 32-bit Quotient or Remainder
    output reg         ready,     // Operation complete
    output reg         busy       // Divider active
);

    // Operation Decoding
    wire is_signed = (funct3 == 3'b100) || (funct3 == 3'b110);
    wire is_rem    = (funct3 == 3'b110) || (funct3 == 3'b111);

    // Corner Cases Detection
    wire is_div_by_zero = (b == 32'h0000_0000);
    wire is_overflow    = is_signed && (a == 32'h8000_0000) && (b == 32'hFFFF_FFFF);

    // Operand Sign Extraction
    wire sign_a = is_signed && a[31];
    wire sign_b = is_signed && b[31];
    wire sign_quot = is_signed && (sign_a ^ sign_b);
    wire sign_rem  = is_signed && sign_a;

    // Absolute Value Calculation
    wire [31:0] abs_a = sign_a ? (-a) : a;
    wire [31:0] abs_b = sign_b ? (-b) : b;

    // State Encoding
    localparam [1:0] STATE_IDLE = 2'b00;
    localparam [1:0] STATE_CALC = 2'b01;
    localparam [1:0] STATE_DONE = 2'b10;

    reg [1:0]  state;
    reg [5:0]  count;
    reg [31:0] reg_dividend;
    reg [31:0] reg_remainder;
    reg [31:0] reg_divisor;
    reg        res_sign_q;
    reg        res_sign_r;
    reg        op_is_rem;
    reg        is_corner;

    // Restoring division subtractor (33-bit operand: {remainder, MSB of dividend} - divisor)
    wire [32:0] sub_res = {reg_remainder, reg_dividend[31]} - {1'b0, reg_divisor};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= STATE_IDLE;
            result        <= 32'h0000_0000;
            ready         <= 1'b0;
            busy          <= 1'b0;
            count         <= 6'd0;
            reg_dividend  <= 32'd0;
            reg_remainder <= 32'd0;
            reg_divisor   <= 32'd0;
            res_sign_q    <= 1'b0;
            res_sign_r    <= 1'b0;
            op_is_rem     <= 1'b0;
            is_corner     <= 1'b0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    ready <= 1'b0;

                    if (start) begin
                        busy      <= 1'b1;
                        op_is_rem <= is_rem;

                        if (is_div_by_zero) begin
                            // Division by zero corner case (fast exit)
                            result    <= is_rem ? a : 32'hFFFF_FFFF;
                            is_corner <= 1'b1;
                            state     <= STATE_DONE;
                        end else if (is_overflow) begin
                            // Overflow corner case (fast exit)
                            result    <= is_rem ? 32'h0000_0000 : 32'h8000_0000;
                            is_corner <= 1'b1;
                            state     <= STATE_DONE;
                        end else begin
                            // Normal iterative restoring division
                            is_corner     <= 1'b0;
                            reg_dividend  <= abs_a;
                            reg_remainder <= 32'd0;
                            reg_divisor   <= abs_b;
                            res_sign_q    <= sign_quot;
                            res_sign_r    <= sign_rem;
                            count         <= 6'd32;
                            state         <= STATE_CALC;
                        end
                    end else begin
                        busy <= 1'b0;
                    end
                end

                STATE_CALC: begin
                    // 1-bit shift-subtract restoring step
                    if (sub_res[32] == 1'b0) begin
                        // Remainder >= Divisor (no borrow)
                        reg_remainder <= sub_res[31:0];
                        reg_dividend  <= {reg_dividend[30:0], 1'b1};
                    end else begin
                        // Borrow occurred: restore remainder
                        reg_remainder <= {reg_remainder[30:0], reg_dividend[31]};
                        reg_dividend  <= {reg_dividend[30:0], 1'b0};
                    end

                    if (count == 6'd1) begin
                        state <= STATE_DONE;
                    end else begin
                        count <= count - 6'd1;
                    end
                end

                STATE_DONE: begin
                    busy  <= 1'b0;
                    ready <= 1'b1;

                    // Final sign correction for normal division
                    if (!is_corner) begin
                        if (op_is_rem) begin
                            result <= res_sign_r ? (-reg_remainder) : reg_remainder;
                        end else begin
                            result <= res_sign_q ? (-reg_dividend) : reg_dividend;
                        end
                    end

                    state <= STATE_IDLE;
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

endmodule
