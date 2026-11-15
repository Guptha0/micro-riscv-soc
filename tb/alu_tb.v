//==============================================================================
// Testbench Name: alu_tb
// Description: Comprehensive multi-cycle self-checking testbench with automated
//              assertions for 32-bit RV32I ALU (alu.v).
// Tests covered:
//   - ADD: Normal addition, zero sum, signed positive/negative overflow, wrap
//   - SUB: Normal subtraction, zero diff, signed positive/negative overflow
//   - AND: Bitwise masking, alternating patterns, identity, all-zeros
//   - OR : Bitwise union, identity, all-zeros, disjoint bitfields
//   - SLT: Signed less than across +/- bounds, MIN_INT, MAX_INT, edge cases
//   - Multi-cycle randomized stress testing against golden reference model
//   - Illegal/default opcode fallback verification
//==============================================================================

`timescale 1ns / 1ps

module alu_tb;

    //--------------------------------------------------------------------------
    // Clock and Timing Parameters
    //--------------------------------------------------------------------------
    localparam CLK_PERIOD = 10; // 10 ns clock period (100 MHz)

    // ALU Operation Codes (matching RV32I funct7[5], funct3 encodings)
    localparam [3:0] ALU_ADD = 4'b0000;
    localparam [3:0] ALU_SUB = 4'b1000;
    localparam [3:0] ALU_SLT = 4'b0010;
    localparam [3:0] ALU_OR  = 4'b0110;
    localparam [3:0] ALU_AND = 4'b0111;

    // Corner Case Constants
    localparam [31:0] ZERO_VAL = 32'h0000_0000;
    localparam [31:0] MAX_POS  = 32'h7FFF_FFFF; // +2147483647
    localparam [31:0] MIN_NEG  = 32'h8000_0000; // -2147483648
    localparam [31:0] ALL_ONES = 32'hFFFF_FFFF; // -1 signed

    //--------------------------------------------------------------------------
    // DUT Signals
    //--------------------------------------------------------------------------
    reg         clk;
    reg  [31:0] tb_a;
    reg  [31:0] tb_b;
    reg  [3:0]  tb_ctrl;
    wire [31:0] tb_result;
    wire        tb_zero;
    wire        tb_ovf;

    // Test Statistics & Cycle Tracking
    integer cycle_count;
    integer test_count;
    integer pass_count;
    integer fail_count;

    //--------------------------------------------------------------------------
    // Device Under Test (DUT) Instantiation
    //--------------------------------------------------------------------------
    alu dut (
        .a        (tb_a),
        .b        (tb_b),
        .alu_ctrl (tb_ctrl),
        .result   (tb_result),
        .zero     (tb_zero),
        .overflow (tb_ovf)
    );

    //--------------------------------------------------------------------------
    // Free-Running Clock Generator
    //--------------------------------------------------------------------------
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    // Cycle Counter
    always @(posedge clk) begin
        cycle_count = cycle_count + 1;
    end

    //--------------------------------------------------------------------------
    // Golden Reference Model Functions
    //--------------------------------------------------------------------------
    function [31:0] golden_alu;
        input [31:0] a;
        input [31:0] b;
        input [3:0]  ctrl;
        begin
            case (ctrl)
                ALU_ADD: golden_alu = a + b;
                ALU_SUB: golden_alu = a - b;
                ALU_AND: golden_alu = a & b;
                ALU_OR:  golden_alu = a | b;
                ALU_SLT: golden_alu = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
                default: golden_alu = 32'h0000_0000;
            endcase
        end
    endfunction

    function golden_ovf;
        input [31:0] a;
        input [31:0] b;
        input [3:0]  ctrl;
        reg [31:0] res;
        begin
            case (ctrl)
                ALU_ADD: begin
                    res = a + b;
                    golden_ovf = (~(a[31] ^ b[31])) & (a[31] ^ res[31]);
                end
                ALU_SUB: begin
                    res = a - b;
                    golden_ovf = (a[31] ^ b[31]) & (a[31] ^ res[31]);
                end
                default: golden_ovf = 1'b0;
            endcase
        end
    endfunction

    function golden_zero;
        input [31:0] res;
        begin
            golden_zero = (res == 32'h0000_0000);
        end
    endfunction

    //--------------------------------------------------------------------------
    // Automated Assertion Task
    // Driven at negedge clk (stable setup), checked at posedge clk
    //--------------------------------------------------------------------------
    task run_test;
        input [31:0]  in_a;
        input [31:0]  in_b;
        input [3:0]   in_ctrl;
        input [31:0]  exp_res;
        input         exp_zero;
        input         exp_ovf;
        input [191:0] test_desc; // string description (up to 24 chars)
        begin
            // Setup inputs on negative clock edge
            @(negedge clk);
            tb_a    = in_a;
            tb_b    = in_b;
            tb_ctrl = in_ctrl;

            // Wait for positive clock edge and small settling delta
            @(posedge clk);
            #1;

            test_count = test_count + 1;

            // Automated Assertions Check
            if ((tb_result === exp_res) && (tb_zero === exp_zero) && (tb_ovf === exp_ovf)) begin
                pass_count = pass_count + 1;
                $display("  [PASS] Cycle %3d | %-24s | A=0x%08h B=0x%08h -> Res=0x%08h [Z=%b V=%b]",
                         cycle_count, test_desc, in_a, in_b, tb_result, tb_zero, tb_ovf);
            end else begin
                fail_count = fail_count + 1;
                $display("  ===============================================================");
                $display("  [FAIL ASSERTION] Cycle %3d | %-24s", cycle_count, test_desc);
                $display("    Stimulus: A=0x%08h, B=0x%08h, CTRL=4'b%04b", in_a, in_b, in_ctrl);
                $display("    Expected: Result=0x%08h, Zero=%b, Overflow=%b",
                         exp_res, exp_zero, exp_ovf);
                $display("    Actual  : Result=0x%08h, Zero=%b, Overflow=%b",
                         tb_result, tb_zero, tb_ovf);
                $display("  ===============================================================");
            end
        end
    endtask

    //--------------------------------------------------------------------------
    // Main Test Sequence
    //--------------------------------------------------------------------------
    integer i;
    reg [31:0] rand_a, rand_b;
    reg [3:0]  rand_op;
    reg [3:0]  ops [0:4];
    reg [31:0] exp_r;
    reg        exp_z;
    reg        exp_v;

    initial begin
        // Setup waveform dump file
        $dumpfile("alu.vcd");
        $dumpvars(0, alu_tb);

        // Initialize variables
        cycle_count = 0;
        test_count  = 0;
        pass_count  = 0;
        fail_count  = 0;
        tb_a        = 32'd0;
        tb_b        = 32'd0;
        tb_ctrl     = 4'b0000;

        ops[0] = ALU_ADD;
        ops[1] = ALU_SUB;
        ops[2] = ALU_AND;
        ops[3] = ALU_OR;
        ops[4] = ALU_SLT;

        $display("\n================================================================================");
        $display("   STARTING RV32I ALU MULTI-CYCLE COMPREHENSIVE VERIFICATION SUITE");
        $display("================================================================================\n");

        // Wait initial clock cycles
        @(posedge clk);
        @(posedge clk);

        //======================================================================
        // SUITE 1: ADD OPERATIONS & SIGNED OVERFLOW / ZERO
        //======================================================================
        $display("--- SUITE 1: ADD Operations & Flags ---");
        // in_a, in_b, ctrl, exp_res, exp_zero, exp_ovf, description
        run_test(32'd15, 32'd27, ALU_ADD, 32'd42, 1'b0, 1'b0, "ADD_Pos_Pos_42");
        run_test(32'd0, 32'd0, ALU_ADD, 32'd0, 1'b1, 1'b0, "ADD_Zero_Zero");
        run_test(32'h1234_5678, 32'd0, ALU_ADD, 32'h1234_5678, 1'b0, 1'b0, "ADD_Identity_Zero");
        run_test(-32'sd10, 32'sd25, ALU_ADD, 32'sd15, 1'b0, 1'b0, "ADD_Neg_Pos_ResultPos");
        run_test(32'sd20, -32'sd50, ALU_ADD, -32'sd30, 1'b0, 1'b0, "ADD_Pos_Neg_ResultNeg");
        run_test(32'sd100, -32'sd100, ALU_ADD, 32'd0, 1'b1, 1'b0, "ADD_Opposites_ZeroFlag");
        run_test(MAX_POS, 32'd1, ALU_ADD, MIN_NEG, 1'b0, 1'b1, "ADD_Pos_Overflow_MaxPos");
        run_test(32'h7000_0000, 32'h2000_0000, ALU_ADD, 32'h9000_0000, 1'b0, 1'b1, "ADD_Pos_Overflow_Big");
        run_test(MIN_NEG, MIN_NEG, ALU_ADD, 32'd0, 1'b1, 1'b1, "ADD_Neg_Ovf_ZeroFlag");
        run_test(MIN_NEG, ALL_ONES, ALU_ADD, MAX_POS, 1'b0, 1'b1, "ADD_Neg_Overflow_MinNeg");
        run_test(ALL_ONES, 32'd1, ALU_ADD, 32'd0, 1'b1, 1'b0, "ADD_UnsignedWrap_NoOvf");
        run_test(-32'sd100, -32'sd200, ALU_ADD, -32'sd300, 1'b0, 1'b0, "ADD_Neg_Neg_NoOvf");

        //======================================================================
        // SUITE 2: SUB OPERATIONS & SIGNED OVERFLOW / ZERO
        //======================================================================
        $display("\n--- SUITE 2: SUB Operations & Flags ---");
        run_test(32'd50, 32'd20, ALU_SUB, 32'd30, 1'b0, 1'b0, "SUB_Pos_Pos_30");
        run_test(32'hCAFE_BABE, 32'hCAFE_BABE, ALU_SUB, 32'd0, 1'b1, 1'b0, "SUB_Identical_ZeroFlag");
        run_test(32'd20, 32'd50, ALU_SUB, -32'sd30, 1'b0, 1'b0, "SUB_Pos_Pos_NegResult");
        run_test(32'd10, -32'sd20, ALU_SUB, 32'd30, 1'b0, 1'b0, "SUB_Pos_Minus_Neg");
        run_test(-32'sd50, -32'sd20, ALU_SUB, -32'sd30, 1'b0, 1'b0, "SUB_Neg_Minus_Neg");
        run_test(MAX_POS, ALL_ONES, ALU_SUB, MIN_NEG, 1'b0, 1'b1, "SUB_Pos_Overflow");
        run_test(MIN_NEG, 32'd1, ALU_SUB, MAX_POS, 1'b0, 1'b1, "SUB_Neg_Overflow");
        run_test(MIN_NEG, MAX_POS, ALU_SUB, 32'd1, 1'b0, 1'b1, "SUB_Extreme_Neg_Pos");
        run_test(32'hDEAD_BEEF, 32'd0, ALU_SUB, 32'hDEAD_BEEF, 1'b0, 1'b0, "SUB_Minus_Zero");

        //======================================================================
        // SUITE 2B: SHIFT & LOGICAL OPERATIONS (SLL, SLTU, XOR, SRL, SRA)
        //======================================================================
        $display("\n--- SUITE 2B: Shift & Logical RV32I Operations ---");
        run_test(32'd1, 32'd4, 4'b0001, 32'd16, 1'b0, 1'b0, "SLL_1_Shift_4");
        run_test(32'd10, 32'd20, 4'b0011, 32'd1, 1'b0, 1'b0, "SLTU_10_Less_20");
        run_test(32'd20, 32'd10, 4'b0011, 32'd0, 1'b1, 1'b0, "SLTU_20_NotLess_10");
        run_test(32'hFF00_FF00, 32'h0F0F_0F0F, 4'b0100, 32'hF00F_F00F, 1'b0, 1'b0, "XOR_Pattern");
        run_test(32'h8000_0000, 32'd4, 4'b0101, 32'h0800_0000, 1'b0, 1'b0, "SRL_Logical_Shift");
        run_test(32'h8000_0000, 32'd4, 4'b1101, 32'hF800_0000, 1'b0, 1'b0, "SRA_Arithmetic_Shift");

        //======================================================================
        // SUITE 3: AND OPERATIONS
        //======================================================================
        $display("\n--- SUITE 3: AND Logical Operations ---");
        run_test(32'hAAAA_AAAA, 32'h5555_5555, ALU_AND, 32'h0000_0000, 1'b1, 1'b0, "AND_Alternating_Zero");
        run_test(ALL_ONES, 32'h1234_5678, ALU_AND, 32'h1234_5678, 1'b0, 1'b0, "AND_Identity_AllOnes");
        run_test(32'hDEAD_BEEF, 32'h0000_FFFF, ALU_AND, 32'h0000_BEEF, 1'b0, 1'b0, "AND_Mask_Lower16");
        run_test(32'hDEAD_BEEF, 32'hFFFF_0000, ALU_AND, 32'hDEAD_0000, 1'b0, 1'b0, "AND_Mask_Upper16");
        run_test(32'd0, 32'hCAFE_BABE, ALU_AND, 32'd0, 1'b1, 1'b0, "AND_Zero_Operand");

        //======================================================================
        // SUITE 4: OR OPERATIONS
        //======================================================================
        $display("\n--- SUITE 4: OR Logical Operations ---");
        run_test(32'hAAAA_AAAA, 32'h5555_5555, ALU_OR, ALL_ONES, 1'b0, 1'b0, "OR_Complementary_Ones");
        run_test(32'd0, 32'hABCD_1234, ALU_OR, 32'hABCD_1234, 1'b0, 1'b0, "OR_Zero_Identity");
        run_test(32'd0, 32'd0, ALU_OR, 32'd0, 1'b1, 1'b0, "OR_Both_Zero_Flag");
        run_test(32'hFF00_0000, 32'h00FF_0000, ALU_OR, 32'hFFFF_0000, 1'b0, 1'b0, "OR_Disjoint_Bytes");
        run_test(32'h1234_5678, ALL_ONES, ALU_OR, ALL_ONES, 1'b0, 1'b0, "OR_With_AllOnes");

        //======================================================================
        // SUITE 5: SLT OPERATIONS (Signed Set Less Than)
        //======================================================================
        $display("\n--- SUITE 5: SLT Signed Comparison ---");
        run_test(32'sd10, 32'sd20, ALU_SLT, 32'd1, 1'b0, 1'b0, "SLT_Pos_Less_Pos");
        run_test(32'sd20, 32'sd10, ALU_SLT, 32'd0, 1'b1, 1'b0, "SLT_Pos_Greater_Pos");
        run_test(32'sd15, 32'sd15, ALU_SLT, 32'd0, 1'b1, 1'b0, "SLT_Pos_Equal_Pos");
        run_test(-32'sd10, 32'sd5, ALU_SLT, 32'd1, 1'b0, 1'b0, "SLT_Neg_Less_Pos");
        run_test(32'sd5, -32'sd10, ALU_SLT, 32'd0, 1'b1, 1'b0, "SLT_Pos_Greater_Neg");
        run_test(-32'sd20, -32'sd10, ALU_SLT, 32'd1, 1'b0, 1'b0, "SLT_Neg_Less_Neg");
        run_test(-32'sd10, -32'sd20, ALU_SLT, 32'd0, 1'b1, 1'b0, "SLT_Neg_Greater_Neg");
        run_test(-32'sd25, -32'sd25, ALU_SLT, 32'd0, 1'b1, 1'b0, "SLT_Neg_Equal_Neg");
        run_test(MIN_NEG, MAX_POS, ALU_SLT, 32'd1, 1'b0, 1'b0, "SLT_MinNeg_Less_MaxPos");
        run_test(MAX_POS, MIN_NEG, ALU_SLT, 32'd0, 1'b1, 1'b0, "SLT_MaxPos_Greater_MinNeg");
        run_test(ALL_ONES, 32'd0, ALU_SLT, 32'd1, 1'b0, 1'b0, "SLT_MinusOne_Less_Zero");
        run_test(32'd0, ALL_ONES, ALU_SLT, 32'd0, 1'b1, 1'b0, "SLT_Zero_Greater_MinusOne");
        run_test(32'd0, 32'd0, ALU_SLT, 32'd0, 1'b1, 1'b0, "SLT_Zero_Equal_Zero");

        //======================================================================
        // SUITE 6: DEFAULT / ILLEGAL OPCODES
        //======================================================================
        $display("\n--- SUITE 6: Illegal / Default Opcode Fallback ---");
        run_test(32'hDEAD_BEEF, 32'hCAFE_BABE, 4'b1111, 32'd0, 1'b1, 1'b0, "ILLEGAL_OP_1111");
        run_test(32'h1111_2222, 32'h3333_4444, 4'b1010, 32'd0, 1'b1, 1'b0, "ILLEGAL_OP_1010");

        //======================================================================
        // SUITE 7: MULTI-CYCLE RANDOMIZED STRESS TESTING
        //======================================================================
        $display("\n--- SUITE 7: 64-Cycle Randomized Stress Testing ---");
        for (i = 0; i < 64; i = i + 1) begin
            rand_a  = $random;
            rand_b  = $random;
            rand_op = ops[$unsigned($random) % 5];

            exp_r = golden_alu(rand_a, rand_b, rand_op);
            exp_z = golden_zero(exp_r);
            exp_v = golden_ovf(rand_a, rand_b, rand_op);

            run_test(rand_a, rand_b, rand_op, exp_r, exp_z, exp_v, "RANDOM_STRESS_CYCLE");
        end

        // Wait another clock cycle to settle
        @(posedge clk);

        //======================================================================
        // FINAL TEST SUMMARY & ASSERTION REPORT
        //======================================================================
        $display("\n================================================================================");
        $display("   TESTBENCH EXECUTION SUMMARY");
        $display("================================================================================");
        $display("   Total Clock Cycles : %0d", cycle_count);
        $display("   Total Tests Run    : %0d", test_count);
        $display("   Tests Passed       : %0d", pass_count);
        $display("   Tests Failed       : %0d", fail_count);
        $display("================================================================================");

        if (fail_count == 0) begin
            $display("\n   [RESULT: SUCCESS] ALL ASSERTIONS PASSED PERFECTLY!\n");
            $finish(0);
        end else begin
            $display("\n   [RESULT: FAILURE] %0d ASSERTIONS FAILED! Please inspect errors above.\n", fail_count);
            $fatal(1, "Testbench failed with assertion errors.");
        end
    end

endmodule
