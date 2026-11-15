//==============================================================================
// Testbench Name: pc_fetch_tb
// Description: Comprehensive multi-cycle self-checking testbench with automated
//              assertions for RV32I Program Counter & Instruction Memory (pc_fetch.v).
//
// Verification Coverage:
//   1. Asynchronous active-low reset assertion and power-on state (PC = 0).
//   2. Sequential PC stepping (+4 each cycle) and instruction word correctness.
//   3. Verification of all pre-loaded test instructions:
//      - addi, add, sub, and, or, slt, beq, bne, jal.
//   4. Branch / Jump target overrides:
//      - Forward branch target update (branch_taken = 1, next_pc = target).
//      - Immediate resumption of sequential +4 stepping.
//      - Backward loop jump target update.
//   5. Mid-execution asynchronous reset:
//      - Instantaneous reset without waiting for active clock edge.
//   6. Out-of-bounds PC access returning safe NOP (32'h00000013).
//==============================================================================

`timescale 1ns / 1ps

module pc_fetch_tb;

    // Clock timing parameter: 10 ns period (100 MHz)
    localparam CLK_PERIOD = 10;

    // Expected machine code constants
    localparam [31:0] INSTR_ADDI_X1  = 32'h00A00093; // PC = 0x00
    localparam [31:0] INSTR_ADDI_X2  = 32'h01900113; // PC = 0x04
    localparam [31:0] INSTR_ADD_X3   = 32'h002081B3; // PC = 0x08
    localparam [31:0] INSTR_SUB_X4   = 32'h40110233; // PC = 0x0C
    localparam [31:0] INSTR_AND_X5   = 32'h0020F2B3; // PC = 0x10
    localparam [31:0] INSTR_OR_X6    = 32'h0020E333; // PC = 0x14
    localparam [31:0] INSTR_SLT_X7   = 32'h0020A3B3; // PC = 0x18
    localparam [31:0] INSTR_BEQ      = 32'h00208863; // PC = 0x1C
    localparam [31:0] INSTR_BNE      = 32'h00209663; // PC = 0x20
    localparam [31:0] INSTR_ADD_X8   = 32'h00418433; // PC = 0x24
    localparam [31:0] INSTR_SUB_X9   = 32'h404184B3; // PC = 0x28
    localparam [31:0] INSTR_ADDI_X10 = 32'h06400513; // PC = 0x2C
    localparam [31:0] INSTR_JAL      = 32'h0000006F; // PC = 0x30
    localparam [31:0] INSTR_NOP      = 32'h00000013; // Out-of-bounds NOP

    // DUT Signals
    reg         clk;
    reg         rst_n;
    reg         branch_taken;
    reg  [31:0] next_pc;
    wire [31:0] pc;
    wire [31:0] instruction;

    // Tracking counters
    integer cycle_count;
    integer test_count;
    integer pass_count;
    integer fail_count;

    // Instantiate Device Under Test (DUT)
    pc_fetch #(
        .MEM_DEPTH(64)
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .branch_taken (branch_taken),
        .next_pc      (next_pc),
        .pc           (pc),
        .instruction  (instruction)
    );

    // Free-Running Clock Generator
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    // Cycle Counter
    always @(posedge clk) begin
        cycle_count = cycle_count + 1;
    end

    //--------------------------------------------------------------------------
    // Automated Assertion Task
    //--------------------------------------------------------------------------
    task assert_state;
        input [31:0]  exp_pc;
        input [31:0]  exp_instruction;
        input [191:0] desc;
        begin
            #1; // Settling delta
            test_count = test_count + 1;

            if ((pc === exp_pc) && (instruction === exp_instruction)) begin
                pass_count = pass_count + 1;
                $display("  [PASS] Cycle %3d | %-28s | PC=0x%08h | Instr=0x%08h",
                         cycle_count, desc, pc, instruction);
            end else begin
                fail_count = fail_count + 1;
                $display("  ===============================================================");
                $display("  [FAIL ASSERTION] Cycle %3d | %-28s", cycle_count, desc);
                $display("    Expected : PC=0x%08h, Instruction=0x%08h", exp_pc, exp_instruction);
                $display("    Actual   : PC=0x%08h, Instruction=0x%08h", pc, instruction);
                $display("  ===============================================================");
            end
        end
    endtask

    //--------------------------------------------------------------------------
    // Test Sequence
    //--------------------------------------------------------------------------
    initial begin
        // Setup VCD waveform generation
        $dumpfile("pc_fetch.vcd");
        $dumpvars(0, pc_fetch_tb);

        // Initialize signals
        cycle_count  = 0;
        test_count   = 0;
        pass_count   = 0;
        fail_count   = 0;
        branch_taken = 1'b0;
        next_pc      = 32'h0000_0000;
        rst_n        = 1'b1;

        $display("\n================================================================================");
        $display("   STARTING RV32I PC & INSTRUCTION FETCH RIGOROUS VERIFICATION SUITE");
        $display("================================================================================\n");

        //----------------------------------------------------------------------
        // Phase 1: Asynchronous Reset Verification
        //----------------------------------------------------------------------
        $display("--- PHASE 1: Asynchronous Reset Verification ---");
        rst_n = 1'b0;
        #15;
        assert_state(32'h0000_0000, INSTR_ADDI_X1, "RESET_PC0_ADDI_X1");

        // De-assert reset synchronously before clock edge
        @(negedge clk);
        rst_n = 1'b1;

        //----------------------------------------------------------------------
        // Phase 2: Sequential Execution & PC Stepping (+4 each cycle)
        //----------------------------------------------------------------------
        $display("\n--- PHASE 2: Sequential PC Stepping (+4 each cycle) ---");
        
        // Cycle 1: PC = 0x00 (held from reset release)
        @(posedge clk);
        assert_state(32'h0000_0004, INSTR_ADDI_X2, "STEP1_PC04_ADDI_X2");

        @(posedge clk);
        assert_state(32'h0000_0008, INSTR_ADD_X3, "STEP2_PC08_ADD_X3");

        @(posedge clk);
        assert_state(32'h0000_000C, INSTR_SUB_X4, "STEP3_PC0C_SUB_X4");

        @(posedge clk);
        assert_state(32'h0000_0010, INSTR_AND_X5, "STEP4_PC10_AND_X5");

        @(posedge clk);
        assert_state(32'h0000_0014, INSTR_OR_X6, "STEP5_PC14_OR_X6");

        @(posedge clk);
        assert_state(32'h0000_0018, INSTR_SLT_X7, "STEP6_PC18_SLT_X7");

        @(posedge clk);
        assert_state(32'h0000_001C, INSTR_BEQ, "STEP7_PC1C_BEQ");

        @(posedge clk);
        assert_state(32'h0000_0020, INSTR_BNE, "STEP8_PC20_BNE");

        //----------------------------------------------------------------------
        // Phase 3: Forward Branch / Jump Override
        //----------------------------------------------------------------------
        $display("\n--- PHASE 3: Forward Branch / Jump Override ---");
        // At PC = 0x20 (bne), suppose branch condition is TRUE:
        // Set branch_taken = 1 and next_pc = 32'h0000_002C (target addi x10)
        @(negedge clk);
        branch_taken = 1'b1;
        next_pc      = 32'h0000_002C;

        @(posedge clk);
        assert_state(32'h0000_002C, INSTR_ADDI_X10, "BRANCH_FORWARD_PC2C");

        // Immediately de-assert branch_taken to resume normal +4 stepping
        @(negedge clk);
        branch_taken = 1'b0;

        @(posedge clk);
        assert_state(32'h0000_0030, INSTR_JAL, "RESUME_SEQ_PC30_JAL");

        //----------------------------------------------------------------------
        // Phase 4: Backward Branch / Loop Jump Override
        //----------------------------------------------------------------------
        $display("\n--- PHASE 4: Backward Branch / Loop Jump Override ---");
        // Jump back to PC = 0x08 (add x3, x1, x2)
        @(negedge clk);
        branch_taken = 1'b1;
        next_pc      = 32'h0000_0008;

        @(posedge clk);
        assert_state(32'h0000_0008, INSTR_ADD_X3, "LOOP_BACKWARD_PC08");

        // Resume sequential stepping
        @(negedge clk);
        branch_taken = 1'b0;

        @(posedge clk);
        assert_state(32'h0000_000C, INSTR_SUB_X4, "RESUME_SEQ_PC0C_SUB");

        @(posedge clk);
        assert_state(32'h0000_0010, INSTR_AND_X5, "RESUME_SEQ_PC10_AND");

        //----------------------------------------------------------------------
        // Phase 5: Mid-Stream Asynchronous Reset Assertion
        //----------------------------------------------------------------------
        $display("\n--- PHASE 5: Mid-Stream Asynchronous Reset Assertion ---");
        // Trigger reset mid-cycle between clock edges at PC = 0x10
        #3;
        rst_n = 1'b0;
        #1;
        // Verify instantaneous combinational response without waiting for posedge clk
        assert_state(32'h0000_0000, INSTR_ADDI_X1, "ASYNC_RESET_MID_STREAM");

        #10;
        @(negedge clk);
        rst_n = 1'b1;

        @(posedge clk);
        assert_state(32'h0000_0004, INSTR_ADDI_X2, "POST_RESET_RESUME_PC04");

        //----------------------------------------------------------------------
        // Phase 6: Out-of-Bounds Memory Access
        //----------------------------------------------------------------------
        $display("\n--- PHASE 6: Out-of-Bounds Memory Access ---");
        // Jump to an address beyond MEM_DEPTH * 4 (e.g. 0x00000200 = 512)
        @(negedge clk);
        branch_taken = 1'b1;
        next_pc      = 32'h0000_0200;

        @(posedge clk);
        assert_state(32'h0000_0200, INSTR_NOP, "OUT_OF_BOUNDS_NOP_PC200");

        // Step by 4 out-of-bounds
        @(negedge clk);
        branch_taken = 1'b0;

        @(posedge clk);
        assert_state(32'h0000_0204, INSTR_NOP, "OUT_OF_BOUNDS_NOP_PC204");

        // Return to normal program area
        @(negedge clk);
        branch_taken = 1'b1;
        next_pc      = 32'h0000_0014;

        @(posedge clk);
        assert_state(32'h0000_0014, INSTR_OR_X6, "RETURN_VALID_SPACE_PC14");

        // Wait final cycle
        @(posedge clk);

        //----------------------------------------------------------------------
        // Final Summary Banner
        //----------------------------------------------------------------------
        $display("\n================================================================================");
        $display("   PC & INSTRUCTION FETCH TESTBENCH EXECUTION SUMMARY");
        $display("================================================================================");
        $display("   Total Clock Cycles : %0d", cycle_count);
        $display("   Total Tests Run    : %0d", test_count);
        $display("   Tests Passed       : %0d", pass_count);
        $display("   Tests Failed       : %0d", fail_count);
        $display("================================================================================");

        if (fail_count == 0) begin
            $display("\n   [RESULT: SUCCESS] ALL PC & FETCH ASSERTIONS PASSED PERFECTLY!\n");
            $finish(0);
        end else begin
            $display("\n   [RESULT: FAILURE] %0d PC & FETCH ASSERTIONS FAILED!\n", fail_count);
            $fatal(1, "PC & Instruction fetch testbench failed with assertion errors.");
        end
    end

endmodule
