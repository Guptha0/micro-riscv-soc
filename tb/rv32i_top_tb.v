//==============================================================================
// Testbench Name: rv32i_top_tb
// Description: Comprehensive system-level verification testbench for the 
//              complete single-cycle RV32I Processor Core (rv32i_top.v).
//
// Verification Coverage:
//   1. Full processor power-on boot and reset sequence.
//   2. Execution of arithmetic instructions (addi, add, sub).
//   3. Execution of logical bitwise instructions (and, or).
//   4. Execution of comparison instructions (slt).
//   5. Conditional branch not-taken path (beq).
//   6. Conditional branch taken path (bne with forward jump).
//   7. Execution bypass validation (ensuring bypassed instructions never write rd).
//   8. Branch target execution and register update (addi x10).
//   9. Unconditional jump / halt loop (jal).
//  10. Hardwired x0 zero-state persistence across all execution cycles.
//  11. Real-time cycle-by-cycle instruction trace & architectural register dump.
//==============================================================================

`timescale 1ns / 1ps

module rv32i_top_tb;

    localparam CLK_PERIOD = 10; // 10 ns clock (100 MHz)

    // Signals
    reg         clk;
    reg         rst_n;

    wire [31:0] pc_out;
    wire [31:0] instruction_out;
    wire [31:0] alu_result_out;
    wire [31:0] rs1_data_out;
    wire [31:0] rs2_data_out;
    wire [31:0] rd_data_out;
    wire [31:0] imm_out;
    wire [3:0]  alu_ctrl_out;
    wire        reg_write_out;
    wire        branch_taken_out;

    // Simulation tracking
    integer cycle;
    integer test_count;
    integer pass_count;
    integer fail_count;

    // Instantiate RV32I Top-Level Core
    rv32i_top #(
        .MEM_DEPTH(64)
    ) dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .pc_out           (pc_out),
        .instruction_out  (instruction_out),
        .alu_result_out   (alu_result_out),
        .rs1_data_out     (rs1_data_out),
        .rs2_data_out     (rs2_data_out),
        .rd_data_out      (rd_data_out),
        .imm_out          (imm_out),
        .alu_ctrl_out     (alu_ctrl_out),
        .reg_write_out    (reg_write_out),
        .branch_taken_out (branch_taken_out)
    );

    // Clock Generator
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    // Instruction disassembler helper for trace logging
    function [191:0] get_mnemonic;
        input [31:0] instr;
        begin
            case (instr)
                32'h00A00093: get_mnemonic = "addi x1, x0, 10";
                32'h01900113: get_mnemonic = "addi x2, x0, 25";
                32'h002081B3: get_mnemonic = "add  x3, x1, x2";
                32'h40110233: get_mnemonic = "sub  x4, x2, x1";
                32'h0020F2B3: get_mnemonic = "and  x5, x1, x2";
                32'h0020E333: get_mnemonic = "or   x6, x1, x2";
                32'h0020A3B3: get_mnemonic = "slt  x7, x1, x2";
                32'h00208863: get_mnemonic = "beq  x1, x2, 16";
                32'h00209663: get_mnemonic = "bne  x1, x2, 12";
                32'h00418433: get_mnemonic = "add  x8, x3, x4";
                32'h404184B3: get_mnemonic = "sub  x9, x3, x4";
                32'h06400513: get_mnemonic = "addi x10, x0, 100";
                32'h0000006F: get_mnemonic = "jal  x0, 0";
                default:      get_mnemonic = "nop / unknown";
            endcase
        end
    endfunction

    // Assertion Check Task
    task check_reg;
        input [4:0]   reg_idx;
        input [31:0]  expected_val;
        input [511:0] reg_name;
        reg [31:0] actual_val;
        begin
            actual_val = dut.u_regfile.registers[reg_idx];
            test_count = test_count + 1;
            if (actual_val === expected_val) begin
                pass_count = pass_count + 1;
                $display("  [ASSERTION PASS] %s == %0d (0x%08h)", reg_name, expected_val, actual_val);
            end else begin
                fail_count = fail_count + 1;
                $display("  ===============================================================");
                $display("  [ASSERTION FAIL] %s Mismatch!", reg_name);
                $display("    Expected : %0d (0x%08h)", expected_val, expected_val);
                $display("    Actual   : %0d (0x%08h)", actual_val, actual_val);
                $display("  ===============================================================");
            end
        end
    endtask

    // Main Test Sequence
    integer r;
    initial begin
        $dumpfile("rv32i_top.vcd");
        $dumpvars(0, rv32i_top_tb);

        cycle      = 0;
        test_count = 0;
        pass_count = 0;
        fail_count = 0;
        rst_n      = 1'b1;

        $display("\n================================================================================");
        $display("   STARTING RV32I SINGLE-CYCLE PROCESSOR SYSTEM-LEVEL EXECUTION TEST");
        $display("================================================================================\n");

        // Step 1: Boot Reset Sequence
        $display("[SYSTEM] Applying active-low processor reset...");
        rst_n = 1'b0;
        #25;
        @(negedge clk);
        rst_n = 1'b1;
        $display("[SYSTEM] Reset released. Beginning program execution trace.\n");

        $display("-------------------------------------------------------------------------------------------------------------------------");
        $display(" Cyc |    PC    |   Machine Code   | Disassembly          | ALU Result | RegWrite | BrTaken | Register Write Target");
        $display("-------------------------------------------------------------------------------------------------------------------------");

        // Step 2: Execute program instructions over 15 clock cycles
        for (cycle = 1; cycle <= 14; cycle = cycle + 1) begin
            // Sample combinational signals before clock edge
            #1;
            $display(" %3d | 0x%08h |    0x%08h    | %-20s | 0x%08h |    %b     |    %b    | x%02d <= 0x%08h",
                     cycle, pc_out, instruction_out, get_mnemonic(instruction_out),
                     alu_result_out, reg_write_out, branch_taken_out,
                     instruction_out[11:7], rd_data_out);

            @(posedge clk);
        end

        #5;
        $display("-------------------------------------------------------------------------------------------------------------------------\n");

        // Step 3: Comprehensive Architectural Assertions
        $display("================================================================================");
        $display("   ARCHITECTURAL REGISTER FILE VERIFICATION ASSERTIONS");
        $display("================================================================================");

        check_reg(5'd0,  32'd0,  "x0 (Zero)");
        check_reg(5'd1,  32'd10, "x1 (addi x1, 10)");
        check_reg(5'd2,  32'd25, "x2 (addi x2, 25)");
        check_reg(5'd3,  32'd35, "x3 (add  x3 = 10 + 25)");
        check_reg(5'd4,  32'd15, "x4 (sub  x4 = 25 - 10)");
        check_reg(5'd5,  32'd8,  "x5 (and  x5 = 10 & 25)");
        check_reg(5'd6,  32'd27, "x6 (or   x6 = 10 | 25)");
        check_reg(5'd7,  32'd1,  "x7 (slt  x7 = 10 < 25)");
        check_reg(5'd8,  32'd0,  "x8 (add  x8, bypassed by BNE, must be 0)");
        check_reg(5'd9,  32'd0,  "x9 (sub  x9, bypassed by BNE, must be 0)");
        check_reg(5'd10, 32'd100,"x10 (addi x10, 100, Branch Target)");

        // Step 4: Full Register Dump Table (x0 - x15)
        $display("\n================================================================================");
        $display("   FINAL ARCHITECTURAL REGISTER STATE DUMP (x0 - x15)");
        $display("================================================================================");
        for (r = 0; r < 16; r = r + 4) begin
            $display("   x%02d: 0x%08h (%5d) | x%02d: 0x%08h (%5d) | x%02d: 0x%08h (%5d) | x%02d: 0x%08h (%5d)",
                     r,   dut.u_regfile.registers[r],   dut.u_regfile.registers[r],
                     r+1, dut.u_regfile.registers[r+1], dut.u_regfile.registers[r+1],
                     r+2, dut.u_regfile.registers[r+2], dut.u_regfile.registers[r+2],
                     r+3, dut.u_regfile.registers[r+3], dut.u_regfile.registers[r+3]);
        end

        // Step 5: Test Execution Summary
        $display("\n================================================================================");
        $display("   RV32I PROCESSOR SYSTEM EXECUTION SUMMARY");
        $display("================================================================================");
        $display("   Total Instruction Cycles : %0d", cycle - 1);
        $display("   Total Assertions Checked : %0d", test_count);
        $display("   Assertions Passed        : %0d", pass_count);
        $display("   Assertions Failed        : %0d", fail_count);
        $display("================================================================================");

        if (fail_count == 0) begin
            $display("\n   [RESULT: SUCCESS] RV32I PROCESSOR EXECUTED ENTIRE TEST SUITE FLAWLESSLY!\n");
            $finish(0);
        end else begin
            $display("\n   [RESULT: FAILURE] %0d ASSERTIONS FAILED!\n", fail_count);
            $fatal(1, "RV32I processor simulation failed with assertion errors.");
        end
    end

endmodule
