//==============================================================================
// Testbench Name: regfile_tb
// Description: Comprehensive multi-cycle self-checking testbench with automated
//              assertions for the RV32I 32-bit Register File (regfile.v).
//
// Verification Coverage:
//   1. Reset state verification across all 32 registers.
//   2. Permanent hardwired x0 protection:
//      - Attempting single and multiple writes to x0 with various non-zero bitmasks.
//      - Verifying x0 readback is strictly 32'h00000000 on both rs1 and rs2 ports.
//   3. Sequential multi-cycle write and read across all 31 registers (x1-x31).
//   4. Simultaneous read/write consistency:
//      - Simultaneous writes to rd while reading independent registers on rs1 and rs2.
//      - Dual read ports simultaneously accessing the same register (rs1 == rs2).
//      - Immediate read-after-write latency across clock boundaries.
//   5. Write-enable gating test:
//      - Ensuring register contents remain unmodified when write_enable = 0.
//   6. Multi-cycle randomized stress testing (64 cycles) validated against an
//      independent architectural golden model.
//==============================================================================

`timescale 1ns / 1ps

module regfile_tb;

    // Clock timing parameter: 10 ns period (100 MHz)
    localparam CLK_PERIOD = 10;

    // DUT Signals
    reg         clk;
    reg         rst_n;
    reg         write_enable;
    reg  [4:0]  rs1;
    reg  [4:0]  rs2;
    reg  [4:0]  rd;
    reg  [31:0] rd_data;
    wire [31:0] rs1_data;
    wire [31:0] rs2_data;

    // Tracking counters
    integer cycle_count;
    integer test_count;
    integer pass_count;
    integer fail_count;

    // Architectural Golden Model Shadow Registers
    reg [31:0] shadow_rf [31:0];

    // Instantiate Device Under Test (DUT)
    regfile #(
        .WRITE_FIRST(0) // Default standard dual-port asynchronous read
    ) dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .write_enable (write_enable),
        .rs1          (rs1),
        .rs2          (rs2),
        .rd           (rd),
        .rd_data      (rd_data),
        .rs1_data     (rs1_data),
        .rs2_data     (rs2_data)
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
    // Automated Assertion Task: Check Read Ports
    //--------------------------------------------------------------------------
    task assert_read;
        input [4:0]   check_rs1;
        input [4:0]   check_rs2;
        input [31:0]  exp_rs1_data;
        input [31:0]  exp_rs2_data;
        input [191:0] desc;
        begin
            // Asynchronous read ports settle immediately after address change
            rs1 = check_rs1;
            rs2 = check_rs2;
            #1; // Short combinational settling delay

            test_count = test_count + 1;

            if ((rs1_data === exp_rs1_data) && (rs2_data === exp_rs2_data)) begin
                pass_count = pass_count + 1;
                $display("  [PASS] Cycle %3d | %-28s | x%02d=0x%08h, x%02d=0x%08h",
                         cycle_count, desc, check_rs1, rs1_data, check_rs2, rs2_data);
            end else begin
                fail_count = fail_count + 1;
                $display("  ===============================================================");
                $display("  [FAIL ASSERTION] Cycle %3d | %-28s", cycle_count, desc);
                $display("    Addresses: rs1=x%02d, rs2=x%02d", check_rs1, check_rs2);
                $display("    Expected : rs1_data=0x%08h, rs2_data=0x%08h", exp_rs1_data, exp_rs2_data);
                $display("    Actual   : rs1_data=0x%08h, rs2_data=0x%08h", rs1_data, rs2_data);
                $display("  ===============================================================");
            end
        end
    endtask

    //--------------------------------------------------------------------------
    // Synchronous Write Task: Applies data on negedge, triggers posedge
    //--------------------------------------------------------------------------
    task write_reg;
        input [4:0]  w_rd;
        input [31:0] w_data;
        input        w_enable;
        begin
            @(negedge clk);
            write_enable = w_enable;
            rd           = w_rd;
            rd_data      = w_data;
            @(posedge clk);
            #1; // Wait for flip-flop settling

            // Update software shadow model
            if (rst_n && w_enable && (w_rd != 5'd0)) begin
                shadow_rf[w_rd] = w_data;
            end
        end
    endtask

    //--------------------------------------------------------------------------
    // Test Sequence
    //--------------------------------------------------------------------------
    integer idx;
    reg [31:0] test_val;
    reg [4:0]  rand_w_rd;
    reg [31:0] rand_w_data;
    reg        rand_we;
    reg [4:0]  rand_r_rs1;
    reg [4:0]  rand_r_rs2;

    initial begin
        // Setup VCD waveform generation
        $dumpfile("regfile.vcd");
        $dumpvars(0, regfile_tb);

        // Initialize signals
        cycle_count  = 0;
        test_count   = 0;
        pass_count   = 0;
        fail_count   = 0;
        write_enable = 1'b0;
        rs1          = 5'd0;
        rs2          = 5'd0;
        rd           = 5'd0;
        rd_data      = 32'd0;
        rst_n        = 1'b1;

        for (idx = 0; idx < 32; idx = idx + 1) begin
            shadow_rf[idx] = 32'h0000_0000;
        end

        $display("\n================================================================================");
        $display("   STARTING RV32I REGISTER FILE RIGOROUS VERIFICATION SUITE");
        $display("================================================================================\n");

        //----------------------------------------------------------------------
        // Phase 0: Reset Application & Verification
        //----------------------------------------------------------------------
        $display("--- PHASE 0: Asynchronous Reset Verification ---");
        rst_n = 1'b0;
        #15;
        rst_n = 1'b1;
        @(posedge clk);

        for (idx = 0; idx < 32; idx = idx + 2) begin
            assert_read(idx[4:0], (idx[4:0] + 5'd1), 32'h0000_0000, 32'h0000_0000, "RESET_CHECK_ALL_ZERO");
        end

        //----------------------------------------------------------------------
        // Phase 1: Hardwired x0 Protection Verification
        //----------------------------------------------------------------------
        $display("\n--- PHASE 1: Hardwired Register x0 Protection ---");
        // Attempt 1: Write all-ones to x0
        write_reg(5'd0, 32'hFFFF_FFFF, 1'b1);
        assert_read(5'd0, 5'd0, 32'h0000_0000, 32'h0000_0000, "x0_WRITE_ALL_ONES_BLOCKED");

        // Attempt 2: Write 0xDEADBEEF to x0
        write_reg(5'd0, 32'hDEAD_BEEF, 1'b1);
        assert_read(5'd0, 5'd0, 32'h0000_0000, 32'h0000_0000, "x0_WRITE_DEADBEEF_BLOCKED");

        // Attempt 3: Write 0x12345678 to x0
        write_reg(5'd0, 32'h1234_5678, 1'b1);
        assert_read(5'd0, 5'd0, 32'h0000_0000, 32'h0000_0000, "x0_WRITE_12345678_BLOCKED");

        // Attempt 4: Read x0 on rs1 while reading another register on rs2 during an active x0 write
        @(negedge clk);
        write_enable = 1'b1;
        rd           = 5'd0;
        rd_data      = 32'hCAFE_BABE;
        rs1          = 5'd0;
        rs2          = 5'd0;
        #1;
        assert_read(5'd0, 5'd0, 32'h0000_0000, 32'h0000_0000, "x0_ACTIVE_WRITE_READ_ZERO");
        @(posedge clk);
        #1;
        assert_read(5'd0, 5'd0, 32'h0000_0000, 32'h0000_0000, "x0_POST_WRITE_READ_ZERO");
        write_enable = 1'b0;

        //----------------------------------------------------------------------
        // Phase 2: Sequential Write Across All General Registers (x1 to x31)
        //----------------------------------------------------------------------
        $display("\n--- PHASE 2: Sequential Multi-Cycle Write (x1 - x31) ---");
        for (idx = 1; idx < 32; idx = idx + 1) begin
            // Unique bit pattern per register
            test_val = (idx << 24) | ((32 - idx) << 16) | (idx << 8) | idx;
            write_reg(idx[4:0], test_val, 1'b1);
        end
        write_enable = 1'b0;

        $display("\n--- PHASE 2 (Cont.): Sequential Readback Verification (x1 - x31) ---");
        for (idx = 1; idx < 32; idx = idx + 1) begin
            test_val = (idx << 24) | ((32 - idx) << 16) | (idx << 8) | idx;
            assert_read(idx[4:0], 5'd0, test_val, 32'h0000_0000, "SEQ_READ_MATCH_CHECK");
        end

        //----------------------------------------------------------------------
        // Phase 3: Simultaneous Read and Write Data Consistency
        //----------------------------------------------------------------------
        $display("\n--- PHASE 3: Simultaneous Read / Write Data Consistency ---");

        // Test 3A: Write to x5 while reading x1 and x2 simultaneously on rs1 and rs2
        // Register x1 has shadow_rf[1], x2 has shadow_rf[2]
        @(negedge clk);
        write_enable = 1'b1;
        rd           = 5'd5;
        rd_data      = 32'h5555_AAAA; // New data for x5
        rs1          = 5'd1;          // Simultaneously read x1
        rs2          = 5'd2;          // Simultaneously read x2
        #1;
        // Verify rs1 and rs2 are stable and uncorrupted during active write setup
        assert_read(5'd1, 5'd2, shadow_rf[1], shadow_rf[2], "SIMULT_RD_DIFF_WR_SETUP");

        @(posedge clk); // Clock edge latches new x5
        #1;
        shadow_rf[5] = 32'h5555_AAAA;
        // Verify rs1 and rs2 remain intact right after the write clock edge
        assert_read(5'd1, 5'd2, shadow_rf[1], shadow_rf[2], "SIMULT_RD_DIFF_WR_HOLD");

        // Test 3B: Read updated x5 on rs1 and rs2
        assert_read(5'd5, 5'd5, 32'h5555_AAAA, 32'h5555_AAAA, "READ_UPDATED_SAME_REG_BOTH");

        // Test 3C: Dual-port simultaneous reading of distinct registers
        assert_read(5'd10, 5'd20, shadow_rf[10], shadow_rf[20], "DUAL_PORT_READ_x10_x20");
        assert_read(5'd31, 5'd1,  shadow_rf[31], shadow_rf[1],  "DUAL_PORT_READ_x31_x1");

        //----------------------------------------------------------------------
        // Phase 4: Write-Enable Gating & Data Retention
        //----------------------------------------------------------------------
        $display("\n--- PHASE 4: Write-Enable Gating & Data Retention ---");
        // Present new data for x15 with write_enable = 0
        test_val = shadow_rf[15]; // current value
        write_reg(5'd15, 32'hBAD0_0000, 1'b0); // write_enable = 0!
        assert_read(5'd15, 5'd0, test_val, 32'h0000_0000, "WE_LOW_DATA_NOT_OVERWRITTEN");

        // Now write with write_enable = 1
        write_reg(5'd15, 32'hBEEF_1515, 1'b1);
        assert_read(5'd15, 5'd0, 32'hBEEF_1515, 32'h0000_0000, "WE_HIGH_DATA_UPDATED");

        //----------------------------------------------------------------------
        // Phase 5: Multi-Cycle Randomized Stress Testing (64 Clock Cycles)
        //----------------------------------------------------------------------
        $display("\n--- PHASE 5: 64-Cycle Randomized Stress Testing ---");
        for (idx = 0; idx < 64; idx = idx + 1) begin
            rand_w_rd   = $unsigned($random) % 32;
            rand_w_data = $random;
            rand_we     = $unsigned($random) % 2; // 0 or 1
            rand_r_rs1  = $unsigned($random) % 32;
            rand_r_rs2  = $unsigned($random) % 32;

            // Apply write stimulus
            write_reg(rand_w_rd, rand_w_data, rand_we);

            // Read ports verification against golden shadow model
            assert_read(rand_r_rs1, rand_r_rs2, shadow_rf[rand_r_rs1], shadow_rf[rand_r_rs2], "RAND_STRESS_CYCLE");
        end

        // Final settling cycle
        @(posedge clk);

        //----------------------------------------------------------------------
        // Final Summary Banner
        //----------------------------------------------------------------------
        $display("\n================================================================================");
        $display("   REGISTER FILE TESTBENCH EXECUTION SUMMARY");
        $display("================================================================================");
        $display("   Total Clock Cycles : %0d", cycle_count);
        $display("   Total Tests Run    : %0d", test_count);
        $display("   Tests Passed       : %0d", pass_count);
        $display("   Tests Failed       : %0d", fail_count);
        $display("================================================================================");

        if (fail_count == 0) begin
            $display("\n   [RESULT: SUCCESS] ALL REGISTER FILE ASSERTIONS PASSED PERFECTLY!\n");
            $finish(0);
        end else begin
            $display("\n   [RESULT: FAILURE] %0d REGISTER FILE ASSERTIONS FAILED!\n", fail_count);
            $fatal(1, "Register file testbench failed with assertion errors.");
        end
    end

endmodule
