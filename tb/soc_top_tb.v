//==============================================================================
// Testbench Name: soc_top_tb
// Description: Comprehensive system-level verification testbench for the 
//              Advanced RV32IM+A System-on-Chip featuring:
//              - 5-Stage Pipelined RV32IM+A Core with Forwarding & Hazard Detection
//              - M-Mode Trap & Exception Handling (mtvec, mepc, mcause, mstatus, mret)
//              - 16-Byte L1 Instruction Cache & Data Cache with Multi-Cycle Burst Refill
//              - Dynamic Branch Predictor (64-entry BTB + 2-bit Saturating BHT)
//              - RV32A Atomic Memory Extension (LR.W / SC.W & Bus Snooping)
//              - Privileged Architecture HPM Telemetry CSRs (mcycle, minstret)
//              - Memory-Mapped UART Transmitter & CLINT Machine Timer
//==============================================================================

`timescale 1ns / 1ps

module soc_top_tb;

    localparam CLK_PERIOD = 10; // 10 ns clock (100 MHz)

    // Signals
    reg         clk;
    reg         rst_n;

    // External Bus Snooping Inputs
    reg         snoop_we;
    reg  [31:0] snoop_addr;

    // External Physical Outputs
    wire        uart_tx_pin;
    wire        timer_irq_out;

    // Diagnostic Tracing Probes
    wire [31:0] trace_pc;
    wire [31:0] trace_wb_data;
    wire [4:0]  trace_wb_rd;
    wire        trace_wb_we;
    wire        trace_stall;
    wire        trace_flush;
    wire        trace_cache_hit;
    wire [31:0] trace_hit_count;
    wire [31:0] trace_miss_count;
    wire [63:0] trace_mtime;
    wire        trace_dcache_hit;
    wire [31:0] trace_dcache_hits;
    wire [31:0] trace_dcache_misses;
    wire [31:0] trace_dcache_evicts;

    // Bleeding-Edge Telemetry Probes
    wire        trace_pred_taken;
    wire        trace_mispredict;
    wire        trace_sc_success;
    wire [63:0] trace_csr_mcycle;
    wire [63:0] trace_csr_minstret;

    // M-Mode Trap & Exception Probes
    wire        trace_trap_taken;
    wire        trace_mret_exec;
    wire [31:0] trace_mtvec;
    wire [31:0] trace_mepc;
    wire [31:0] trace_mcause;
    wire [31:0] trace_mstatus;

    // Simulation tracking
    integer cycle;
    integer test_count;
    integer pass_count;
    integer fail_count;
    integer btb_pred_taken_count;
    integer btb_zero_penalty_hits;
    integer trap_taken_count;
    integer mret_exec_count;

    // Captured UART characters buffer
    reg [7:0] uart_rx_chars [0:15];
    integer   uart_rx_count;

    // Instantiate SoC Top-Level Module
    soc_top #(
        .ROM_DEPTH(128),
        .RAM_WORDS(1024)
    ) dut (
        .clk                 (clk),
        .rst_n               (rst_n),
        .uart_tx_pin         (uart_tx_pin),
        .timer_irq_out       (timer_irq_out),
        .snoop_we            (snoop_we),
        .snoop_addr          (snoop_addr),
        .trace_pc            (trace_pc),
        .trace_wb_data       (trace_wb_data),
        .trace_wb_rd         (trace_wb_rd),
        .trace_wb_we         (trace_wb_we),
        .trace_stall         (trace_stall),
        .trace_flush         (trace_flush),
        .trace_cache_hit     (trace_cache_hit),
        .trace_hit_count     (trace_hit_count),
        .trace_miss_count    (trace_miss_count),
        .trace_mtime         (trace_mtime),
        .trace_dcache_hit    (trace_dcache_hit),
        .trace_dcache_hits   (trace_dcache_hits),
        .trace_dcache_misses (trace_dcache_misses),
        .trace_dcache_evicts (trace_dcache_evicts),
        .trace_pred_taken    (trace_pred_taken),
        .trace_mispredict    (trace_mispredict),
        .trace_sc_success    (trace_sc_success),
        .trace_csr_mcycle    (trace_csr_mcycle),
        .trace_csr_minstret  (trace_csr_minstret),
        .trace_trap_taken    (trace_trap_taken),
        .trace_mret_exec     (trace_mret_exec),
        .trace_mtvec         (trace_mtvec),
        .trace_mepc          (trace_mepc),
        .trace_mcause        (trace_mcause),
        .trace_mstatus       (trace_mstatus)
    );

    // Free-Running Clock Generator
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    // Monitor Memory-Mapped UART writes in real-time
    always @(posedge clk) begin
        if (rst_n && dut.u_uart_tx.tx_start) begin
            uart_rx_chars[uart_rx_count] = dut.u_uart_tx.tx_data;
            $display("  [UART MMIO TX] Cycle %3d | Emitted Byte: 0x%02h ('%c')",
                     cycle, dut.u_uart_tx.tx_data, dut.u_uart_tx.tx_data);
            uart_rx_count = uart_rx_count + 1;
        end
    end

    // Monitor Hardware Timer Interrupt
    reg timer_logged;
    always @(posedge clk) begin
        if (timer_irq_out && !timer_logged) begin
            $display("  [CLINT IRQ] *** HARDWARE TIMER INTERRUPT ASSERTED at Cycle %0d (mtime=%0d) ***",
                     cycle, trace_mtime);
            timer_logged = 1'b1;
        end
    end

    // Assertion Check Task for Architectural Registers
    task check_reg;
        input [4:0]   reg_idx;
        input [31:0]  expected_val;
        input [511:0] reg_name;
        reg [31:0] actual_val;
        begin
            actual_val = dut.u_cpu.u_regfile.registers[reg_idx];
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
        $dumpfile("soc_top.vcd");
        $dumpvars(0, soc_top_tb);

        cycle                   = 0;
        test_count              = 0;
        pass_count              = 0;
        fail_count              = 0;
        uart_rx_count           = 0;
        timer_logged            = 1'b0;
        btb_pred_taken_count    = 0;
        btb_zero_penalty_hits   = 0;
        trap_taken_count        = 0;
        mret_exec_count         = 0;
        snoop_we                = 1'b0;
        snoop_addr              = 32'd0;
        rst_n                   = 1'b1;

        $display("\n================================================================================");
        $display("   STARTING ADVANCED RV32IM+A SOC FULL REGRESSION EXECUTION");
        $display("   (M-Mode Traps, 16-Byte L1 Caches, Dynamic BTB, Atomics LR/SC, CSRs)");
        $display("================================================================================\n");

        // Step 1: Boot Reset Sequence
        $display("[SYSTEM] Asserting processor active-low reset...");
        rst_n = 1'b0;
        #25;
        @(negedge clk);
        rst_n = 1'b1;
        $display("[SYSTEM] Reset released. Pipelined CPU core execution commenced.\n");

        $display("-----------------------------------------------------------------------------------------------------------------------------------------");
        $display(" Cyc |  WB PC     |  WB RegWrite  | Stall | Flush | PredTk | Mispr | D-Hits/Miss/Evict | System Event");
        $display("-----------------------------------------------------------------------------------------------------------------------------------------");

        // Step 2: Cycle-by-Cycle Pipeline Trace over 320 Clock Cycles
        for (cycle = 1; cycle <= 320; cycle = cycle + 1) begin
            @(posedge clk);
            #1; // Settling delay for combinational probes

            if (trace_pred_taken) begin
                btb_pred_taken_count = btb_pred_taken_count + 1;
            end

            // Check if zero-penalty branch prediction occurred
            if (dut.u_cpu.id_ex_is_branch && dut.u_cpu.id_ex_pred_taken && !trace_mispredict && dut.u_cpu.branch_taken_ex) begin
                btb_zero_penalty_hits = btb_zero_penalty_hits + 1;
            end

            // Track M-Mode Trap Events
            if (trace_trap_taken) begin
                trap_taken_count = trap_taken_count + 1;
                $display("  [TRAP FIRED] Cycle %3d | Interrupted PC=0x%08h | Target mtvec=0x%08h | mcause=0x%08h",
                         cycle, trace_mepc, trace_mtvec, trace_mcause);
            end

            if (trace_mret_exec) begin
                mret_exec_count = mret_exec_count + 1;
                $display("  [MRET EXECUTED] Cycle %3d | Returning to mepc=0x%08h | Restored mstatus=0x%08h",
                         cycle, trace_mepc, trace_mstatus);
            end

            $write(" %3d | 0x%08h |  x%02d <= 0x%08h (we=%b) |   %b   |   %b   |   %b    |   %b   |    %2d / %2d / %2d    | ",
                   cycle, trace_pc, trace_wb_rd, trace_wb_data, trace_wb_we,
                   trace_stall, trace_flush, trace_pred_taken, trace_mispredict,
                   trace_dcache_hits[7:0], trace_dcache_misses[7:0], trace_dcache_evicts[7:0]);

            if (trace_stall) begin
                if (dut.u_dcache.cpu_stall) begin
                    $write("[PIPELINE STALL: 16-Byte D-Cache Burst Refill / Writeback]");
                end else if (dut.u_icache.cache_stall) begin
                    $write("[PIPELINE STALL: 16-Byte I-Cache Burst Refill]");
                end else if (dut.u_cpu.stall_m_ext) begin
                    $write("[PIPELINE STALL: Multi-Cycle M-Extension]");
                end else begin
                    $write("[PIPELINE STALL: Load-Use Hazard Bubble]");
                end
            end else if (trace_flush) begin
                if (trace_trap_taken) begin
                    $write("[PIPELINE FLUSH: Trap Vectored to mtvec]");
                end else if (trace_mret_exec) begin
                    $write("[PIPELINE FLUSH: MRET Returning to mepc]");
                end else begin
                    $write("[PIPELINE FLUSH: Branch / Mispredict Taken]");
                end
            end else if (trace_wb_we && (trace_wb_rd != 5'd0)) begin
                $write("[WB Commit: x%02d = 0x%08h (%0d)]", trace_wb_rd, trace_wb_data, trace_wb_data);
            end else begin
                $write("[Pipeline Advancing]");
            end
            $write("\n");
        end

        $display("-----------------------------------------------------------------------------------------------------------------------------------------\n");

        // Step 3: Comprehensive Architectural Assertions
        $display("================================================================================");
        $display("   ARCHITECTURAL & SYSTEM VERIFICATION ASSERTIONS");
        $display("================================================================================");

        // 1. Hardwired zero
        check_reg(5'd0, 32'd0, "x0 (Hardwired Zero)");

        // 2. Pipeline Data Forwarding Tests
        check_reg(5'd2, 32'd15, "x2 (addi x2, x1, 5 -> Forwarded from EX/MEM)");
        check_reg(5'd3, 32'd25, "x3 (add  x3, x2, x1 -> Forwarded from EX/MEM & MEM/WB)");

        // 3. Data Memory & L1 D-Cache Access Tests
        check_reg(5'd4, 32'd25, "x4 (lw x4, 0(x0) from D-Cache -> Immediate Hit)");
        check_reg(5'd5, 32'd40, "x5 (addi x5, x4, 15 -> 1-Cycle Load-Use Stall & Forward)");

        // 4. Memory-Mapped UART Address Test
        check_reg(5'd6, 32'h8000_0000, "x6 (lui x6, 0x80000 -> UART Base Address)");

        // 5. Branch Bypass / Pipeline Flush Tests
        check_reg(5'd9, 32'd0,  "x9 (bypassed by BNE, flushed from pipeline, must be 0)");
        check_reg(5'd10, 32'd200,"x10 (addi x10, 200 -> Branch Target Successfully Reached)");

        // 6. RV32M Extension Execution Tests
        check_reg(5'd11, 32'd150, "x11 (mul x11 = 10 * 15 -> Multiplier)");
        check_reg(5'd12, 32'd20,  "x12 (div x12 = 200 / 10 -> Divider)");
        check_reg(5'd13, 32'd5,   "x13 (rem x13 = 15 % 10 -> Remainder)");

        // 7. L1 D-Cache Conflict, Eviction & Write-Back Tests (16-Byte Lines)
        check_reg(5'd14, 32'd25,  "x14 (lw x14, 0(x0) -> Read back after dirty eviction of 0x400)");
        check_reg(5'd15, 32'd200, "x15 (lw x15, 1024(x0) -> Read back after dirty eviction of 0x00)");
        check_reg(5'd16, 32'd200, "x16 (lw x16, 1024(x0) -> Immediate D-Cache HIT)");

        // 8. Hardware Timer Interrupt Assertion
        test_count = test_count + 1;
        if (timer_irq_out === 1'b1 || timer_logged) begin
            pass_count = pass_count + 1;
            $display("  [ASSERTION PASS] Hardware Timer Interrupt timer_irq_out Asserted (mtime >= mtimecmp)");
        end else begin
            fail_count = fail_count + 1;
            $display("  [ASSERTION FAIL] Hardware Timer Interrupt timer_irq_out is NOT asserted!");
        end

        // 9. UART Character Transmission Assertion
        test_count = test_count + 1;
        if (uart_rx_count >= 3 && uart_rx_chars[0] == 8'd82 && uart_rx_chars[1] == 8'd86) begin
            pass_count = pass_count + 1;
            $display("  [ASSERTION PASS] UART MMIO Stream Emitted Correct Text: 'R', 'V', '\\n'");
        end else begin
            fail_count = fail_count + 1;
            $display("  [ASSERTION FAIL] UART MMIO Stream Did Not Match Expected Characters!");
        end

        // 10. 16-Byte L1 Instruction Cache Hit Assertion
        test_count = test_count + 1;
        if (trace_hit_count > 0) begin
            pass_count = pass_count + 1;
            $display("  [ASSERTION PASS] 1KB 16-Byte L1 I-Cache Verified Active (Hits = %0d, Misses = %0d)",
                     trace_hit_count, trace_miss_count);
        end else begin
            fail_count = fail_count + 1;
            $display("  [ASSERTION FAIL] L1 I-Cache recorded zero hits!");
        end

        // 11. 16-Byte L1 Data Cache Hits Assertion
        test_count = test_count + 1;
        if (trace_dcache_hits > 0) begin
            pass_count = pass_count + 1;
            $display("  [ASSERTION PASS] 1KB 16-Byte L1 D-Cache Hits Verified Active (Hits = %0d)",
                     trace_dcache_hits);
        end else begin
            fail_count = fail_count + 1;
            $display("  [ASSERTION FAIL] L1 D-Cache recorded zero hits!");
        end

        // 12. 16-Byte L1 Data Cache Dirty Line Evictions & Write-Back Assertion
        test_count = test_count + 1;
        if (trace_dcache_evicts > 0) begin
            pass_count = pass_count + 1;
            $display("  [ASSERTION PASS] 1KB 16-Byte L1 D-Cache Burst Evictions Verified Active (Evictions = %0d)",
                     trace_dcache_evicts);
        end else begin
            fail_count = fail_count + 1;
            $display("  [ASSERTION FAIL] L1 D-Cache recorded zero evictions!");
        end

        // 13. RV32A Atomic Memory Extension: LR.W & SC.W Tests
        check_reg(5'd17, 32'd0,  "x17 (lr.w x17, 0(x1) -> Load-Reserved initial memory value)");
        check_reg(5'd18, 32'd0,  "x18 (sc.w x18, x2, 0(x1) -> First SC Success return 0)");
        check_reg(5'd19, 32'd1,  "x19 (sc.w x19, x2, 0(x1) -> Second SC Failure return 1: reservation cleared)");
        check_reg(5'd20, 32'd15, "x20 (lr.w x20, 0(x1) -> Load-Reserved after SC updated memory to 15)");
        check_reg(5'd21, 32'd0,  "x21 (sc.w x21, x2, 0(x1) -> Third SC Success return 0)");

        // 14. Privileged Architecture HPM Telemetry CSR Assertions
        test_count = test_count + 1;
        if (dut.u_cpu.u_regfile.registers[22] > 32'd0) begin
            pass_count = pass_count + 1;
            $display("  [ASSERTION PASS] CSR mcycle Readout x22 == %0d (> 0)",
                     dut.u_cpu.u_regfile.registers[22]);
        end else begin
            fail_count = fail_count + 1;
            $display("  [ASSERTION FAIL] CSR mcycle Readout x22 is 0!");
        end

        test_count = test_count + 1;
        if (dut.u_cpu.u_regfile.registers[23] > 32'd0) begin
            pass_count = pass_count + 1;
            $display("  [ASSERTION PASS] CSR minstret Readout x23 == %0d (> 0 retired instructions)",
                     dut.u_cpu.u_regfile.registers[23]);
        end else begin
            fail_count = fail_count + 1;
            $display("  [ASSERTION FAIL] CSR minstret Readout x23 is 0!");
        end

        test_count = test_count + 1;
        if (dut.u_cpu.u_regfile.registers[24] > 32'd0) begin
            pass_count = pass_count + 1;
            $display("  [ASSERTION PASS] CSR mhpmcounter3 [Branches] Readout x24 == %0d (> 0)",
                     dut.u_cpu.u_regfile.registers[24]);
        end else begin
            fail_count = fail_count + 1;
            $display("  [ASSERTION FAIL] CSR mhpmcounter3 [Branches] Readout x24 is 0!");
        end

        test_count = test_count + 1;
        if (dut.u_cpu.u_regfile.registers[25] >= 32'd1) begin
            pass_count = pass_count + 1;
            $display("  [ASSERTION PASS] CSR mhpmcounter4 [Mispredicts] Readout x25 == %0d (>= 1)",
                     dut.u_cpu.u_regfile.registers[25]);
        end else begin
            fail_count = fail_count + 1;
            $display("  [ASSERTION FAIL] CSR mhpmcounter4 [Mispredicts] Readout x25 is 0!");
        end

        // 15. Dynamic Branch Predictor Hit & Training Assertions
        test_count = test_count + 1;
        if (btb_pred_taken_count > 0) begin
            pass_count = pass_count + 1;
            $display("  [ASSERTION PASS] Dynamic BTB Predicted-Taken Probes Verified Active (%0d cycles)",
                     btb_pred_taken_count);
        end else begin
            fail_count = fail_count + 1;
            $display("  [ASSERTION FAIL] Dynamic BTB predicted taken was never asserted!");
        end

        // 16. Dynamic Branch Loop Counter Down to Zero Assertion
        check_reg(5'd26, 32'd0, "x26 (Dynamic branch loop completed countdown to 0)");

        // 17. M-Mode Trap Configuration Assertions
        check_reg(5'd27, 32'h0000_00C0, "x27 (Configured mtvec Trap Handler Address 0x000000C0)");
        check_reg(5'd28, 32'h0000_0080, "x28 (Configured mie.MTIE Timer Interrupt Mask 0x00000080)");
        check_reg(5'd29, 32'h0000_0008, "x29 (Configured mstatus.MIE Global Interrupt Mask 0x00000008)");

        // 18. M-Mode Trap Handler Assertions
        check_reg(5'd30, 32'h8000_0007, "x30 (Trap Handler Read mcause == 0x80000007 Machine Timer Interrupt)");
        check_reg(5'd31, 32'h0000_00B8, "x31 (Trap Handler Read mepc == 0x000000B8 Interrupted Instruction PC)");

        // 19. Trap Entry and MRET Execution Event Assertions
        test_count = test_count + 1;
        if (trap_taken_count >= 1) begin
            pass_count = pass_count + 1;
            $display("  [ASSERTION PASS] M-Mode Trap Entry Verified (%0d trap taken events)", trap_taken_count);
        end else begin
            fail_count = fail_count + 1;
            $display("  [ASSERTION FAIL] M-Mode Trap was never taken!");
        end

        test_count = test_count + 1;
        if (mret_exec_count >= 1) begin
            pass_count = pass_count + 1;
            $display("  [ASSERTION PASS] M-Mode MRET Instruction Execution Verified (%0d mret events)", mret_exec_count);
        end else begin
            fail_count = fail_count + 1;
            $display("  [ASSERTION FAIL] MRET instruction was never executed!");
        end

        // 20. Resumed Execution Assertion: x1 was 10, incremented to 11 upon MRET return!
        check_reg(5'd1, 32'd11, "x1 (Resumed execution after MRET: addi x1, x1, 1 -> x1 = 11)");

        // Step 4: Final Register State Dump (x0 - x31)
        $display("\n================================================================================");
        $display("   FINAL ARCHITECTURAL REGISTER STATE DUMP (x0 - x31)");
        $display("================================================================================");
        for (r = 0; r < 32; r = r + 4) begin
            $display("   x%02d: 0x%08h (%10d) | x%02d: 0x%08h (%10d) | x%02d: 0x%08h (%10d) | x%02d: 0x%08h (%10d)",
                     r,   dut.u_cpu.u_regfile.registers[r],   dut.u_cpu.u_regfile.registers[r],
                     r+1, dut.u_cpu.u_regfile.registers[r+1], dut.u_cpu.u_regfile.registers[r+1],
                     r+2, dut.u_cpu.u_regfile.registers[r+2], dut.u_cpu.u_regfile.registers[r+2],
                     r+3, dut.u_cpu.u_regfile.registers[r+3], dut.u_cpu.u_regfile.registers[r+3]);
        end

        // Step 5: Test Execution Summary
        $display("\n================================================================================");
        $display("   RV32IM+A ADVANCED SOC FULL REGRESSION EXECUTION SUMMARY");
        $display("================================================================================");
        $display("   Total Simulated Cycles         : %0d", cycle - 1);
        $display("   Total Assertions Checked       : %0d", test_count);
        $display("   Assertions Passed              : %0d", pass_count);
        $display("   Assertions Failed              : %0d", fail_count);
        $display("   16-Byte I-Cache Hit Count      : %0d", trace_hit_count);
        $display("   16-Byte I-Cache Miss Count     : %0d", trace_miss_count);
        $display("   16-Byte D-Cache Hit Count      : %0d", trace_dcache_hits);
        $display("   16-Byte D-Cache Miss Count     : %0d", trace_dcache_misses);
        $display("   16-Byte D-Cache Evict Count    : %0d", trace_dcache_evicts);
        $display("   Dynamic BTB Predicted-Taken    : %0d", btb_pred_taken_count);
        $display("   Trap Entry Events              : %0d", trap_taken_count);
        $display("   MRET Execution Events          : %0d", mret_exec_count);
        $display("   mcause Latched Value           : 0x%08h", dut.u_cpu.u_hpm_csrs.mcause);
        $display("   mepc Latched Value             : 0x%08h", dut.u_cpu.u_hpm_csrs.mepc);
        $display("================================================================================");

        if (fail_count == 0) begin
            $display("\n   [RESULT: SUCCESS] ADVANCED RV32IM+A SOC PASSED ALL TRAP & CACHE ASSERTIONS!\n");
            $finish(0);
        end else begin
            $display("\n   [RESULT: FAILURE] %0d ASSERTIONS FAILED!\n", fail_count);
            $fatal(1, "SoC system testbench failed with assertion errors.");
        end
    end

endmodule
