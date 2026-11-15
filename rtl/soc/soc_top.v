//==============================================================================
// Module Name: soc_top
// Description: Advanced Modular RISC-V System-on-Chip (SoC) Top-Level Integration
//              Architecture: RV32IMZicsr_Zalrsc Motherboard Subsystem
//
// Integrated Subsystems:
//   1. 5-Stage Pipelined RV32IMZicsr_Zalrsc Processor Core (pipeline_cpu)
//      - Complete RV32I Base Integer Instruction Set (Shifts, Compares, Sub-word Load/Store)
//      - Full Data Forwarding Unit (EX-to-EX, MEM-to-EX)
//      - Load-Use Hazard Detection Unit (1-cycle pipeline stall)
//      - Control Hazard Flushing (speculative branch execution)
//      - Hardware Multiplier & Hardware Restoring Divider (RV32M)
//      - Dynamic Branch Predictor (64-entry BTB + 2-bit Saturating BHT)
//      - RV32A Atomic Memory Extension Controller (LR.W / SC.W with Bus Snooping)
//      - Privileged M-Mode Trap Controller & HPM Telemetry CSRs (mtvec, mepc, mcause, mstatus)
//   2. 1KB Direct-Mapped L1 Instruction Cache (icache) with 16-Byte Lines
//   3. 1KB Direct-Mapped L1 Data Cache (dcache) with 16-Byte Lines, Byte-Masked Writes & Burst Write-Back
//   4. Decoupled Dedicated Boot Instruction ROM (boot_rom)
//   5. 4KB Main Synchronous Data Memory RAM (dmem) with Byte-Level Write Enable
//   6. Memory-Mapped UART Transmitter (uart_tx) at 0x80000000
//   7. Hardware Core Local Interruptor (clint_timer) at 0x02004000 / 0x80000010
//   8. MMIO Interconnect Bus with Non-Cacheable Bypass
//==============================================================================

`timescale 1ns / 1ps

module soc_top #(
    parameter ROM_DEPTH = 128,
    parameter RAM_WORDS = 1024
)(
    input  wire        clk,
    input  wire        rst_n,

    // External Physical & Bus Interfaces
    output wire        uart_tx_pin,      // Serial TX physical line
    output wire        timer_irq_out,    // Hardware Timer Interrupt flag
    input  wire        snoop_we,         // Bus-snooping write strobe for atomics
    input  wire [31:0] snoop_addr,       // Bus-snooping address

    // System Observation / Diagnostic Bus
    output wire [31:0] trace_pc,
    output wire [31:0] trace_wb_data,
    output wire [4:0]  trace_wb_rd,
    output wire        trace_wb_we,
    output wire        trace_stall,
    output wire        trace_flush,
    output wire        trace_cache_hit,
    output wire [31:0] trace_hit_count,
    output wire [31:0] trace_miss_count,
    output wire [63:0] trace_mtime,
    output wire        trace_dcache_hit,
    output wire [31:0] trace_dcache_hits,
    output wire [31:0] trace_dcache_misses,
    output wire [31:0] trace_dcache_evicts,

    // Advanced Telemetry Ports
    output wire        trace_pred_taken,
    output wire        trace_mispredict,
    output wire        trace_sc_success,
    output wire [63:0] trace_csr_mcycle,
    output wire [63:0] trace_csr_minstret,

    // M-Mode Trap & Exception Diagnostics
    output wire        trace_trap_taken,
    output wire        trace_mret_exec,
    output wire [31:0] trace_mtvec,
    output wire [31:0] trace_mepc,
    output wire [31:0] trace_mcause,
    output wire [31:0] trace_mstatus
);

    //==========================================================================
    // 1. INSTRUCTION ROM & L1 I-CACHE SUBSYSTEM (16-BYTE LINES)
    //==========================================================================
    wire [31:0] cpu_pc;
    wire        cpu_read_en;
    wire [31:0] icache_instruction;
    wire        icache_stall;
    wire        cache_hit;
    wire [31:0] rom_addr;
    wire [31:0] rom_instruction;

    // Decoupled Pre-loaded Instruction Memory ROM
    boot_rom #(
        .ROM_DEPTH(ROM_DEPTH)
    ) u_boot_rom (
        .clk   (clk),
        .addr  (rom_addr),
        .rdata (rom_instruction)
    );

    // 1KB Direct-Mapped L1 Instruction Cache Instance (16-Byte Lines)
    icache #(
        .ENABLE_MISS_STALL(1) // Production multi-cycle burst refill enabled
    ) u_icache (
        .clk             (clk),
        .rst_n           (rst_n),
        .cpu_pc          (cpu_pc),
        .cpu_read_en     (cpu_read_en),
        .cpu_instruction (icache_instruction),
        .cache_hit       (cache_hit),
        .cache_stall     (icache_stall),
        .rom_addr        (rom_addr),
        .rom_instruction (rom_instruction),
        .hit_count       (trace_hit_count),
        .miss_count      (trace_miss_count)
    );

    //==========================================================================
    // 2. 5-STAGE PIPELINED PROCESSOR CORE (RV32IMZicsr_Zalrsc)
    //==========================================================================
    wire [31:0] cpu_dmem_addr;
    wire [31:0] cpu_dmem_wdata;
    wire [3:0]  cpu_dmem_wstrb;
    wire        cpu_dmem_we;
    wire        cpu_dmem_re;
    wire [31:0] cpu_dmem_rdata;
    wire        dcache_stall;
    wire        dcache_hit;

    pipeline_cpu u_cpu (
        .clk                (clk),
        .rst_n              (rst_n),
        .icache_pc          (cpu_pc),
        .icache_read_en     (cpu_read_en),
        .icache_instruction (icache_instruction),
        .icache_stall       (icache_stall),
        .dmem_addr          (cpu_dmem_addr),
        .dmem_wdata         (cpu_dmem_wdata),
        .dmem_wstrb         (cpu_dmem_wstrb),
        .dmem_we            (cpu_dmem_we),
        .dmem_re            (cpu_dmem_re),
        .dmem_rdata         (cpu_dmem_rdata),
        .dmem_stall         (dcache_stall),
        .snoop_we           (snoop_we),
        .snoop_addr         (snoop_addr),
        .debug_if_pc        (),
        .debug_id_pc        (),
        .debug_ex_pc        (),
        .debug_mem_pc       (),
        .debug_wb_pc        (trace_pc),
        .debug_wb_data      (trace_wb_data),
        .debug_wb_rd        (trace_wb_rd),
        .debug_wb_reg_write (trace_wb_we),
        .debug_stall        (trace_stall),
        .debug_flush        (trace_flush),
        .trace_pred_taken   (trace_pred_taken),
        .trace_mispredict   (trace_mispredict),
        .trace_sc_success   (trace_sc_success),
        .trace_csr_mcycle   (trace_csr_mcycle),
        .trace_csr_minstret (trace_csr_minstret),
        .timer_irq_in       (timer_irq_out),
        .trace_trap_taken   (trace_trap_taken),
        .trace_mret_exec    (trace_mret_exec),
        .trace_mtvec        (trace_mtvec),
        .trace_mepc         (trace_mepc),
        .trace_mcause       (trace_mcause),
        .trace_mstatus      (trace_mstatus)
    );

    //==========================================================================
    // 3. 1KB DIRECT-MAPPED L1 DATA CACHE (16-BYTE LINES, BYTE WRITE-MASKING)
    //==========================================================================
    wire [31:0] dmem_addr;
    wire [31:0] dmem_wdata;
    wire [3:0]  dmem_wstrb;
    wire        dmem_we;
    wire        dmem_re;
    reg  [31:0] dmem_rdata;

    dcache #(
        .CACHE_LINES(64)
    ) u_dcache (
        .clk           (clk),
        .rst_n         (rst_n),
        .cpu_addr      (cpu_dmem_addr),
        .cpu_wdata     (cpu_dmem_wdata),
        .cpu_wstrb     (cpu_dmem_wstrb),
        .cpu_mem_write (cpu_dmem_we),
        .cpu_mem_read  (cpu_dmem_re),
        .cpu_rdata     (cpu_dmem_rdata),
        .cpu_stall     (dcache_stall),
        .cache_hit     (dcache_hit),
        .mem_addr      (dmem_addr),
        .mem_wdata     (dmem_wdata),
        .mem_wstrb     (dmem_wstrb),
        .mem_write     (dmem_we),
        .mem_read      (dmem_re),
        .mem_rdata     (dmem_rdata),
        .hit_count     (trace_dcache_hits),
        .miss_count    (trace_dcache_misses),
        .evict_count   (trace_dcache_evicts)
    );

    //==========================================================================
    // 4. MAIN DATA MEMORY (4 KB RAM WITH BYTE-WRITE STROBES)
    //==========================================================================
    wire [31:0] ram_rdata;

    dmem #(
        .RAM_WORDS(RAM_WORDS)
    ) u_dmem (
        .clk       (clk),
        .rst_n     (rst_n),
        .mem_write (dmem_we),
        .mem_read  (dmem_re),
        .wstrb     (dmem_wstrb),
        .addr      (dmem_addr),
        .wdata     (dmem_wdata),
        .rdata     (ram_rdata)
    );

    //==========================================================================
    // 5. MEMORY-MAPPED UART TRANSMITTER (0x80000000 - Non-Cacheable MMIO)
    //==========================================================================
    wire is_uart_addr = (dmem_addr == 32'h8000_0000);
    wire uart_tx_start = dmem_we && is_uart_addr;
    wire uart_busy;
    wire uart_done;

    uart_tx #(
        .DEFAULT_CLKS_PER_BIT(4) // 4 cycles/bit for rapid simulation
    ) u_uart_tx (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_start (uart_tx_start),
        .tx_data  (dmem_wdata[7:0]),
        .tx_pin   (uart_tx_pin),
        .tx_busy  (uart_busy),
        .tx_done  (uart_done)
    );

    //==========================================================================
    // 6. HARDWARE MACHINE TIMER (CLINT - Non-Cacheable MMIO)
    //==========================================================================
    wire is_clint_addr = (dmem_addr[31:24] == 8'h02) || (dmem_addr[31:4] == 28'h8000_001);
    wire clint_we = dmem_we && is_clint_addr;
    wire clint_re = dmem_re && is_clint_addr;
    wire [31:0] clint_rdata;

    clint_timer u_clint_timer (
        .clk          (clk),
        .rst_n        (rst_n),
        .mem_we       (clint_we),
        .mem_re       (clint_re),
        .mem_addr     (dmem_addr),
        .mem_wdata    (dmem_wdata),
        .mem_rdata    (clint_rdata),
        .timer_irq    (timer_irq_out),
        .mtime_out    (trace_mtime),
        .mtimecmp_out ()
    );

    //==========================================================================
    // 7. MMIO READ DATA ROUTING MULTIPLEXER
    //==========================================================================
    always @(*) begin
        if (is_uart_addr) begin
            dmem_rdata = {31'b0, uart_busy};
        end else if (is_clint_addr) begin
            dmem_rdata = clint_rdata;
        end else begin
            dmem_rdata = ram_rdata;
        end
    end

    // Diagnostic Probes
    assign trace_cache_hit  = cache_hit;
    assign trace_dcache_hit = dcache_hit;

endmodule
