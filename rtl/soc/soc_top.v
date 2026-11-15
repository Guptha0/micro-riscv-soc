//==============================================================================
// Module Name: soc_top
// Description: Advanced RISC-V System-on-Chip (SoC) Top-Level Integration
//
// Integrated Subsystems:
//   1. 5-Stage Pipelined RV32IM+A Processor Core (pipeline_cpu)
//      - Full Data Forwarding Unit (EX-to-EX, MEM-to-EX)
//      - Load-Use Hazard Detection Unit (1-cycle pipeline stall)
//      - Control Hazard Flushing (speculative branch execution)
//      - Hardware Multiplier & Hardware Restoring Divider (RV32M)
//      - Dynamic Branch Predictor (64-entry BTB + 2-bit Saturating BHT)
//      - RV32A Atomic Memory Extension Controller (LR.W / SC.W)
//      - Privileged M-Mode Trap Controller & HPM Telemetry CSRs (mtvec, mepc, mcause, mstatus)
//   2. 1KB Direct-Mapped L1 Instruction Cache (icache) with 16-Byte Lines
//   3. 1KB Direct-Mapped L1 Data Cache (dcache) with 16-Byte Lines (Write-Back/Allocate)
//   4. Pre-loaded Instruction Memory ROM (rom_mem)
//   5. 4KB Main Synchronous Data Memory RAM (dmem)
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

    // Bleeding-Edge Research Telemetry Ports
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

    // Embedded Instruction ROM Storage (Pre-loaded SoC Program)
    reg [31:0] rom_mem [0:ROM_DEPTH-1];
    integer i;

    initial begin
        for (i = 0; i < ROM_DEPTH; i = i + 1) begin
            rom_mem[i] = 32'h0000_0013; // NOP
        end

        // 1. Pipeline Forwarding Sequence
        rom_mem[0]  = 32'h00A00093; // 0x00: addi x1, x0, 10
        rom_mem[1]  = 32'h00508113; // 0x04: addi x2, x1, 5   (RAW hazard on x1 -> Forwarded from EX/MEM!)
        rom_mem[2]  = 32'h001101B3; // 0x08: add  x3, x2, x1  (RAW hazard on x2, x1 -> Forwarded! x3 = 25)

        // 2. Data Memory & L1 D-Cache Access (Write-Allocate & Hit)
        rom_mem[3]  = 32'h00302023; // 0x0C: sw   x3, 0(x0)   (Store x3=25 to DMEM[0] -> D-Cache Write-Allocate Miss)
        rom_mem[4]  = 32'h00002203; // 0x10: lw   x4, 0(x0)   (Load word into x4 from 0x00 -> D-Cache HIT! x4 = 25)
        rom_mem[5]  = 32'h00F20293; // 0x14: addi x5, x4, 15  (Load-Use hazard on x4 -> 1-cycle STALL! x5 = 40)

        // 3. Memory-Mapped UART Transmission Sequence (0x80000000 - Non-cacheable MMIO Bypass)
        rom_mem[6]  = 32'h80000337; // 0x18: lui  x6, 0x80000 (x6 = 0x80000000)
        rom_mem[7]  = 32'h05200393; // 0x1C: addi x7, x0, 82  (ASCII 'R')
        rom_mem[8]  = 32'h00732023; // 0x20: sw   x7, 0(x6)   (Send 'R' to UART)
        rom_mem[9]  = 32'h05600393; // 0x24: addi x7, x0, 86  (ASCII 'V')
        rom_mem[10] = 32'h00732023; // 0x28: sw   x7, 0(x6)   (Send 'V' to UART)
        rom_mem[11] = 32'h00A00393; // 0x2C: addi x7, x0, 10  (ASCII '\n')
        rom_mem[12] = 32'h00732023; // 0x30: sw   x7, 0(x6)   (Send '\n' to UART)

        // 4. Hardware Machine Timer Interrupt Configuration (mtimecmp = 50 - Non-cacheable MMIO Bypass)
        rom_mem[13] = 32'h03200413; // 0x34: addi x8, x0, 50
        rom_mem[14] = 32'h00832823; // 0x38: sw   x8, 16(x6)  (Store to mtimecmp at 0x80000010)

        // 5. Branch Execution Sequence
        rom_mem[15] = 32'h00209663; // 0x3C: bne  x1, x2, 12  (10 != 15 -> TAKEN to 0x48!)
        rom_mem[16] = 32'h3E700493; // 0x40: addi x9, x0, 999 (FLUSHED ON INITIAL UNPREDICTED MISPREDICT!)
        rom_mem[17] = 32'h37800493; // 0x44: addi x9, x0, 888 (FLUSHED BY BRANCH!)
        rom_mem[18] = 32'h0C800513; // 0x48: addi x10, x0, 200 (Branch target reached!)

        // 6. RV32M Extension Execution (MUL, DIV, REM)
        rom_mem[19] = 32'h022085B3; // 0x4C: mul  x11, x1, x2  (10 * 15 = 150 -> Multiplier!)
        rom_mem[20] = 32'h02154633; // 0x50: div  x12, x10, x1 (200 / 10 = 20 -> Multi-cycle Divider!)
        rom_mem[21] = 32'h021166B3; // 0x54: rem  x13, x2, x1  (15 % 10 = 5  -> Multi-cycle Divider!)

        // 7. L1 D-Cache Conflict, Eviction & Write-Back Stress Verification
        rom_mem[22] = 32'h40A02023; // 0x58: sw   x10, 1024(x0) (Store 200 to 0x400 -> Evicts dirty 0x00 to DMEM)
        rom_mem[23] = 32'h00002703; // 0x5C: lw   x14, 0(x0)    (Load from 0x00 -> x14 = 25)
        rom_mem[24] = 32'h40002783; // 0x60: lw   x15, 1024(x0) (Load from 0x400 -> x15 = 200)
        rom_mem[25] = 32'h40002803; // 0x64: lw   x16, 1024(x0) (Load from 0x400 again -> D-Cache HIT! x16 = 200)

        // 8. RV32A Atomic Memory Extension Verification (LR.W & SC.W)
        rom_mem[26] = 32'h1000A8AF; // 0x68: lr.w x17, 0(x1)     (Load-Reserved from address x1=10, sets reservation)
        rom_mem[27] = 32'h1820A92F; // 0x6C: sc.w x18, x2, 0(x1) (Store-Conditional: reservation valid -> x18 = 0 SUCCESS)
        rom_mem[28] = 32'h1820A9AF; // 0x70: sc.w x19, x2, 0(x1) (Store-Conditional: reservation cleared -> x19 = 1 FAIL)
        rom_mem[29] = 32'h1000AA2F; // 0x74: lr.w x20, 0(x1)     (Load-Reserved from address x1=10 -> sets reservation)
        rom_mem[30] = 32'h1820AAAF; // 0x78: sc.w x21, x2, 0(x1) (Store-Conditional: reservation valid -> x21 = 0 SUCCESS)

        // 9. Privileged HPM Telemetry CSR Reads
        rom_mem[31] = 32'hC0002B73; // 0x7C: csrrs x22, 0xC00, x0 (Read mcycle counter into x22)
        rom_mem[32] = 32'hC0202BF3; // 0x80: csrrs x23, 0xC02, x0 (Read minstret counter into x23)
        rom_mem[33] = 32'hB0302C73; // 0x84: csrrs x24, 0xB03, x0 (Read mhpmcounter3 [branches] into x24)
        rom_mem[34] = 32'hB0402CF3; // 0x88: csrrs x25, 0xB04, x0 (Read mhpmcounter4 [mispredicts] into x25)

        // 10. Dynamic Branch Predictor Training & Zero-Penalty Hit Loop
        rom_mem[35] = 32'h00300D13; // 0x8C: addi x26, x0, 3      (Initialize loop counter x26 = 3)
        rom_mem[36] = 32'hFFFD0D13; // 0x90: addi x26, x26, -1    (Loop start: decrement counter rs1=x26)
        rom_mem[37] = 32'hFE0D1EE3; // 0x94: bne  x26, x0, -4     (Iter 1 trains BTB; Iter 2 PREDICTED TAKEN with 0 stalls!)

        // 11. M-Mode Trap & Interrupt Configuration Sequence
        rom_mem[38] = 32'h0C000D93; // 0x98: addi x27, x0, 192    (Trap handler target address 0x000000C0)
        rom_mem[39] = 32'h305D9073; // 0x9C: csrrw x0, 0x305, x27 (Configure mtvec = 0x000000C0)
        rom_mem[40] = 32'h08000E13; // 0xA0: addi x28, x0, 128    (MTIE mask = bit 7 = 0x80)
        rom_mem[41] = 32'h304E1073; // 0xA4: csrrw x0, 0x304, x28 (Configure mie: MTIE = 1)
        rom_mem[42] = 32'h00A00413; // 0xA8: addi x8, x0, 10      (Set mtimecmp threshold = 10, mtime is ~100+)
        rom_mem[43] = 32'h00832823; // 0xAC: sw   x8, 16(x6)      (Write to mtimecmp -> timer_irq_out is high!)
        rom_mem[44] = 32'h00800E93; // 0xB0: addi x29, x0, 8      (MIE mask = bit 3 = 0x08)
        rom_mem[45] = 32'h300E9073; // 0xB4: csrrw x0, 0x300, x29 (Configure mstatus: MIE=1 -> TRAP TRIGGERED!)

        // 12. Interrupted Path & Resumed Execution after MRET
        rom_mem[46] = 32'h00108093; // 0xB8: addi x1, x1, 1       (Interrupted instruction: x1 increments 10 -> 11 upon MRET)
        rom_mem[47] = 32'h0000006F; // 0xBC: jal  x0, 0           (Program completed: infinite halt)

        // 13. M-Mode Trap Handler Routine (Located at mtvec = 0x000000C0)
        rom_mem[48] = 32'h34202F73; // 0xC0: csrrs x30, 0x342, x0 (Read mcause into x30 -> Expected 0x80000007)
        rom_mem[49] = 32'h34102FF3; // 0xC4: csrrs x31, 0x341, x0 (Read mepc into x31 -> Expected 0x000000B8)
        rom_mem[50] = 32'h00010437; // 0xC8: lui  x8, 0x10        (x8 = 0x10000 = 65536)
        rom_mem[51] = 32'h00832823; // 0xCC: sw   x8, 16(x6)      (Clear timer interrupt by moving mtimecmp far ahead)
        rom_mem[52] = 32'h30200073; // 0xD0: mret                 (Return from Trap -> Restores MIE and jumps to mepc=0xB8!)
    end

    // ROM Readout
    assign rom_instruction = (rom_addr[31:2] < ROM_DEPTH) ? rom_mem[rom_addr[31:2]] : 32'h0000_0013;

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
    // 2. 5-STAGE PIPELINED PROCESSOR CORE WITH BLEEDING-EDGE MODULES
    //==========================================================================
    wire [31:0] cpu_dmem_addr;
    wire [31:0] cpu_dmem_wdata;
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
    // 3. 1KB DIRECT-MAPPED L1 DATA CACHE (16-BYTE LINES)
    //==========================================================================
    wire [31:0] dmem_addr;
    wire [31:0] dmem_wdata;
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
        .cpu_mem_write (cpu_dmem_we),
        .cpu_mem_read  (cpu_dmem_re),
        .cpu_rdata     (cpu_dmem_rdata),
        .cpu_stall     (dcache_stall),
        .cache_hit     (dcache_hit),
        .mem_addr      (dmem_addr),
        .mem_wdata     (dmem_wdata),
        .mem_write     (dmem_we),
        .mem_read      (dmem_re),
        .mem_rdata     (dmem_rdata),
        .hit_count     (trace_dcache_hits),
        .miss_count    (trace_dcache_misses),
        .evict_count   (trace_dcache_evicts)
    );

    //==========================================================================
    // 4. MAIN DATA MEMORY (4 KB RAM)
    //==========================================================================
    wire [31:0] ram_rdata;

    dmem #(
        .RAM_WORDS(RAM_WORDS)
    ) u_dmem (
        .clk       (clk),
        .rst_n     (rst_n),
        .mem_write (dmem_we),
        .mem_read  (dmem_re),
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
