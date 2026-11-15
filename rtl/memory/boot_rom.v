//==============================================================================
// Module Name: boot_rom
// Description: Decoupled Pre-loaded Instruction Memory (Boot ROM) for RISC-V SoC.
//
// Separates the CPU execution firmware from the SoC chipset architecture.
// Contains comprehensive architectural verification routines exercising:
//   1. Full data forwarding & hazard detection
//   2. L1 Cache hits, misses, line evictions & write-backs
//   3. Non-cacheable MMIO UART and CLINT timer peripheral access
//   4. Dynamic branch prediction loops (BTB/BHT)
//   5. Hardware multiplier & divider (RV32M)
//   6. Atomic memory operations (LR.W / SC.W) with bus-snooping invalidation
//   7. M-Mode Machine Timer Interrupts, Traps, and MRET
//   8. Complete RV32I Base Integer Operations:
//      - Sub-word stores & loads (SB, SH, LBU, LH, LW)
//      - Immediate shifts & logic (SLLI, SRAI, XORI, SLTIU)
//   9. Synchronous M-Mode Exception Handling (ECALL -> mcause 0x0000000B)
//  10. Privileged HPM Telemetry Counters (mcycle, minstret)
//==============================================================================

`timescale 1ns / 1ps

module boot_rom #(
    parameter ROM_DEPTH = 128
)(
    input  wire        clk,
    input  wire [31:0] addr,
    output wire [31:0] rdata
);

    reg [31:0] rom_mem [0:ROM_DEPTH-1];
    integer i;

    initial begin
        for (i = 0; i < ROM_DEPTH; i = i + 1) begin
            rom_mem[i] = 32'h0000_0013; // Default NOP
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

        // 8. RV32A Atomic Memory Extension Verification (LR.W & SC.W with Bus Snooping)
        rom_mem[26] = 32'h1000A8AF; // 0x68: lr.w x17, 0(x1)     (Load-Reserved from address x1=10, sets reservation)
        rom_mem[27] = 32'h1820A92F; // 0x6C: sc.w x18, x2, 0(x1) (Store-Conditional: reservation valid -> x18 = 0 SUCCESS)
        rom_mem[28] = 32'h1820A9AF; // 0x70: sc.w x19, x2, 0(x1) (Store-Conditional: reservation cleared -> x19 = 1 FAIL)
        rom_mem[29] = 32'h1000AA2F; // 0x74: lr.w x20, 0(x1)     (Load-Reserved from address x1=10 -> sets reservation)
        rom_mem[30] = 32'h1820AAAF; // 0x78: sc.w x21, x2, 0(x1) (Snoop Invalidation: TB asserts snoop_we -> x21 = 1 FAIL!)

        // 9. Privileged HPM Telemetry CSR Initial Reads
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
        rom_mem[45] = 32'h300E9073; // 0xB4: csrrw x0, 0x300, x29 (Configure mstatus: MIE=1 -> TRAP 1 FIRES!)

        // 12. Interrupted Path & Resumed Execution after MRET
        rom_mem[46] = 32'h00108093; // 0xB8: addi x1, x1, 1       (Interrupted instruction: x1 increments 10 -> 11 upon MRET)
        rom_mem[47] = 32'h0280006F; // 0xBC: jal  x0, 40          (Jump to 0xE4 for post-trap full RV32I test suite!)

        // 13. M-Mode Trap Handler Routine (Located at mtvec = 0x000000C0)
        rom_mem[48] = 32'h34202F73; // 0xC0: csrrs x30, 0x342, x0 (Read mcause into x30)
        rom_mem[49] = 32'h34102FF3; // 0xC4: csrrs x31, 0x341, x0 (Read mepc into x31)
        rom_mem[50] = 32'h000F4863; // 0xC8: blt   x30, x0, 16    (If mcause < 0 [Interrupt], jump to 0xD8)
        rom_mem[51] = 32'h004F8F93; // 0xCC: addi  x31, x31, 4    (Exception handler: advance mepc past faulting instruction)
        rom_mem[52] = 32'h341F9073; // 0xD0: csrrw x0, 0x341, x31 (Update mepc with return address)
        rom_mem[53] = 32'h30200073; // 0xD4: mret                 (Return from exception trap!)
        rom_mem[54] = 32'h00010437; // 0xD8: lui   x8, 0x10       (Interrupt handler: x8 = 65536)
        rom_mem[55] = 32'h00832823; // 0xDC: sw    x8, 16(x6)     (Clear timer interrupt by moving mtimecmp far ahead)
        rom_mem[56] = 32'h30200073; // 0xE0: mret                 (Return from timer interrupt to mepc=0xB8!)

        // 14. Post-Trap Full RV32I Test Routine (Located at 0xE4)
        rom_mem[57] = 32'h20200023; // 0xE4: sb    x2, 512(x0)    (Store byte: DMEM[512] byte 0 <= 15 [0x0F])
        rom_mem[58] = 32'h20301123; // 0xE8: sh    x3, 514(x0)    (Store halfword: DMEM[512] halfword 1 <= 25 [0x0019])
        rom_mem[59] = 32'h60A02023; // 0xEC: sw    x10, 1536(x0)  (Evict dirty line 512 to main RAM[128]!)
        rom_mem[60] = 32'h20004703; // 0xF0: lbu   x14, 512(x0)   (Load unsigned byte -> x14 = 15)
        rom_mem[61] = 32'h20201783; // 0xF4: lh    x15, 514(x0)   (Load signed halfword -> x15 = 25)
        rom_mem[62] = 32'h20002803; // 0xF8: lw    x16, 512(x0)   (Load full word -> x16 = 0x0019000F)
        rom_mem[63] = 32'h00209893; // 0xFC: slli  x17, x1, 2     (Shift left logical: 11 << 2 = 44)
        rom_mem[64] = 32'h4018D913; // 0x100: srai  x18, x17, 1   (Shift right arithmetic: 44 >> 1 = 22)
        rom_mem[65] = 32'h00F0C993; // 0x104: xori x19, x1, 15   (XOR immediate: 11 ^ 15 = 4)
        rom_mem[66] = 32'h0140BA13; // 0x108: sltiu x20, x1, 20  (Set less than immediate unsigned: 11 < 20 = 1)
        rom_mem[67] = 32'h00000073; // 0x10C: ecall              (TRAP 2: Environment Call Exception -> mcause 0x0000000B!)
        rom_mem[68] = 32'hC0002B73; // 0x110: csrrs x22, 0xC00, x0(Read final mcycle)
        rom_mem[69] = 32'hC0202BF3; // 0x114: csrrs x23, 0xC02, x0(Read final minstret)
        rom_mem[70] = 32'h0000006F; // 0x118: jal   x0, 0        (Execution finished: infinite halt)
    end

    assign rdata = (addr[31:2] < ROM_DEPTH) ? rom_mem[addr[31:2]] : 32'h0000_0013;

endmodule
