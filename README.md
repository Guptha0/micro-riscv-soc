# RV32IM+A Advanced System-on-Chip (SoC) Hardware Project

A production-grade 32-bit RISC-V (**RV32IM+A**) processor core and System-on-Chip implemented in synthesizable Verilog-2001/2005, featuring bleeding-edge research modules: dynamic 2-bit branch prediction, atomic memory reservation, and privileged hardware performance monitoring counters. Verified with automated assertion testbenches under Icarus Verilog and VVP.

---

## 📂 Project Directory Structure

The repository is organized following industry-standard VLSI hardware layout conventions:

```text
f:\my_cpu\
├── rtl/                        # Synthesizable Hardware Source Code
│   ├── core/                   # CPU Core Components
│   │   ├── alu.v               # 32-bit Arithmetic Logic Unit
│   │   ├── regfile.v           # 32x32-bit Dual-Read Synchronous Register File
│   │   ├── pc_fetch.v          # Program Counter & ROM fetcher
│   │   ├── multiplier.v        # RV32M Multi-cycle Hardware Multiplier
│   │   ├── divider.v           # RV32M Restoring Multi-cycle Hardware Divider
│   │   ├── btb_bht.v           # 64-Entry BTB + 2-Bit Saturating BHT Branch Predictor
│   │   ├── atomics.v           # RV32A Atomic Memory Extension Controller (LR.W / SC.W)
│   │   ├── hpm_csrs.v          # Privileged Architecture HPM Telemetry CSRs (mcycle, minstret)
│   │   ├── rv32i_top.v         # Single-Cycle Integrated RV32I Processor Core
│   │   └── pipeline_cpu.v      # 5-Stage Pipelined RV32IM+A Core with Hazard & Prediction
│   │
│   ├── memory/                 # Caches & Storage Hierarchy
│   │   ├── icache.v            # 1KB Direct-Mapped L1 Instruction Cache
│   │   ├── dcache.v            # 1KB Direct-Mapped L1 Data Cache (Write-Back/Allocate)
│   │   └── dmem.v              # 4KB Synchronous Main Data RAM (1024 words)
│   │
│   ├── peripherals/            # Memory-Mapped I/O Devices
│   │   ├── uart_tx.v           # Serial UART Transmitter (0x80000000)
│   │   └── clint_timer.v       # 64-bit Real-Time CLINT Machine Timer (0x02004000)
│   │
│   └── soc/                    # System Integration
│       └── soc_top.v           # System-on-Chip Top-Level Integration
│
├── tb/                         # Verification Testbenches
│   ├── alu_tb.v                # Standalone ALU multi-cycle testbench (111 tests)
│   ├── regfile_tb.v            # Register File verification suite (123 tests)
│   ├── pc_fetch_tb.v           # PC Fetch Unit testbench (19 tests)
│   ├── rv32i_top_tb.v          # Single-Cycle Top processor testbench
│   └── soc_top_tb.v            # Full RV32IM+A SoC testbench (38 comprehensive assertions)
│
├── scripts/                    # Automation & Regression Scripts
│   ├── run_alu.sh              # Standalone ALU compilation & simulation
│   ├── run_regfile.sh          # Register File compilation & simulation
│   ├── run_pc.sh               # PC Fetch compilation & simulation
│   ├── run_top.sh              # Single-Cycle Core compilation & simulation
│   ├── run_soc.sh              # 5-Stage SoC compilation & simulation
│   └── run_all.sh              # Master Regression Suite runner
│
├── sim/                        # Simulation Workspace (excluded from git)
│   ├── bin/                    # Compiled simulation binaries (*_sim)
│   ├── waves/                  # GTKWave VCD dump files (*.vcd)
│   └── logs/                   # Execution trace logs (*_simulation.log)
│
├── Makefile                    # Standard build & test automation
├── run_soc.sh                  # Root convenience wrapper -> scripts/run_soc.sh
├── run_all.sh                  # Root convenience wrapper -> scripts/run_all.sh
└── README.md                   # Architectural specifications and documentation
```

---

## ⚡ Architecture Specifications

### 1. 5-Stage RV32IM+A Pipeline Core (`rtl/core/pipeline_cpu.v`)
- **Pipeline Stages**: `IF` $\rightarrow$ `ID` $\rightarrow$ `EX` $\rightarrow$ `MEM` $\rightarrow$ `WB`.
- **Data Forwarding Unit**:
  - `EX-to-EX`: Bypasses `EX/MEM` ALU result directly into `EX` stage inputs.
  - `MEM-to-EX`: Bypasses `MEM/WB` writeback data into `EX` stage inputs.
- **Load-Use Hazard Unit**: Stalls `IF` and `ID` stages for 1 cycle on load-to-use dependency, inserting an execution bubble (NOP) into `ID/EX`.
- **Control Hazard & Dynamic Branch Speculation**:
  - Speculative fetching in `IF` stage guided by `btb_bht.v`.
  - Misprediction resolution in `EX` stage flushes `IF/ID` and `ID/EX` pipelines and redirects PC to correct path. Zero-penalty execution on correct prediction hits!
- **Synchronous Cache Stall Management**: Freezes `IF`, `ID`, `EX`, and `MEM` stages synchronously during D-Cache miss refills or dirty evictions, bubbling `MEM/WB`.

### 2. Dynamic Branch Predictor (`rtl/core/btb_bht.v`)
- **Branch Target Buffer (BTB)**: 64-entry direct-mapped storage mapping PC[7:2] to cached target branch addresses with 24-bit tag comparison.
- **Branch History Table (BHT)**: 2-bit saturating counters per entry:
  - `2'b00`: Strongly Not Taken (SNT)
  - `2'b01`: Weakly Not Taken (WNT)
  - `2'b10`: Weakly Taken (WT)
  - `2'b11`: Strongly Taken (ST)
- **Zero-Penalty Speculation**: Combinational prediction in `IF` stage redirects the next fetch cycle without pipeline bubbles when predicted taken.

### 3. RV32A Atomic Memory Extension (`rtl/core/atomics.v`)
- **Load-Reserved Word (`LR.W`)**: Loads a word from memory and registers an active address reservation in hardware.
- **Store-Conditional Word (`SC.W`)**: Conditionally executes store only if the reservation address matches and is still valid; writes 0 to destination register `rd` on success, or 1 on failure. Always clears the reservation unconditionally.
- **Bus-Snooping Invalidation**: Listens to external bus/DMA write traffic (`snoop_we`, `snoop_addr`) and invalidates reservation if a colliding address write occurs.

### 4. Machine-Mode (M-Mode) Trap & Exception Controller (`rtl/core/hpm_csrs.v`)
- **Standard Control & Status Registers (CSRs)**:
  - `mstatus` (`0x300`): Machine status register managing `MIE` (bit 3: global interrupt enable), `MPIE` (bit 7: previous interrupt enable), and `MPP` (bits 12:11: mode privilege bits hardwired to `2'b11`).
  - `mie` (`0x304`): Machine interrupt enable register managing `MTIE` (bit 7: machine timer interrupt enable).
  - `mtvec` (`0x305`): Machine trap-vector base-address register configuring target trap handler entry address.
  - `mepc` (`0x341`): Machine exception program counter capturing the exact PC of the interrupted instruction.
  - `mcause` (`0x342`): Machine trap cause register recording the exception/interrupt reason (`0x80000007` for Machine Timer Interrupt).
  - `mip` (`0x344`): Machine interrupt pending register reflecting active hardware interrupt lines.
  - `mcycle` (`0xC00` / `0xB00`), `minstret` (`0xC02` / `0xB02`), `mhpmcounter3` (`0xC03`), `mhpmcounter4` (`0xC04`), and `mcountinhibit` (`0x320`).
- **Hardware Trap Sequence**:
  - Asserting `timer_irq_out` from CLINT timer combined with `mstatus.MIE=1` and `mie.MTIE=1` asserts qualified `irq_pending`.
  - Atomically flushes `IF/ID`, `ID/EX`, and `EX/MEM` pipeline registers, latches interrupted PC into `mepc`, writes cause `0x80000007` to `mcause`, disables `MIE` (`mstatus_mie <= 0`), saves previous state in `MPIE` (`mstatus_mpie <= mstatus_mie`), and redirects fetch PC to `mtvec`.
- **M-Mode Return (`mret`) Instruction**:
  - Decoded opcode `32'h30200073` (`SYSTEM`, funct3=0, funct12=0x302).
  - Restores global interrupts (`mstatus_mie <= mstatus_mpie; mstatus_mpie <= 1'b1;`), flushes speculative stages, and vectors PC back to `mepc` to seamlessly resume interrupted execution.

### 5. 16-Byte L1 Data Cache (`rtl/memory/dcache.v`)
- **Geometry**: 1KB direct-mapped cache (64 lines $\times$ 16 bytes/line, 4 words per line).
- **Address Breakdown**:
  - `addr[1:0]`: Byte offset (2 bits)
  - `addr[3:2]`: Word select within line (2 bits, 0..3)
  - `addr[9:4]`: Cache line index (6 bits, 64 lines)
  - `addr[31:10]`: Cache tag (22 bits)
- **Multi-Cycle Burst Policies**:
  - **4-Word Burst Writeback**: When a dirty line eviction occurs, the controller sequentially bursts all 4 words (`data_w0..3`) to main memory before servicing the new line.
  - **4-Word Burst Refill & Write-Allocate**: On load/store misses, bursts 4 words from main memory into the cache line; on stores, in-flight CPU write data updates the targeted word with dirty bit asserted.
  - **Single-Cycle Hit**: 0-stall read/write latency on tag match.
  - **Non-Cacheable MMIO Bypass**: Memory-mapped addresses (UART, CLINT) bypass cache directly.
- **Hardware Telemetry**: Probes for real-time hits, misses, and dirty line evictions.

### 6. 16-Byte L1 Instruction Cache (`rtl/memory/icache.v`)
- **Geometry**: 1KB direct-mapped instruction cache (64 lines $\times$ 16 bytes/line, 4 words per line).
- **Multi-Cycle Burst Refill FSM**:
  - On an instruction miss, stalls the fetch unit and bursts 4 consecutive instruction words (`data_w0..3`) from ROM into the cache line across 4 cycles.
  - Single-cycle 0-stall latency on subsequent hits, providing high spatial locality performance across 4 consecutive instructions per miss.
- **Dynamic Pipeline Decoupling**:
  - While I-Cache is bursting from ROM, `if_id` injects clean `NOP` bubbles into the `ID` stage, allowing downstream execution (`EX`, `MEM`, `WB`) to finish and retire without pipeline lockup or hazard suppression.

### 7. RV32M Integer Multiplication & Division Extension
- **Hardware Multiplier (`rtl/core/multiplier.v`)**:
  - Instructions: `MUL`, `MULH`, `MULHSU`, `MULHU`.
  - Multi-cycle signed/unsigned product generator with full 64-bit precision preservation.
- **Hardware Restoring Divider (`rtl/core/divider.v`)**:
  - Instructions: `DIV`, `DIVU`, `REM`, `REMU`.
  - 32-bit iterative shift-subtract restoring divider.
  - Spec Compliant Corner Cases:
    - Division by zero: `DIV`/`DIVU` returns `32'hFFFF_FFFF` ($-1$), `REM`/`REMU` returns dividend `a` (1-cycle fast exit).
    - Signed overflow (`0x80000000 / -1`): `DIV` returns `0x80000000`, `REM` returns `0` (1-cycle fast exit).

### 8. Memory-Mapped Peripherals
| Peripheral | Base Address | Register Map / Description |
| :--- | :--- | :--- |
| **UART TX** | `0x80000000` | Write: Emits 8-bit character byte over serial line.<br>Read: Bit 0 indicates busy status. |
| **CLINT Timer** | `0x02004000` / `0x80000010` | `0x10`: `mtimecmp[31:0]`, `0x14`: `mtimecmp[63:32]`<br>`0x18`: `mtime[31:0]`, `0x1C`: `mtime[63:32]`<br>Asserts `timer_irq` when `mtime >= mtimecmp`. |
| **Main RAM** | `0x00000000` | 4KB internal data storage (1024 words) for word-aligned loads and stores. |

---

## 🚀 Execution & Build Guide

### Quick Start with Make
Inside WSL:
```bash
make test          # Run all 5 regression test suites
make soc           # Run full 5-stage RV32IM+A SoC testbench
make top           # Run single-cycle CPU testbench
make pc            # Run PC fetch and ROM testbench
make regfile       # Run register file testbench
make alu           # Run ALU testbench
make clean         # Clean simulation binaries, waves, and logs
```

### Running Convenience Scripts
```bash
./run_all.sh       # Master regression runner
./run_soc.sh       # Full SoC simulation
./run_top.sh       # Single-cycle CPU simulation
./run_pc.sh        # PC fetch simulation
./run_regfile.sh   # Register file simulation
./run.sh           # Standalone ALU simulation
```

### Inspect Waveforms
```bash
gtkwave sim/waves/soc_top.vcd &
```
