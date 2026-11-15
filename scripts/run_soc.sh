#!/usr/bin/env bash
#===============================================================================
# Script: scripts/run_soc.sh
# Description: Compiles and verifies the Advanced RV32IMZicsr_Zalrsc Pipelined System-on-Chip.
#===============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SIM_BIN="${ROOT_DIR}/sim/bin/soc_sim"
LOG_FILE="${ROOT_DIR}/sim/logs/soc_simulation.log"
WAVE_DIR="${ROOT_DIR}/sim/waves"

mkdir -p "${ROOT_DIR}/sim/bin" "${ROOT_DIR}/sim/logs" "${WAVE_DIR}"

echo -e "\033[1;36m================================================================\033[0m"
echo -e "\033[1;36m    Advanced RV32IMZicsr_Zalrsc Pipelined SoC Simulation Runner \033[0m"
echo -e "\033[1;36m================================================================\033[0m"

echo -e "\n\033[1m[1/2] Compiling RTL and Testbench with iverilog...\033[0m"
iverilog -g2012 -Wall -o "${SIM_BIN}" \
    "${ROOT_DIR}/rtl/core/alu.v" \
    "${ROOT_DIR}/rtl/core/regfile.v" \
    "${ROOT_DIR}/rtl/core/multiplier.v" \
    "${ROOT_DIR}/rtl/core/divider.v" \
    "${ROOT_DIR}/rtl/core/btb_bht.v" \
    "${ROOT_DIR}/rtl/core/atomics.v" \
    "${ROOT_DIR}/rtl/core/hpm_csrs.v" \
    "${ROOT_DIR}/rtl/peripherals/uart_tx.v" \
    "${ROOT_DIR}/rtl/peripherals/clint_timer.v" \
    "${ROOT_DIR}/rtl/memory/boot_rom.v" \
    "${ROOT_DIR}/rtl/memory/icache.v" \
    "${ROOT_DIR}/rtl/memory/dcache.v" \
    "${ROOT_DIR}/rtl/memory/dmem.v" \
    "${ROOT_DIR}/rtl/core/pipeline_cpu.v" \
    "${ROOT_DIR}/rtl/soc/soc_top.v" \
    "${ROOT_DIR}/tb/soc_top_tb.v"

echo -e "      \033[32mCompilation successful.\033[0m"

echo -e "\n\033[1m[2/2] Running Simulation with vvp...\033[0m"
cd "${WAVE_DIR}"
vvp "${SIM_BIN}" 2>&1 | tee "${LOG_FILE}"

echo -e "\n\033[1;32m[SoC TEST] Simulation complete!\033[0m"
echo -e "Waveform log : \033[1msim/waves/soc_top.vcd\033[0m"
echo -e "Execution log: \033[1msim/logs/soc_simulation.log\033[0m\n"
