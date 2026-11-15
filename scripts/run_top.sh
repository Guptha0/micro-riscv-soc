#!/usr/bin/env bash
#===============================================================================
# Script: scripts/run_top.sh
# Description: Compiles and verifies the Single-Cycle RV32I Processor Core.
#===============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SIM_BIN="${ROOT_DIR}/sim/bin/rv32i_sim"
LOG_FILE="${ROOT_DIR}/sim/logs/top_simulation.log"
WAVE_DIR="${ROOT_DIR}/sim/waves"

mkdir -p "${ROOT_DIR}/sim/bin" "${ROOT_DIR}/sim/logs" "${WAVE_DIR}"

echo -e "\033[1;36m[RV32I TOP TEST] Compiling with iverilog...\033[0m"
iverilog -g2012 -Wall -o "${SIM_BIN}" \
    "${ROOT_DIR}/rtl/core/alu.v" \
    "${ROOT_DIR}/rtl/core/regfile.v" \
    "${ROOT_DIR}/rtl/core/pc_fetch.v" \
    "${ROOT_DIR}/rtl/core/rv32i_top.v" \
    "${ROOT_DIR}/tb/rv32i_top_tb.v"

echo -e "\033[1;36m[RV32I TOP TEST] Running simulation with vvp...\033[0m"
cd "${WAVE_DIR}"
vvp "${SIM_BIN}" 2>&1 | tee "${LOG_FILE}"

echo -e "\033[1;32m[RV32I TOP TEST] Complete! Log: sim/logs/top_simulation.log, Wave: sim/waves/rv32i_top.vcd\033[0m\n"
