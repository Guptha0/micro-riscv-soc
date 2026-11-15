#!/usr/bin/env bash
#===============================================================================
# Script: scripts/run_alu.sh
# Description: Compiles and verifies the 32-bit ALU module.
#===============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SIM_BIN="${ROOT_DIR}/sim/bin/alu_sim"
LOG_FILE="${ROOT_DIR}/sim/logs/simulation.log"
WAVE_DIR="${ROOT_DIR}/sim/waves"

mkdir -p "${ROOT_DIR}/sim/bin" "${ROOT_DIR}/sim/logs" "${WAVE_DIR}"

echo -e "\033[1;36m[ALU TEST] Compiling with iverilog...\033[0m"
iverilog -g2012 -Wall -o "${SIM_BIN}" "${ROOT_DIR}/rtl/core/alu.v" "${ROOT_DIR}/tb/alu_tb.v"

echo -e "\033[1;36m[ALU TEST] Running simulation with vvp...\033[0m"
cd "${WAVE_DIR}"
vvp "${SIM_BIN}" 2>&1 | tee "${LOG_FILE}"

echo -e "\033[1;32m[ALU TEST] Complete! Log: sim/logs/simulation.log, Wave: sim/waves/alu.vcd\033[0m\n"
