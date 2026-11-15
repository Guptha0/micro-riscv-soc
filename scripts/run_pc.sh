#!/usr/bin/env bash
#===============================================================================
# Script: scripts/run_pc.sh
# Description: Compiles and verifies the Program Counter & Instruction ROM unit.
#===============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SIM_BIN="${ROOT_DIR}/sim/bin/pc_sim"
LOG_FILE="${ROOT_DIR}/sim/logs/pc_simulation.log"
WAVE_DIR="${ROOT_DIR}/sim/waves"

mkdir -p "${ROOT_DIR}/sim/bin" "${ROOT_DIR}/sim/logs" "${WAVE_DIR}"

echo -e "\033[1;36m[PC FETCH TEST] Compiling with iverilog...\033[0m"
iverilog -g2012 -Wall -o "${SIM_BIN}" "${ROOT_DIR}/rtl/core/pc_fetch.v" "${ROOT_DIR}/tb/pc_fetch_tb.v"

echo -e "\033[1;36m[PC FETCH TEST] Running simulation with vvp...\033[0m"
cd "${WAVE_DIR}"
vvp "${SIM_BIN}" 2>&1 | tee "${LOG_FILE}"

echo -e "\033[1;32m[PC FETCH TEST] Complete! Log: sim/logs/pc_simulation.log, Wave: sim/waves/pc_fetch.vcd\033[0m\n"
