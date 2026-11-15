#!/usr/bin/env bash
#===============================================================================
# Script: scripts/run_regfile.sh
# Description: Compiles and verifies the 32x32-bit Register File.
#===============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SIM_BIN="${ROOT_DIR}/sim/bin/regfile_sim"
LOG_FILE="${ROOT_DIR}/sim/logs/regfile_simulation.log"
WAVE_DIR="${ROOT_DIR}/sim/waves"

mkdir -p "${ROOT_DIR}/sim/bin" "${ROOT_DIR}/sim/logs" "${WAVE_DIR}"

echo -e "\033[1;36m[REGFILE TEST] Compiling with iverilog...\033[0m"
iverilog -g2012 -Wall -o "${SIM_BIN}" "${ROOT_DIR}/rtl/core/regfile.v" "${ROOT_DIR}/tb/regfile_tb.v"

echo -e "\033[1;36m[REGFILE TEST] Running simulation with vvp...\033[0m"
cd "${WAVE_DIR}"
vvp "${SIM_BIN}" 2>&1 | tee "${LOG_FILE}"

echo -e "\033[1;32m[REGFILE TEST] Complete! Log: sim/logs/regfile_simulation.log, Wave: sim/waves/regfile.vcd\033[0m\n"
