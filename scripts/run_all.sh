#!/usr/bin/env bash
#===============================================================================
# Script: scripts/run_all.sh
# Description: Master regression suite that executes all unit and system tests.
#===============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

C_RESET="\033[0m"
C_BOLD="\033[1m"
C_GREEN="\033[32m"
C_RED="\033[31m"
C_CYAN="\033[36m"
C_YELLOW="\033[33m"

echo -e "${C_BOLD}${C_CYAN}================================================================${C_RESET}"
echo -e "${C_BOLD}${C_CYAN}    RV32IM SoC Project Master Regression Test Suite             ${C_RESET}"
echo -e "${C_BOLD}${C_CYAN}================================================================${C_RESET}\n"

TESTS=("run_alu.sh" "run_regfile.sh" "run_pc.sh" "run_top.sh" "run_soc.sh")
NAMES=("32-bit ALU Testbench" "32x32 Register File Testbench" "PC Fetch & ROM Testbench" "Single-Cycle RV32I Core" "5-Stage RV32IM SoC with D-Cache")
RESULTS=()

TOTAL=0
PASSED=0
FAILED=0

for i in "${!TESTS[@]}"; do
    test_script="${SCRIPT_DIR}/${TESTS[$i]}"
    test_name="${NAMES[$i]}"
    TOTAL=$((TOTAL + 1))

    echo -e "${C_BOLD}[TEST ${TOTAL}/${#TESTS[@]}] Running: ${test_name}...${C_RESET}"
    if bash "${test_script}" > /dev/null 2>&1; then
        echo -e "         ${C_GREEN}PASS${C_RESET}"
        RESULTS+=("${C_GREEN}PASS${C_RESET}")
        PASSED=$((PASSED + 1))
    else
        echo -e "         ${C_RED}FAIL${C_RESET}"
        RESULTS+=("${C_RED}FAIL${C_RESET}")
        FAILED=$((FAILED + 1))
    fi
done

echo -e "\n${C_BOLD}${C_CYAN}================================================================${C_RESET}"
echo -e "${C_BOLD}${C_CYAN}   REGRESSION TEST SUMMARY RESULTS                              ${C_RESET}"
echo -e "${C_BOLD}${C_CYAN}================================================================${C_RESET}"
for i in "${!TESTS[@]}"; do
    printf "  %-35s : %b\n" "${NAMES[$i]}" "${RESULTS[$i]}"
done
echo -e "${C_BOLD}${C_CYAN}----------------------------------------------------------------${C_RESET}"
echo -e "  Total Test Suites : ${TOTAL}"
echo -e "  Suites Passed     : ${C_GREEN}${PASSED}${C_RESET}"
echo -e "  Suites Failed     : ${C_RED}${FAILED}${C_RESET}"
echo -e "${C_BOLD}${C_CYAN}================================================================${C_RESET}\n"

if [ ${FAILED} -eq 0 ]; then
    echo -e "${C_BOLD}${C_GREEN}>>> ALL REGRESSION SUITES PASSED FLAWLESSLY! <<<${C_RESET}\n"
    exit 0
else
    echo -e "${C_BOLD}${C_RED}>>> REGRESSION SUITE FAILED WITH ${FAILED} ERRORS! <<<${C_RESET}\n"
    exit 1
fi
