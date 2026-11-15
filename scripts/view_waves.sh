#!/usr/bin/env bash
#===============================================================================
# Script: scripts/view_waves.sh
# Description: Launches GTKWave with pre-loaded signals and formatting.
#===============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

VCD_FILE="${ROOT_DIR}/sim/waves/soc_top.vcd"
GTKW_FILE="${ROOT_DIR}/sim/waves/soc_top.gtkw"

if [ ! -f "${VCD_FILE}" ]; then
    echo -e "\033[1;33m[NOTICE] Waveform file not found. Running simulation first...\033[0m"
    bash "${SCRIPT_DIR}/run_soc.sh"
fi

if command -v gtkwave &> /dev/null; then
    echo -e "\033[1;32m[GTKWAVE] Launching GTKWave with pre-configured signal tree...\033[0m"
    gtkwave "${VCD_FILE}" "${GTKW_FILE}" &
else
    echo -e "\033[1;31m[ERROR] gtkwave command not found.\033[0m"
    echo -e "Install with: \033[1msudo apt-get update && sudo apt-get install -y gtkwave\033[0m"
    exit 1
fi
