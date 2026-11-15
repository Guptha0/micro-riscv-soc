#===============================================================================
# Makefile for RV32IM System-on-Chip (SoC) Project
#===============================================================================

SHELL := /bin/bash

.PHONY: all test soc top pc regfile alu waves clean help

all: test

help:
	@echo "RV32IM SoC Build & Test Targets:"
	@echo "  make test      - Run all 5 unit and system test suites"
	@echo "  make soc       - Run 5-stage pipelined RV32IM SoC testbench"
	@echo "  make waves     - Launch GTKWave with pre-loaded signal waveforms"
	@echo "  make top       - Run single-cycle RV32I processor testbench"
	@echo "  make pc        - Run PC fetch and ROM testbench"
	@echo "  make regfile   - Run 32x32 register file testbench"
	@echo "  make alu       - Run 32-bit ALU testbench"
	@echo "  make clean     - Clean simulation binaries, waves, and logs"

test:
	@bash scripts/run_all.sh

soc:
	@bash scripts/run_soc.sh

waves:
	@bash scripts/view_waves.sh

top:
	@bash scripts/run_top.sh

pc:
	@bash scripts/run_pc.sh

regfile:
	@bash scripts/run_regfile.sh

alu:
	@bash scripts/run_alu.sh

clean:
	@rm -rf sim/bin/* sim/waves/* sim/logs/*
	@echo "Cleaned simulation binaries, waveforms, and logs."
