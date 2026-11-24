#!/usr/bin/env bash
# Runs the SPI register file test with Synopsys VCS. Build files go in ./build.
set -eu
cd "$(dirname "$0")"
mkdir -p build && cd build
vcs -sverilog -full64 -q -timescale=1ns/1ps ../../../src/spi_regs.sv ../tb_spi_regs.sv -top tb_spi_regs -o simv_spi_regs
./simv_spi_regs | grep -a -E "WRONG|checks|TEST"
