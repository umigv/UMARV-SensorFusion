#!/usr/bin/env bash
# Kalman filter tests on Synopsys VCS. Build files go in ./build.
#   1. the arithmetic unit (fp_serial) against numpy float32, 45,000 vectors
#   2. the filter (ukf_top) against the exact model of the machine in ukf_prog.py, every storage
#      word after every beat, bit for bit
# gen_tb.py also rewrites ../../src/ukf_tables.sv from ukf_prog.py, so the tables in src/ are
# always the ones that were tested.
# The accuracy comparison with the simulation's filter is separate (it takes minutes):
#   python3 -s mc.py <path to UMARV-SensorFusion/sim/rate_study>
set -eu
cd "$(dirname "$0")"
mkdir -p build && cd build
S=../../../src
python3 -s ../genvec.py 30000
vcs -sverilog -full64 -q -timescale=1ns/1ps $S/fp_serial.sv ../tb_fp_serial.sv -top tb_fp_serial -o simv_fp
./simv_fp | grep -a -E "MISMATCH|HANG|vectors"
PYTHONPATH=.. python3 -s ../gen_tb.py 14
cp ukf_tables.sv $S/ukf_tables.sv
vcs -sverilog -full64 -q -timescale=1ns/1ps $S/fp_serial.sv $S/ukf_tables.sv $S/ukf_top.sv ../tb_ukf.sv -top tb_ukf -o simv_ukf
./simv_ukf | grep -a -E "word|HANG|beats,|TEST|constant"
