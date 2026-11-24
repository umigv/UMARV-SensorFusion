#!/usr/bin/env bash
# Self-checking test of src/lidar_motion.sv with src/lidar_cordic.sv, using Synopsys VCS.
set -eu
cd "$(dirname "$0")"
mkdir -p build && cd build
vcs -sverilog -full64 -q -timescale=1ns/1ps ../../../src/lidar_motion.sv ../../../src/lidar_cordic.sv ../tb_lidar_motion.sv -top tb_lidar_motion -o simv_lidar_motion
./simv_lidar_motion
