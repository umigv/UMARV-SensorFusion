#!/usr/bin/env bash
# Self-checking test of src/mii_recv.sv with Synopsys VCS. Build files go in ./build.
set -eu
cd "$(dirname "$0")"
mkdir -p build && cd build
vcs -sverilog -full64 -q -timescale=1ns/1ps ../../../src/mii_recv.sv ../../../src/v16PacketParser.v \
    ../../../src/lidar_vlp16_rx.sv ../tb_mii_recv.sv -top tb -o simv_mii_recv
./simv_mii_recv
