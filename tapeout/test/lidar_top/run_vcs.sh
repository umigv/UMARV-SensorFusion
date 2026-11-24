#!/usr/bin/env bash
# Runs the lidar top-level test with Synopsys VCS at several PHY clock rates. Build files go in ./build.
set -eu
cd "$(dirname "$0")"
mkdir -p build && cd build
S=../../../src
vcs -sverilog -full64 -q -timescale=1ps/1ps $S/v16PacketParser.v $S/lidar_vlp16_rx.sv $S/lidar_cordic.sv $S/mii_recv.sv $S/mii_send.sv \
    $S/async_fifo.sv $S/reset_release.sv $S/lidar_laser_table.sv $S/lidar_motion.sv $S/lidar_latch_mem.sv $S/lidar_point_buffer.sv $S/lidar_xyz.sv \
    $S/lidar_frame_tx.sv $S/lidar_top.sv ../tb_lidar_top.sv -top tb_lidar_top -o simv_lidar_top
# receive and transmit PHY clocks, picoseconds per half period: both near 25 MHz, then 0.2% apart each way
# (Ethernet allows 0.02%). Bypass needs the two clocks within about 0.3% of each other: it has only the 16-entry FIFO to absorb the difference.
for pair in "20003 19996" "19997 20004" "20000 20000" "20020 19980" "19980 20020"; do
    set -- $pair
    ./simv_lidar_top +rxper=$1 +txper=$2 | grep -a -E "PHY clock|packets|points|firings|frames|bypass|wrong:|TEST"
done
