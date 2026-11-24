#!/usr/bin/env bash
# Runs the whole-chip test, with the simulation stand-in in the Kalman filter's place, on Synopsys VCS.
set -eu
cd "$(dirname "$0")"
mkdir -p build && cd build
S=../../../src
vcs -sverilog -full64 -q -timescale=1ps/1ps +define+WITH_UKF $S/v16PacketParser.v $S/lidar_vlp16_rx.sv $S/lidar_cordic.sv $S/mii_recv.sv $S/mii_send.sv \
    $S/async_fifo.sv $S/reset_release.sv $S/lidar_laser_table.sv $S/lidar_motion.sv $S/lidar_latch_mem.sv $S/lidar_point_buffer.sv \
    $S/lidar_xyz.sv $S/lidar_frame_tx.sv $S/lidar_top.sv $S/timestamper.sv $S/spi_regs.sv $S/tt_um_umigv_sensorfusion.sv \
    ../ukf_standin.sv ../tb_system.sv -top tb_system -o simv_system
./simv_system | grep -a -E "WRONG|wrong|frame carries|checks|TEST"
