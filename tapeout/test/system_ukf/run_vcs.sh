#!/usr/bin/env bash
# Whole-chip test with the real Kalman filter in its socket, on Synopsys VCS. Then the logged
# measurements are replayed through the exact model of the filter and compared bit for bit.
set -eu
cd "$(dirname "$0")"
mkdir -p build && cd build
S=../../../src
vcs -sverilog -full64 -q -timescale=1ps/1ps +define+WITH_UKF $S/v16PacketParser.v $S/lidar_vlp16_rx.sv $S/lidar_cordic.sv $S/mii_recv.sv $S/mii_send.sv \
    $S/async_fifo.sv $S/reset_release.sv $S/lidar_laser_table.sv $S/lidar_motion.sv $S/lidar_latch_mem.sv $S/lidar_point_buffer.sv \
    $S/lidar_xyz.sv $S/lidar_frame_tx.sv $S/lidar_top.sv $S/timestamper.sv $S/spi_regs.sv $S/fp_serial.sv $S/ukf_tables.sv $S/ukf_top.sv \
    $S/tt_um_umigv_sensorfusion.sv ../tb_system_ukf.sv -top tb_system_ukf -o simv_system_ukf
./simv_system_ukf | grep -a -E "WRONG|wrong|frame carries|checks|TEST|states"
python3 -s ../check_sys.py sys_log.txt ../../ukf
