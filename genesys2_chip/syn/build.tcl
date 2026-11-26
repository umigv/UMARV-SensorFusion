# Vivado 2026.1 project-mode build script for genesys2_chip
set script_dir [file dirname [file normalize [info script]]]
set root_dir   [file normalize "$script_dir/.."]
set tape_src   [file normalize "$root_dir/../tapeout/src"]
set build_dir  "$script_dir/build"
set proj_name  "genesys2_chip"
set part       "xc7k325tffg900-2"

file mkdir $build_dir
create_project -force $proj_name $build_dir -part $part

# Add unmodified ASIC RTL (excluding Sky130 lidar_latch_mem.sv and unused fp32_mac.sv)
set asic_files [list \
    "$tape_src/v16PacketParser.v" \
    "$tape_src/lidar_vlp16_rx.sv" \
    "$tape_src/lidar_cordic.sv" \
    "$tape_src/mii_recv.sv" \
    "$tape_src/mii_send.sv" \
    "$tape_src/async_fifo.sv" \
    "$tape_src/reset_release.sv" \
    "$tape_src/lidar_laser_table.sv" \
    "$tape_src/lidar_motion.sv" \
    "$tape_src/lidar_point_buffer.sv" \
    "$tape_src/lidar_xyz.sv" \
    "$tape_src/lidar_frame_tx.sv" \
    "$tape_src/lidar_top.sv" \
    "$tape_src/timestamper.sv" \
    "$tape_src/spi_regs.sv" \
    "$tape_src/fp_serial.sv" \
    "$tape_src/ukf_tables.sv" \
    "$tape_src/ukf_top.sv" \
    "$tape_src/tt_um_umigv_sensorfusion.sv" \
]

# Add FPGA-only wrapper, RGMII/MDIO shim, LUTRAM point buffer, and self-test harness
set fpga_files [list \
    "$root_dir/rtl/lidar_latch_mem.sv" \
    "$root_dir/rtl/mdio_init_100m.sv" \
    "$root_dir/rtl/rgmii_mii_100m.sv" \
    "$root_dir/rtl/fpga_selftest.sv" \
    "$root_dir/rtl/fpga_top.sv" \
]

add_files -norecurse [concat $asic_files $fpga_files]
add_files -fileset constrs_1 -norecurse "$root_dir/xdc/genesys2_chip.xdc"
set_property top fpga_top [current_fileset]
set_property verilog_define {WITH_UKF FPGA} [current_fileset]

# ---- Generate VIO IP ---------------------------------------------------------
create_ip -name vio -vendor xilinx.com -library ip -module_name vio_ctrl
set_property -dict [list \
    CONFIG.C_NUM_PROBE_IN  {20} \
    CONFIG.C_NUM_PROBE_OUT {11} \
    CONFIG.C_PROBE_IN0_WIDTH  {16} \
    CONFIG.C_PROBE_IN1_WIDTH  {32} \
    CONFIG.C_PROBE_IN2_WIDTH  {32} \
    CONFIG.C_PROBE_IN3_WIDTH  {32} \
    CONFIG.C_PROBE_IN4_WIDTH  {32} \
    CONFIG.C_PROBE_IN5_WIDTH  {32} \
    CONFIG.C_PROBE_IN6_WIDTH  {32} \
    CONFIG.C_PROBE_IN7_WIDTH  {32} \
    CONFIG.C_PROBE_IN8_WIDTH  {32} \
    CONFIG.C_PROBE_IN9_WIDTH  {32} \
    CONFIG.C_PROBE_IN10_WIDTH {32} \
    CONFIG.C_PROBE_IN11_WIDTH {32} \
    CONFIG.C_PROBE_IN12_WIDTH {32} \
    CONFIG.C_PROBE_IN13_WIDTH {32} \
    CONFIG.C_PROBE_IN14_WIDTH {32} \
    CONFIG.C_PROBE_IN15_WIDTH {32} \
    CONFIG.C_PROBE_IN16_WIDTH {32} \
    CONFIG.C_PROBE_IN17_WIDTH {32} \
    CONFIG.C_PROBE_IN18_WIDTH {32} \
    CONFIG.C_PROBE_IN19_WIDTH {32} \
    CONFIG.C_PROBE_OUT0_WIDTH  {1} \
    CONFIG.C_PROBE_OUT1_WIDTH  {1} \
    CONFIG.C_PROBE_OUT2_WIDTH  {1} \
    CONFIG.C_PROBE_OUT3_WIDTH  {1} \
    CONFIG.C_PROBE_OUT4_WIDTH  {1} \
    CONFIG.C_PROBE_OUT5_WIDTH  {7} \
    CONFIG.C_PROBE_OUT6_WIDTH  {8} \
    CONFIG.C_PROBE_OUT7_WIDTH  {1} \
    CONFIG.C_PROBE_OUT8_WIDTH  {1} \
    CONFIG.C_PROBE_OUT9_WIDTH  {1} \
    CONFIG.C_PROBE_OUT10_WIDTH {1} \
] [get_ips vio_ctrl]

# ---- Generate 50 MHz Core ILA IP ---------------------------------------------
create_ip -name ila -vendor xilinx.com -library ip -module_name ila_core50
set_property -dict [list \
    CONFIG.C_NUM_OF_PROBES   {7} \
    CONFIG.C_DATA_DEPTH      {2048} \
    CONFIG.C_PROBE0_WIDTH    {8} \
    CONFIG.C_PROBE1_WIDTH    {30} \
    CONFIG.C_PROBE2_WIDTH    {8} \
    CONFIG.C_PROBE3_WIDTH    {32} \
    CONFIG.C_PROBE4_WIDTH    {20} \
    CONFIG.C_PROBE5_WIDTH    {16} \
    CONFIG.C_PROBE6_WIDTH    {20} \
    CONFIG.C_ENABLE_ILA_AXI_MON {false} \
    CONFIG.C_MONITOR_TYPE    {Native} \
] [get_ips ila_core50]

# ---- Generate 25 MHz MII RX ILA IP -------------------------------------------
create_ip -name ila -vendor xilinx.com -library ip -module_name ila_mii_rx
set_property -dict [list \
    CONFIG.C_NUM_OF_PROBES   {2} \
    CONFIG.C_DATA_DEPTH      {2048} \
    CONFIG.C_PROBE0_WIDTH    {8} \
    CONFIG.C_PROBE1_WIDTH    {4} \
    CONFIG.C_ENABLE_ILA_AXI_MON {false} \
    CONFIG.C_MONITOR_TYPE    {Native} \
] [get_ips ila_mii_rx]

generate_target all [get_ips]
synth_ip [get_ips]

# ---- Synthesis & Implementation ----------------------------------------------
launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    error "Synthesis failed!"
}

launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
    error "Implementation / bitstream generation failed!"
}

open_run impl_1
report_utilization -file "$build_dir/utilization.rpt"
report_timing_summary -file "$build_dir/timing.rpt"

set wns [get_property STATS.WNS [get_runs impl_1]]
set whs [get_property STATS.WHS [get_runs impl_1]]
puts "=== BUILD COMPLETE: WNS=$wns ns, WHS=$whs ns ==="

file copy -force "$build_dir/${proj_name}.runs/impl_1/fpga_top.bit" "$build_dir/fpga_top.bit"
file copy -force "$build_dir/${proj_name}.runs/impl_1/fpga_top.ltx" "$build_dir/fpga_top.ltx"
exit
