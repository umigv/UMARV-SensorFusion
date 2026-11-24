read_liberty {/home/Nabhah/sky130_lib/sky130_fd_sc_hd__tt_025C_1v80.lib}
read_verilog {/home/Nabhah/sqrtUKF/UMARV-SensorFusion/benchmarks/mac/results/sky130_area_mapped/booth_fast_shifts/mapped.v}
link_design mac_physical_top
set_cmd_units -time ns
read_sdc {/home/Nabhah/sqrtUKF/UMARV-SensorFusion/benchmarks/mac/results/sky130_area_mapped/mac.sdc}
report_checks -path_delay max -group_path_count 5 -format full_clock_expanded -digits 4
report_worst_slack -max
report_wns
report_clock_min_period -include_port_paths
exit
