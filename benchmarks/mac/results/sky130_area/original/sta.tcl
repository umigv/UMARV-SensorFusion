read_liberty {REQUIRED_SKY130_HD_LIBERTY.lib}
read_verilog {/home/Nabhah/sqrtUKF/UMARV-SensorFusion/benchmarks/mac/results/sky130_area/original/mapped.v}
link_design mac_physical_top
set_cmd_units -time ns
read_sdc {/home/Nabhah/sqrtUKF/UMARV-SensorFusion/benchmarks/mac/results/sky130_area/mac.sdc}
report_checks -path_delay max -group_path_count 5 -format full_clock_expanded -digits 4
report_worst_slack -max
report_wns
report_clock_min_period -include_port_paths
exit
