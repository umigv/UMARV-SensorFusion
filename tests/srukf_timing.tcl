# Provide LIBERTY and MAPPED_VERILOG in the environment.
read_liberty $::env(LIBERTY)
read_verilog $::env(MAPPED_VERILOG)
link_design UKF
create_clock -name core -period 20 [get_ports clk]
set_false_path -from [get_ports rst_n]
set_input_delay -clock core 0 [get_ports {ukf_beat ukf_pulse ukf_beat_ts* ukf_vn_valid ukf_vn_yaw_f32* ukf_vn_rate_f32* ukf_vn_lat_f64* ukf_vn_lon_f64* ukf_dw_valid ukf_dw_delta* measurement_s* prediction_done prediction_result* prediction_mac_request* prediction_div_request*}]
set_output_delay -clock core 0 [all_outputs]
report_checks -path_delay max -group_count 5 -digits 4
report_worst_slack -digits 4
report_tns -digits 4
exit
