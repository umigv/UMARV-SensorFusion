create_clock -name mac_clk -period 20.0 [get_ports clk]
set_input_delay -clock mac_clk 2.0 [get_ports {start clear last a* b*}]
set_output_delay -clock mac_clk 2.0 [all_outputs]
set_false_path -from [get_ports rst_n]
