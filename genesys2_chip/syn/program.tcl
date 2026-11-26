set script_dir [file dirname [file normalize [info script]]]
set bit_file   "$script_dir/build/fpga_top.bit"
set ltx_file   "$script_dir/build/fpga_top.ltx"

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target
set dev [lindex [get_hw_devices xc7k325t*] 0]
current_hw_device $dev
set_property PROGRAM.FILE $bit_file $dev
if {[file exists $ltx_file]} {
    set_property PROBES.FILE $ltx_file $dev
    set_property FULL_PROBES.FILE $ltx_file $dev
}
program_hw_devices $dev
refresh_hw_device $dev
puts "=== PROGRAMMED $dev SUCCESSFULLY ==="
close_hw_target
disconnect_hw_server
close_hw_manager
exit
