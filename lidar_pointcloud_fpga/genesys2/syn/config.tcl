# Build-time configuration (top-level generics of fpga.sv).
# Uncomment and edit to override the defaults.

set params [dict create]

# UDP port the lidar sends to; 0 accepts any port with a VLP-16 sized payload
#dict set params LIDAR_PORT "16'd2368"

# Where the point cloud goes.  Default is Ethernet + IP broadcast, which needs
# no ARP and reaches the laptop whatever its address is.
#dict set params DEST_MAC  "48'h001122334455"
#dict set params DEST_IP   "32'hC0A80164"   ;# 192.168.1.100
#dict set params DEST_PORT "16'd2370"

# This board
#dict set params LOCAL_MAC "48'h020000000001"
#dict set params LOCAL_IP  "32'hC0A80132"   ;# 192.168.1.50

set param_list {}
dict for {name value} $params {
    lappend param_list $name=$value
}
set_property generic $param_list [get_filesets sources_1]

# sys_defs.svh is an include file; the .mem file initialises the trig ROM
set_property file_type {Verilog Header} [get_files -quiet sys_defs.svh]
set_property include_dirs [file normalize ../../verilog] [get_filesets sources_1]
