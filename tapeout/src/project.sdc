# Timing constraints for the sensor-fusion chip: the flow's standard constraints, plus three
# two-clock paths. Each is a place where the logic waits a clock (or more) before using a value,
# and each has a matching check in simulation that makes the value unknown during the wait.
source $::env(SCRIPTS_DIR)/base.sdc

proc two_clock_from { what patterns } {
    set nets {}
    foreach p $patterns { set nets [concat $nets [get_nets -quiet $p]] }
    puts "\[INFO\] two-clock path, $what: [llength $nets] nets"
    if { [llength $nets] > 0 } {
        set_multicycle_path -setup 2 -through $nets
        set_multicycle_path -hold 1 -through $nets
    }
}

# 1. The Kalman filter's sequencer waits one clock after its program counter changes before it
#    uses anything derived from the step table (ukf_top.sv, word_ok).
two_clock_from "filter program counter" [list {u_ukf.pc[*]} {u_ukf\.pc\[*\]}]

# 2. The SPI block registers the address of the next byte to send and takes the byte two clocks
#    later (spi_regs.sv, rd_addr_q and fetch).
two_clock_from "SPI read address" [list {u_regs.rd_addr_q[*]} {u_regs\.rd_addr_q\[*\]}]

# 3. The lidar point buffer's read pointer moves at least nine clocks before the distance it
#    selects is taken (lidar_point_buffer.sv and lidar_xyz.sv). Only the way through the
#    256-word selector is relaxed: the pointer's other use (how full the buffer is) is not.
set rp [concat [get_nets -quiet {u_lidar.u_buffer.rd_ptr[*]}] [get_nets -quiet {u_lidar\.u_buffer\.rd_ptr\[*\]}]]
set rd [concat [get_nets -quiet {u_lidar.buf_rd_data[*]}] [get_nets -quiet {u_lidar\.buf_rd_data\[*\]}] \
               [get_nets -quiet {u_lidar.u_buffer.rd_data[*]}] [get_nets -quiet {u_lidar.u_buffer.mem_rdata[*]}]]
puts "\[INFO\] two-clock path, lidar buffer read: [llength $rp] pointer nets, [llength $rd] data nets"
if { [llength $rp] > 0 && [llength $rd] > 0 } {
    set_multicycle_path -setup 2 -through $rp -through $rd
    set_multicycle_path -hold 1 -through $rp -through $rd
}
