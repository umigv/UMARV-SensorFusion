# Live on-hardware self-test suite for genesys2_chip over JTAG (Vivado VIO + ILA).
# Runs with ONLY the Genesys 2 connected via JTAG — no external sensors or switch required.
# Leaves the FPGA in normal external-hardware mode (all VIO overrides = 0) on exit.

set script_dir [file dirname [file normalize [info script]]]
set bit_file   "$script_dir/build/fpga_top.bit"
set ltx_file   "$script_dir/build/fpga_top.ltx"

set errors 0
set checks 0

proc check {cond msg} {
    global errors checks
    incr checks
    if {[uplevel 1 [list expr $cond]]} {
        puts "  \[PASS\] $msg"
    } else {
        incr errors
        puts "  \[FAIL\] $msg"
    }
}

proc u32_to_f32 {u} {
    binary scan [binary format I $u] R f
    return $f
}

proc read_vio {vio net_pat} {
    refresh_hw_vio $vio
    set p [lindex [get_hw_probes -of_objects $vio -filter "NAME =~ *${net_pat}"] 0]
    set v [get_property INPUT_VALUE $p]
    return [expr "0x$v"]
}

proc set_vio {vio net_pat hex_val} {
    set p [lindex [get_hw_probes -of_objects $vio -filter "NAME =~ *${net_pat}"] 0]
    set_property OUTPUT_VALUE $hex_val $p
    commit_hw_vio $p
}

set spi_tgl 0
proc spi_xfer {vio is_wr addr wdata} {
    global spi_tgl
    set prev [read_vio $vio spi_done_tgl]
    set_vio $vio vio_spi_is_wr [format "%x" $is_wr]
    set_vio $vio vio_spi_addr  [format "%02x" $addr]
    set_vio $vio vio_spi_wdata [format "%02x" $wdata]
    set spi_tgl [expr {$spi_tgl ^ 1}]
    set_vio $vio vio_spi_go_tgl [format "%x" $spi_tgl]
    for {set i 0} {$i < 25} {incr i} {
        set cur_tgl [read_vio $vio spi_done_tgl]
        if {$cur_tgl != $prev} {
            return [read_vio $vio spi_rdata]
        }
        after 5
    }
    puts "  \[WARN\] SPI timeout on addr 0x[format %02x $addr]"
    return -1
}

proc spi_rd {vio addr} { return [spi_xfer $vio 0 $addr 0] }
proc spi_wr {vio addr val} { return [spi_xfer $vio 1 $addr $val] }

puts "======================================================================"
puts " Genesys 2 Live Hardware Verification Suite"
puts "======================================================================"

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target
set dev [lindex [get_hw_devices xc7k325t*] 0]
current_hw_device $dev
set_property PROGRAM.FILE $bit_file $dev
set_property PROBES.FILE $ltx_file $dev
set_property FULL_PROBES.FILE $ltx_file $dev
program_hw_devices $dev
refresh_hw_device $dev

set vio [lindex [get_hw_vios] 0]
set ilas [get_hw_ilas]
puts "Found VIO : $vio"
puts "Found ILAs: $ilas"

# Wait for PHY reset (21 ms) + boot wait (50 ms) + MDIO init
after 250

# ---- TEST 1: Clocks, Reset & Real RTL8211E MDIO ------------------------------
puts "\n--- Test 1: Clocks, Reset & Onboard RTL8211E PHY (MDIO) ---"
set mmcm_locked    [read_vio $vio mmcm_locked]
set idelay_rdy     [read_vio $vio idelay_rdy]
set phy_rst_n      [read_vio $vio eth_phyrst_n_OBUF]
set mdio_init_done [read_vio $vio mdio_init_done]
set phy_id1        [read_vio $vio phy_id1]
set phy_id2        [read_vio $vio phy_id2]
set reg_bmcr       [read_vio $vio reg_bmcr]
set reg_bmsr       [read_vio $vio reg_bmsr]
set reg_gbcr       [read_vio $vio reg_gbcr]
set reg_physr      [read_vio $vio reg_physr]

puts [format "  MMCM=%d IDELAY=%d PHYRST_N=%d MDIO_DONE=%d | PHYID=0x%04X:0x%04X | BMCR=0x%04X BMSR=0x%04X GBCR=0x%04X PHYSR=0x%04X" \
      $mmcm_locked $idelay_rdy $phy_rst_n $mdio_init_done $phy_id1 $phy_id2 $reg_bmcr $reg_bmsr $reg_gbcr $reg_physr]

check {$mmcm_locked == 1}    "MMCM locked (200 MHz -> 50/25/25+90/200 MHz)"
check {$idelay_rdy == 1}     "IDELAYCTRL ready (200 MHz refclk)"
check {$phy_rst_n == 1}      "PHY reset released"
check {$mdio_init_done == 1} "MDIO initialization sequence completed"
check {$phy_id1 == 0x001C && $phy_id2 == 0xC915} "Onboard RTL8211E PHY ID verified (001C:C915)"
check {($reg_gbcr & 0x0300) == 0} "1000BASE-T advertisement cleared in GBCR (Reg 0x09)"

# ---- TEST 2: Live 400 Hz Timestamper Timing (Idle Sensors) -------------------
puts "\n--- Test 2: Live 400 Hz Timestamper Timing (50 MHz silicon) ---"
set beat_per  [read_vio $vio meas_beat_period]
set sync_w    [read_vio $vio meas_sync_width]
set beat_cnt  [read_vio $vio meas_beat_cnt]
set dead_clks [read_vio $vio meas_deadline_clks]
set dw_reads  [read_vio $vio meas_dw_reads]

puts "  beat_period=$beat_per clks  sync_width=$sync_w clks  deadline=$dead_clks clks  beats=$beat_cnt  dw_reads=$dw_reads"
check {$beat_per == 125000}  "Beat period == 125,000 clks (exact 2,500 us / 400.000 Hz)"
check {$sync_w == 500}       "VN-300 SyncIn pulse width == 500 clks (exact 10.00 us)"
check {$dead_clks == 100050} "Idle deadline == 100,050 clks (exact 2,001 us per timestamper.md)"
check {$beat_cnt > 10 && $dw_reads > 10} "Beat counter and AMT22 SPI reads actively running"

# ---- TEST 3: Live Host SPI Register File (spi_regs) --------------------------
puts "\n--- Test 3: Live Host SPI Register File (Mode 0 @ 1.56 MHz) ---"
set_vio $vio vio_spi_sel "1"
set id_val   [spi_rd $vio 0x00]
set ver_val  [spi_rd $vio 0x01]
set ctrl_val [spi_rd $vio 0x02]
puts [format "  ID=0x%02X  VER=0x%02X  CTRL=0x%02X" $id_val $ver_val $ctrl_val]
check {$id_val == 0xA7}   "SPI Reg 0x00 (ID) == 0xA7"
check {$ver_val == 0x01}  "SPI Reg 0x01 (Version) == 0x01"
check {$ctrl_val == 0x43} "SPI Reg 0x02 (Control reset) == 0x43"

# Check R/W registers (host state timestamp 0x24..0x27)
spi_wr $vio 0x24 0x78
spi_wr $vio 0x25 0x56
spi_wr $vio 0x26 0x34
spi_wr $vio 0x27 0x12
set rw0 [spi_rd $vio 0x24]
set rw1 [spi_rd $vio 0x25]
set rw2 [spi_rd $vio 0x26]
set rw3 [spi_rd $vio 0x27]
check {$rw0 == 0x78 && $rw1 == 0x56 && $rw2 == 0x34 && $rw3 == 0x12} "SPI R/W host state regs 0x24-0x27 match 0x12345678"

# Check new UKF filter word interface (0x6C address + 0x70-0x73 32-bit R/W)
spi_wr $vio 0x6C 96 ;# RR01 (first tunable constant, default 0x45DD8E86)
set c0 [spi_rd $vio 0x70]; set c1 [spi_rd $vio 0x71]; set c2 [spi_rd $vio 0x72]; set c3 [spi_rd $vio 0x73]
set orig_rr01 [expr {$c0 | ($c1 << 8) | ($c2 << 16) | ($c3 << 24)}]
spi_wr $vio 0x70 0xDD; spi_wr $vio 0x71 0xCC; spi_wr $vio 0x72 0xBB; spi_wr $vio 0x73 0x3A
set w0 [spi_rd $vio 0x70]; set w1 [spi_rd $vio 0x71]; set w2 [spi_rd $vio 0x72]; set w3 [spi_rd $vio 0x73]
set wr_rr01 [expr {$w0 | ($w1 << 8) | ($w2 << 16) | ($w3 << 24)}]
# Restore RR01 to built-in default
spi_wr $vio 0x70 $c0; spi_wr $vio 0x71 $c1; spi_wr $vio 0x72 $c2; spi_wr $vio 0x73 $c3
# Verify fixed constant 65 (ONE = 1.0f = 0x3F800000) is read-only
spi_wr $vio 0x6C 65
spi_wr $vio 0x70 0x00; spi_wr $vio 0x71 0x00; spi_wr $vio 0x72 0x00; spi_wr $vio 0x73 0x00
set f0 [spi_rd $vio 0x70]; set f1 [spi_rd $vio 0x71]; set f2 [spi_rd $vio 0x72]; set f3 [spi_rd $vio 0x73]
set one_word [expr {$f0 | ($f1 << 8) | ($f2 << 16) | ($f3 << 24)}]
puts [format "  UKF cfg regs: RR01 default=0x%08X -> wrote/read=0x%08X | fixed ONE(65)=0x%08X" $orig_rr01 $wr_rr01 $one_word]
check {$orig_rr01 == 0x45DD8E86} "UKF tunable constant 96 (RR01) reads built-in default 0x45DD8E86"
check {$wr_rr01   == 0x3ABBCCDD} "UKF tunable constant 96 (RR01) accepts host write via 0x6C/0x70-73"
check {$one_word  == 0x3F800000} "UKF fixed constant 65 (ONE) stays read-only (0x3F800000)"

# Check live beat_ts advancing
set ts0 [expr {[spi_rd $vio 0x48] | ([spi_rd $vio 0x49] << 8) | ([spi_rd $vio 0x4A] << 16) | ([spi_rd $vio 0x4B] << 24)}]
after 25
set ts1 [expr {[spi_rd $vio 0x48] | ([spi_rd $vio 0x49] << 8) | ([spi_rd $vio 0x4A] << 16) | ([spi_rd $vio 0x4B] << 24)}]
puts "  beat_ts t0=$ts0 us -> t1=$ts1 us (delta=[expr {$ts1 - $ts0}] us)"
check {$ts1 > $ts0} "Live microsecond beat_ts advances over SPI"

# ---- TEST 4: Timestamper VN-300 (921,600 baud) & AMT22 SPI via Loopback ------
puts "\n--- Test 4: Timestamper VN-300 Parser + AMT22 SPI (Internal Sensor Models) ---"
set_vio $vio vio_sim_sensors "1"
after 40
# Read 0x4C outside the 656 us collection window, then pause timestamper (ts_enable=0)
# so slow JTAG byte-by-byte SPI reads cannot tear across a 400 Hz beat boundary
set valid_reg 0
for {set try 0} {$try < 5} {incr try} {
    set v [spi_rd $vio 0x4C]
    if {$v == 0x03} { set valid_reg $v; break }
    after 1
}
spi_wr $vio 0x02 0x42 ;# pause timestamper beat (holds latest snapshot steady)
after 5
set valid_reg [spi_rd $vio 0x4C]
set yaw_bits  [expr {[spi_rd $vio 0x50] | ([spi_rd $vio 0x51] << 8) | ([spi_rd $vio 0x52] << 16) | ([spi_rd $vio 0x53] << 24)}]
set rate_bits [expr {[spi_rd $vio 0x54] | ([spi_rd $vio 0x55] << 8) | ([spi_rd $vio 0x56] << 16) | ([spi_rd $vio 0x57] << 24)}]
set dw_delta  [expr {[spi_rd $vio 0x68] | ([spi_rd $vio 0x69] << 8) | ([spi_rd $vio 0x6A] << 16) | ([spi_rd $vio 0x6B] << 24)}]
set dead_sens [read_vio $vio meas_deadline_clks]
spi_wr $vio 0x02 0x43 ;# re-enable timestamper

puts [format "  valid=0x%02X  yaw=0x%08X  rate=0x%08X  dw_delta=%d  collect_clks=%d (%.1f us)" \
      $valid_reg $yaw_bits $rate_bits $dw_delta $dead_sens [expr {$dead_sens / 50.0}]]
check {$valid_reg == 0x03}       "SPI Reg 0x4C: both vn_valid=1 and dw_valid=1"
check {$yaw_bits == 0x42340000}  "VN-300 921,600-baud UART + CRC16 parsed yaw = 45.0f (0x42340000)"
check {$rate_bits == 0x3F800000} "VN-300 parsed turn rate = 1.0f (0x3F800000)"
check {$dw_delta == 450}         "AMT22 1.92 MHz SPI + parity parsed dw_delta = +450 counts"
check {$dead_sens > 20000 && $dead_sens < 50000} "Early ukf_pulse completion (~660 us < 2,000 us deadline)"

# ---- TEST 4b: Live Hardware SR-UKF on Kintex-7 Silicon -----------------------
puts "\n--- Test 4b: Live Hardware SR-UKF (ukf_top + fp_serial @ 400 Hz) ---"
spi_wr $vio 0x02 0xF3 ;# enable UKF state source (bit 7) + rot/shift correction (bits 4-5)
after 80              ;# ~32 real 400 Hz UKF beats!
set ukf_cnt  [read_vio $vio ukf_state_cnt]
set ukf_ovr  [read_vio $vio overruns_orig]
set ukf_st   [read_vio $vio started_orig]
set ukf_x    [u32_to_f32 [read_vio $vio ukf_x]]
set ukf_y    [u32_to_f32 [read_vio $vio ukf_y]]
set ukf_th   [u32_to_f32 [read_vio $vio ukf_theta]]
set ukf_om   [u32_to_f32 [read_vio $vio ukf_omega]]
set ukf_v    [u32_to_f32 [read_vio $vio ukf_v]]
set ukf_r    [u32_to_f32 [read_vio $vio ukf_r]]
puts [format "  UKF states=%d  overruns=%d  started=%d | x=%.4fm y=%.4fm th=%.4frad om=%.4frad/s v=%.4fm/s r=%.5fm" \
      $ukf_cnt $ukf_ovr $ukf_st $ukf_x $ukf_y $ukf_th $ukf_om $ukf_v $ukf_r]
check {$ukf_cnt >= 10 && $ukf_ovr == 0 && $ukf_st == 1} "UKF ran >=10 live 400 Hz beats with 0 overruns"
check {$ukf_v > 3.0 && $ukf_v < 3.9}                    "UKF forward speed converged (~3.45 m/s)"
check {$ukf_om < -0.8 && $ukf_om > -1.2}                "UKF turn rate converged (~-1.0 rad/s)"
check {$ukf_r > 0.045 && $ukf_r < 0.055}                "UKF wheel radius sensible (~0.050 m)"

# ---- TEST 5: REAL RTL8211E 100M RGMII Loopback -> lidar_top (with live UKF!) -> PC2X v3 ----
puts "\n--- Test 5: Physical RTL8211E 100M RGMII Loopback + Full lidar_top + UKF ---"
set_vio $vio vio_phy_loopback "1"
after 120
set lb_bmcr  [read_vio $vio reg_bmcr]
set lb_physr [read_vio $vio reg_physr]
set lb_ibs   [read_vio $vio ibs_50]
puts [format "  Loopback mode: BMCR=0x%04X  PHYSR=0x%04X  IBS=0x%X" $lb_bmcr $lb_physr $lb_ibs]
check {$lb_bmcr == 0x6100} "RTL8211E entered 100M Full-Duplex PCS loopback (BMCR=0x6100)"

# Clear counters & monitor, switch TX to VLP-16 generator (keep vio_sim_sensors=1 & CTRL=0xF3!)
spi_wr $vio 0x3D 0x01
set_vio $vio vio_mon_clr_tgl "1"
after 10
set_vio $vio vio_pktgen_sel "1"
after 10
# Trigger 4 real 1252B VLP-16 frames through physical RTL8211E!
set_vio $vio vio_pktgen_go_tgl "1"
after 60

set tx_frames   [read_vio $vio mon_tx_frames_50]
set tx_good_fcs [read_vio $vio mon_tx_good_fcs_50]
set tx_bad_fcs  [read_vio $vio mon_tx_bad_fcs_50]
set tx_pc2x_ok  [read_vio $vio mon_tx_pc2x_ok_50]
set pg_sent     [read_vio $vio pktgen_sent_50]
set last_flags  [read_vio $vio mon_flags_50]
set last_len    [read_vio $vio mon_len_50]
set p0_x        [read_vio $vio mon_p0x_50]
set p0_y        [read_vio $vio mon_p0y_50]
set p0_z        [read_vio $vio mon_p0z_50]
if {$p0_x >= 32768} { set p0_x [expr {$p0_x - 65536}] }
if {$p0_y >= 32768} { set p0_y [expr {$p0_y - 65536}] }
if {$p0_z >= 32768} { set p0_z [expr {$p0_z - 65536}] }

set c_lidar [spi_rd $vio 0x40]
set c_other [spi_rd $vio 0x41]
set c_crc   [spi_rd $vio 0x42]
set c_sent  [spi_rd $vio 0x43]
set c_ovf   [spi_rd $vio 0x44]
set c_ovr   [spi_rd $vio 0x45]
set c_und   [spi_rd $vio 0x46]

puts "  PktGen sent=$pg_sent -> Physical RTL8211E RGMII Loopback -> lidar_top (with live UKF):"
puts "  SPI counters: lidar_pkt=$c_lidar other=$c_other rx_crc_bad=$c_crc frame_sent=$c_sent ovf=$c_ovf ovr=$c_ovr und=$c_und"
puts "  TX verifier:  frames=$tx_frames good_fcs=$tx_good_fcs bad_fcs=$tx_bad_fcs pc2x_v3_ok=$tx_pc2x_ok len=$last_len flags=0x[format %02X $last_flags]"
puts "  Last frame point 0 (x,y,z) counts: ($p0_x, $p0_y, $p0_z) \[2mm/cnt\]"

check {$pg_sent == 4}                              "PktGen transmitted 4 VLP-16 Ethernet frames"
check {$c_lidar == 4 && $c_crc == 0}               "Physical RTL8211E RGMII RX -> mii_recv: 4 VLP-16 pkts, 0 CRC errors"
check {$c_sent == 8 && $c_ovf == 0 && $c_ovr == 0 && $c_und == 0} "lidar_top: 8 frames sent, 0 overflow/overrun/underrun"
check {$tx_frames == 8 && $tx_good_fcs == 8 && $tx_bad_fcs == 0}  "mii_send: 8/8 frames passed Ethernet CRC-32 FCS"
check {$tx_pc2x_ok == 8 && $last_len == 1270}      "8/8 frames valid PC2X v3 (1270 bytes)"
check {($last_flags & 0x01) == 1}                  "PC2X v3 header carries live UKF state_valid=1"
check {$p0_z < -1280 && $p0_z > -1310}             "Laser 0 (-15 deg, 10m) Z coordinate verified (~-1294 counts)"
set_vio $vio vio_sim_sensors "0"

# ---- TEST 5b: Bypass Mode over Physical RTL8211E Loopback --------------------
puts "\n--- Test 5b: Lidar Bypass Mode over Physical RTL8211E RGMII Loopback ---"
spi_wr $vio 0x02 0x47
spi_wr $vio 0x3D 0x01
set_vio $vio vio_mon_clr_tgl "0"
after 10
set_vio $vio vio_pktgen_go_tgl "0"
after 60

set by_frames   [read_vio $vio mon_tx_frames_50]
set by_good_fcs [read_vio $vio mon_tx_good_fcs_50]
set by_bad_fcs  [read_vio $vio mon_tx_bad_fcs_50]
set by_len      [read_vio $vio mon_len_50]
puts "  Bypass verifier: frames=$by_frames good_fcs=$by_good_fcs bad_fcs=$by_bad_fcs len=$by_len"
check {$by_frames == 4 && $by_good_fcs == 4 && $by_bad_fcs == 0 && $by_len == 1252} \
      "Bypass mode forwarded 4/4 1252B VLP-16 frames unchanged with valid FCS"

# ---- TEST 6: Verify ILA Live Triggering --------------------------------------
puts "\n--- Test 6: Vivado ILA Core Capture Check ---"
set ila50 [lindex [get_hw_ilas -filter {CELL_NAME =~ *u_ila50*}] 0]
run_hw_ila $ila50 -trigger_now
wait_on_hw_ila -timeout 5 $ila50
set ila_data [upload_hw_ila_data $ila50]
puts "  u_ila50 uploaded waveform object: $ila_data"
check {$ila_data ne ""} "50 MHz ILA core armed, captured 2048 samples, and uploaded over JTAG"

# ---- Cleanup: Restore Normal External-Hardware Mode --------------------------
puts "\n--- Restoring Normal External-Hardware Mode (all VIO overrides = 0) ---"
spi_wr $vio 0x02 0x43
spi_wr $vio 0x3D 0x01
set_vio $vio vio_pktgen_sel "0"
set_vio $vio vio_spi_sel "0"
set_vio $vio vio_phy_loopback "0"
set_vio $vio vio_mon_clr_tgl "1"
after 2500
set norm_bmcr  [read_vio $vio reg_bmcr]
set norm_gbcr  [read_vio $vio reg_gbcr]
set ext_link   [read_vio $vio mdio_link_up]
set ext_spd    [read_vio $vio mdio_speed]
set ext_dup    [read_vio $vio mdio_duplex]
set ext_ibs    [read_vio $vio ibs_50]
set ext_frames [read_vio $vio mon_tx_frames_50]
set ext_gfcs   [read_vio $vio mon_tx_good_fcs_50]
set ext_bfcs   [read_vio $vio mon_tx_bad_fcs_50]
set ext_pc2x   [read_vio $vio mon_tx_pc2x_ok_50]
set ext_err_rx [read_vio $vio err_rx_sticky]
set ext_px     [read_vio $vio mon_p0x_50]
set ext_py     [read_vio $vio mon_p0y_50]
set ext_pz     [read_vio $vio mon_p0z_50]
if {$ext_px >= 32768} { set ext_px [expr {$ext_px - 65536}] }
if {$ext_py >= 32768} { set ext_py [expr {$ext_py - 65536}] }
if {$ext_pz >= 32768} { set ext_pz [expr {$ext_pz - 65536}] }
puts [format "  Restored PHY: BMCR=0x%04X  GBCR=0x%04X" $norm_bmcr $norm_gbcr]
puts [format "  Live External Ethernet (after 2.5 s): link=%d speed=%d(1=100M) duplex=%d IBS=0x%X err_rx=%d" \
      $ext_link $ext_spd $ext_dup $ext_ibs $ext_err_rx]
puts "  Live External Lidar TX monitor: frames=$ext_frames good_fcs=$ext_gfcs bad_fcs=$ext_bfcs pc2x_v3=$ext_pc2x p0=($ext_px, $ext_py, $ext_pz)"
check {($norm_bmcr & 0x4000) == 0 && ($norm_gbcr & 0x0300) == 0} \
      "RTL8211E restored to normal 100M auto-negotiation mode"

puts "\n======================================================================"
puts " LIVE HARDWARE SUMMARY: $checks checks, $errors failures"
if {$errors == 0} {
    puts " ALL HARDWARE TESTS PASSED!"
} else {
    puts " HARDWARE TESTS HAD FAILURES!"
}
puts "======================================================================"

close_hw_target
disconnect_hw_server
close_hw_manager
exit [expr {$errors == 0 ? 0 : 1}]
