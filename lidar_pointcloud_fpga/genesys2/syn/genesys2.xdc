# Digilent Genesys 2 (xc7k325tffg900-2) constraints
# Pin locations from Digilent's Genesys-2-Master.xdc

set_property CFGBVS VCCO                      [current_design]
set_property CONFIG_VOLTAGE 3.3               [current_design]
set_property BITSTREAM.GENERAL.COMPRESS true  [current_design]

# 200 MHz system clock
set_property -dict {LOC AD12 IOSTANDARD LVDS} [get_ports sysclk_p]
set_property -dict {LOC AD11 IOSTANDARD LVDS} [get_ports sysclk_n]
create_clock -period 5.000 -name sysclk [get_ports sysclk_p]

# Reset button (active low)
set_property -dict {LOC R19 IOSTANDARD LVCMOS33} [get_ports cpu_resetn]
set_false_path -from [get_ports cpu_resetn]
set_input_delay 0 [get_ports cpu_resetn]

# LEDs
set_property -dict {LOC T28 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[0]}]
set_property -dict {LOC V19 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[1]}]
set_property -dict {LOC U30 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[2]}]
set_property -dict {LOC U29 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[3]}]
set_property -dict {LOC V20 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[4]}]
set_property -dict {LOC V26 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[5]}]
set_property -dict {LOC W24 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[6]}]
set_property -dict {LOC W23 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[7]}]
set_false_path -to [get_ports {led[*]}]
set_output_delay 0 [get_ports {led[*]}]

# Ethernet: RTL8211E-VL, RGMII, bank 33 (1.5 V)
set_property -dict {LOC AG10 IOSTANDARD LVCMOS15} [get_ports eth_rxck]
set_property -dict {LOC AH11 IOSTANDARD LVCMOS15} [get_ports eth_rxctl]
set_property -dict {LOC AJ14 IOSTANDARD LVCMOS15} [get_ports {eth_rxd[0]}]
set_property -dict {LOC AH14 IOSTANDARD LVCMOS15} [get_ports {eth_rxd[1]}]
set_property -dict {LOC AK13 IOSTANDARD LVCMOS15} [get_ports {eth_rxd[2]}]
set_property -dict {LOC AJ13 IOSTANDARD LVCMOS15} [get_ports {eth_rxd[3]}]
set_property -dict {LOC AE10 IOSTANDARD LVCMOS15 SLEW FAST DRIVE 16} [get_ports eth_txck]
set_property -dict {LOC AK14 IOSTANDARD LVCMOS15 SLEW FAST DRIVE 16} [get_ports eth_txctl]
set_property -dict {LOC AJ12 IOSTANDARD LVCMOS15 SLEW FAST DRIVE 16} [get_ports {eth_txd[0]}]
set_property -dict {LOC AK11 IOSTANDARD LVCMOS15 SLEW FAST DRIVE 16} [get_ports {eth_txd[1]}]
set_property -dict {LOC AJ11 IOSTANDARD LVCMOS15 SLEW FAST DRIVE 16} [get_ports {eth_txd[2]}]
set_property -dict {LOC AK10 IOSTANDARD LVCMOS15 SLEW FAST DRIVE 16} [get_ports {eth_txd[3]}]
set_property -dict {LOC AH24 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports eth_phyrst_n]

create_clock -period 8.000 -name eth_rxck [get_ports eth_rxck]

set_false_path -to [get_ports eth_phyrst_n]
set_output_delay 0 [get_ports eth_phyrst_n]

# ---- RGMII timing tuning knobs ------------------------------------------------
# Same starting point as the taxi / verilog-ethernet designs for this PHY:
# no extra RX data delay (the PHY delays RXC itself) and the TX clock shifted
# 90 degrees in the FPGA.  If LED5 (bad FCS) lights or nothing is received, try
# IDELAY_VALUE in steps of ~8 (78 ps per tap); if the laptop sees no frames, try
# CLKOUT1_PHASE 0.
set_property IDELAY_VALUE 0 [get_cells {phy_rx_ctl_idelay phy_rxd_idelay_bit[*].idelay_inst}]
set_property CLKOUT1_PHASE 90 [get_cells clk_mmcm_inst]
