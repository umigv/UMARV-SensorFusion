# Digilent Genesys 2 (xc7k325tffg900-2) constraints for the sensor-fusion chip port.
# Pin locations verified against Digilent's Genesys-2-Master.xdc and lidar_pointcloud_fpga/genesys2/syn/genesys2.xdc.

set_property CFGBVS VCCO                      [current_design]
set_property CONFIG_VOLTAGE 3.3               [current_design]
set_property BITSTREAM.GENERAL.COMPRESS true  [current_design]

# ---- 200 MHz LVDS system clock -----------------------------------------------
set_property -dict {PACKAGE_PIN AD12 IOSTANDARD LVDS} [get_ports sysclk_p]
set_property -dict {PACKAGE_PIN AD11 IOSTANDARD LVDS} [get_ports sysclk_n]
create_clock -period 5.000 -name sysclk [get_ports sysclk_p]

# ---- Reset button (active low, Bank 14 3.3 V) --------------------------------
set_property -dict {PACKAGE_PIN R19 IOSTANDARD LVCMOS33} [get_ports cpu_resetn]
set_false_path -from [get_ports cpu_resetn]

# ---- User LEDs (3.3 V) -------------------------------------------------------
set_property -dict {PACKAGE_PIN T28 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN V19 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN U30 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN U29 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[3]}]
set_property -dict {PACKAGE_PIN V20 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[4]}]
set_property -dict {PACKAGE_PIN V26 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[5]}]
set_property -dict {PACKAGE_PIN W24 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[6]}]
set_property -dict {PACKAGE_PIN W23 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports {led[7]}]
set_false_path -to [get_ports {led[*]}]

# ---- Onboard Ethernet: RTL8211E-VL RGMII + MDIO (Bank 33 1.5 V) --------------
set_property -dict {PACKAGE_PIN AG10 IOSTANDARD LVCMOS15} [get_ports eth_rxck]
set_property -dict {PACKAGE_PIN AH11 IOSTANDARD LVCMOS15} [get_ports eth_rxctl]
set_property -dict {PACKAGE_PIN AJ14 IOSTANDARD LVCMOS15} [get_ports {eth_rxd[0]}]
set_property -dict {PACKAGE_PIN AH14 IOSTANDARD LVCMOS15} [get_ports {eth_rxd[1]}]
set_property -dict {PACKAGE_PIN AK13 IOSTANDARD LVCMOS15} [get_ports {eth_rxd[2]}]
set_property -dict {PACKAGE_PIN AJ13 IOSTANDARD LVCMOS15} [get_ports {eth_rxd[3]}]
set_property -dict {PACKAGE_PIN AE10 IOSTANDARD LVCMOS15 SLEW FAST DRIVE 16} [get_ports eth_txck]
set_property -dict {PACKAGE_PIN AK14 IOSTANDARD LVCMOS15 SLEW FAST DRIVE 16} [get_ports eth_txctl]
set_property -dict {PACKAGE_PIN AJ12 IOSTANDARD LVCMOS15 SLEW FAST DRIVE 16} [get_ports {eth_txd[0]}]
set_property -dict {PACKAGE_PIN AK11 IOSTANDARD LVCMOS15 SLEW FAST DRIVE 16} [get_ports {eth_txd[1]}]
set_property -dict {PACKAGE_PIN AJ11 IOSTANDARD LVCMOS15 SLEW FAST DRIVE 16} [get_ports {eth_txd[2]}]
set_property -dict {PACKAGE_PIN AK10 IOSTANDARD LVCMOS15 SLEW FAST DRIVE 16} [get_ports {eth_txd[3]}]
set_property -dict {PACKAGE_PIN AF12 IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 8}  [get_ports eth_mdc]
set_property -dict {PACKAGE_PIN AG12 IOSTANDARD LVCMOS15 SLEW SLOW DRIVE 8}  [get_ports eth_mdio]
set_property -dict {PACKAGE_PIN AH24 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports eth_phyrst_n]

# RGMII RX clock: 25 MHz at 100BASE-TX (constrain conservatively at 40 ns;
# also handles transient 125 MHz prior to MDIO 100M configuration)
create_clock -period 40.000 -name eth_rxck [get_ports eth_rxck]

# RX data delay (PHY has RXDLY=1 -> +2 ns internal skew; 0 taps starting point)
set_property IDELAY_VALUE 0 [get_cells {gen_xilinx_idelay.phy_rx_ctl_idelay gen_xilinx_idelay.phy_rxd_idelay_bit[*].idelay_inst}]

set_false_path -to   [get_ports eth_phyrst_n]
set_false_path -to   [get_ports eth_mdc]
set_false_path -to   [get_ports eth_mdio]
set_false_path -from [get_ports eth_mdio]

# ---- Pmod JC (Bank 13, 3.3 V): Host ESP32 SPI + meas_ready -------------------
set_property -dict {PACKAGE_PIN AC26 IOSTANDARD LVCMOS33 PULLDOWN true} [get_ports jc_sck]        ;# JC1
set_property -dict {PACKAGE_PIN AJ27 IOSTANDARD LVCMOS33 PULLUP   true} [get_ports jc_cs_n]       ;# JC2 (idle high)
set_property -dict {PACKAGE_PIN AH30 IOSTANDARD LVCMOS33 PULLDOWN true} [get_ports jc_mosi]       ;# JC3
set_property -dict {PACKAGE_PIN AK29 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports jc_miso]       ;# JC4
set_property -dict {PACKAGE_PIN AD26 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports jc_meas_ready] ;# JC7

set_false_path -from [get_ports {jc_sck jc_cs_n jc_mosi}]
set_false_path -to   [get_ports {jc_miso jc_meas_ready}]

# ---- Pmod JD (3.3 V): VN-300 TTL Serial/SyncIn + AMT22 SPI -------------------
set_property -dict {PACKAGE_PIN V27 IOSTANDARD LVCMOS33 PULLUP   true} [get_ports jd_vn_rx]    ;# JD1 (UART idle high)
set_property -dict {PACKAGE_PIN Y30 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports jd_vn_sync]  ;# JD2
set_property -dict {PACKAGE_PIN V24 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports jd_dw_cs_n]  ;# JD3
set_property -dict {PACKAGE_PIN W22 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports jd_dw_sck]   ;# JD4
set_property -dict {PACKAGE_PIN U24 IOSTANDARD LVCMOS33 SLEW SLOW DRIVE 12} [get_ports jd_dw_mosi]  ;# JD7
set_property -dict {PACKAGE_PIN Y26 IOSTANDARD LVCMOS33 PULLDOWN true} [get_ports jd_dw_miso]  ;# JD8

set_false_path -from [get_ports {jd_vn_rx jd_dw_miso}]
set_false_path -to   [get_ports {jd_vn_sync jd_dw_cs_n jd_dw_sck jd_dw_mosi}]

# ---- Asynchronous clock domain crossings -------------------------------------
# The 50 MHz core clock, 25 MHz TX clock, and PHY-recovered 25 MHz eth_rxck are
# joined exclusively through Gray-coded async_fifos and 2FF synchronizers.
set_clock_groups -asynchronous \
    -group [get_clocks -include_generated_clocks sysclk] \
    -group [get_clocks eth_rxck]

# 50 MHz core and 25 MHz TX clocks are MMCM-derived (phase-aligned), but cross
# via Gray-coded async_fifo and 2FF synchronizers; keep async_fifo CDC clean.
set clk50_obj [get_clocks -of_objects [get_pins gen_xilinx_clk.u_mmcm/CLKOUT0]]
set clk25_obj [get_clocks -of_objects [get_pins gen_xilinx_clk.u_mmcm/CLKOUT1]]
set_max_delay -datapath_only -from $clk50_obj -to $clk25_obj 10.000
set_max_delay -datapath_only -from $clk25_obj -to $clk50_obj 10.000
