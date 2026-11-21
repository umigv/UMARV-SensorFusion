# Vivado build for the Genesys 2.   make  /  make program  /  make clean
# Uses the Vivado makefile shipped with taxi (handles the .f file lists).

FPGA_PART = xc7k325tffg900-2
FPGA_TOP = fpga
FPGA_ARCH = kintex7

RTL_DIR = ../rtl
ISP_DIR = ../../verilog
TAXI_SRC_DIR = ../../lib/taxi/src
PARSER_DIR = ../../../lidar_fpga

SYN_FILES  = $(RTL_DIR)/fpga.sv
SYN_FILES += $(RTL_DIR)/fpga_core.sv
SYN_FILES += $(RTL_DIR)/lidar_pc_core.sv
SYN_FILES += $(RTL_DIR)/vlp16_udp_rx.sv
SYN_FILES += $(PARSER_DIR)/v16PacketParser.v
SYN_FILES += $(RTL_DIR)/pc_fix2float.sv
SYN_FILES += $(RTL_DIR)/pc_point_fifo.sv
SYN_FILES += $(RTL_DIR)/pc_udp_tx.sv
SYN_FILES += $(ISP_DIR)/stage_t.sv
SYN_FILES += $(ISP_DIR)/trig_lut.sv
SYN_FILES += $(ISP_DIR)/sys_defs.svh
SYN_FILES += $(ISP_DIR)/azimuth_sincos.mem
SYN_FILES += $(TAXI_SRC_DIR)/eth/rtl/taxi_eth_mac_1g_rgmii_fifo.f
SYN_FILES += $(TAXI_SRC_DIR)/sync/rtl/taxi_sync_reset.sv
SYN_FILES += $(TAXI_SRC_DIR)/sync/rtl/taxi_sync_signal.sv

XDC_FILES  = genesys2.xdc
XDC_FILES += $(TAXI_SRC_DIR)/eth/syn/vivado/taxi_rgmii_phy_if.tcl
XDC_FILES += $(TAXI_SRC_DIR)/eth/syn/vivado/taxi_eth_mac_fifo.tcl
XDC_FILES += $(TAXI_SRC_DIR)/axis/syn/vivado/taxi_axis_async_fifo.tcl
XDC_FILES += $(TAXI_SRC_DIR)/sync/syn/vivado/taxi_sync_reset.tcl
XDC_FILES += $(TAXI_SRC_DIR)/sync/syn/vivado/taxi_sync_signal.tcl

CONFIG_TCL_FILES = config.tcl

include $(TAXI_SRC_DIR)/eth/example/KC705/fpga/common/vivado.mk

program: $(PROJECT).bit
	echo "open_hw_manager" > program.tcl
	echo "connect_hw_server" >> program.tcl
	echo "open_hw_target" >> program.tcl
	echo "current_hw_device [lindex [get_hw_devices] 0]" >> program.tcl
	echo "refresh_hw_device -update_hw_probes false [current_hw_device]" >> program.tcl
	echo "set_property PROGRAM.FILE {$(PROJECT).bit} [current_hw_device]" >> program.tcl
	echo "program_hw_devices [current_hw_device]" >> program.tcl
	echo "exit" >> program.tcl
	vivado -nojournal -nolog -mode batch -source program.tcl
