SIM ?= icarus
TOPLEVEL_LANG = verilog
TOPLEVEL = tb
COCOTB_TEST_MODULES = test
COMPILE_ARGS += -DCOCOTB_TEST
ifeq ($(GATES),yes)
SIM_BUILD = sim_build/gl
COMPILE_ARGS += -DGL_TEST -DFUNCTIONAL -DUSE_POWER_PINS -DSIM -DUNIT_DELAY=\#1
PDK_LIB = $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog
VERILOG_SOURCES += $(PDK_LIB)/primitives.v $(PDK_LIB)/sky130_fd_sc_hd.v $(CURDIR)/gate_level_netlist.v
else
SIM_BUILD = sim_build/rtl
VERILOG_SOURCES += $(CURDIR)/../src/tt_um_umarv_srukf.v
endif
VERILOG_SOURCES += $(CURDIR)/tb.v
include $(shell cocotb-config --makefiles)/Makefile.sim
