VIVADO ?= /home/bany/2026.1/Vivado/bin/vivado

.PHONY: all build program selftest clean

all: build

build:
	$(VIVADO) -nojournal -nolog -mode batch -source build.tcl

program: build/fpga_top.bit
	$(VIVADO) -nojournal -nolog -mode batch -source program.tcl

selftest: build/fpga_top.bit
	$(VIVADO) -nojournal -nolog -mode batch -source run_hw_selftest.tcl

clean:
	rm -rf build
