PYTHON ?= python3
IVERILOG ?= iverilog
YOSYS ?= yosys

.PHONY: test synth-check synth synth-sky130
test:
	$(PYTHON) tests/test_fp32_mac.py

synth-check:
	$(IVERILOG) -g2012 -Wall -S -tnull -s fp32_mac fp32_mac.sv

synth:
	$(YOSYS) -s tests/synth_fp32_mac.ys

synth-sky130:
	test -r "$(SKY130_LIBERTY)"
	$(YOSYS) -p 'script tests/synth_fp32_mac.ys; dfflibmap -liberty "$(SKY130_LIBERTY)"; abc -liberty "$(SKY130_LIBERTY)"; clean; stat -liberty "$(SKY130_LIBERTY)"'
