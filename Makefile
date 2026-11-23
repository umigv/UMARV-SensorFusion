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

.PHONY: test-divsqrt synth-check-divsqrt synth-divsqrt synth-divsqrt-sky130
test: test-divsqrt
test-divsqrt:
	CC="$(CC)" IVERILOG="$(IVERILOG)" $(PYTHON) tests/test_fp32_divsqrt.py

synth-check: synth-check-divsqrt
synth-check-divsqrt:
	$(IVERILOG) -g2012 -Wall -S -tnull -s fp32_divsqrt fp32_divsqrt.sv

synth-divsqrt:
	$(YOSYS) -s tests/synth_fp32_divsqrt.ys

synth-divsqrt-sky130:
	test -r "$(SKY130_LIBERTY)"
	$(YOSYS) -p 'script tests/synth_fp32_divsqrt.ys; dfflibmap -liberty "$(SKY130_LIBERTY)"; abc -liberty "$(SKY130_LIBERTY)"; clean; stat -liberty "$(SKY130_LIBERTY)"'
