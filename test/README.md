# Tiny Tapeout simulation

Run from the repository root for a standalone Icarus test:

```sh
iverilog -g2005 -s tb -o /tmp/srukf-tt.vvp src/tt_um_umarv_srukf.v test/tb.v
vvp /tmp/srukf-tt.vvp
```

For the template's Cocotb flow, install `test/requirements.txt` and run `make -C test`. The same pin-level Verilog fixture runs in gate mode. The Tiny Tapeout GDS action supplies `test/gate_level_netlist.v` and the PDK. To run it locally with those artifacts, use `make -C test GATES=yes PDK_ROOT=/path/to/pdk`.
