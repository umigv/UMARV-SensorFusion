# UMARV-SensorFusion
info/planning/work:
https://docs.google.com/document/d/1aGUzmQvTQ0tSf5vLGLYOxvFpVVRiFReiAoUfX6Jszvw/edit?tab=t.t4ux1nkxj3xk

## Tiny Tapeout SKY130 project

The template top is `tt_um_umarv_srukf`. Its generated Verilog is in `src/` and uses the standard Tiny Tapeout pins. The byte interface carries wheel counts, VN300 coordinates and noise, and the corrected state and covariance factor. See [the pin protocol](docs/info.md) and [simulation instructions](test/README.md).

The editable wrapper is `Dev/tt_um_umarv_srukf.v`. After changing it or the SystemVerilog core, regenerate the submission source with Yosys 0.52 using `python3 -B tests/export_tt_verilog.py`. Use `info.yaml` for submission metadata. The GDS workflow builds the SKY130 design and runs the Tiny Tapeout precheck and gate-level test. The 8×2 footprint and 50 MHz clock are provisional until the Tiny Tapeout flow completes placement, routing, and timing checks.
