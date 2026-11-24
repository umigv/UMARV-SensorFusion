# Architecture A · SKY130 1×2 floorplan capacity

**FAIL for the existing mapped netlist under the default Tiny Tapeout SKY130 margins.**

| Quantity | Value |
| --- | ---: |
| Tile rectangle | 161.00 × 225.76 µm |
| Core after template margins | 155.48 × 220.32 µm |
| Maximum full placement sites before obstructions | 338 × 81 = 27,378 |
| Maximum placement-row area | 34,255.3536 µm² |
| Mapped cell area | 35,774.3104 µm² |
| Mapped site-area equivalents | 28,592 |
| Required utilization of available rows | 104.43% |
| Area shortfall | 1,518.9568 µm² |

Each horizontal margin is six 0.46 µm sites. Each vertical margin is one 2.72 µm site. The calculation assumes every full site inside the resulting rectangle is available, before tap cells, obstructions, or a Tiny Tapeout I/O wrapper. It uses the existing standalone mapped netlist. [Tiny Tapeout tile dimensions](https://raw.githubusercontent.com/TinyTapeout/tt-support-tools/main/tech/sky130A/tile_sizes.yaml) · [Default SKY130 template](https://github.com/TinyTapeout/ttsky-verilog-template/blob/main/src/config.json) · [LibreLane margin definitions](https://librelane.readthedocs.io/en/latest/reference/step_config_vars.html) · [SKY130 HD technology LEF](https://github.com/google/skywater-pdk-libs-sky130_fd_sc_hd/blob/main/tech/sky130_fd_sc_hd.tlef) · [Mapped cell area](area.json)

OpenROAD and the local SKY130 physical LEFs were unavailable. The shell could not resolve GitHub to fetch them. This is a floorplan capacity test. Placement and routing were not run.
