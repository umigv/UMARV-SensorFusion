# SKY130 mapped area and RTL latency of the FP32 MACs

Yosys 0.52 successfully mapped all five requested designs to SKY130 HD at TT, 25 °C, 1.80 V. The original has the smallest mapped area, and Architecture F completes in the fewest RTL cycles. Physical timing and 50 MHz closure remain unverified because OpenSTA and OpenROAD were unavailable.

| Area rank | Architecture | Total area (µm²) | Cells | DFFs | Combinational (µm²) | Sequential (µm²) | Area increase | Latency rank | Mean cycles | Max observed cycles |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | Original FP32 MAC | 18,450.195 | 2,181 | 300 | 10,942.995 | 7,507.200 | 0.00% | 5 | 53.197 | 177 |
| 2 | Original multiplier with fast normalization/alignment | 31,908.102 | 4,222 | 300 | 24,400.902 | 7,507.200 | 72.94% | 4 | 29.349 | 33 |
| 3 | A · Radix-4 Booth | 35,774.310 | 4,642 | 375 | 26,390.310 | 9,384.000 | 93.90% | 3 | 19.188 | 22 |
| 4 | D · 24×8 partial-parallel | 39,724.349 | 5,379 | 368 | 30,515.517 | 9,208.832 | 115.31% | 2 | 9.950 | 12 |
| 5 | F · Parallel Dadda | 51,197.853 | 6,665 | 324 | 43,090.077 | 8,107.776 | 177.49% | 1 | 8.103 | 10 |

Area ranks use summed Liberty cell area. Latency ranks use maximum observed RTL cycles, with the same ordering by mean cycles. All five designs passed the same 135,113 bit-exact requests. The original, A, D and F also have analytically derived cycle bounds equal to their observed maxima. The 33-cycle figure for original-fast is an observed maximum. [Benchmark results](benchmarks/mac/results/summary.json) · [Original-fast latency](benchmarks/mac/results/sky130_area_mapped/original_fast/latency.json) · [Cycle-bound derivation](MAC_ARCHITECTURE_COMPARISON.md)

## Mapping and verification

The existing runner used a common wrapper and matched `synth -flatten -noabc`, `dfflibmap -liberty`, and `abc -D 20000 -liberty` settings. The ABC 20,000 ps target is a mapping constraint. Post-mapping `check -assert` found zero problems for every top. Every physical instance matched a Liberty cell with a known area, and every sequential instance was a DFF. The report excludes Yosys scope metadata from the cell count. Summed Liberty areas matched Yosys `stat -liberty` totals. Production RTL was unchanged. [Mapping checks](benchmarks/mac/results/sky130_area_mapped/mapping_checks.json) · [Area data](benchmarks/mac/results/sky130_area_mapped/area_summary.json) · [Run metadata and Liberty checksum](benchmarks/mac/results/sky130_area_mapped/run.json)

Run from `UMARV-SensorFusion`:

```sh
python3 benchmarks/mac/sky130_area.py \
  --liberty /home/Nabhah/sky130_lib/sky130_fd_sc_hd__tt_025C_1v80.lib \
  --out benchmarks/mac/results/sky130_area_mapped
```

Per-design directories contain the Yosys script and log, mapped Verilog and JSON netlists, and a cell breakdown with area. [Original log](benchmarks/mac/results/sky130_area_mapped/original/yosys.log) · [Original-fast log](benchmarks/mac/results/sky130_area_mapped/original_fast/yosys.log) · [A log](benchmarks/mac/results/sky130_area_mapped/radix4_a/yosys.log) · [D log](benchmarks/mac/results/sky130_area_mapped/partial_d/yosys.log) · [F log](benchmarks/mac/results/sky130_area_mapped/dadda_f/yosys.log)

## Tiny Tapeout floorplan area

The published SKY130 1×1 macro rectangle is **161.00 × 111.52 µm = 17,954.72 µm²**. That is floorplan area. The table measures standard-cell area before placement. The original alone uses 102.76% of the nominal rectangle by area, so all five mapped netlists exceed a single tile even before accounting for placement margins, power distribution, routing, and a Tiny Tapeout I/O wrapper. Multi-tile fit remains unverified. [Tiny Tapeout SKY130 tile dimensions](https://raw.githubusercontent.com/TinyTapeout/tt-support-tools/main/tech/sky130A/tile_sizes.yaml)

## Multiplier and shift isolation

The runner generates the controlled variants outside production RTL. Original-fast keeps the production radix-2 multiplier, carry-lookahead adder and remaining controller, and transplants the four fast-shift state bodies and their helper logic. Both matched Booth variants use the production final FP32 adder. Public A maps separately with its default hybrid adder.

| Controlled configuration | Max observed cycles | Mapped area (µm²) |
| --- | ---: | ---: |
| Original multiplier and original shifts | 177 | 18,450.195 |
| Original multiplier and fast shifts | 33 | 31,908.102 |
| Narrow radix-4 Booth and original shifts | 166 | 22,429.011 |
| Narrow radix-4 Booth and fast shifts | 22 | 35,960.739 |

Fast shifts add 13,457.907 µm² with the original multiplier. The matched Booth multiplier adds 3,978.816 µm² with original shifts. Their area interaction is +73.821 µm². Public A saves 186.429 µm² compared with the matched fast-shift Booth configuration. These are whole-design deltas because flattening and ABC can share logic across design boundaries. [Earlier isolation verification and blocked-run logs](benchmarks/mac/results/sky130_area)
