# Experimental DFF/latch Radix-4 Booth MAC

The earlier production integration did not include a latch experiment. `MAC_LATCH_OPTIMIZATION.md` was absent from the checkout and named Git history, so this run benchmarks a new, isolated hybrid against the current production RTL. The runner generates `fp32_mac_round_latch` from the production source without editing it. [Runner](benchmarks/mac/latch_benchmark.py) · [Generated RTL](benchmarks/mac/results/latch_benchmark/fp32_mac_round_latch.sv)

The hybrid replaces 34 DFF bits between ROUND and PACK with low-phase transparent storage for the rounded significand, discard bit and biased exponent. One high-phase latch holds the enable stable during the low phase. The data bank has no reset value because PACK always follows a fresh ROUND capture. The controller, Booth recurrence, accumulator and published outputs retain their production DFFs. Both designs use the same request interface and cycle schedule.

## Matched SKY130 mapping

Both designs used Yosys 0.52, the SKY130 HD TT 25 °C, 1.80 V Liberty, the same flat wrapper, `dfflibmap`, and `abc -D 20000 -liberty`. The flow explicitly maps inferred positive-enable latches to `sky130_fd_sc_hd__dlxtp_1`. The mapping checks found zero unmapped logic and no unexpected storage types. All area values are sums of mapped Liberty cell areas.

| Metric | Production | Hybrid |
|---|---:|---:|
| Total mapped area (µm²) | 35,656.698 | 35,051.117 |
| Combinational area (µm²) | 26,272.698 | 25,992.429 |
| Sequential area (µm²) | 9,384.000 | 9,058.688 |
| Cells | 4,807 | 4,620 |
| DFFs | 375 | 341 |
| Latches | 0 | 35 |

The hybrid saves **605.581 µm² (1.70%)**. [Machine-readable summary](benchmarks/mac/results/latch_benchmark/summary.json) · [Production synthesis](benchmarks/mac/results/latch_benchmark/production/yosys.log) · [Hybrid synthesis](benchmarks/mac/results/latch_benchmark/hybrid/yosys.log)

## Correctness and cycle timing

Each RTL version passed **135,113 expanded requests** and **10,112 directed/protocol requests**, with zero numerical or flag mismatches and zero timeouts. The testbench checks result bits, accumulator, all exception flags, idle clear, publication under last, reset cancellation, and back-to-back acceptance. The expanded traces matched on every acceptance-to-done latency. Both have a **19.188-cycle mean**, **22-cycle maximum**, and can accept a request on the edge after completion. The supplemental mapped simulation uses zero-delay functional models from the Liberty. Both mapped netlists passed all 10,112 directed/protocol requests with the same 22-cycle maximum. [Production RTL log](benchmarks/mac/results/latch_benchmark/production/protocol.log) · [Hybrid RTL log](benchmarks/mac/results/latch_benchmark/hybrid/protocol.log) · [Production mapped log](benchmarks/mac/results/latch_benchmark/production/mapped_protocol.log) · [Hybrid mapped log](benchmarks/mac/results/latch_benchmark/hybrid/mapped_protocol.log)

**Physical timing is unverified.** OpenSTA and OpenROAD were unavailable. A latch-aware timing flow must model the low-phase latch gate, clock duty cycle and skew, pulse widths, and setup and hold through ROUND and PACK. The ABC delay target and zero-delay simulation provide no physical frequency measurement.

## Reproduce the RTL benchmark and area

From `UMARV-SensorFusion`:

```sh
python3 -B -u benchmarks/mac/latch_benchmark.py \
  --liberty /home/Nabhah/sky130_lib/sky130_fd_sc_hd__tt_025C_1v80.lib
```
