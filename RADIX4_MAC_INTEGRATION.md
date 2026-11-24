# Production FP32 MAC · Radix-4 Booth Architecture A

The production `fp32_mac.sv` now implements Architecture A. It keeps the existing module name, ports and accumulative interface, including the internal `accumulator` used by the testbenches. The production file is self-contained and has one MAC lane. Its Booth recurrence uses a single narrow adder to build the product in 13 cycles. The wide hybrid adder handles the fused addition, magnitude and rounding. The hierarchical leading-zero normalizer and barrel alignment with sticky-bit jamming retain the verified fast paths. The production specialization fixes the original configuration at `ARCH=0`, `TOPOLOGY=4`, `FAST_SHIFT=1`. No production divider/root or Cholesky mathematics changed. [Production RTL](fp32_mac.sv) · [Verified benchmark core](benchmarks/mac/fp32_mac_experiment.sv) · [Architecture comparison](MAC_ARCHITECTURE_COMPARISON.md)

The serial implementation is preserved in `fp32_mac_serial_legacy.sv` with only its module name changed. Default production synthesis reads `fp32_mac.sv` only. Historical comparison runners select the legacy file explicitly so the original and fast-shift-only reference arms retain their intended datapaths. [Legacy RTL](fp32_mac_serial_legacy.sv)

## Arithmetic, protocol and latency

| Check | Result |
|---|---|
| Expanded Architecture A distribution | 135,113 requests passed, with exact result, accumulator and five flag bits |
| Same distribution, latency | Minimum **1**, mean **19.187716948**, median **21**, p95/p99 **22**, maximum **22** cycles from accepted start to done |
| Prior experimental cycle stream | All 135,113 individual latency values match |
| Default directed/random MAC regression | 10,112 requests passed, including deliberate busy-edge requests |
| Directed MAC timing workload | 15,112 requests passed, maximum 22 cycles |
| Existing shared FP32 unit | 12,162 divide and 12,162 sqrt requests passed, maxima 81 and 54 cycles |
| Existing Cholesky RTL suite | 43,348 arithmetic requests passed across 96 schedules |
| Yosys equivalence to experimental A | 409 equivalence points proven, zero unproven, with asynchronous reset converted for the induction check |
| Legacy restore smoke test | 15,113 requests passed, maximum 177 cycles |

The archived isolation variants also passed 15,113 requests each. The standalone adder SAT check proved the sums for all five experimental topologies over all operands and carry inputs. The supplemental randomized adder simulation was stopped after the exhaustive proof passed. [Isolation log](benchmarks/mac/results/radix4_integration/isolation_smoke.log) · [Adder proof](benchmarks/mac/results/radix4_integration/adder_formal.log)

The MAC regression covers multiplication and FMA, both signs, subnormals, NaNs, infinities, exact cancellation, rounding boundaries, underflow, overflow and all exception flags. It checks accumulator state with and without publication, idle `clear`, `clear` on a request, latched `last`, reset cancellation, one-cycle completion pulses, and acceptance on the edge after completion. The recurrence consumes 13 cycles. The long path adds nine cycles for normalization, alignment, fused add, optional negate, rounding, packing and completion. Its 22-cycle bound is attained in RTL. The maximum acceptance-to-acceptance interval is 23 cycles. The timing target formerly asserted the serial maximum of 177 cycles and now checks 22 by default. [Expanded log](benchmarks/mac/results/radix4_integration/expanded_regression.log) · [Latency summary](benchmarks/mac/results/summary_production.json) · [Exact trace comparison](benchmarks/mac/results/radix4_integration/comparison.json) · [Default regression](benchmarks/mac/results/radix4_integration/regression.log) · [Equivalence check](benchmarks/mac/results/radix4_integration/equiv.log) · [Cholesky log](benchmarks/mac/results/radix4_integration/cholesky.log)

## SR-UKF cycle budgets with original Cholesky

The production lane and unchanged `fp32_divsqrt.sv` replayed **27,944 bit-exact arithmetic requests across 24 synthetic phase schedules**. Each schedule matched the prior experimental Architecture A timing and flag histogram, including modeled register accesses, issue and retirement. There were no additional replay stalls, timeouts, result mismatches or flag mismatches. The reference schedule issues 605 clear-start MACs, 550 continued MACs, 220 divisions and 55 roots for prediction. Correction issues 179 clear-start MACs, 274 continued MACs, 150 divisions and 30 roots. [Replay data](benchmarks/mac/results/radix4_integration/phase_replay/replay.json) · [Schedule rationale](FAST_SHIFT_MAC_UKF_FEASIBILITY.md)

| Phase | Replay mean | Expected known work | Conservative known bound | Cycles remaining of 62,500 |
|---|---:|---:|---:|---:|
| Prediction, positive central update | 37,333.000 | 37,689.000 | 50,962 | 11,538 |
| Prediction, negative central downdate | 37,338.000 | 37,694.000 | 50,962 | 11,538 |
| VN-300 correction, four posterior downdates | 16,835.375 | 16,845.375 | 26,217 | 36,283 |

At the assumed 50 MHz clock, 400 Hz operation allocates 62,500 cycles to each phase. The prediction margin must cover encoder conversion, 22 sin/cos calls, 45 angle wraps, and extra integration work. The correction margin must cover two wraps and extra integration work. The harness supplies software results for those operations and models controller and single-port register traffic. Full integrated 400 Hz operation and physical 50 MHz timing remain unverified. [Phase log](benchmarks/mac/results/radix4_integration/phase_replay.log) · [Cycle model](benchmarks/mac/results/radix4_integration/phase_replay/estimate.json)

## SKY130 mapped area

Yosys 0.52 used the same wrapper, SKY130 HD TT 25 °C, 1.80 V Liberty, flattening, `dfflibmap` and `abc -D 20000 -liberty` settings used for the archived Architecture A result. The 20,000 ps ABC target is a mapping option. The matched mapping passed `check -assert`. Every physical cell is in the Liberty file and all 375 sequential cells mapped to resettable DFFs. There are zero latches and zero unmapped logic instances. Generic synthesis rejects inferred multiplier and divider operators, and the variable shifts lower to logic. The source instantiates a single core in the comparison wrapper and contains a single Booth recurrence.

| Metric | Experimental A | Production A |
|---|---:|---:|
| Total mapped area (µm²) | 35,774.3104 | **35,656.6976** |
| Combinational area (µm²) | 26,390.3104 | 26,272.6976 |
| Sequential area (µm²) | 9,384.0000 | 9,384.0000 |
| Mapped cells | 4,642 | 4,807 |
| DFFs | 375 | 375 |

Production area is **117.6128 µm² lower (0.329%)**. Specializing the parameterized core and putting its Booth and prefix logic in the same module changes the generic combinational network presented to ABC. ABC chose more cells with lower summed combinational area. The DFF mix and measured behavior are identical. The separate Makefile SKY130 target also passed. The table uses the matched flow for comparability. Physical timing and routed utilization are unverified. [Production area and cell histogram](benchmarks/mac/results/radix4_integration/production/area.json) · [Yosys log](benchmarks/mac/results/radix4_integration/production/yosys.log) · [Liberty checksum and tools](benchmarks/mac/results/radix4_integration/run.json) · [Archived A area](benchmarks/mac/results/sky130_area_mapped/radix4_a/area.json)

## Reproduce

From `UMARV-SensorFusion`:

```sh
python3 -B -u benchmarks/mac/benchmark.py --only production
PYTHONDONTWRITEBYTECODE=1 make test synth-check timing synth
python3 -B -u tests/cholesky_benchmark.py --rtl
python3 -B -u benchmarks/mac/fast_shift_feasibility.py --production
python3 -B benchmarks/mac/sky130_area.py --production \
  --liberty /home/Nabhah/sky130_lib/sky130_fd_sc_hd__tt_025C_1v80.lib
yosys -s benchmarks/mac/results/radix4_integration/equiv.ys
out=benchmarks/mac/results/radix4_integration
yosys -p "read_verilog -sv benchmarks/mac/mac_adders.sv $out/adder_miter.sv; prep -top adder_miter -flatten; sat -verify -prove ok 1"
diff <(gzip -dc benchmarks/mac/results/a.cycles.gz) \
     <(gzip -dc benchmarks/mac/results/production.cycles.gz)
```

The last command uses Bash process substitution.

## Restore the serial core if needed

Save the current Booth source through version control before restoring. From `UMARV-SensorFusion`, regenerate `fp32_mac.sv` from the archived serial copy and run the interface-compatible tests:

```sh
sed 's/^module fp32_mac_serial_legacy (/module fp32_mac (/' \
  fp32_mac_serial_legacy.sv > fp32_mac.sv
make test synth-check synth
FP32_MAC_LATENCY_BOUND=177 make timing
```

The named legacy module can also be instantiated explicitly in an isolated diagnostic build. The production replay and expanded production benchmark in this report expect Architecture A until those entry points are switched back.
