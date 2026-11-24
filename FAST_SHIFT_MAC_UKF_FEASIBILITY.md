# SR-UKF timing feasibility with the serial FP32 MAC and fast shifts

**The serial-fast MAC is not a conservative pass with the original Cholesky prediction.** Estimated work is below 62,500 cycles on the replay operands, but the conservative prediction subtotal is **63,667 cycles** before trig, wraps, or conversion execute. A hybrid prediction with circular positive updates and the original negative central downdate is a **conditional** pass. It leaves **8,678 cycles** for the missing work with a positive center, or **7,783 cycles** with a negative center. Correction leaves **31,300 cycles**. Physical 50 MHz operation remains unverified.

**Keep Booth A for the original Cholesky schedule.** The serial-fast area saving does not buy a two-tile implementation with the separately mapped divider/root. Choose serial-fast with the hybrid only if measured conversion, trig, wrap and integration costs fit the remaining margin, the numerical failure policy is validated, and routed timing and area justify the swap.

Each 400 Hz epoch lasts 2.5 ms. The archived scheduler reserves 1.25 ms for prediction and 1.25 ms for correction. At the assumed 50 MHz, each phase gets 62,500 cycles. The analysis uses the planned, serialized five-state FP32 schedule for `[x, y, θ, ω, b_v]ᵀ` and the archived four-component VN-300 observation structure. The active tree supplies arithmetic RTL and a simulation harness. A production FP32 sequencer, register file, signed encoder-to-FP32 converter, trigonometry and wrap blocks remain unimplemented in this tree. [FP32 schedule](FP32_TIMING_ANALYSIS.md) · [Previous phase ledger](ARCH_A_TIMING_FEASIBILITY.md)

## Prediction operation counts

A clear-start MAC clears the accumulator and multiplies. To add an external term, the microcode seeds the MAC with that term times one, then issues a continued MAC. The table includes those seed instructions. Sign changes select the operand sign bit.

| Stage | Clear-start MAC | Continued MAC | Divide | Square root | Other execution |
|---|---:|---:|---:|---:|---|
| Convert signed 28-bit encoder delta and scale to dead-wheel speed | 1 | 0 | 0 | 0 | 1 integer conversion |
| Generate 11 sigma points per pass, twice | 60 | 60 | 0 | 0 | |
| Midpoint propagation for 22 points | 154 | 110 | 0 | 0 | 22 sin/cos pairs and 44 wraps |
| Five weighted 11-term means | 5 | 50 | 0 | 0 | 1 wrap |
| Form 55 weighted deviations | 110 | 55 | 0 | 0 | |
| A. Original Cholesky, 11 rows | 275 | 275 | 220 | 55 | |
| B. Ten circular rows and one original negative central row | 375 | 275 | 70 | 55 | |
| C. Eleven rotation rows, either central sign | 385 | 275 | 55 | 55 | |
| **A total** | **605** | **550** | **220** | **55** | **68 commands** |
| **B total, negative central** | **705** | **550** | **70** | **55** | **68 commands** |
| **B total, positive central / C total** | **715** | **550** | **55** | **55** | **68 commands** |

The replay starts with the signed count delta already available. It uses a fixture calibration of 50 µm/count and the fixed 2.5 ms epoch, so wheel speed uses one multiplication by a configured constant. Each pass regenerates the 11 sigma points, sharing unchanged components. Propagation computes `d=(v_wheel−b_v)Δt`, turn `ωΔt`, a wrapped midpoint for trig, a lifted endpoint for heading statistics, and two fused position updates. The schedule also counts one endpoint wrap per point per pass. Weighted covariance starts at the supplied process factor `S_Q`. Ten weighted outer deviations stream into it, then the central deviation updates or downdates it. The historical sigma scale is 2 and the central covariance weight is positive. Negative central is a sensitivity case using the same magnitude with reversed covariance sign. The planned schedule omits the historical wheel lever arm and optional wheel-noise row. If required, count acquisition/differencing, those terms, and sensor ingress need separate budget.

Each five-dimensional original rank-one operation traverses five pivots and ten trailing entries. It issues 25 clear starts, 25 continues, 20 divisions and five roots. Rotation trades 15 divisions for 10 additional MAC starts and additional moves. Both schedules execute the final pivot coefficients even when unused. The hybrid uses circular rotations for all positive rows, including a positive central row. Fully rotation-based mode additionally uses hyperbolic rotations for a negative center. [Rank-one reference and numerical results](CHOLESKY_OPTIMIZATION_ANALYSIS.md)

## Correction operation counts

Correction observes the first four state components with `H=[I₄ 0]`. The replay begins with the predicted factor and a diagonal positive measurement-noise factor. It forms the four-dimensional innovation covariance factor by streaming four noise rows into the predicted upper block, computes the cross covariance and Kalman gain via forward and backward triangular solves, updates all five state components, and performs four original posterior downdates with the forward-solve columns. It wraps the heading innovation and the corrected heading. The fixture and bound below apply to diagonal measurement noise with the indicated active suffixes.

| Stage | Clear-start MAC | Continued MAC | Divide | Square root | Other execution |
|---|---:|---:|---:|---:|---|
| Four innovations | 4 | 4 | 0 | 0 | 1 wrap |
| Innovation covariance factor | 30 | 30 | 30 | 10 | |
| Cross covariance and forward solves | 20 | 90 | 20 | 0 | |
| Backward solves for gain | 20 | 30 | 20 | 0 | |
| Five state corrections | 5 | 20 | 0 | 0 | 1 wrap |
| Four posterior downdates | 100 | 100 | 80 | 20 | |
| **Total** | **179** | **274** | **150** | **30** | **2 wraps** |

The 4D innovation factor schedules ten pivots and ten trailing entries. The dense original recurrence divides twice per pivot and once per trailing entry, for **30 divisions**. The earlier Architecture A ledger used 20 at this stage. The comparison below uses the same traced schedule for both MACs. A full measurement-noise factor can change the active row counts.

## Verified RTL replay

The Python harness emits the arithmetic and data movement for eight seeded state/factor and wheel scenarios. Correction uses each original positive-central predicted state and factor and a seeded VN-300-like observation. The common Icarus harness executes the same stream on the generated serial-fast variant and public Architecture A, with the unchanged FP32 division/square-root RTL. The operation stream follows the planned FP32 schedule rather than the archived fixed-point instruction stream. The fixtures are synthetic. The phases start with sensor and state inputs loaded. Python supplies FP32-rounded encoder conversion, sin/cos and wrap results to the harness. Those results exercise downstream arithmetic and heading branch crossings, and their execution latency is outside the replay.

Each MAC, divide and square root is timed from the accepting edge to `done`. Each result and all five flags are compared with the bit-exact arithmetic oracle at completion, including unpublished MAC accumulator values. The replay cycle totals include simulated single-port reads, issue, retirement, writes and explicit moves. A separate binary64 calculation checks reconstructed prediction and posterior covariances. That check measures factor accuracy on the selected fixtures. [Replay runner](benchmarks/mac/fast_shift_feasibility.py) · [Testbench](tests/tb_cholesky_experiment.sv)

| Scenario | Serial-fast min / mean / max | Booth A min / mean / max |
|---|---:|---:|
| A. Original, positive central | 48,991 / 49,675 / 50,118 | 36,869 / 37,333 / 37,622 |
| A. Original, negative central | 48,994 / 49,680 / 50,124 | 36,872 / 37,338 / 37,628 |
| B/C. Positive central | 47,073 / 47,756.5 / 48,199 | 33,862 / 34,315.9 / 34,603 |
| B. Hybrid, negative central | 47,220 / 47,906.4 / 48,350 | 34,108 / 34,574.4 / 34,864 |
| C. All rotation, negative central | 47,076 / 47,761.5 / 48,205 | 33,865 / 34,320.9 / 34,609 |
| VN-300 correction | 21,236 / 21,246.4 / 21,276 | 16,825 / 16,835.4 / 16,865 |

Each MAC architecture completed **71,984 arithmetic requests in 56 phase replays** with zero result mismatches, flag mismatches, timeouts, or numerical failures. Relative covariance reconstruction error stayed below **1.74×10⁻⁷** for prediction and **2.01×10⁻⁶** for correction. The serial-fast MAC measured 1 or 30 cycles on clear starts and 1, 30, 32 or 33 on continues. Division took 1 or 31 cycles and all roots took 31 on these fixtures.

The prior boundary sweep rejected 6 of 67 reference-SPD cases with the original downdate and 7 with the hyperbolic variant. It also accepted 2 of 13 reference-invalid cases with the original downdate and 4 with the hyperbolic variant. The current fixture pass provides no change to that boundary evidence. Error detection, rollback and recovery need integration and separate timing. [Downdate boundary results](CHOLESKY_OPTIMIZATION_ANALYSIS.md)

## Modeled phase budgets

The expected subtotal uses the mean of each scenario's measured arithmetic and serialized traffic, plus modeled issue/retire and RF traffic for the host commands, phase entry/exit, and the initial prediction factor copy. It preserves measured zero-product shortcuts, zero accumulators, nonzero additions, cancellation and negation. It never substitutes the overall 29.349-cycle MAC benchmark mean for the actual phase operands.

A clear-start serial-fast nonzero product takes 30 edges after acceptance, 24 multiply steps plus product normalization, zero-accumulator handling, sum normalization, round, pack and complete. Continued normal work adds alignment and add; a negate can add one more edge. Exact cancellation bypasses round and pack. The serial-fast FSM gives conservative clear/continue limits of **30/33**, compared with **19/22** for public A. The established divider/root limits are **81/54** cycles. The conservative subtotal applies these limits to the exact dense counts and adds the largest scheduled overhead. This bounds identified work for the modeled schedule, including in cases where independent primitive maxima cannot occur simultaneously. [Primitive timing](FP32_TIMING_ANALYSIS.md) · [MAC implementation and mapping](MAC_SKY130_AREA_COMPARISON.md)

For each serialized command the harness charges one edge per variable operand read, one edge per destination write, plus two controller edges for issue and retirement. An identical source can fan out to both inputs. FP32 zero and one are constants, and other loaded constants occupy modeled RF slots. The model adds 294 cycles for the prediction converter/trig/wrap operand traffic and instruction boundaries, 60 for 15 `S_Q` copies, and two phase edges. Correction adds eight cycles for the two wrap commands' traffic and issue/retire, plus two phase edges. The register-file organization described here is a simulation policy. Physical RF latency, fetch stalls, arbitration, and additional phase or safety control have not been measured.

| Scenario | Expected cycles | Expected remaining | Expected used | Conservative cycles | Conservative remaining | Conservative used |
|---|---:|---:|---:|---:|---:|---:|
| A. Original, positive central | 50,031 | 12,469 | 80.05% | 63,667 | -1,167 | 101.87% |
| A. Original, negative central | 50,036 | 12,464 | 80.06% | 63,667 | -1,167 | 101.87% |
| B/C. Positive central | 48,112.5 | 14,387.5 | 76.98% | 53,822 | 8,678 | 86.12% |
| B. Hybrid, negative central | 48,262.4 | 14,237.6 | 77.22% | 54,717 | 7,783 | 87.55% |
| C. All rotation, negative central | 48,117.5 | 14,382.5 | 76.99% | 53,822 | 8,678 | 86.12% |
| VN-300 correction | 21,256.4 | 41,243.6 | 34.01% | 31,200 | 31,300 | 49.92% |

| Serial-fast policy | Arithmetic plus move commands | RF reads | RF writes | Total overhead including host boundaries |
|---|---:|---:|---:|---:|
| A prediction | 1,485 | 2,399 | 1,040 | 6,577 |
| B negative-central prediction | 1,535 | 2,449 | 1,090 | 6,777 |
| B positive-central / C prediction | 1,540 | 2,454 | 1,095 | 6,797 |
| Correction | 723 | 1,115 | 451 | 3,018 |

RF counts include the modeled host traffic and prediction factor-copy traffic. The command count in the overhead table lists replayed arithmetic and moves only.

For prediction, the conservative remaining cycles cover `C_convert + 22×C_sincos + 45×C_wrap + C_extra`, using per-command worst-case execution latency. For correction they cover `2×C_wrap + C_extra`. Here `C_extra` includes omitted integration work such as sensor ingress, banking costs beyond the model, and any required replay or recovery. In policy A the negative allowance means the primitive-max subtotal itself exceeds the budget. It does not establish a missed hardware deadline on realistic operands.

## Comparison with Radix-4 Booth

Prediction uses the historical positive central sign. Both architectures use the same schedule in each row. Remaining budgets are shown after the conservative known-work bound.

| Metric | Original + fast shifts | Radix-4 Booth A |
|---|---:|---:|
| Mapped MAC area | 31,908.102 µm² | 35,774.310 µm² |
| Mapped DFFs | 300 | 375 |
| Prediction, original Cholesky expected cycles | 50,031 | 37,689 |
| Prediction, original Cholesky conservative bound | 63,667 | 50,962 |
| Prediction, original Cholesky remaining budget | -1,167 | 11,538 |
| Prediction, rotation-positive hybrid expected cycles | 48,112.5 | 34,671.9 |
| Prediction, rotation-positive hybrid conservative bound | 53,822 | 39,907 |
| Prediction, rotation-positive hybrid remaining budget | 8,678 | 22,593 |
| Correction expected cycles | 21,256.4 | 16,845.4 |
| Correction conservative bound | 31,200 | 26,217 |
| Correction remaining budget | 31,300 | 36,283 |
| Full 400 Hz feasibility | Original Cholesky exceeds the conservative prediction budget. Hybrid is conditional. | Both known-work bounds fit. Full phase pass is conditional. |

On the matched original Cholesky schedule, serial-fast costs **12,342 more expected prediction cycles** and **4,411 more expected correction cycles**. At an assumed 50 MHz these are 246.84 µs and 88.22 µs. Keeping the MAC choice fixed and changing prediction to positive rotations saves about **1,919 expected cycles** and **9,845 conservative cycles** on serial-fast. The conservative gain comes principally from issuing fewer worst-case divisions.

## Area and physical timing

The serial-fast MAC saves **3,866.208 µm²** (10.81% of Booth A's mapped area) and **75 DFFs**. Its **31,908.102 µm²** is **93.15%** of the estimated **34,255.35 µm²** of placement rows under the default SKY130 1×2 template. Only **2,347.25 µm²** of that raw row area remains. The corresponding standalone divider/root mapped to **11,325.862 µm²** with the same Liberty and ABC target. The two separately mapped arithmetic blocks sum to **43,233.965 µm²**, already above the available rows before adding the register file, sequencer, trig, angle wrap, input conversion, Tiny Tapeout wrapper, clock tree, or tap cells. That sum describes a two-block implementation with these mappings. An integrated mapping can differ. [MAC area report](MAC_SKY130_AREA_COMPARISON.md) · [1×2 row capacity](benchmarks/mac/results/sky130_area_mapped/radix4_a/tt_1x2_capacity.md) · [Division/root synthesis log](benchmarks/mac/results/fast_shift_feasibility/divsqrt_yosys.log)

**50 MHz closure remains unverified.** RTL cycle counts and the 20 ns ABC mapping target supply no physical clock-period measurement. Placement and routing were unavailable. An integrated design would need placed and routed utilization, and timing across the arithmetic, storage and control paths.

## Reproduction

From `UMARV-SensorFusion`:

```bash
set -o pipefail
out=benchmarks/mac/results/fast_shift_feasibility
mkdir -p "$out"
python3 -B -u benchmarks/mac/fast_shift_feasibility.py 2>&1 | tee "$out/run.log"
PYTHONDONTWRITEBYTECODE=1 make test synth-check > "$out/regression.log" 2>&1
lib=/home/Nabhah/sky130_lib/sky130_fd_sc_hd__tt_025C_1v80.lib
yosys -p "read_liberty -lib -ignore_miss_func $lib; read_verilog -sv fp32_divsqrt.sv; hierarchy -check -top fp32_divsqrt; synth -flatten -noabc -top fp32_divsqrt; dfflibmap -liberty $lib; abc -D 20000 -liberty $lib; clean -purge; check -assert; stat -liberty $lib" > "$out/divsqrt_yosys.log" 2>&1
git diff HEAD --exit-code -- fp32_mac.sv fp32_divsqrt.sv
```

The test-only changes parameterize the simulation register file, enable per-request latency logging, and reuse the existing rank-one reference with already-loaded rows. [Serial-fast log](benchmarks/mac/results/fast_shift_feasibility/original_fast.log) · [Booth log](benchmarks/mac/results/fast_shift_feasibility/radix4_a.log) · [Replay and oracle data](benchmarks/mac/results/fast_shift_feasibility/replay.json) · [Timing model data](benchmarks/mac/results/fast_shift_feasibility/estimate.json) · [Run summary](benchmarks/mac/results/fast_shift_feasibility/run.log) · [Regression log](benchmarks/mac/results/fast_shift_feasibility/regression.log)
