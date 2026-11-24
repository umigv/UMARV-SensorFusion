# Architecture A timing feasibility at 400 Hz

**Verdict: projected cycle-budget pass for prediction and correction, with no confirmed end-to-end or physical 50 MHz pass.** Under the documented serialized schedule, Architecture A leaves **11,775 cycles** after the conservative known-work subtotal of the original prediction and **36,606 cycles** after the corresponding correction subtotal. The missing operations and integration overhead must fit those margins, and the mapped system must close at 20 ns. The repository does not contain that integrated FP32 design.

## Evidence and scope

One 20 ns clock gives 62,500 cycles per 1.25 ms window. The archived 400 Hz scheduler uses `SLOT_CYCLES = CLK_HZ / 800` and a 2,500 µs timestamp period. Its source and the archived controller are available through `git show fixed-point-baseline:srukf_400hz_scheduler.sv` and `git show fixed-point-baseline:srukf_bias_filter.sv`. Their arithmetic is fixed-point. The active tree has the two FP32 arithmetic units, the experimental MACs, and simulation microcode. It lacks the integrated FP32 sequencer, synthesized register file, trigonometry, angle wrap, and integer-to-FP32 conversion. The count below uses the documented planned FP32 schedule and the archived four-measurement correction structure. [Arithmetic timing report](FP32_TIMING_ANALYSIS.md) · [MAC comparison](MAC_ARCHITECTURE_COMPARISON.md) · [Cholesky benchmark](CHOLESKY_OPTIMIZATION_ANALYSIS.md)

Architecture A here is `fp32_mac_a` with the default hybrid adder and fast leading-zero and barrel-jam paths. Its expanded regression measured 19.188 cycles overall and reached the derived 22-cycle bound. A normal product with a cleared accumulator takes 19 cycles. A continued finite MAC can take 19 on exact cancellation, 21 on the ordinary two-term path, or 22 with negation. Classified shortcuts can take one cycle. The maximum acceptance interval is 23 cycles, one more than completion latency. The benchmark's matched normal-input subgroup means are 19.000 and 21.349 cycles for clear-start and continued MAC respectively. Division measured 33.792 mean and 81 maximum, and square root measured 31.045 mean and 54 maximum. The nonexceptional div/root floor is 31 cycles. All latencies are acceptance-to-done. Issue and retirement already include request boundaries in the schedule. [MAC latency data](benchmarks/mac/results/summary.json) · [Matched operands](benchmarks/mac/results/schedule.json) · [Division and square-root RTL](fp32_divsqrt.sv)

## Prediction cycle ledger

| Stage | Clear-start MAC | Continued MAC | Division | Square root | Other commands |
| --- | ---: | ---: | ---: | ---: | --- |
| Encoder delta conversion and wheel velocity | 1 | 0 | 0 | 0 | 1 integer-to-FP32 |
| Sigma-point generation, both passes | 60 | 60 | 0 | 0 | |
| Midpoint propagation, both passes | 154 | 110 | 0 | 0 | 22 sin/cos pairs and 44 wraps |
| Weighted mean | 5 | 50 | 0 | 0 | 1 wrap |
| Weighted deviations | 110 | 55 | 0 | 0 | |
| Cholesky original, 11 rank-one rows | 275 | 275 | 220 | 55 | |
| Cholesky rotation, 11 rank-one rows | 385 | 275 | 55 | 55 | |
| Cholesky hybrid, 10 rotation updates + 1 original downdate | 375 | 275 | 70 | 55 | |

The model accepts a signed encoder delta and scales its converted value with a configured velocity-per-count constant. Encoder acquisition and raw-count differencing would need additional accounting if placed in the execution window. As in the previous plan, the factor starts at `S_Q` and the motion equations use wheel speed directly. The archived fixed-point filter's separate lever arm and wheel-noise row are outside this ledger. Their inclusion would require additional scheduled commands and operand timings.

The MAC has no arbitrary accumulator-load port. The ledger includes clear-start seed instructions for external addends. The original row uses 25 starts and 25 continues, 20 divisions, and five roots. The previous Cholesky replay established the 20-division count (ten pivot ratios and ten trailing-entry divisions). The rotation row uses 35 starts, 25 continues, five divisions, and five roots. It also preserves the old off-diagonal operands for each rotation. Dense schedules retain the last-pivot coefficient calculations even though their results are unused.

| Prediction policy | Total MAC starts / continues | Div / root | RF read + write cycles | Issue, retire, and phase cycles | Total overhead |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original | 605 / 550 | 220 / 55 | 2,162 + 1,040 | 3,138 | 6,340 |
| All rotation | 715 / 550 | 55 / 55 | 2,217 + 1,095 | 3,248 | 6,560 |
| Hybrid | 705 / 550 | 70 / 55 | 2,212 + 1,090 | 3,238 | 6,540 |

The model charges one cycle per single-port word access and two controller cycles per issued command, including moves, plus two phase-boundary cycles. Original prediction uses 1,430 arithmetic commands, 68 conversion/trig/wrap commands, and 70 moves. A rotation row adds five reads, five writes, and five total commands after accounting for moves and removed divisions. The model credits no parallel overlap. In particular, a continued FMA consumes the previous accumulator result, while division and root share their lane. Expansion to additional instruction-fetch cycles, memory arbitration, or phase-control cost needs a measured implementation.

Prediction unknowns are the execution cycles of one encoder-delta conversion, 22 sine/cosine commands, and 45 wraps, plus any integration overhead beyond this model. Issue/retire and RF traffic for those commands are already in the table.

## Correction cycle ledger

The four-dimensional VN-300 model is `H=[I₄ 0]`. Innovation QR forms the four-dimensional covariance factor. The two triangular solves produce the gain for all five state components, and four column downdates form the posterior factor.

| Stage | Clear-start MAC | Continued MAC | Division | Square root |
| --- | ---: | ---: | ---: | ---: |
| Four innovations | 4 | 4 | 0 | 0 |
| Innovation QR | 30 | 30 | 20 | 10 |
| Forward solves | 20 | 90 | 20 | 0 |
| Backward solves | 20 | 30 | 20 | 0 |
| Five state corrections | 5 | 20 | 0 | 0 |
| Four original posterior downdates | 100 | 100 | 80 | 20 |
| **Total** | **179** | **274** | **140** | **30** |

The model copies the predicted upper block into the empty innovation factor. The four measurement-noise rows contribute at most ten active pivots and ten trailing pairs in total. For the remaining stages, each of the five forward solve rows executes four cross-covariance terms per column followed by the relevant triangular terms. The backward solve seeds each column from the forward result, and the state correction seeds the state before its four gain products.

Overhead is 3,505 cycles, comprising 2,063 potential single-port accesses and 1,442 controller cycles. This charges up to three accesses per arithmetic command, two accesses for each of 95 moves and two heading wraps, two issue/retire cycles per command or move, and two phase-boundary cycles. Two wrap executions and any extra integration overhead remain outside this subtotal. The schedule keeps the original downdate, since the earlier numerical experiment found that hyperbolic rotation accepted more reference-indefinite near-boundary cases.

## New RTL replay

The new test reuses the existing Cholesky microcode testbench and bit-exact reference. It runs eight seeded cases for each applicable schedule, on A, D and F. Prediction cases use ten noncentral updates followed by either central sign. The hybrid cases use the original algorithm on the negative central row, operating on the factor actually produced by the preceding ten rotations. Correction samples use eight synthetic predicted factors and positive measurement-noise covariances. A binary64 `H=[I₄ 0]` innovation factor and triangular solve supply four FP32 downdate vectors per sample. This drives representative posterior operands at the repository's state scales. It does not simulate the preceding innovation, solve, or gain RTL.

The replay checks results and all flags on the real arithmetic modules. Its cycles include serialized register reads, issue, execution, retire, and writes in the covariance kernel. Inputs are preloaded because the full producer datapaths are absent. The full-phase expected proxy adds measured mean kernel latency to the remainder of the schedule, which uses normal-input MAC subgroup means and the supplied div/root averages. The samples are synthetic and deterministic, so this is an engineering proxy for representative operands rather than a measured production workload. [Replay runner](benchmarks/mac/arch_a_feasibility.py)

### Measured kernel latency on A

| Scenario | Samples | Minimum cycles | Mean cycles | Maximum cycles |
| --- | ---: | ---: | ---: | ---: |
| Prediction, original, positive central | 8 | 23,219 | 23,223.000 | 23,228 |
| Prediction, original, negative central | 8 | 23,222 | 23,227.625 | 23,232 |
| Prediction, all rotation, positive central | 8 | 20,187 | 20,193.500 | 20,199 |
| Prediction, all rotation, negative central (experimental) | 8 | 20,193 | 20,200.875 | 20,208 |
| Prediction, ten rotations and original negative central | 8 | 20,448 | 20,455.500 | 20,463 |
| Correction, original posterior | 8 | 8,541 | 8,546.500 | 8,561 |

The testbench verified 34,120 requests in 48 schedules per architecture. On A the eleven circular rotations saved 3,029.5 kernel cycles on average against the original positive-central schedule. Retaining the original negative central downdate after ten outer rotations saved 2,772.1 cycles versus the original negative-central schedule. These are differences between paired synthetic replays. The all-rotation negative-central result is a timing experiment. It does not resolve the previously observed positive-definiteness failures near the downdate boundary.

## A phase timing and margins

The absolute floor charges one cycle per arithmetic request plus modeled overhead. It gives a lower bound on the known work and does not describe a feasible valid operand sequence. The finite recurrence floor uses 19 cycles per MAC and 31 per div/root. Its scope is the schedule in which all operations take recurrence paths and each takes its individual minimum simultaneously. Specials may take less, and dependencies may prevent the recurrence floor from being attained. No complete-phase minimum was measured. The expected column combines measured synthetic kernel execution with a non-kernel latency proxy. The conservative known-work bound charges 19/22 cycles for each clear-start/continued MAC, 81 per division and 54 per root, plus the listed overhead. Average estimates are fractional cycles until rounded for presentation. For example, original prediction is bounded by `605×19 + 550×22 + 220×81 + 55×54 + 6,340 = 50,725` known cycles. Original correction is bounded by `179×19 + 274×22 + 140×81 + 30×54 + 3,505 = 25,894`.

| Scenario | Absolute floor | Finite recurrence floor | Expected known cycles | Expected margin | Conservative known bound | Conservative remaining cycles |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Prediction, original, positive central | 7,770 | 36,810 | 37,909 | 24,591 | 50,725 | 11,775 |
| Prediction, original, negative central | 7,770 | 36,810 | 37,914 | 24,586 | 50,725 | 11,775 |
| Prediction, all rotation, positive central | 7,935 | 34,005 | 34,879 | 27,621 | 39,670 | 22,830 |
| Prediction, all rotation, negative central (experimental) | 7,935 | 34,005 | 34,887 | 27,613 | 39,670 | 22,830 |
| Prediction, ten rotations and original negative central | 7,920 | 34,260 | 35,141 | 27,359 | 40,675 | 21,825 |
| Correction, original posterior | 4,128 | 17,382 | 18,225 | 44,275 | 25,894 | 36,606 |

For the original positive-central prediction the expected subtotal is 758.2 µs and the conservative known subtotal is 1,014.5 µs, at the assumed clock. Original correction is 364.5 µs expected and 517.9 µs conservative for known work. The all-primitive proxy, using the supplied benchmark means rather than the new measured kernels, is about 38,719 cycles for original prediction and 18,418 cycles for original correction. These are separate estimates of the same schedules.

For a conservative projected pass with A, the following inequalities must hold under the stated serialization:

- **Original prediction:** `conversion + 22×worst_sincos + 45×worst_wrap + additional_control <= 11,775` cycles.
- **All-rotation prediction:** the same unknown work must be at most **22,830 cycles**. For the hybrid negative-central policy the ceiling is **21,825 cycles**.
- **Correction:** `2×worst_wrap + additional_control <= 36,606` cycles.

The extra-control terms include integration costs beyond the modeled RF, launch, retirement and boundary charges. The unknowns are deliberately left unassigned. The conservative sums bound the modeled work, not the latency of an absent implementation.

## A, D and F comparison

Prediction uses the original positive-central policy in this table, and correction uses the original four posterior downdates. Phase cells show the hybrid expected subtotal and conservative known-work bound. Margins are budget minus subtotal, with the expected value followed by the conservative value. All figures exclude the same unknown operations.

| Metric | A: Radix-4 | D: 24×8 | F: Dadda |
| --- | ---: | ---: | ---: |
| MAC mean cycles, expanded workload | 19.188 | 9.950 | 8.103 |
| Prediction cycles, expected / bound | 37,909 / 50,725 | 26,459 / 39,175 | 24,169 / 36,865 |
| Correction cycles, expected / bound | 18,225 / 25,894 | 13,695 / 21,364 | 12,789 / 20,458 |
| Prediction timing margin, cycles | 24,591 / 11,775 | 36,041 / 23,325 | 38,331 / 25,635 |
| Correction timing margin, cycles | 44,275 / 36,606 | 48,805 / 41,136 | 49,711 / 42,042 |
| SKY130 cell area | Unmeasured | Unmeasured | Unmeasured |
| Critical-path delay | Unmeasured | Unmeasured | Unmeasured |
| Meets 50 MHz? | Unverified | Unverified | Unverified |

## Physical 50 MHz verification

Icarus ran the functional RTL plus the existing synthesis-elaboration checks. Its clock edges carry no mapped combinational delays. An Architecture A acceptance-to-completion measurement of 22 cycles establishes a **cycle** bound. The execution time of those cycles depends on 20 ns timing closure.

The SKY130 mapping frontend was invoked. Yosys, a SKY130 Liberty library, OpenSTA, OpenROAD and Tiny Tapeout tooling were unavailable. As a result, standard-cell area, critical-path delay, 20 ns slack, and tile utilization are unmeasured. A physical pass requires a common SKY130 library and corner, 20 ns clock and I/O constraints, and timing for the integrated register-file, control, and arithmetic paths. Tiny Tapeout fit also requires the chosen wrapper and routed utilization. [Tool availability](benchmarks/mac/results/arch_a_feasibility/physical/availability.json) · [Mapping flow](benchmarks/mac/synthesize.py)

## Recommendation and decision

**Use A with the original factor algorithm as the first minimal-area candidate to map.** It uses an iterative narrow Booth recurrence and avoids a wide parallel product fabric. Its original schedule has appreciable conditional headroom, and neither mapping data nor a verified tile fit supports an area ranking. If the missing work cannot fit the A margin, D is the next comparison point. It raises the conservative original prediction allowance from 11,775 to 23,325 cycles. F raises it to 25,635 cycles with a fully parallel multiplier. Those figures compare cycle reserves at the same assumed clock.

The circular update is an experimental way to recover about 3,030 cycles per prediction on the measured synthetic workload without replacing the MAC. Keep the original negative central downdate in a hybrid if pursuing that optimization. The previous boundary tests leave the hyperbolic replacement unsuitable for an unguarded switch.

**Can A complete both windows at a physically achievable 50 MHz? The complete-system result is unverified.** The covariance and posterior kernel replays passed, and both complete phase schedules project below 62,500 cycles for known work, including modeled controller and register traffic. A confirmed pass still needs the missing conversion, trig and wrap execution bounds, integrated sequencer measurements with realistic sensor streams, and SKY130 timing closure at 20 ns.

## Reproduction

From the hardware repository:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -u benchmarks/mac/arch_a_feasibility.py
make test synth-check
python3 benchmarks/mac/synthesize.py --out benchmarks/mac/results/arch_a_feasibility/physical
git diff HEAD --exit-code -- fp32_mac.sv fp32_divsqrt.sv
```

The replay writes a raw cycle log per MAC variant plus the machine-readable accounting. The existing regression passed with 10,112 MAC requests and 12,162 requests per division/square-root mode. The mapping command reported unavailable Yosys, and the production arithmetic diff is empty. [Replay data](benchmarks/mac/results/arch_a_feasibility/replay.json) · [Cycle accounting](benchmarks/mac/results/arch_a_feasibility/estimate.json) · [Regression output](benchmarks/mac/results/arch_a_feasibility/regression.log)

