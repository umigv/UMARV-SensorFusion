# FP32 MAC architecture comparison

The production [fp32_mac.sv](fp32_mac.sv) is the unchanged baseline. All alternatives preserve its IEEE binary32 fused accumulation, signed zero, canonical NaN, gradual underflow, five exception flags, and start/busy/done/last interface. The alternative tops and their common FP32 datapath are under [benchmarks/mac](benchmarks/mac). All operate on one in-flight accumulator.

## Implementations

| Top | Significand product | Product cycles |
| --- | --- | ---: |
| Baseline | Original one-bit shift and add with the original block-prefix adder | 24 |
| A · `fp32_mac_a` | Radix-4 Booth with signed digits 0, ±1, ±2 | 13 |
| B · `fp32_mac_b` | Radix-8 Booth with signed digits 0, ±1, ±2, ±3, ±4 and a registered triple multiplicand | 9 |
| C · `fp32_mac_c` | A 24×4 unsigned slice, reused in a Horner recurrence | 6 |
| D · `fp32_mac_d` | A 24×8 unsigned slice, reused in a Horner recurrence | 3 |
| E · `fp32_mac_e` | A 24×12 unsigned slice, reused in a Horner recurrence | 2 |
| F · `fp32_mac_f` | A 24-row unsigned Dadda compressor followed by the shared carry-propagate adder | 1 |
| G · `fp32_mac_g` | Radix-4 Booth recoding with 13 signed rows and Dadda compression, followed by the shared adder | 1 |

Unsigned 24-bit Booth operands require a zero-extended final digit. A and B use a narrow 28-bit Booth add/subtract step, keep a signed upper remainder, and commit two or three low bits per iteration. Their final zero-extended digit corrects the upper half, yielding the exact 48-bit unsigned product. The recurrence retains the quotient after removing its low digit modulo the radix. That digit contributes the next disjoint group of product bits. Even the radix-8 step adds a term of magnitude below 2²⁶ to a bounded upper remainder, so the signed 28-bit step does not wrap. The narrow Booth adder is separate from the configurable 52-bit FP32 adder. The chunked variants use a bounded-width integer multiply for each slice. The Dadda generator emits explicit full and half adders with descending target heights. F uses 483 full and 23 half adders. G uses 253 full and 23 half adders plus Booth row selection, negation and sign corrections. These are source structural counts before mapping.

The shared alternative core has a parameter for its 52-bit adder. The full MAC comparison uses the hybrid block ripple-prefix implementation, with 4-bit local carry chains and a prefix network across blocks. Independent adder verification passed 10,052 exact 52-bit vectors per topology, including directed carry chains, for ripple, Brent–Kung, Han–Carlson, Kogge–Stone, and hybrid. The synthesis script varies the adders on A to hold the rest of the core fixed.

| Shared adder | RTL carry network | Experiment A Icarus logic nodes |
| --- | --- | ---: |
| Ripple | One carry chain through bit 50 | 351 |
| Brent–Kung | Six prefixes up, five down | 636 |
| Han–Carlson | Pair first, scan the odd positions, fill even positions at the end | 801 |
| Kogge–Stone | Six doubling-distance prefix levels | 1,098 |
| Hybrid | Four prefix levels over 13 blocks and up to three local ripple steps | 579 |

Logic-node counts above include the surrounding A datapath and omit separate Icarus mux and arithmetic records. The counts are structural diagnostics. All adders have the same architectural cycle count at a given multiplier and shift setting.

### Normalization, alignment, and analytical latency

The fast mode uses hierarchical leading-zero detection and one-edge left shifts for product and accumulator normalization. Alignment subtracts the signed exponents, then shifts and jams the smaller significand through a six-stage barrel path. The right shift saturates to a sticky-only word for distances of at least 52. Repeated one-bit right jamming and direct barrel jamming produce the same bit pattern. The normalizer also handles cancellation and gradual underflow in one edge with exponent clamping. This replaces iterative control latency with combinational shift and selection logic, so millisecond estimates remain conditional on a 20 ns mapped clock.

Measure rising edges from start acceptance at edge 0 to `done`. Let `M` be the product cycles in the table. A classified shortcut completes in one cycle. Fast finite paths complete in `M+6` cycles with a zero addend, `M+8` with two nonzero terms and no extra negate cycle, `M+9` when negate is required, and `M+6` for exact cancellation. Product normalization, addend normalization, alignment, ALU, optional negate, sum normalization, rounding, packing and completion each take a single edge when visited. Exact zero bypasses rounding and packing. It cannot take the negate path. The worst fast path is therefore `M+9`.

With iterative shifting enabled, the old loop bounds apply because the multiplication produces the same normalized inputs to those loops. Product normalization can shift 47 times, accumulator normalization 23, and alignment 51. If the exponent gap is at least two, sum normalization is at most 23 shifts, and a negate adds at most one edge. If it is at most one, cancellation can take at most 50 left shifts, with only one alignment shift. The resulting nonzero-accumulator bound is `M+153`; the zero-accumulator bound is `M+104`. The unchanged baseline uses `M=24`. All units can accept again on the edge after completion, so their initiation interval is `L+1`, with a minimum of 2 cycles and a maximum one greater than the relevant bound.

## Benchmark method

The repository's FP32 MAC testbench and integer FMA oracle are reused. The runner copies the testbench into a temporary directory and increases its global timeout for this workload. The per-request timeout, handshake assertions, result bits, accumulator bits and flags are checked as before. A test-only wrapper supplies each experimental top under the same testbench module name. All nine configurations use identical vectors. The suite preserves the earlier timing workload, adds 100,000 independently generated target pairs under seed `0xDA44A`, includes explicit subnormal lanes and cancellation seeds, and finishes with the existing 1,024-term sequence. The previous worst request uses two minimum positive subnormals with a negative minimum-subnormal accumulator. The timed stream also includes special values, large exponent differences, mixed signs, overflow, and underflow.

Each saved `.cycles.gz` contains one integer latency per reference request in deterministic generation order. The summary retains the exact maximum witness, histograms, and normal-input subgroup statistics. The expanded workload has a different operand mix from the previously reported baseline mean of 42.394 cycles. The system estimate below uses the original workload subset for direct comparison.

### Measured latency

| Variant | Checked requests | Min | Mean | Median | P95 | P99 | Observed max | Proven bound | Max II |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Baseline | 135,113 | 1 | 53.197 | 57 | 86 | 88 | 177 | 177 | 178 |
| A | 135,113 | 1 | 19.188 | 21 | 22 | 22 | 22 | 22 | 23 |
| B | 135,113 | 1 | 15.493 | 17 | 18 | 18 | 18 | 18 | 19 |
| C | 135,113 | 1 | 12.722 | 14 | 15 | 15 | 15 | 15 | 16 |
| D | 135,113 | 1 | 9.950 | 11 | 12 | 12 | 12 | 12 | 13 |
| E | 135,113 | 1 | 9.027 | 10 | 11 | 11 | 11 | 11 | 12 |
| F | 135,113 | 1 | 8.103 | 9 | 10 | 10 | 10 | 10 | 11 |
| G | 135,113 | 1 | 8.103 | 9 | 10 | 10 | 10 | 10 | 11 |
| A, iterative shifts | 135,113 | 1 | 43.036 | 46 | 75 | 77 | 166 | 166 | 167 |

Every recorded command passed the bit-exact result, accumulator and flag checks. The maxima were attained in simulation. The previous 177-cycle request, `a=b=0x00000001` with accumulator `0x80000001`, still attains the bound in each design. The fast-shift variants first attain their bounds on a normal near-cancellation case with `a=0x3f800001`, `b=0x3f7ffffe`, and accumulator `0xbf800000`. For A, switching from iterative shifts to the leading-zero and barrel paths reduced the same workload mean from 43.036 to 19.188 cycles and the worst bound from 166 to 22. F and G have the same state schedule and their per-request latencies agree throughout this run.


## ASIC synthesis and physical measurements

Icarus synthesis elaboration ran on all seven alternatives, the baseline, the iterative-shift A ablation and the four additional adder settings. The [structural output](benchmarks/mac/results/iverilog_structure.json) is an unpruned Icarus netlist inventory. It exposes, for example, the distinct 24×4/8/12 multiplication operators and the explicit compressor networks. Icarus counts some declared registers even when their outputs are unused. A vector gate, inferred multiplier, mux and register group each have different mapping costs, so these node counts cannot establish a standard-cell area ranking.

The [SKY130 synthesis script](benchmarks/mac/synthesize.py) was invoked. Its [availability record](benchmarks/mac/results/synthesis/availability.json) shows that Yosys, a SKY130 Liberty library, OpenSTA, OpenROAD and Tiny Tapeout tooling were unavailable. A package download attempt failed because the package host did not resolve. The script applies the same mapping flow and 20 ns clock to each top when the tools and library are supplied, separates mapped combinational and sequential cell area, and saves the OpenSTA path, slack, and minimum-period reports. That STA flow has no routed parasitics.

| Physical metric | Baseline and all alternatives |
| --- | --- |
| Mapped standard-cell count, combinational/sequential/total area | Unmeasured |
| Critical path delay, maximum frequency, 20 ns slack | Unmeasured |
| Routed congestion and Tiny Tapeout tile usage | Unmeasured |

The Icarus diagnostic reports 4,143 simple logic nodes for F and 2,588 for G, alongside 85 versus 124 separate mux records and 14 versus 27 arithmetic records. The Dadda reduction count and these unweighted diagnostics motivate mapping G against F. The opaque slice multipliers in C–E make those node counts unsuitable for area comparison with F/G.

No verified area–latency Pareto frontier is available. F and G share the best verified *cycle* bound, while G uses fewer Dadda compressors and adds Booth logic. D and E offer intermediate cycle counts with a reused slice. Those facts identify candidates for mapping, but they do not determine either area or time per operation. A Tiny Tapeout fit also depends on integrating a suitable top-level I/O wrapper, physical utilization and routing.

| Area budget | Fastest verified design |
| --- | --- |
| 1 tile | Undetermined, no routed fit |
| 2 tiles | Undetermined, no routed fit |
| 4 tiles | Undetermined, no routed fit |
| 8 tiles | Undetermined, no routed fit |

## SR-UKF schedule model

Prediction uses the two-pass five-state schedule from [FP32_TIMING_ANALYSIS.md](FP32_TIMING_ANALYSIS.md) with the correction in [CHOLESKY_OPTIMIZATION_ANALYSIS.md](CHOLESKY_OPTIMIZATION_ANALYSIS.md). Its original dense Cholesky algorithm has 20 divisions per rank-one operation. The result is 605 clear-start and 550 continued MAC requests, 220 divisions, 55 roots, and 6,340 serialized register-file and control cycles. Unknowns are 22 sine/cosine pairs, 45 wraps, and one integer conversion. The model changes only the MAC architecture.

No active FP32 correction sequencer exists in the current tree. The correction model below is a serialized FP32 translation of the archived five-state `srukf_bias_filter.sv` at `fixed-point-baseline`. It assumes four observations `H=[I₄ 0]`, valid input and pivots, no stalls beyond the stated model, and the original dense posterior downdate.

| Correction stage | Clear starts | Continued MACs | Divisions | Roots |
| --- | ---: | ---: | ---: | ---: |
| Four innovations, each external subtraction seeded by `value*1` | 4 | 4 | 0 | 0 |
| Innovation QR | 30 | 30 | 20 | 10 |
| Five forward triangular solves | 20 | 90 | 20 | 0 |
| Five backward triangular solves | 20 | 30 | 20 | 0 |
| Five state corrections, seed state then accumulate four gain products | 5 | 20 | 0 | 0 |
| Four posterior downdates | 100 | 100 | 80 | 20 |
| **Total** | **179** | **274** | **140** | **30** |

The first four innovation rows copy the predicted upper block into the empty factor. The four subsequent measurement-noise rows have at most 4, 3, 2 and 1 active pivots. Their 10 pivots and 10 trailing pairs cost two MACs, two divisions and one root per pivot and four MACs per trailing pair. The forward solve follows the archived four cross-covariance terms per element, then subtracts 0–3 solved terms. The backward solve seeds each element from the forward result and accumulates 0–3 solved terms. The archived state ordering requires two wraps, one for the heading innovation and one for the posterior heading.

Correction conservatively charges five cycles per arithmetic command for two operand reads, one write, issue and retire. In addition, it charges four cycles each for 95 moves (20 innovation inputs, 10 initial factor entries, 10 innovation pivot writes, 15 posterior initialization entries, 20 downdate-vector words, and 20 posterior pivot writes), four overhead cycles per wrap, and two boundary cycles. This totals 3,505 controller/RF cycles and excludes the unknown execution time of the two wraps.

Neither schedule credits overlap. Independent commands could be reordered with a real sequencer, but accumulator-dependent MACs require the previous update and the next idle acceptance edge. Issue and retirement are already charged, so adding the initiation interval separately would count an issue boundary twice. Division and root share one unit. The proxy uses measured normal-operand MAC means from the original timing workload subset, 33.771 cycles for normal-operand division and 31 cycles for positive normal root. The conservative known-work subtotal uses each MAC's appropriate zero-addend or general bound and 81/54 cycles per division/root. Both calculations are at a hypothetical 50 MHz.

### Budget results

| Variant | Prediction proxy (cycles / µs) | Prediction bound subtotal | Prediction proxy room | Correction proxy (cycles / µs) | Correction bound subtotal | Correction proxy room |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Baseline | 71,338 / 1,426.8 | 201,920 | -8,838 | 33,125 / 662.5 | 87,875 | 29,375 |
| A | 38,712 / 774.2 | 50,725 | 23,788 | 18,414 / 368.3 | 25,894 | 44,086 |
| B | 34,092 / 681.8 | 46,105 | 28,408 | 16,602 / 332.0 | 24,082 | 45,898 |
| C | 30,627 / 612.5 | 42,640 | 31,873 | 15,243 / 304.9 | 22,723 | 47,257 |
| D | 27,162 / 543.2 | 39,175 | 35,338 | 13,884 / 277.7 | 21,364 | 48,616 |
| E | 26,007 / 520.1 | 38,020 | 36,493 | 13,431 / 268.6 | 20,911 | 49,069 |
| F | 24,852 / 497.0 | 36,865 | 37,648 | 12,978 / 259.6 | 20,458 | 49,522 |
| G | 24,852 / 497.0 | 36,865 | 37,648 | 12,978 / 259.6 | 20,458 | 49,522 |
| A, iterative shifts | 58,633 / 1,172.7 | 189,215 | 3,867 | 28,142 / 562.8 | 82,892 | 34,358 |

The baseline prediction proxy exceeds 62,500 cycles before unimplemented work. A with iterative shifts fits that proxy subtotal but exceeds the window under the conservative arithmetic bound. All seven fast-shift designs fit the **known-work subtotal** of both windows using the conservative primitive bounds. In prediction, A leaves 11,775 cycles in that bound and F/G leave 25,635. In correction, the baseline and iterative-shift A exceed the budget in the bound calculation. Every fast-shift correction subtotal leaves room for the two wraps under the stated model. A full deadline pass still requires the missing execution latencies, an integrated sequencer and physical timing closure.

At the primitive level, using acceptance-to-completion bounds, no variant meets 6 cycles. F/G meet 10 cycles. C through G meet 15 cycles. B through G meet 20 cycles. All seven fast-shift variants meet 30 cycles. Add one cycle to use an acceptance-to-acceptance budget.

## Reproduction and artifacts

Run from the hardware repository. The scripts save data in `benchmarks/mac/results`.

```sh
python3 benchmarks/mac/generate_dadda.py
python3 benchmarks/mac/benchmark.py --jobs 9
python3 benchmarks/mac/benchmark.py --collect
python3 benchmarks/mac/structure.py
python3 benchmarks/mac/schedule.py
git diff HEAD --exit-code -- fp32_mac.sv
```

For the independent topology verification:

```sh
adder_run=$(mktemp -d)
iverilog -g2012 -Wall -s test_adders -Ptest_adders.VECTORS=10000 \
    -o "$adder_run/sim" benchmarks/mac/mac_adders.sv benchmarks/mac/test_adders.sv
vvp "$adder_run/sim"
```

For SKY130 mapping and pre-route timing with a suitable library and tool installation:

```sh
python3 benchmarks/mac/synthesize.py --liberty /path/to/sky130_tt.lib
```

Saved files include [latency summaries](benchmarks/mac/results/summary.json), [edge-count logs](benchmarks/mac/results), [schedule calculations](benchmarks/mac/results/schedule.json), and [Icarus structural data](benchmarks/mac/results/iverilog_structure.json). [Simulation pass logs](benchmarks/mac/results/verification.log) cover the unchanged baseline and every final configuration. The earlier wide-remainder Booth prototype exceeded the simulator wall-clock limit and was superseded by the narrow recurrence. Both final Booth designs completed the full workload. C–G were rerun on the final shared source, and every saved latency matched the earlier run. [Stream digests](benchmarks/mac/results/latency_sha256.txt) refer to the decompressed cycle files. The existing regression also passed under `make test synth-check`.
