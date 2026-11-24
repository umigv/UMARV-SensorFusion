# FP32 arithmetic latency and SR-UKF prediction budget

At the target 50 MHz, one cycle is 20 ns and each 1.25 ms prediction or correction window contains 62,500 cycles. This report measures the arithmetic RTL and constructs a serialized prediction schedule. It does not establish 20 ns physical timing closure or integrated filter latency.

The active design contains [fp32_mac.sv](fp32_mac.sv) and [fp32_divsqrt.sv](fp32_divsqrt.sv). The testbenches are [tb_fp32_mac.sv](tests/tb_fp32_mac.sv) and [tb_fp32_divsqrt.sv](tests/tb_fp32_divsqrt.sv). There is no active FP32 microsequencer, register file, conversion, wrap, or trigonometry RTL in this tree. The former scheduler and five-state fixed-point datapath can be inspected with `git show fixed-point-baseline:srukf_400hz_scheduler.sv` and `git show fixed-point-baseline:srukf_bias_filter.sv`. The archived filter builds a factor from zero with streamed QR, explicit process-noise rows, and an optional wheel-noise row, then applies its central update. It also has a wheel lever-arm term. The schedule below uses the supplied motion equations and initializes the running factor to `S_Q` as requested.

## Measurement method

The rising edge accepting `start` with `busy=0` is edge zero. The testbench counts rising edges and samples outputs on falling edges after the RTL's registered outputs have settled. The reported latency is the completion-edge number minus the acceptance-edge number. The testbenches run with a 20 ns clock, require idle before issuing, assert `start` for one sampling edge in timing mode, check the completion against the numerical oracle, and fail on a 256-cycle per-request timeout. Consecutive requests are accepted on the first rising edge after completion, so the primitive acceptance-to-acceptance interval is `L+1`. The ordinary correctness run still tests competing starts while busy, reset recovery, and result/flag behavior.

The latency workload includes 10,000 seeded finite nonzero MAC operand pairs, a separate 1,024 near-cancellation vector generator and its seed operations, a long accumulation, and directed exceptional inputs. Division and square root each use 12,000 seeded valid randomized requests and 162 directed requests. MAC result and flag checks use the pre-existing exact-integer FMA oracle, including a per-operation accumulator check even when `last=0`. Division and square root compare their result bits and five flags against the existing native binary32 C reference. Percentiles below use nearest rank `ceil(p*N)`. Group membership overlaps and the means describe these workloads.

## FSM derivation

### MAC

| State after acceptance | Cycles |
| --- | ---: |
| `MULTIPLY` | 24 serial steps |
| `NORMALIZE_PRODUCT` | `p+1`, up to 47 left shifts and one exit |
| `NORMALIZE_ACC` | `c+1`, up to 23 left shifts and one exit. A zero addend exits in one cycle. |
| `ALIGN` | `d+1` if the addend is nonzero. `d = min(exponent gap, 51)`. |
| `ADD`, `NEGATE_SUM` | One add/subtract edge and one optional negate edge, both bypassed with a zero addend |
| `NORMALIZE_SUM` | `n+1` including the exit, or one edge for exact zero |
| `ROUND`, `PACK`, `COMPLETE` | One each. Exact zero bypasses round and pack. |
| Special input shortcut | `COMPLETE` after one edge |

The product starts with its least possible set bit at position 3 and normalizes to bit 50, giving `p<=47`. The accumulator starts with its least possible set bit at position 27, giving `c<=23`. Alignment moves the smaller normalized significand right with jamming. After 50 shifts only bit 0 can remain. If exponent equality has yet to occur, one more edge jumps to the larger exponent. The aligned sum's exponent is the larger of the two normalized exponents.

For a nonzero result with a zero addend, `L = 30 + p + n`. The normalized product can require at most 51 right-shift iterations including the sticky-only exponent jump. Thus `L<=128`, attained by the product of two minimum subnormals with a cleared accumulator. For two nonzero terms and a nonzero result, `L = 32 + p + c + d + u + n`, where `u` is the optional negate cycle. For exact cancellation, omit the two rounding/packing cycles and take `n=0`.

- For an exponent gap of at least two, the smaller aligned term is below bit 49 and the larger term has bit 50 set. Subtraction can require at most one left normalization shift. The larger exponent is at least −149 because one term is an FP32 addend. Right normalization to the minimum normal exponent therefore takes at most 23 iterations, including any carry-clearing shift. A carry with exponent at least −126 takes one shift. These right-shift paths cannot then enter left normalization. Consequently `n<=23` and `L<=32+47+23+51+1+23=177`.
- For an exponent gap of zero or one, even a difference with only bit 0 set takes at most 50 left shifts. Right normalization still takes at most 23 shifts. Using the larger bound gives `L<=32+47+23+1+1+50=154`. The exact-zero shortcut is shorter.

The **177-cycle bound is attained** with `a=b=0x00000001` and `c=0x80000001`. With the positive addend `0x00000001`, the same path takes 176 cycles without negation. The earlier 176-cycle observation was a sample maximum, and the directed opposite-sign case disproves it as a bound. The addend in both cases is loaded legally by a separate clear-and-multiply before the measured request.

### Division and square root

| State after acceptance | Cycles |
| --- | ---: |
| `NORMALIZE` | `s+1` including exit. Division shifts both operands concurrently, so `s=max(s_a,s_b)`. Square root shifts only `a`. Nonzero subnormals require at most 23 shifts. |
| `SEED` | 1. Division compares and optionally doubles the normalized numerator on this edge. Square root selects exponent parity and seeds the radicand stream. |
| `DIGIT` | 26 radix-2 iterations |
| `FINISH_DIGITS` | 1 |
| `DENORMALIZE` | `t+1` including exit. Division can shift or jam from bit 26 to bit 0 in 26 edges and jump the exponent on the next edge. |
| `ROUND` | 1, asserts `done` |
| `SPECIAL` | One-edge alternative that asserts `done` |

The recurrence formula is `L=31+s+t`. There is no separate exponent alignment loop. For division, `t=min(max(0,-126-E),27)` with `E` equal to the quotient exponent after seeding. Both maxima occur for minimum positive subnormal divided by maximum finite, yielding **81 cycles**. All positive nonzero FP32 square roots are normal, so `t=0`. Minimum positive subnormal takes 23 input shifts, attaining **54 cycles**. All three operations have a one-cycle special-input path.

## Measured results

| Group | N | Min | Mean | Median | P95 | P99 | Max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| MAC all / consecutive | 15112 | 1 | 42.394 | 41 | 84 | 85 | 177 |
| MAC normal operands, zero accumulator | 2153 | 30 | 31.596 | 31 | 31 | 75 | 82 |
| MAC normal operands, normal nonzero accumulator | 8229 | 31 | 66.813 | 74 | 85 | 85 | 86 |
| MAC normal operands, both positive | 4025 | 30 | 53.183 | 42 | 84 | 85 | 86 |
| MAC normal operands, at least one negative | 6357 | 30 | 63.516 | 70 | 85 | 85 | 86 |
| MAC exponent gap >= 50 | 3864 | 82 | 84.181 | 84 | 85 | 96 | 177 |
| MAC near cancellation | 1026 | 33 | 58.843 | 59 | 62 | 64 | 81 |
| MAC exact cancellation | 9 | 31 | 41.222 | 31 | 77 | 77 | 77 |
| MAC subnormal operand or accumulator | 453 | 1 | 40.951 | 1 | 107 | 153 | 177 |
| MAC subnormal result | 163 | 1 | 67.331 | 77 | 81 | 176 | 177 |
| MAC overflow | 443 | 30 | 76.521 | 83 | 84 | 84 | 106 |
| MAC underflow | 68 | 34 | 79.662 | 78 | 153 | 177 | 177 |
| MAC NaN / infinity input | 4192 | 1 | 1.000 | 1 | 1 | 1 | 1 |
| MAC zero multiplier input | 565 | 1 | 1.000 | 1 | 1 | 1 | 1 |
| Division all | 12162 | 1 | 33.792 | 31 | 58 | 58 | 81 |
| Division recurrence | 12054 | 31 | 34.086 | 31 | 58 | 58 | 81 |
| Division normal operands | 10440 | 31 | 33.771 | 31 | 58 | 58 | 58 |
| Square root all | 12162 | 1 | 31.045 | 31 | 33 | 35 | 54 |
| Square root recurrence | 12065 | 31 | 31.286 | 31 | 33 | 35 | 54 |
| Square root positive normal | 10500 | 31 | 31.000 | 31 | 31 | 31 | 31 |

“Normal operands” classifies input words. For division and MAC it can include an underflowing or overflowing result. The MAC near-cancellation group has a nonzero exact residual no greater than 1/1,024 of the product magnitude. The all/consecutive group verifies every inter-request interval, including the 1,024-term accumulation. For example, its final `1×1+1023` takes 43 cycles and the unit could accept again on edge 44.

### Directed observations

| MAC case | a | b | Pre-operation addend c | Result | Flags | Cycles |
| --- | --- | --- | --- | --- | --- | ---: |
| Normal multiply | 3f800000 | 3f800000 | 00000000 | 3f800000 | 00 | 31 |
| Negative multiply | bf800000 | 3f800000 | 00000000 | bf800000 | 00 | 31 |
| Normal FMA at a rounding tie | 33800000 | 3f800000 | 3f800000 | 3f800000 | 01 | 57 |
| Near cancellation | 3f800001 | 3f7ffffe | bf800000 | a8800000 | 00 | 81 |
| Exact cancellation | 3f800000 | 3f800000 | bf800000 | 00000000 | 00 | 31 |
| Large exponent gap | 00000001 | 00000001 | 7f7fffff | 7f7fffff | 01 | 130 |
| Smallest exact subnormal | 00000001 | 3f800000 | 00000000 | 00000001 | 00 | 77 |
| Underflow to zero | 00000001 | 3f000000 | 00000000 | 00000000 | 03 | 78 |
| Overflow | 73000000 | 3f800000 | 7f7fffff | 7f800000 | 05 | 57 |
| Negative zero multiply/add | 80000000 | 3f800000 | 80000000 | 80000000 | 00 | 1 |
| Same-sign worst path | 00000001 | 00000001 | 00000001 | 00000001 | 03 | 176 |
| Opposite-sign worst path | 00000001 | 00000001 | 80000001 | 80000001 | 03 | 177 |

Words are hexadecimal binary32 encodings. Flags are hexadecimal in the order invalid, divide by zero, overflow, underflow, inexact from most to least significant bit. The MAC NaN/infinity input group and zero operand group both complete in one cycle.

| Operation | a | b | Result | Flags | Cycles |
| --- | --- | --- | --- | --- | ---: |
| DIV | 3f800000 | 3f800000 | 3f800000 | 00 | 31 |
| DIV | 3f800000 | 40400000 | 3eaaaaab | 01 | 31 |
| DIV | 7f7fffff | 00000001 | 7f800000 | 05 | 54 |
| DIV | 00000001 | 7f7fffff | 00000000 | 03 | 81 |
| DIV | bf800000 | 40400000 | beaaaaab | 01 | 31 |
| DIV | 00000001 | 40000000 | 00000000 | 03 | 78 |
| DIV | 3f800000 | 00000001 | 7f800000 | 05 | 54 |
| DIV | 00000001 | 00000001 | 3f800000 | 00 | 54 |
| DIV | 00800000 | 40000000 | 00400000 | 00 | 32 |
| DIV | 00000000 | 3f800000 | 00000000 | 00 | 1 |
| DIV | 7f800000 | 3f800000 | 7f800000 | 00 | 1 |
| DIV | 3f800000 | 7f800000 | 00000000 | 00 | 1 |
| DIV | 3f800000 | 80000000 | ff800000 | 08 | 1 |
| DIV | 00000000 | 00000000 | 7fc00000 | 10 | 1 |
| DIV | 7f800000 | 7f800000 | 7fc00000 | 10 | 1 |
| DIV | 7fc12345 | 3f800000 | 7fc00000 | 00 | 1 |
| SQRT | 3f800000 | — | 3f800000 | 00 | 31 |
| SQRT | 40000000 | — | 3fb504f3 | 01 | 31 |
| SQRT | 40800000 | — | 40000000 | 00 | 31 |
| SQRT | 00000000 | — | 00000000 | 00 | 1 |
| SQRT | 80000000 | — | 80000000 | 00 | 1 |
| SQRT | 7f7fffff | — | 5f7fffff | 01 | 31 |
| SQRT | 00800000 | — | 20000000 | 00 | 31 |
| SQRT | 00000001 | — | 1a3504f3 | 01 | 54 |
| SQRT | 00000002 | — | 1a800000 | 00 | 53 |
| SQRT | 00000003 | — | 1a9cc471 | 01 | 53 |
| SQRT | 00000004 | — | 1ab504f3 | 01 | 52 |
| SQRT | 007fffff | — | 1fffffff | 01 | 32 |
| SQRT | bf800000 | — | 7fc00000 | 10 | 1 |
| SQRT | 7f800000 | — | 7f800000 | 00 | 1 |
| SQRT | ff800000 | — | 7fc00000 | 10 | 1 |
| SQRT | 7fc12345 | — | 7fc00000 | 00 | 1 |
| SQRT | ff812345 | — | 7fc00000 | 10 | 1 |

## Five-state prediction instruction ledger

This is an exact count for the following planned, dense instruction sequence. The historical sequencer uses different arithmetic, and no compiled FP32 instruction trace exists yet. Assume:

- One signed encoder count converts to FP32 and is multiplied by a configured meters-per-second-per-count constant that incorporates the sample period. The formulas use the resulting `v_wheel` directly. The archived lever arm and extra wheel-noise row are outside this model. Additional wheel noise would require a specified model and an extra update.
- The five-dimensional scaled UT uses the historical sigma scale `γ=2` and fixed weights. The mean weights and square roots of absolute covariance weights are available as FP32 constants. Heading differences use a consistent local lift. Wrap each midpoint before sine/cosine, wrap each propagated endpoint as required by the model while retaining its lift, and wrap the final mean heading. Each trig command supplies a sine/cosine pair.
- Seed the running upper factor with all 15 entries of the supplied `S_Q`. Process ten outer deviations and one central deviation. Count a dense loop including every pivot and trailing entry and both divisions at the final pivot. The signed central covariance update/downdate has the same operation count. This schedule models successful execution with valid pivots. An error policy for invalid pivots and FP32 faults needs its own control implementation.
- For an FMA `a*b+c` with external `c`, first execute `clear, c*1`, then execute `a*b` into the running MAC accumulator. Use `last` only on a result that is consumed outside the accumulator. A standalone `x±y` similarly uses `clear, x*1`, then `(±y)*1`. Sign changes are operand sign-bit selection. A product or first dot term takes one clear-and-start, with no independent clear instruction. Rounding occurs at each MAC completion.

| Stage | Planned sequence | Clear-start MACs | Continued MACs | Standalone FP32 add/sub¹ | Division | Sqrt | Sin/cos pairs | Wrap | Int→FP32 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Wheel conversion | Convert one count, multiply by scale | 1 | 0 | 0 | 0 | 0 | 0 | 0 | 1 |
| Sigma points, both passes | Per pass, two signs for each of 15 stored triangular entries. Seed center component, then accumulate signed `γ*S`. Central point and structurally untouched components alias the center. | 60 | 60 | 0 | 0 | 0 | 0 | 0 | 0 |
| Propagation, 22 points | Per point, compute `v=v_wheel-b_v`, `d=v*dt`, `t=ω*dt`, two-term midpoint and endpoint, and two-term x/y updates. | 154 | 110 | 44 | 0 | 0 | 22 | 44 | 0 |
| Weighted mean | Five 11-term dots from retained first-pass values, then wrap mean heading | 5 | 50 | 0 | 0 | 0 | 0 | 1 | 0 |
| Weighted deviations | For each of 55 components, subtract the mean, then multiply by the configured covariance weight root | 110 | 55 | 55 | 0 | 0 | 0 | 0 | 0 |
| Streaming rank-one factor | 11 rows, five pivots and ten trailing entries per row | 275 | 275 | 0 | 165 | 55 | 0 | 0 | 0 |
| **Total** | | **605** | **550** | **99** | **165** | **55** | **22** | **45** | **1** |

¹ The 99 standalone add/sub expressions are already implemented inside the **1,155 MAC instructions** counted in the two MAC columns. There are 451 other continued MAC instructions carrying product-plus/minus-accumulator terms. The 605 clear-start instructions comprise 335 accumulator seeds and 270 product starts. There is no separate FP32 adder instruction in this schedule.

The two-pass propagation uses seven clear-start and five continued MACs per point. In order, the clear/continue counts are `v:1/1, d:1/0, t:1/0, midpoint:1/1, endpoint:1/1, x:1/1, y:1/1`. Both x and y updates seed the original position before the fused term. Store the first-pass propagated values or their stable aliases for the five mean dots. The second pass can process one weighted deviation at a time.

With `σ=+1` for update and `σ=−1` for downdate, use the following row recurrence at each pivot `k`.

1. Use two MAC instructions for `z=Rkk²+σ*w_k²` and one square root for `r=sqrt(z)`.
2. Use two divisions for `c=r/Rkk` and `s=w_k/Rkk`. Write `r` to the diagonal once the old value is no longer needed.
3. For each `j>k`, seed `Rkj`, accumulate `σ*s*w_j`, and divide by `c` to form the new `Rkj`. Then multiply `c*w_j` and accumulate `-s*new_Rkj` to form the new `w_j`.

For five pivots and ten trailing entries this is `5×2+10×4=50` MACs, `5×2+10=15` divisions and five roots per row. The covariance factor alone consumes 550 MACs, 165 divisions and 55 roots. This planned arithmetic is separate from a batch QR implementation.

### Register accesses and sequencing model

Count a read for each variable operand, with one read fanned out for a square. Constants and sign selection need no register-file access. The MAC accumulator holds intermediate terms internally. Count each externally published temporary as a write. Alias unchanged sigma or propagated components, and retain the first pass until the mean is available. Each wrap reads and writes one word. Each paired trig command reads one word and writes two. Under this explicit storage policy:

| Stage | Reads | Writes |
| --- | ---: | ---: |
| Wheel conversion and scaling | 2 | 2 |
| Sigma points | 120 | 60 |
| Propagation arithmetic, before wrap/trig traffic | 308 | 154 |
| Mean dots | 55 | 5 |
| Weighted deviations | 165 | 110 |
| Factor recurrence and diagonal writes | 1,430 | 605 |
| Initial factor copy | 15 | 15 |
| Wrap and trig traffic | 67 | 89 |
| **Total** | **2,162** | **1,040** |

The factor recurrence uses `7R/4W` per pivot for z, sqrt, and the two ratios, `9R/3W` per trailing entry, and `1R/1W` per diagonal copy after the ratios. Its 55 pivots and 110 trailing entries give `1,430R/605W`. The factor and committed-state bank publication is one bank-selection action. The register file and banking assumptions are part of this schedule.

Model a fully serialized single-port register file with one cycle per word access and no overlap with execution. Charge two controller cycles per command for issue and retirement, inclusive of instruction fetch on the issue edge in this model. Charge the same for each move, and one cycle each for prediction entry and bank publication. A statically unrolled schedule removes runtime loop-branch instructions. The accounting is:

- 1,375 arithmetic commands, plus 68 conversion/wrap/trig commands and 70 moves (55 diagonal, 15 initialization).
- Controller overhead `2×(1375+68+70)+2 = 3,028` cycles.
- Serialized register-file overhead `2162+1040 = 3,202` cycles.
- Total modeled overhead **6,230 cycles**, separate from arithmetic acceptance-to-done cycles. It includes the issue/retire boundaries for the three unimplemented units. A multi-cycle instruction fetch, pipeline bubbles, or RF arbitration would need additional measured terms.

### Prediction cycle budget at 50 MHz

Let `U` be the sum of the unknown acceptance-to-completion latencies of one integer conversion, 22 trig pairs, and 45 wraps. The table is the known arithmetic subtotal plus the modeled sequencing and RF overhead. Add `U` cycles, `0.02×U` µs, and `U/625` percentage points to every row.

| Scenario | Clear / continue MAC latency | Div / sqrt latency | Cycles before U | Time before U (µs) | Prediction budget before U |
| --- | --- | --- | ---: | ---: | ---: |
| Optimistic finite-path floor | 30 / 30 | 31 / 31 | 47,700 | 954.0 | 76.32% |
| Typical proxy using these test distributions | 31.596 / 66.813 | 33.771 / 31.000 | 69,370 | 1,387.4 | 110.99% |
| Conservative arithmetic bound | 128 / 177 | 81 / 54 | 197,355 | 3,947.1 | 315.77% |

The finite-path floor assumes recurrence executions and gives every instruction a favorable latency simultaneously. It is a floor, not an observed complete prediction. Treating every operation as a one-cycle shortcut would give an even looser mathematical floor of 7,605 cycles before `U`, with no claim that a valid prediction can realize it. The typical proxy uses the normal operand and normal accumulator groups where applicable. It includes deliberately broad exponent distributions and cancellation tests, so a sensor trace and the actual instruction stream are needed for an application-typical latency. The worst-case row uses independent primitive maxima even when they cannot co-occur in this filter. It bounds the identified arithmetic and modeled overhead only. The best finite-path row leaves **14,800 cycles** for `U` and any scheduling differences. The typical proxy exceeds the budget even before those terms. No complete prediction or correction deadline is established.

### Potential cycle savings

Ranked by the change supported by this ledger, with data-dependent and unknown savings identified separately.

1. **Reduce covariance rows when the noise model permits.** This plan already seeds `R=S_Q` and skips five explicit process-noise rows. Against a version of the *same dense kernel* that processed those rows, this avoids 250 MACs, 75 divisions and 25 roots, offset by the factor initialization. Streaming also holds a 15-word factor plus one five-word row rather than the batch matrix. The archived QR path has different operations and short-circuits, so these counts do not quantify a speedup over that RTL.
2. **Remove accumulator seed transactions through an accumulator-load feature or a dedicated add datapath.** There are 335 seeds. At the measured clear-start proxy latency, eliminating those requests would remove about 10,600 arithmetic cycles before accounting for replacement load traffic. Under the current interface the plan already fuses product-plus/minus terms, and independent accumulation streams still require explicit seeds.
3. **Eliminate dead arithmetic and memory handoffs.** The final pivot of each covariance row has no trailing entries, so its two ratios are unused. Removing 22 divisions would save about 743 arithmetic cycles at the division proxy, plus their issue and memory traffic. Test and skip a zero pivot-vector component only with the required pivot and error checks. For a forwarded published temporary, avoiding its write and later read saves two serialized RF cycles. If wrapped endpoints are never consumed, using their lifted values internally can eliminate 22 of the 45 wraps. The wrap cycle benefit is unknown.
4. **Consider reciprocal reuse per pivot only with a measured break-even.** For each pivot with trailing entries, replacing direct division with `1/Rkk` and `1/c` reduces the count by the number of trailing entries. Across 11 rows it removes 110 divisions and adds 198 clear-start multiplies under the current interface. The measured proxy arithmetic cost increases by about 2,540 cycles before RF effects. Faster products or reuse beyond one pivot could change the trade-off. Separate reciprocal rounding changes numerical results and needs validation.

## Physical timing

The 50 MHz conversion assumes a 20 ns period. The RTL measurements do not include mapped cell or wire delay. Yosys, a SKY130 Liberty library, and a static timing tool were unavailable, so 20 ns closure is **unverified**. This applies to the MAC carry-lookahead path and the shared division/root datapath. No arithmetic RTL was changed for this analysis.

## Reproduction

Run from `UMARV-SensorFusion`. The reported run used Icarus Verilog 12.0 and GCC 15.2 on revision `bb1c7a8` with the measurement-only test changes. The full timing tables and directed rows above come from the first command. Both commands exited successfully with no arithmetic mismatch or stalled request.

```sh
PYTHONDONTWRITEBYTECODE=1 make timing
make test synth-check
git diff --exit-code -- fp32_mac.sv fp32_divsqrt.sv
```

Timing command output through the pass counters, with its detailed rows reproduced above:

```text
CC="cc" IVERILOG="iverilog" python3 tests/fp32_timing.py
PASS: 15112 reference MAC operations, max sampled latency 177 cycles
tests/tb_fp32_mac.sv:135: $finish called at 13121420000 (1ps)
PASS mode 0: 12162 results and flags, latency min/max/mean 1/81/33.792 cycles
     recurrence only: 12054 operations, min/max/mean 31/81/34.086 cycles
PASS mode 1: 12162 results and flags, latency min/max/mean 1/54/31.045 cycles
     recurrence only: 12065 operations, min/max/mean 31/54/31.286 cycles
tests/tb_fp32_divsqrt.sv:146: $finish called at 16263080000 (1ps)
Group | N | Min | Mean | Median | P95 | P99 | Max
```

Correctness and synthesis-elaboration output:

```text
CC="cc" IVERILOG="iverilog" python3 tests/test_fp32_divsqrt.py
PASS mode 0: 12162 results and flags, latency min/max/mean 1/81/33.792 cycles
     recurrence only: 12054 operations, min/max/mean 31/81/34.086 cycles
PASS mode 1: 12162 results and flags, latency min/max/mean 1/54/31.045 cycles
     recurrence only: 12065 operations, min/max/mean 31/54/31.286 cycles
tests/tb_fp32_divsqrt.sv:146: $finish called at 16263080000 (1ps)
python3 tests/test_fp32_mac.py
PASS: 10112 reference MAC operations, max sampled latency 177 cycles
tests/tb_fp32_mac.sv:135: $finish called at 8264420000 (1ps)
iverilog -g2012 -Wall -S -tnull -s fp32_divsqrt fp32_divsqrt.sv
iverilog -g2012 -Wall -S -tnull -s fp32_mac fp32_mac.sv
```

The RTL diff check returned no output.
