# FP32 Cholesky update and downdate benchmark

The experiment compares the original ratio recurrence (A) with reciprocal-based rotations (B) and a binary64 covariance and Cholesky reference (C). The production arithmetic RTL is unchanged. The two FP32 schedules use the same single MAC and single shared division/square-root unit. Prediction timing figures assume a 20 ns clock period.

## Rotation identity and validity

Write `σ=+1` for update and `σ=−1` for downdate. Given a positive pivot `d=Rkk` and current vector component `v=u_k`, define

```text
r = sqrt(d² + σ*v²)
c = d/r
s = v/r
G = [[ c, σ*s],
     [-s,   c]]
J = diag(1, σ)
```

Whenever `r>0`, `c²+σ*s²=1`. Direct multiplication gives `Gᵀ J G = J`, and `G[d,v]ᵀ=[r,0]ᵀ`. For any two trailing columns, the bilinear form `x₁y₁+σ*x₂y₂` is preserved. Applying the transform to the factor row and vector together therefore preserves the signed Gram matrix `RᵀR+σ*u*uᵀ` while zeroing `u_k`. The trailing updates are `R'=cR+σ*s*u` and `u'=c*u-s*R`, with both right-hand sides using the old values. Substituting the original ratios `c_A=r/d` and `s_A=v/d` into A gives the same equations in exact arithmetic, including the minus sign in the downdate’s first equation.

For a nonsingular initial factor, set `w=R⁻ᵀu`. The exact downdate is `Rᵀ(I-wwᵀ)R`, so it is positive definite exactly when `||w||₂<1`. Equality is semidefinite and a larger norm is indefinite. The FP32 schedule aborts for a nonpositive input diagonal, a nonpositive computed radicand or radius, or an invalid, divide-by-zero, or overflow flag. Positive computed pivots can still accept an indefinite *target* near the boundary. A returned triangular factor with positive diagonal only establishes that its own reconstructed covariance is positive definite.

## Numerical simulation

The [Python benchmark](tests/cholesky_benchmark.py) uses exact integer FMA, division and square-root rounding to binary32. Each FP32 multiplication is a clear-and-start MAC. Each two-term sum uses a product or an external-value seed, followed by a continued FMA. Reciprocal coefficients are rounded separately. In particular, the pivot radicand starts from a rounded `d*d` before accumulating the second square. The binary64 reference rebuilds each target from the originally represented factor and all represented vectors up to that row, then independently factors it. It uses compensated summation and checks each binary64 pivot. The reference is binary64, so classification at a separation comparable to binary64 roundoff should be interpreted accordingly.

Seed `0xC401` generates 906 cases. They include 80 cases per regular group, 24 runs of 32 repeated updates, and two directed boundary cases. The coordinate scales in state order `[x,y,θ,ω,b_v]` are `[2,2,0.3,0.1,0.025]`. Tests cover randomly generated SPD covariances, diagonal process roots, strongly correlated covariances, scaled ill-conditioned matrices, small and large vectors, and ten outer updates followed by either sign of central operation. Forced invalid downdates use `||w||=1.05` before input rounding. Boundary downdates start with norms `1-2⁻ᵏ` for `k=16…28`; rounding the supplied vectors to FP32 can place the represented target on either side of the boundary.

Relative error is the Frobenius norm of reconstructed minus target covariance divided by the target Frobenius norm. Maximum elementwise error is the absolute maximum over all 25 entries. Error statistics use final factors from accepted reference-SPD cases. A/B win counts use only paired cases where both algorithms finished and compare relative error. No algorithm produced a completed factor with a nonpositive or nonfinite diagonal.

| Group | N | Reference invalid | A/B valid rejected | A/B invalid accepted | A/B bad factor |
| --- | --- | --- | --- | --- | --- |
| random update | 80 | 0 | [0, 0] | [0, 0] | [0, 0] |
| valid downdate | 80 | 0 | [0, 0] | [0, 0] | [0, 0] |
| diagonal process noise | 80 | 0 | [0, 0] | [0, 0] | [0, 0] |
| very small vector | 80 | 0 | [0, 0] | [0, 0] | [0, 0] |
| very large vector | 80 | 0 | [0, 0] | [0, 0] | [0, 0] |
| ill-conditioned update | 80 | 0 | [0, 0] | [0, 0] | [0, 0] |
| correlated update | 80 | 0 | [0, 0] | [0, 0] | [0, 0] |
| near-boundary downdate | 80 | 13 | [6, 7] | [2, 4] | [0, 0] |
| invalid downdate | 80 | 80 | [0, 0] | [0, 0] | [0, 0] |
| 11-row central update | 80 | 0 | [0, 0] | [0, 0] | [0, 0] |
| 11-row central downdate | 80 | 0 | [0, 0] | [0, 0] | [0, 0] |
| 32 sequential updates | 24 | 0 | [0, 0] | [0, 0] | [0, 0] |
| exact zero pivot | 1 | 1 | [0, 0] | [0, 0] | [0, 0] |
| tiny positive pivot lost to rounding | 1 | 0 | [1, 0] | [0, 0] | [0, 0] |

| Group | N paired A/B/tie | A median/max relative; max abs | B same | C median/max relative; max abs |
| --- | --- | --- | --- | --- |
| random update | [37, 43, 0] | 4.972e-08/1.824e-07; 2.016e-05 | 4.842e-08/1.337e-07; 2.016e-05 | 6.980e-17/2.167e-16; 5.684e-14 |
| valid downdate | [52, 28, 0] | 4.382e-08/1.202e-07; 3.818e-06 | 4.401e-08/1.095e-07; 3.818e-06 | 9.982e-17/2.645e-16; 7.105e-15 |
| diagonal process noise | [32, 48, 0] | 5.312e-08/1.408e-07; 4.718e-07 | 5.088e-08/1.287e-07; 2.884e-07 | 8.574e-17/2.146e-16; 2.220e-16 |
| very small vector | [38, 0, 42] | 1.970e-11/1.955e-10; 4.981e-09 | 4.500e-11/5.671e-08; 1.715e-06 | 8.376e-17/3.021e-16; 7.105e-15 |
| very large vector | [35, 45, 0] | 5.659e-08/2.549e-07; 4.206e-03 | 5.581e-08/1.979e-07; 4.734e-03 | 8.402e-18/2.015e-16; 3.638e-12 |
| ill-conditioned update | [37, 37, 6] | 4.623e-08/1.443e-07; 1.030e-05 | 5.200e-08/1.402e-07; 1.030e-05 | 4.119e-17/1.494e-16; 1.421e-14 |
| correlated update | [43, 37, 0] | 5.524e-08/1.452e-07; 9.794e-06 | 5.616e-08/1.263e-07; 1.048e-05 | 6.709e-17/1.506e-16; 1.421e-14 |
| near-boundary downdate | [37, 20, 0] | 4.201e-08/1.370e-07; 6.909e-06 | 7.421e-08/3.166e-07; 9.111e-06 | 1.963e-17/1.703e-16; 1.421e-14 |
| invalid downdate | [0, 0, 0] | — | — | — |
| 11-row central update | [46, 34, 0] | 1.124e-07/3.280e-07; 1.738e-06 | 1.044e-07/3.572e-07; 2.311e-06 | 7.316e-17/2.016e-16; 8.882e-16 |
| 11-row central downdate | [42, 38, 0] | 1.239e-07/3.831e-07; 1.713e-06 | 1.184e-07/3.839e-07; 1.648e-06 | 8.227e-17/2.140e-16; 8.882e-16 |
| 32 sequential updates | [9, 15, 0] | 1.861e-07/3.952e-07; 4.894e-06 | 1.697e-07/3.956e-07; 4.894e-06 | 9.585e-17/1.766e-16; 1.776e-15 |
| exact zero pivot | [0, 0, 0] | — | — | — |
| tiny positive pivot lost to rounding | [0, 0, 0] | — | 4.865e-08/4.865e-08; 7.338e-08 | 1.917e-20/1.917e-20; 2.711e-20 |

The large-vector absolute discrepancies, up to 0.00473, occur on much larger covariance entries than the other tests. The median relative errors remain near `6e-8`. Over 32 updates the median grows to `1.86e-7` for A and `1.70e-7` for B. B wins 43 of 80 ordinary random updates and 15 of 24 long sequences. A wins 52 of 80 ordinary downdates and every nontied very-small-vector case. For an almost-zero vector, A can preserve the input when its computed scale rounds exactly to one. B can perturb it because a rounded reciprocal multiplied by the original pivot need not round to one. In random-update case 54 the errors are `8.82e-9` for A and `2.17e-8` for B. In case 12 B is better, `2.69e-8` versus `1.29e-7`.

The boundary group is the main numerical concern. Of 67 reference-SPD downdates A rejects 6 and B rejects 7. Of 13 reference-invalid target covariances A accepts 2 and B accepts 4. All rejections in that group are nonpositive computed radicands. Both reject all 80 clearly indefinite cases and the exact zero pivot. In the separate directed case with a tiny positive final pivot, A rejects and B completes. For valid boundary cases that finish, B's maximum relative error is `3.17e-7`, versus `1.37e-7` for A. Small global covariance error is insufficient to validate an eigenvalue or pivot near zero. The hyperbolic transform becomes ill-conditioned as `r` tends to zero, since `c²−s²=1` can coexist with large `c` and `s`. In addition, the initial rounded square can lose the tiny positive difference before sqrt. A scaled or compensated radicand and a margin-aware downdate check are separate design work. A failure must discard the trial factor rather than publish its partially updated rows.

## Exact instruction and storage counts

A five-state update has five pivots and `4+3+2+1=10` off-diagonal entries. **The original dense schedule has 20 divisions per row.** The earlier estimate reported 15 after summing ten pivot-ratio divisions and ten trailing divisions. This report charges all 20 and retains the same other-stage schedule and register access accounting from that estimate.

| Dense five-state rank-one operation | A original | B rotation |
| --- | ---: | ---: |
| Clear-start MACs | 25 | 35 |
| Continued MACs | 25 | 25 |
| **Total MAC requests** | **50** | **60** |
| Standalone multiplication results, including first terms, excluding seeds | 15 | 35 |
| Multiplications fused into a continued MAC | 25 | 25 |
| Identity multiplications used as accumulator seeds | 10 | 0 |
| Standalone FP32 add/subtract requests | 0 | 0 |
| Add/subtract terms fused into a MAC | 25 | 25 |
| Divisions | 20 | 5 |
| Square roots | 5 | 5 |
| Register reads | 130 | 135 |
| Register writes | 55 | 60 |
| Explicit register moves | 5 | 15 |
| Total commands, including moves | 80 | 85 |
| Issue and retire cycles, two per command | 160 | 170 |
| Serialized register access cycles | 185 | 195 |
| **Modeled overhead cycles** | **345** | **365** |

- Each pivot uses two MACs and one root in both schedules. A uses two ratios per pivot. B uses one reciprocal plus two clear-start coefficient products per pivot. Both use one diagonal move.
- Each A trailing entry uses a clear-start `R_old*1` seed, continued signed `s*u_old`, division, clear-start `c*u_old`, and continued `-s*R_new`. Its four MACs include one seed. Each B trailing entry uses two product-plus-MAC pairs. B saves `R_new` in scratch until the second pair has captured `R_old`, then moves it into the factor. There are no externally loaded accumulators.
- A pivot uses 8 reads and 5 writes including its diagonal move. B uses 9 and 6. Every trailing entry uses 9 reads and 3 writes in both models, including B's deferred commit. The benchmark counts the already-available five input words at the stage that generates the deviation. It also excludes the initial `S_Q` load from the isolated kernel; the prediction model retains this copy outside the kernel.
- The dense apples-to-apples replay retains dead last-pivot coefficient calculations. Removing all such calculations would change A from 20 to 18 divisions per row, and B from 5 to 4 divisions and 60 to 58 MACs. The main tables do not credit either variant for that additional pruning.

Both algorithms reuse the same arithmetic lanes. A names five scratch words and B names six. The reciprocal is dead after the two coefficient products, so its slot can share storage with the temporary new factor value. The experiment instantiates the same testbench register capacity for both. The unrolled B stream has five more commands per rank-one update, which would mean five more instruction words if encoded in a flat ROM at the same instruction width. Looped microcode area depends on address/control encoding. No hardware multiplier or divider is added. Neither controller storage nor the effect of register reuse on SKY130 area has been synthesized.

## Cycle estimates from prior latency measurements

Using the workload-wide averages provided in the request (42.394 MAC, 33.792 division, 31.045 sqrt), the serialized estimate saves **62.94 cycles per rank-one operation**: `15×33.792−10×42.394−20`. This mixes MAC starts and continuations. To stay consistent with the earlier prediction estimate, the following proxy uses its measured normal-operand means of 31.596 for clear-start, 66.813 for continue, 33.771 for division, and 31.000 for sqrt. It adds the overhead above. These figures use statistical proxies for the instruction mix. The RTL replay below measures the covariance kernel.

| Schedule | A cycles | B cycles | Expected savings |
| --- | ---: | ---: | ---: |
| One rank-one update | 3,635.65 | 3,465.04 | 170.61 |
| Ten noncentral updates | 36,356.45 | 34,650.40 | 1,706.05 |
| One central update or downdate, dense proxy | 3,635.65 | 3,465.04 | 170.61 |
| All eleven | 39,992.10 | 38,115.44 | 1,876.66 |
| Complete prediction, before unknown unit latencies | 71,337.35 | 69,460.70 | 1,876.66 |

The corrected original prediction has **1,155 MACs, 220 divisions, and 55 roots**. The dense candidate has **1,265 MACs, 55 divisions, and 55 roots**. Both retain 99 standalone mathematical add/subtract expressions elsewhere in prediction, implemented by MAC seeding and continuation. The previous register-access model already included all the trailing ratio traffic. Fixing the 55 missing divisions adds both their compute time and 110 issue/retire cycles, bringing the original proxy from 69,370 to 71,337 cycles. Replacing its covariance term gives 69,461 for rotation, or approximately **1,389.2 µs and 111.14%** of the prediction window before the unknown unit latencies. There are still 22 sine/cosine pairs, 45 wraps and one integer conversion to time. Their modeled issue/retire and RF traffic are included in the subtotal, while their arithmetic latency is unknown.

## Experimental cycle-accurate RTL replay

The [experimental microcode testbench](tests/tb_cholesky_experiment.sv) executes register-indexed instruction streams from both algorithms on the existing FP32 modules. It supplies the same initial factor and vector sequence to A and B. Operand reads, issue, acceptance-to-completion, retirement, and writes consume actual simulation edges under the serialized model above. A row's first controller edge through its final retirement/write is timed. The phase entry and final bank publication are part of the prediction estimate, outside this kernel replay. Input and reference initialization use testbench loads rather than timed architectural writes.

Runtime arithmetic operands come from the simulated register file. Each result and exception vector is checked against the bit-exact Python schedule, including the accumulator on an unpublished MAC. The Python schedule emits the accepted-operation prefix for a rejected case. Consequently the replay verifies arithmetic up to the rejection, while dynamic fault branching and rollback are outside the experimental controller. The first bring-up run detected a MAC mismatch due to the testbench holding `clear` asserted through retirement. Since idle clear discards the accumulated value, the testbench now deasserts it immediately after the issue edge. The functional arithmetic RTL was not modified.

The completed replay checked **43,348 arithmetic requests across 96 schedules** (48 paired inputs). There were zero final-run bit or flag mismatches and no timeouts. Successful paired timings follow.

| RTL group | Paired samples | Mean A | Mean B | Mean savings | Min/max savings |
| --- | ---: | ---: | ---: | ---: | ---: |
| random update | 3 | 2826.667 | 2671.000 | +155.667 | +146/+163 |
| valid downdate | 3 | 2822.333 | 2690.333 | +132.000 | +126/+137 |
| diagonal process noise | 3 | 2427.667 | 1939.333 | +488.333 | +481/+497 |
| very small vector | 3 | 3257.667 | 3110.000 | +147.667 | +146/+149 |
| very large vector | 3 | 2806.000 | 2643.333 | +162.667 | +146/+171 |
| ill-conditioned update | 3 | 3003.000 | 2848.000 | +155.000 | +151/+157 |
| correlated update | 3 | 2833.333 | 2680.667 | +152.667 | +141/+161 |
| near-boundary downdate | 3 | 2805.333 | 2670.333 | +135.000 | +130/+139 |
| 11-row central update | 8 | 30418.875 | 28364.875 | +2054.000 | +2029/+2070 |
| 11-row central update first ten | 8 | 27614.250 | 25717.500 | +1896.750 | +1871/+1913 |
| 11-row central update central only | 8 | 2804.625 | 2647.375 | +157.250 | +150/+166 |
| 11-row central downdate | 8 | 30430.500 | 28393.625 | +2036.875 | +2000/+2073 |
| 11-row central downdate first ten | 8 | 27625.875 | 25724.875 | +1901.000 | +1865/+1937 |
| 11-row central downdate central only | 8 | 2804.625 | 2668.750 | +135.875 | +132/+139 |
| 32 sequential updates | 3 | 89622.333 | 84253.667 | +5368.667 | +5350/+5381 |

The ten outer updates plus the central operation saved **2,054 cycles with the central update** and **2,037 cycles with the central downdate**, on average across eight pairs of each kind. This is 41.08 µs and 40.74 µs at 50 MHz, respectively. The savings are about 6.7% of measured covariance execution, or 3.3% of the full 62,500-cycle budget. The central operation alone saves less than the diagonal first row, where exact zeros change primitive latency. The full discrete logs include failed-case prefixes. For example, both algorithms reject the exact zero pivot after 69 cycles and no completed row.

### Projecting the measured covariance results into prediction

Holding the other prediction stages at their earlier 31,345.255-cycle proxy and substituting the measured 11-row kernel means gives:

| Central operation | A known cycles / µs | B known cycles / µs | B budget consumed | B remaining cycles before unknown work |
| --- | ---: | ---: | ---: | ---: |
| Update | 61,764 / 1,235.3 | 59,710 / 1,194.2 | 95.54% | 2,790 |
| Downdate | 61,776 / 1,235.5 | 59,739 / 1,194.8 | 95.58% | 2,761 |

These hybrid results use synthetic covariance workloads and a statistical estimate for the rest of prediction. The remaining 55–56 µs must cover the 22 trig pairs, 45 wraps, one integer conversion, and any controller costs above the serialization assumptions. The benchmark provides no latency for those unimplemented units and no integrated prediction measurement. In the other direction, the operand-independent single-row ceiling obtained by substituting the primitive bounds is 9,860 cycles for A and 9,945 for B. This is a conservative independent-max sum, and the additional MACs give B a higher ceiling despite its measured speedup.

## Recommendation

Keep the rotation **as the experimental update path**. Its measured speedup is useful on the existing lanes, and ordinary update reconstruction accuracy is broadly comparable on these vectors. Keep the production path unchanged until representative prediction streams, the actual trig and wrap implementations, and a mapped 50 MHz timing run establish the deadline. **Do not adopt the hyperbolic downdate as a drop-in replacement.** It accepted more reference-indefinite boundary inputs in this run. Both implementations need a positive-definiteness margin with explicit reject/rollback behavior before near-singular downdates may be committed. SKY130 area and 20 ns physical closure remain unverified.

## Reproduction

From the hardware repository:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -u tests/cholesky_benchmark.py --rtl
make test synth-check
git diff --exit-code -- fp32_mac.sv fp32_divsqrt.sv
```

The first command used Python 3.14 and Icarus Verilog 12.0. It exited successfully and printed:

```text
tests/tb_cholesky_experiment.sv:116: $finish called at 32175060000 (1ps)
RTL checked 43348 arithmetic requests across 96 schedules
```

The existing regression also passed, with 10,112 checked MAC requests and 12,162 checked requests in each division/square-root mode. Both Icarus synthesis-elaboration commands passed, and the RTL diff was empty.
