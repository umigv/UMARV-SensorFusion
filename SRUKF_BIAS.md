# Five-state SR-UKF with dead-wheel bias

The five-state design is archived under `legacy/fixed-point` with top `srukf_bias_filter`. The four-state `srukf_filter` remains a separate top for the baseline. Both use the existing 400 Hz scheduler, CORDIC and restoring arithmetic modules. The five-state top has one MAC, one CORDIC, one square root and one divider.

## Interface and arithmetic

All signal words are signed 32-bit. Lane zero is least significant. Initialization and output state order is `x, y, theta, omega, b_v`, with fractional widths **21, 21, 23, 25, 25**. Configure zero initial bias by setting the fifth initialization word to zero. Set bias prior standard deviation in the fifth diagonal of `init_root` and the per-step bias random-walk standard deviation in lane 4 of `process_std`. Both must be positive. The other four lanes of `process_std` are diagonal process factor entries. `init_root`, `root` and `measurement_root` are **upper** factors in row-major triangular order, 15 or 10 words, and use **F26**. The state factor satisfies P = SᵀS. QR rotation and gain words use F27.

The wheel sample, `wheel_offset` in meters, and `wheel_std` in m/s use F21. The standard deviation is for independently fluctuating wheel speed error. The filter projects `wheel_std * dt` through the central midpoint heading as one additional process QR row. Zero disables that contribution. Body +x is forward, +y is left, and positive yaw is counterclockwise, so a left wheel reads `v_center - omega*y_offset` before additive bias. At each sigma point, the controller forms `v_wheel - b_v + omega*y_offset`, then integrates position at the midpoint heading. It retains the bias and yaw-rate sigma states across motion propagation.

The scheduler starts prediction 1.25 ms before the next state epoch. At 400 Hz every valid prediction advances the committed timestamp by exactly 2,500 µs, including during packet loss. The scheduler verifies that elapsed interval and presents 5,368,709 in Q1.31. A correction consumes a VN-300 packet only at its matching epoch. The four packet lanes use F21/F21/F23/F25 and carry principal-branch heading. This interface uses the scheduled epoch grid, with no variable-interval replay for off-grid packets.

The unscented parameters are alpha=1, beta=2, kappa=−1. The sigma scale is 2, the ten outer means and covariance weights are 1/8, the central mean weight is −1/4, and the central covariance weight is 7/4. The first pass uses all 11 points and a signed 36-bit lane sum. The second pass streams ten noncentral deviations, five process-noise rows and the wheel-noise row, then applies the positive central Cholesky update. A signed local heading lift travels with the current sigma point. It determines the heading mean and covariance deviations even when a broad distribution crosses the ±π branch. The propagated and published angles are wrapped. A wrapped innovation maps the measured heading to the principal error branch.

Correction uses H = [I₄ 0]. QR forms a four-dimensional innovation factor. For each of five state rows, two right triangular solves produce one row of U and the 5×4 gain from the selected cross covariance. The bias correction uses the fifth gain row and all four innovations. Four column downdates of U update the five-dimensional posterior. State and factor commit together. Invalid pivots, nonpositive downdate radicands, overflow and saturation stop publication.

| Boundary | Fixed-point treatment |
| --- | --- |
| Sigma span | Twice the packed F26 factor entry, rounded once to the target lane before the signed offset is applied. |
| Velocity and yaw | Convert the F25 bias to F21. Round F46 yaw times offset to F21, then combine in signed arithmetic with the wheel input. Q1.31 time products use the complete product for distance, midpoint angle and final angle. |
| CORDIC | One F23 circular rotator, reused for every sigma. Wrapped midpoint input and F23 trig outputs. |
| Mean and deviations | `(-2*center + sum(outer))/8` rounded once. Local heading sums preserve the signed sigma lifts. Weighted deviations narrow to F26. |
| QR and Cholesky | F52 sums or differences of squares reach the integer root at full precision. Round trailing F53 rotations once to F26. The reference rank-one formulation divides the rounded F26 trailing numerator by the F27 coefficient. |
| Gain and correction | Raw F52 forward and F53 backward numerators reach the shared divider. U is F26, gain is F27. Corrected state writes narrow once per lane after the aligned dot product. |

## Verification

Run from the workspace root. The C++ build reuses the repository's four-state baseline, Q arithmetic, rotation tables, full-root and division functions, and separate double reference execution.

```sh
python3 tests/srukf_bias/run.py
g++ -std=c++17 -O3 -Wall -Wextra -Werror -Wno-unused-function tests/srukf_bias/study.cpp -o /tmp/bias-study
python3 UMARV-SensorFusion/legacy/fixed-point/tests/test_srukf_bias.py positive 72 --study /tmp/bias-study --stalls
python3 UMARV-SensorFusion/legacy/fixed-point/tests/test_srukf_bias.py dropout_short 800 --study /tmp/bias-study --skip-idle --stalls
python3 UMARV-SensorFusion/legacy/fixed-point/tests/test_srukf_bias.py legacy-dropout_coast 160 --start 10060 --study /tmp/bias-study --skip-idle --stalls
python3 UMARV-SensorFusion/legacy/fixed-point/tests/test_srukf_bias.py zero 1 --study /tmp/bias-study --reject-zero-bias
```

The bias trials use three shared seeds (17, 29 and 47) across 14 scenarios. They include straight and turning motion, +0.35 and −0.30 m/s biases, drift, 12% speed scale error, a 0.45 s slip, stationary intervals, a 17 s fix gap, correlated VN-300 noise, a thin factor pivot, low noise with both matched and wide priors, and 600 s operation. The archived 21 four-state trajectories are replayed separately on the same sensor streams. Nominal bias trials use 0.08 m position and 0.006 rad heading sensor deviations, with 100 Hz packet availability on the 400 Hz correction grid. The low-noise cases offer all 400 packets per second. The correlated test sets an AR(1) coefficient of 0.98 and keeps the same marginal R.

Model A is the original four-state algorithm (fixed and double). With nonzero mounting offset its motion input uses the previous yaw-rate estimate for the lever correction. Model B is the five-state filter. Model C uses the double four-state filter and an external estimate based on filtered, nonoverlapping one-second differences between the integrated wheel reading and the projected position displacement. It freezes through unavailable measurements and discards a window that straddles a long gap. This model is a software cost/accuracy comparison only.

CSV convergence is the first start of a two-second interval continuously within 0.05 m/s of the true additive bias. Initial near-zero bias estimates can meet that definition before a scale error or slip. Bias RMSE and the final/peak estimates capture those errors separately. The fixed reference comparisons use physical state and covariance, with shortest-angle errors at publication. The CSV includes truth-relative NEES/NIS, and paired normalized covariance RMS using the double covariance's marginal variances. High NEES during outages or mismatched sensor conditions is evidence of model and noise limitations even when the paired fixed-point error is small.

### Measured software results

[Bias and precision trials](../tests/srukf_bias/results.csv), [original-trial replay](../tests/srukf_bias/legacy.csv), and [bias trajectories and standard deviations](../tests/srukf_bias/trajectory.csv) contain the per-seed measurements. The original four-state fixed baseline reproduced all 21 archived position and heading RMSE, NEES and NIS values to their published precision.

The chosen F26-factor/F25-bias configuration completed 63/63 trials, 2,316,000 predictions and 719,100 corrections with zero QR failures, downdate failures, rounded-to-zero root inputs, saturations, accumulator overflows or divergence. Its largest per-trial discrepancies from the five-state double model were 6.363 mm position RMSE, 0.216 mrad heading RMSE, 1.489 mm/s bias RMSE and 0.052 normalized covariance RMS.

| Factor precision and bias F | Completed | Observed fault |
| --- | ---: | --- |
| F25 factors, each of F21/F23/F25 bias | 39/42 each | First correction of all wide-low-noise trials. The third downdate pivot reached an exact integer zero after the solve and factor quantization. |
| F26 factors, F25 bias | 42/42 new + 21/21 archived | Zero numerical failures. |
| F27 factors, F25 bias | 6/6 targeted | Wide-low-noise and dropout replays completed. |
| F28 factors, F25 bias | 5/6 targeted | Prediction QR saturated during one dropout. |
| F29 factors, F25 bias | 3/6 targeted | Prediction QR saturated during all three dropouts. |

The F25-factor low-noise failure persists even with full-width radicands because the two F25 operands reaching the squared subtraction have already quantized to equal magnitudes. Increasing only the bias-state fraction did not fix it. The F26 factor retains one additional fraction bit and a signed range of about ±32. An initial heading implementation that independently wrapped each sigma deviation also departed from double on the six original 25-second dropout trials. Retaining signed heading lifts through the weighted mean and factor construction eliminated those departures.

Median across the three seeds. Model B is the selected fixed-point design. Bias columns report m/s.

| Scenario | A pos. RMSE m | B pos. RMSE m | C pos. RMSE m | B heading RMSE mrad | B final bias | True final bias | B bias RMSE | B convergence s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| zero | 0.0079 | 0.0109 | 0.0298 | 1.14 | -0.0007 | 0.0000 | 0.0261 | 0.733 |
| positive | 0.8692 | 0.0107 | 0.1402 | 1.11 | 0.3496 | 0.3500 | 0.0288 | 0.703 |
| negative | 0.7438 | 0.0106 | 0.1276 | 1.11 | -0.3004 | -0.3000 | 0.0279 | 0.843 |
| drift | 0.5442 | 0.0200 | 0.1282 | 1.11 | 0.3677 | 0.3698 | 0.0300 | 0.632 |
| scale | 0.5242 | 0.0273 | 0.1156 | 1.12 | 0.2107 | 0.0000 | 0.2172 | — |
| slip | 0.1772 | 0.1056 | 0.1871 | 1.12 | 0.0021 | 0.0000 | 0.0862 | 0.733 |
| turn | 0.3729 | 0.0103 | 0.0841 | 1.11 | 0.2916 | 0.3000 | 0.0237 | 0.652 |
| stationary | 0.7516 | 0.0107 | 0.1132 | 1.11 | 0.3018 | 0.3000 | 0.0258 | 0.703 |
| dropout | 11.4758 | 9.8221 | 10.3418 | 1216.38 | 0.3200 | 0.3200 | 0.1838 | 0.458 |
| correlated | 0.6906 | 0.0451 | 0.1134 | 3.66 | 0.3504 | 0.3500 | 0.0410 | 2.433 |
| thin | 0.0001 | 0.0000 | 0.0000 | 0.01 | 0.0003 | 0.0003 | 0.0002 | 0.003 |
| quiet | 0.0001 | 0.0000 | 0.0000 | 0.01 | 0.0003 | 0.0003 | 0.0002 | 0.003 |
| quiet_wide | 0.0002 | 0.0003 | 0.0013 | 0.40 | 0.0002 | 0.0003 | 0.0004 | 0.003 |
| long | 0.5630 | 0.0093 | 0.0423 | 1.13 | 0.2771 | 0.2800 | 0.0078 | 0.652 |

The stationary segment's median bias RMSE was 3.50 mm/s with position fixes. The filter inferred drift in predicted position from those fixes even at zero true speed. During the fix gap the estimated bias remained constant. The 17-second gap caused a large position error under changing true angular rate, and reacquisition contaminated the bias estimate temporarily. In that scenario the median peak absolute bias estimate was 1.87 m/s for a 0.32 m/s true bias. Seed 17 went from 0.322 m/s at the start of loss to 1.769 m/s on the first returning packet. Estimator agreement with double does not establish calibrated covariance. Median NEES was near the five-state dimension in ordinary biased straight and turning trials, and much higher in the slip, scale, wide-low-noise and dropout tests.

The scale test had zero true additive bias. Model B ended near 0.211 m/s by absorbing part of the scale error. The slip result likewise includes an apparent bias response. A single additive state cannot identify those error sources separately from other motion mismatch.

**Recommendation:** Keep the five-state design selectable for persistent additive bias. It reduced position RMSE roughly 80-fold on the constant-bias trials and 60-fold in the 600-second trial. The external estimator is less accurate but substantially improves constant-bias tracking with the original filter. A production choice depends on SKY130 mapping and the allowable reacquisition transient. Keep the four-state option for zero-bias or hardware-constrained deployments.


## Hardware scope

Icarus elaborated one `srukf_mac`, `cordic`, `srukf_full_root` and `srukf_full_div`. The five-state top stores 160 state bits and a 480-bit committed triangular root, up from 128 and 320 logical bits. It streams one sigma point, keeps one 5-word candidate bank and a 15-word factor workspace, and stores 20 U words with four gain scratch words. There are 4,899 declared register bits at the complete top before optimization. The earlier four-state source has 4,918 declared bits and stores full rectangular factor banks. This source-count comparison includes constant and redundant registers and is independent of mapped area.

Both designs use one signed 32×32 product path with a 72-bit accumulator. The two prediction passes launch up to 22 trig rotations per prediction; the original configuration's zero central mean weight permitted 17. The innovation QR retains four pivots. The additional state adds four forward and four backward solve launches and a fifth pivot to the posterior downdates. Testbench cycle counts below include resource waits; longer experiments skip idle clocks between slots only.

The local Icarus BLIF backend rejected the full top's variable-index memory writes. Yosys, a SKY130 Liberty file and an implemented target clock were unavailable in this environment. [The generic synthesis script](legacy/fixed-point/tests/synth_srukf_bias.ys) provides an entry point for full-design mapping. The shared standalone root and divider synthesized to 1,407 and 2,052 nonconstant generic BLIF logic nodes with 187 and 179 sequential bits respectively. Full SKY130 area, physical clock closure and the 4×8 Tiny Tapeout fit remain unverified.

### Measured RTL latency

Every state word and upper-factor entry agreed with the fixed-point reference at each committed prediction and correction. The separate invalid-initialization test rejected a zero bias-prior factor. The 72-step positive and legacy quiet tests used the uncompressed staggered schedule. The other cases retained all active phase clocks and used shortened idle gaps.

| Case | Reference checkpoint | Predictions | Corrections | Max prediction clocks | Max correction clocks |
| --- | ---: | ---: | ---: | ---: | ---: |
| positive | 0 | 72 | 18 | 15,214 | 12,910 |
| positive | 0 | 1024 | 256 | 16,367 | 13,292 |
| negative | 0 | 384 | 96 | 15,492 | 13,292 |
| quiet | 0 | 512 | 512 | 14,400 | 12,268 |
| thin | 0 | 512 | 512 | 14,313 | 12,522 |
| quiet_wide | 0 | 220 | 220 | 14,833 | 13,037 |
| turn | 1600 | 192 | 48 | 16,998 | 12,784 |
| stationary | 1700 | 192 | 48 | 15,329 | 12,892 |
| dropout | 8760 | 192 | 152 | 16,962 | 13,262 |
| dropout_short | 0 | 800 | 400 | 17,012 | 13,009 |
| legacy-dropout_coast | 10060 | 160 | 0 | 16,962 | 0 |
| legacy-dropout | 11390 | 640 | 30 | 16,962 | 12,801 |
| legacy-quiet | 0 | 72 | 72 | 14,571 | 11,480 |

[Cycle-count CSV](../tests/srukf_bias/rtl_cycles.csv). Across these 4,972 predictions and 2,364 corrections, the peak phases were **17,012 prediction clocks** and **13,292 correction clocks**. At the simulated 50 MHz clock these take 0.34024 ms and 0.26584 ms within 1.25 ms. The observed worst phase would require at least 13.6096 MHz to fit that budget, conditional on the hardware achieving that clock and comparable resource waits. The four-state replay of the same legacy quiet input took 5,290 prediction and 7,727 correction clocks. The five-state replay took 14,571 and 11,480 clocks.


## Files

- Five-state top and integration report: `srukf_bias_filter.sv`, `SRUKF_BIAS.md`.
- RTL test and synthesis entry points: `tests/tb_srukf_bias.sv`, `tests/test_srukf_bias.py`, `tests/synth_srukf_bias.ys`.
- Software reference, runner and results: `../tests/srukf_bias/`.
- Shared simulation helpers: `../tests/srukf_precision/study.cpp`, with dimension-generic QR and rank-one kernels and a library include guard. The original pipeline RTL sources remain selectable.
