# Four-stage SR-UKF correction

The correction engine defaults to all four rows. The first three stage
pulses, `x_update_valid`, `xy_update_valid`, and `xyz_update_valid`, each
identify their state and factor snapshot while the engine is busy.
On successful fourth-stage completion, sample `ukf_output` and
`corrected_s` together when `ukf_output.ukf_valid` and `done` pulse.
A failed transaction pulses `failed` with `done` and leaves
`ukf_valid` low. The working registers may be partly updated on failure.
Retry by reloading the original prediction and measurements.

Set `ENABLE_FOURTH_UPDATE=0` for the prior three-stage completion behavior,
`ENABLE_THIRD_UPDATE=0` for x–y, or `ENABLE_SECOND_UPDATE=0` for x only.
The engine keeps `ukf_valid` low in those partial modes. Hold the full
`measurement_s` factor stable through `done`. The program counter and
branch targets are now seven bits. Operand addresses remain seven bits.

The fourth row solves `SR*h = [0, 0, 0, 1]^T` by backward substitution using
the existing MAC and divider. It forms the scalar innovation from the four
residuals remaining after the first three measurements, computes `t = S*h`,
and seeds the innovation variance with FP32 one at ROM address 94. The
program forms `u = S^T*t/sqrt(1 + t^T*t)` one column at a time, skipping
below-diagonal entries. The existing loops update all five state words,
all four residuals, and downdate the upper-triangular factor. Heading
uses ordinary FP32 arithmetic.

## Scratch lifetimes

The existing 12-word scratchpad remains at 25–36. PC numbers and column
indices are zero-based.

| Address | Fourth-stage use |
| --- | --- |
| 25 | Fourth whitening coefficient from PC 62 through PC 85, then `t4` through the last PC 87 |
| 26 | `q` from PC 38 through the last PC 42 |
| 27 | `g` from PC 39 through the last state delta at PC 11 |
| 28 | Back-substitution numerators at PCs 63–64, 66–67, and 70–71, then variance at PC 86, square root at PCs 37–38, and one projection numerator per column before PC 42 |
| 29–32 | First four downdate-vector words produced by PC 42 |
| 33 | Third whitening coefficient from PC 64 through PC 84, then `t3` through the last PC 60, then final downdate-vector word |
| 34 | First whitening coefficient from PC 71 through PC 76, then `t1` through the last PC 40 |
| 35 | Second whitening coefficient from PC 67 through PC 80, then `t2` through the last PC 41 |
| 36 | Fourth innovation from PC 75 through PC 39 |

The peak is **10 live FP32 words** at the start of the final projection
column. Four completed downdate words coexist with `t1`–`t4`, `q`, and
`g`. The preceding column also reaches ten with three completed words,
those six scalars, and its projection numerator. The existing state-update
and Cholesky routines reuse the scratch slots after the projections.

## Simulation results

Run the Icarus Verilog and Python suites:

```sh
python3 tests/run_corr_x.py
python3 tests/run_corr_xy.py
python3 tests/run_corr_xyz.py
python3 tests/run_corr_full.py
```

All 15 x-only, 16 two-stage, 17 three-stage, and 18 four-stage success
cases passed. The full suite compared the whitening coefficients,
intermediate vector and variance, state, residuals, and upper-triangular
factor to the sequential FP32 reference. Against an independent joint
4×4 correlated Kalman update solved by partial-pivot elimination,
the largest absolute errors were `7.85926614e-7` for the state and
`1.78938516e-6` for the reconstructed covariance. The sequential
FP32 comparisons had zero numerical error on these fixtures.

The simulation covers positive and negative entries in all six
off-diagonal positions in the measurement-noise factor, cross-correlated
predictions, arithmetic stalls and bounded writeback, reset in each
stage, repeated requests, invalid noise pivots, and a failed fourth
downdate at the rate pivot. It checks that intermediate, failed, and
partial-mode completions leave `ukf_valid` low.

| Baseline fixture latency | Cycles |
| --- | ---: |
| X stage | 4,913 |
| Whitened Y stage | 4,919 |
| Whitened heading stage | 5,194 |
| Whitened turn-rate stage | 5,492 |
| Total core | 20,518 |
| Engine including testbench converter handshake | 20,524 |

Across successful fixtures, the fourth stage took 5,419–5,532 cycles
and total core latency was 19,930–20,518 cycles.
