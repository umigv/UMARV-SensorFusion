# Third whitened correction

Set `ENABLE_FOURTH_UPDATE=0` to run x, whitened y, and whitened heading
in sequence. See [corr_full.md](corr_full.md) for the default four-stage
engine. Set `ENABLE_THIRD_UPDATE=0` for the x–y transaction or
`ENABLE_SECOND_UPDATE=0` for x only. The core and engine pulse
`x_update_valid`, `xy_update_valid`, and `xyz_update_valid` with their
respective state and `corrected_s` snapshots. In three-stage mode, the
first two pulses occur while the transaction is busy. The third pulse
coincides with successful `done`. `ukf_valid` remains low.

Keep `measurement_s` stable through `done`. The heading stage back-substitutes
the third column of the inverse factor using noise words 0, 1, 2, 4, 5, and 7
at read-only addresses 83, 84, 85, 87, 88, and 90. It projects the current x,
y, and heading residuals and the current upper-triangular factor. The five-element
downdate vector feeds the same state update and Cholesky downdate as the first
two stages. Heading arithmetic is ordinary FP32.

A failure pulses `failed` with `done` and suppresses the failed stage's
success pulse. Earlier stage snapshots remain valid if captured on their
pulses. The live register file can be partially updated on failure, so a
retry must reload the original prediction and measurements.

## Scratch lifetimes

The measurement residuals remain at 21–24. No scratch addresses were added.
PC numbers and vector indices are zero-based.

| Addresses | Heading-stage values |
| --- | --- |
| 25 | Variance `d`, PCs 59 to 37 |
| 26 | `q`, PCs 38 to 42 in the last vector column |
| 27 | `g`, PC 39 through the last state delta at PC 11 |
| 28 | Back-substitution numerators at PCs 45–46 and 48–49, then square root at PCs 37–38 and one projection numerator per column at PCs 40/41/60–42 |
| 29–32 | First four `u` values from PC 42, then the shared downdate vector |
| 33 | Whitening coefficient `c` from PC 44 through PC 58, `t3` from PC 58 through the last PC 60, then the last `u` word from PC 42 |
| 34 | `a` from PC 49 through PC 53, then `t1` from PC 55 through the last PC 40 |
| 35 | `b` from PC 46 through PC 56, then `t2` from PC 57 through the last PC 41 |
| 36 | `epsilon3` from PC 52 through PC 39 |

Addresses 28 and 34–36 revert to the existing shared arithmetic and
downdate uses after the projection. The peak is **nine live FP32 words**.
At entry to the last projection iteration, four completed `u` words
coexist with `t1`, `t2`, `t3`, `q`, and `g`. In the preceding iteration
the temporary numerator coexists with three completed `u` words and
those five scalars. The final `u` word replaces the dead `t3` slot.
The allocation remains **12 FP32 words at 25–36**.

## Simulation

Run `python3 tests/run_corr_x.py`, `python3 tests/run_corr_xy.py`, and
`python3 tests/run_corr_xyz.py` with Icarus Verilog. The previous suites
passed all 15 x-only and 16 x–y success cases. The new suite passed 17
three-stage success cases and checked the earlier snapshots, third
whitening coefficients, residual projection, variance, gain scalar
and downdate vector. It exercised both signs of each SR12, SR13, and
SR23, cross-correlated prediction factors, all state updates, the
shared ROM addresses and operand reads, stalls and bounded writeback,
repeated starts, reset during each stage, and invalid downdates.
It also checked a third-stage failure on a negative SR33.

The results matched the sequential FP32 reference numerically. Against
an independent joint 3×3 correlated Kalman update solved with
partial-pivot elimination, maximum absolute state error was
`7.31459499e-7` and maximum absolute reconstructed covariance error
was `5.58162247e-7`.

| Baseline fixture latency | Cycles |
| --- | ---: |
| X stage | 4,913 |
| Whitened Y stage | 4,919 |
| Whitened heading stage | 5,194 |
| Total core | 15,026 |
| Engine through converter handshake | 15,032 |

Heading latency ranged from 5,078 to 5,204 core cycles, and total core
latency ranged from 14,431 to 15,026. Counting the start cycle and
subsequent busy cycles, the baseline heading stage spent 4,089 cycles
in the MAC and 960 in division/square-root operations. Across all three
stages those arithmetic counts were 11,867 and 2,752 cycles, respectively.

## SKY130 HD mapping

Yosys and ABC used the existing SKY130 HD TT 1.80 V, 25 °C Liberty flow.
The register-file split uses the netlist attribution described in
[corr_xy.md](corr_xy.md). The shared Cholesky routine has no separate area.

| Mapped cell area (µm²) | X–Y shared-ROM baseline | X–Y–heading |
| --- | ---: | ---: |
| FP32 MAC | 19,537.49 | 19,518.72 |
| FP32 divider/square-root | 11,345.88 | 11,334.62 |
| Register file and attributable muxes | 60,399.18 | 60,347.88 |
| Controller and shared logic | 9,581.69 | 10,742.80 |
| Engine wrapper | 41,317.13 | 41,349.66 |
| **Full correction engine** | **142,181.36** | **143,293.68** |

The change is **+1,112.32 µm² (+0.78%)**. Mapped cells increased from
14,466 to 14,876 and DFFs from 2,710 to 2,714. All four added DFFs are
in control and completion logic. Register-file storage remains at 1,184
DFFs, including 384 SPAD DFFs. Both versions have zero latches and zero
constant-storage DFFs. The PC remains six bits and operand addresses
remain seven bits. The mapping excludes the converter, prediction
engine, physical routing, and timing. Tiny Tapeout fit and 50 MHz
operation remain unverified.

The fourth stage and full UKF validity pulse are documented in
[corr_full.md](corr_full.md). Angle wrapping remains a TODO.
