# Second whitened correction

Set `ENABLE_SECOND_UPDATE=1` and `ENABLE_THIRD_UPDATE=0` to run the x stage
and then the second whitened row. Set `ENABLE_SECOND_UPDATE=0` for the original
x-only transaction. See [corr_full.md](corr_full.md) for the default four-stage
engine.
The noise input keeps the existing packed upper-triangular ordering and must
remain stable through `done`. The second stage reads SR11, SR12, and SR22 from
words 0, 1, and 4. The caller retains the other words for future rows.

In two-stage mode, sample the x-only state and factor on `x_update_valid` while
`busy` is high. Sample the combined state and `corrected_s` on
`xy_update_valid`, which coincides with successful `done`. In x-only mode
`x_update_valid` and `done` coincide as before. The packet's `ukf_valid` bit
is reserved for completion of all four measurement rows.

A failed stage pulses `failed` and `done` without a success pulse for that
stage. A second-stage failure leaves the earlier x-stage pulse valid, but
its state and factor must have been captured at that pulse to retain them.
The in-place working registers may be partially updated when failure is
reported. Discard those outputs and reload the original prediction to
retry. The engine has no separate rollback bank.

## Scratch allocation and peak occupancy

Addresses 0–24 retain their previous meanings. PC numbers and columns
below are zero-based. The second stage borrows 34–36 only before their
normal Cholesky uses.

| Address | Second-stage values and lifetimes |
| --- | --- |
| 25 | `d` written at PC 36, consumed by PC 37 |
| 26 | `q` written at PC 38, last read at PC 42, column 4 |
| 27 | `g` written at PC 39, last read by shared PC 11, column 4 |
| 28 | `r11*r22` at PCs 26–27, `sqrt(d)` at PCs 37–38, one projection numerator at PCs 40/41–42, then shared delta and downdate temporaries |
| 29–33 | `u[0:4]` written one word per PC 42 iteration, then consumed and updated in place by the shared downdate |
| 34 | `a` at PCs 27–31, `t1` at PCs 32–40 (last column), then shared radicand |
| 35 | `b` at PCs 28–33, `t2` at PCs 33–41 (last column), then shared downdate coefficient |
| 36 | `epsilon2` at PCs 30–39, then shared downdate coefficient |

Peak occupancy is **8 simultaneously live FP32 words**. At entry to the
last projection iteration, four completed `u` words coexist with
`t1`, `t2`, `q`, and `g`. In the preceding iteration, three completed
`u` words and those four scalars coexist with the temporary numerator.
The existing state-update and downdate routines each peak at seven live
scratch words. The allocation remains **12 words (25–36)**.

## Numerical and latency results

Run the two suites with Icarus Verilog and Python:

```sh
python3 tests/run_corr_x.py
python3 tests/run_corr_xy.py
```

All 15 x-only and 16 combined success cases passed. The combined cases
matched the sequential FP32 reference numerically. Against the independent
joint correlated 2×2 Kalman update, maximum absolute errors were
`5.29221269e-7` for state and `5.48322803e-7` for reconstructed covariance.
The suite checks both signs of SR12, correlated prediction factors, all
five state updates, in-place residuals, arithmetic handshake and write
bounds, repeated transactions, reset during either stage, and failures
in each stage. The heading stays in ordinary FP32 arithmetic.

| Measured cycles | Baseline fixture | Range across combined successes |
| --- | ---: | ---: |
| First x stage | 4,913 | 4,378–4,944 |
| Second whitened stage | 4,919 | 4,833–4,959 |
| Total core | 9,832 | 9,227–9,860 |

The baseline engine transaction is 9,838 cycles with the testbench converter
handshake, up from 4,919 cycles for the x-only engine.

## SKY130 HD synthesis

The comparison uses the same Yosys 0.52 and ABC flow and the SKY130 HD
TT 1.80 V, 25 °C Liberty file for the original x-only source and the extended
source with both stages enabled. Each column reports mapped standard cells.
The converter, prediction engine, physical routing, and timing analysis
are outside this synthesis hierarchy.

| Mapped area (µm²) | X-only | X and whitened y |
| --- | ---: | ---: |
| FP32 MAC | 19,478.68 | 19,366.07 |
| FP32 divider/square-root | 11,412.20 | 11,343.38 |
| Controller, microcode, and shared logic | 8,859.75 | 9,835.68 |
| Register file and attributable read/write muxes | 60,984.74 | 60,882.14 |
| ↳ Register-file storage alone | 29,628.42 | 29,628.42 |
| ↳ SPAD storage alone, included in register storage | 9,609.22 | 9,609.22 |
| Engine wrapper | 41,275.84 | 41,317.13 |
| **Full correction engine** | **142,011.20** | **142,744.40** |

The physical cell count increased from 14,375 to 14,450, and the DFF count
increased from 2,707 to 2,710. Both designs mapped zero latches. SPAD storage
remained at 384 DFFs. The register file and wrapper accounted for about 72%
of the x-only area. The wrapper includes the prediction and measurement
snapshots. The Cholesky routine shares the controller and arithmetic units,
so it has no separately mapped area. The arithmetic RTL is identical for
both variants. Its small area differences come from the two synthesis runs.

The register-file split uses netlist attribution. It includes storage
DFFs, logic used exclusively to drive their D inputs, and cells on
combinational paths from their Q outputs to read operands. The controller
row includes the remaining local logic and shared decode. Total hierarchy
area is the mapped area reported by Yosys. Tiny Tapeout fit and 50 MHz
operation remain unverified.

To reproduce the breakdown, run `tests/synth_corr_x.py <sky130.lib> <netlist.json>`
on each source revision, then run
`tests/report_corr_area.py <sky130.lib> <baseline.json> <extended.json>`.

### Shared math ROM follow-up

The prediction source defines the math address layout. The shared combinational
decoder now supplies zero at 93 and one at 94. The remaining math slots are
reserved. Correction reads the measurement-noise words SR11, SR12, and SR22 at
83, 84, and 87, respectively. None of these addresses is writable. Operand
addresses use seven bits and the correction PC uses six.

With the shared decoder, the same SKY130 flow mapped 142,181.36 µm² and
14,466 cells. The design still has 2,710 DFFs, including 384 SPAD DFFs, and
zero latches. The shared decoder uses no storage DFFs.

## Third and fourth measurements

Extend the stage transition after the second downdate. Use the remaining
rows of the triangular solve with `SR^T` to form the next whitened
measurement rows and their innovations from the current residuals. Project
each row through the current factor, run the shared state update and
downdate, and assert `ukf_valid` after the fourth successful row. Heading
wrapping remains a separate TODO. The prediction consumer still needs to
capture the state and covariance together at the chosen completion point.
