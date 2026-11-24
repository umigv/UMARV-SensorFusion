# First x-position SR-UKF correction

This document describes x-only mode (`ENABLE_SECOND_UPDATE=0`). See
[corr_full.md](corr_full.md) for the default four-stage mode and
[corr_xy.md](corr_xy.md) for the two-stage regression and area baseline.

The correction consumes an FP32 predicted state ordered x, y, theta, omega, and
wheel bias. The predicted factor satisfies `P = S^T S`. Pack the upper triangle
in row-major order, with S11 in word 0, S12 through S15 in words 1–4, S22 in
word 5, and S55 in word 14. The same order applies to `corrected_s`.

## Interface

Present `pred_s` with the matching `pred_packet.pred_valid` pulse. The engine
latches both while it waits for the VN300 converter. Return local x, y, theta,
and omega with `convert_done`. Provide the complete row-major upper-triangular
measurement-noise factor on `measurement_s` and hold it stable until `done`.
The caller retains this factor, including its off-diagonal elements. The x
program reads word 0 (SR11). For a full `R = SR^T SR`, `R[0,0] = SR11^2`.

`done` indicates that the x stage finished or failed. Sample the state
`ukf_output` together with `corrected_s` on `x_update_valid`. The output packet's
`ukf_valid` bit remains low until the four-measurement correction is implemented.
A prediction integration that elects to use the x-only posterior must capture
both sidebands on `x_update_valid`. `UKF.sv` and the prediction consumer are
still stubs in this checkout.

On failure, `done` and `failed` pulse together and `x_update_valid` stays low.
The regfile can contain a partially updated state and factor, so the caller
must discard both outputs and retain or reissue the original prediction. The
next accepted transaction reloads the state, factor, and measurements. Internal
rollback would require an additional copy or a preflight with recomputation.
A full snapshot of the state and factor alone costs 20 FP32 words.

## Register allocation

| Addresses | Contents |
| --- | --- |
| 0–4 | Predicted state, then x-corrected state |
| 5 | Reserved |
| 6–20 | Row-major upper triangle of S, updated in place |
| 21–24 | Converted measurements, initial innovations, then updated innovations |
| 25 | Innovation variance d |
| 26 | Inverse square root q |
| 27 | State correction scalar g |
| 28 | Shared temporary for sqrt(d), each p_j and delta, the new diagonal, and the off-diagonal numerator |
| 29–33 | Downdate vector u, consumed and updated in place |
| 34 | Downdate radicand |
| 35 | Downdate coefficient t = u[k]/S[k,k] |
| 36 | Downdate coefficient c = new_diagonal/S[k,k] |

The controller allocates 12 scratch words. At most seven scratch words are
numerically live at once, excluding dead values left readable for debugging.
The peak occurs during vector construction, during state update (five u words,
g, and delta), and in the first off-diagonal loop (four remaining u words, t, c,
and the numerator). The MAC retains its partial sum internally. FP32 one and
SR11 use read-only operand decode addresses 94 and 83.

## Verification and synthesis

Run `python3 tests/run_corr_x.py` with Icarus Verilog. The test checks the
FP32 intermediate and output values and independently compares the reconstructed
covariance against the rank-one posterior identity. It runs repeated
transactions with nonzero cross-correlations in both factors, exercises both
arithmetic wait paths, aborts on reset, and rejects loss of a positive pivot
and a rounded zero radicand. The heading test checks the raw cross-boundary
difference because heading wrapping is deferred.

The 15 successful fixtures matched the FP32 reference values exactly
numerically. The largest absolute reconstructed covariance error was
`5.78911033e-7`. Core latency from `start` to `done` was 4,913 cycles for
the baseline and 4,378–4,944 cycles across successful fixtures. With the
testbench converter handshake, engine latency was 4,919 cycles for the
baseline and 4,384–4,950 cycles across successful fixtures.

Run `python3 tests/synth_corr_x.py /path/to/sky130_fd_sc_hd__tt_025C_1v80.lib`
for a cell-area estimate. The script uses a temporary package-qualified
compilation unit for Yosys 0.52's packed-struct frontend. The estimate covers
the correction engine, the microcode controller, and one instance each of
the MAC and division/square-root unit. It excludes the converter, the
prediction engine, physical routing, and timing closure.
The SKY130 HD TT 1.80 V, 25 °C mapping reported 142,011.2 µm² of standard
cells for that hierarchy. Tiny Tapeout fit and 50 MHz operation are unverified.

## Remaining measurement updates

After the x correction, extend the program at `PC_X_DONE`. With the full
noise factor, solve the lower-triangular system using `SR^T` to whiten the
remaining measurement rows and their residuals. The first whitened row is
equivalent to the unwhitened x update up to scale and has already been
consumed. For each of the next three whitened rows, compute its projection
through the current S and apply a scalar update and factor downdate. Carry
the updated residuals and factor forward between rows, and assert
`ukf_valid` only after all four measurements succeed. Add heading wrapping
at the marked innovation and state-update sites before using those rows
with heading measurements across the angle boundary.
