# Sigma-point generation

`sigma_generator.sv` implements a five-dimensional state and 11 sigma points
for an upper-triangular factor with `P = S^T*S`:

- `sigma[0][j] = state[j]`
- `sigma[i+1][j] = state[j] + gamma*S[i][j]`
- `sigma[i+6][j] = state[j] - gamma*S[i][j]`

Here i and j run from 0 to 4; for j<i the offset is zero. The point direction
uses a row of S, not a column. Lower-triangular input values are ignored.
State component order is x, y, theta, omega, v. All data, including gamma,
uses signed WIDTH-bit fixed point with FRAC fractional bits. Gamma is provided
by the caller; this block does not derive UKF scaling parameters or weights.
No angle normalization is performed.

## Shared MAC interface

No MAC instances or multiplication operators are inside the generator. Connect
its ports to a single shared `MAC_array` configured with 11 lanes and matching
WIDTH/FRAC:

| Generator port | Array port |
| --- | --- |
| mac_start | start |
| mac_clear | clear |
| mac_last | last |
| mac_a, mac_b | a, b |
| mac_result | result |
| mac_busy | busy |
| mac_result_valid | result_valid |
| mac_overflow | overflow |
| mac_acc_overflow | acc_overflow |

Array done is not needed: result_valid identifies finalized products. The array
must have exclusive ownership assigned to this generator throughout a request.
An arbiter must route operands/controls and return values and flags for whichever
client owns the array. Grant this client only after earlier operations drain,
and retain ownership until mac_request goes low. The grant is acquired once;
it is not a per-batch ready signal. While this is the only client, connecting
mac_grant to 1 and wiring the ports directly is sufficient.

Both batch launches and their FSM transitions also check mac_grant, so a
withdrawn grant stalls an unissued batch. This guard does not replace the
arbiter's ownership guarantee while issued operations are completing.

Do not grant this generator during a live dot-product sum that must be retained:
it clears all 11 lane accumulators. Other clients must finish or save their sums
before ownership transfers. The unfinished SQUKF FSM has not been integrated.

Pulse start while busy is low. State, S, and gamma are captured at acceptance.
The generator initializes a private sigma buffer, waits for a grant and an idle
array, then issues 15 products in packed upper-triangle order:

1. All 11 lanes: S00,S01,S02,S03,S04,S11,S12,S13,S14,S22,S23.
2. Lanes 0 through 3: S24,S33,S34,S44.

Every issued lane gets clear=1 and last=1 to compute a single product using the
wide accumulator. Completions are tracked per lane.
MAC_accumulator samples clear and last with start; clear selects a zero base
for the captured product, so asserting all three together is supported.
Dedicated WIDTH+1-bit
add/subtract logic forms both signs, with saturation to WIDTH bits.
The complete sigma buffer publishes together; old sigma outputs hold during
work. done pulses for one clock and busy drops on publication. Results remain
until the next completion. Starts while busy are ignored.

Product rounding follows MAC_accumulator: nearest, ties away from zero.
Offsets are returned at WIDTH bits; product clipping or final addition/subtraction
clipping sets transaction overflow. A clipped offset can affect both signs,
even if one mathematical final value would fit. Treat overflow as an invalid
or degraded transaction according to the filter's recovery policy, and choose
a wider data format if valid gamma/S/state bounds can exceed the current range.
Overflow clears at the next accepted start or reset.

Reset is synchronous active low. Reset the generator and array together to abort
work. If resetting only the generator, drain the array before granting it again.
No RAM port assumptions are made here: input registers capture 25 S entries and
five state components together. A later RAM-backed integration needs a load
stage with sufficient bandwidth before asserting start.

## Simulation

From the workspace root:

```bash
python3 tests/test_sigma_generator.py
```

This compiles the generator and real shared MAC array, checks 129 complete sigma
sets against a Python integer model across FRAC=0 and fractional configurations,
and checks input capture, ignored busy starts, delayed and withdrawn grants, batch masks,
saturation flags, done pulses, retained outputs, and reset/recovery.
Target FPGA area, clock timing, arbitration among clients, and full-filter
numerical accuracy require integration and synthesis validation.
