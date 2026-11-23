# Sigma-point generation

`sigma_generator.sv` captures the state in order `[x, y, theta, omega]`,
the ten upper-triangular entries of `S`, and `gamma` on a rising edge with
`start` asserted while idle. Forward velocity belongs to the process model.
Data is signed fixed point and defaults to `WIDTH=32`, `FRAC=16`.

For `P = S^T*S` and indices `i, j = 0..3`, the output is:

- `sigma[0][j] = state[j]`
- `sigma[i+1][j] = state[j] + gamma*S[i][j]`
- `sigma[i+5][j] = state[j] - gamma*S[i][j]`

Each offset uses a row of `S`. When `j < i`, the offset is zero and the
corresponding lower-triangular input is ignored.

## Shared MAC

Wire the generator's scalar `mac_*` signals to one shared
`MAC_accumulator` with matching `WIDTH` and `FRAC`. The generator uses
`result_valid` for completion. With multiple clients, the arbiter must route
controls and operands to the MAC and return status and results to its owner.
Grant only after the previous client has finished its sum and both `busy`
and `result_valid` have cleared. Hold grant until `mac_request` drops.
For a single client, `mac_grant` can be tied high.

The generator issues ten products in row-major order
`S00, S01, S02, S03, S11, S12, S13, S22, S23, S33`.
The MAC samples `clear` and `last` with `start` to finalize a one-term
sum. The generator waits for a granted, idle MAC with no pending valid,
then observes the launched operation go busy before accepting its result.
It stores each offset before issuing the next product.

Product rounding is nearest with ties away from zero. The MAC saturates
products, and the generator saturates the final sums and differences.
`overflow` is sticky across the transaction and clears on the next accepted
start or reset. A completed transaction with overflow publishes the clipped
values and flags them for the caller. All nine points publish on the same
edge as the one-cycle `done` pulse and the release of `mac_request`.

`MAC_TIMEOUT_CYCLES` defaults to 16 and applies separately to each launched
product. Set it to at least 2 with the current MAC, which takes two clocks
after launch for the generator to observe `result_valid`. A result on the
deadline succeeds. Grant and idle waits occur before the watchdog starts.
On timeout, `error`, `busy`, and `mac_request` stay asserted and published
sigma points hold until reset. Use synchronous active-low reset on both the
generator and MAC to abort and recover.

## Simulation and integration

From the workspace root, run:

```bash
python3 UMARV-SensorFusion/legacy/fixed-point/tests/test_sigma_generator.py
python3 tests/test_sigma_generator.py
```

The software-reference tests use dense upper-triangular matrices and check
all 36 output words across multiple fixed-point formats. The SystemVerilog
benches exercise a competing MAC client, sequential launches, completion
timing, watchdog faults, and active resets.

The generator takes 44 clocks from accepted start to publication with a
grant ready and the current MAC latency. Arbitration stalls add to that
count. Production arbitration and scheduler wiring, Cyclone V synthesis and
timing closure, and the full filter's 1 kHz budget remain to be verified.
