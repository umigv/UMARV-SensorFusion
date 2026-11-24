# Single-lane FP32 MAC

[fp32_mac.sv](fp32_mac.sv) implements `acc = roundToNearestEven(a × b + acc)`. Each operation starts from an FP32 accumulator and rounds once before storing the next FP32 accumulator. For example, with `acc = -1`, `a = 1 + 2⁻²³`, and `b = 1 - 2⁻²³`, the lane returns `-2⁻⁴⁶`. A separately rounded product would be 1 and yield zero.

## Interface

Inputs are sampled on rising clock edges while `busy = 0`. Active-low asynchronous reset discards an operation, clears the accumulator, result and exception flags to zero, and returns the lane to idle.

| Inputs sampled while idle | Effect |
| --- | --- |
| `clear = 1, start = 0` | Initialize the accumulator to +0. |
| `clear = 0, start = 1` | Accept `a`, `b`, and `last` and use the current accumulator. |
| `clear = 1, start = 1` | Accept `a`, `b`, and `last` and use +0 as the addend. |
| `clear = 0, start = 0` | Stay idle. |

`last` is sampled with `start`. It requests result publication and leaves the completed value in the accumulator. Use `clear` with the first request of a new sequence or on a preceding idle edge. While busy, the lane ignores requests including requests present at the completion edge. A request held high through the next idle sampling edge is accepted on that edge.

`done` pulses for one cycle at each commit. `result_valid` pulses with `done` if the accepted request carried `last`. `result` holds the most recent published value. The lane can accept a request at the rising edge following the completion edge.

`exception_flags[4:0]` are **invalid, divide by zero, overflow, underflow, inexact**, in descending bit order. They update on every `done` and hold between completions. They report the completed operation. A clear-only request affects the accumulator.

## Exceptional values

- Signaling NaNs, zero multiplied by infinity, and an infinite product added to an opposite-signed infinite accumulator return canonical quiet NaN `0x7fc00000` and raise invalid. The zero-times-infinity rule also applies with a quiet NaN accumulator.
- Quiet NaNs propagate as the same canonical quiet NaN. Infinite operands otherwise yield the appropriate signed infinity. Divide by zero is always zero.
- Exact cancellation of nonzero terms yields +0. The sum of two signed zeros yields −0 only when both zeros are negative. A nonzero result rounded to zero retains its sign.
- Rounding uses round-to-nearest, ties-to-even, including gradual underflow. Underflow is raised for an inexact result that is tiny after rounding, including rounded zero. An exact subnormal raises neither underflow nor inexact.
- An out-of-range rounded result yields signed infinity and raises overflow and inexact. Finite product overflow or underflow is evaluated only in the context of the final fused result.

## Area and latency

| Option | Precision | Area and timing trade-off |
| --- | --- | --- |
| A. Non-fused | Rounds the product and then rounds the addition. Intermediate product overflow and underflow can change the final value. | FP32 addition can use a narrower significand datapath after product rounding. An iterative lane still needs the exact multiplication state and goes through the rounding and packing path twice. |
| B. Fused, implemented | Keeps the exact product through alignment and rounds the combined value once. | Uses wider alignment registers and a wider shared adder, with one rounding and packing pass. |

The production lane uses Architecture A's 13-cycle Radix-4 Booth multiply. A single narrow adder commits two product bits per ordinary step and applies the final unsigned correction digit. The 52-bit adder for add, subtract, negate and round uses four-bit local ripple groups and a four-level parallel prefix between groups. Hierarchical leading-zero detection normalizes the product and accumulator. A barrel shifter aligns significands and jams discarded bits into the least significant bit. The exact product has up to 48 significant bits and begins with three low padding bits. Close cancellation occurs with an exponent gap of at most one, where alignment is exact. With wider gaps, cancellation cannot expose the jam bit as a significant result bit.

Exceptional and zero-product shortcuts complete one clock after acceptance. Other reachable paths complete in 19, 21, or 22 cycles. Large exponent gaps saturate directly to a sticky bit. The build uses no multiplier primitive or FPGA DSP block. The matched SKY130 mapping and the legacy restore procedure are recorded in the integration report. Physical timing remains unverified. [Radix-4 integration](RADIX4_MAC_INTEGRATION.md)

## Verification and synthesis

From this directory:

```sh
make test
make synth-check
make synth
make synth-sky130 SKY130_LIBERTY=/path/to/sky130.lib
```

`make test` uses Python 3 and Icarus Verilog. The Python oracle computes the finite fused result as a signed arbitrary-precision integer in units of 2⁻²⁹⁸ and rounds that exact result to FP32. It checks result bits and all flag bits for directed and seeded random operands. The testbench also checks result holding, a long accumulation, idle clear, reset during execution, and requests attempted while busy. `make synth-check` runs Icarus synthesis elaboration. The Yosys targets reject inferred latches, multipliers and dividers before generic synthesis. Normalization and alignment shifts must lower to logic during generic synthesis.
