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
| B. Fused, implemented | Keeps the exact product through alignment and rounds the combined value once. | Uses wider alignment registers and a wider shared adder, with one rounding and packing pass. Iterative normalization and alignment give operand-dependent latency. |

The lane uses 24 shift-and-add multiply clocks with one shared 52-bit significand adder for multiplication, addition, subtraction, and rounding. A negative difference uses another cycle on the same adder to take its magnitude. Alignment and normalization use one-bit shifts. A right shift jams all discarded bits into the least significant bit. The exact product has up to 48 significant bits and begins with at least three low padding bits. Close cancellation occurs with an exponent gap of at most one, where alignment is exact. For wider gaps, cancellation cannot expose the jam bit as a significant result bit, and the rounding logic uses the guard bit and the reduction of all lower bits.

Exceptional and zero-product shortcuts complete one clock after acceptance. The reference regression reports the longest latency it observes. Large exponent gaps are handled by iterating until the smaller term has become a sticky bit, then skipping the remaining shifts. The build uses no multiplier primitive or FPGA DSP blocks. The area comparison is architectural. SKY130 library area and clock timing require mapping and timing analysis with the target library.

## Verification and synthesis

From this directory:

```sh
make test
make synth-check
make synth
make synth-sky130 SKY130_LIBERTY=/path/to/sky130.lib
```

`make test` uses Python 3 and Icarus Verilog. The Python oracle computes the finite fused result as a signed arbitrary-precision integer in units of 2⁻²⁹⁸ and rounds that exact result to FP32. It checks result bits and all flag bits for directed and seeded random operands. The testbench also checks result holding, a long accumulation, idle clear, reset during execution, and requests attempted while busy. `make synth-check` runs Icarus synthesis elaboration. The Yosys targets check for latches and inferred multiply, divide, and variable-shift operators before generic synthesis and optional SKY130 mapping.

## Fixed-point dependencies

| Archived RTL | Existing users |
| --- | --- |
| `MAC.sv`, `MAC_array.sv` | Standalone arithmetic tests. The array instantiates `MAC_accumulator` per lane. |
| `MAC_accumulator.sv` | MAC array and the shared MAC interfaces exercised by the sigma generator and process model testbenches. |
| `cordic.sv` | Both SR-UKF filters, the process-model testbench, and the historical precision studies. |
| `SQRT.sv`, `division.sv` | Standalone tests and earlier prototype interfaces. The old `SQUKF.sv` scaffold references `division`. |
| `srukf_arithmetic.sv` | Both SR-UKF filters and their arithmetic testbench. Provides fixed-point MAC, full root and full division. |
| `srukf_filter.sv`, `srukf_bias_filter.sv`, `srukf_400hz_scheduler.sv`, `process_model.sv`, `sigma_generator.sv` | Legacy UKF integration or verification. |

The active Makefile and synthesis sources select only `fp32_mac`. Historical scripts under `legacy/fixed-point/tests` use the archived sources. The sensor FIFO scheduler in `ukf_fpga.sv` exposes datapath handshakes and remains at its original path.
