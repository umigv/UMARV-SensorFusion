# Shared FP32 division and square root

[fp32_divsqrt.sv](fp32_divsqrt.sv) performs division in mode 0 and square root in mode 1. It uses the same start/busy/done convention as the FP32 MAC. At a rising edge with `start=1` and `busy=0`, it samples the operation and its operands. Square root uses only `a`. Requests sampled while busy are ignored, including at the completion edge. On completion, `busy` goes low, `done` pulses for one cycle, and `result` and all five exception flags update together. The result and flags hold until the next completion. The next rising edge can accept a new request. Active-low asynchronous reset cancels pending work and clears the result, flags, and handshake.

## Arithmetic

Both modes normalize nonzero subnormal inputs by one-bit shifts and decrement the corresponding signed exponent. Division compares the normalized significands and, if the numerator is smaller, doubles it and reduces the result exponent by one. Square root doubles the significand for odd exponents and uses the floor of half the exponent. Either adjustment produces a result significand in [1, 2).

The radix-2 restoring recurrence produces one bit per cycle for 26 cycles. Division tries to subtract the denominator and shifts the accepted or restored remainder by one. Square root shifts the remainder by two, brings in the next radicand pair, and tries to subtract four times the partial root plus one. Both modes select the tentative difference when the shared carry chain reports no borrow. A failed trial selects the original remainder through a mux.

A single 28-bit datapath with carry-in and carry-out handles the trial subtraction for either mode, the initial division comparison, and the rounding increment. The modes share the remainder register, digit register, counter, and round/pack path. Square root reuses the operand registers to stream radicand pairs. Compared with independent restoring engines, the implementation removes the separate division trial subtractor and second set of recurrence registers. It adds operand-selection muxes. Net cell-area savings need technology mapping. Non-restoring alternatives can avoid the restore mux, but require signed partial remainders, add/subtract control and terminal correction or recoding. The restoring approach uses a single unsigned recurrence with a common remainder-to-sticky path.

The digit stream provides 24 retained bits, a guard bit and a round bit. A nonzero final remainder sets sticky. When the division result is subnormal, right shifts jam discarded bits into sticky before rounding. Round-to-nearest, ties-to-even increments on a set guard bit with either a nonzero tail or an odd retained word. Tininess is detected after rounding. An exact subnormal raises neither underflow nor inexact. A positive, nonzero binary32 square root always lies in the normal binary32 range.

## Exceptional values

| Condition | Result and flags |
| --- | --- |
| Quiet NaN or signaling NaN in a used operand | Canonical quiet NaN `0x7fc00000`. Any signaling NaN raises `invalid`. |
| Division of zero by zero or infinity by infinity | Canonical quiet NaN and `invalid`. |
| Finite nonzero dividend divided by signed zero | Signed infinity and `divide_by_zero`. |
| Infinite dividend divided by a finite operand, including zero | Signed infinity. |
| Finite dividend divided by infinity, or zero divided by finite nonzero | Signed zero. |
| Square root of a negative nonzero value, including −∞ | Canonical quiet NaN and `invalid`. |
| Square root of ±0 or +∞ | Input value, including the sign of zero. |
| Rounded finite division overflows | Signed infinity with `overflow` and `inexact`. |
| Inexact result tiny after rounding | Subnormal or signed zero with `underflow` and `inexact`. |

Division uses the XOR of operand signs for its signed results.

## Verification and measured latency

Run `make test synth-check` for both FP32 regressions and Icarus synthesis elaboration. Run `make test-divsqrt synth-check-divsqrt` for this unit alone. The testbench compares every result word and each exception bit against native IEEE binary32 division or `sqrtf` with C floating-point exception reporting. The harness selects ties-to-even, canonicalizes NaN payloads, and applies after-rounding tininess to host underflow reports. It checks reset during both modes, held outputs, attempted requests through every busy edge, and acceptance at the first idle edge after completion.

These are the measured acceptance-to-completion clock counts with Icarus Verilog 12.0 and GCC 15.2. Each mode ran 12,000 seeded randomized valid operations and 162 directed operations.

| Mode | Operations | Min / max / mean cycles | Recurrence path min / max / mean cycles |
| --- | ---: | --- | --- |
| Division | 12,162 | 1 / 81 / 33.792 | 31 / 81 / 34.086 |
| Square root | 12,162 | 1 / 54 / 31.045 | 31 / 54 / 31.286 |

Icarus synthesis elaboration reports 173 register bits in 19 register groups and no latches. The recurrence path from the remainder/digit registers through input selection, the shared carry chain, and the restore mux is a candidate timing path. SKY130 cell area, actual critical path, and maximum frequency require technology mapping and static timing analysis. Yosys, a SKY130 Liberty library, and a static timing tool were unavailable in this workspace. With those available, `make synth-divsqrt` provides generic synthesis and `make synth-divsqrt-sky130 SKY130_LIBERTY=/path/to/sky130.lib` provides a mapped area report. The cycle measurements apply to the arithmetic unit in isolation. Verification of the 1.25 ms phase budgets requires the integrated SR-UKF.
