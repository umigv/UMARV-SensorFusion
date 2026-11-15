# Wide dot-product accumulation

Use `MAC_accumulator.sv` for sums of products such as weighted means. Keep
`MAC.sv` for individual `a*b+c` operations. `MAC_array.sv` instantiates 11
independent wide accumulators by default, using a generate loop. It exposes
per-lane start, clear, last, busy, done, result_valid, overflow, and acc_overflow.
The old array c input is removed: each lane stores its own running sum.
LANES and ACC_WIDTH are configurable. The array is not yet wired into the FSM.

The default inputs are signed 32-bit values with 16 fractional bits. Products
retain all 64 bits and 32 fractional bits. The internal sum defaults to 72 bits.
Set ACC_WIDTH to at least `2*WIDTH + ceil(log2(N))` for at most N full-range
products (and always at least 2*WIDTH). More terms can be safe with proven
operand bounds. FPGA resources and timing still require target synthesis.

For every term, when busy is low:

1. Set a and b, and pulse start for one clock.
2. Set clear on the first term; it replaces the old sum with this product.
3. Set last on the final term; it includes this product and publishes result.
4. Wait for done before issuing the next term.

clear and last can both be set for a single-term sum. All controls and operands
are captured with start. Acceptance sets busy; completion is one clock later.
Starts while busy are ignored. done pulses on every completed term;
result_valid pulses only on final terms. Output result and overflow hold between
final terms. No explicit c input is needed: the sum is stored internally.
A start must include a product, so finalize using a zero product if needed.

Only finalization reduces precision: round to nearest with ties away from zero,
then saturate to the signed WIDTH-bit output range. A final term does not clear
the sum; use clear to begin the next independent dot product.

acc_overflow is sticky on internal accumulator overflow and resets on completion
of a term marked clear, or synchronous active-low reset. Internal overflow wraps;
an affected sum is invalid. Final overflow flags either this accumulated fault
or output saturation. Widen ACC_WIDTH if internal overflow occurs. Reset aborts
pending work and clears the sum, outputs, and flags.

For covariance terms with three factors, this module sums products of two
inputs. Any preceding weight/deviation multiplication needs its own width
analysis; the wide sum cannot recover precision already discarded upstream.

Run `python3 tests/test_mac_accumulator.py` from the workspace root. Tests compare
against Python integer arithmetic and cover rounding, cancellation, output
saturation, sticky internal overflow, starting a new sum, latched inputs/controls,
completion timing, held outputs, and reset during work.

Run `python3 tests/test_mac_array.py` for parallel array verification at 2, 5,
and the default 11 lanes. Each configuration checks 500 cycles against separate
Python lane models, including cancellation, retained fractions, independently
timed requests, ignored starts while busy, final-result pulses, and reset.
Compile the array with both `MAC_array.sv` and `MAC_accumulator.sv`.
