# Fixed-point division for this SR-UKF

## Current implementation

`division.sv` is a shared, signed, sequential restoring divider. It uses a
WIDTH+1-bit subtractor, shift registers, and control; it requests no DSPs or RAM.
Actual LUT count and maximum clock frequency require synthesis on the target FPGA.

All operands have the same fractional width. The default is 32 total bits,
including sign, and 16 fractional bits. Set FRAC to match your real data format.
The mathematical raw output is trunc_toward_zero((a_raw * 2^FRAC) / b_raw).
The implementation extends the numerator before shifting, preserving those bits.
Overflow saturates and raises overflow. Division by zero saturates according to
the numerator sign (0/0 gives positive maximum), raises div_by_zero, and leaves
overflow low. The caller must handle the fault flags.

Pulse start while busy is low, with a and b valid at that rising edge. Inputs are
latched; starts while busy are ignored. Normal completion is WIDTH+FRAC clocks
after acceptance. Divide by zero completes on acceptance. done is a one-cycle
pulse; c and flags hold until another completion. Reset is synchronous active low.
A held-high start can trigger another operation once idle; use a pulse.

Example instance inside a datapath, with signals declared by the parent:

```systemverilog
division #(.WIDTH(32), .FRAC(16)) div (
    .clk(clk), .resetn(resetn), .start(start_div),
    .a(div_numerator), .b(div_denominator), .c(div_result),
    .busy(div_busy), .done(done_div),
    .div_by_zero(div_zero), .overflow(div_overflow)
);
```

This is a standalone unit. SQUKF.sv and schedular.sv are unfinished skeletons;
ukf_fpga.sv has no division handshake or operand-routing datapath yet. They have
not been wired to this divider. In particular, S8_GAIN's two MAC micro-ops are
placeholders, not complete triangular solve implementations.

## Architecture choice

| Approach | Resources | Appropriate use |
| --- | --- | --- |
| Sequential restoring division (implemented) | LUTs and registers; no DSP required | Modest operation count, preserve MAC availability, exact truncated quotient |
| Normalized reciprocal + Newton iteration + multiply | Seed ROM, normalization shifts, shared DSP multiplications | Many numerators share a denominator; higher throughput required |
| Pipelined divider IP | More hardware; depends on architecture | Sustained stream of independent divisions |
| Linear vectoring CORDIC | Shift/add datapath with suitable modes and range reduction | Existing custom CORDIC explicitly supports linear vectoring |

A conventional circular CORDIC for sin/cos or vector magnitude/angle cannot
simply be switched to division unless linear vectoring is implemented. Standard
AMD CORDIC IP does not expose a general division function. A square-root block
alone also does not provide reciprocal or division.

For a clock of 100 MHz, the default divider takes 0.48 microseconds per operation,
excluding issue/control overhead. That is an example budget, not a measured clock
frequency. Count total operations per filter update before choosing more hardware.

## SR-UKF optimization

Keep the covariance factors triangular and use triangular solves for the gain;
do not construct a dense matrix inverse. Each solve contains terms of the form:

    x_i = (rhs_i - sum_j(L_ij * x_j)) / L_ii

The sum belongs on the MAC. A shared divider can handle the final division.
If many right-hand sides use the same diagonal, compute r_i = 1/L_ii once and
cache it for that factor, then use MAC multiplications for all those divisions.
Invalidate the cached reciprocals when the factor changes. Repeated solves with
the factor and its transpose can reuse those diagonal reciprocals.

For a faster reciprocal engine, normalize abs(b) = m*2^e with 1 <= m < 2,
seed r approximately 1/m from a small ROM, iterate r <- r*(2-m*r), and restore
the exponent and sign. Each iteration uses two multiplications; the final quotient
uses another multiplication. Seed precision and internal guard bits determine
iteration count and accuracy. A 32-bit multiplication may use multiple DSP slices
or several cycles, depending on the FPGA and existing MAC width. Measure resource
use and latency rather than assuming one multiplication means one DSP or cycle.
An approximate reciprocal does not guarantee the exact truncated quotient.

Do not force reciprocals into the same Q format as the signal: small diagonal
values may have large reciprocals. Use normalization/exponent tracking or prove
bounds for a separate reciprocal format. Monitor tiny/nonpositive covariance
factor diagonals and route failures to a defined filter recovery policy. The
arithmetic unit only detects exact zero and representational overflow.

## Validation

Run `python3 tests/test_division.py` with Icarus Verilog installed. It checks
3,620 divisions against a Python integer reference, exhaustive 4-bit cases and
32-bit edge/random cases at FRAC=0,16,31, including latency and done pulse width.
Full-filter numerical behavior, synthesis timing/area, reset during a transaction,
and arbitration with other datapath clients are not validated by this test.

References:
- AMD Divider Generator: https://docs.amd.com/r/2021.1-English/ug1483-model-composer-sys-gen-user-guide/Divider-Generator-5.1
- AMD CORDIC supported modes: https://www.amd.com/content/dam/xilinx/support/documents/ip_documentation/cordic/v6_0/pg105-cordic.pdf
