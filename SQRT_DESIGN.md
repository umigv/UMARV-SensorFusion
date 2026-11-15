# SR-UKF square-root datapath

`SQRT.sv` provides module `SQRT`, a sequential restoring square root using
shifts, registers, and one subtractor. It uses no multiply, divide, or real-valued
square-root operators. FPGA area and clock frequency require target synthesis.

The signed input `a` and signed nonnegative output `result` share `WIDTH` total
bits and `FRAC` fractional bits (defaults: 32 and 16). The exact raw result is
`floor(sqrt(a_raw << FRAC))`, so error is less than one output LSB. Supported
parameters are `WIDTH >= 2` and `0 <= FRAC < WIDTH`; the nonnegative result fits
in the signed output without saturation. For example, 4.0 encoded as 262144
returns 2.0 encoded as 131072 at the default format.

Pulse `start` with the operand valid while `busy` is low. The operand is latched;
starts while busy are ignored. Nonnegative inputs complete after
`ceil((WIDTH + FRAC)/2)` clocks following acceptance: 24 clocks by default.
Zero follows the same latency. A held-high start can issue again once idle.

A negative radicand has no real square root: it completes on the acceptance
edge with `result=0`, `negative_input=1`, `busy=0`, and `done=1`. Successful
completion sets `negative_input=0`. `done` pulses for one cycle; the result and
flag persist until another completion. Synchronous active-low `resetn` clears
outputs and aborts any operation.

Example connection, with operand routing and signals declared in the parent:

```systemverilog
SQRT #(.WIDTH(32), .FRAC(16)) sqrt_unit (
    .clk(clk), .resetn(resetn), .start(start_sqrt),
    .a(sqrt_operand), .result(sqrt_result),
    .busy(sqrt_busy), .done(done_sqrt),
    .negative_input(sqrt_negative)
);
```

Sample the fault with `done_sqrt`. During covariance factorization or a Cholesky
downdate, a negative radicand indicates that the requested real factor cannot
be computed. The controller must reject that update or apply its chosen recovery
policy; zero is a deterministic error output, not a valid factor for that input.
A zero radicand is accepted, but may still produce a singular factor that the
filter must handle separately.

This unit is standalone. The existing scheduler has SQRT start/done ports but
no operand routing or arithmetic fault recovery; the unfinished filter wrapper
has not been wired to this unit.

Run `python3 tests/test_sqrt.py` with Icarus Verilog installed. The tests compare
3,836 inputs with Python's integer square root, including exhaustive small-width
cases, odd scaling widths, 32-bit edge/random cases, flags, latency, held outputs,
competing starts, and reset during operation. Full-filter accuracy and FPGA
timing/resource utilization are not covered.
