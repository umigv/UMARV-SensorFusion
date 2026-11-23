# Mixed-precision four-state SR-UKF

The synthesizable entry point is `srukf_filter` in [srukf_filter.sv](legacy/fixed-point/srukf_filter.sv). Build it with `srukf_400hz_scheduler.sv`, `srukf_arithmetic.sv`, and the existing `cordic.sv`. It combines the 400 Hz scheduling and two-pass prediction architecture from the staged prototype with the rounding boundaries from the successful integer reference.

## Interface and scheduling

The lower 32 bits of a packed vector hold lane zero. State and VN-300 sample lanes are `x, y, theta, omega`, with signed fractional widths `21, 21, 23, 25`. Wheel velocity is signed F21 in m/s. Initialization, process, measurement and published factors use signed F25, lower triangular, in row-major order. Slots above the diagonal are ignored on input and zero on output. The process and measurement inputs can contain lower-triangular off-diagonals.

The packet producer converts sensor values to these formats using nearest rounding with ties away from zero. The producer supplies continuous, prediction-aligned VN-300 heading, as in the software comparison and staged interface. It must resolve the heading revolution before rounding to signed F23. The filter consumes the complete aligned four-state packet on the correction boundary. An output timestamp accompanies each accepted prediction or correction. State and factor publish together.

Initialization establishes the timestamp epoch. Prediction starts 1.25 ms after the previous correction epoch and targets the next 2.5 ms timestamp. Correction starts at that timestamp when a matching packet is present. The scheduler emits `5368709`, the rounded Q1.31 value of 0.0025 s, and checks that the elapsed timestamp is exactly 2,500 µs. It leaves a future packet queued and drains stale ones. Prediction continues through a missing packet. At 50 MHz, each phase gets 62,500 clocks. A missed deadline rejects publication and sets a sticky fault.

The filter makes two passes over the nonlinear nine-point prediction. The mean's central weight is zero for the validated tuning and the first pass therefore evaluates only the eight contributing points. The second pass evaluates those eight points and the center, then applies the center as a Cholesky update after the process-noise rows. No bank of propagated points is required. The parameters are the tested `alpha=1, beta=2, kappa=0`, using exact mean weights of 1/8 and F27 covariance weights `47453133` and `189812531` (the two square-root weights). One MAC, one square root, one divider and one CORDIC serve the complete filter. The mixed controller sequences sigma generation and motion directly on those units. The standalone legacy sigma generator, process model, MAC, divider and square-root interfaces remain usable by their existing callers. Availability inputs can delay the MAC and CORDIC up to the slot deadline.

## Fixed-point conversions

F denotes the number of fractional bits. Raw products are signed and retained in a 72-bit accumulator. The MAC has a 73-bit sign/range check at accumulation. Between products within a dot, shifts align to the **largest** number of fractional bits, so alignment never discards product bits. Conversions below round nearest with ties away from zero except the integer square root and division, which floor the nonnegative root and truncate the signed quotient toward zero.

| Calculation | Raw format and conversion |
| --- | --- |
| Sigma points | Twice the signed F25 factor component, shifted right with rounding by 4 for XY, 2 for heading or 0 for yaw rate. Add or subtract in the corresponding state format, checking both the displacement and resulting state. |
| Wheel displacement | F21 wheel times Q1.31 time is F52. Round right by 31 to F21. |
| Midpoint and final heading | F25 yaw rate times Q1.31 time is F56. Round the original product by 34 and 33 respectively to F23, then add the original F23 heading. |
| CORDIC and XY motion | CORDIC takes and returns F23, using 27 iterations and F34 internally. Displacement times sine/cosine is F44. Round right by 23 and add the F21 position. |
| Prediction mean | Sum eight state words in a signed 35-bit accumulator per lane. Round right by three in the lane's own format. |
| Deviation row | Multiply same-format deviation by the F27 square-root weight. Round right by 23 for XY, 25 for heading, or 27 for yaw rate to get F25. |
| QR, Cholesky update and downdate norms | Combine both F50 squares with no rounding or narrowing before the square root. Use an unsigned restoring integer root only after checking the signed radicand. Output F25. Reject a nonzero-required zero pivot, negative radicand or out-of-range root. An empty QR diagonal uses the absolute pivot and signed row copy, as in the reference. |
| QR ratios and rank-one ratios | Shift a signed F25 numerator left by 27 and divide by an F25 pivot to obtain F27. Trailing two-term products use F52 accumulation and one final shift by 27 for F25. Rank-one updates/downdates use the reference's rounded F25 trailing numerator divided by the F27 coefficient. |
| First triangular solve | Accumulate the F50 cross product and subtract F50 substitution products in the same accumulator. Divide the raw numerator by the F25 innovation pivot to produce an F25 entry of U. No F30 cross-covariance word is stored. |
| Second triangular solve | Accumulate the F25 U entry (times exact F27 unity) and subtract F52 substitution products. Divide the raw numerator by the F25 pivot to obtain F27 gain. |
| State correction | Subtract each sample in its state format. Left-align the four F48/F50/F52 products to F52, accumulate, then narrow once by 31, 29 or 27 to the destination state lane. |

Range failures set sticky overflow/saturation and suppress partial commits. Negative input to the square root sets both the negative-radicand and invalid-pivot indications. A missing pivot or divide by zero sets invalid pivot. A raw MAC accumulator overflow sets overflow. All numeric faults halt scheduling until reset.

## Reproduction

From the workspace root, build the fixture generator once and run the Icarus tests.

```sh
g++ -std=c++17 -O3 -Wall -Wextra -Werror tests/srukf_precision/study.cpp -o /tmp/srukf-mixed-ref
python3 UMARV-SensorFusion/legacy/fixed-point/tests/test_srukf_mixed.py quiet 72 --study /tmp/srukf-mixed-ref --stalls
python3 UMARV-SensorFusion/legacy/fixed-point/tests/test_srukf_mixed.py thin 1024 --study /tmp/srukf-mixed-ref --skip-idle --stalls
python3 UMARV-SensorFusion/legacy/fixed-point/tests/test_srukf_mixed.py dropout_short 800 --study /tmp/srukf-mixed-ref --skip-idle --stalls
python3 UMARV-SensorFusion/legacy/fixed-point/tests/test_srukf_mixed.py dropout_coast 12400 --study /tmp/srukf-mixed-ref --skip-idle --stalls
python3 UMARV-SensorFusion/legacy/fixed-point/tests/test_srukf_mixed.py dropout 12064 --study /tmp/srukf-mixed-ref --skip-idle --stalls
```

The fixture generator replays the precision study's seed 17. It packs each input in its validated format and computes expected prediction and correction state/factor words separately. The test checks all 20 words at every commit. With `--skip-idle`, the test advances the slot counter across **idle** clocks only. All active phase clocks, deadlines and resource stalls still execute in RTL. The 72-step quiet replay uses the full uncompressed schedule.

```sh
cd UMARV-SensorFusion/legacy/fixed-point
iverilog -g2012 -s tb_srukf_arithmetic -o /tmp/srukf-arithmetic srukf_arithmetic.sv tests/tb_srukf_arithmetic.sv
vvp /tmp/srukf-arithmetic
iverilog -g2012 -s tb_srukf_faults -o /tmp/srukf-faults srukf_filter.sv srukf_400hz_scheduler.sv srukf_arithmetic.sv cordic.sv tests/tb_srukf_faults.sv
vvp /tmp/srukf-faults
```

The arithmetic test covers the old raw QR sum, a positive near-cancelled downdate, a negative radicand, root and quotient saturation, divide by zero, signed MAC alignment and a wide solve numerator representative of reacquisition. The integrated fault test covers invalid initialization, state saturation, a singular downdate and a blocked-resource deadline. It checks that failed phases preserve the previous state, root and timestamp.

## RTL results

The original staged prototype reproduced the quiet-case fault after 69 successful predictions and 69 corrections with no missed deadline. Its fourth QR pivot had raw F28 inputs 8,791 and −63. Hardware formed a nonzero squared sum of 77,285,650, then rounded the root input to zero on the F28 boundary. The mixed RTL ran through the failure point and beyond. For the same sensor and truth sequence, every committed mixed-format state and factor word matched the software reference.

| Icarus regression | Predictions | Corrections | Max prediction clocks | Max correction clocks |
| --- | ---: | ---: | ---: | ---: |
| Quiet failure replay, full staggered schedule | 72 | 72 | 5,290 | 7,727 |
| Extended quiet | 2,048 | 2,048 | 6,139 | 9,009 |
| Nearly singular factor | 1,024 | 1,024 | 9,497 | 10,373 |
| Tight turns | 800 | 200 | 9,213 | 10,340 |
| Position crossing the former 512 m limit | 2,500 | 625 | 10,013 | 10,350 |
| Continuous 1-second dropout and reacquisition | 800 | 400 | 10,057 | 10,350 |
| Packet loss onset, checkpoint start | 320 | 120 | 9,904 | 10,335 |
| Constant-turn dropout tail and reacquisition, checkpoint start | 144 | 80 | 10,259 | 10,339 |
| Variable-turn dropout tail and reacquisition, checkpoint start | 128 | 64 | 10,259 | 10,336 |

The packet-loss replay starts 120 predictions before the first missing packet. The two reacquisition replays start 64 predictions before the first returning packet. Each checkpoint supplies the software state and factor at the selected start. The test delays requests with periodic MAC and CORDIC availability stalls. The slot-start-to-commit counts include those stalls, dependencies, factor updates, and state/factor publication. The original repository's CORDIC and Euler process-model tests also matched all 1,220 CORDIC and 105 process-model records in the separate bit-exact study probe.

## Synthesis scope

The restoring square-root and divider cores synthesized individually with Icarus's generic BLIF backend. These numbers are unoptimized logic-node counts and logic depth, with no physical cell library.

| Core | Nonconstant BLIF logic nodes | Sequential bits | Longest combinational path |
| --- | ---: | ---: | ---: |
| Full-precision square root | 1,407 | 187 | 51 logic levels |
| Full-precision divider | 2,052 | 179 | 76 logic levels |

The block netlists can be reproduced with:

```sh
iverilog -g2012 -t blif -s srukf_full_root -o /tmp/srukf-root.blif srukf_arithmetic.sv
iverilog -g2012 -t blif -s srukf_full_div -o /tmp/srukf-div.blif srukf_arithmetic.sv
```

There are 4,918 declared storage bits in the complete design at 50 MHz before synthesis optimizations, including constant, redundant and unused lanes. The shared MAC has one signed 32×32 multiplier. The root uses a 39-bit trial subtraction with a 72-bit radicand shift register. The divider scans a 72-bit numerator/quotient with a 33-bit restoring subtractor. The root and divider use 36 and 72 iterative clocks respectively per normal launch, with interface/dispatch overhead measured in the complete-phase tests. The 56-bit/F28 prototype used 42 root iterations, 84 divider iterations and a 57-bit divider subtractor. The mixed design adds binary-point alignment and checked rounding logic around the shared MAC. Compared with the prototype, the reference’s rank-one algorithm also adds one shared division per trailing entry, up to six extra launches per update or downdate row (432 divider iterations plus dispatch).

The Icarus synthesis backend rejects the complete design's variable-index memory writes and some CORDIC and multiplier primitives. Yosys, ABC and device tools were unavailable. The package download attempt failed on DNS resolution. Run `yosys -s tests/synth_srukf.ys` from `legacy/fixed-point` with Yosys available to obtain a complete generic logic count and path depth. Technology-mapped cell area, FPGA DSP/LUT use and a critical path in nanoseconds require a target technology and appropriate synthesis/timing tools. The cycle tests use a 50 MHz simulation clock and do not establish physical closure at that clock.
