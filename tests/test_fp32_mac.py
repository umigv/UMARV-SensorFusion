"""Generate exact FMA results and check the RTL with Icarus Verilog."""
import os
from pathlib import Path
import random
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
INF = 0x7F800000
NAN = 0x7FC00000
SIGN = 1 << 31


def magnitude(word):
    return word & (SIGN - 1)


def finite_integer(word):
    """Express a finite binary32 number in units of 2**-149."""
    exp = (word >> 23) & 255
    fraction = word & 0x7FFFFF
    integer = ((1 << 23) | fraction) << (exp - 1) if exp else fraction
    return -integer if word & SIGN else integer


def fma(a, b, c):
    ma, mb, mc = map(magnitude, (a, b, c))
    product_sign = (a ^ b) & SIGN
    invalid_product = (ma, mb) in ((INF, 0), (0, INF))
    signaling_nan = any(magnitude(w) > INF and not (w & (1 << 22))
                        for w in (a, b, c))
    if invalid_product or signaling_nan:
        return NAN, 16
    if max(ma, mb, mc) > INF:
        return NAN, 0
    if ma == INF or mb == INF:
        if mc == INF and (c & SIGN) != product_sign:
            return NAN, 16
        return INF | product_sign, 0
    if mc == INF:
        return c, 0

    # Compute a*b+c exactly in units of 2**-298, then round once.
    exact = finite_integer(a) * finite_integer(b) + (finite_integer(c) << 149)
    if not exact:
        return (product_sign & c & SIGN) if not (ma * mb or mc) else 0, 0
    sign = SIGN if exact < 0 else 0
    absolute = abs(exact)
    shift = max(149, absolute.bit_length() - 24)
    mantissa, remainder = divmod(absolute, 1 << shift)
    midpoint = 1 << (shift - 1)
    mantissa += remainder > midpoint or (remainder == midpoint and mantissa & 1)
    if mantissa == 1 << 24:
        mantissa >>= 1
        shift += 1
    if shift > 402:
        return sign | INF, 5
    if mantissa < 1 << 23:
        return sign | mantissa, 3 if remainder else 0
    return sign | ((shift - 148) << 23) | (mantissa & 0x7FFFFF), int(bool(remainder))


def operations(timing=False):
    one = 0x3F800000
    directed = [
        # Seed accumulator, then apply the product.
        (0, 0x00000001, one),                # exact minimum subnormal
        (0, 0x00000001, 0x3F000000),         # halfway to minimum subnormal
        (0, 0x80000001, 0x3F000000),         # rounded negative zero
        (0x80000000, 0x80000000, one),       # negative zero plus negative zero
        (0x80000000, 0, one),                # opposite signed zeros
        (0x007FFFFF, 1, 0x3F000000),         # tiny before rounding only
        (0x007FFFFE, 1, 0x3F000000),         # subnormal tie rounds down
        (one, 0x33800000, one),              # normal tie rounds down
        (one + 1, 0x33800000, one),          # normal tie rounds up
        (0xBF800000, one + 1, one - 2),      # fused cancellation to -2**-46
        (0xFF7FFFFF, 0x7F7FFFFF, 0x40000000),  # finite despite product overflow
        (0x7F7FFFFF, 0x73000000, one),       # overflow on a rounding tie
        (0x7F7FFFFF, 0x72FFFFFF, one),       # just below overflow threshold
        (one | SIGN, one, one),              # exact cancellation to +0
        (0x80800000, 0x00800000, one + 1),   # cancellation into subnormal
        (0x7F7FFFFF, 1, 1),                 # extreme positive exponent gap
        (1, 1, 1),                          # 176-cycle same-sign witness
        (0x80000001, 1, 1),                 # 177-cycle opposite-sign witness
        (0xFF7FFFFF, 1, 1),                 # extreme opposite-sign exponent gap
        (1, 0x7F7FFFFF, 0x7F7FFFFF),        # extreme gap and overflow
    ]
    for acc, a, b in directed:
        # Negative zero needs a rounded negative underflow to seed it from +0.
        yield 1, 0, 0x80000001 if acc == SIGN else acc, 0x3F000000 if acc == SIGN else one
        yield 0, 1, a, b

    specials = [0, SIGN, 1, 0x00800000, one, one | SIGN, INF, INF | SIGN,
                NAN | 1234, 0xFF800001]
    for acc in specials:
        for a in specials:
            for b in specials:
                yield 1, 0, acc, one
                yield 0, 1, a, b

    rng = random.Random(0x754)

    def finite_nonzero():
        while True:
            word = rng.getrandbits(32)
            if 0 < magnitude(word) < INF:
                return word

    for index in range(10_000 if timing else 5000):
        if index % 2:
            a, b = (finite_nonzero() for _ in range(2)) if timing else (
                rng.getrandbits(32) for _ in range(2))
        else:
            # Bias toward similarly scaled products for alignment and cancellation.
            a, b = (((rng.randrange(115, 140) << 23) | rng.getrandbits(23)
                     | (rng.getrandbits(1) << 31)) for _ in range(2))
        yield int(index % 13 == 0), int(index % 13 == 12), a, b
    for _ in range(1024):
        # Cancel against the separately rounded product, exposing its residual.
        a = (rng.randrange(96, 159) << 23) | rng.getrandbits(23) | (rng.getrandbits(1) << 31)
        b = (rng.randrange(96, 159) << 23) | rng.getrandbits(23)
        product, _ = fma(a, b, 0)
        yield 1, 0, product ^ SIGN, one
        yield 0, 1, a, b

    for index in range(1024):
        yield int(index == 0), int(index == 1023), one, one


def write_vectors(path, requests):
    accumulator = 0
    result = 0
    lines = []
    for clear, last, a, b in requests:
        accumulator, flags = fma(a, b, 0 if clear else accumulator)
        if last:
            result = accumulator
        lines.append(f"{clear} {last} {a:08x} {b:08x} {result:08x} {accumulator:08x} {flags:02x}\n")
    path.write_text("".join(lines))


def main():
    timing = os.environ.get("FP32_TIMING") == "1"
    with tempfile.TemporaryDirectory() as tmp:
        executable = Path(tmp) / "sim"
        vectors = Path(tmp) / "vectors.txt"
        write_vectors(vectors, operations(timing))
        subprocess.run(["iverilog", "-g2012", "-Wall", "-s", "tb_fp32_mac",
                        "-o", executable, "fp32_mac.sv", "tests/tb_fp32_mac.sv"],
                       cwd=ROOT, check=True)
        subprocess.run(["vvp", executable, f"+vectors={vectors}", *(["+latency"] if timing else [])],
                       check=True, timeout=180)


if __name__ == "__main__":
    main()
