"""Compare the iterative CORDIC with a 65-digit Decimal sin/cos reference."""
from decimal import Decimal, ROUND_HALF_UP, getcontext
from pathlib import Path
import random
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
getcontext().prec = 65
PI = Decimal("3.1415926535897932384626433832795028841971693993751058209749445923")


def reference(raw, scale):
    angle = (Decimal(raw) / scale) % (2 * PI)
    if angle > PI:
        angle -= 2 * PI
    if angle < -PI:
        angle += 2 * PI

    square = -angle * angle
    sin_term = sine = angle
    cos_term = cosine = Decimal(1)
    for k in range(1, 50):
        sin_term *= square / ((2 * k) * (2 * k + 1))
        cos_term *= square / ((2 * k - 1) * (2 * k))
        sine += sin_term
        cosine += cos_term
    return sine, cosine


def run(width, frac, iterations):
    scale = 1 << frac
    low = -(1 << (width - 1))
    high = -low - 1
    mask = (1 << width) - 1
    rng = random.Random(193)
    if width <= 12:
        angles = list(range(low, high + 1))
    else:
        limit = int(Decimal(high) / scale / (2 * PI))
        turns = {0, -1, 1, -limit, limit}
        angles = [low, low + 1, high - 1, high, 0, 1, -1]
        for turn in turns:
            for quadrant in range(-4, 5):
                boundary = int((2 * turn + Decimal(quadrant) / 2) * PI * scale)
                angles.extend(boundary + delta for delta in range(-2, 3)
                              if low <= boundary + delta <= high)
        angles.extend(rng.randint(low, high) for _ in range(3000))
        span = min(high, int(2 * PI * scale))
        angles.extend(rng.randint(-span, span) for _ in range(1000))
        angles = list(dict.fromkeys(angles))

    working_frac = max(34, frac + 8, iterations + 2)
    latency = width + working_frac - frac + iterations + 2
    parameters = dict(WIDTH=width, FRAC=frac, ITERATIONS=iterations,
                      CASES=len(angles), LATENCY=latency)
    with tempfile.TemporaryDirectory(prefix="cordic-") as directory:
        tmp = Path(directory)
        vectors = tmp / "angles.hex"
        vectors.write_text("".join(f"{angle & mask:x}\n" for angle in angles))
        executable = tmp / "sim"
        subprocess.run(
            ["iverilog", "-g2012", "-Wall", "-s", "tb_cordic",
             *(f"-Ptb_cordic.{key}={value}" for key, value in parameters.items()),
             "-o", str(executable), "cordic.sv", "tests/tb_cordic.sv"],
            cwd=ROOT, check=True,
        )
        simulation = subprocess.run(
            ["vvp", str(executable), f"+vectors={vectors}"],
            capture_output=True, text=True, timeout=30,
        )
        if simulation.returncode:
            raise AssertionError(simulation.stdout + simulation.stderr)

    results = [tuple(map(int, line.split()[1:]))
               for line in simulation.stdout.splitlines() if line.startswith("R ")]
    assert len(results) == len(angles)
    maximum = [Decimal(0), Decimal(0)]
    max_counts = 0
    clipped = 0
    for raw, (sine, cosine, overflow) in zip(angles, results):
        exact = reference(raw, scale)
        rounded = [int((value * scale).to_integral_value(rounding=ROUND_HALF_UP))
                   for value in exact]
        expected = [max(low, min(high, value)) for value in rounded]
        if raw == 0:
            assert overflow == (scale > high), (raw, overflow)
        if overflow:
            assert scale > high and high in (sine, cosine), (raw, sine, cosine)
            clipped += 1
        for index, actual in enumerate((sine, cosine)):
            counts = abs(actual - expected[index])
            assert counts <= 1, (width, frac, raw, index, actual, expected[index])
            max_counts = max(max_counts, counts)
            maximum[index] = max(maximum[index], abs(Decimal(actual) / scale - exact[index]))
    assert bool(clipped) == (scale > high)
    print(f"PASS {width}/{frac}, {iterations} iterations: {len(angles)} angles, "
          f"{latency} clocks, max absolute sin/cos errors "
          f"{maximum[0]:.8g}/{maximum[1]:.8g}, max rounded difference {max_counts} LSB")


if __name__ == "__main__":
    for configuration in ((32, 16, 20), (16, 12, 16), (8, 7, 11),
                          (8, 0, 4), (32, 31, 40)):
        run(*configuration)
