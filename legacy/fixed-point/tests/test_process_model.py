"""Run the process model with the real MAC/CORDIC and a Decimal reference."""
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path
import random
import subprocess
import tempfile

from test_cordic import PI, reference as sincos

ROOT = Path(__file__).resolve().parents[1]
SCALE = 1 << 16
DT_SCALE = 1 << 31
LOW, HIGH = -(1 << 31), (1 << 31) - 1


def rounded(value):
    return int(value.to_integral_value(rounding=ROUND_HALF_UP))


def q(value):
    return rounded(Decimal(str(value)) * SCALE)


def cases():
    millisecond = Decimal("0.001")
    ordinary = [(q(i-4), q(4-i), q(Decimal(i-4) / 2), q(i-4)) for i in range(9)]
    yield millisecond, q(10), [(x, y, 0, 0) for x, y, _, _ in ordinary]
    yield millisecond, q(2.5), ordinary  # clockwise/counterclockwise, negative headings
    yield millisecond, 0, ordinary
    yield millisecond, q(-3), [(x, y, theta, 0) for x, y, theta, _ in ordinary]
    yield Decimal(0), q(40), ordinary
    headings = [PI, PI - Decimal("0.00015"), 2*PI, 5*PI, -PI, -2*PI, -9*PI,
                Decimal("20000"), Decimal("-30000")]
    yield millisecond, q(20), [(0, 0, q(theta), q((-1)**i))
                               for i, theta in enumerate(headings)]
    yield millisecond, q(100), [
        (HIGH, HIGH, q(1), HIGH), (LOW, LOW, q(-2), LOW), (0, 0, HIGH, HIGH),
        (0, 0, LOW, LOW), (HIGH-5, LOW+5, q(-1), 0), *ordinary[:4]
    ]
    # Signed products at ties, including endpoints for the timestep format.
    yield Decimal("0.5"), 1, [(0, 0, 0, (-1)**i) for i in range(9)]
    yield Decimal(HIGH) / DT_SCALE, LOW, [(0, 0, 0, LOW)] * 9

    rng = random.Random(821)
    for n in range(80):
        points = [(rng.randint(-q(200), q(200)), rng.randint(-q(200), q(200)),
                   rng.randint(-q(50), q(50)), rng.randint(-q(30), q(30))) for _ in range(9)]
        dt = millisecond if n < 50 else Decimal(rng.randint(0, 4000)) / 1_000_000
        yield dt, rng.randint(-q(30), q(30)), points
    for _ in range(16):
        yield millisecond, rng.randint(LOW, HIGH), [
            tuple(rng.randint(LOW, HIGH) for _ in range(4)) for _ in range(9)]


def main():
    vectors = list(cases())
    words = []
    for dt, velocity, points in vectors:
        words.extend([velocity, rounded(dt * DT_SCALE)])
        words.extend(value for point in points for value in point)
    with tempfile.TemporaryDirectory(prefix="process-model-") as directory:
        tmp = Path(directory)
        vectors_file = tmp / "vectors.hex"
        vectors_file.write_text("".join(f"{value & 0xffffffff:x}\n" for value in words))
        exe = tmp / "sim"
        subprocess.run(["iverilog", "-g2012", "-Wall", "-s", "tb_process_model",
                        f"-Ptb_process_model.CASES={len(vectors)}", "-o", str(exe),
                        "process_model.sv", "MAC_accumulator.sv", "cordic.sv",
                        "tests/tb_process_model.sv"], cwd=ROOT, check=True)
        simulation = subprocess.run(["vvp", str(exe), f"+vectors={vectors_file}"],
                                    capture_output=True, text=True, timeout=30)
    if simulation.returncode:
        raise AssertionError(simulation.stdout + simulation.stderr)
    results = [list(map(int, line.split()[1:])) for line in simulation.stdout.splitlines()
               if line.startswith("R ")]
    assert len(results) == len(vectors)
    maximum = [Decimal(0)] * 4
    ordinary_maximum = maximum.copy()
    rounded_error = [0] * 4
    for n, ((dt, velocity, points), (overflow, *actual)) in enumerate(zip(vectors, results)):
        # Compare with the requested interval, before Q1.31 quantization.
        should_overflow = False
        for i, (x, y, theta, omega) in enumerate(points):
            sine, cosine = sincos(theta, SCALE)
            expected = [Decimal(x) + velocity*cosine*dt, Decimal(y) + velocity*sine*dt,
                        Decimal(theta) + omega*dt, Decimal(omega)]
            for j, value in enumerate(expected):
                wanted = rounded(value)
                should_overflow |= not LOW <= wanted <= HIGH
                clipped = max(LOW, min(HIGH, value))
                actual_value = actual[i*4+j]
                error = abs(actual_value - clipped)
                counts = abs(actual_value - max(LOW, min(HIGH, wanted)))
                maximum[j] = max(maximum[j], error / SCALE)
                if n < len(vectors) - 16 and n != 8:  # exclude full-scale stress
                    ordinary_maximum[j] = max(ordinary_maximum[j], error / SCALE)
                rounded_error[j] = max(rounded_error[j], counts)
                assert counts <= (24, 24, 1, 0)[j], (n, i, j, actual_value, wanted)
        assert overflow == should_overflow, (n, overflow, should_overflow)
    assert "PASS process timing" in simulation.stdout
    print(f"PASS {len(vectors)} transactions, {9*len(vectors)} points; 760 clocks, 28 MAC starts, 9 CORDIC starts")
    print("Maximum absolute errors x/y/theta/omega:", *(f"{value:.8g}" for value in maximum))
    print("Non-full-scale max absolute errors:", *(f"{value:.8g}" for value in ordinary_maximum))
    print("Maximum rounded-reference differences (LSB):", *rounded_error)
    print("PASS input capture, atomic output, arbitration, watchdog deadlines, errors and active reset")


if __name__ == "__main__":
    main()
