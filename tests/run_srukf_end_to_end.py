"""Run the real SR-UKF and compare both stages against dense covariance equations."""

from argparse import ArgumentParser
from math import cos, isfinite, pi, radians, sin, sqrt
from pathlib import Path
from subprocess import run
from tempfile import TemporaryDirectory
from statistics import mean

from run_corr_x import covariance, f32, upper_triangle, word, wrap_heading
from run_corr_xyz import solve_system
from run_ukf_integration import bits, words

INITIAL_FACTOR = [.30, .02, .01, -.005, .03, .25, -.015, .008, -.01,
                  .12, .025, .012, .18, -.022, .22]
PROCESS_FACTOR = [.012, .002, 0, 0, 0, .011, .001, 0, 0, .008, 0, 0, .009, .001, .007]
NOISE = [.20, .025, -.01, .012, .18, .017, -.01, .09, .019, .14]
DT = f32(.0025)
SOURCES = ("sys_defs FP_MAC FP_SQRTDIV corr_microcode corr_engine int_to_fp32 "
           "deadwheel_velocity lat_to_xy pred_engine pred_microcode pred_sequencer "
           "vn_to_correction UKF").split()


def packed(values):
    return "".join(map(word, reversed(values)))


def predict(x, factor, counts, wheel, central):
    offsets = upper_triangle(factor, 5)
    points = [x] + [[v + sign * sqrt(1.25) * d for v, d in zip(x, row, strict=True)]
                    for sign in (1, -1) for row in offsets]
    propagated = []
    for px, py, theta, omega, bias in points:
        travel = (counts / 8 - bias) * DT if wheel else 0
        mid = theta + DT * omega / 2
        propagated.append([px + travel * cos(mid), py + travel * sin(mid),
                           theta + DT * omega, omega, bias])
    mean = [-3 * col[0] + .4 * sum(col[1:]) for col in zip(*propagated, strict=True)]
    matrix = covariance(upper_triangle(PROCESS_FACTOR, 5))
    for index, point in enumerate(propagated):
        d = [v - m for v, m in zip(point, mean, strict=True)]
        d[2] = wrap_heading(d[2])
        weight = central if index == 0 else .4
        for i in range(5):
            for j in range(5):
                matrix[i][j] += weight * d[i] * d[j]
    mean[2] = wrap_heading(mean[2])
    return mean, matrix


def correct(x, p, measurement, noise):
    r = covariance(upper_triangle(noise, 4))
    gain = [solve_system([[p[i][j] + r[i][j] for j in range(4)] for i in range(4)], row[:4])
            for row in p]
    e = [v - m for v, m in zip(measurement, x)]
    e[2] = wrap_heading(e[2])
    state = [v + sum(k * d for k, d in zip(row, e, strict=True))
             for v, row in zip(x, gain, strict=True)]
    state[2] = wrap_heading(state[2])
    return state, [[p[i][j] - sum(gain[i][k] * p[k][j] for k in range(4))
                    for j in range(5)] for i in range(5)]


def cholesky_upper(matrix):
    upper = [[0.0] * 5 for _ in range(5)]
    for i in range(5):
        upper[i][i] = sqrt(matrix[i][i] - sum(upper[k][i] ** 2 for k in range(i)))
        for j in range(i + 1, 5):
            upper[i][j] = (matrix[i][j] - sum(upper[k][i] * upper[k][j]
                                              for k in range(i))) / upper[i][i]
    return [upper[i][j] for i in range(5) for j in range(i, 5)]


def fixtures(path, initial, invalid):
    # Mode 4 injects a late downdate failure in addition to the invalid noise fixture.
    entries = [(0, 1, 1, 24), (0, 1, 1, 26), (1, 1, 1, 28),
               (0, 1, 1, 30), (0, 0, 1, 0), (2, 1, 1, 24),
               (0, 1, 1, -16), (0, 1, 0, 12), (3, 1, 1, 24),
               (4, 1, 1, 24), (0, 1, 1, 22)]
    if invalid:
        entries = [(1, 1, 1, 24)]
    result = {}
    with path.open("w") as out:
        for index, (mode, wheel, vn, counts) in enumerate(entries):
            east, north = initial[0] + .009 * index, initial[1] - .006 * index
            lat, lon = north / (6371000 * radians(1)), east / (6371000 * radians(1))
            yaw = f32((90 - (initial[2] + initial[3] * DT * index) * 180 / pi) % 360)
            rate = f32(-initial[3])
            noise = list(map(f32, NOISE))
            if mode == 1 and not invalid:
                noise[-1] = 0.0
            print(index, mode, wheel, vn, counts, f"{bits(lat, 64):016x}",
                  f"{bits(lon, 64):016x}", word(yaw), word(rate), packed(noise), file=out)
            measurement = [east, north, pi / 2 - radians(yaw), -rate]
            result[index] = mode, wheel, vn, counts, measurement, noise
    return result


def check(label, got, expected, errors, abs_tol):
    for a, b in zip(got, expected, strict=True):
        absolute = abs(a - b)
        relative = absolute / max(abs(b), abs_tol)
        assert isfinite(a) and absolute <= abs_tol + 2e-4 * abs(b), (label, a, b)
        errors[label][0] = max(errors[label][0], absolute)
        errors[label][1] = max(errors[label][1], relative)


def flatten(matrix):
    return [v for row in matrix for v in row]


def simulate(root, tmp, initial, positive=False, invalid=0, source_unit=None):
    fixture = tmp / "fixtures.txt"
    cases = fixtures(fixture, initial, invalid)
    elf = tmp / "full.vvp"
    params = {
        "START_STATE": f"161'h{(int(''.join(map(word, initial)), 16) << 1) | 1:x}",
        "START_FACTOR": f"480'h{packed(INITIAL_FACTOR)}",
        "PROCESS_FACTOR": f"480'h{packed([0] * 15 if invalid == 1 else PROCESS_FACTOR)}",
        "INVALID_KIND": invalid,
        "POSITIVE_CENTRAL": int(positive),
    }
    testbench = root / "tests/srukf_end_to_end_tb.sv"
    sources = [f"Dev/{name}.sv" for name in SOURCES]
    if source_unit:
        sources = [source_unit.resolve()]
        packaged_testbench = tmp / testbench.name
        packaged_testbench.write_text(testbench.read_text().replace(
            "    logic clk", "    import ukf_defs::*;\n    logic clk", 1))
        testbench = packaged_testbench
    compiled = run(["iverilog", "-g2012", "-s", "srukf_end_to_end_tb", "-o", elf,
         *[f"-Psrukf_end_to_end_tb.{name}={value}" for name, value in params.items()],
         *sources, testbench],
        cwd=root, capture_output=True, text=True)
    assert compiled.returncode == 0, compiled.stderr
    hierarchy = [line for line in elf.read_text().splitlines() if ".scope module," in line]
    for module in ("fp32_mac", "fp32_divsqrt", "UKF_pred_sequencer"):
        assert sum(f'"{module}"' in line for line in hierarchy) == 1, module
    assert not any('"pred_angle_math"' in line for line in hierarchy)
    sim = run(["vvp", elf, f"+fixtures={fixture}"], cwd=root, text=True,
              capture_output=True, timeout=120)
    assert sim.returncode == 0 and "\nPASS\n" in sim.stdout, sim.stdout + sim.stderr
    return cases, sim.stdout


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--source-unit", type=Path, help="optional generated Verilog compilation unit")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    errors = {label: [0.0, 0.0] for label in
              ("predicted state", "predicted covariance", "corrected state", "corrected covariance",
               "heading", "rate", "velocity bias", "VN conversion")}
    all_latency = []
    conversion_latency = []
    with TemporaryDirectory() as directory:
        for initial, positive, invalid in (
            ([.1, -.2, 0., 0., .27], False, 0),
            ([-.3, .2, 3.139, 1.4, -.31], True, 0),
            ([.1, -.2, .7, -.8, 0.], False, 1),
            ([.1, -.2, .7, -.8, 0.], False, 2),
        ):
            cases, output = simulate(root, Path(directory), initial, positive, invalid, args.source_unit)
            prior, factor = list(map(f32, initial)), list(map(f32, INITIAL_FACTOR))
            predicted, corrected, measurements, failed, resets = {}, set(), {}, set(), set()
            for line in output.splitlines():
                tag, *fields = line.split()
                if not fields or tag not in ("MEAS", "PRED", "CORR", "FAIL", "RESET"):
                    continue
                index = int(fields[0])
                mode, wheel, vn, counts, measurement, noise = cases[index]
                if tag == "MEAS":
                    measurements[index] = words(fields[1], 4)
                    if not invalid and mode != 2:
                        conversion_latency.append(int(fields[2]))
                    check("VN conversion", measurements[index], measurement, errors, 2e-6)
                elif tag == "PRED":
                    assert not invalid and index not in predicted
                    if mode == 3:
                        prior, factor = list(map(f32, initial)), list(map(f32, INITIAL_FACTOR))
                    expected, expected_cov = predict(prior, factor, counts, wheel, .25 if positive else -.25)
                    actual = words(fields[1], 5)
                    packed_s = words(fields[2], 15)[::-1]
                    p = covariance(upper_triangle(packed_s, 5))
                    heading = expected[2] + wrap_heading(actual[2] - expected[2])
                    check("predicted state", [*actual[:2], heading, *actual[3:]], expected, errors, 4e-6)
                    check("predicted covariance", flatten(p), flatten(expected_cov), errors, 4e-6)
                    predicted[index] = (expected, expected_cov, int(fields[3]), int(fields[4]))
                    prior, factor = expected, cholesky_upper(expected_cov)
                elif tag == "CORR":
                    assert index not in corrected
                    corrected.add(index)
                    x, p, input_cycles, pred_cycles = predicted[index]
                    expected, expected_cov = correct(x, p, measurement, noise)
                    actual = words(fields[1], 5)
                    packed_s = words(fields[2], 15)[::-1]
                    heading = expected[2] + wrap_heading(actual[2] - expected[2])
                    check("corrected state", [*actual[:2], heading, *actual[3:]], expected, errors, 8e-6)
                    check("corrected covariance", flatten(covariance(upper_triangle(packed_s, 5))),
                          flatten(expected_cov), errors, 6e-6)
                    for name, a, b in zip(("heading", "rate", "velocity bias"),
                                          (heading, actual[3], actual[4]), expected[2:], strict=True):
                        check(name, [a], [b], errors, 8e-6)
                    prior, factor = expected, cholesky_upper(expected_cov)
                    all_latency.append((input_cycles, pred_cycles, int(fields[3]), int(fields[4])))
                elif tag == "FAIL":
                    failed.add(index)
                elif tag == "RESET":
                    resets.add(index)
                    prior, factor = list(map(f32, initial)), list(map(f32, INITIAL_FACTOR))
            if invalid:
                assert failed == {0} and not predicted and not corrected, output
                expected_pc = 265 if invalid == 1 else 341
                assert any(line.split()[-1] == str(expected_pc)
                           for line in output.splitlines() if line.startswith("FAIL ")), output
            else:
                assert set(predicted) == set(cases) - {5} and corrected == {0, 1, 3, 4, 6, 8, 10}, output
                assert failed == {2, 9} and resets == {2, 5, 9}, output
    for label, (absolute, relative) in errors.items():
        print(f"{label}: max absolute {absolute:.6g}, max floored relative {relative:.6g}")
    for name, values in zip(("input", "prediction", "correction", "end-to-end"),
                            zip(*all_latency, strict=True), strict=True):
        print(f"{name}: {min(values)}–{max(values)} cycles, average {mean(values):.2f}")
    print(f"VN conversion: {min(conversion_latency)}–{max(conversion_latency)} cycles (after prediction in the built-in path)")
    worst = max(row[-1] for row in all_latency)
    assert worst <= 125_000, worst
    print(f"worst observed: {worst / 50_000:.3f} ms at 50 MHz; {len(all_latency)} corrected beats")


if __name__ == "__main__":
    main()
