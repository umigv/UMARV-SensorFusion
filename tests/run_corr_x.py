"""Run the correction RTL against FP32 steps and the covariance identity."""

from math import sqrt
from pathlib import Path
from random import Random
from struct import pack, unpack
from subprocess import run
from tempfile import TemporaryDirectory


def f32(value):
    return unpack(">f", pack(">f", value))[0]


def word(value):
    return pack(">f", value).hex()


def float_word(value):
    return unpack(">f", bytes.fromhex(value))[0]


def upper_triangle(values, order):
    matrix = [[0.0] * order for _ in range(order)]
    words = iter(values)
    for row in range(order):
        for col in range(row, order):
            matrix[row][col] = next(words)
    return matrix


def covariance(factor):
    order = len(factor)
    return [[sum(factor[k][i] * factor[k][j] for k in range(order))
             for j in range(order)] for i in range(order)]


def fp32_reference(state, measurement, packed_s, noise):
    s = upper_triangle(packed_s, 5)
    innovation = [f32(z - x) for z, x in zip(measurement, state)]
    d = f32(f32(s[0][0] ** 2) + noise[0] ** 2)
    q = f32(1.0 / f32(sqrt(d)))
    g = f32(innovation[0] * q)
    u = [f32(f32(s[0][0] * value) * q) for value in s[0]]
    vector = [d, q, g, *u]
    updated, residual, factor = apply_update(state, innovation, s, u, g)
    return {"INNOV": innovation, "VECTOR": vector, "STATE": updated,
            "FACTOR": factor, "RESIDUAL": residual}


def apply_update(state, innovation, s, u, g):
    delta = [f32(value * g) for value in u]
    updated = [f32(x + dx) for x, dx in zip(state, delta)]
    residual = [f32(e - dx) for e, dx in zip(innovation, delta)]
    for k in range(5):
        diagonal = s[k][k]
        t = f32(u[k] / diagonal)
        r = f32(sqrt(f32(f32(diagonal ** 2) - u[k] ** 2)))
        c = f32(r / diagonal)
        s[k][k] = r
        for j in range(k + 1, 5):
            s[k][j] = f32(f32(s[k][j] - t * u[j]) / c)
            u[j] = f32(f32(c * u[j]) - t * s[k][j])
    return updated, residual, [s[i][j] for i in range(5) for j in range(i, 5)]


def fixtures():
    state = [1.0, -0.5, 3.125, 0.75, 0.125]
    measurement = [2.0, -2.25, -3.125, -0.25]
    s = [1.5, 0.3, -0.4, 0.2, 0.15, 1.25, 0.2, -0.15, 0.125,
         1.1, 0.23, -0.14, 0.95, 0.09, 0.8]
    noise = [0.85, 0.1, -0.2, 0.125, 1.2, 0.2, 0.1, 0.9, -0.2, 1.15]
    yield 0, 0, state, measurement, s, noise
    yield 0, 0, state, measurement, s, [noise[0], *[-x for x in noise[1:4]], *noise[4:]]
    yield 1, 0, state, measurement, [*s[:-1], -s[-1]], noise
    yield 1, 0, state, measurement, [1.0, *s[1:]], [2.0**-20, *noise[1:]]
    # Positive |u| < S11, but the rounded diagonal square and u square
    # produce a zero radicand at sqrt issue.
    tiny = f32(sqrt(2.0**-149)) * (1.0 + 2.0**-20)
    yield 1, 0, state, measurement, [tiny, *s[1:]], [tiny, *noise[1:]]
    yield 0, 1, state, measurement, s, noise
    yield 0, 0, state, [state[0], *measurement[1:]], s, noise
    rng = Random(1630)
    for _ in range(12):
        state = [rng.uniform(-3, 3) for _ in range(5)]
        measurement = [rng.uniform(-3, 3) for _ in range(4)]
        s = [rng.uniform(0.75, 2.25) if i == j else rng.uniform(-0.6, 0.6)
             for i in range(5) for j in range(i, 5)]
        noise = [rng.uniform(0.25, 1.25) if i == j else rng.uniform(-0.4, 0.4)
                 for i in range(4) for j in range(i, 4)]
        yield 0, 0, state, measurement, s, noise


def simulate(inputs, second_stage=False, third_stage=False, fourth_stage=False):
    root = Path(__file__).resolve().parents[1]
    with TemporaryDirectory() as temp:
        vec = Path(temp) / "fixtures.hex"
        elf = Path(temp) / "corr_x.vvp"
        vec.write_text("".join(
            f"{i} {failure} {reset} " + " ".join(word(x) for words in data for x in words) + "\n"
            for i, (failure, reset, *data) in enumerate(inputs)))
        run(["iverilog", "-g2012", "-s", "corr_x_tb", "-o", elf,
             f"-Pcorr_x_tb.TEST_SECOND={int(second_stage or third_stage or fourth_stage)}",
             f"-Pcorr_x_tb.TEST_THIRD={int(third_stage or fourth_stage)}",
             f"-Pcorr_x_tb.TEST_FOURTH={int(fourth_stage)}",
             "Dev/sys_defs.sv", "Dev/FP_MAC.sv", "Dev/FP_SQRTDIV.sv",
             "Dev/corr_microcode.sv", "Dev/corr_engine.sv", "tests/corr_x_tb.sv"],
            cwd=root, check=True)
        sim = run(["vvp", elf, f"+vectors={vec}"], cwd=root, text=True, capture_output=True)
        if sim.returncode:
            print(sim.stdout, sim.stderr)
            sim.check_returncode()
    return sim.stdout


def parse_simulation(output, extra_float_tags=()):
    results = {}
    for line in output.splitlines():
        tag, *data = line.split()
        if tag == "CASE":
            case = results[int(data[0])] = {}
        elif tag in ("INNOV", "VECTOR", "STATE", "FACTOR", "RESIDUAL", *extra_float_tags):
            case[tag] = [float_word(item) for item in data]
        elif tag in ("CYCLES", "FAILURE", "RESET", "STAGE_CYCLES", "ARITH_CYCLES"):
            case[tag] = list(map(int, data))
        elif tag == "PASS":
            print(line)
    return results


def main():
    inputs = [(failure, reset, *[list(map(f32, words)) for words in data])
              for failure, reset, *data in fixtures()]
    results = parse_simulation(simulate(inputs))

    max_error = {key: 0.0 for key in ("INNOV", "VECTOR", "STATE", "FACTOR", "RESIDUAL")}
    max_cov_error = 0.0
    for i, (failure, reset, *data) in enumerate(inputs):
        measured = results[i]
        if reset:
            assert "RESET" in measured and "CYCLES" not in measured
            continue
        if failure:
            assert "FAILURE" in measured and "CYCLES" not in measured
            print(f"case {i}: rejected at row, PC {measured['FAILURE']}")
            continue
        expected = fp32_reference(*data)
        for key, reference in expected.items():
            for actual, target in zip(measured[key], reference, strict=True):
                error = abs(actual - target)
                max_error[key] = max(max_error[key], error)
                assert error <= 2.0e-6 * max(1.0, abs(target)), (i, key, actual, target)
        _, _, packed_s, noise = data
        r = covariance(upper_triangle(noise, 4))
        assert any(r[a][b] != 0 for a in range(4) for b in range(a + 1, 4))
        prior = covariance(upper_triangle(packed_s, 5))
        posterior = covariance(upper_triangle(measured["FACTOR"], 5))
        for a in range(5):
            for b in range(5):
                expected_cov = prior[a][b] - prior[a][0] * prior[0][b] / (prior[0][0] + r[0][0])
                error = abs(posterior[a][b] - expected_cov)
                max_cov_error = max(max_cov_error, error)
                assert error < 4.0e-6, (i, a, b, error)

    for key in max_error:
        assert results[0][key] == results[1][key], f"off-diagonal R changed the x update: {key}"
    assert results[2]["FAILURE"] == [4, 14], "late invalid pivot did not abort"
    assert results[3]["FAILURE"] == [0, 14], "rank loss did not abort"
    assert results[4]["FAILURE"] == [0, 17], "zero radicand did not abort at square-root issue"
    assert results[0]["INNOV"][2] == -6.25, "expected raw cross-boundary heading difference"
    cycles = [case["CYCLES"] for case in results.values() if "CYCLES" in case]
    print(f"successful cases: {len(cycles)}; baseline engine/core cycles: {cycles[0]}")
    print(f"observed engine/core cycle ranges: {min(x[0] for x in cycles)}–{max(x[0] for x in cycles)} / "
          f"{min(x[1] for x in cycles)}–{max(x[1] for x in cycles)}")
    print(f"max absolute errors vs FP32 steps: {max_error}")
    print(f"max absolute reconstructed covariance error: {max_cov_error:.9g}")


if __name__ == "__main__":
    main()
