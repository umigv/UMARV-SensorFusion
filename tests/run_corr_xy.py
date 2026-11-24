"""Verify the two-stage correction against FP32 steps and a joint update."""

from math import isfinite, sqrt

from run_corr_x import (
    apply_update, covariance, f32, fixtures, fp32_reference, parse_simulation,
    simulate, upper_triangle,
)


def fp32_second(first, noise):
    state = first["STATE"]
    residual = first["RESIDUAL"]
    s = upper_triangle(first["FACTOR"], 5)
    r11, r12, r22 = noise[0], noise[1], noise[4]
    a = f32(-r12 / f32(r11 * r22))
    b = f32(1.0 / r22)
    eps = f32(f32(a * residual[0]) + b * residual[1])
    t1 = f32(f32(s[0][0] * a) + s[0][1] * b)
    t2 = f32(s[1][1] * b)
    d = f32(f32(1.0 + t1**2) + t2**2)
    q = f32(1.0 / f32(sqrt(d)))
    g = f32(eps * q)
    projection = [f32(f32(x * t1) + y * t2) for x, y in zip(s[0], s[1])]
    u = [f32(p * q) for p in projection]
    vector = [d, q, g, *u]
    updated, residual, factor = apply_update(state, residual, s, u, g)
    return {
        "WEIGHTS": [a, b], "PROJECTION": [eps, t1, t2],
        "XY_VECTOR": vector, "STATE": updated, "FACTOR": factor,
        "RESIDUAL": residual,
    }


def joint_reference(state, measurement, packed_s, noise):
    """Apply the two observations together in double precision."""
    p = covariance(upper_triangle(packed_s, 5))
    r = covariance(upper_triangle(noise, 4))
    d00, d01, d11 = p[0][0] + r[0][0], p[0][1] + r[0][1], p[1][1] + r[1][1]
    determinant = d00 * d11 - d01**2
    gain = [((pi[0] * d11 - pi[1] * d01) / determinant,
             (pi[1] * d00 - pi[0] * d01) / determinant) for pi in p]
    ex, ey = measurement[0] - state[0], measurement[1] - state[1]
    posterior = [x + k0 * ex + k1 * ey for x, (k0, k1) in zip(state, gain)]
    new_covariance = [[p[i][j] - k0 * p[0][j] - k1 * p[1][j] for j in range(5)]
                      for i, (k0, k1) in enumerate(gain)]
    return posterior, new_covariance


def combined_fixtures():
    base = list(fixtures())
    for failure, reset, *data in base:
        yield failure, reset, *data
    _, _, state, measurement, s, noise = base[0]
    # X succeeds. The whitened y update rounds its downdate to rank loss.
    near_singular_s = [s[0], 0.0, 0.0, 0.0, 0.0, 1.0, *s[6:]]
    near_singular_r = [noise[0], 0.0, *noise[2:4], 2.0**-20, *noise[5:]]
    yield 2, 0, state, measurement, near_singular_s, near_singular_r
    yield 2, 0, state, measurement, s, [*noise[:4], -noise[4], *noise[5:]]
    yield 0, 2, state, measurement, s, noise
    yield 0, 0, state, measurement, s, noise


def compare(observed, expected, record):
    assert len(observed) == len(expected)
    for actual, target in zip(observed, expected, strict=True):
        error = abs(actual - target)
        assert isfinite(actual) and error <= 3e-6 * max(1.0, abs(target)), (record, actual, target)
        yield error


def main():
    inputs = [(failure, reset, *[list(map(f32, words)) for words in data])
              for failure, reset, *data in combined_fixtures()]
    tags = ("X_STATE", "X_FACTOR", "X_RESIDUAL", "WEIGHTS", "PROJECTION", "XY_VECTOR")
    results = parse_simulation(simulate(inputs, second_stage=True), extra_float_tags=tags)
    max_fp32 = max_joint_state = max_joint_cov = 0.0
    signs = set()
    for i, (failure, reset, *data) in enumerate(inputs):
        observed = results[i]
        if reset:
            assert "RESET" in observed and "CYCLES" not in observed
            continue
        if failure == 1:
            assert "X_STATE" not in observed and "FAILURE" in observed
            continue
        first = fp32_reference(*data)
        for key in ("STATE", "FACTOR", "RESIDUAL"):
            max_fp32 = max(max_fp32, *compare(observed["X_" + key], first[key], (i, "X_" + key)))
        if failure == 2:
            expected_failure = [1, 14] if data[-1][4] > 0 else [0, 26]
            assert observed["FAILURE"] == expected_failure
            print(f"case {i}: second stage rejected at row, PC {observed['FAILURE']}")
            continue

        second = fp32_second(first, data[-1])
        for key, targets in second.items():
            max_fp32 = max(max_fp32, *compare(observed[key], targets, (i, key)))
        joint_state, joint_cov = joint_reference(*data)
        for actual, target in zip(observed["STATE"], joint_state, strict=True):
            error = abs(actual - target)
            max_joint_state = max(max_joint_state, error)
            assert error <= 4e-6 * max(1.0, abs(target)), (i, actual, target)
        observed_cov = covariance(upper_triangle(observed["FACTOR"], 5))
        for actual_row, expected_row in zip(observed_cov, joint_cov, strict=True):
            for actual, target in zip(actual_row, expected_row, strict=True):
                error = abs(actual - target)
                max_joint_cov = max(max_joint_cov, error)
                assert error < 6e-6, (i, actual, target)
        signs.add(data[-1][1] > 0)

    assert len(signs) == 2
    assert all(a != b for a, b in zip(results[0]["STATE"], results[0]["X_STATE"], strict=True))
    assert results[0]["STAGE_CYCLES"][0] == 4913
    cycles = [case["STAGE_CYCLES"] for case in results.values() if "STAGE_CYCLES" in case]
    assert all(sum(case["STAGE_CYCLES"]) == case["CYCLES"][1]
               for case in results.values() if "CYCLES" in case)
    print(f"successful cases: {len(cycles)}")
    print(f"baseline core x / second / total: {cycles[0][0]} / {cycles[0][1]} / {sum(cycles[0])}")
    print(f"baseline engine cycles: {results[0]['CYCLES'][0]}")
    print(f"second / total core ranges: {min(c[1] for c in cycles)}–{max(c[1] for c in cycles)} / "
          f"{min(map(sum, cycles))}–{max(map(sum, cycles))}")
    print(f"max absolute error vs sequential FP32: {max_fp32:.9g}")
    print(f"max absolute state error vs joint update: {max_joint_state:.9g}")
    print(f"max absolute covariance error vs joint update: {max_joint_cov:.9g}")


if __name__ == "__main__":
    main()
