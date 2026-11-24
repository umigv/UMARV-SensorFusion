"""Verify the four-row correction against FP32 steps and a joint 4x4 update."""

from math import sqrt

from run_corr_x import (
    apply_update, covariance, f32, fp32_reference, parse_simulation, simulate, upper_triangle, wrap_heading,
)
from run_corr_xy import compare, fp32_second
from run_corr_xyz import fp32_third, heading_fixtures, mac_dot, solve_system


def fp32_fourth(third, noise):
    sr = upper_triangle(noise, 4)
    s = upper_triangle(third["FACTOR"], 5)
    residual = third["RESIDUAL"]
    h = [0.0, 0.0, 0.0, f32(1.0 / sr[3][3])]
    for i in (2, 1, 0):
        numerator = mac_dot(zip(sr[i][i + 1:], h[i + 1:], strict=True))
        h[i] = f32(-numerator / sr[i][i])
    eps = mac_dot(zip(h, residual, strict=True))
    t = [mac_dot(zip(s[i][i:4], h[i:], strict=True)) for i in range(4)]
    variance = mac_dot(zip(t, t, strict=True), initial=1.0)
    q = f32(1.0 / f32(sqrt(variance)))
    g = f32(eps * q)
    u = [f32(q * mac_dot((s[i][j], t[i]) for i in range(min(j + 1, 4))))
         for j in range(5)]
    vector = [variance, q, g, *u]
    state, residual, factor = apply_update(third["STATE"], residual, s, u, g)
    return {"RATE_WEIGHTS": h, "RATE_PROJECTION": [eps, *t], "FULL_VECTOR": vector,
            "STATE": state, "RESIDUAL": residual, "FACTOR": factor}


def joint_reference(state, measurement, packed_s, noise):
    p = covariance(upper_triangle(packed_s, 5))
    r = covariance(upper_triangle(noise, 4))
    innovation_cov = [[p[i][j] + r[i][j] for j in range(4)] for i in range(4)]
    gains = [solve_system(innovation_cov, row[:4]) for row in p]
    innovation = [z - x for z, x in zip(measurement, state)]
    innovation[2] = wrap_heading(innovation[2])
    posterior = [x + sum(k * e for k, e in zip(gain, innovation, strict=True))
                 for x, gain in zip(state, gains, strict=True)]
    posterior[2] = wrap_heading(posterior[2])
    new_covariance = [[p[i][j] - sum(gains[i][k] * p[k][j] for k in range(4))
                       for j in range(5)] for i in range(5)]
    return posterior, new_covariance


def full_fixtures():
    base = list(heading_fixtures())
    yield from base
    _, _, state, measurement, s, noise = base[0]
    # Make the rate pivot independent of the preceding rows, then force rank
    # loss through a positive but tiny fourth measurement-noise pivot.
    uncoupled_rate = s.copy()
    uncoupled_rate[3] = uncoupled_rate[7] = uncoupled_rate[10] = 0.0
    uncoupled_rate[12] = 1.0
    near_singular = noise.copy()
    near_singular[3] = near_singular[6] = near_singular[8] = 0.0
    near_singular[9] = 2.0**-20
    yield 4, 0, state, measurement, uncoupled_rate, near_singular
    yield 4, 0, state, measurement, s, [*noise[:9], -noise[9]]
    yield 0, 4, state, measurement, s, noise
    yield 0, 0, state, measurement, s, noise


def check_values(observed, expected, case):
    return max(error for key, values in expected.items()
               for error in compare(observed[key], values, (case, key)))


def main():
    inputs = [(failure, reset, *[list(map(f32, words)) for words in data])
              for failure, reset, *data in full_fixtures()]
    state_tags = ("STATE", "FACTOR", "RESIDUAL")
    tags = tuple(f"{stage}_{key}" for stage in ("X", "XY", "XYZ") for key in state_tags)
    tags += ("RATE_WEIGHTS", "RATE_PROJECTION", "FULL_VECTOR")
    results = parse_simulation(simulate(inputs, fourth_stage=True), extra_float_tags=tags)
    max_fp32 = max_joint_state = max_joint_cov = 0.0
    correlation_signs = [set() for _ in range(6)]
    for i, (failure, reset, *data) in enumerate(inputs):
        observed = results[i]
        if reset:
            assert "RESET" in observed and "CYCLES" not in observed
            continue

        reference_steps = [lambda _: fp32_reference(*data), lambda first: fp32_second(first, data[-1]),
                           lambda second: fp32_third(second, data[-1])]
        posterior = None
        for stage, (prefix, reference) in enumerate(zip(("X", "XY", "XYZ"), reference_steps), 1):
            if failure == stage:
                assert prefix + "_STATE" not in observed and "FAILURE" in observed
                break
            posterior = reference(posterior)
            snapshot = {f"{prefix}_{key}": posterior[key] for key in state_tags}
            max_fp32 = max(max_fp32, check_values(observed, snapshot, i))
        if failure:
            assert "CYCLES" not in observed
            if failure == 4:
                expected_failure = [3, 14] if data[-1][9] > 0 else [0, 62]
                assert observed["FAILURE"] == expected_failure, (i, observed["FAILURE"])
                print(f"case {i}: fourth stage rejected at row, PC {observed['FAILURE']}")
            continue

        max_fp32 = max(max_fp32, check_values(observed, fp32_fourth(posterior, data[-1]), i))
        expected_state, expected_cov = joint_reference(*data)
        for actual, target in zip(observed["STATE"], expected_state, strict=True):
            error = abs(actual - target)
            max_joint_state = max(max_joint_state, error)
            assert error <= 6e-6 * max(1.0, abs(target)), (i, actual, target)
        observed_cov = covariance(upper_triangle(observed["FACTOR"], 5))
        for actual_row, expected_row in zip(observed_cov, expected_cov, strict=True):
            for actual, target in zip(actual_row, expected_row, strict=True):
                error = abs(actual - target)
                max_joint_cov = max(max_joint_cov, error)
                assert error < 8e-6, (i, actual, target)
        for signs, r_index in zip(correlation_signs, (1, 2, 3, 5, 6, 8), strict=True):
            assert data[-1][r_index] != 0.0
            signs.add(data[-1][r_index] > 0.0)

    assert all(len(signs) == 2 for signs in correlation_signs)
    assert all(a != b for a, b in zip(results[0]["STATE"], results[0]["XYZ_STATE"], strict=True))
    cycles = [case["STAGE_CYCLES"] for case in results.values() if "CYCLES" in case]
    assert cycles[0][:3] == [4987, 4916, 5213], cycles[0]
    assert all(sum(case["STAGE_CYCLES"]) == case["CYCLES"][1]
               for case in results.values() if "CYCLES" in case)
    print(f"successful cases: {len(cycles)}")
    print(f"baseline core stages: {cycles[0]}; total {sum(cycles[0])}")
    print(f"baseline engine cycles: {results[0]['CYCLES'][0]}")
    print(f"fourth / total core ranges: {min(c[3] for c in cycles)}–{max(c[3] for c in cycles)} / "
          f"{min(map(sum, cycles))}–{max(map(sum, cycles))}")
    print(f"max absolute error vs sequential FP32: {max_fp32:.9g}")
    print(f"max absolute state error vs joint update: {max_joint_state:.9g}")
    print(f"max absolute covariance error vs joint update: {max_joint_cov:.9g}")


if __name__ == "__main__":
    main()
