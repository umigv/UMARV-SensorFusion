"""Verify all three stages against sequential FP32 and a joint 3x3 update."""

from math import sqrt

from run_corr_x import (
    apply_update, covariance, f32, fp32_reference, parse_simulation, simulate, upper_triangle, wrap_heading,
)
from run_corr_xy import combined_fixtures, compare, fp32_second


def mac_dot(pairs, initial=None):
    terms = iter(pairs)
    if initial is None:
        left, right = next(terms)
        initial = f32(left * right)
    for left, right in terms:
        initial = f32(initial + left * right)
    return initial


def fp32_third(second, noise):
    s = upper_triangle(second["FACTOR"], 5)
    residual = second["RESIDUAL"]
    c = f32(1.0 / noise[7])
    b = f32(-f32(noise[5] * c) / noise[4])
    a = f32(-mac_dot(((noise[1], b), (noise[2], c))) / noise[0])
    h = [a, b, c]
    eps = mac_dot(zip(h, residual[:3], strict=True))
    t = [mac_dot(zip(s[i][i:3], h[i:], strict=True)) for i in range(3)]
    d = mac_dot(zip(t, t, strict=True), initial=1.0)
    q = f32(1.0 / f32(sqrt(d)))
    g = f32(eps * q)
    u = [f32(q * mac_dot((s[i][j], t[i]) for i in range(min(j + 1, 3))))
         for j in range(5)]
    vector = [d, q, g, *u]
    state, residual, factor = apply_update(second["STATE"], residual, s, u, g)
    return {"HEADING_WEIGHTS": h, "HEADING_PROJECTION": [eps, *t], "XYZ_VECTOR": vector,
            "STATE": state, "RESIDUAL": residual, "FACTOR": factor}


def solve_system(matrix, rhs):
    """Double-precision elimination with partial pivoting."""
    order = len(matrix)
    augmented = [list(row) + [rhs[i]] for i, row in enumerate(matrix)]
    for col in range(order):
        pivot = max(range(col, order), key=lambda i: abs(augmented[i][col]))
        augmented[col], augmented[pivot] = augmented[pivot], augmented[col]
        top = augmented[col]
        for i in range(col + 1, order):
            row = augmented[i]
            scale = row[col] / top[col]
            for j in range(col, order + 1):
                row[j] -= scale * top[j]
    answer = [0.0] * order
    for i in reversed(range(order)):
        row = augmented[i]
        answer[i] = (row[order] - sum(row[j] * answer[j] for j in range(i + 1, order))) / row[i]
    return answer


def joint_reference(state, measurement, packed_s, noise):
    p = covariance(upper_triangle(packed_s, 5))
    r = covariance(upper_triangle(noise, 4))
    d = [[p[i][j] + r[i][j] for j in range(3)] for i in range(3)]
    gains = [solve_system(d, row[:3]) for row in p]
    innovation = [measurement[i] - state[i] for i in range(3)]
    innovation[2] = wrap_heading(innovation[2])
    posterior = [state[i] + sum(gains[i][k] * innovation[k] for k in range(3))
                 for i in range(5)]
    posterior[2] = wrap_heading(posterior[2])
    new_covariance = [[p[i][j] - sum(gains[i][k] * p[k][j] for k in range(3))
                       for j in range(5)] for i in range(5)]
    return posterior, new_covariance


def heading_fixtures():
    base = list(combined_fixtures())
    yield from base
    _, _, state, measurement, s, noise = base[0]
    # The first two rows succeed. With the third diagonal close to zero,
    # the third downdate loses rank in FP32 at the heading pivot.
    uncoupled_heading = s.copy()
    uncoupled_heading[2] = uncoupled_heading[6] = 0.0
    uncoupled_heading[9] = 1.0
    near_singular = noise.copy()
    near_singular[2] = near_singular[5] = 0.0
    near_singular[7] = 2.0**-20
    yield 3, 0, state, measurement, uncoupled_heading, near_singular
    yield 3, 0, state, measurement, s, [*noise[:7], -noise[7], *noise[8:]]
    yield 0, 3, state, measurement, s, noise
    yield 0, 0, state, measurement, s, noise


def main():
    inputs = [(failure, reset, *[list(map(f32, words)) for words in data])
              for failure, reset, *data in heading_fixtures()]
    tags = ("X_STATE", "X_FACTOR", "X_RESIDUAL", "XY_STATE", "XY_FACTOR", "XY_RESIDUAL",
            "WEIGHTS", "PROJECTION", "XY_VECTOR", "HEADING_WEIGHTS", "HEADING_PROJECTION", "XYZ_VECTOR")
    results = parse_simulation(simulate(inputs, third_stage=True), extra_float_tags=tags)
    max_fp32 = max_joint_state = max_joint_cov = 0.0
    signs = [set() for _ in range(3)]
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
            assert "XY_STATE" not in observed and "FAILURE" in observed
            continue
        second = fp32_second(first, data[-1])
        for key in ("STATE", "FACTOR", "RESIDUAL"):
            max_fp32 = max(max_fp32, *compare(observed["XY_" + key], second[key], (i, "XY_" + key)))
        if failure == 3:
            expected_failure = [2, 14] if data[-1][7] > 0 else [0, 44]
            assert observed["FAILURE"] == expected_failure, (i, observed["FAILURE"])
            assert "CYCLES" not in observed
            print(f"case {i}: third stage rejected at row, PC {observed['FAILURE']}")
            continue

        third = fp32_third(second, data[-1])
        for key, targets in third.items():
            max_fp32 = max(max_fp32, *compare(observed[key], targets, (i, key)))
        joint_state, joint_cov = joint_reference(*data)
        for actual, target in zip(observed["STATE"], joint_state, strict=True):
            error = abs(actual - target)
            max_joint_state = max(max_joint_state, error)
            assert error <= 5e-6 * max(1.0, abs(target)), (i, actual, target)
        observed_cov = covariance(upper_triangle(observed["FACTOR"], 5))
        for actual_row, expected_row in zip(observed_cov, joint_cov, strict=True):
            for actual, target in zip(actual_row, expected_row, strict=True):
                error = abs(actual - target)
                max_joint_cov = max(max_joint_cov, error)
                assert error < 7e-6, (i, actual, target)
        for signs_seen, index in zip(signs, (1, 2, 5), strict=True):
            assert data[-1][index] != 0.0
            signs_seen.add(data[-1][index] > 0)

    assert all(len(values) == 2 for values in signs)
    assert all(a != b for a, b in zip(results[0]["STATE"], results[0]["XY_STATE"], strict=True))
    cycles = [case["STAGE_CYCLES"] for case in results.values() if "CYCLES" in case]
    assert cycles[0][:2] == [4987, 4916], cycles[0]
    assert all(sum(case["STAGE_CYCLES"]) == case["CYCLES"][1]
               for case in results.values() if "CYCLES" in case)
    print(f"successful cases: {len(cycles)}")
    print(f"baseline core x / second / third: {cycles[0]}; total {sum(cycles[0])}")
    print(f"baseline engine cycles: {results[0]['CYCLES'][0]}")
    print(f"baseline MAC/divsqrt arithmetic cycles (third, total): {results[0]['ARITH_CYCLES']}")
    print(f"third / total core ranges: {min(c[2] for c in cycles)}–{max(c[2] for c in cycles)} / "
          f"{min(map(sum, cycles))}–{max(map(sum, cycles))}")
    print(f"max absolute error vs sequential FP32: {max_fp32:.9g}")
    print(f"max absolute state error vs joint update: {max_joint_state:.9g}")
    print(f"max absolute covariance error vs joint update: {max_joint_cov:.9g}")


if __name__ == "__main__":
    main()
