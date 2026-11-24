"""Exercise the real wheel, coordinate and correction RTL with a prediction driver."""

from math import cos, pi, radians
from pathlib import Path
from struct import pack, unpack
from subprocess import PIPE, STDOUT, run
from tempfile import TemporaryDirectory

from run_corr_full import fp32_fourth, full_fixtures, joint_reference
from run_corr_x import covariance, f32, fp32_reference, upper_triangle
from run_corr_xy import fp32_second
from run_corr_xyz import fp32_third


def bits(value, width=32):
    return int.from_bytes(pack(">f" if width == 32 else ">d", value), "big")


def words(value, count):
    return [unpack(">f", bytes.fromhex(value[offset:offset + 8]))[0]
            for offset in range(0, count * 8, 8)]


def fixture_file(path):
    fixtures = list(full_fixtures())
    _, _, base_state, _, factor, noise = fixtures[0]
    _, _, _, _, singular_factor, singular_noise = next(
        fixture for fixture in fixtures if fixture[0] == 4 and fixture[-1][9] > 0
    )
    references = []
    with path.open("w") as stream:
        for i in range(9):
            latitude, longitude = 42.001 + i * 0.0001, -83.002 - i * 0.0001
            yaw, rate = 30.0 + i, 0.25 + i * 0.125
            state = [value + i * 0.125 for value in base_state]
            scaled_factor = [f32(value * (1.0 + i / 16.0)) for value in factor]
            selected_noise = noise
            if i == 3:
                scaled_factor = list(map(f32, singular_factor))
                selected_noise = singular_noise
            packed_state = (int("".join(f"{bits(value):08x}" for value in state), 16) << 1) | 1
            packed_factor = "".join(f"{bits(value):08x}" for value in reversed(scaled_factor))
            packed_noise = "".join(f"{bits(value):08x}" for value in reversed(selected_noise))
            print(f"{bits(latitude, 64):016x} {bits(longitude, 64):016x} "
                  f"{bits(yaw):08x} {bits(rate):08x} {packed_state:041x} "
                  f"{packed_factor} {packed_noise}", file=stream)
            scale = 6_371_000 * radians(1)
            measurement = [(f32(longitude) + 83) * scale * cos(radians(42)),
                           (f32(latitude) - 42) * scale, pi / 2 - radians(yaw), -rate]
            references.append((state, scaled_factor, list(map(f32, selected_noise)), measurement))
    return references


def main():
    root = Path(__file__).resolve().parents[1]
    sources = ("sys_defs FP_MAC FP_SQRTDIV corr_microcode corr_engine int_to_fp32 "
               "deadwheel_velocity lat_to_xy pred_engine pred_microcode pred_sequencer vn_to_correction UKF")
    with TemporaryDirectory() as directory:
        fixture = Path(directory) / "fixtures.txt"
        reference = fixture_file(fixture)
        executable = Path(directory) / "integration.vvp"
        run(["iverilog", "-g2012", "-s", "ukf_integration_tb", "-o", executable,
             *[f"Dev/{name}.sv" for name in sources.split()],
             "tests/ukf_integration_tb.sv"], cwd=root, check=True)
        simulation = run(["vvp", executable, f"+fixtures={fixture}"], cwd=root,
                         text=True, stdout=PIPE, stderr=STDOUT, timeout=30)
        output = simulation.stdout
        assert simulation.returncode == 0, output
    measurements = {}
    completed = set()
    for line in output.splitlines():
        tag, *fields = line.split()
        if tag == "MEAS":
            index, packed_measurement = fields
            index = int(index)
            measured = words(packed_measurement, 4)
            expected = reference[index][3]
            for actual, target, tolerance in zip(measured, expected, [0.0002] * 2 + [2e-7, 0]):
                assert abs(actual - target) <= tolerance, (index, actual, target)
            measurements[index] = measured
        elif tag == "RESULT":
            index, packed_state, packed_factor = fields
            index = int(index)
            state, factor, noise, _ = reference[index]
            _, expected_cov = joint_reference(state, measurements[index], factor, noise)
            # Large position updates can move the heading residual across the
            # branch cut between stages. Model each wrap before the next row.
            first = fp32_reference(state, measurements[index], factor, noise)
            second = fp32_second(first, noise)
            third = fp32_third(second, noise)
            expected_state = fp32_fourth(third, noise)["STATE"]
            assert int(packed_state, 16) & 1
            actual_state = words(f"{int(packed_state, 16) >> 1:040x}", 5)
            actual_cov = covariance(upper_triangle(words(packed_factor, 15)[::-1], 5))
            state_error = max(abs(a - b) for a, b in zip(actual_state, expected_state))
            cov_error = max(abs(a - b) for actual, expected in zip(actual_cov, expected_cov)
                            for a, b in zip(actual, expected))
            assert state_error < 8e-5 and cov_error < 2e-6, (index, state_error, cov_error)
            print(f"transaction {index}: max state error {state_error:.3g}, "
                  f"max covariance error {cov_error:.3g}")
            completed.add(index)
    assert completed == {0, 1, 2, 8} and "PASS" in output
    print("UKF integration PASS")


if __name__ == "__main__":
    main()
