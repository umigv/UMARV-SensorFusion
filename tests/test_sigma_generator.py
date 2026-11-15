"""Run software-reference, timing, contention, overflow and reset regressions."""
import random
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def vectors(width, frac):
    rng = random.Random(741)
    lo, hi = -(1 << (width - 1)), (1 << (width - 1)) - 1

    def clip(x):
        return max(lo, min(hi, x))

    cases = [([0]*5, [0]*25, 0), ([hi]*5, [hi]*25, hi),
             ([lo]*5, [lo]*25, hi), ([hi]*5, [1]*25, 1 << frac),
             ([lo]*5, [1]*25, 1 << frac)]
    if frac:
        cases += [([0]*5, [sign]*(25), 1 << (frac-1)) for sign in (-1, 1)]
    for n in range(120):
        bound = min(hi, 4 << frac) if n % 2 else hi
        cases.append(([rng.randint(-bound, bound) for _ in range(5)],
                      [rng.randint(-bound, bound) for _ in range(25)],
                      rng.randint(-bound, bound)))
    words = []
    for state, matrix, gamma in cases:
        expected = [state.copy() for _ in range(11)]
        overflow = False
        for r in range(5):
            for c in range(r, 5):
                product = gamma * matrix[r*5+c]
                magnitude = abs(product)
                rounded = ((magnitude + (1 << (frac-1))) >> frac) if frac else magnitude
                scaled = -rounded if product < 0 else rounded
                offset = clip(scaled)
                overflow |= offset != scaled
                for row, value in ((r+1, state[c]+offset), (r+6, state[c]-offset)):
                    expected[row][c] = clip(value)
                    overflow |= clip(value) != value
        words.extend(state + matrix + [gamma] + sum(expected, []) + [int(overflow)])
    mask = (1 << width)-1
    return '\n'.join(f'{v & mask:x}' for v in words), len(cases)


def run(args):
    subprocess.run(args, cwd=ROOT, check=True)


with tempfile.TemporaryDirectory(prefix='sigma-regression-') as directory:
    tmp = Path(directory)
    for width, frac in ((8, 0), (16, 4), (32, 16)):
        data, count = vectors(width, frac)
        path = tmp / 'vectors.hex'
        path.write_text(data + '\n')
        binary = str(tmp / 'reference')
        run(['iverilog', '-g2012', '-s', 'tb_sigma_reference',
             f'-Ptb_sigma_reference.WIDTH={width}', f'-Ptb_sigma_reference.FRAC={frac}',
             f'-Ptb_sigma_reference.CASES={count}', '-o', binary,
             'tests/tb_sigma_reference.sv', 'sigma_generator.sv', 'MAC_array.sv', 'MAC_accumulator.sv'])
        run(['vvp', binary, f'+vectors={path}'])
    for timeout in (2, 3, 16):
        binary = str(tmp / 'watchdog')
        run(['iverilog', '-g2012', '-s', 'tb_sigma_watchdog',
             f'-Ptb_sigma_watchdog.TEST_TIMEOUT={timeout}', '-o', binary,
             'tests/tb_sigma_watchdog.sv', 'sigma_generator.sv', 'MAC_array.sv', 'MAC_accumulator.sv'])
        run(['vvp', binary])
