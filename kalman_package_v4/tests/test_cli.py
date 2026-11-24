#!/usr/bin/env python3
"""Parser/integration tests. Python standard library only; run via make test."""
import math
from pathlib import Path
import random
import subprocess

EXE = Path(__file__).resolve().parents[1] / 'kalman_filter'

def run(lines):
    proc = subprocess.run([str(EXE)], input=lines, text=True, capture_output=True,
                          timeout=30, check=True)
    return proc.stdout.splitlines(), proc.stderr

def estimate(line):
    fields = line.split()
    assert len(fields) == 7 and fields[0] == 'F', line
    values = list(map(float, fields[1:6]))
    assert all(map(math.isfinite, values)), line
    assert -math.pi-1e-9 <= values[3] <= math.pi+1e-9
    return values, fields[6]

lines, _ = run('# startup\n\n E 0 2\n vn 0 1 2 0 0\nE .1 2\nVN .1 1.2 2 0 0\nE .2 0\n')
assert lines[0] == 'WAIT need_initial_vn'
a, status = estimate(lines[1])
assert a == [0, 1, 2, 0, 0] and status == 'initialized'
b, status = estimate(lines[2])
assert 1.199 < b[1] < 1.201 and status == 'encoder_predicted'
c, status = estimate(lines[3])
assert status == 'vn_updated'
d, _ = estimate(lines[4])
assert abs(d[1]-c[1]) < 1e-8  # no held previous velocity

# Only the first VN initializes. Subsequent VN alone use an identity prediction.
lines, _ = run('VN 0 0 0 .3 .7\nVN 1 0 0 .3 .7\nE 1.1 0\nE 1.1 100\n')
a, _ = estimate(lines[1]); b, _ = estimate(lines[2]); c, _ = estimate(lines[3])
assert abs(a[3]-.3) < 1e-8 and abs(b[3]-.37) < 1e-8
assert b == c  # same timestamp E is a no-op

malformed = ['G 0 0 0', 'O 0 0 0 0', 'VN 1 0 0 0', 'VN 1 0 0 0 0 1',
             'E 1', 'E 1 0 2', 'VN1 0 0 0 0', 'ENC 1 0', 'E 1 2junk',
             'E 1 nan', 'VN 1 0 inf 0 0', 'E 1 1e309', 'E 1 1e-999',
             'VN 1 0 0 0 0 # no inline comments', 'x'*600, 'E 1 1e308']
lines, err = run('VN 0 1 2 .3 .4\n'+'\n'.join(malformed)+'\nE .1 0\nE .05 0\n')
assert len(lines) == len(malformed)+3
for line in lines[1:1+len(malformed)]:
    values, status = estimate(line)
    assert values == [0,1,2,.3,.4] and status == 'invalid_packet', line
values, status = estimate(lines[-1])
assert values[0] == .1 and status == 'stale_packet'
assert 'ERROR' in err
# A failed numerical update must not advance the watermark.
lines, _ = run('VN 0 0 0 0 0\nE 1e308 1\nE .1 1')
assert estimate(lines[1])[1] == 'update_failed'
assert estimate(lines[2])[0][0] == .1
# A rejected VN changes the timestamp but not state or covariance.
lines, _ = run('VN 0 0 0 0 0\nVN 1 10000 0 0 0\nE 1.1 0\n')
values, status = estimate(lines[1])
assert values == [1,0,0,0,0] and status == 'measurement_rejected'


def ou(rng, value, sigma, tau, dt):
    a = math.exp(-dt/tau)
    return a*value + sigma*math.sqrt(-math.expm1(-2*dt/tau))*rng.gauss(0,1)

# End-to-end exercising the reference simulator error laws. These tests check
# numerical behavior, not that a second filter always beats an INS solution.
def trajectory(seed, vn_hz):
    rng = random.Random(seed)
    hz, duration = 400, 60
    dt = 1/hz
    x, y, heading = 0., 0., 0.
    scale = bx = by = bh = bw = 0.
    packets, truth, vn_error = [], [], []
    vn_every = hz//vn_hz
    for i in range(hz*duration+1):
        t = i*dt
        v = 2.5 + 1.2*math.sin(.2*t)
        w = .22*math.sin(.3*t)
        if i:
            x += v*dt*math.cos(heading + .5*w*dt)
            y += v*dt*math.sin(heading + .5*w*dt)
            heading += w*dt
        # One common sensor tick; sample the VN stream at the selected rate.
        scale = ou(rng, scale, .02, 60, dt)
        bx = ou(rng, bx, .5, 120, dt)
        by = ou(rng, by, .5, 120, dt)
        bh = ou(rng, bh, math.radians(.3), 300, dt)
        bw = ou(rng, bw, .002, 120, dt)
        ev = v*(1+scale) + rng.gauss(0,.01)
        nx, ny = x+bx+rng.gauss(0,.05), y+by+rng.gauss(0,.05)
        nh = (heading+bh+rng.gauss(0,math.radians(.5))+math.pi)%(2*math.pi)-math.pi
        nw = w+bw+rng.gauss(0,.005)
        if i:
            packets.append(f'E {t:.9f} {ev:.12g}')
            truth.append((x,y))
        if i % vn_every == 0:
            packets.append(f'VN {t:.9f} {nx:.12g} {ny:.12g} {nh:.12g} {nw:.12g}')
            truth.append((x,y))
            vn_error.append((nx-x)**2+(ny-y)**2)
    output, err = run('\n'.join(packets)+'\n')
    assert len(output) == len(packets)
    assert 'ERROR' not in err and 'Filter error' not in err, err
    squared, squared_at_vn, rejects = [], [], 0
    for packet, line, (tx,ty) in zip(packets,output,truth):
        values, status = estimate(line)
        assert status in {'initialized','vn_updated','encoder_predicted','measurement_rejected'}
        rejects += status == 'measurement_rejected'
        squared.append((values[1]-tx)**2 + (values[2]-ty)**2)
        if packet.startswith('VN '):
            squared_at_vn.append(squared[-1])
    print(f'Trajectory seed={seed}, VN={vn_hz}Hz, E=400Hz: '
          f'{len(output)} packets, {rejects} gated VN, '
          f'filtered position RMSE={math.sqrt(sum(squared)/len(squared)):.4f}m '
          f'(all packets); filtered RMSE={math.sqrt(sum(squared_at_vn)/len(squared_at_vn)):.4f}m, '
          f'VN RMSE={math.sqrt(sum(vn_error)/len(vn_error)):.4f}m '
          f'(VN samples).')

trajectory(42,100)
trajectory(7,100)
trajectory(42,5)
print('CLI tests passed: parser, initialization, timing, rollback and mixed-rate streams.')
