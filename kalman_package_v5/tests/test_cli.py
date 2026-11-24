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

# Synchronized bootstrap works in either packet order.
for prefix in ('E 0 2\nVN 0 1 2 0 0\n','VN 0 1 2 0 0\nE 0 2\n'):
    lines, _ = run(prefix+'E .1 2\nVN .15 1.3 2 0 0\nE .2 2\n')
    assert lines[0].startswith('WAIT need_initial_')
    a, status = estimate(lines[1])
    assert a == [0,1,2,0,0] and status == 'initialized'
    b, status = estimate(lines[2])
    assert 1.199 < b[1] < 1.201 and status == 'encoder_updated'
    c, status = estimate(lines[3])
    assert 1.299 < c[1] < 1.301 and status == 'vn_updated'
    d, status = estimate(lines[4])
    assert 1.399 < d[1] < 1.401 and status == 'encoder_updated'

# Equal-time E corrects velocity but never moves the pose twice.
lines, _ = run('E 0 2\nVN 0 0 0 0 0\nE .1 2\nE .1 2.01\nVN .1 .2 0 0 0\n')
a, _ = estimate(lines[2]); b, status = estimate(lines[3])
assert a == b and status == 'encoder_updated'
# VN can propagate between encoder samples, even through an innovation rejection.
lines, _ = run('E 0 2\nVN 0 0 0 0 0\nVN .1 10000 0 0 0\nE .2 2\n')
a, status = estimate(lines[2]); b, _ = estimate(lines[3])
assert status == 'measurement_rejected' and .199 < a[1] < .201
assert .399 < b[1] < .401
# Reject a bad speed observation without using it for motion prediction.
lines, _ = run('E 0 2\nVN 0 0 0 0 0\nE .1 1000\nE .2 2\n')
a, status = estimate(lines[2]); b, _ = estimate(lines[3])
assert status == 'measurement_rejected' and .199 < a[1] < .201
assert .399 < b[1] < .401
# Bootstrap waits for a recent pair, without double-fusing either startup sample.
lines, _ = run('VN 0 0 0 0 0\nE .2 2\nVN .2 .4 0 0 0\n')
assert lines[:2] == ['WAIT need_initial_encoder','WAIT initial_timestamps_too_far_apart']
assert estimate(lines[2])[1] == 'initialized'
malformed = ['G 0 0 0','O 0 0 0 0','VN 1 0 0 0','VN 1 0 0 0 0 1',
             'E 1','E 1 0 2','VN1 0 0 0 0','ENC 1 0','E 1 2junk',
             'E 1 nan','VN 1 0 inf 0 0','E 1 1e309','E 1 1e-999',
             'VN 1 0 0 0 0 # inline comment','x'*600,'E 1 1e308']
lines, err = run('E 0 2\nVN 0 1 2 .3 .4\n'+'\n'.join(malformed)+'\nE .1 2\nE .05 2\n')
assert len(lines) == len(malformed)+4
for line in lines[2:2+len(malformed)]:
    values, status = estimate(line)
    assert values == [0,1,2,.3,.4] and status == 'invalid_packet', line
values, status = estimate(lines[-1])
assert values[0] == .1 and status == 'stale_packet'
assert 'ERROR' in err
lines, _ = run('E 0 2\nVN 0 0 0 0 0\nE 1e308 1\nE .1 2')
assert estimate(lines[2])[1] == 'update_failed'
assert estimate(lines[3])[0][0] == .1


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
        packets.append(f'E {t:.9f} {ev:.12g}')
        truth.append((x,y))
        if i % vn_every == 0:
            packets.append(f'VN {t:.9f} {nx:.12g} {ny:.12g} {nh:.12g} {nw:.12g}')
            truth.append((x,y))
            vn_error.append((nx-x)**2+(ny-y)**2)
    output, err = run('\n'.join(packets)+'\n')
    assert len(output) == len(packets)
    assert output[0] == 'WAIT need_initial_vn'
    assert 'ERROR' not in err and 'Filter error' not in err, err
    squared, squared_at_vn, rejects_vn, rejects_e = [], [], 0, 0
    for packet, line, (tx,ty) in zip(packets[1:],output[1:],truth[1:]):
        values, status = estimate(line)
        assert status in {'initialized','vn_updated','encoder_updated','measurement_rejected'}
        if status == 'measurement_rejected':
            if packet.startswith('VN '): rejects_vn += 1
            else: rejects_e += 1
        squared.append((values[1]-tx)**2 + (values[2]-ty)**2)
        if packet.startswith('VN '):
            squared_at_vn.append(squared[-1])
    print(f'Trajectory seed={seed}, VN={vn_hz}Hz, E=400Hz: '
          f'{len(output)} packets, {rejects_vn} gated VN / {rejects_e} gated E, '
          f'filtered position RMSE={math.sqrt(sum(squared)/len(squared)):.4f}m '
          f'(all packets); filtered RMSE={math.sqrt(sum(squared_at_vn)/len(squared_at_vn)):.4f}m, '
          f'VN RMSE={math.sqrt(sum(vn_error)/len(vn_error)):.4f}m '
          f'(VN samples).')

trajectory(42,400)
trajectory(42,100)
trajectory(7,100)
trajectory(42,5)
print('CLI tests passed: parser, initialization, timing, rollback and mixed-rate streams.')
