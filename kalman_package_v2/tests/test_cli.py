import math
import subprocess

def run(data):
    p = subprocess.run(['./kalman_filter'], input=data, text=True, capture_output=True, timeout=15)
    assert p.returncode == 0, p.stderr
    return p

p = run('G 0 0 0 junk\nG 0 0 0 0\nG 0 0 0\nO 0 1 0 0\nO 0.02 1 0 0 trailing\nG 0.02 nan 0\nO 0.02 1 0 0\nO 0.01 1 0 0\n')
assert p.stderr.count('malformed') == 4, p.stderr
assert 'stale_packet' in p.stdout and 'updated' in p.stdout
assert len(run('G 0 0 0\nO 0 1 0 0').stdout.splitlines()) == 2
p = run('X'*600+'\nG 0 0 0\nO 0 1 0 0\n')
assert 'line too long' in p.stderr and 'initialized' in p.stdout
# A failed future update must not prevent a later valid packet.
p = run('G 0 0 0\nO 0 1 0 0\nO 0.02 1e308 0 0\nO 0.01 1 0 0\n')
assert 'update_failed' in p.stdout and p.stdout.splitlines()[-1].endswith('updated'), p.stdout
# 20-second constant-turn trajectory, repeatedly crossing heading wrap.
lines=['G 0 0 0', 'O 0 1 0 0.4']
for i in range(1,1001):
    t=i*0.02; heading=math.atan2(math.sin(0.4*t),math.cos(0.4*t))
    lines.append(f'O {t} 1 {heading} 0.4')
    if i%10==0: lines.append(f'G {t} {math.sin(0.4*t)/0.4} {(1-math.cos(0.4*t))/0.4}')
p=run('\n'.join(lines)+'\n')
assert 'failed' not in p.stdout and 'rejected' not in p.stdout, p.stdout[-1000:]
last=p.stdout.splitlines()[-1].split()
assert abs(float(last[2])-math.sin(8)/0.4)<0.1
assert abs(float(last[3])-(1-math.cos(8))/0.4)<0.1
print('CLI parser, rollback, and 1,100-update trajectory tests passed')
