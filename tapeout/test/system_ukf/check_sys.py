"""Replays the measurements logged by tb_system_ukf.sv through the exact model of the filter and
compares each state the hardware produced, bit for bit."""
import sys
sys.path.insert(0, sys.argv[2] if len(sys.argv) > 2 else "..")
import ukf_prog as U
consts, info = U.constants()
p = U.build(consts); mc = U.Machine(p.resolve(), consts, p.cnames)
want, got, n_meas = [], [], 0
for line in open(sys.argv[1]):
    f = line.split()
    if f[0] == "M":
        vn, dw = int(f[1]), int(f[2])
        yaw, rate, lat, lon, delta, ts = (int(v, 16) for v in f[3:9])
        ext = [yaw, rate, (lat >> 20) & 0x3FFFFFF, (lon >> 20) & 0x3FFFFFF, delta & 0xFFFFFFF, ts, U.double_to_single_bits(lat), 0]
        n_meas += 1
        if mc.beat(ext, bool(vn), bool(dw)):
            want.append(tuple(mc.mem[i] for i in range(6)))
    else:
        got.append(tuple(int(v, 16) for v in f[1:7]))
n = min(len(want), len(got))
bad = sum(1 for i in range(n) if want[i] != got[i])
for i in range(n):
    if want[i] != got[i]:
        print("state %d differs: hardware %s model %s" % (i, " ".join("%08x" % v for v in got[i]), " ".join("%08x" % v for v in want[i]))); break
print("%d beats logged, hardware produced %d states, model %d; %d compared, %d differ" % (n_meas, len(got), len(want), n, bad))
last = got[-1]
print("last state: x %.4f m, y %.4f m, theta %.4f rad, omega %.4f rad/s, v %.4f m/s" % tuple(float(U.b2f(v)) for v in last[1:6]))
print("REPLAY PASSED" if bad == 0 and len(got) >= len(want) - 1 and n > 5 else "REPLAY FAILED")
