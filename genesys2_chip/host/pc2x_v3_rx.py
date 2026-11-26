#!/usr/bin/env python3
"""PC2X version 3 UDP receiver and live diagnostics for the Genesys 2 chip port.

Matches tapeout/docs/lidar_output_format.md (version 3, 1224-byte UDP datagrams
on port 2370). Prints sweep rate, point count, sequence gaps, flags, and pose.
"""
import argparse
import socket
import struct
import time

HDR = struct.Struct("<4sBBHIHHBBBBIIfffff")  # 48 bytes
BLK_HDR = struct.Struct("<HBB")              # 4 bytes
BLOCK_SIZE = 196


def main():
    ap = argparse.ArgumentParser(description="Receive PC2X v3 UDP point cloud packets")
    ap.add_argument("--port", type=int, default=2370)
    ap.add_argument("--bind", default="")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind((args.bind, args.port))
    print(f"Listening for PC2X v3 datagrams on udp/{args.port} ...")

    t0 = time.time()
    dgrams = 0
    sweeps = 0
    valid_pts = 0
    lost = 0
    bad = 0
    last_seq = None
    last_sweep = None
    last_pose = None
    last_flags = 0

    while True:
        data, src = sock.recvfrom(2048)
        if len(data) != HDR.size + 6 * BLOCK_SIZE:
            bad += 1
            continue
        (magic, version, flags, npts, seq, sweep_id, first_az, model, step,
         n_blocks, per_block, sweep_us, state_us, x, y, theta, omega, v) = HDR.unpack_from(data)
        if magic != b"PC2X" or version != 3 or step != 6 or n_blocks != 6 or per_block != 32:
            bad += 1
            continue

        if last_seq is not None:
            lost += (seq - last_seq - 1) & 0xFFFFFFFF
        last_seq = seq
        if last_sweep is not None and sweep_id != last_sweep:
            sweeps += 1
        last_sweep = sweep_id
        last_flags = flags
        last_pose = (x, y, theta, omega, v, sweep_us, state_us)
        dgrams += 1

        for b in range(n_blocks):
            off = HDR.size + b * BLOCK_SIZE
            az, bflags, _ = BLK_HDR.unpack_from(data, off)
            if bflags & 0x02:
                continue
            if bflags & 0x01:
                sweeps += 1
            for p in range(per_block):
                px, py, pz = struct.unpack_from("<hhh", data, off + 4 + 6 * p)
                if not (px == -32768 and py == -32768 and pz == -32768):
                    valid_pts += 1

        now = time.time()
        if now - t0 >= 1.0:
            scale = 0.004 if (last_flags & 0x02) else 0.002
            print(f"src={src[0]} dgrams/s={dgrams} sweeps/s={sweeps} pts/s={valid_pts} "
                  f"lost={lost} bad={bad} flags=0x{last_flags:02X} scale={scale*1000:.0f}mm "
                  f"state_ok={last_flags & 1} pose=({last_pose[0]:.2f},{last_pose[1]:.2f},"
                  f"{last_pose[2]:.2f} rad, w={last_pose[3]:.2f}, v={last_pose[4]:.2f})")
            t0 = now
            dgrams = sweeps = valid_pts = 0


if __name__ == "__main__":
    main()
