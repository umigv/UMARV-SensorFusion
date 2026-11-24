# Lidar output: data format and timing

For whoever writes the program on the computer that receives the chip's lidar output and
publishes it to ROS.

**Status: version 3 of the format.** This is what the chip's lidar block produces today in
simulation. It has not run on hardware yet, and the Kalman filter is not connected yet (see
`state_valid` under Flags). The version byte in every frame tells you which layout you have.
Items still open are listed at the end.

Version 3 replaces version 2, which described points in the world frame. That was a
misunderstanding of what was wanted.

## What the chip sends

The lidar (a VLP-16 today) plugs into the board instead of the computer. The chip converts each lidar return to
x, y, z and corrects it for the robot's motion during the sweep, then sends the result to the
computer as ordinary UDP packets over 100 Mbit Ethernet. The Kalman filter's state rides along
in every packet.

```
VLP-16 --Ethernet--> board (chip) --Ethernet--> computer --> your program --> ROS topics
```

You do not need a raw socket or a special driver. Open a normal UDP socket on the destination
port and read datagrams.

| Setting | Default (proposed) | Notes |
|---|---|---|
| Destination IP | 255.255.255.255 (broadcast) | settable on the chip |
| Destination UDP port | 2370 | settable |
| Source IP, port | 192.168.1.50, 2370 | settable |
| UDP checksum | 0 (not used) | allowed for IPv4; do not reject on it |

## What the correction does

The lidar takes about 100 ms to spin once. If the robot turns or drives during that time, the
points of one sweep are taken from different places facing different ways.

The chip puts **each sweep into one coordinate frame: the robot's own frame at the moment that
sweep started.** Every point is turned back and moved back by however much the robot has turned
and travelled since then, using the turn rate and speed from the Kalman filter.

- The points are **not** rotated or moved into the world or map frame. A sweep looks the way
  it would if the robot had stood still at the sweep's start.
- A sweep starts when the lidar's own azimuth passes its zero mark.
- The start of the sweep is the reference, not the end, because the chip sends points as they
  arrive and cannot know how the robot will move after a point is sent.

To place a sweep on the map, use the filter state in the header (below). That part is yours.

## Datagram layout

All fields are little-endian. One datagram is always 1,224 bytes: a 48-byte header, then 6
blocks of 196 bytes. A block is one data block of the lidar's own packet: 32 returns that share
one azimuth reading.

### Header (48 bytes)

| Offset | Size | Type | Field | Meaning |
|---|---|---|---|---|
| 0 | 4 | bytes | `magic` | ASCII `PC2X` |
| 4 | 1 | uint8 | `version` | 3 |
| 5 | 1 | uint8 | `flags` | see below |
| 6 | 2 | uint16 | `num_points` | 192 |
| 8 | 4 | uint32 | `seq` | counts datagrams, wraps; a gap means a datagram was lost |
| 12 | 2 | uint16 | `sweep_id` | the sweep the first block belongs to. Counts sweeps, wraps |
| 14 | 2 | uint16 | `first_azimuth` | lidar's own azimuth of the first block, 0.01 degree, 0 to 35999 |
| 16 | 1 | uint8 | `model` | lidar the chip is set for: 0 VLP-16 or Puck LITE, 1 Puck Hi-Res, 2 VLP-32C, 3 HDL-32E |
| 17 | 1 | uint8 | `point_step` | bytes per point: 6 |
| 18 | 1 | uint8 | `num_blocks` | 6 |
| 19 | 1 | uint8 | `points_per_block` | 32 |
| 20 | 4 | uint32 | `sweep_time_us` | chip time when sweep `sweep_id` started. **This is the time that sweep's frame belongs to** |
| 24 | 4 | uint32 | `state_time_us` | chip time of the filter state below |
| 28 | 4 | float32 | `x` | filter state: position, metres |
| 32 | 4 | float32 | `y` | filter state: position, metres |
| 36 | 4 | float32 | `theta` | filter state: heading, radians |
| 40 | 4 | float32 | `omega` | filter state: turn rate, radians per second |
| 44 | 4 | float32 | `v` | forward speed, metres per second |

The five state values are the filter's own numbers, copied without change. They are the latest
state when the datagram was started, so `state_time_us` is usually a few milliseconds after
`sweep_time_us` and moves on through the sweep. The filter updates 400 times a second.

`sweep_time_us` is taken when the sweep's first block reaches the chip, which is up to 1.4 ms
after the lidar fired it.

### Flags

| Bit | Name | Meaning when 1 |
|---|---|---|
| 0 | `state_valid` | the filter has produced a state. **When 0, the points are not corrected and the five state fields and `state_time_us` mean nothing: ignore them** |
| 1 | `scale_4mm` | one count is 4 mm. When 0, one count is 2 mm |
| 2 | `rx_bad` | a lidar packet arrived corrupted since the previous datagram. Its points were still sent; some points in this datagram or the one before may be wrong |
| 3 | `overflow` | the chip had to discard lidar data since the previous datagram |
| 4-7 | | reserved, ignore |

Until the filter is connected, `state_valid` is always 0 and the points are the lidar's own
x, y, z with no motion correction.

### Blocks (6 x 196 bytes, starting at offset 48)

| Offset in block | Size | Type | Field | Meaning |
|---|---|---|---|---|
| 0 | 2 | uint16 | `azimuth` | the lidar's own azimuth for this block, 0.01 degree |
| 2 | 1 | uint8 | `block_flags` | bit 0: this block is the first of a new sweep. bit 1: padding, skip the whole block |
| 3 | 1 | | reserved | 0 |
| 4 | 192 | | 32 points | 6 bytes each |

Each point:

| Offset in point | Size | Type | Field |
|---|---|---|---|
| 0 | 2 | int16 | `x` forward |
| 2 | 2 | int16 | `y` left |
| 4 | 2 | int16 | `z` up |

- Multiply by the scale from the flags (0.002 or 0.004) to get metres. The values are in the
  robot's frame at the start of the point's sweep. Nothing needs adding to them.
- **A point with x = y = z = -32768 is "no return". Skip it.** Empty returns are kept in the
  stream so that position in the block always tells you which laser a point came from.
- A value of exactly 32767 or -32767 means the point was out of range of the 16-bit number.
- With a 16-laser lidar (`model` 0 or 1), point `i` of a block came from laser `i % 16` (the
  lidar's laser ID, the "ring"); points 0 to 15 are one firing and 16 to 31 the next. With a
  32-laser lidar, point `i` came from laser `i`.
- **Sweeps and datagrams do not line up.** A new sweep can start at any block. Walk the blocks in
  order: a block with bit 0 of `block_flags` set starts sweep `sweep_id + 1`, and so do the
  blocks after it in that datagram. The new sweep's `sweep_time_us` is in the next datagram's
  header.
- A padding block appears only if the chip ran out of lidar data in the middle of a datagram
  (for example the lidar was unplugged). It never appears in normal running.

## Using it

**One ROS point cloud per sweep.** Collect blocks until one has the new-sweep bit set. Then
publish what you have as one `sensor_msgs/PointCloud2` in the lidar's frame, stamped with that
sweep's `sweep_time_us` converted to ROS time (see Timestamps), and start the next cloud with
that block.

**The pose that goes with a sweep.** The cloud is in the robot's frame at `sweep_time_us`. The
state samples in the headers are taken at `state_time_us`, a little later. To get the pose at
exactly `sweep_time_us`, step back from the first sample of the sweep:

```
dt      = (state_time_us - sweep_time_us) * 1e-6        # unsigned 32-bit subtraction, then to seconds
theta_s = theta - omega * dt
x_s     = x - v * cos(theta_s) * dt
y_s     = y - v * sin(theta_s) * dt
```

or interpolate between the samples either side of it. Publish that as odometry or a transform
with the same stamp as the cloud.

**Odometry at full rate.** The state in the headers arrives about 1,500 times a second and
changes 400 times a second, so the same stream can drive an odometry topic.

## Timing

For a VLP-16 in single-return mode at 600 RPM:

| Quantity | Value |
|---|---|
| Lidar packets in | about 754 per second, one every 1.33 ms |
| Datagrams out | 2 per lidar packet, about 1,507 per second |
| Points per datagram | 192 (6 blocks x 32) |
| Datagram size | 48 + 6 x 196 = 1,224 bytes of UDP payload |
| Point rate | about 289,000 per second, including "no return" entries |
| Data rate | about 15 Mbit/s |
| One sweep | 100 ms, about 151 datagrams, about 29,000 points |
| Filter state | updated 400 times per second |
| Delay added by the chip | about 0.2 ms from a lidar packet arriving to its second datagram leaving (simulation) |

### Timestamps

`sweep_time_us` and `state_time_us` are the chip's own clock. It starts at 0 when the chip
powers up and wraps every 71.6 minutes. It is not the computer's time.

- For time differences, use them directly with unsigned 32-bit subtraction, which stays correct
  across the wrap.
- For a ROS timestamp, keep a running estimate of `computer_receive_time - state_time_us` (the
  smallest value seen recently is the best estimate, since network delay only ever adds) and add
  it to the chip time you want to convert.

### Lost data

UDP does not retransmit. Use `seq` to count gaps. A lost datagram is 192 missing points out of
about 29,000 in a sweep, so publish the sweep anyway. If the lost datagram held the block that
starts a sweep, you will see `sweep_id` in the next header jump instead; treat that as the
boundary.

## Bypass mode

If the chip's lidar block is switched to bypass, it forwards every frame the lidar sends,
unchanged (the lidar's data is on UDP port 2368), and the standard Velodyne ROS driver works as
if the lidar were plugged straight into the computer. This is the fallback if the correction has
a problem. Nothing travels from the computer back to the lidar in either mode.

## Minimal receiver (Python)

```python
import socket, struct
import numpy as np

HDR = struct.Struct("<4sBBHIHHBBBBIIfffff")       # 48 bytes
BLOCK = 196

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", 2370))

def publish(points, sweep_id):
    if points:
        cloud = np.concatenate(points)             # one whole sweep, robot frame at its start
        # ... publish a PointCloud2 here, stamped with this sweep's sweep_time_us

sweep, current = [], None
while True:
    data, _ = sock.recvfrom(2048)
    (magic, version, flags, n, seq, sweep_id, first_az, model, step, n_blocks, per_block,
     sweep_time_us, state_time_us, x, y, theta, omega, v) = HDR.unpack_from(data)
    if magic != b"PC2X" or version != 3:
        continue
    state_ok = bool(flags & 0x01)                  # if False, ignore x, y, theta, omega, v
    scale = 0.004 if flags & 0x02 else 0.002
    if current is not None and sweep_id != current:   # the boundary block was in a lost datagram
        publish(sweep, current); sweep = []
    current = sweep_id
    for b in range(n_blocks):
        off = HDR.size + b * BLOCK
        azimuth, block_flags = struct.unpack_from("<HB", data, off)
        if block_flags & 0x02:                     # padding
            continue
        if block_flags & 0x01:                     # this block starts the next sweep
            publish(sweep, current); sweep = []
            current = (current + 1) & 0xFFFF
        xyz = np.frombuffer(data, dtype="<i2", offset=off + 4, count=per_block * 3).reshape(per_block, 3)
        keep = ~np.all(xyz == -32768, axis=1)      # drop "no return"
        sweep.append(xyz[keep].astype(np.float32) * scale)
```

`genesys2/host/pc2x_bridge.py` in the UMARV-SensorFusion repository is a complete ROS bridge for
version 1 of this format (the FPGA prototype) and is a good starting point.

## Still open

| Item | Why it matters to you |
|---|---|
| Which way the robot-frame axes point | proposed: x forward, y left, z up, as in ROS REP-103 |
| Which way the world axes point and which way heading is positive | decides how you use `x`, `y`, `theta`. Proposed: x east, y north, heading counter-clockwise from x |
| Which other filter values you need | for example the estimated wheel radius, or the filter's uncertainty in each value |
| Whether intensity is sent | it is not sent now. If added, `point_step` becomes 7 or 8 and each point gains an intensity byte |
| The default addresses and port | may change to suit the robot's network |
| Whether zero position on z should be the lidar or the ground | currently the lidar |
| Dual-return mode | not supported; the lidar must be set to single return |
| When the filter is connected | until then `state_valid` is 0 and points are uncorrected |

Tell the chip team if any of these would make your side harder. They are cheap to change now and
impossible after the chip is made.
