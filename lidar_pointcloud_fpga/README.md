# Genesys 2 Ethernet front end (FPGA-only, no HPS)

VLP-16 UDP packets go straight into the FPGA through the on-board RGMII PHY.
The FPGA parses them, runs the existing `stage_t` polar-to-cartesian transform,
and sends the XYZ points back out of the same port as UDP for a laptop.
The ground filter and occupancy grid (`stage_f`, `stage_rmw`, `double_buffer`)
are not part of this build.

```
RJ45 -> RTL8211E -> taxi 1G RGMII MAC -> vlp16_udp_rx -> stage_t -> pc_fix2float x3
                                                                         |
RJ45 <- RTL8211E <- taxi 1G RGMII MAC <- pc_udp_tx <- point FIFO <-------+
```

## Wiring

The Genesys 2 has one Ethernet port, so put the lidar, the board and the laptop
on a small gigabit switch. A factory-default VLP-16 broadcasts its data, so the
board sees it; the board broadcasts its output, so the laptop sees it. The
laptop needs an address on the lidar's subnet (e.g. 192.168.1.100/24).

## Output format

One UDP datagram (port 2370, from 192.168.1.50) per up to 120 points, little endian:

| offset | size | field |
|---|---|---|
| 0 | 4 | magic `PC2X` |
| 4 | 1 | version = 1 |
| 5 | 1 | point_step = 12 |
| 6 | 2 | num_points |
| 8 | 4 | seq (datagram counter) |
| 12 | 2 | lidar azimuth of the first point, 0.01 deg |
| 14 | 2 | flags = 0 |
| 16 | 12 x N | `float32 x, y, z` in metres |

The point area is exactly the `data` array of a `sensor_msgs/PointCloud2` with
fields x/y/z = FLOAT32 at offsets 0/4/8 and `point_step` 12. Axes follow
REP-103 like `velodyne_pointcloud` (x forward, y left, z up); set
`ROS_FRAME = 0` for the raw `stage_t` axes. Returns with distance 0 are dropped.

`genesys2/host/pc2x_bridge.py` turns the datagrams into one PointCloud2 per revolution
on `/fpga_points` (ROS 2). `--no-ros` prints statistics without ROS.

## Build

```
git submodule update --init          # lidar_pointcloud_fpga/lib/taxi (Ethernet MAC)
cd lidar_pointcloud_fpga/genesys2/syn && make              # Vivado, xc7k325tffg900-2
make program
```

Addresses, ports and the lidar port filter are generics of `genesys2/rtl/fpga.sv`;
override them in `genesys2/syn/config.tcl`.

LEDs: 0 lidar packets parsed (blinks), 1 point cloud datagrams sent (blinks),
2 other frames seen, 3 link 1000 Mb/s, 4 link 100 Mb/s, 5 bad FCS received,
6 MAC RX overflow, 7 point FIFO overflow (5-7 sticky until reset).

## Simulation

`genesys2/tb/run_vcs.sh` drives `fpga_core` at the RGMII pins and checks every
transmitted frame (FCS, IP/UDP headers, IP checksum, every coordinate).

## Licence note

`lib/taxi` (submodule) is CERN-OHL-S-2.0 (strongly reciprocal). A bitstream that includes
it has to be distributed under compatible terms.

## Origin

`verilog/` (the `stage_t` transform, its trig ROM and `lut_generator.py`) is
copied from the DE10-Nano lidar accelerator in RISC-M/FPGA-Based-LIDAR-ISP.

The VLP-16 payload parser is William Kletzka's `v16PacketParser`
(`lidar_fpga/v16PacketParser.v` at the repo root, taken unmodified from the
`William-Kletzka` branch). `genesys2/rtl/vlp16_udp_rx.sv` wraps it and adds the
Ethernet/IPv4/UDP header check, back-pressure and the laser id, which that
parser does not provide.
