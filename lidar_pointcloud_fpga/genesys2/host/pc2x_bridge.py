#!/usr/bin/env python3
"""Laptop side of the Genesys 2 lidar front end.

Receives the "PC2X" UDP datagrams produced by the FPGA and publishes one
sensor_msgs/PointCloud2 per lidar revolution (ROS 2).

    ros2 run / python3 pc2x_bridge.py                 # publish on /fpga_points
    python3 pc2x_bridge.py --no-ros                   # no ROS needed: print stats

Datagram layout (little endian):
    0   4s  magic "PC2X"
    4   B   version (1)
    5   B   point_step (12)
    6   H   num_points
    8   I   seq
    12  H   azimuth of first point, 0.01 deg
    14  H   flags
    16  ... num_points * (float32 x, float32 y, float32 z), metres

The point area is copied unmodified into PointCloud2.data.
"""
import argparse
import socket
import struct
import time

HEADER = struct.Struct("<4sBBHIHH")
MAGIC = b"PC2X"


class Assembler:
    """Groups datagrams into revolutions (split where the azimuth wraps)."""

    def __init__(self):
        self.chunks = []
        self.last_az = None
        self.last_seq = None
        self.lost = 0
        self.bad = 0

    def feed(self, dgram):
        """Returns the finished revolution as bytes, or None."""
        if len(dgram) < HEADER.size:
            self.bad += 1
            return None
        magic, ver, step, n, seq, az, _flags = HEADER.unpack_from(dgram)
        if magic != MAGIC or ver != 1 or step != 12 or len(dgram) != HEADER.size + n * step:
            self.bad += 1
            return None
        if self.last_seq is not None:
            self.lost += (seq - self.last_seq - 1) & 0xFFFFFFFF
        self.last_seq = seq

        done = None
        if self.last_az is not None and az < self.last_az and self.chunks:
            done = b"".join(self.chunks)
            self.chunks = []
        self.last_az = az
        self.chunks.append(dgram[HEADER.size:])
        return done


def open_socket(port, bind_addr):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    sock.bind((bind_addr, port))
    return sock


def run_plain(args):
    sock = open_socket(args.port, args.bind)
    asm = Assembler()
    print(f"listening on udp/{args.port} ...")
    t0, revs, pts = time.time(), 0, 0
    while True:
        cloud = asm.feed(sock.recv(2048))
        if cloud is None:
            continue
        revs += 1
        pts += len(cloud) // 12
        if time.time() - t0 >= 1.0:
            x, y, z = struct.unpack_from("<3f", cloud)
            print(f"{revs} rev/s, {pts} pts/s, lost datagrams {asm.lost}, bad {asm.bad}, "
                  f"first point ({x:.3f}, {y:.3f}, {z:.3f}) m")
            t0, revs, pts = time.time(), 0, 0


def run_ros(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField

    rclpy.init()
    node = rclpy.create_node("pc2x_bridge")
    pub = node.create_publisher(PointCloud2, args.topic, qos_profile_sensor_data)
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate("xyz")]
    sock = open_socket(args.port, args.bind)
    sock.settimeout(0.5)
    asm = Assembler()
    node.get_logger().info(f"udp/{args.port} -> {args.topic} (frame '{args.frame_id}')")

    try:
        while rclpy.ok():
            try:
                cloud = asm.feed(sock.recv(2048))
            except socket.timeout:
                continue
            if cloud is None:
                continue
            msg = PointCloud2()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = args.frame_id
            msg.height = 1
            msg.width = len(cloud) // 12
            msg.fields = fields
            msg.is_bigendian = False
            msg.point_step = 12
            msg.row_step = len(cloud)
            msg.is_dense = True
            msg.data = cloud
            pub.publish(msg)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=2370, help="UDP port the FPGA sends to")
    ap.add_argument("--bind", default="0.0.0.0", help="local address to bind")
    ap.add_argument("--topic", default="fpga_points")
    ap.add_argument("--frame-id", default="velodyne")
    ap.add_argument("--no-ros", action="store_true", help="just print statistics")
    args = ap.parse_args()
    (run_plain if args.no_ros else run_ros)(args)


if __name__ == "__main__":
    main()
