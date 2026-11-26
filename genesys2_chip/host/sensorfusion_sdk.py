#!/usr/bin/env python3
"""
UMARV Sensor-Fusion Chip Python SDK (over ESP32 SPI Host bridge).

Zero third-party dependencies (stdlib only). Provides a clean, thread-safe API
to read live sensor observations, read/write every chip register and tuning
parameter, inject host pose states, and optionally bridge a USB VN-300T-CR
into the FPGA via the ESP32's GPIO 16/17 Pmod JD link.

Example:
    from sensorfusion_sdk import SensorFusionChip
    with SensorFusionChip("/dev/ttyUSB0") as chip:
        cfg = chip.read_config()
        tel = chip.read_telemetry()
        print(f"ID=0x{tel.chip_id:02X} yaw={tel.vn_yaw_deg:.2f} deg lidar={tel.cnt_lidar}")
"""
from __future__ import annotations

from dataclasses import dataclass
import fcntl
import glob
import math
import os
import select
import socket
import struct
import subprocess
import termios
import threading
import time
from typing import Dict, List, Optional, Tuple


LIDAR_MODELS = [
    "0: VLP-16 / Puck LITE",
    "1: Puck Hi-Res",
    "2: VLP-32C",
    "3: HDL-32E",
]

# UKF filter constants (addresses 96..111 via SPI reg 0x6C + 0x70..0x73).
# With NCFG=8 (default in ukf_top.sv), words 96..103 are host-writable while
# CTRL bit 7 = 0 (filter off & idle), and words 104..111 are readable constants.
UKF_CONST_DEFS: List[Tuple[int, str, bool, str, str]] = [
    # (word_addr, name, writable_ncfg8, units, description)
    (96,  "RR01",    True,  "m^2",        "Position measurement noise variance"),
    (97,  "RR2",     True,  "rad^2",      "Heading measurement noise variance"),
    (98,  "RR3",     True,  "(rad/s)^2",  "Turn-rate measurement noise variance"),
    (99,  "SQ01",    True,  "m",          "Position process noise std / beat"),
    (100, "SQ3",     True,  "rad/s",      "Turn-rate process noise std / beat"),
    (101, "SQ2",     True,  "rad",        "Heading process noise std / beat"),
    (102, "CNT2RAD", True,  "rad/cnt",    "Wheel radians per encoder count (2pi/16384)"),
    (103, "RNOM",    True,  "m",          "Starting dead-wheel radius"),
    (104, "YAWK",    False, "rad/deg",    "Heading scale (-pi/180)"),
    (105, "YAW0",    False, "rad",        "Heading offset (+pi/2)"),
    (106, "RATEK",   False, "1",          "Turn-rate sign/scale (-1.0)"),
    (107, "KLONU",   False, "m/cnt",      "Equator metres east per lon slice count"),
    (108, "KLAT",    False, "m/cnt",      "Metres north per lat slice count"),
    (109, "DT",      False, "s",          "Beat period (0.0025 s)"),
    (110, "HALFDT",  False, "s",          "Half beat period (0.00125 s)"),
    (111, "HZ",      False, "Hz",         "Beat frequency (400.0 Hz)"),
]

UKF_NAMED_WORDS: List[Tuple[int, str, str]] = [
    (0,   "OUT_TS",   "u32",),
    (1,   "OUT_X",    "f32",),
    (2,   "OUT_Y",    "f32",),
    (3,   "OUT_TH",   "f32",),
    (4,   "OUT_OM",   "f32",),
    (5,   "OUT_V",    "f32",),
    (6,   "XH_X",     "f32",),
    (7,   "XH_Y",     "f32",),
    (8,   "XH_TH",    "f32",),
    (9,   "XH_OM",    "f32",),
    (10,  "XH_R",     "f32",),
    (52,  "DPHI",     "f32",),
    (53,  "LAT0",     "u32",),
    (54,  "LON0",     "u32",),
    (55,  "KLON",     "f32",),
    (56,  "GS",       "f32",),
    (64,  "ZERO",     "f32",),
    (65,  "ONE",      "f32",),
    (96,  "RR01",     "f32",),
    (97,  "RR2",      "f32",),
    (98,  "RR3",      "f32",),
    (99,  "SQ01",     "f32",),
    (100, "SQ3",      "f32",),
    (101, "SQ2",      "f32",),
    (102, "CNT2RAD",  "f32",),
    (103, "RNOM",     "f32",),
    (120, "EXT_YAW",  "f32",),
    (121, "EXT_RATE", "f32",),
    (124, "EXT_DW",   "i32",),
    (125, "EXT_TS",   "u32",),
]

SPI_REGISTER_MAP: List[Tuple[int, str, str, str]] = [
    # (addr, name, access, description)
    (0x00, "CHIP_ID",        "RO", "Chip ID (0xA7)"),
    (0x01, "VERSION",        "RO", "Version (0x01)"),
    (0x02, "CTRL",           "RW", "Control bits [7:0] (reset 0x43)"),
    (0x03, "LIDAR_MODEL",    "RW", "0=VLP-16, 1=Hi-Res, 2=VLP-32C, 3=HDL-32E"),
    (0x04, "PERIOD_US_LO",   "RW", "Beat period us low byte (2500)"),
    (0x05, "PERIOD_US_HI",   "RW", "Beat period us high byte"),
    (0x06, "DEADLINE_US_LO", "RW", "Meas deadline us low byte (2000)"),
    (0x07, "DEADLINE_US_HI", "RW", "Meas deadline us high byte"),
    (0x08, "LIDAR_PORT_LO",  "RW", "Lidar UDP port low byte (0=any, reset 2368)"),
    (0x09, "LIDAR_PORT_HI",  "RW", "Lidar UDP port high byte"),
    (0x0A, "YAW_OFF_LO",     "RW", "Yaw offset 0.01 deg signed low byte"),
    (0x0B, "YAW_OFF_HI",     "RW", "Yaw offset 0.01 deg signed high byte"),
    (0x20, "SRC_PORT_LO",    "RW", "Outgoing UDP src port low byte (2370)"),
    (0x22, "DST_PORT_LO",    "RW", "Outgoing UDP dst port low byte (2370)"),
    (0x24, "HOST_TS",        "RW", "State ts u32 (0x24..27; UKF word 0, requires CTRL[7]=0)"),
    (0x28, "HOST_X",         "RW", "State x f32 (0x28..2B; UKF word 1, requires CTRL[7]=0)"),
    (0x2C, "HOST_Y",         "RW", "State y f32 (0x2C..2F; UKF word 2, requires CTRL[7]=0)"),
    (0x30, "HOST_THETA",     "RW", "State theta f32 (0x30..33; UKF word 3, requires CTRL[7]=0)"),
    (0x34, "HOST_OMEGA",     "RW", "State omega f32 (0x34..37; UKF word 4, requires CTRL[7]=0)"),
    (0x38, "HOST_V",         "RW", "State v f32 (0x38..3B; UKF word 5, requires CTRL[7]=0)"),
    (0x3C, "HOST_STROBE",    "WO", "Write any byte: strobe host state to lidar"),
    (0x3D, "CNT_CLEAR",      "WO", "Write any byte: clear event counters 0x40..46"),
    (0x40, "CNT_LIDAR",      "RO", "Lidar data packets parsed (sat 255)"),
    (0x4C, "VALID_BITS",     "RO", "bit 0=vn_valid, bit 1=dw_valid"),
    (0x6C, "UKF_WORD_ADDR",  "WO", "Kalman filter word address (0..126, reset 96)"),
    (0x70, "UKF_WORD_B0",    "RW", "Kalman filter word byte 0 (LSB); writes 96..103 if CTRL[7]=0"),
    (0x71, "UKF_WORD_B1",    "RW", "Kalman filter word byte 1"),
    (0x72, "UKF_WORD_B2",    "RW", "Kalman filter word byte 2"),
    (0x73, "UKF_WORD_B3",    "RW", "Kalman filter word byte 3 (MSB)"),
]


def _vn_nmea(body: str) -> bytes:
    ck = 0
    for ch in body.encode("ascii"):
        ck ^= ch
    return f"${body}*{ck:02X}\r\n".encode("ascii")


def _verify_vn_nmea(line: str) -> Optional[str]:
    """Validate '$BODY*XX' XOR checksum and return BODY, or None if invalid."""
    s = line.strip()
    if not s.startswith("$") or "*" not in s:
        return None
    body, chk = s[1:].rsplit("*", 1)
    if len(chk) < 2:
        return None
    try:
        want = int(chk[:2], 16)
    except ValueError:
        return None
    ck = 0
    for b in body.encode("ascii", errors="ignore"):
        ck ^= b
    return body if ck == want else None


GNSS_FIX_NAMES: Dict[int, str] = {
    0: "No Fix",
    1: "Time Only",
    2: "2D Fix",
    3: "3D Fix",
}

INS_MODE_NAMES: Dict[int, str] = {
    0: "Not Tracking",
    1: "Aligning",
    2: "Tracking",
    3: "Loss of GPS",
}


@dataclass
class VN300GnssStatus:
    """Live VN-300T-CR GNSS1/GNSS2 satellite count, fix type, and INS status."""
    gnss1_sats: int = 0
    gnss1_fix: int = 0
    gnss1_lat_deg: float = 0.0
    gnss1_lon_deg: float = 0.0
    gnss1_alt_m: float = 0.0
    gnss1_pos_u_m: float = 0.0
    gnss2_sats: int = 0
    gnss2_fix: int = 0
    gnss2_lat_deg: float = 0.0
    gnss2_lon_deg: float = 0.0
    gnss2_alt_m: float = 0.0
    gnss2_pos_u_m: float = 0.0
    ins_status: int = 0
    last_gnss_monotonic: float = 0.0
    last_ins_monotonic: float = 0.0

    @property
    def gnss1_fix_name(self) -> str:
        return GNSS_FIX_NAMES.get(self.gnss1_fix, f"Fix({self.gnss1_fix})")

    @property
    def gnss2_fix_name(self) -> str:
        return GNSS_FIX_NAMES.get(self.gnss2_fix, f"Fix({self.gnss2_fix})")

    @property
    def ins_mode(self) -> int:
        return self.ins_status & 0x03

    @property
    def ins_mode_name(self) -> str:
        return INS_MODE_NAMES.get(self.ins_mode, f"Mode({self.ins_mode})")

    @property
    def ins_gps_fix(self) -> bool:
        return bool(self.ins_status & 0x04)

    @property
    def ins_gps_compass(self) -> bool:
        return bool(self.ins_status & 0x200)

    @property
    def chip_status_ok(self) -> bool:
        """Exact match to timestamper.sv vn_good status check."""
        return (self.ins_status & 0x03) == 2 and bool(self.ins_status & 0x04) and ((self.ins_status >> 3) & 0x0F) == 0


def _parse_vn_gnss_nmea(line: str) -> Optional[Tuple[int, int, int, float, float, float, float]]:
    """
    Parse a checksum-verified VN-300 Reg 58 (GNSS1) or Reg 103 (GNSS2) ASCII line.
    Returns (reg_id, gnss_fix, num_sats, lat, lon, alt, horiz_pos_u_m) or None.
    """
    body = _verify_vn_nmea(line)
    if not body:
        return None
    parts = body.split(",")
    if len(parts) < 15 or parts[0] != "VNRRG":
        return None
    try:
        reg_id = int(parts[1])
        if reg_id not in (58, 103):
            return None
        gnss_fix = int(parts[4])
        num_sats = int(parts[5])
        lat = float(parts[6])
        lon = float(parts[7])
        alt = float(parts[8])
        pu_n = float(parts[12])
        pu_e = float(parts[13])
        horiz_u = math.hypot(pu_n, pu_e)
        return (reg_id, gnss_fix, num_sats, lat, lon, alt, horiz_u)
    except (ValueError, IndexError):
        return None


def _crc16_ccitt(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc >> 8) | ((crc & 0xFF) << 8)) & 0xFFFF
        crc ^= b
        crc ^= (crc & 0xFF) >> 4
        crc ^= (crc << 12) & 0xFFFF
        crc ^= ((crc & 0xFF) << 5) & 0xFFFF
    return crc


def list_serial_ports() -> List[Tuple[str, str]]:
    """Return [(device_path, description), ...] for connected USB serial ports."""
    out: List[Tuple[str, str]] = []
    for p in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        desc = p
        try:
            props = subprocess.check_output(
                ["udevadm", "info", "-q", "property", "-n", p],
                text=True,
                stderr=subprocess.DEVNULL,
            )
            kv = dict(line.split("=", 1) for line in props.splitlines() if "=" in line)
            vid = kv.get("ID_VENDOR_ID", "")
            model = kv.get("ID_MODEL", "").replace("_", " ")
            if vid == "10c4":
                desc = f"{p} (ESP32 CP210x)"
            elif vid == "0403" and "RS232" in model.upper():
                desc = f"{p} (VN-300 FTDI RS232)"
            elif model:
                desc = f"{p} ({model})"
        except Exception:
            pass
        out.append((p, desc))
    return out


def autodetect_ports() -> Tuple[Optional[str], Optional[str]]:
    """Return (esp_port, vn300_port) by inspecting udev USB IDs."""
    esp_port = None
    vn_port = None
    for path, desc in list_serial_ports():
        if "ESP32" in desc and esp_port is None:
            esp_port = path
        elif "VN-300" in desc and vn_port is None:
            vn_port = path
    return esp_port, vn_port


def format_mac(b: bytes) -> str:
    return ":".join(f"{x:02X}" for x in b[:6])


def parse_mac(s: str) -> bytes:
    parts = [p.strip() for p in s.replace("-", ":").split(":") if p.strip()]
    if len(parts) != 6:
        raise ValueError(f"Invalid MAC address: {s!r}")
    return bytes(int(p, 16) for p in parts)


def format_ip(b: bytes) -> str:
    return ".".join(str(x) for x in b[:4])


def parse_ip(s: str) -> bytes:
    parts = [p.strip() for p in s.split(".") if p.strip()]
    if len(parts) != 4:
        raise ValueError(f"Invalid IPv4 address: {s!r}")
    vals = [int(p) for p in parts]
    if any(v < 0 or v > 255 for v in vals):
        raise ValueError(f"IPv4 octet out of range: {s!r}")
    return bytes(vals)


@dataclass
class ChipConfig:
    """Writable chip configuration registers 0x02..0x23."""
    ts_enable: bool = True
    lidar_enable: bool = True
    lidar_bypass: bool = False
    scale_4mm: bool = False
    corr_rot_en: bool = False
    corr_shift_en: bool = False
    dw_check_turns: bool = True
    state_from_filter: bool = False
    lidar_model: int = 0
    period_us: int = 2500
    deadline_us: int = 1000
    lidar_port: int = 2368
    yaw_offset_deg: float = 0.0
    src_mac: str = "02:00:00:00:00:01"
    dst_mac: str = "FF:FF:FF:FF:FF:FF"
    src_ip: str = "192.168.1.50"
    dst_ip: str = "255.255.255.255"
    src_port: int = 2370
    dst_port: int = 2370

    @property
    def ctrl_byte(self) -> int:
        return (
            (int(self.ts_enable) << 0)
            | (int(self.lidar_enable) << 1)
            | (int(self.lidar_bypass) << 2)
            | (int(self.scale_4mm) << 3)
            | (int(self.corr_rot_en) << 4)
            | (int(self.corr_shift_en) << 5)
            | (int(self.dw_check_turns) << 6)
            | (int(self.state_from_filter) << 7)
        )

    @classmethod
    def from_regs(cls, regs_02_to_23: bytes) -> "ChipConfig":
        if len(regs_02_to_23) < 0x22:
            raise ValueError("Expected 34 bytes (regs 0x02..0x23)")
        b = regs_02_to_23
        ctrl = b[0]
        model = b[1] & 0x03
        period_us, deadline_us, lidar_port, yaw_raw = struct.unpack_from("<HHHh", b, 2)
        src_mac = format_mac(b[10:16])
        dst_mac = format_mac(b[16:22])
        src_ip = format_ip(b[22:26])
        dst_ip = format_ip(b[26:30])
        src_port, dst_port = struct.unpack_from("<HH", b, 30)
        return cls(
            ts_enable=bool(ctrl & 0x01),
            lidar_enable=bool(ctrl & 0x02),
            lidar_bypass=bool(ctrl & 0x04),
            scale_4mm=bool(ctrl & 0x08),
            corr_rot_en=bool(ctrl & 0x10),
            corr_shift_en=bool(ctrl & 0x20),
            dw_check_turns=bool(ctrl & 0x40),
            state_from_filter=bool(ctrl & 0x80),
            lidar_model=model,
            period_us=period_us,
            deadline_us=deadline_us,
            lidar_port=lidar_port,
            yaw_offset_deg=yaw_raw * 0.01,
            src_mac=src_mac,
            dst_mac=dst_mac,
            src_ip=src_ip,
            dst_ip=dst_ip,
            src_port=src_port,
            dst_port=dst_port,
        )

    def to_regs(self) -> bytes:
        yaw_raw = int(round(self.yaw_offset_deg * 100.0))
        yaw_raw = max(-32768, min(32767, yaw_raw))
        return (
            struct.pack(
                "<BBHHHh",
                self.ctrl_byte,
                self.lidar_model & 0x03,
                int(self.period_us) & 0xFFFF,
                int(self.deadline_us) & 0xFFFF,
                int(self.lidar_port) & 0xFFFF,
                yaw_raw,
            )
            + parse_mac(self.src_mac)
            + parse_mac(self.dst_mac)
            + parse_ip(self.src_ip)
            + parse_ip(self.dst_ip)
            + struct.pack("<HH", int(self.src_port) & 0xFFFF, int(self.dst_port) & 0xFFFF)
        )


@dataclass
class HostState:
    """Host-injected pose state (regs 0x24..0x3B)."""
    ts_us: int = 0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    omega: float = 0.0
    v: float = 0.0

    @classmethod
    def from_regs(cls, regs_24_to_3b: bytes) -> "HostState":
        ts_us, x, y, theta, omega, v = struct.unpack("<Ifffff", regs_24_to_3b[:24])
        return cls(ts_us=ts_us, x=x, y=y, theta=theta, omega=omega, v=v)

    def to_regs(self) -> bytes:
        return struct.pack(
            "<Ifffff",
            int(self.ts_us) & 0xFFFFFFFF,
            float(self.x),
            float(self.y),
            float(self.theta),
            float(self.omega),
            float(self.v),
        )


@dataclass
class UkfState:
    """Live Kalman filter output state (words 0..5) + learned wheel radius (word 10)."""
    ts_us: int = 0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    omega: float = 0.0
    v: float = 0.0
    wheel_r: float = 0.05


@dataclass
class ChipTelemetry:
    """Live observed chip status, counters, and sensor measurements."""
    timestamp: float
    chip_id: int
    version: int
    ctrl: int
    lidar_model: int
    cnt_lidar: int
    cnt_other: int
    cnt_rx_crc_bad: int
    cnt_sent: int
    cnt_buf_ovf: int
    cnt_mot_ovr: int
    cnt_tx_und: int
    beat_ts_us: int
    vn_valid: bool
    dw_valid: bool
    vn_yaw_deg: float
    vn_rate_rads: float
    vn_lat_deg: float
    vn_lon_deg: float
    dw_delta_counts: int
    esp_sync_irq: int = 0
    esp_vn_tx: int = 0
    esp_vn_bridge_en: bool = False
    dw_abs_counts: int = 0
    dw_good_beats: int = 0
    dw_last_valid: bool = False


class SensorFusionChip:
    """Thread-safe Python SDK client talking to the Genesys 2 chip via ESP32 SPI."""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200, timeout: float = 0.6):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._fd: Optional[int] = None
        self._lock = threading.Lock()
        self._rx_buf = bytearray()

    BAUD_MAP = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
        460800: termios.B460800,
        921600: termios.B921600,
    }

    def connect(self) -> None:
        with self._lock:
            if self._fd is not None:
                return
            # Open O_NONBLOCK and configure with TCSANOW + TCOON directly.
            # Avoids stty (TCSADRAIN hangs if boot bytes at 9600 trigger IXON XOFF)
            # and clears Linux CIBAUD bits in c_cflag so RX/TX speeds always match.
            fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            try:
                bconst = self.BAUD_MAP[self.baud]
                attrs = termios.tcgetattr(fd)
                attrs[0] = 0                                                   # iflag (clears IXON/ICRNL)
                attrs[1] = 0                                                   # oflag
                attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | bconst  # cflag (clears CIBAUD/HUPCL/CRTSCTS)
                attrs[3] = 0                                                   # lflag
                attrs[4] = bconst                                              # ispeed
                attrs[5] = bconst                                              # ospeed
                attrs[6][termios.VMIN] = 0
                attrs[6][termios.VTIME] = 0
                termios.tcsetattr(fd, termios.TCSANOW, attrs)
                termios.tcflow(fd, termios.TCOON)
                # Release DTR & RTS so ESP32 stays running cleanly
                fcntl.ioctl(fd, 0x5417, struct.pack("I", 0x006))
                termios.tcflush(fd, termios.TCIOFLUSH)
            except Exception:
                os.close(fd)
                raise
            self._fd = fd
            self._rx_buf.clear()
        # Ensure auto-dump is off and verify burst protocol works
        self._send_raw(b"\na 0\n")
        time.sleep(0.05)
        with self._lock:
            if self._fd is not None:
                termios.tcflush(self._fd, termios.TCIOFLUSH)
                self._rx_buf.clear()
        hdr = self.read_regs(0x00, 2)
        if hdr[0] != 0xA7:
            raise RuntimeError(
                f"Unexpected chip ID 0x{hdr[0]:02X} (expected 0xA7). "
                "Check Genesys 2 power/bitstream and Pmod JC SPI wiring."
            )

    def close(self) -> None:
        with self._lock:
            if self._fd is not None:
                try:
                    os.close(self._fd)
                except OSError:
                    pass
                self._fd = None
                self._rx_buf.clear()

    def __enter__(self) -> "SensorFusionChip":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @property
    def is_connected(self) -> bool:
        return self._fd is not None

    def _send_raw(self, data: bytes) -> None:
        with self._lock:
            if self._fd is None:
                raise RuntimeError("Not connected")
            view = memoryview(data)
            written = 0
            while written < len(data):
                try:
                    n = os.write(self._fd, view[written:])
                    written += n
                except BlockingIOError:
                    select.select([], [self._fd], [], 0.1)

    def _transact_prefix(self, cmd_line: str, expected_prefix: str) -> str:
        with self._lock:
            if self._fd is None:
                raise RuntimeError("Not connected")
            # Drain any stale buffered bytes before issuing command
            while select.select([self._fd], [], [], 0.0)[0]:
                try:
                    chunk = os.read(self._fd, 4096)
                    if not chunk:
                        break
                except BlockingIOError:
                    break
            self._rx_buf.clear()

            payload = (cmd_line.rstrip("\r\n") + "\n").encode("ascii")
            written = 0
            while written < len(payload):
                try:
                    written += os.write(self._fd, payload[written:])
                except BlockingIOError:
                    select.select([], [self._fd], [], 0.1)

            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                rem = max(0.01, deadline - time.monotonic())
                r, _, _ = select.select([self._fd], [], [], min(rem, 0.05))
                if r:
                    try:
                        chunk = os.read(self._fd, 4096)
                        if chunk:
                            self._rx_buf.extend(chunk)
                    except BlockingIOError:
                        pass
                while b"\n" in self._rx_buf:
                    idx = self._rx_buf.index(b"\n")
                    line = bytes(self._rx_buf[:idx]).decode("utf-8", errors="replace").strip()
                    del self._rx_buf[: idx + 1]
                    if line.startswith(expected_prefix):
                        return line
                    if line.startswith("@ERR"):
                        raise RuntimeError(f"ESP32 command error: {line}")
            raise TimeoutError(f"Timed out waiting for '{expected_prefix}' after '{cmd_line}'")

    def read_regs(self, addr: int, length: int) -> bytes:
        """Read `length` contiguous registers starting at `addr` (0x00..0x7F)."""
        if length <= 0:
            return b""
        out = bytearray()
        cur = addr & 0x7F
        rem = length
        while rem > 0:
            chunk = min(rem, 64)
            line = self._transact_prefix(f"R {cur:02X} {chunk}", "@R ")
            parts = line.split()
            if len(parts) < 3:
                raise RuntimeError(f"Malformed @R response: {line!r}")
            raw = bytes.fromhex(parts[2])
            if len(raw) != chunk:
                raise RuntimeError(f"Short @R response: got {len(raw)} bytes, expected {chunk}")
            out.extend(raw)
            cur = (cur + chunk) & 0x7F
            rem -= chunk
        return bytes(out)

    def write_regs(self, addr: int, data: bytes) -> bytes:
        """Write contiguous registers starting at `addr` and return SPI readback bytes."""
        if not data:
            return b""
        rb = bytearray()
        cur = addr & 0x7F
        off = 0
        while off < len(data):
            chunk = data[off : off + 64]
            line = self._transact_prefix(f"W {cur:02X} {chunk.hex().upper()}", "@W ")
            parts = line.split()
            if len(parts) < 3:
                raise RuntimeError(f"Malformed @W response: {line!r}")
            rb.extend(bytes.fromhex(parts[2]))
            cur = (cur + len(chunk)) & 0x7F
            off += len(chunk)
        return bytes(rb)

    def read_reg(self, addr: int) -> int:
        return self.read_regs(addr, 1)[0]

    def write_reg(self, addr: int, val: int) -> int:
        return self.write_regs(addr, bytes([val & 0xFF]))[0]

    def read_config(self) -> ChipConfig:
        return ChipConfig.from_regs(self.read_regs(0x02, 0x22))

    def write_config(self, cfg: ChipConfig) -> ChipConfig:
        rb = self.write_regs(0x02, cfg.to_regs())
        return ChipConfig.from_regs(rb)

    def read_host_state(self) -> HostState:
        return HostState.from_regs(self.read_regs(0x24, 24))

    def inject_host_state(self, state: HostState, auto_disable_ukf: bool = False) -> None:
        """
        Write host state to 0x24..0x3B and strobe 0x3C.
        With WITH_UKF, 0x24..0x3B live in UKF storage words 0..5 and are only accepted
        while CTRL bit 7 (state_from_filter) == 0 and the UKF is idle (~2.5 ms).
        """
        ctrl = self.read_reg(0x02)
        if ctrl & 0x80:
            if not auto_disable_ukf:
                raise RuntimeError(
                    "CTRL bit 7 (Use On-Chip UKF State) is ON. Clear bit 7 (or enable auto-pause) "
                    "before writing host state registers 0x24..0x3B."
                )
            self.write_reg(0x02, ctrl & ~0x80)
            time.sleep(0.004)  # allow active UKF beat (<2.5 ms) to return to Q_IDLE
        self.write_regs(0x24, state.to_regs())
        self.write_reg(0x3C, 0x01)

    def clear_counters(self) -> None:
        self.write_reg(0x3D, 0x01)

    # ---- Kalman filter word window (regs 0x6C + 0x70..0x73) ------------------
    def read_ukf_word_u32(self, word_addr: int) -> int:
        """Select filter word `word_addr` (0..126) in reg 0x6C and read 32-bit word at 0x70..0x73."""
        self.write_reg(0x6C, word_addr & 0x7F)
        raw = self.read_regs(0x70, 4)
        return struct.unpack("<I", raw)[0]

    def read_ukf_word_f32(self, word_addr: int) -> float:
        u = self.read_ukf_word_u32(word_addr)
        return struct.unpack("<f", struct.pack("<I", u))[0]

    def write_ukf_word_u32(
        self, word_addr: int, value_u32: int, auto_pause_ukf: bool = True
    ) -> int:
        """
        Write 32-bit word to tunable UKF constant `word_addr` (96..103 with NCFG=8)
        via reg 0x6C + 0x70..0x73. Returns the SPI readback uint32.
        """
        waddr = word_addr & 0x7F
        ctrl_orig = self.read_reg(0x02)
        paused = False
        try:
            if (ctrl_orig & 0x80) != 0:
                if not auto_pause_ukf:
                    raise RuntimeError(
                        "Cannot write UKF constant while CTRL bit 7 (UKF) is enabled. "
                        "Disable CTRL bit 7 or enable auto_pause_ukf."
                    )
                self.write_reg(0x02, ctrl_orig & ~0x80)
                paused = True
                time.sleep(0.004)  # allow active beat (<2.5 ms) to reach Q_IDLE
            self.write_reg(0x6C, waddr)
            raw = struct.pack("<I", value_u32 & 0xFFFFFFFF)
            rb = self.write_regs(0x70, raw)
            return struct.unpack("<I", rb)[0]
        finally:
            if paused:
                self.write_reg(0x02, ctrl_orig)

    def write_ukf_word_f32(
        self, word_addr: int, value_f32: float, auto_pause_ukf: bool = True
    ) -> float:
        u_in = struct.unpack("<I", struct.pack("<f", float(value_f32)))[0]
        u_out = self.write_ukf_word_u32(word_addr, u_in, auto_pause_ukf=auto_pause_ukf)
        return struct.unpack("<f", struct.pack("<I", u_out))[0]

    def read_ukf_tunables(self) -> Dict[int, Tuple[float, int]]:
        """Read all 16 UKF tunable constant slots (96..111) -> {addr: (f32, u32)}."""
        out: Dict[int, Tuple[float, int]] = {}
        for addr, _name, _wr, _unit, _desc in UKF_CONST_DEFS:
            u = self.read_ukf_word_u32(addr)
            f = struct.unpack("<f", struct.pack("<I", u))[0]
            out[addr] = (f, u)
        return out

    def write_ukf_tunables(
        self, values_f32: Dict[int, float], auto_pause_ukf: bool = True
    ) -> Dict[int, Tuple[float, int]]:
        """Batch-write tunable UKF constants (96..103) with a single pause/restore."""
        ctrl_orig = self.read_reg(0x02)
        paused = False
        out: Dict[int, Tuple[float, int]] = {}
        try:
            if (ctrl_orig & 0x80) != 0:
                if not auto_pause_ukf:
                    raise RuntimeError("Disable CTRL bit 7 (UKF) before writing UKF constants.")
                self.write_reg(0x02, ctrl_orig & ~0x80)
                paused = True
                time.sleep(0.004)
            for addr, fval in values_f32.items():
                u_in = struct.unpack("<I", struct.pack("<f", float(fval)))[0]
                self.write_reg(0x6C, addr & 0x7F)
                rb = self.write_regs(0x70, struct.pack("<I", u_in))
                u_out = struct.unpack("<I", rb)[0]
                f_out = struct.unpack("<f", struct.pack("<I", u_out))[0]
                out[addr] = (f_out, u_out)
            return out
        finally:
            if paused:
                self.write_reg(0x02, ctrl_orig)

    def read_ukf_state(self) -> UkfState:
        """Read live UKF state words 0..5 (via 0x24..0x3B) + learned wheel radius word 10."""
        st = self.read_host_state()
        r = self.read_ukf_word_f32(10)
        return UkfState(
            ts_us=st.ts_us,
            x=st.x,
            y=st.y,
            theta=st.theta,
            omega=st.omega,
            v=st.v,
            wheel_r=r,
        )

    def read_bridge_status(self) -> Tuple[int, int, bool, int, int, bool]:
        """Return (sync_irq, vn_tx, bridge_en, dw_abs_counts, dw_good_beats, dw_last_valid)."""
        line = self._transact_prefix("S", "@S ")
        parts = line.split()
        if len(parts) >= 7:
            return (
                int(parts[1]),
                int(parts[2]),
                bool(int(parts[3])),
                int(parts[4]),
                int(parts[5]),
                bool(int(parts[6])),
            )
        if len(parts) >= 4:
            return int(parts[1]), int(parts[2]), bool(int(parts[3])), 0, 0, False
        return 0, 0, False, 0, 0, False

    def zero_deadwheel(self) -> None:
        """Reset the ESP32's 400 Hz dead-wheel absolute count accumulator to 0."""
        self._transact_prefix("Z", "@Z ")

    @staticmethod
    def deadwheel_distance_m(abs_counts: int, cnt2rad: float = 0.0003834951969714103, wheel_radius_m: float = 0.05) -> float:
        """Convert accumulated encoder counts to linear distance in metres."""
        return float(abs_counts) * float(cnt2rad) * float(wheel_radius_m)

    def set_vn300_frame(self, frame56: bytes) -> None:
        if len(frame56) != 56:
            raise ValueError("VN-300 binary frame must be 56 bytes")
        self._send_raw(f"v {frame56.hex()}\n".encode("ascii"))

    def disable_vn300_bridge(self) -> None:
        self._send_raw(b"v 0\n")

    def read_telemetry(self, include_bridge: bool = True) -> ChipTelemetry:
        hdr = self.read_regs(0x00, 4)
        blk = self.read_regs(0x40, 0x2C)  # 0x40..0x6B (44 bytes)
        c0, c1, c2, c3, c4, c5, c6 = blk[0:7]
        beat_ts = struct.unpack_from("<I", blk, 0x08)[0]
        vbits = blk[0x0C]
        yaw, rate = struct.unpack_from("<ff", blk, 0x10)
        lat, lon = struct.unpack_from("<dd", blk, 0x18)
        dw_delta = struct.unpack_from("<i", blk, 0x28)[0]

        sync_irq, vn_tx, br_en, dw_abs, dw_beats, dw_last_ok = (0, 0, False, 0, 0, False)
        if include_bridge:
            try:
                sync_irq, vn_tx, br_en, dw_abs, dw_beats, dw_last_ok = self.read_bridge_status()
            except Exception:
                pass

        return ChipTelemetry(
            timestamp=time.monotonic(),
            chip_id=hdr[0],
            version=hdr[1],
            ctrl=hdr[2],
            lidar_model=hdr[3] & 0x03,
            cnt_lidar=c0,
            cnt_other=c1,
            cnt_rx_crc_bad=c2,
            cnt_sent=c3,
            cnt_buf_ovf=c4,
            cnt_mot_ovr=c5,
            cnt_tx_und=c6,
            beat_ts_us=beat_ts,
            vn_valid=bool(vbits & 0x01),
            dw_valid=bool(vbits & 0x02),
            vn_yaw_deg=yaw,
            vn_rate_rads=rate,
            vn_lat_deg=lat,
            vn_lon_deg=lon,
            dw_delta_counts=dw_delta,
            esp_sync_irq=sync_irq,
            esp_vn_tx=vn_tx,
            esp_vn_bridge_en=br_en,
            dw_abs_counts=dw_abs,
            dw_good_beats=dw_beats,
            dw_last_valid=dw_last_ok,
        )


class VN300USBBridge:
    """Background worker forwarding live USB VN-300T-CR frames into SensorFusionChip."""

    BAUD_MAP = {
        115200: termios.B115200,
        230400: termios.B230400,
        460800: termios.B460800,
        921600: termios.B921600,
    }

    def __init__(self, chip: SensorFusionChip, vn_port: str = "/dev/ttyUSB1", vn_baud: int = 230400, gnss_poll_sec: float = 1.0):
        self.chip = chip
        self.vn_port = vn_port
        self.vn_baud = vn_baud
        self.gnss_poll_sec = max(0.25, float(gnss_poll_sec))
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._gnss_lock = threading.Lock()
        self._gnss = VN300GnssStatus()
        self.frames_forwarded = 0
        self.last_error: Optional[str] = None

    def get_gnss_status(self) -> VN300GnssStatus:
        """Return a thread-safe snapshot of live GNSS1/GNSS2 satellite counts, fix, and INS status."""
        with self._gnss_lock:
            return VN300GnssStatus(**self._gnss.__dict__)

    @classmethod
    def query_gnss_status(cls, vn_port: str, vn_baud: int = 230400, timeout: float = 0.8) -> VN300GnssStatus:
        """One-shot standalone query of VN-300 GNSS1 (Reg 58), GNSS2 (Reg 103), and INS status (Reg 63)."""
        status = VN300GnssStatus()
        fd = os.open(vn_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            bconst = cls.BAUD_MAP[vn_baud]
            attrs = termios.tcgetattr(fd)
            attrs[0] = attrs[1] = attrs[3] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | bconst
            attrs[4] = attrs[5] = bconst
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIOFLUSH)

            os.write(fd, _vn_nmea("VNRRG,58"))
            os.write(fd, _vn_nmea("VNRRG,103"))
            os.write(fd, _vn_nmea("VNRRG,63"))

            buf = bytearray()
            got = set()
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and len(got) < 3:
                r, _, _ = select.select([fd], [], [], 0.05)
                if not r:
                    continue
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not chunk:
                    continue
                buf.extend(chunk)
                while b"\n" in buf:
                    nl = buf.index(b"\n")
                    raw_line = bytes(buf[: nl + 1])
                    del buf[: nl + 1]
                    text = raw_line.decode("ascii", errors="ignore")
                    dollar = text.find("$VNRRG,")
                    if dollar < 0:
                        continue
                    text = text[dollar:]
                    parsed = _parse_vn_gnss_nmea(text)
                    if parsed is not None:
                        reg_id, gfix, nsats, lat, lon, alt, pu = parsed
                        now = time.monotonic()
                        if reg_id == 58:
                            status.gnss1_fix, status.gnss1_sats = gfix, nsats
                            status.gnss1_lat_deg, status.gnss1_lon_deg, status.gnss1_alt_m, status.gnss1_pos_u_m = lat, lon, alt, pu
                        else:
                            status.gnss2_fix, status.gnss2_sats = gfix, nsats
                            status.gnss2_lat_deg, status.gnss2_lon_deg, status.gnss2_alt_m, status.gnss2_pos_u_m = lat, lon, alt, pu
                        status.last_gnss_monotonic = now
                        got.add(reg_id)
                        continue
                    body = _verify_vn_nmea(text)
                    if body:
                        parts = body.split(",")
                        if len(parts) >= 5 and parts[0] == "VNRRG" and parts[1] == "63":
                            try:
                                status.ins_status = int(parts[4], 16)
                                status.last_ins_monotonic = time.monotonic()
                                got.add(63)
                            except ValueError:
                                pass
        finally:
            try:
                os.close(fd)
            except OSError:
                pass
        return status

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.is_running:
            return
        self._stop.clear()
        self.last_error = None
        self.frames_forwarded = 0
        self._thread = threading.Thread(target=self._run, name="VN300USBBridge", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None

    def _open_vn(self) -> int:
        fd = os.open(self.vn_port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        bconst = self.BAUD_MAP[self.vn_baud]
        attrs = termios.tcgetattr(fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | bconst
        attrs[3] = 0
        attrs[4] = bconst
        attrs[5] = bconst
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        return fd

    def _consume_ascii_bytes(self, data: bytes, ascii_buf: bytearray) -> None:
        if not data:
            return
        ascii_buf.extend(data)
        if len(ascii_buf) > 2048:
            del ascii_buf[:-1024]
        while b"\n" in ascii_buf:
            nl = ascii_buf.index(b"\n")
            raw_line = bytes(ascii_buf[: nl + 1])
            del ascii_buf[: nl + 1]
            text = raw_line.decode("ascii", errors="ignore")
            dollar = text.find("$VNRRG,")
            if dollar < 0:
                continue
            parsed = _parse_vn_gnss_nmea(text[dollar:])
            if parsed is not None:
                reg_id, gfix, nsats, lat, lon, alt, pu = parsed
                now = time.monotonic()
                with self._gnss_lock:
                    if reg_id == 58:
                        self._gnss.gnss1_fix = gfix
                        self._gnss.gnss1_sats = nsats
                        self._gnss.gnss1_lat_deg = lat
                        self._gnss.gnss1_lon_deg = lon
                        self._gnss.gnss1_alt_m = alt
                        self._gnss.gnss1_pos_u_m = pu
                    elif reg_id == 103:
                        self._gnss.gnss2_fix = gfix
                        self._gnss.gnss2_sats = nsats
                        self._gnss.gnss2_lat_deg = lat
                        self._gnss.gnss2_lon_deg = lon
                        self._gnss.gnss2_alt_m = alt
                        self._gnss.gnss2_pos_u_m = pu
                    self._gnss.last_gnss_monotonic = now

    def _run(self) -> None:
        fd = None
        try:
            fd = self._open_vn()
            os.write(fd, _vn_nmea("VNWRG,06,0"))
            time.sleep(0.05)
            os.write(fd, _vn_nmea("VNWRG,75,1,20,01,1068"))
            rx_buf = bytearray()
            ascii_buf = bytearray()
            next_gnss_poll = 0.0  # trigger immediate initial satellite query

            while not self._stop.is_set():
                now = time.monotonic()
                if now >= next_gnss_poll:
                    try:
                        os.write(fd, _vn_nmea("VNRRG,58"))
                        os.write(fd, _vn_nmea("VNRRG,103"))
                    except OSError:
                        pass
                    next_gnss_poll = now + self.gnss_poll_sec

                r, _, _ = select.select([fd], [], [], 0.1)
                if not r:
                    continue
                try:
                    chunk = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not chunk:
                    continue
                rx_buf.extend(chunk)
                while len(rx_buf) >= 56:
                    idx = rx_buf.find(b"\xFA\x01\x68\x10")
                    if idx < 0:
                        self._consume_ascii_bytes(bytes(rx_buf[:-3]), ascii_buf)
                        del rx_buf[:-3]
                        break
                    if idx > 0:
                        self._consume_ascii_bytes(bytes(rx_buf[:idx]), ascii_buf)
                        del rx_buf[:idx]
                    if len(rx_buf) < 56:
                        break
                    pkt = bytes(rx_buf[:56])
                    del rx_buf[:56]
                    if _crc16_ccitt(pkt[1:]) == 0:
                        self.frames_forwarded += 1
                        ins_st = struct.unpack_from("<H", pkt, 52)[0]
                        with self._gnss_lock:
                            self._gnss.ins_status = ins_st
                            self._gnss.last_ins_monotonic = time.monotonic()
                        if self.chip.is_connected:
                            self.chip.set_vn300_frame(pkt)
                    else:
                        # False sync match inside ASCII or noise: keep bytes after first byte
                        rx_buf[0:0] = pkt[1:]
        except Exception as exc:
            self.last_error = str(exc)
        finally:
            if fd is not None:
                try:
                    os.write(fd, _vn_nmea("VNWRG,75,0,0,00"))
                    time.sleep(0.05)
                    os.write(fd, _vn_nmea("VNWRG,06,22"))
                    time.sleep(0.03)
                except OSError:
                    pass
                try:
                    os.close(fd)
                except OSError:
                    pass
            try:
                if self.chip.is_connected:
                    self.chip.disable_vn300_bridge()
            except Exception:
                pass


@dataclass
class LidarSweep:
    """One complete 360-degree PC2X v3 lidar sweep decoded from UDP."""
    sweep_id: int
    timestamp_us: int
    flags: int
    scale_m: float
    state_valid: bool
    state_ts_us: int
    pose_x: float
    pose_y: float
    pose_theta: float
    pose_omega: float
    pose_v: float
    src_ip: str
    points_xyz: List[Tuple[float, float, float]]


@dataclass
class LidarStreamStats:
    """Rolling 1-second PC2X v3 UDP stream statistics."""
    dgrams_per_sec: float = 0.0
    sweeps_per_sec: float = 0.0
    pts_per_sec: float = 0.0
    total_lost: int = 0
    total_bad: int = 0
    last_rx_monotonic: float = 0.0


class PC2XReceiver:
    """
    Background UDP listener for PC2X v3 motion-corrected point-cloud datagrams.

    Matches tapeout/docs/lidar_output_format.md & genesys2_chip/host/pc2x_v3_rx.py:
      - 48-byte header: <4sBBHIHHBBBBIIfffff
      - 6 blocks x 196 bytes (4-byte block hdr <HBB + 32 x <hhh points) = 1224 bytes.
      - Sentinel for no-return point: (-32768, -32768, -32768).
    """

    HDR = struct.Struct("<4sBBHIHHBBBBIIfffff")  # 48 bytes
    BLK_HDR = struct.Struct("<HBB")              # 4 bytes
    BLOCK_SIZE = 196
    DATAGRAM_SIZE = HDR.size + 6 * BLOCK_SIZE    # 1224 bytes
    POINT_STRUCT = struct.Struct("<hhh")

    def __init__(self, udp_port: int = 2370, max_points_per_sweep: int = 30000):
        self.udp_port = udp_port
        self.max_points_per_sweep = max_points_per_sweep
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._latest_sweep: Optional[LidarSweep] = None
        self._stats = LidarStreamStats()
        self.last_error: Optional[str] = None

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.is_running:
            return
        self._stop.clear()
        self.last_error = None
        self._thread = threading.Thread(target=self._run, name="PC2XReceiver", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None

    def get_latest(self) -> Tuple[Optional[LidarSweep], LidarStreamStats]:
        with self._lock:
            return self._latest_sweep, LidarStreamStats(**self._stats.__dict__)

    def _run(self) -> None:
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            except OSError:
                pass
            sock.bind(("0.0.0.0", self.udp_port))
            sock.settimeout(0.2)

            cur_sweep_id: Optional[int] = None
            cur_pts: List[Tuple[float, float, float]] = []
            cur_meta = None
            last_seq: Optional[int] = None

            win_t0 = time.monotonic()
            win_dgrams = 0
            win_sweeps = 0
            win_pts = 0
            total_lost = 0
            total_bad = 0

            hdr_unpack = self.HDR.unpack_from
            blk_unpack = self.BLK_HDR.unpack_from
            iter_pts = self.POINT_STRUCT.iter_unpack
            hdr_size = self.HDR.size
            blk_size = self.BLOCK_SIZE
            dgram_size = self.DATAGRAM_SIZE

            def publish_sweep(sid: int, meta, pts: List[Tuple[float, float, float]]) -> None:
                # Guard against tiny partial slivers at startup or mid-datagram boundaries
                if meta is None or len(pts) < 1000:
                    return
                snap = LidarSweep(
                    sweep_id=sid,
                    timestamp_us=meta[0],
                    flags=meta[1],
                    scale_m=meta[2],
                    state_valid=meta[3],
                    state_ts_us=meta[4],
                    pose_x=meta[5],
                    pose_y=meta[6],
                    pose_theta=meta[7],
                    pose_omega=meta[8],
                    pose_v=meta[9],
                    src_ip=meta[10],
                    points_xyz=pts,
                )
                with self._lock:
                    self._latest_sweep = snap

            flushed_mid_pkt = False

            while not self._stop.is_set():
                try:
                    data, addr = sock.recvfrom(2048)
                except socket.timeout:
                    now = time.monotonic()
                    if now - win_t0 >= 1.0:
                        with self._lock:
                            self._stats.dgrams_per_sec = 0.0
                            self._stats.sweeps_per_sec = 0.0
                            self._stats.pts_per_sec = 0.0
                        win_t0 = now
                        win_dgrams = win_sweeps = win_pts = 0
                    continue

                if len(data) != dgram_size:
                    total_bad += 1
                    continue

                (
                    magic,
                    ver,
                    flags,
                    _npts,
                    seq,
                    sweep_id,
                    _first_az,
                    _model,
                    step,
                    n_blocks,
                    per_block,
                    sweep_us,
                    state_us,
                    sx,
                    sy,
                    sth,
                    som,
                    sv,
                ) = hdr_unpack(data, 0)

                if magic != b"PC2X" or ver != 3 or step != 6 or n_blocks != 6 or per_block != 32:
                    total_bad += 1
                    continue

                if last_seq is not None:
                    diff = (seq - last_seq) & 0xFFFFFFFF
                    if 1 < diff < 100000:
                        total_lost += diff - 1
                last_seq = seq

                scale = 0.004 if (flags & 0x02) else 0.002
                state_ok = bool(flags & 0x01)
                meta = (sweep_us, flags, scale, state_ok, state_us, sx, sy, sth, som, sv, addr[0])

                # Only flush on header sweep_id change if we didn't already flush on a mid-packet
                # block sweep-start flag (prevents publishing tiny 1-5 block slivers -> flicker!).
                if cur_sweep_id is not None and sweep_id != cur_sweep_id:
                    if not flushed_mid_pkt and cur_pts:
                        win_sweeps += 1
                        publish_sweep(cur_sweep_id, cur_meta, cur_pts)
                        cur_pts = []
                    flushed_mid_pkt = False
                cur_sweep_id = sweep_id
                cur_meta = meta

                mv = memoryview(data)
                nz = 0
                for b in range(n_blocks):
                    off = hdr_size + b * blk_size
                    _az, bflags, _ = blk_unpack(data, off)
                    if bflags & 0x02:
                        continue
                    if (bflags & 0x01) and cur_pts:
                        win_sweeps += 1
                        publish_sweep(cur_sweep_id, cur_meta, cur_pts)
                        cur_pts = []
                        if b > 0:
                            flushed_mid_pkt = True
                    if len(cur_pts) < self.max_points_per_sweep:
                        blk_view = mv[off + 4 : off + 4 + per_block * 6]
                        for ix, iy, iz in iter_pts(blk_view):
                            if not (ix == -32768 and iy == -32768 and iz == -32768):
                                cur_pts.append((ix * scale, iy * scale, iz * scale))
                                nz += 1
                win_dgrams += 1
                win_pts += nz

                now = time.monotonic()
                dt = now - win_t0
                if dt >= 0.5:
                    with self._lock:
                        self._stats.dgrams_per_sec = win_dgrams / dt
                        self._stats.sweeps_per_sec = win_sweeps / dt
                        self._stats.pts_per_sec = win_pts / dt
                        self._stats.total_lost = total_lost
                        self._stats.total_bad = total_bad
                        self._stats.last_rx_monotonic = now
                    win_t0 = now
                    win_dgrams = win_sweeps = win_pts = 0
        except Exception as exc:
            self.last_error = str(exc)
        finally:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass


if __name__ == "__main__":
    esp, _ = autodetect_ports()
    port = esp or "/dev/ttyUSB0"
    with SensorFusionChip(port) as chip:
        cfg = chip.read_config()
        tel = chip.read_telemetry()
        print("Config:", cfg)
        print("Telemetry:", tel)
