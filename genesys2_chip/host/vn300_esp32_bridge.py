#!/usr/bin/env python3
"""
Live USB VN-300T-CR -> ESP32 -> Genesys 2 FPGA Bridge.

Temporarily enables VN-300 Binary Output 1 (exact 56-byte format expected by
tapeout/src/timestamper.sv: FA 01 68 10 + 50B payload + CRC16-CCITT) in RAM
(never touches NVM), reads real live frames from /dev/ttyUSB1, forwards them
verbatim to the ESP32 on /dev/ttyUSB0 ("v <112-hex>"), and restores the
VN-300 to its original $VNINS configuration cleanly on exit.

Usage:
    python3 vn300_esp32_bridge.py [--vn /dev/ttyUSB1] [--esp /dev/ttyUSB0] [--seconds 10]
"""
import argparse
import fcntl
import os
import select
import struct
import sys
import termios
import time


def vn_nmea(body: str) -> bytes:
    ck = 0
    for ch in body.encode("ascii"):
        ck ^= ch
    return f"${body}*{ck:02X}\r\n".encode("ascii")


def crc16_ccitt(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc >> 8) | ((crc & 0xFF) << 8)) & 0xFFFF
        crc ^= b
        crc ^= (crc & 0xFF) >> 4
        crc ^= (crc << 12) & 0xFFFF
        crc ^= ((crc & 0xFF) << 5) & 0xFFFF
    return crc


BAUD_MAP = {
    115200: termios.B115200,
    230400: termios.B230400,
    460800: termios.B460800,
    921600: termios.B921600,
}


def open_raw(port: str, baud: int) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    bconst = BAUD_MAP[baud]
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                                        # iflag: raw
    attrs[1] = 0                                        # oflag: raw
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | bconst  # cflag (keeps baud!)
    attrs[3] = 0                                        # lflag: raw
    attrs[4] = bconst                                   # ispeed
    attrs[5] = bconst                                   # ospeed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    # Release DTR & RTS so ESP32 does not reset
    fcntl.ioctl(fd, 0x5417, struct.pack("I", 0x006))
    return fd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vn", default="/dev/ttyUSB1")
    ap.add_argument("--esp", default="/dev/ttyUSB0")
    ap.add_argument("--vn-baud", type=int, default=230400)
    ap.add_argument("--esp-baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=8.0, help="Run duration (0 = until Ctrl-C)")
    args = ap.parse_args()

    vn_fd = open_raw(args.vn, args.vn_baud)
    esp_fd = open_raw(args.esp, args.esp_baud)
    time.sleep(0.1)

    # Drain both ports cleanly (non-blocking tcflush!)
    for fd in (vn_fd, esp_fd):
        termios.tcflush(fd, termios.TCIOFLUSH)

    try:
        # Enable VN-300 Binary Output 1 in RAM:
        # asyncMode=1 (Serial 1), rateDivisor=20 (40 Hz), outputGroup=01, commonField=1068
        os.write(vn_fd, vn_nmea("VNWRG,06,0"))          # pause ASCII output cleanly
        time.sleep(0.05)
        os.write(vn_fd, vn_nmea("VNWRG,75,1,20,01,1068"))
        time.sleep(0.05)

        rx_buf = bytearray()
        frames_ok = 0
        t_start = time.time()
        last_dump = t_start

        while True:
            now = time.time()
            if args.seconds > 0 and (now - t_start) >= args.seconds:
                break

            r, _, _ = select.select([vn_fd, esp_fd], [], [], 0.1)
            if vn_fd in r:
                chunk = os.read(vn_fd, 4096)
                if chunk:
                    rx_buf.extend(chunk)
                    while len(rx_buf) >= 56:
                        idx = rx_buf.find(b"\xFA\x01\x68\x10")
                        if idx < 0:
                            del rx_buf[:-3]
                            break
                        if idx > 0:
                            del rx_buf[:idx]
                        if len(rx_buf) < 56:
                            break
                        pkt = bytes(rx_buf[:56])
                        del rx_buf[:56]
                        if crc16_ccitt(pkt[1:]) == 0:
                            frames_ok += 1
                            os.write(esp_fd, ("v " + pkt.hex() + "\n").encode("ascii"))

            if esp_fd in r:
                txt = os.read(esp_fd, 4096).decode("utf-8", errors="replace")
                if txt.strip():
                    print(txt.rstrip(), flush=True)

            if now - last_dump >= 1.5 and frames_ok > 0:
                last_dump = now
                print(f"\n[Bridge] Forwarded {frames_ok} live VN-300 frames -> querying FPGA via ESP32 SPI:", flush=True)
                os.write(esp_fd, b"d\n")

        # Final FPGA SPI readout
        time.sleep(0.2)
        os.write(esp_fd, b"d\n")
        t_end = time.time()
        while time.time() - t_end < 0.4:
            if select.select([esp_fd], [], [], 0.1)[0]:
                txt = os.read(esp_fd, 4096).decode("utf-8", errors="replace")
                if txt.strip():
                    print(txt.rstrip(), flush=True)
        print(f"\n[Bridge] Done. Total real VN-300 frames forwarded: {frames_ok}")
    finally:
        # Restore VN-300 RAM back to original $VNINS (never touches NVM)
        os.write(vn_fd, vn_nmea("VNWRG,75,0,0,00"))
        time.sleep(0.05)
        os.write(vn_fd, vn_nmea("VNWRG,06,22"))
        os.write(esp_fd, b"v 0\n")
        time.sleep(0.05)
        os.close(vn_fd)
        os.close(esp_fd)


if __name__ == "__main__":
    main()
