#!/usr/bin/env python3
"""
UMARV Sensor-Fusion Chip Studio (PySide6 GUI + Live 2D Lidar Map + Register Write Console).

Clean 2-section workspace:
  - LEFT:  Live Observed Data (flicker-free 2D top-down PC2X v3 Lidar Map +
           live VN-300 / AMT22 / UKF state telemetry, compass, sparklines, counters).
  - RIGHT: Dedicated Register Writes & Chip Tuning Console (UKF tunable constants
           via 0x6C/0x70-73, Control & Timing 0x02-0B, Network 0x0C-23, Host Pose
           0x24-3C, and Direct Register Read/Write + 0x00-7F Hex Dump & Write Log).

Usage:
    python3 sensorfusion_gui.py [--port /dev/ttyUSB0] [--vn /dev/ttyUSB1] [--udp-port 2370]
"""
from __future__ import annotations

import argparse
from collections import deque
import math
import random
import struct
import sys
import time
from typing import Deque, Dict, List, Optional, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
from PySide6.QtGui import (
    QColor,
    QFont,
    QLinearGradient,
    QPainter,
    QPainterPath,
    QPen,
    QPolygonF,
)
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from sensorfusion_sdk import (
    LIDAR_MODELS,
    SPI_REGISTER_MAP,
    UKF_CONST_DEFS,
    UKF_NAMED_WORDS,
    ChipConfig,
    ChipTelemetry,
    HostState,
    LidarStreamStats,
    LidarSweep,
    PC2XReceiver,
    SensorFusionChip,
    UkfState,
    VN300GnssStatus,
    VN300USBBridge,
    autodetect_ports,
    list_serial_ports,
)


DARK_QSS = """
QMainWindow, QWidget {
    background-color: #0d1219;
    color: #e2e8f0;
    font-family: "Inter", "Segoe UI", "Ubuntu", sans-serif;
    font-size: 13px;
}
QGroupBox {
    background-color: #141c26;
    border: 1px solid #233042;
    border-radius: 8px;
    margin-top: 12px;
    padding: 12px 10px 10px 10px;
    font-weight: 600;
    color: #94a3b8;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 6px;
    color: #cbd5e1;
}
QPushButton {
    background-color: #1e293b;
    border: 1px solid #334155;
    border-radius: 6px;
    padding: 5px 10px;
    color: #f1f5f9;
    font-weight: 600;
}
QPushButton:hover {
    background-color: #273549;
    border-color: #38bdf8;
}
QPushButton:pressed {
    background-color: #0f172a;
}
QPushButton:disabled {
    color: #64748b;
    background-color: #141b24;
    border-color: #1e293b;
}
QPushButton#primaryBtn {
    background-color: #0284c7;
    border-color: #0ea5e9;
    color: #ffffff;
}
QPushButton#primaryBtn:hover {
    background-color: #0369a1;
}
QPushButton#dangerBtn {
    background-color: #991b1b;
    border-color: #ef4444;
    color: #ffffff;
}
QPushButton#successBtn {
    background-color: #15803d;
    border-color: #22c55e;
    color: #ffffff;
}
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit, QPlainTextEdit {
    background-color: #0a0f15;
    border: 1px solid #2b384c;
    border-radius: 5px;
    padding: 4px 8px;
    color: #f8fafc;
}
QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus {
    border-color: #38bdf8;
}
QTabWidget::pane {
    border: 1px solid #233042;
    border-radius: 8px;
    background-color: #141c26;
    padding: 6px;
}
QTabBar::tab {
    background-color: #0f151f;
    border: 1px solid #233042;
    border-bottom: none;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    padding: 6px 11px;
    margin-right: 3px;
    color: #94a3b8;
    font-weight: 600;
}
QTabBar::tab:selected {
    background-color: #141c26;
    color: #38bdf8;
    border-color: #38bdf8;
}
QCheckBox {
    spacing: 7px;
    color: #e2e8f0;
}
QCheckBox::indicator {
    width: 15px;
    height: 15px;
    border-radius: 4px;
    border: 1px solid #475569;
    background-color: #0a0f15;
}
QCheckBox::indicator:checked {
    background-color: #0ea5e9;
    border-color: #38bdf8;
}
QFrame#metricTile {
    background-color: #0f1620;
    border: 1px solid #233044;
    border-radius: 6px;
}
QFrame#sectionHeader {
    background-color: #111b29;
    border: 1px solid #1e3a5f;
    border-radius: 7px;
}
QTableWidget {
    background-color: #0a0f15;
    alternate-background-color: #0f1620;
    gridline-color: #1e293b;
    border: 1px solid #233042;
    border-radius: 6px;
    color: #e2e8f0;
}
QHeaderView::section {
    background-color: #15202e;
    color: #94a3b8;
    padding: 5px 6px;
    border: none;
    border-bottom: 1px solid #233042;
    font-weight: 700;
    font-size: 11px;
}
QStatusBar {
    background-color: #0a0f15;
    color: #94a3b8;
    border-top: 1px solid #1e293b;
}
"""


Z_PALETTE = [
    QColor("#2563eb"),
    QColor("#0284c7"),
    QColor("#06b6d4"),
    QColor("#14b8a6"),
    QColor("#10b981"),
    QColor("#22c55e"),
    QColor("#84cc16"),
    QColor("#eab308"),
    QColor("#f59e0b"),
    QColor("#f97316"),
    QColor("#ef4444"),
    QColor("#f43f5e"),
]


def fit_ground_plane(
    points_xyz: List[Tuple[float, float, float]],
    max_tilt_deg: float = 20.0,
    inlier_thresh: float = 0.08,
    iters: int = 40,
) -> Optional[Tuple[float, float, float, float, int, int]]:
    """
    Fast pure-Python ground plane estimator for 3D lidar sweeps (local robot frame).

    1. Extracts lowest-Z seed point per 0.5m x 0.5m XY cell (ignores self/far outliers).
    2. Runs deterministic RANSAC enforcing near-horizontal normal (tilt <= max_tilt_deg).
    3. Refines winning plane via 3x3 least-squares fit (z = alpha*x + beta*y + gamma).

    Returns (a, b, c, d, n_inliers, n_seeds) with unit normal (a, b, c), c > 0,
    such that signed height above ground is h = a*x + b*y + c*z + d; or None if no plane.
    """
    n = len(points_xyz)
    if n < 12:
        return None

    # Step 1: Lowest-Z point per 0.5m x 0.5m grid cell
    stride = 2 if n > 4000 else 1
    cells: Dict[Tuple[int, int], Tuple[float, float, float]] = {}
    for i in range(0, n, stride):
        x, y, z = points_xyz[i]
        if abs(x) > 25.0 or abs(y) > 25.0:
            continue
        if x * x + y * y < 0.16:  # < 0.4m from lidar origin
            continue
        key = (int(math.floor(x * 2.0)), int(math.floor(y * 2.0)))
        prev = cells.get(key)
        if prev is None or z < prev[2]:
            cells[key] = (x, y, z)

    seeds = list(cells.values())
    if len(seeds) < 6:
        return None

    # Keep the lower 75% of cell minimums so wall-only/ceiling cells don't dilute RANSAC
    if len(seeds) >= 16:
        seeds.sort(key=lambda p: p[2])
        seeds = seeds[: max(12, int(len(seeds) * 0.75))]
    if len(seeds) > 200:
        step = len(seeds) / 200.0
        seeds = [seeds[int(i * step)] for i in range(200)]

    ns = len(seeds)
    min_nz = math.cos(math.radians(max_tilt_deg))
    rng = random.Random(1337)

    # Step 2: Deterministic RANSAC
    best_count = 0
    best_plane: Optional[Tuple[float, float, float, float]] = None
    for _ in range(iters):
        i0 = rng.randrange(ns)
        i1 = rng.randrange(ns)
        i2 = rng.randrange(ns)
        if i0 == i1 or i1 == i2 or i0 == i2:
            continue
        x0, y0, z0 = seeds[i0]
        x1, y1, z1 = seeds[i1]
        x2, y2, z2 = seeds[i2]
        ux, uy, uz = x1 - x0, y1 - y0, z1 - z0
        vx, vy, vz = x2 - x0, y2 - y0, z2 - z0
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        nlen = math.sqrt(nx * nx + ny * ny + nz * nz)
        if nlen < 1e-4:
            continue
        if nz < 0.0:
            nx, ny, nz = -nx, -ny, -nz
        inv_n = 1.0 / nlen
        nx *= inv_n
        ny *= inv_n
        nz *= inv_n
        if nz < min_nz:
            continue
        d = -(nx * x0 + ny * y0 + nz * z0)

        cnt = 0
        for sx, sy, sz in seeds:
            if abs(nx * sx + ny * sy + nz * sz + d) <= inlier_thresh:
                cnt += 1
        if cnt > best_count:
            best_count = cnt
            best_plane = (nx, ny, nz, d)

    if best_plane is None or best_count < 6:
        return None

    # Step 3: Refine via 3x3 OLS z = alpha*x + beta*y + gamma on RANSAC inliers
    bx, by, bz, bd = best_plane
    sxx = sxy = syy = sx = sy = sxz = syz = sz = 0.0
    m = 0
    for px, py, pz in seeds:
        if abs(bx * px + by * py + bz * pz + bd) <= inlier_thresh:
            sxx += px * px
            sxy += px * py
            syy += py * py
            sx += px
            sy += py
            sxz += px * pz
            syz += py * pz
            sz += pz
            m += 1

    if m >= 6:
        # Solve 3x3 symmetric normal equations via Cramer's rule
        a11, a12, a13 = sxx, sxy, sx
        a21, a22, a23 = sxy, syy, sy
        a31, a32, a33 = sx, sy, float(m)
        det = (
            a11 * (a22 * a33 - a23 * a32)
            - a12 * (a21 * a33 - a23 * a31)
            + a13 * (a21 * a32 - a22 * a31)
        )
        if abs(det) > 1e-9:
            inv_det = 1.0 / det
            alpha = (
                sxz * (a22 * a33 - a23 * a32)
                - a12 * (syz * a33 - a23 * sz)
                + a13 * (syz * a32 - a22 * sz)
            ) * inv_det
            beta = (
                a11 * (syz * a33 - a23 * sz)
                - sxz * (a21 * a33 - a23 * a31)
                + a13 * (a21 * sz - syz * a31)
            ) * inv_det
            gamma = (
                a11 * (a22 * sz - syz * a32)
                - a12 * (a21 * sz - syz * a31)
                + sxz * (a21 * a32 - a22 * a31)
            ) * inv_det
            inv_norm = 1.0 / math.sqrt(alpha * alpha + beta * beta + 1.0)
            rc = inv_norm
            if rc >= min_nz:
                ra = -alpha * inv_norm
                rb = -beta * inv_norm
                rd = -gamma * inv_norm
                return (ra, rb, rc, rd, m, ns)

    return (bx, by, bz, bd, best_count, ns)


class LidarMapWidget(QWidget):
    """
    Real-time interactive 2D Global-Frame Lidar Map with 1-second fading scan trail.

    Transforms each incoming PC2X v3 sweep from robot frame (at sweep start) into
    global (East=+X, North=+Y) coordinates using the chip's fused pose
    (pose_x, pose_y, pose_theta) — with optional fallback to live VN-300 heading
    when state_valid==0 — and renders the last `fade_sec` (default 1.0 s) of scans
    with older scans smoothly fading out in intensity.
    """

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._stats = LidarStreamStats()
        # Rolling history of transformed sweeps:
        #   [t_rx, sweep_id, px, py, theta, pts_g4, sweep_meta, filter_key, polys, gnd_poly, n_obs, n_gnd]
        #   where pts_g4 is [(gx, gy, z, h_above_ground_or_nan), ...]
        self._history: Deque[list] = deque(maxlen=64)
        self._trail: Deque[Tuple[float, float, float]] = deque(maxlen=800)  # (t_rx, px, py)
        self._last_ingested_sid: Optional[int] = None

        # Smoothed ground plane in local sensor frame: (a, b, c, d), unit normal with c > 0
        self._ground_plane: Optional[Tuple[float, float, float, float]] = None
        self._ground_fit_info: Tuple[int, int] = (0, 0)  # (inliers, seeds)

        # Live real-time pose from SPI (UKF state + VN-300 heading + AMT22 dead-wheel odometry)
        self._vn_yaw_deg: Optional[float] = None
        self._live_ukf: Tuple[float, float, float, float, float, bool] = (0.0, 0.0, 0.0, 0.0, 0.0, False)
        self._dr_x = 0.0
        self._dr_y = 0.0
        self._prev_dw_counts: Optional[int] = None
        self._has_live_pose = False

        self.frame_mode = "global_auto"   # "global_auto" | "global_ukf" | "global_vn" | "robot"
        self.ground_mode = "hide"         # "hide" | "dim" | "off"
        self.ground_margin_m = 0.12       # points with h_above_ground <= margin are ground
        self.follow_robot = False         # False = fixed world origin so arrow visibly moves across map
        self.fade_sec = 1.0
        self.range_m = 6.0
        self.pan_x_m = 0.0
        self.pan_y_m = 0.0
        self.point_size = 2
        self.z_min = -2.0
        self.z_max = 2.5

        self._dragging = False
        self._drag_start = QPointF()
        self._drag_pan_start = (0.0, 0.0)
        self.setMinimumSize(320, 220)
        self.setMouseTracking(True)

    def set_vn_yaw_deg(self, yaw_deg: float) -> None:
        if math.isfinite(yaw_deg) and abs(yaw_deg) <= 1e6:
            self._vn_yaw_deg = yaw_deg
            self._has_live_pose = True
            if not self._history:
                self.update()

    def reset_dead_reckoning(self) -> None:
        self._dr_x = 0.0
        self._dr_y = 0.0
        self._prev_dw_counts = None
        self._trail.clear()
        self.update()

    def update_live_pose(
        self,
        vn_yaw_deg: float,
        dw_abs_counts: int,
        cnt2rad: float,
        rnom: float,
        ukf: Optional[UkfState] = None,
        state_from_filter: bool = False,
    ) -> None:
        """Update real-time robot arrow pose & trail from 10 Hz SPI telemetry + UKF state."""
        now = time.monotonic()
        if math.isfinite(vn_yaw_deg) and abs(vn_yaw_deg) <= 1e6:
            self._vn_yaw_deg = vn_yaw_deg
            self._has_live_pose = True

        # Update live UKF state if valid/finite
        if ukf is not None:
            ux = ukf.x if (math.isfinite(ukf.x) and abs(ukf.x) < 1e6) else 0.0
            uy = ukf.y if (math.isfinite(ukf.y) and abs(ukf.y) < 1e6) else 0.0
            uth = ukf.theta if (math.isfinite(ukf.theta) and abs(ukf.theta) < 1e4) else 0.0
            uom = ukf.omega if (math.isfinite(ukf.omega) and abs(ukf.omega) < 1e4) else 0.0
            uv = ukf.v if (math.isfinite(ukf.v) and abs(ukf.v) < 1e4) else 0.0
            u_active = bool(state_from_filter or ukf.ts_us != 0 or abs(ux) > 1e-4 or abs(uy) > 1e-4 or abs(uth) > 1e-4)
            self._live_ukf = (ux, uy, uth, uom, uv, u_active)
            if u_active:
                self._has_live_pose = True

        # Integrate AMT22 dead-wheel displacement along live VN-300 heading
        if self._prev_dw_counts is None:
            self._prev_dw_counts = dw_abs_counts
        else:
            d_cnt = dw_abs_counts - self._prev_dw_counts
            self._prev_dw_counts = dw_abs_counts
            # Ignore huge single-poll discontinuities (e.g. ESP32 reset / zeroing)
            if 0 < abs(d_cnt) < 500_000:
                ds = SensorFusionChip.deadwheel_distance_m(d_cnt, cnt2rad, rnom)
                vn_th = (
                    (math.pi * 0.5 - math.radians(self._vn_yaw_deg))
                    if self._vn_yaw_deg is not None
                    else 0.0
                )
                self._dr_x += ds * math.cos(vn_th)
                self._dr_y += ds * math.sin(vn_th)
                self._has_live_pose = True

        # Record global trajectory trail whenever robot moves >= 1.5 cm
        if self.frame_mode != "robot":
            px, py, _th, _om, _v, _lbl, _is_ukf = self._resolve_live_pose()
            if not self._trail:
                self._trail.append((now, px, py))
            else:
                _, lx, ly = self._trail[-1]
                if (px - lx) ** 2 + (py - ly) ** 2 >= 0.015 ** 2:
                    self._trail.append((now, px, py))
            trail_cutoff = now - 120.0
            while len(self._trail) > 2 and self._trail[0][0] < trail_cutoff:
                self._trail.popleft()

        self.update()

    def clear_history(self) -> None:
        self._history.clear()
        self._trail.clear()
        self._last_ingested_sid = None
        self._ground_plane = None
        self._ground_fit_info = (0, 0)
        self.update()

    def _resolve_live_pose(self) -> Tuple[float, float, float, float, float, str, bool]:
        """Return (px_east, py_north, theta_ccw_east, omega, v, source_label, is_ukf) in real time."""
        if self.frame_mode == "robot":
            return 0.0, 0.0, 0.0, 0.0, 0.0, "ROBOT", False

        ux, uy, uth, uom, uv, u_active = self._live_ukf
        vn_th = (
            (math.pi * 0.5 - math.radians(self._vn_yaw_deg))
            if self._vn_yaw_deg is not None
            else 0.0
        )

        if self.frame_mode == "global_ukf":
            return ux, uy, uth, uom, uv, "GLOBAL·UKF", u_active
        if self.frame_mode == "global_vn":
            return self._dr_x, self._dr_y, vn_th, 0.0, 0.0, "GLOBAL·VN+WHEEL", False

        # "global_auto": prefer live chip UKF pose when active & non-trivial, else VN-300 + Wheel DR
        if u_active and (abs(ux) > 1e-4 or abs(uy) > 1e-4 or abs(uth) > 1e-4):
            return ux, uy, uth, uom, uv, "GLOBAL·UKF", True
        if self._vn_yaw_deg is not None or abs(self._dr_x) > 1e-5 or abs(self._dr_y) > 1e-5:
            return self._dr_x, self._dr_y, vn_th, 0.0, 0.0, "GLOBAL·VN+WHEEL", False
        return ux, uy, uth, uom, uv, "GLOBAL·POSE", False

    def _resolve_pose(self, sw: LidarSweep) -> Tuple[float, float, float, str]:
        """Return (px_east, py_north, theta_ccw_east, source_label) for an incoming PC2X sweep."""
        if self.frame_mode == "robot":
            return 0.0, 0.0, 0.0, "ROBOT"

        sw_x = sw.pose_x if math.isfinite(sw.pose_x) else 0.0
        sw_y = sw.pose_y if math.isfinite(sw.pose_y) else 0.0
        sw_th = sw.pose_theta if math.isfinite(sw.pose_theta) else 0.0
        sw_nonzero = sw.state_valid and (abs(sw_x) > 1e-4 or abs(sw_y) > 1e-4 or abs(sw_th) > 1e-4)

        lx, ly, lth, _lom, _lv, llbl, l_ukf = self._resolve_live_pose()

        if self.frame_mode == "global_ukf":
            if sw.state_valid:
                return sw_x, sw_y, sw_th, "GLOBAL·UKF"
            return lx, ly, lth, "GLOBAL·UKF"
        if self.frame_mode == "global_vn":
            return lx, ly, lth, "GLOBAL·VN+WHEEL"

        # "global_auto": prefer PC2X embedded UKF pose when valid & non-trivial, else live SPI pose (UKF or VN+Wheel)
        if sw_nonzero:
            return sw_x, sw_y, sw_th, "GLOBAL·UKF"
        return lx, ly, lth, llbl

    @staticmethod
    def _build_z_polys(
        pts_g4: List[Tuple[float, float, float, float]],
        z_lo: float,
        z_hi: float,
        ground_mode: str,
        ground_margin: float,
    ) -> Tuple[List[Optional[QPolygonF]], Optional[QPolygonF], int, int]:
        nb = len(Z_PALETTE)
        inv_span = (nb - 1) / max(0.05, z_hi - z_lo)
        buckets: List[List[QPointF]] = [[] for _ in range(nb)]
        gnd_pts: List[QPointF] = []
        n_obs = 0
        n_gnd = 0
        filter_on = (ground_mode != "off")
        keep_gnd = (ground_mode == "dim")

        for x, y, z, h_gnd in pts_g4:
            if z < z_lo or z > z_hi:
                continue
            if filter_on and h_gnd <= ground_margin:
                n_gnd += 1
                if keep_gnd:
                    gnd_pts.append(QPointF(x, y))
                continue
            bi = int((z - z_lo) * inv_span)
            bi = 0 if bi < 0 else (nb - 1 if bi >= nb else bi)
            buckets[bi].append(QPointF(x, y))
            n_obs += 1

        polys: List[Optional[QPolygonF]] = [QPolygonF(b) if b else None for b in buckets]
        gnd_poly: Optional[QPolygonF] = QPolygonF(gnd_pts) if gnd_pts else None
        return polys, gnd_poly, n_obs, n_gnd

    def _update_ground_plane(self, raw_pts: List[Tuple[float, float, float]]) -> Optional[Tuple[float, float, float, float]]:
        res = fit_ground_plane(raw_pts)
        if res is not None:
            a, b, c, d, n_in, n_sd = res
            self._ground_fit_info = (n_in, n_sd)
            if self._ground_plane is None:
                self._ground_plane = (a, b, c, d)
            else:
                oa, ob, oc, od = self._ground_plane
                al = 0.25
                na = (1.0 - al) * oa + al * a
                nb = (1.0 - al) * ob + al * b
                nc = (1.0 - al) * oc + al * c
                nd = (1.0 - al) * od + al * d
                nlen = math.sqrt(na * na + nb * nb + nc * nc)
                if nlen > 1e-6:
                    inv = 1.0 / nlen
                    self._ground_plane = (na * inv, nb * inv, nc * inv, nd * inv)
        return self._ground_plane

    def set_frame(self, sweep: Optional[LidarSweep], stats: LidarStreamStats) -> None:
        now = time.monotonic()
        self._stats = stats

        if sweep is not None and sweep.sweep_id != self._last_ingested_sid:
            self._last_ingested_sid = sweep.sweep_id
            px, py, th, _ = self._resolve_pose(sweep)
            c, s = math.cos(th), math.sin(th)
            raw_pts = sweep.points_xyz

            # Estimate/smooth ground plane in local sensor frame (valid regardless of yaw)
            plane = self._update_ground_plane(raw_pts)
            no_xform = (abs(th) < 1e-7 and abs(px) < 1e-7 and abs(py) < 1e-7)

            if plane is not None:
                ga, gb, gc, gd = plane
                if no_xform:
                    pts_g4 = [(x, y, z, ga * x + gb * y + gc * z + gd) for x, y, z in raw_pts]
                else:
                    pts_g4 = [
                        (px + x * c - y * s, py + x * s + y * c, z, ga * x + gb * y + gc * z + gd)
                        for x, y, z in raw_pts
                    ]
            else:
                # No ground plane fitted yet: mark height as +inf so nothing is wrongly hidden
                inf = 1e9
                if no_xform:
                    pts_g4 = [(x, y, z, inf) for x, y, z in raw_pts]
                else:
                    pts_g4 = [
                        (px + x * c - y * s, py + x * s + y * c, z, inf)
                        for x, y, z in raw_pts
                    ]

            fkey = (self.z_min, self.z_max, self.ground_mode, self.ground_margin_m)
            polys, gnd_poly, n_obs, n_gnd = self._build_z_polys(
                pts_g4, fkey[0], fkey[1], fkey[2], fkey[3]
            )
            self._history.append(
                [now, sweep.sweep_id, px, py, th, pts_g4, sweep, fkey, polys, gnd_poly, n_obs, n_gnd]
            )
            if self.frame_mode != "robot":
                if not self._trail or ((px - self._trail[-1][1]) ** 2 + (py - self._trail[-1][2]) ** 2) >= 0.015 ** 2:
                    self._trail.append((now, px, py))

        # Prune sweeps older than fade_sec (and trail older than 120s)
        cutoff = now - self.fade_sec
        while self._history and self._history[0][0] < cutoff:
            self._history.popleft()
        trail_cutoff = now - max(120.0, self.fade_sec)
        while len(self._trail) > 2 and self._trail[0][0] < trail_cutoff:
            self._trail.popleft()

        self.update()

    def reset_view(self) -> None:
        self.range_m = 6.0
        self.pan_x_m = 0.0
        self.pan_y_m = 0.0
        self.update()

    def wheelEvent(self, event) -> None:
        factor = 0.85 if event.angleDelta().y() > 0 else 1.18
        self.range_m = max(1.0, min(50.0, self.range_m * factor))
        self.update()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self._dragging = True
            self._drag_start = event.position()
            self._drag_pan_start = (self.pan_x_m, self.pan_y_m)

    def mouseMoveEvent(self, event) -> None:
        if self._dragging:
            px_per_m = min(self.width(), self.height()) / (2.0 * self.range_m)
            dx = (event.position().x() - self._drag_start.x()) / max(1e-6, px_per_m)
            dy = (event.position().y() - self._drag_start.y()) / max(1e-6, px_per_m)
            self.pan_x_m = self._drag_pan_start[0] + dx
            self.pan_y_m = self._drag_pan_start[1] - dy
            self.update()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self._dragging = False

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        try:
            now = time.monotonic()
            p.setRenderHint(QPainter.Antialiasing, False)
            w, h = self.width(), self.height()
            p.fillRect(0, 0, w, h, QColor("#070b11"))

            px_per_m = min(w, h) / (2.0 * self.range_m)

            # Latest real-time robot pose in global frame (live SPI UKF / VN+Wheel DR, or PC2X header)
            cur_px, cur_py, cur_th, cur_om, cur_v, frame_lbl, pose_is_ukf = self._resolve_live_pose()
            cur_sw: Optional[LidarSweep] = None
            if self._history:
                h_last = self._history[-1]
                cur_sw = h_last[6]
                # If live PC2X header carries valid UKF state and we aren't overriding with VN+Wheel DR,
                # sync with it when live SPI UKF isn't active
                if (
                    self.frame_mode in ("global_auto", "global_ukf")
                    and cur_sw.state_valid
                    and not pose_is_ukf
                    and (abs(cur_sw.pose_x) > 1e-4 or abs(cur_sw.pose_y) > 1e-4 or abs(cur_sw.pose_theta) > 1e-4)
                ):
                    cur_px, cur_py, cur_th = h_last[2], h_last[3], h_last[4]
                    cur_om, cur_v = cur_sw.pose_omega, cur_sw.pose_v
                    frame_lbl = "GLOBAL·UKF"
                    pose_is_ukf = True

            # Camera center (follow robot in global frame unless unchecked)
            cam_x = (cur_px if (self.follow_robot and self.frame_mode != "robot") else 0.0) - self.pan_x_m
            cam_y = (cur_py if (self.follow_robot and self.frame_mode != "robot") else 0.0) - self.pan_y_m
            cx = w * 0.5 - cam_x * px_per_m   # screen X of world origin (0, 0)
            cy = h * 0.5 + cam_y * px_per_m   # screen Y of world origin (0, 0)
            rx = cx + cur_px * px_per_m       # screen X of robot
            ry = cy - cur_py * px_per_m       # screen Y of robot

            # Global grid lines + range rings centered on robot
            p.setRenderHint(QPainter.Antialiasing, True)
            ring_step = 1.0 if self.range_m <= 8.0 else (2.0 if self.range_m <= 16.0 else 5.0)

            # Subtle global 1m/2m/5m Cartesian grid
            grid_pen = QPen(QColor("#111a26"), 1)
            p.setPen(grid_pen)
            x_min_w = cam_x - (w * 0.5) / px_per_m
            x_max_w = cam_x + (w * 0.5) / px_per_m
            y_min_w = cam_y - (h * 0.5) / px_per_m
            y_max_w = cam_y + (h * 0.5) / px_per_m
            gx0 = math.floor(x_min_w / ring_step) * ring_step
            gy0 = math.floor(y_min_w / ring_step) * ring_step
            gx = gx0
            while gx <= x_max_w:
                sx = cx + gx * px_per_m
                p.drawLine(QPointF(sx, 0), QPointF(sx, h))
                gx += ring_step
            gy = gy0
            while gy <= y_max_w:
                sy = cy - gy * px_per_m
                p.drawLine(QPointF(0, sy), QPointF(w, sy))
                gy += ring_step

            # World origin axes (East / North) + clear (0,0) origin marker
            p.setPen(QPen(QColor("#1e2d40"), 1, Qt.DashLine))
            if 0 <= cy <= h:
                p.drawLine(QPointF(0, cy), QPointF(w, cy))
            if 0 <= cx <= w:
                p.drawLine(QPointF(cx, 0), QPointF(cx, h))
            if self.frame_mode != "robot" and -20 <= cx <= w + 20 and -20 <= cy <= h + 20:
                p.setPen(QPen(QColor("#475569"), 1.5))
                p.setBrush(Qt.NoBrush)
                p.drawEllipse(QPointF(cx, cy), 5.0, 5.0)
                p.drawLine(QPointF(cx - 8, cy), QPointF(cx + 8, cy))
                p.drawLine(QPointF(cx, cy - 8), QPointF(cx, cy + 8))
                p.setFont(QFont("JetBrains Mono, Monospace", 7))
                p.setPen(QColor("#64748b"))
                p.drawText(QPointF(cx + 7, cy + 12), "Origin (0,0)")

            # Concentric range rings around robot
            max_ring = int(math.ceil(self.range_m * 1.4 / ring_step))
            ring_pen = QPen(QColor("#1a2738"), 1)
            p.setFont(QFont("JetBrains Mono, Monospace", 8))
            for k in range(1, max_ring + 1):
                rm = k * ring_step
                rr = rm * px_per_m
                if rr > max(w, h) * 1.4:
                    break
                p.setPen(ring_pen)
                p.drawEllipse(QPointF(rx, ry), rr, rr)
                p.setPen(QColor("#475569"))
                p.drawText(QPointF(rx + 4, ry - rr + 11), f"{rm:g}m")

            # Global cardinal labels
            p.setPen(QColor("#64748b"))
            p.setFont(QFont("JetBrains Mono, Monospace", 8, QFont.Bold))
            if self.frame_mode == "robot":
                p.drawText(QPointF(w - 50, ry - 6), "+X Fwd")
                p.drawText(QPointF(rx + 6, 16), "+Y Left")
            else:
                p.drawText(QPointF(w - 56, h * 0.5 - 6), "+X East")
                p.drawText(QPointF(w * 0.5 + 6, 16), "+Y North")

            # Draw last `fade_sec` of sweeps: OLDEST FIRST -> NEWEST LAST, fading intensity!
            # Uses pre-bucketed world-coordinate QPolygonF + QPainter world transform so
            # 10+ full-density sweeps (144k+ points) render in pure C++ with zero Python point loop.
            p.setRenderHint(QPainter.Antialiasing, False)
            drawn_pts = 0
            gnd_pts_total = 0
            n_sweeps = len(self._history)
            fkey = (self.z_min, self.z_max, self.ground_mode, self.ground_margin_m)
            fade_window = max(0.1, self.fade_sec)

            p.save()
            p.translate(cx, cy)
            p.scale(px_per_m, -px_per_m)

            for s_idx, entry in enumerate(self._history):
                t_rx = entry[0]
                age = max(0.0, now - t_rx)
                if age > fade_window:
                    continue
                if entry[7] != fkey:
                    entry[8], entry[9], entry[10], entry[11] = self._build_z_polys(
                        entry[5], fkey[0], fkey[1], fkey[2], fkey[3]
                    )
                    entry[7] = fkey
                polys = entry[8]
                gnd_poly = entry[9]
                drawn_pts += entry[10]
                gnd_pts_total += entry[11]

                # Smooth perceptual fade: newest = 1.0, older decays to ~0.06 at age == fade_sec
                fresh = max(0.0, 1.0 - (age / fade_window))
                intensity = fresh ** 1.6
                alpha = max(16, min(255, int(255.0 * intensity)))
                is_newest = (s_idx == n_sweeps - 1)
                pt_w = self.point_size if is_newest else max(1, self.point_size - 1)
                dim = 0.35 + 0.65 * intensity

                # Optional dimmed ground plane points underneath obstacles ("dim" mode)
                if gnd_poly is not None:
                    g_alpha = max(8, int(alpha * 0.28))
                    g_pen = QPen(QColor(71, 85, 105, g_alpha), 1)
                    g_pen.setCosmetic(True)
                    p.setPen(g_pen)
                    p.drawPoints(gnd_poly)

                for bi, poly in enumerate(polys):
                    if poly is None:
                        continue
                    c = QColor(Z_PALETTE[bi])
                    c.setRgb(int(c.red() * dim), int(c.green() * dim), int(c.blue() * dim), alpha)
                    pen = QPen(c, pt_w)
                    pen.setCosmetic(True)
                    p.setPen(pen)
                    p.drawPoints(poly)

            p.restore()

            # Draw robot trajectory trail (connecting straight to live arrow position)
            p.setRenderHint(QPainter.Antialiasing, True)
            if self.frame_mode != "robot" and len(self._trail) >= 1:
                trail_path = QPainterPath()
                for idx, (_t, tx, ty) in enumerate(self._trail):
                    pt = QPointF(cx + tx * px_per_m, cy - ty * px_per_m)
                    if idx == 0:
                        trail_path.moveTo(pt)
                    else:
                        trail_path.lineTo(pt)
                trail_path.lineTo(QPointF(rx, ry))
                p.setPen(QPen(QColor(56, 189, 248, 175), 2.0))
                p.drawPath(trail_path)

            # Robot heading arrow at real-time (rx, ry) rotated by global heading cur_th (CCW from East)
            c_r, s_r = math.cos(cur_th), math.sin(cur_th)
            def r_pt(lx: float, ly: float) -> QPointF:
                return QPointF(rx + lx * c_r - ly * s_r, ry - (lx * s_r + ly * c_r))

            p.setPen(QPen(QColor("#38bdf8"), 2))
            p.setBrush(QColor("#0ea5e9"))
            p.drawPolygon(QPolygonF([
                r_pt(12.0, 0.0),
                r_pt(-7.5, 6.0),
                r_pt(-3.5, 0.0),
                r_pt(-7.5, -6.0),
            ]))
            if self.frame_mode != "robot":
                p.setFont(QFont("JetBrains Mono, Monospace", 8, QFont.Bold))
                p.setPen(QColor("#7dd3fc"))
                p.drawText(QPointF(rx + 14, ry - 8), f"({cur_px:+.2f}, {cur_py:+.2f})m")

            # Live HUD overlay
            age_rx = now - self._stats.last_rx_monotonic if self._stats.last_rx_monotonic > 0 else 999.0
            live = age_rx < 1.5 and self._stats.dgrams_per_sec > 0
            p.setPen(QPen(QColor("#22c55e" if live else ("#0ea5e9" if self._has_live_pose else "#334155")), 1))
            p.setBrush(QColor(10, 15, 22, 215))
            hud_h = 74
            p.drawRoundedRect(QRectF(10, 10, 485, hud_h), 6, 6)

            p.setFont(QFont("JetBrains Mono, Monospace", 9, QFont.Bold))
            if live and cur_sw is not None:
                p.setPen(QColor("#4ade80"))
                p.drawText(
                    QRectF(18, 14, 470, 18),
                    Qt.AlignLeft | Qt.AlignVCenter,
                    f"● {frame_lbl}  {self._stats.dgrams_per_sec:,.0f} dgrams/s · "
                    f"{self._stats.sweeps_per_sec:.1f} Hz · {n_sweeps} scans ({self.fade_sec:.1f}s fade)",
                )
                p.setPen(QColor("#cbd5e1"))
                p.setFont(QFont("JetBrains Mono, Monospace", 8))
                p.drawText(
                    QRectF(18, 32, 470, 16),
                    Qt.AlignLeft | Qt.AlignVCenter,
                    f"src={cur_sw.src_ip}  obs={drawn_pts:,} pts  "
                    f"scale={cur_sw.scale_m*1000:.0f}mm  lost={self._stats.total_lost}",
                )
                p.setPen(QColor("#38bdf8" if pose_is_ukf else "#f59e0b"))
                p.drawText(
                    QRectF(18, 47, 470, 16),
                    Qt.AlignLeft | Qt.AlignVCenter,
                    f"live pose: E={cur_px:+.2f}m N={cur_py:+.2f}m θ={math.degrees(cur_th):+.1f}° "
                    f"ω={cur_om:+.2f} v={cur_v:+.2f}",
                )
                if self.ground_mode == "off":
                    gnd_txt = "ground filter: OFF"
                    p.setPen(QColor("#64748b"))
                elif self._ground_plane is not None:
                    ga, gb, gc, gd = self._ground_plane
                    z_fl = -gd / gc if abs(gc) > 1e-6 else 0.0
                    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, gc))))
                    verb = "hid" if self.ground_mode == "hide" else "dimmed"
                    gnd_txt = (
                        f"ground plane: z0={z_fl:+.2f}m tilt={tilt_deg:.1f}° "
                        f"margin={self.ground_margin_m:.2f}m ({verb} {gnd_pts_total:,} pts)"
                    )
                    p.setPen(QColor("#a78bfa"))
                else:
                    gnd_txt = f"ground filter: {self.ground_mode.upper()} (searching for plane...)"
                    p.setPen(QColor("#94a3b8"))
                p.drawText(QRectF(18, 62, 470, 16), Qt.AlignLeft | Qt.AlignVCenter, gnd_txt)
            else:
                gnd_lbl = (
                    f"GND:{self.ground_mode.upper()} ({self.ground_margin_m:.2f}m)"
                    if self.ground_mode != "off"
                    else "GND:OFF"
                )
                p.setPen(QColor("#38bdf8" if self._has_live_pose else "#94a3b8"))
                p.drawText(
                    QRectF(18, 14, 470, 18),
                    Qt.AlignLeft | Qt.AlignVCenter,
                    f"{'●' if self._has_live_pose else '○'} {frame_lbl} · {gnd_lbl} · Waiting for PC2X v3 UDP (2370)",
                )
                p.setFont(QFont("JetBrains Mono, Monospace", 8, QFont.Bold))
                p.setPen(QColor("#7dd3fc" if self._has_live_pose else "#64748b"))
                p.drawText(
                    QRectF(18, 34, 470, 16),
                    Qt.AlignLeft | Qt.AlignVCenter,
                    f"live pose: E={cur_px:+.2f}m  N={cur_py:+.2f}m  θ={math.degrees(cur_th):+.1f}°  "
                    f"trail={len(self._trail)} pts",
                )
                p.setFont(QFont("JetBrains Mono, Monospace", 8))
                p.setPen(QColor("#64748b"))
                p.drawText(
                    QRectF(18, 52, 470, 16),
                    Qt.AlignLeft | Qt.AlignVCenter,
                    f"Arrow tracks live pose · Last {self.fade_sec:.1f}s scans fade · Scroll=Zoom · Drag=Pan",
                )

            # Z color legend
            bar_w, bar_h = 135, 10
            bx, by = w - bar_w - 14, h - 28
            grad = QLinearGradient(bx, 0, bx + bar_w, 0)
            for i, c in enumerate(Z_PALETTE):
                grad.setColorAt(i / (len(Z_PALETTE) - 1), c)
            p.setPen(QPen(QColor("#334155"), 1))
            p.setBrush(grad)
            p.drawRoundedRect(QRectF(bx, by, bar_w, bar_h), 3, 3)
            p.setPen(QColor("#94a3b8"))
            p.setFont(QFont("JetBrains Mono, Monospace", 8))
            p.drawText(QRectF(bx - 4, by + 11, 50, 14), Qt.AlignLeft, f"Z:{self.z_min:+.1f}m")
            p.drawText(QRectF(bx + bar_w - 46, by + 11, 50, 14), Qt.AlignRight, f"{self.z_max:+.1f}m")
        finally:
            p.end()


class CompassWidget(QWidget):
    """Live yaw heading compass + turn-rate arc."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._yaw_deg = 0.0
        self._rate_rads = 0.0
        self._valid = False
        self.setMinimumSize(104, 104)
        self.setMaximumSize(132, 132)

    def set_data(self, yaw_deg: float, rate_rads: float, valid: bool) -> None:
        if math.isfinite(yaw_deg) and abs(yaw_deg) <= 1e6:
            self._yaw_deg = yaw_deg
        if math.isfinite(rate_rads) and abs(rate_rads) <= 1e4:
            self._rate_rads = rate_rads
        self._valid = valid
        self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        try:
            p.setRenderHint(QPainter.Antialiasing)
            side = min(self.width(), self.height()) - 12
            cx, cy = self.width() / 2.0, self.height() / 2.0
            r = side / 2.0

            p.setPen(QPen(QColor("#2b3a4e"), 2))
            p.setBrush(QColor("#0a0f15"))
            p.drawEllipse(QPointF(cx, cy), r, r)

            for deg in range(0, 360, 30):
                rad = math.radians(deg - 90)
                major = (deg % 90 == 0)
                r0 = r - (9 if major else 5)
                p.setPen(QPen(QColor("#64748b" if major else "#334155"), 2 if major else 1))
                p.drawLine(
                    QPointF(cx + r0 * math.cos(rad), cy + r0 * math.sin(rad)),
                    QPointF(cx + (r - 1) * math.cos(rad), cy + (r - 1) * math.sin(rad)),
                )

            p.setPen(QColor("#94a3b8"))
            p.setFont(QFont("Inter", 8, QFont.Bold))
            for lbl, deg in [("N", 0), ("E", 90), ("S", 180), ("W", 270)]:
                rad = math.radians(deg - 90)
                p.drawText(QRectF(cx + (r - 17) * math.cos(rad) - 9, cy + (r - 17) * math.sin(rad) - 7, 18, 14),
                           Qt.AlignCenter, lbl)

            yaw_wrapped = self._yaw_deg % 360.0 if math.isfinite(self._yaw_deg) else 0.0
            span_deg = max(-120.0, min(120.0, self._rate_rads * 45.0)) if math.isfinite(self._rate_rads) else 0.0
            if abs(span_deg) > 0.5:
                arc_pen = QPen(QColor("#f59e0b"), 3.5)
                arc_pen.setCapStyle(Qt.RoundCap)
                p.setPen(arc_pen)
                start_16 = int(((90.0 - yaw_wrapped) % 360.0) * 16.0)
                span_16 = int(-span_deg * 16.0)
                p.drawArc(QRectF(cx - r + 4, cy - r + 4, 2 * (r - 4), 2 * (r - 4)), start_16, span_16)

            ang = math.radians(yaw_wrapped - 90.0)
            tip = QPointF(cx + (r - 20) * math.cos(ang), cy + (r - 20) * math.sin(ang))
            tail = QPointF(cx - (r * 0.25) * math.cos(ang), cy - (r * 0.25) * math.sin(ang))
            c = QColor("#22c55e") if self._valid else QColor("#38bdf8")
            p.setPen(QPen(c, 2.8, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(tail, tip)
            p.setBrush(c)
            p.setPen(Qt.NoPen)
            p.drawEllipse(QPointF(cx, cy), 3.8, 3.8)

            p.setPen(QColor("#f8fafc"))
            p.setFont(QFont("JetBrains Mono, Monospace", 9, QFont.Bold))
            p.drawText(QRectF(cx - 42, cy + r * 0.30, 84, 18), Qt.AlignCenter, f"{self._yaw_deg:+.1f}°")
        finally:
            p.end()


class SparklineWidget(QWidget):
    """Lightweight rolling sparkline plot."""

    def __init__(self, title: str, unit: str, color: str = "#38bdf8", maxlen: int = 120, parent=None):
        super().__init__(parent)
        self._title = title
        self._unit = unit
        self._color = QColor(color)
        self._data: Deque[float] = deque(maxlen=maxlen)
        self.setMinimumHeight(44)

    def push(self, val: float) -> None:
        if math.isfinite(val) and abs(val) < 1e9:
            self._data.append(val)
            self.update()

    def paintEvent(self, _event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        p.setPen(QPen(QColor("#233044"), 1))
        p.setBrush(QColor("#0a0f15"))
        p.drawRoundedRect(QRectF(0.5, 0.5, w - 1, h - 1), 6, 6)

        cur = self._data[-1] if self._data else 0.0
        p.setPen(QColor("#94a3b8"))
        p.setFont(QFont("Inter", 8, QFont.Bold))
        p.drawText(QRectF(8, 2, w - 16, 15), Qt.AlignLeft | Qt.AlignVCenter, self._title)
        p.setPen(self._color)
        p.setFont(QFont("JetBrains Mono, Monospace", 8, QFont.Bold))
        p.drawText(QRectF(8, 2, w - 16, 15), Qt.AlignRight | Qt.AlignVCenter, f"{cur:.4g} {self._unit}")

        if len(self._data) < 2:
            return

        vals = list(self._data)
        vmin, vmax = min(vals), max(vals)
        span = vmax - vmin
        if span < 1e-4:
            mid = 0.5 * (vmin + vmax)
            vmin, vmax = mid - 0.5, mid + 0.5
        else:
            pad = span * 0.15
            vmin -= pad
            vmax += pad

        left, right, top, bottom = 7.0, w - 7.0, 19.0, h - 5.0
        pw, ph = max(1.0, right - left), max(1.0, bottom - top)

        if vmin < 0.0 < vmax:
            zy = bottom - ((0.0 - vmin) / (vmax - vmin)) * ph
            p.setPen(QPen(QColor("#293548"), 1, Qt.DashLine))
            p.drawLine(QPointF(left, zy), QPointF(right, zy))

        path = QPainterPath()
        fill_path = QPainterPath()
        n = len(vals)
        for i, v in enumerate(vals):
            x = left + (i / max(1, n - 1)) * pw
            y = bottom - ((v - vmin) / (vmax - vmin)) * ph
            if i == 0:
                path.moveTo(x, y)
                fill_path.moveTo(x, bottom)
                fill_path.lineTo(x, y)
            else:
                path.lineTo(x, y)
                fill_path.lineTo(x, y)
        fill_path.lineTo(right, bottom)
        fill_path.closeSubpath()

        grad = QLinearGradient(0, top, 0, bottom)
        c_top = QColor(self._color); c_top.setAlpha(55)
        c_bot = QColor(self._color); c_bot.setAlpha(5)
        grad.setColorAt(0.0, c_top)
        grad.setColorAt(1.0, c_bot)
        p.fillPath(fill_path, grad)
        p.setPen(QPen(self._color, 1.7))
        p.drawPath(path)


class CounterTile(QFrame):
    """Compact metric tile for a single lidar/chip counter."""

    def __init__(self, label: str, is_error: bool = False, parent=None):
        super().__init__(parent)
        self.setObjectName("metricTile")
        self._is_error = is_error
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 5, 8, 5)
        lay.setSpacing(1)
        self.lbl_name = QLabel(label)
        self.lbl_name.setStyleSheet("color: #94a3b8; font-size: 11px; font-weight: 600; background: transparent;")
        self.lbl_val = QLabel("0")
        self.lbl_val.setStyleSheet("color: #f8fafc; font-size: 15px; font-weight: 700; font-family: monospace; background: transparent;")
        self.lbl_sub = QLabel("")
        self.lbl_sub.setStyleSheet("color: #64748b; font-size: 10px; background: transparent;")
        lay.addWidget(self.lbl_name)
        lay.addWidget(self.lbl_val)
        lay.addWidget(self.lbl_sub)

    def set_value(self, val: int, sub: str = "") -> None:
        self.lbl_val.setText(str(val))
        self.lbl_sub.setText(sub)
        color = ("#ef4444" if val > 0 else "#22c55e") if self._is_error else ("#38bdf8" if val > 0 else "#cbd5e1")
        self.lbl_val.setStyleSheet(
            f"color: {color}; font-size: 15px; font-weight: 700; font-family: monospace; background: transparent;"
        )


class SensorFusionWindow(QMainWindow):
    def __init__(self, init_esp: Optional[str] = None, init_vn: Optional[str] = None, udp_port: int = 2370):
        super().__init__()
        self.setWindowTitle("UMARV Sensor-Fusion Chip Studio · Live Telemetry & Register Write Console")
        self._fit_to_screen()

        self.chip: Optional[SensorFusionChip] = None
        self.vn_bridge: Optional[VN300USBBridge] = None
        self.pc2x = PC2XReceiver(udp_port=udp_port)
        self._last_tel: Optional[ChipTelemetry] = None
        self._last_ukf: Optional[UkfState] = None
        self._last_drawn_sweep_id: Optional[int] = None
        self._rate_window_start: Optional[float] = None
        self._last_lidar_rate = 0.0
        self._last_sent_rate = 0.0
        self._poll_errors = 0
        self._poll_tick = 0

        # Track UKF tunable constant editors: {word_addr: (lbl_cur, line_edit)}
        self._ukf_const_widgets: Dict[int, Tuple[QLabel, QLineEdit]] = {}
        self._cached_cnt2rad = 2.0 * math.pi / 16384.0
        self._cached_rnom = 0.05

        self._build_ui(udp_port)
        self._refresh_ports(init_esp, init_vn)

        self._spi_timer = QTimer(self)
        self._spi_timer.timeout.connect(self._poll_spi_once)

        self._map_timer = QTimer(self)
        self._map_timer.timeout.connect(self._refresh_lidar_map)
        self._map_timer.start(40)

        self._toggle_lidar_udp()
        if self.cmb_esp.currentData():
            QTimer.singleShot(250, lambda: self._toggle_connect(silent_err=True))

    def _build_ui(self, udp_port: int) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        main_lay = QVBoxLayout(root)
        main_lay.setContentsMargins(10, 8, 10, 6)
        main_lay.setSpacing(8)

        # ---- Top Connection Bar ----------------------------------------------
        top_bar = QHBoxLayout()
        top_bar.setSpacing(7)

        top_bar.addWidget(QLabel("ESP32:"))
        self.cmb_esp = QComboBox()
        self.cmb_esp.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.cmb_esp.setMinimumContentsLength(12)
        self.cmb_esp.setMinimumWidth(110)
        self.cmb_esp.setMaximumWidth(210)
        self.cmb_esp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        top_bar.addWidget(self.cmb_esp)

        self.btn_refresh_ports = QPushButton("↻")
        self.btn_refresh_ports.setToolTip("Rescan /dev/ttyUSB* ports")
        self.btn_refresh_ports.setFixedWidth(32)
        self.btn_refresh_ports.clicked.connect(lambda: self._refresh_ports())
        top_bar.addWidget(self.btn_refresh_ports)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setObjectName("primaryBtn")
        self.btn_connect.clicked.connect(lambda: self._toggle_connect(silent_err=False))
        top_bar.addWidget(self.btn_connect)

        self.cmb_rate = QComboBox()
        self.cmb_rate.addItems(["5 Hz", "10 Hz", "20 Hz"])
        self.cmb_rate.setCurrentIndex(1)
        self.cmb_rate.currentIndexChanged.connect(self._update_poll_rate)
        top_bar.addWidget(self.cmb_rate)

        top_bar.addSpacing(6)
        top_bar.addWidget(QLabel("VN-300:"))
        self.cmb_vn = QComboBox()
        self.cmb_vn.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.cmb_vn.setMinimumContentsLength(12)
        self.cmb_vn.setMinimumWidth(110)
        self.cmb_vn.setMaximumWidth(210)
        self.cmb_vn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        top_bar.addWidget(self.cmb_vn)

        self.btn_vn_bridge = QPushButton("VN Bridge")
        self.btn_vn_bridge.setEnabled(False)
        self.btn_vn_bridge.clicked.connect(self._toggle_vn_bridge)
        top_bar.addWidget(self.btn_vn_bridge)

        top_bar.addSpacing(6)
        top_bar.addWidget(QLabel("UDP:"))
        self.spn_udp_port = QSpinBox()
        self.spn_udp_port.setRange(1, 65535)
        self.spn_udp_port.setValue(udp_port)
        self.spn_udp_port.setFixedWidth(68)
        top_bar.addWidget(self.spn_udp_port)

        self.btn_lidar_udp = QPushButton("Start Map")
        self.btn_lidar_udp.clicked.connect(self._toggle_lidar_udp)
        top_bar.addWidget(self.btn_lidar_udp)

        top_bar.addStretch(1)

        self.lbl_status_pill = QLabel("ESP32: DISCONNECTED")
        self.lbl_status_pill.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Fixed)
        self.lbl_status_pill.setStyleSheet(
            "background-color: #1e293b; color: #94a3b8; border: 1px solid #334155; "
            "border-radius: 12px; padding: 4px 10px; font-weight: 700;"
        )
        top_bar.addWidget(self.lbl_status_pill)
        main_lay.addLayout(top_bar)

        # ---- Main 2-Section Workspace ----------------------------------------
        splitter = QSplitter(Qt.Horizontal)
        main_lay.addWidget(splitter, 1)

        # ======================================================================
        # LEFT SECTION: LIVE OBSERVED DATA (Lidar Map + Sensors + UKF + Counters)
        # ======================================================================
        left = QWidget()
        left_lay = QVBoxLayout(left)
        left_lay.setContentsMargins(0, 0, 5, 0)
        left_lay.setSpacing(6)

        lhdr = QFrame()
        lhdr.setObjectName("sectionHeader")
        lhdr_lay = QHBoxLayout(lhdr)
        lhdr_lay.setContentsMargins(10, 5, 10, 5)
        lbl_l = QLabel("LIVE OBSERVED DATA")
        lbl_l.setStyleSheet("color: #38bdf8; font-weight: 800; letter-spacing: 0.8px; background: transparent;")
        lbl_lsub = QLabel("2D Lidar Map (PC2X v3) · VN-300 · AMT22 · On-Chip UKF State")
        lbl_lsub.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        lbl_lsub.setStyleSheet("color: #94a3b8; font-size: 11px; background: transparent;")
        lhdr_lay.addWidget(lbl_l)
        lhdr_lay.addStretch(1)
        lhdr_lay.addWidget(lbl_lsub)
        left_lay.addWidget(lhdr)

        left_vsplit = QSplitter(Qt.Vertical)
        left_lay.addWidget(left_vsplit, 1)

        # Top-Left: Live 2D Visual Lidar Map
        map_box = QWidget()
        map_lay = QVBoxLayout(map_box)
        map_lay.setContentsMargins(0, 0, 0, 0)
        map_lay.setSpacing(5)
        self.lidar_map = LidarMapWidget()
        map_lay.addWidget(self.lidar_map, 1)

        mc1 = QHBoxLayout()
        mc1.setSpacing(6)
        self.cmb_frame = QComboBox()
        self.cmb_frame.addItem("Global (Auto)", "global_auto")
        self.cmb_frame.addItem("Global (UKF)", "global_ukf")
        self.cmb_frame.addItem("Global (VN+Wheel)", "global_vn")
        self.cmb_frame.addItem("Robot Frame", "robot")
        self.cmb_frame.setFixedWidth(134)
        self.cmb_frame.currentIndexChanged.connect(self._on_frame_mode)
        mc1.addWidget(self.cmb_frame)

        mc1.addWidget(QLabel("Fade:"))
        self.spn_fade = QDoubleSpinBox()
        self.spn_fade.setRange(0.1, 5.0); self.spn_fade.setSingleStep(0.25); self.spn_fade.setValue(1.0)
        self.spn_fade.setSuffix("s"); self.spn_fade.setFixedWidth(64)
        self.spn_fade.valueChanged.connect(lambda v: setattr(self.lidar_map, "fade_sec", v))
        mc1.addWidget(self.spn_fade)

        self.chk_follow = QCheckBox("Follow")
        self.chk_follow.setChecked(False)
        self.chk_follow.setToolTip("Lock camera onto robot (unchecked = fixed world origin so arrow moves across map)")
        self.chk_follow.toggled.connect(lambda v: (setattr(self.lidar_map, "follow_robot", v), self.lidar_map.update()))
        mc1.addWidget(self.chk_follow)

        mc1.addWidget(QLabel("Ground:"))
        self.cmb_ground = QComboBox()
        self.cmb_ground.addItem("Hide Ground", "hide")
        self.cmb_ground.addItem("Dim Ground", "dim")
        self.cmb_ground.addItem("Off (Raw)", "off")
        self.cmb_ground.setFixedWidth(108)
        self.cmb_ground.setToolTip("RANSAC + OLS tilted ground-plane filter")
        self.cmb_ground.currentIndexChanged.connect(self._on_ground_filter)
        mc1.addWidget(self.cmb_ground)

        mc1.addWidget(QLabel("±"))
        self.spn_gnd_margin = QDoubleSpinBox()
        self.spn_gnd_margin.setRange(0.02, 0.60)
        self.spn_gnd_margin.setSingleStep(0.02)
        self.spn_gnd_margin.setValue(0.12)
        self.spn_gnd_margin.setSuffix("m")
        self.spn_gnd_margin.setFixedWidth(68)
        self.spn_gnd_margin.setToolTip("Height margin above fitted ground plane to classify as ground")
        self.spn_gnd_margin.valueChanged.connect(self._on_ground_filter)
        mc1.addWidget(self.spn_gnd_margin)
        mc1.addStretch(1)
        map_lay.addLayout(mc1)

        mc2 = QHBoxLayout()
        mc2.setSpacing(6)
        mc2.addWidget(QLabel("Rng:"))
        self.sld_range = QSlider(Qt.Horizontal)
        self.sld_range.setRange(2, 30)
        self.sld_range.setValue(6)
        self.sld_range.setFixedWidth(95)
        self.lbl_range = QLabel("6m")
        self.sld_range.valueChanged.connect(self._on_range_slider)
        mc2.addWidget(self.sld_range)
        mc2.addWidget(self.lbl_range)

        mc2.addWidget(QLabel("Z:"))
        self.spn_zmin = QDoubleSpinBox()
        self.spn_zmin.setRange(-10.0, 10.0); self.spn_zmin.setSingleStep(0.25); self.spn_zmin.setValue(-2.0)
        self.spn_zmin.setFixedWidth(62); self.spn_zmin.valueChanged.connect(self._on_z_filter)
        self.spn_zmax = QDoubleSpinBox()
        self.spn_zmax.setRange(-10.0, 10.0); self.spn_zmax.setSingleStep(0.25); self.spn_zmax.setValue(2.5)
        self.spn_zmax.setFixedWidth(62); self.spn_zmax.valueChanged.connect(self._on_z_filter)
        mc2.addWidget(self.spn_zmin)
        mc2.addWidget(self.spn_zmax)

        mc2.addWidget(QLabel("Pt:"))
        self.spn_ptsz = QSpinBox()
        self.spn_ptsz.setRange(1, 5); self.spn_ptsz.setValue(2); self.spn_ptsz.setFixedWidth(44)
        self.spn_ptsz.setToolTip("Point size (px)")
        self.spn_ptsz.valueChanged.connect(lambda v: setattr(self.lidar_map, "point_size", v))
        mc2.addWidget(self.spn_ptsz)

        mc2.addStretch(1)
        btn_rst_ukf = QPushButton("Reset UKF")
        btn_rst_ukf.setToolTip("Restart on-chip UKF (re-latch GPS origin LAT0/LON0 and clear trail)")
        btn_rst_ukf.clicked.connect(self._reset_ukf)
        mc2.addWidget(btn_rst_ukf)
        btn_zero_pos = QPushButton("Zero Pos")
        btn_zero_pos.setToolTip("Reset map position & trail back to Origin (0,0)")
        btn_zero_pos.clicked.connect(self.lidar_map.reset_dead_reckoning)
        mc2.addWidget(btn_zero_pos)
        btn_rst = QPushButton("Reset View")
        btn_rst.clicked.connect(self._reset_map_view)
        mc2.addWidget(btn_rst)
        map_lay.addLayout(mc2)
        left_vsplit.addWidget(map_box)

        # Bottom-Left: Compact Live Sensors, UKF State, Sparklines & Counters Tabs
        self.obs_tabs = QTabWidget()
        self.obs_tabs.setUsesScrollButtons(True)
        self._build_obs_sensors_tab()
        self._build_obs_counters_tab()
        left_vsplit.addWidget(self.obs_tabs)
        left_vsplit.setStretchFactor(0, 11)
        left_vsplit.setStretchFactor(1, 9)

        splitter.addWidget(left)

        # ======================================================================
        # RIGHT SECTION: DEDICATED REGISTER WRITES & CHIP TUNING CONSOLE
        # ======================================================================
        right = QWidget()
        right_lay = QVBoxLayout(right)
        right_lay.setContentsMargins(5, 0, 0, 0)
        right_lay.setSpacing(6)

        rhdr = QFrame()
        rhdr.setObjectName("sectionHeader")
        rhdr_lay = QHBoxLayout(rhdr)
        rhdr_lay.setContentsMargins(10, 5, 10, 5)
        lbl_r = QLabel("REGISTER WRITES & CHIP TUNING")
        lbl_r.setStyleSheet("color: #f59e0b; font-weight: 800; letter-spacing: 0.8px; background: transparent;")
        lbl_rsub = QLabel("SPI 0x02–0x3D · New UKF Filter Constants 0x6C / 0x70–73")
        lbl_rsub.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        lbl_rsub.setStyleSheet("color: #94a3b8; font-size: 11px; background: transparent;")
        rhdr_lay.addWidget(lbl_r)
        rhdr_lay.addStretch(1)
        rhdr_lay.addWidget(lbl_rsub)
        right_lay.addWidget(rhdr)

        # Always-visible Last Write Verification Banner
        self.lbl_write_banner = QLabel("Ready for SPI register writes (readback verified on every write).")
        self.lbl_write_banner.setStyleSheet(
            "background-color: #0f172a; color: #94a3b8; border: 1px solid #233042; "
            "border-radius: 6px; padding: 6px 10px; font-family: monospace; font-size: 11px;"
        )
        self.lbl_write_banner.setWordWrap(True)
        self.lbl_write_banner.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        right_lay.addWidget(self.lbl_write_banner)

        self.wr_tabs = QTabWidget()
        self.wr_tabs.setUsesScrollButtons(True)
        self._build_wr_ukf_constants_tab()
        self._build_wr_control_tab()
        self._build_wr_network_tab()
        self._build_wr_host_pose_tab()
        self._build_wr_direct_reg_tab()
        right_lay.addWidget(self.wr_tabs, 1)

        splitter.addWidget(right)
        splitter.setStretchFactor(0, 11)
        splitter.setStretchFactor(1, 11)

        self.statusBar().showMessage("Lidar UDP/2370 active. Connect ESP32 to stream sensors and write chip registers.")

    @staticmethod
    def _wrap_scroll(w: QWidget) -> QScrollArea:
        sc = QScrollArea()
        sc.setWidgetResizable(True)
        sc.setFrameShape(QFrame.NoFrame)
        sc.setStyleSheet("QScrollArea { background: transparent; border: none; }")
        sc.setWidget(w)
        sc.setMinimumSize(0, 0)
        return sc

    # ---- Left Observed Sub-Tabs ----------------------------------------------
    def _build_obs_sensors_tab(self) -> None:
        tab = QWidget()
        lay = QHBoxLayout(tab)
        lay.setSpacing(8)

        # Left half: Compass + numeric sensor + UKF state readouts
        info_col = QVBoxLayout()
        top_row = QHBoxLayout()
        self.compass = CompassWidget()
        top_row.addWidget(self.compass)

        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(3)

        def mk_val(size: int = 11) -> QLabel:
            lbl = QLabel("—")
            lbl.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
            lbl.setStyleSheet(f"font-family: monospace; font-size: {size}px; font-weight: 700; color: #f8fafc; background: transparent;")
            return lbl

        self.val_beat = mk_val()
        self.val_yaw = mk_val()
        self.val_rate = mk_val()
        self.val_latlon = mk_val(11)
        self.val_gnss_sats = mk_val(11)
        self.val_gnss_sats.setStyleSheet("font-family: monospace; font-size: 11px; font-weight: 700; color: #f59e0b; background: transparent;")
        self.val_dw = mk_val()
        self.val_dw_abs = mk_val(12)
        self.val_dw_abs.setStyleSheet("font-family: monospace; font-size: 12px; font-weight: 700; color: #a78bfa; background: transparent;")
        self.val_ukf_pose = mk_val(11)
        self.val_ukf_pose.setStyleSheet("font-family: monospace; font-size: 11px; font-weight: 700; color: #38bdf8; background: transparent;")
        self.val_bridge = mk_val(10)
        self.val_bridge.setStyleSheet("font-family: monospace; font-size: 10px; color: #94a3b8; background: transparent;")

        dw_abs_box = QWidget()
        dw_abs_lay = QHBoxLayout(dw_abs_box)
        dw_abs_lay.setContentsMargins(0, 0, 0, 0)
        dw_abs_lay.setSpacing(6)
        dw_abs_lay.addWidget(self.val_dw_abs, 1)
        btn_zero_dw = QPushButton("Zero")
        btn_zero_dw.setFixedWidth(48)
        btn_zero_dw.setToolTip("Zero 400 Hz dead-wheel absolute count & distance")
        btn_zero_dw.clicked.connect(self._zero_deadwheel)
        dw_abs_lay.addWidget(btn_zero_dw)

        for r_idx, (name, wdg) in enumerate([
            ("Beat", self.val_beat),
            ("VN Yaw", self.val_yaw),
            ("VN Rate", self.val_rate),
            ("Lat/Lon", self.val_latlon),
            ("GNSS Sats", self.val_gnss_sats),
            ("AMT22 Pos", dw_abs_box),
            ("AMT22 Δ", self.val_dw),
            ("UKF State", self.val_ukf_pose),
            ("JD Bridge", self.val_bridge),
        ]):
            n_lbl = QLabel(name)
            n_lbl.setStyleSheet("color: #94a3b8; font-size: 11px; font-weight: 600; background: transparent;")
            grid.addWidget(n_lbl, r_idx, 0)
            grid.addWidget(wdg, r_idx, 1)

        self.pill_vn = QLabel("VN-300: IDLE")
        self.pill_sats = QLabel("SATS: —")
        self.pill_dw = QLabel("AMT22: IDLE")
        for pill in (self.pill_vn, self.pill_sats, self.pill_dw):
            pill.setAlignment(Qt.AlignCenter)
            pill.setStyleSheet("background-color: #1e293b; color: #94a3b8; border-radius: 9px; padding: 2px 7px; font-size: 10px; font-weight: 700;")
        prow = QHBoxLayout()
        prow.addWidget(self.pill_vn)
        prow.addWidget(self.pill_sats)
        prow.addWidget(self.pill_dw)
        grid.addLayout(prow, 9, 0, 1, 2)

        top_row.addLayout(grid, 1)
        info_col.addLayout(top_row)
        lay.addLayout(info_col, 12)

        # Right half: 3 compact rolling sparklines
        plot_col = QVBoxLayout()
        plot_col.setSpacing(4)
        self.plot_yaw = SparklineWidget("VN-300 Yaw", "deg", "#38bdf8")
        self.plot_rate = SparklineWidget("VN-300 Rate (gz)", "rad/s", "#f59e0b")
        self.plot_dw = SparklineWidget("AMT22 Abs Pos", "cnt", "#a78bfa")
        plot_col.addWidget(self.plot_yaw)
        plot_col.addWidget(self.plot_rate)
        plot_col.addWidget(self.plot_dw)
        lay.addLayout(plot_col, 9)

        self.obs_tabs.addTab(self._wrap_scroll(tab), "Sensors, UKF State & Plots")

    def _build_obs_counters_tab(self) -> None:
        tab = QWidget()
        cnt_grid = QGridLayout(tab)
        cnt_grid.setSpacing(6)

        self.tile_lidar = CounterTile("Lidar Pkts", is_error=False)
        self.tile_sent = CounterTile("PC2X Sent", is_error=False)
        self.tile_other = CounterTile("Other Frames", is_error=False)
        self.tile_crc = CounterTile("RX CRC Bad", is_error=True)
        self.tile_ovf = CounterTile("Buf Overflow", is_error=True)
        self.tile_ovr = CounterTile("Motion Overrun", is_error=True)
        self.tile_und = CounterTile("TX Underrun", is_error=True)

        cnt_grid.addWidget(self.tile_lidar, 0, 0)
        cnt_grid.addWidget(self.tile_sent, 0, 1)
        cnt_grid.addWidget(self.tile_other, 0, 2)
        cnt_grid.addWidget(self.tile_crc, 0, 3)
        cnt_grid.addWidget(self.tile_ovf, 1, 0)
        cnt_grid.addWidget(self.tile_ovr, 1, 1)
        cnt_grid.addWidget(self.tile_und, 1, 2)

        cnt_ctrl = QVBoxLayout()
        self.btn_clear_cnt = QPushButton("Clear Counters (0x3D)")
        self.btn_clear_cnt.clicked.connect(self._clear_counters)
        self.chk_rate_mode = QCheckBox("Live pkt/s mode\n(auto-clear 0x3D)")
        self.chk_rate_mode.setChecked(True)
        cnt_ctrl.addWidget(self.btn_clear_cnt)
        cnt_ctrl.addWidget(self.chk_rate_mode)
        cnt_grid.addLayout(cnt_ctrl, 1, 3)

        self.obs_tabs.addTab(self._wrap_scroll(tab), "Lidar Pipeline Counters (0x40–46)")

    # ---- Right Register Write Tab 1: NEW UKF Filter Constants (0x6C / 70-73) -
    def _build_wr_ukf_constants_tab(self) -> None:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setSpacing(8)

        # Header + action row
        top = QHBoxLayout()
        self.chk_ukf_auto_pause = QCheckBox("Auto-pause UKF (CTRL bit 7) during write & restore")
        self.chk_ukf_auto_pause.setChecked(True)
        self.chk_ukf_auto_pause.setToolTip(
            "UKF constants 96..103 are only accepted while CTRL bit 7 == 0 and UKF is idle (~2.5 ms)."
        )
        top.addWidget(self.chk_ukf_auto_pause, 1)
        btn_rd_all = QPushButton("Read All")
        btn_wr_all = QPushButton("Write All 8 Writable")
        btn_wr_all.setObjectName("primaryBtn")
        btn_rd_all.clicked.connect(self._read_all_ukf_constants)
        btn_wr_all.clicked.connect(self._write_all_ukf_constants)
        top.addWidget(btn_rd_all)
        top.addWidget(btn_wr_all)
        lay.addLayout(top)

        # Scrollable list of the 8 writable UKF constants (96..103)
        grp_w = QGroupBox("Writable UKF Constants (Reg 0x6C = 96..103, Word @ 0x70–73, IEEE float32)")
        w_lay = QGridLayout(grp_w)
        w_lay.setHorizontalSpacing(8)
        w_lay.setVerticalSpacing(5)

        for col, title in enumerate(["Word", "Name", "Current (Chip)", "New Float Value", "Action"]):
            h = QLabel(title)
            h.setStyleSheet("color: #94a3b8; font-size: 11px; font-weight: 700; background: transparent;")
            w_lay.addWidget(h, 0, col)

        row_i = 1
        for addr, name, writable, unit, desc in UKF_CONST_DEFS:
            if not writable:
                continue
            lbl_addr = QLabel(f"{addr}")
            lbl_addr.setStyleSheet("font-family: monospace; color: #38bdf8; font-weight: 700; background: transparent;")
            lbl_name = QLabel(f"{name} ({unit})")
            lbl_name.setToolTip(desc)
            lbl_name.setStyleSheet("font-weight: 700; background: transparent;")

            lbl_cur = QLabel("—")
            lbl_cur.setStyleSheet("font-family: monospace; font-size: 11px; color: #cbd5e1; background: transparent;")
            lbl_cur.setMinimumWidth(135)

            ed_new = QLineEdit("")
            ed_new.setPlaceholderText("float32")
            ed_new.setToolTip(f"{desc} ({unit})")
            ed_new.setMinimumWidth(115)

            btn_w = QPushButton("Write")
            btn_w.setFixedWidth(62)
            btn_w.clicked.connect(lambda _=False, a=addr, n=name: self._write_one_ukf_const(a, n))

            w_lay.addWidget(lbl_addr, row_i, 0)
            w_lay.addWidget(lbl_name, row_i, 1)
            w_lay.addWidget(lbl_cur, row_i, 2)
            w_lay.addWidget(ed_new, row_i, 3)
            w_lay.addWidget(btn_w, row_i, 4)

            self._ukf_const_widgets[addr] = (lbl_cur, ed_new)
            row_i += 1
        lay.addWidget(grp_w)

        # Any-Word UKF Inspector (0..126 via 0x6C / 0x70-73)
        grp_insp = QGroupBox("UKF Word Inspector (Reg 0x6C = 0..126 -> Read 0x70–73)")
        insp_lay = QVBoxLayout(grp_insp)
        insp_lay.setSpacing(6)

        irow = QHBoxLayout()
        self.cmb_ukf_word = QComboBox()
        for waddr, wname, wfmt in UKF_NAMED_WORDS:
            self.cmb_ukf_word.addItem(f"{waddr:3d} · {wname} ({wfmt})", waddr)
        self.cmb_ukf_word.currentIndexChanged.connect(
            lambda: self.spn_ukf_word.setValue(self.cmb_ukf_word.currentData())
        )
        self.spn_ukf_word = QSpinBox()
        self.spn_ukf_word.setRange(0, 126)
        self.spn_ukf_word.setValue(96)
        self.spn_ukf_word.setFixedWidth(66)
        btn_rd_w = QPushButton("Read Word")
        btn_rd_w.clicked.connect(self._read_selected_ukf_word)
        irow.addWidget(self.cmb_ukf_word, 1)
        irow.addWidget(QLabel("Addr:"))
        irow.addWidget(self.spn_ukf_word)
        irow.addWidget(btn_rd_w)
        insp_lay.addLayout(irow)

        self.lbl_ukf_word_out = QLabel("Select any word (0–61 storage, 64–111 constants, 112–126 sensor words) and click Read Word.")
        self.lbl_ukf_word_out.setStyleSheet(
            "font-family: monospace; font-size: 12px; color: #38bdf8; background-color: #0a0f15; "
            "border: 1px solid #233042; border-radius: 5px; padding: 6px 8px;"
        )
        insp_lay.addWidget(self.lbl_ukf_word_out)
        lay.addWidget(grp_insp)
        lay.addStretch(1)

        self.wr_tabs.addTab(self._wrap_scroll(tab), "UKF Const (6C/70)")

    # ---- Right Register Write Tab 2: Control & Timing (0x02-0B) --------------
    def _build_wr_control_tab(self) -> None:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setSpacing(9)

        preset_row = QHBoxLayout()
        for label, val in [
            ("Single-Turn AMT22 (0x03)", 0x03),
            ("Multi-Turn (0x43)", 0x43),
            ("UKF + Corr (0xB3)", 0xB3),
            ("Bypass (0x47)", 0x47),
        ]:
            b = QPushButton(label)
            b.clicked.connect(lambda _=False, v=val: self._apply_ctrl_preset(v))
            preset_row.addWidget(b)
        lay.addLayout(preset_row)

        grp_bits = QGroupBox("Control Register (0x02)")
        bg = QGridLayout(grp_bits)
        self.chk_ts_en = QCheckBox("Bit 0: Timestamper Enable (400 Hz)")
        self.chk_lidar_en = QCheckBox("Bit 1: Lidar Pipeline Enable")
        self.chk_bypass = QCheckBox("Bit 2: Lidar Raw Bypass")
        self.chk_4mm = QCheckBox("Bit 3: 4 mm Output Scale (off=2 mm)")
        self.chk_rot = QCheckBox("Bit 4: Motion Correction — Rotation")
        self.chk_shift = QCheckBox("Bit 5: Motion Correction — Translation")
        self.chk_dw_chk = QCheckBox("Bit 6: AMT22 Multi-Turn Parity Check")
        self.chk_ukf = QCheckBox("Bit 7: Use On-Chip UKF State")
        for i, cb in enumerate([
            self.chk_ts_en, self.chk_lidar_en, self.chk_bypass, self.chk_4mm,
            self.chk_rot, self.chk_shift, self.chk_dw_chk, self.chk_ukf,
        ]):
            bg.addWidget(cb, i // 2, i % 2)
        lay.addWidget(grp_bits)

        grp_params = QGroupBox("Sensor & Timing Registers (0x03–0x0B)")
        form = QFormLayout(grp_params)
        form.setSpacing(8)

        self.cmb_model = QComboBox()
        self.cmb_model.addItems(LIDAR_MODELS)
        form.addRow("Lidar Model (0x03):", self.cmb_model)

        self.spn_period = QSpinBox()
        self.spn_period.setRange(100, 65535); self.spn_period.setSuffix(" µs"); self.spn_period.setValue(2500)
        self.lbl_period_hz = QLabel("400.0 Hz")
        self.lbl_period_hz.setStyleSheet("color: #38bdf8; font-weight: 600;")
        self.spn_period.valueChanged.connect(lambda v: self.lbl_period_hz.setText(f"{1e6 / max(1, v):.1f} Hz"))
        per_row = QHBoxLayout(); per_row.addWidget(self.spn_period, 1); per_row.addWidget(self.lbl_period_hz)
        form.addRow("Beat Period (0x04–05):", per_row)

        self.spn_deadline = QSpinBox()
        self.spn_deadline.setRange(50, 65535); self.spn_deadline.setSuffix(" µs"); self.spn_deadline.setValue(1000)
        self.spn_deadline.setToolTip("Keep <= 1000 us at 400 Hz so UKF (~1.3 ms) always finishes before next beat's VN-300 UART shift")
        form.addRow("Meas Deadline (0x06–07):", self.spn_deadline)

        self.spn_lidar_port = QSpinBox()
        self.spn_lidar_port.setRange(0, 65535); self.spn_lidar_port.setValue(0)
        btn_any = QPushButton("Any (0)"); btn_2368 = QPushButton("2368")
        btn_any.clicked.connect(lambda: self.spn_lidar_port.setValue(0))
        btn_2368.clicked.connect(lambda: self.spn_lidar_port.setValue(2368))
        prow = QHBoxLayout(); prow.addWidget(self.spn_lidar_port, 1); prow.addWidget(btn_any); prow.addWidget(btn_2368)
        form.addRow("Lidar UDP Port (0x08–09):", prow)

        self.spn_yaw_off = QDoubleSpinBox()
        self.spn_yaw_off.setRange(-327.68, 327.67); self.spn_yaw_off.setDecimals(2)
        self.spn_yaw_off.setSingleStep(1.0); self.spn_yaw_off.setSuffix(" °")
        form.addRow("Yaw Mount Offset (0x0A–0B):", self.spn_yaw_off)
        lay.addWidget(grp_params)

        btn_row = QHBoxLayout()
        btn_rd = QPushButton("Read Config")
        btn_wr = QPushButton("Write Config (0x02–0x23)")
        btn_wr.setObjectName("primaryBtn")
        btn_rd.clicked.connect(self._read_config_from_chip)
        btn_wr.clicked.connect(self._apply_config_to_chip)
        btn_row.addWidget(btn_rd)
        btn_row.addWidget(btn_wr)
        lay.addLayout(btn_row)
        lay.addStretch(1)

        self.wr_tabs.addTab(self._wrap_scroll(tab), "Control (02–0B)")

    # ---- Right Register Write Tab 3: Network (0x0C-23) -----------------------
    def _build_wr_network_tab(self) -> None:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        grp = QGroupBox("Outgoing PC2X v3 Ethernet / UDP Header (0x0C–0x23)")
        form = QFormLayout(grp)
        form.setSpacing(10)

        self.ed_src_mac = QLineEdit("02:00:00:00:00:01")
        self.ed_dst_mac = QLineEdit("FF:FF:FF:FF:FF:FF")
        self.ed_src_ip = QLineEdit("192.168.1.50")
        self.ed_dst_ip = QLineEdit("255.255.255.255")
        self.spn_src_port = QSpinBox(); self.spn_src_port.setRange(0, 65535); self.spn_src_port.setValue(2370)
        self.spn_dst_port = QSpinBox(); self.spn_dst_port.setRange(0, 65535); self.spn_dst_port.setValue(2370)

        form.addRow("Source MAC (0x0C–11):", self.ed_src_mac)
        form.addRow("Dest MAC (0x12–17):", self.ed_dst_mac)
        form.addRow("Source IP (0x18–1B):", self.ed_src_ip)
        form.addRow("Dest IP (0x1C–1F):", self.ed_dst_ip)
        form.addRow("Source UDP Port (0x20–21):", self.spn_src_port)
        form.addRow("Dest UDP Port (0x22–23):", self.spn_dst_port)
        lay.addWidget(grp)

        btn_row = QHBoxLayout()
        btn_rd = QPushButton("Read Config")
        btn_wr = QPushButton("Write Network Registers")
        btn_wr.setObjectName("primaryBtn")
        btn_rd.clicked.connect(self._read_config_from_chip)
        btn_wr.clicked.connect(self._apply_config_to_chip)
        btn_row.addWidget(btn_rd)
        btn_row.addWidget(btn_wr)
        lay.addLayout(btn_row)
        lay.addStretch(1)

        self.wr_tabs.addTab(self._wrap_scroll(tab), "Network (0C–23)")

    # ---- Right Register Write Tab 4: Host Pose (0x24-3C) ---------------------
    def _build_wr_host_pose_tab(self) -> None:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        note = QLabel(
            "In commit 04025c5+, registers 0x24–0x3B share UKF storage words 0–5 and are "
            "accepted only while CTRL bit 7 (UKF) = 0 and the filter is idle (~2.5 ms)."
        )
        note.setStyleSheet("color: #94a3b8;")
        note.setWordWrap(True)
        lay.addWidget(note)

        grp = QGroupBox("Host Pose State Registers (0x24–0x3B + Strobe 0x3C)")
        form = QFormLayout(grp)
        form.setSpacing(8)
        self.chk_auto_ts = QCheckBox("Use latest chip Beat timestamp automatically")
        self.chk_auto_ts.setChecked(True)
        self.chk_host_dis_ukf = QCheckBox("Clear CTRL bit 7 automatically if UKF is ON")
        self.chk_host_dis_ukf.setChecked(True)
        form.addRow(self.chk_auto_ts)
        form.addRow(self.chk_host_dis_ukf)

        def mk_dspin(lo: float, hi: float, step: float, unit: str) -> QDoubleSpinBox:
            sp = QDoubleSpinBox()
            sp.setRange(lo, hi); sp.setDecimals(4); sp.setSingleStep(step); sp.setSuffix(f" {unit}")
            return sp

        self.spn_hx = mk_dspin(-10000.0, 10000.0, 0.1, "m")
        self.spn_hy = mk_dspin(-10000.0, 10000.0, 0.1, "m")
        self.spn_htheta = mk_dspin(-6.2832, 6.2832, 0.05, "rad")
        self.spn_homega = mk_dspin(-20.0, 20.0, 0.2, "rad/s")
        self.spn_hv = mk_dspin(-30.0, 30.0, 0.5, "m/s")

        form.addRow("X (0x28–2B):", self.spn_hx)
        form.addRow("Y (0x2C–2F):", self.spn_hy)
        form.addRow("Theta (0x30–33):", self.spn_htheta)
        form.addRow("Omega (0x34–37):", self.spn_homega)
        form.addRow("Velocity v (0x38–3B):", self.spn_hv)
        lay.addWidget(grp)

        btn_row = QHBoxLayout()
        btn_rd = QPushButton("Read State (0x24–3B)")
        btn_inj = QPushButton("Write State & Strobe 0x3C")
        btn_inj.setObjectName("primaryBtn")
        btn_rd.clicked.connect(self._read_host_state)
        btn_inj.clicked.connect(self._inject_host_state)
        btn_row.addWidget(btn_rd)
        btn_row.addWidget(btn_inj)
        lay.addLayout(btn_row)
        lay.addStretch(1)

        self.wr_tabs.addTab(self._wrap_scroll(tab), "Host Pose (24–3C)")

    # ---- Right Register Write Tab 5: Direct Register Write + Hex Dump --------
    def _build_wr_direct_reg_tab(self) -> None:
        tab = QWidget()
        lay = QVBoxLayout(tab)
        lay.setSpacing(8)

        grp_rw = QGroupBox("Direct SPI Register Read / Write (0x00–0x7F)")
        rw_lay = QVBoxLayout(grp_rw)
        rw_lay.setSpacing(6)

        row1 = QHBoxLayout()
        self.cmb_reg_preset = QComboBox()
        for addr, name, acc, desc in SPI_REGISTER_MAP:
            self.cmb_reg_preset.addItem(f"0x{addr:02X} · {name} [{acc}] — {desc}", addr)
        self.cmb_reg_preset.currentIndexChanged.connect(
            lambda: self.ed_reg_addr.setText(f"{self.cmb_reg_preset.currentData():02X}")
        )
        row1.addWidget(self.cmb_reg_preset, 1)
        rw_lay.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Addr (hex):"))
        self.ed_reg_addr = QLineEdit("02")
        self.ed_reg_addr.setFixedWidth(54)
        row2.addWidget(self.ed_reg_addr)

        row2.addWidget(QLabel("Type:"))
        self.cmb_reg_fmt = QComboBox()
        self.cmb_reg_fmt.addItems(["u8 (hex)", "u16 (dec)", "u32 (hex)", "float32"])
        self.cmb_reg_fmt.setFixedWidth(96)
        row2.addWidget(self.cmb_reg_fmt)

        row2.addWidget(QLabel("Value:"))
        self.ed_reg_val = QLineEdit("03")
        self.ed_reg_val.setMinimumWidth(110)
        row2.addWidget(self.ed_reg_val, 1)

        btn_r = QPushButton("Read")
        btn_w = QPushButton("Write")
        btn_w.setObjectName("primaryBtn")
        btn_r.clicked.connect(self._raw_read_typed)
        btn_w.clicked.connect(self._raw_write_typed)
        row2.addWidget(btn_r)
        row2.addWidget(btn_w)
        rw_lay.addLayout(row2)
        lay.addWidget(grp_rw)

        # Full 0x00..0x7F Hex Dump
        dump_hdr = QHBoxLayout()
        dump_hdr.addWidget(QLabel("Full Register Dump (0x00–0x7F):"))
        dump_hdr.addStretch(1)
        btn_dump = QPushButton("Refresh Dump")
        btn_dump.clicked.connect(self._refresh_raw_dump)
        dump_hdr.addWidget(btn_dump)
        lay.addLayout(dump_hdr)

        self.txt_hex = QPlainTextEdit()
        self.txt_hex.setReadOnly(True)
        self.txt_hex.setStyleSheet("font-family: 'JetBrains Mono', monospace; font-size: 11px;")
        self.txt_hex.setMaximumHeight(175)
        lay.addWidget(self.txt_hex)

        # Recent Register Writes History Log
        lay.addWidget(QLabel("Register Write Log:"))
        self.txt_wr_log = QPlainTextEdit()
        self.txt_wr_log.setReadOnly(True)
        self.txt_wr_log.setStyleSheet("font-family: 'JetBrains Mono', monospace; font-size: 11px; color: #94a3b8;")
        lay.addWidget(self.txt_wr_log, 1)

        self.wr_tabs.addTab(self._wrap_scroll(tab), "Direct R/W")

    # ---- Helper: Write Verification Banner & Log -----------------------------
    def _set_write_banner(self, msg: str, ok: bool = True) -> None:
        ts = time.strftime("%H:%M:%S")
        if ok:
            self.lbl_write_banner.setStyleSheet(
                "background-color: #052e16; color: #4ade80; border: 1px solid #22c55e; "
                "border-radius: 6px; padding: 6px 10px; font-family: monospace; font-size: 11px; font-weight: 700;"
            )
            prefix = "✓"
        else:
            self.lbl_write_banner.setStyleSheet(
                "background-color: #450a0a; color: #fca5a5; border: 1px solid #ef4444; "
                "border-radius: 6px; padding: 6px 10px; font-family: monospace; font-size: 11px; font-weight: 700;"
            )
            prefix = "✗"
        self.lbl_write_banner.setText(f"{prefix} [{ts}] {msg}")
        self.txt_wr_log.appendPlainText(f"[{ts}] {prefix} {msg}")
        self.statusBar().showMessage(msg)

    # ---- Lidar Map Controls --------------------------------------------------
    def _on_frame_mode(self) -> None:
        self.lidar_map.frame_mode = self.cmb_frame.currentData()
        self.lidar_map.clear_history()

    def _on_ground_filter(self) -> None:
        self.lidar_map.ground_mode = self.cmb_ground.currentData()
        self.lidar_map.ground_margin_m = self.spn_gnd_margin.value()
        self.lidar_map.update()

    def _on_range_slider(self, val: int) -> None:
        self.lidar_map.range_m = float(val)
        self.lbl_range.setText(f"{val}m")
        self.lidar_map.update()

    def _on_z_filter(self) -> None:
        self.lidar_map.z_min = self.spn_zmin.value()
        self.lidar_map.z_max = max(self.spn_zmin.value() + 0.1, self.spn_zmax.value())
        self.lidar_map.update()

    def _reset_map_view(self) -> None:
        self.sld_range.setValue(6)
        self.lidar_map.reset_view()

    def _reset_ukf(self) -> None:
        if not self.chip or not self.chip.is_connected:
            self.lidar_map.reset_dead_reckoning()
            return
        try:
            ctrl = self.chip.read_reg(0x02)
            self.chip.write_reg(0x02, ctrl & ~0x80)
            time.sleep(0.03)
            self.chip.write_reg(0x02, ctrl | 0x80)
            self.chk_ukf.setChecked(True)
            self.lidar_map.reset_dead_reckoning()
            self.lidar_map.clear_history()
            self._set_write_banner(
                f"Reset on-chip UKF (CTRL=0x{ctrl | 0x80:02X}); re-latching GPS origin at (0, 0).",
                ok=True,
            )
        except Exception as exc:
            self._set_write_banner(f"Reset UKF failed: {exc}", ok=False)

    def _toggle_lidar_udp(self) -> None:
        if self.pc2x.is_running:
            self.pc2x.stop()
            self.btn_lidar_udp.setText("Start Map")
            self.btn_lidar_udp.setObjectName("")
        else:
            self.pc2x = PC2XReceiver(udp_port=self.spn_udp_port.value())
            self.pc2x.start()
            self.btn_lidar_udp.setText("Stop Map")
            self.btn_lidar_udp.setObjectName("successBtn")
        self.btn_lidar_udp.style().unpolish(self.btn_lidar_udp)
        self.btn_lidar_udp.style().polish(self.btn_lidar_udp)

    def _refresh_lidar_map(self) -> None:
        sweep, stats = self.pc2x.get_latest()
        # Update smoothly on every new sweep OR while history is fading out
        sid = sweep.sweep_id if sweep is not None else None
        if sid != self._last_drawn_sweep_id or self.lidar_map._history or sweep is None:
            self._last_drawn_sweep_id = sid
            self.lidar_map.set_frame(sweep, stats)

    # ---- ESP32 / VN-300 Connection -------------------------------------------
    def _refresh_ports(self, prefer_esp: Optional[str] = None, prefer_vn: Optional[str] = None) -> None:
        ports = list_serial_ports()
        auto_esp, auto_vn = autodetect_ports()
        target_esp = prefer_esp or auto_esp
        target_vn = prefer_vn or auto_vn

        self.cmb_esp.clear()
        self.cmb_vn.clear()
        for path, desc in ports:
            self.cmb_esp.addItem(desc, path)
            self.cmb_vn.addItem(desc, path)

        if target_esp:
            idx = self.cmb_esp.findData(target_esp)
            if idx >= 0:
                self.cmb_esp.setCurrentIndex(idx)
        if target_vn:
            idx = self.cmb_vn.findData(target_vn)
            if idx >= 0:
                self.cmb_vn.setCurrentIndex(idx)

    def _update_poll_rate(self) -> None:
        hz = [5, 10, 20][self.cmb_rate.currentIndex()]
        if self._spi_timer.isActive():
            self._spi_timer.setInterval(int(1000 / hz))

    def _toggle_connect(self, silent_err: bool = False) -> None:
        if self.chip is not None and self.chip.is_connected:
            self._disconnect()
            return
        port = self.cmb_esp.currentData()
        if not port:
            if not silent_err:
                QMessageBox.warning(self, "No Port", "No ESP32 serial port selected.")
            return
        try:
            self.chip = SensorFusionChip(port)
            self.chip.connect()
            self._poll_errors = 0
            self._poll_tick = 0
            self._rate_window_start = time.monotonic()
            self.btn_connect.setText("Disconnect")
            self.btn_connect.setObjectName("dangerBtn")
            self.btn_connect.style().unpolish(self.btn_connect)
            self.btn_connect.style().polish(self.btn_connect)
            self.btn_vn_bridge.setEnabled(True)

            self._read_config_from_chip()
            # Guard against timestamper.sv in-place UART shift overlapping UKF reads on late beats:
            # at 400 Hz (period <= 2600 us), clamp deadline_us to <= 1000 us so UKF always finishes
            # before beat N+1 starts shifting bytes into yaw/rate/lat/lon registers.
            if self.spn_period.value() <= 2600 and self.spn_deadline.value() > 1000:
                self.chip.write_regs(0x06, struct.pack("<H", 1000))
                self.spn_deadline.setValue(1000)
                self._set_write_banner(
                    "Auto-set Meas Deadline (0x06–07) = 1000 µs to keep UKF clear of next beat's UART shift.",
                    ok=True,
                )
            self._read_host_state()
            self._read_all_ukf_constants()
            self._refresh_raw_dump()

            hz = [5, 10, 20][self.cmb_rate.currentIndex()]
            self._spi_timer.start(int(1000 / hz))
            self.statusBar().showMessage(f"Connected to chip via ESP32 on {port}")
            if silent_err and self.cmb_vn.currentData() and self.cmb_vn.currentData() != port:
                if "VN-300" in self.cmb_vn.currentText():
                    QTimer.singleShot(200, self._toggle_vn_bridge)
        except Exception as exc:
            if self.chip:
                self.chip.close()
                self.chip = None
            self.statusBar().showMessage(f"ESP32 connect failed: {exc}")
            if not silent_err:
                QMessageBox.critical(self, "ESP32 Connection Failed", str(exc))

    def _disconnect(self) -> None:
        self._spi_timer.stop()
        if self.vn_bridge and self.vn_bridge.is_running:
            self._toggle_vn_bridge()
        if self.chip:
            self.chip.close()
            self.chip = None
        self.btn_connect.setText("Connect ESP32")
        self.btn_connect.setObjectName("primaryBtn")
        self.btn_connect.style().unpolish(self.btn_connect)
        self.btn_connect.style().polish(self.btn_connect)
        self.btn_vn_bridge.setEnabled(False)
        self.lbl_status_pill.setText("ESP32: DISCONNECTED")
        self.lbl_status_pill.setStyleSheet(
            "background-color: #1e293b; color: #94a3b8; border: 1px solid #334155; "
            "border-radius: 12px; padding: 4px 11px; font-weight: 700;"
        )

    def _toggle_vn_bridge(self) -> None:
        if self.vn_bridge and self.vn_bridge.is_running:
            self.vn_bridge.stop()
            self.vn_bridge = None
            self.btn_vn_bridge.setText("Start VN Bridge")
            self.btn_vn_bridge.setObjectName("")
            self.btn_vn_bridge.style().unpolish(self.btn_vn_bridge)
            self.btn_vn_bridge.style().polish(self.btn_vn_bridge)
            self.statusBar().showMessage("VN-300 USB bridge stopped (restored to $VNINS).")
            return
        if not self.chip or not self.chip.is_connected:
            return
        vn_port = self.cmb_vn.currentData()
        if not vn_port or vn_port == self.chip.port:
            QMessageBox.warning(self, "VN-300 Port", "Select the VN-300 USB port (different from ESP32).")
            return
        self.vn_bridge = VN300USBBridge(self.chip, vn_port=vn_port)
        self.vn_bridge.start()
        self.btn_vn_bridge.setText("Stop VN Bridge")
        self.btn_vn_bridge.setObjectName("successBtn")
        self.btn_vn_bridge.style().unpolish(self.btn_vn_bridge)
        self.btn_vn_bridge.style().polish(self.btn_vn_bridge)
        self.statusBar().showMessage(f"Live VN-300 USB bridge active ({vn_port} -> ESP32 -> Pmod JD).")

    # ---- Live SPI Telemetry Poll ---------------------------------------------
    def _poll_spi_once(self) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        try:
            tel = self.chip.read_telemetry(include_bridge=True)
            self._poll_errors = 0
        except Exception as exc:
            self._poll_errors += 1
            self.statusBar().showMessage(f"SPI poll warning ({self._poll_errors}): {exc}")
            if self._poll_errors >= 8:
                self._disconnect()
            return

        self._last_tel = tel
        self._poll_tick += 1

        # Read live UKF output state + learned radius every 2 ticks (~5 Hz)
        if self._poll_tick % 2 == 1:
            try:
                ukf = self.chip.read_ukf_state()
                self._last_ukf = ukf
                self.val_ukf_pose.setText(
                    f"x={ukf.x:+.2f} y={ukf.y:+.2f} θ={ukf.theta:+.2f} ω={ukf.omega:+.2f} v={ukf.v:+.2f} r={ukf.wheel_r:.4f}m"
                )
            except Exception:
                pass

        self.lbl_status_pill.setText(f"CHIP ONLINE · 0x{tel.chip_id:02X} · CTRL 0x{tel.ctrl:02X}")
        self.lbl_status_pill.setStyleSheet(
            "background-color: #052e16; color: #4ade80; border: 1px solid #22c55e; "
            "border-radius: 12px; padding: 4px 11px; font-weight: 700;"
        )

        self.compass.set_data(tel.vn_yaw_deg, tel.vn_rate_rads, tel.vn_valid)
        self.lidar_map.update_live_pose(
            vn_yaw_deg=tel.vn_yaw_deg,
            dw_abs_counts=tel.dw_abs_counts,
            cnt2rad=self._cached_cnt2rad,
            rnom=self._cached_rnom,
            ukf=self._last_ukf,
            state_from_filter=bool(tel.ctrl & 0x80),
        )
        self.val_beat.setText(f"{tel.beat_ts_us / 1e6:9.3f} s")
        self.val_yaw.setText(f"{tel.vn_yaw_deg:+8.3f} °")
        self.val_rate.setText(f"{tel.vn_rate_rads:+8.4f} rad/s")
        self.val_latlon.setText(f"{tel.vn_lat_deg:+.6f}, {tel.vn_lon_deg:+.6f}")
        self.val_dw.setText(f"{tel.dw_delta_counts:+d} cnt/beat")
        revs = tel.dw_abs_counts / 16384.0
        dist_m = SensorFusionChip.deadwheel_distance_m(
            tel.dw_abs_counts, self._cached_cnt2rad, self._cached_rnom
        )
        self.val_dw_abs.setText(f"{tel.dw_abs_counts:+,d} cnt  ({revs:+.2f} rev · {dist_m:+.3f} m)")

        br_state = "TX ON" if tel.esp_vn_bridge_en else "TX OFF"
        if self.vn_bridge and self.vn_bridge.last_error:
            br_state = f"ERR: {self.vn_bridge.last_error[:18]}"
        self.val_bridge.setText(f"{br_state} · sync={tel.esp_sync_irq} · dw_beats={tel.dw_good_beats}")

        def set_pill(lbl: QLabel, text: str, ok: bool) -> None:
            lbl.setText(text)
            lbl.setStyleSheet(
                "background-color: #052e16; color: #4ade80; border: 1px solid #22c55e; border-radius: 9px; padding: 2px 7px; font-size: 10px; font-weight: 700;"
                if ok else
                "background-color: #1e293b; color: #94a3b8; border: 1px solid #334155; border-radius: 9px; padding: 2px 7px; font-size: 10px; font-weight: 700;"
            )

        dw_ok_live = tel.dw_valid or tel.dw_last_valid
        set_pill(self.pill_vn, "VN-300: VALID" if tel.vn_valid else "VN-300: NO FIX / IDLE", tel.vn_valid)
        set_pill(self.pill_dw, "AMT22: VALID" if dw_ok_live else "AMT22: IDLE", dw_ok_live)

        if self.vn_bridge and self.vn_bridge.is_running:
            gs = self.vn_bridge.get_gnss_status()
            if gs.last_gnss_monotonic > 0 or gs.last_ins_monotonic > 0:
                fix_ok = (gs.gnss1_fix >= 2) or (gs.gnss2_fix >= 2)
                sat_color = "#4ade80" if (fix_ok and gs.chip_status_ok) else ("#38bdf8" if fix_ok else "#f59e0b")
                self.val_gnss_sats.setStyleSheet(
                    f"font-family: monospace; font-size: 11px; font-weight: 700; color: {sat_color}; background: transparent;"
                )
                u1_str = f"±{gs.gnss1_pos_u_m:.1f}m" if gs.gnss1_fix >= 2 and gs.gnss1_pos_u_m < 1e4 else ""
                self.val_gnss_sats.setText(
                    f"G1:{gs.gnss1_sats} ({gs.gnss1_fix_name}{(' ' + u1_str) if u1_str else ''}) · "
                    f"G2:{gs.gnss2_sats} ({gs.gnss2_fix_name}) · INS:{gs.ins_mode_name}"
                )
                pill_ok = fix_ok or (gs.gnss1_sats >= 4) or (gs.gnss2_sats >= 4)
                set_pill(self.pill_sats, f"SATS: G1={gs.gnss1_sats} G2={gs.gnss2_sats}", pill_ok)
            else:
                self.val_gnss_sats.setText("Polling VN-300 GNSS1/GNSS2...")
                set_pill(self.pill_sats, "SATS: ...", False)
        else:
            self.val_gnss_sats.setText("Start VN Bridge for live satellites")
            set_pill(self.pill_sats, "SATS: OFF", False)

        self.plot_yaw.push(tel.vn_yaw_deg)
        self.plot_rate.push(tel.vn_rate_rads)
        self.plot_dw.push(float(tel.dw_abs_counts))

        if self.chk_rate_mode.isChecked():
            now = tel.timestamp
            if self._rate_window_start is None:
                self._rate_window_start = now
            dt = now - self._rate_window_start
            if dt >= 0.08:
                self._last_lidar_rate = tel.cnt_lidar / dt
                self._last_sent_rate = tel.cnt_sent / dt
                try:
                    self.chip.clear_counters()
                    self._rate_window_start = time.monotonic()
                except Exception:
                    pass
            l_sub = f"~{self._last_lidar_rate:,.0f} pkt/s"
            s_sub = f"~{self._last_sent_rate:,.0f} fps"
        else:
            l_sub = "sat @ 255" if tel.cnt_lidar == 255 else "cumulative"
            s_sub = "sat @ 255" if tel.cnt_sent == 255 else "cumulative"

        self.tile_lidar.set_value(tel.cnt_lidar, l_sub)
        self.tile_sent.set_value(tel.cnt_sent, s_sub)
        self.tile_other.set_value(tel.cnt_other, "non-lidar")
        self.tile_crc.set_value(tel.cnt_rx_crc_bad, "OK" if tel.cnt_rx_crc_bad == 0 else "BAD FCS")
        self.tile_ovf.set_value(tel.cnt_buf_ovf, "OK" if tel.cnt_buf_ovf == 0 else "DROPPED")
        self.tile_ovr.set_value(tel.cnt_mot_ovr, "OK" if tel.cnt_mot_ovr == 0 else "OVERRUN")
        self.tile_und.set_value(tel.cnt_tx_und, "OK" if tel.cnt_tx_und == 0 else "UNDERRUN")

    # ---- UKF Constants (0x6C / 0x70-73) Handlers -----------------------------
    def _zero_deadwheel(self) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        try:
            self.chip.zero_deadwheel()
            self.lidar_map.reset_dead_reckoning()
            self._set_write_banner("Zeroed 400 Hz dead-wheel counts & reset map position to Origin (0,0).", ok=True)
        except Exception as exc:
            self._set_write_banner(f"Zero dead-wheel failed: {exc}", ok=False)

    def _read_all_ukf_constants(self) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        try:
            vals = self.chip.read_ukf_tunables()
            if 102 in vals and math.isfinite(vals[102][0]):
                self._cached_cnt2rad = vals[102][0]
            if 103 in vals and math.isfinite(vals[103][0]):
                self._cached_rnom = vals[103][0]
            for addr, (lbl_cur, ed_new) in self._ukf_const_widgets.items():
                fval, uval = vals[addr]
                lbl_cur.setText(f"{fval:.6g}  (0x{uval:08X})")
                if not ed_new.text().strip():
                    ed_new.setText(f"{fval:.7g}")
            self.statusBar().showMessage("Read all 8 writable UKF constants (words 96..103).")
        except Exception as exc:
            self._set_write_banner(f"Read UKF constants failed: {exc}", ok=False)

    def _write_one_ukf_const(self, addr: int, name: str) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        lbl_cur, ed_new = self._ukf_const_widgets[addr]
        try:
            target_f = float(ed_new.text().strip())
            u_in = struct.unpack("<I", struct.pack("<f", target_f))[0]
            u_rb = self.chip.write_ukf_word_u32(
                addr, u_in, auto_pause_ukf=self.chk_ukf_auto_pause.isChecked()
            )
            f_rb = struct.unpack("<f", struct.pack("<I", u_rb))[0]
            lbl_cur.setText(f"{f_rb:.6g}  (0x{u_rb:08X})")
            if addr == 102 and math.isfinite(f_rb):
                self._cached_cnt2rad = f_rb
            elif addr == 103 and math.isfinite(f_rb):
                self._cached_rnom = f_rb
            matched = (u_rb == u_in)
            self._set_write_banner(
                f"UKF word {addr} ({name}): wrote {target_f:.6g} (0x{u_in:08X}) -> readback {f_rb:.6g} (0x{u_rb:08X})",
                ok=matched,
            )
        except Exception as exc:
            self._set_write_banner(f"Write UKF {name} ({addr}) failed: {exc}", ok=False)

    def _write_all_ukf_constants(self) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        try:
            to_write: Dict[int, float] = {}
            for addr, (_lbl, ed) in self._ukf_const_widgets.items():
                txt = ed.text().strip()
                if txt:
                    to_write[addr] = float(txt)
            if not to_write:
                return
            rb = self.chip.write_ukf_tunables(
                to_write, auto_pause_ukf=self.chk_ukf_auto_pause.isChecked()
            )
            mismatches = 0
            for addr, (f_rb, u_rb) in rb.items():
                lbl_cur, _ = self._ukf_const_widgets[addr]
                lbl_cur.setText(f"{f_rb:.6g}  (0x{u_rb:08X})")
                u_exp = struct.unpack("<I", struct.pack("<f", to_write[addr]))[0]
                if u_rb != u_exp:
                    mismatches += 1
            self._set_write_banner(
                f"Batch-wrote {len(to_write)} UKF constants (96..103): "
                + ("all verified!" if mismatches == 0 else f"{mismatches} readback mismatch(es)!"),
                ok=(mismatches == 0),
            )
        except Exception as exc:
            self._set_write_banner(f"Batch write UKF constants failed: {exc}", ok=False)

    def _read_selected_ukf_word(self) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        try:
            addr = self.spn_ukf_word.value()
            u = self.chip.read_ukf_word_u32(addr)
            f = struct.unpack("<f", struct.pack("<I", u))[0]
            i32 = struct.unpack("<i", struct.pack("<I", u))[0]
            self.lbl_ukf_word_out.setText(
                f"Word {addr:3d} (0x{addr:02X}):  hex=0x{u:08X}  |  f32={f:.7g}  |  i32={i32}  |  u32={u}"
            )
        except Exception as exc:
            self._set_write_banner(f"Read UKF word failed: {exc}", ok=False)

    # ---- Config / Network / Host Pose / Direct Reg Handlers ------------------
    def _populate_config_ui(self, cfg: ChipConfig) -> None:
        self.chk_ts_en.setChecked(cfg.ts_enable)
        self.chk_lidar_en.setChecked(cfg.lidar_enable)
        self.chk_bypass.setChecked(cfg.lidar_bypass)
        self.chk_4mm.setChecked(cfg.scale_4mm)
        self.chk_rot.setChecked(cfg.corr_rot_en)
        self.chk_shift.setChecked(cfg.corr_shift_en)
        self.chk_dw_chk.setChecked(cfg.dw_check_turns)
        self.chk_ukf.setChecked(cfg.state_from_filter)
        self.cmb_model.setCurrentIndex(cfg.lidar_model & 0x03)
        self.spn_period.setValue(cfg.period_us)
        self.spn_deadline.setValue(cfg.deadline_us)
        self.spn_lidar_port.setValue(cfg.lidar_port)
        self.spn_yaw_off.setValue(cfg.yaw_offset_deg)
        self.ed_src_mac.setText(cfg.src_mac)
        self.ed_dst_mac.setText(cfg.dst_mac)
        self.ed_src_ip.setText(cfg.src_ip)
        self.ed_dst_ip.setText(cfg.dst_ip)
        self.spn_src_port.setValue(cfg.src_port)
        self.spn_dst_port.setValue(cfg.dst_port)

    def _collect_config_from_ui(self) -> ChipConfig:
        return ChipConfig(
            ts_enable=self.chk_ts_en.isChecked(),
            lidar_enable=self.chk_lidar_en.isChecked(),
            lidar_bypass=self.chk_bypass.isChecked(),
            scale_4mm=self.chk_4mm.isChecked(),
            corr_rot_en=self.chk_rot.isChecked(),
            corr_shift_en=self.chk_shift.isChecked(),
            dw_check_turns=self.chk_dw_chk.isChecked(),
            state_from_filter=self.chk_ukf.isChecked(),
            lidar_model=self.cmb_model.currentIndex(),
            period_us=self.spn_period.value(),
            deadline_us=self.spn_deadline.value(),
            lidar_port=self.spn_lidar_port.value(),
            yaw_offset_deg=self.spn_yaw_off.value(),
            src_mac=self.ed_src_mac.text().strip(),
            dst_mac=self.ed_dst_mac.text().strip(),
            src_ip=self.ed_src_ip.text().strip(),
            dst_ip=self.ed_dst_ip.text().strip(),
            src_port=self.spn_src_port.value(),
            dst_port=self.spn_dst_port.value(),
        )

    def _apply_ctrl_preset(self, ctrl_val: int) -> None:
        self.chk_ts_en.setChecked(bool(ctrl_val & 0x01))
        self.chk_lidar_en.setChecked(bool(ctrl_val & 0x02))
        self.chk_bypass.setChecked(bool(ctrl_val & 0x04))
        self.chk_4mm.setChecked(bool(ctrl_val & 0x08))
        self.chk_rot.setChecked(bool(ctrl_val & 0x10))
        self.chk_shift.setChecked(bool(ctrl_val & 0x20))
        self.chk_dw_chk.setChecked(bool(ctrl_val & 0x40))
        self.chk_ukf.setChecked(bool(ctrl_val & 0x80))
        if self.chip and self.chip.is_connected:
            self._apply_config_to_chip()

    def _read_config_from_chip(self) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        try:
            cfg = self.chip.read_config()
            self._populate_config_ui(cfg)
            self.statusBar().showMessage(
                f"Read config: CTRL=0x{cfg.ctrl_byte:02X}, port={cfg.lidar_port}, period={cfg.period_us} µs"
            )
        except Exception as exc:
            self._set_write_banner(f"Read config failed: {exc}", ok=False)

    def _apply_config_to_chip(self) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        try:
            req = self._collect_config_from_ui()
            rb = self.chip.write_config(req)
            self._populate_config_ui(rb)
            self._set_write_banner(
                f"Wrote 0x02–0x23: CTRL=0x{rb.ctrl_byte:02X}, port={rb.lidar_port}, "
                f"period={rb.period_us}µs, yaw_off={rb.yaw_offset_deg:+.2f}°",
                ok=(rb.ctrl_byte == req.ctrl_byte and rb.lidar_port == req.lidar_port),
            )
        except Exception as exc:
            self._set_write_banner(f"Write config failed: {exc}", ok=False)

    def _clear_counters(self) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        try:
            self.chip.clear_counters()
            self._rate_window_start = time.monotonic()
            self._set_write_banner("Wrote Reg 0x3D = 0x01 (event counters cleared).", ok=True)
        except Exception as exc:
            self._set_write_banner(f"Clear counters failed: {exc}", ok=False)

    def _read_host_state(self) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        try:
            st = self.chip.read_host_state()
            self.spn_hx.setValue(st.x if math.isfinite(st.x) else 0.0)
            self.spn_hy.setValue(st.y if math.isfinite(st.y) else 0.0)
            self.spn_htheta.setValue(st.theta if math.isfinite(st.theta) else 0.0)
            self.spn_homega.setValue(st.omega if math.isfinite(st.omega) else 0.0)
            self.spn_hv.setValue(st.v if math.isfinite(st.v) else 0.0)
        except Exception as exc:
            self._set_write_banner(f"Read host state failed: {exc}", ok=False)

    def _inject_host_state(self) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        try:
            ts = self._last_tel.beat_ts_us if (self.chk_auto_ts.isChecked() and self._last_tel) else int(time.monotonic() * 1e6)
            st = HostState(
                ts_us=ts,
                x=self.spn_hx.value(),
                y=self.spn_hy.value(),
                theta=self.spn_htheta.value(),
                omega=self.spn_homega.value(),
                v=self.spn_hv.value(),
            )
            self.chip.inject_host_state(st, auto_disable_ukf=self.chk_host_dis_ukf.isChecked())
            self._read_config_from_chip()
            self._set_write_banner(
                f"Wrote Host Pose 0x24–3B + strobed 0x3C: ω={st.omega:+.3f} rad/s, v={st.v:+.3f} m/s",
                ok=True,
            )
        except Exception as exc:
            self._set_write_banner(f"Inject host pose failed: {exc}", ok=False)

    def _raw_read_typed(self) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        try:
            addr = int(self.ed_reg_addr.text().strip(), 16) & 0x7F
            fmt_idx = self.cmb_reg_fmt.currentIndex()
            if fmt_idx == 0:
                val = self.chip.read_reg(addr)
                self.ed_reg_val.setText(f"{val:02X}")
                self._set_write_banner(f"Read Reg 0x{addr:02X} = 0x{val:02X} ({val})", ok=True)
            elif fmt_idx == 1:
                raw = self.chip.read_regs(addr, 2)
                v = struct.unpack("<H", raw)[0]
                self.ed_reg_val.setText(str(v))
                self._set_write_banner(f"Read Reg 0x{addr:02X}..{addr+1:02X} = {v} (0x{v:04X})", ok=True)
            elif fmt_idx == 2:
                raw = self.chip.read_regs(addr, 4)
                v = struct.unpack("<I", raw)[0]
                self.ed_reg_val.setText(f"{v:08X}")
                self._set_write_banner(f"Read Reg 0x{addr:02X}..{addr+3:02X} = 0x{v:08X}", ok=True)
            else:
                raw = self.chip.read_regs(addr, 4)
                v = struct.unpack("<f", raw)[0]
                self.ed_reg_val.setText(f"{v:.7g}")
                self._set_write_banner(f"Read Reg 0x{addr:02X}..{addr+3:02X} = {v:.7g} (0x{struct.unpack('<I', raw)[0]:08X})", ok=True)
        except Exception as exc:
            self._set_write_banner(f"Direct read failed: {exc}", ok=False)

    def _raw_write_typed(self) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        try:
            addr = int(self.ed_reg_addr.text().strip(), 16) & 0x7F
            fmt_idx = self.cmb_reg_fmt.currentIndex()
            txt = self.ed_reg_val.text().strip()
            if fmt_idx == 0:
                payload = bytes([int(txt, 16) & 0xFF])
            elif fmt_idx == 1:
                payload = struct.pack("<H", int(txt, 0) & 0xFFFF)
            elif fmt_idx == 2:
                payload = struct.pack("<I", int(txt, 16) & 0xFFFFFFFF)
            else:
                payload = struct.pack("<f", float(txt))

            rb = self.chip.write_regs(addr, payload)
            matched = (rb == payload) or addr in (0x3C, 0x3D, 0x6C)  # WO strobes/addr reg
            self._set_write_banner(
                f"Wrote 0x{addr:02X} ({len(payload)}B): {payload.hex().upper()} -> readback {rb.hex().upper()}",
                ok=matched,
            )
            self._refresh_raw_dump()
        except Exception as exc:
            self._set_write_banner(f"Direct write failed: {exc}", ok=False)

    def _refresh_raw_dump(self) -> None:
        if not self.chip or not self.chip.is_connected:
            return
        try:
            data = self.chip.read_regs(0x00, 128)
            lines = ["Addr  00 01 02 03 04 05 06 07  08 09 0A 0B 0C 0D 0E 0F"]
            lines.append("----  -----------------------  -----------------------")
            for base in range(0, 128, 16):
                row = data[base : base + 16]
                left = " ".join(f"{b:02X}" for b in row[:8])
                right = " ".join(f"{b:02X}" for b in row[8:])
                lines.append(f"0x{base:02X}  {left}  {right}")
            self.txt_hex.setPlainText("\n".join(lines))
        except Exception as exc:
            self.statusBar().showMessage(f"Dump failed: {exc}")

    def _fit_to_screen(self) -> None:
        """Size and center the window cleanly inside the active screen's available geometry."""
        screen = QApplication.primaryScreen()
        if screen is not None:
            avail = screen.availableGeometry()
            w = min(1360, max(920, int(avail.width() * 0.94)))
            h = min(760, max(600, int(avail.height() * 0.92)))
            w = min(w, avail.width())
            h = min(h, avail.height())
            self.resize(w, h)
            self.move(
                avail.x() + max(0, (avail.width() - w) // 2),
                avail.y() + max(0, (avail.height() - h) // 2),
            )
        else:
            self.resize(1280, 720)

    def closeEvent(self, event) -> None:
        self._map_timer.stop()
        self.pc2x.stop()
        self._disconnect()
        super().closeEvent(event)


def main() -> None:
    ap = argparse.ArgumentParser(description="UMARV Sensor-Fusion Chip Studio")
    ap.add_argument("--port", default=None, help="ESP32 serial port (auto-detected if omitted)")
    ap.add_argument("--vn", default=None, help="VN-300 USB serial port (auto-detected if omitted)")
    ap.add_argument("--udp-port", type=int, default=2370, help="PC2X v3 UDP port (default 2370)")
    args = ap.parse_args()

    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_QSS)
    win = SensorFusionWindow(init_esp=args.port, init_vn=args.vn, udp_port=args.udp_port)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
