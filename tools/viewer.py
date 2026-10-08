#!/usr/bin/env python3
"""
Whale Detangler viewer: live 3D view and graphs of the data logger, with recording and playback.

Live (main firmware, or bringup/accel_spi):
  python tools/viewer.py live --port COM5                  # sends "live on" after the board boots
  python tools/viewer.py live --port COM5 --record bench.csv

Playback (a viewer recording, or a FRAM dump captured with serial_capture.py):
  python tools/viewer.py play bench.csv
  python tools/serial_capture.py --port COM5 --send dump --until-end -o dump.csv
  python tools/viewer.py play dump.csv

What you see:
  3D view   the logger as a box, tilted by the measured gravity direction (yaw cannot be measured
            without a magnetometer), the acceleration vector (yellow), and a trail of recent vector
            tips coloured by device state. Box faces: +X red, +Y green, +Z (top) blue.
  graphs    acceleration (g), depth (m, with deployment phases shaded), motion (mg), battery (V)
  panel     every field of the current sample, decoded flags, and the latest event
  console   (live) firmware text output and a command box: try info, logsurface on, dump

Requires: pip install pyserial numpy PyQt6 pyqtgraph PyOpenGL
"""

import argparse
import bisect
import math
import sys
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional

import numpy as np

STATE_NAMES = {0: "BOOT", 1: "SAFE_IDLE", 2: "MONITORING", 3: "CHARGING", 4: "FIRED", 5: "FAULT"}
PHASE_NAMES = {0: "SURFACE", 1: "DESCENT", 2: "SOAK", 3: "ASCENT"}
FLAG_NAMES = [(1, "ACCEL"), (2, "PRESSURE"), (4, "DEPTH"), (8, "BATT"), (16, "TEMP"),
              (32, "ACTIVITY"), (64, "GAUGE_ALERT"), (128, "P_CLIP")]
STATE_COLORS = {0: (0.6, 0.6, 0.6, 1), 1: (0.4, 0.8, 1.0, 1), 2: (0.2, 1.0, 0.2, 1),
                3: (1.0, 1.0, 0.1, 1), 4: (1.0, 0.15, 0.15, 1), 5: (1.0, 0.2, 1.0, 1)}
PHASE_BRUSHES = {1: (60, 120, 255, 45), 2: (60, 200, 90, 45), 3: (255, 150, 40, 45)}
RECORD_TAGS = {"S", "SMP", "EVT", "CYC", "BOOT", "R"}
GRAVITY_TAU_S = 60.0  # same baseline time constant as the firmware's motion_mg


# ---------------------------------------------------------------------------------------------
# parsing (no GUI dependencies, so it can be tested on its own)
# ---------------------------------------------------------------------------------------------
@dataclass
class Sample:
    boot: Optional[int]
    t_ms: int
    state: Optional[int]
    phase: Optional[int]
    ax: float  # mg
    ay: float
    az: float
    motion: Optional[int]
    temp: Optional[int]
    p_raw: Optional[int]
    depth_cm: Optional[int]
    mv: Optional[int]
    soc: Optional[int]
    flags: Optional[int]
    t: float = 0.0  # continuous timeline, seconds
    partial: bool = False  # recording sample (R line): acceleration only, other fields filled in on load


@dataclass
class Event:
    boot: Optional[int]
    t_ms: int
    name: str
    d0: Optional[int]
    d1: Optional[int]
    t: float = 0.0


@dataclass
class Cycle:
    boot: Optional[int]
    fields: List[Optional[int]]  # cycle,start_ms,end_ms,bottom_cm,soak_min,soak_max,soak_ms,drop..,ret..


def _num(s):
    s = s.strip()
    if not s:
        return None
    try:
        return int(s)
    except ValueError:
        try:
            return float(s)
        except ValueError:
            return None


def strip_host_time(line):
    """Recordings and serial_capture files prefix each line with '<host seconds>,'."""
    parts = line.split(",", 2)
    if len(parts) >= 2 and parts[1] in RECORD_TAGS:
        try:
            float(parts[0])
            return line.split(",", 1)[1]
        except ValueError:
            pass
    return line


def _sample(boot, v):
    # v = t_ms,state,phase,ax,ay,az,motion,temp,p_raw,depth,mv,soc,flags (missing fields -> None)
    v = list(v) + [None] * (13 - len(v))
    if v[0] is None or v[3] is None or v[4] is None or v[5] is None:
        return None
    return Sample(boot, int(v[0]), v[1], v[2], float(v[3]), float(v[4]), float(v[5]),
                  v[6], v[7], v[8], v[9], v[10], v[11], v[12])


def parse_line(line):
    """Returns ('S', Sample) | ('EVT', Event) | ('CYC', Cycle) | ('BOOT', (boot, fw)) | None."""
    line = strip_host_time(line.strip())
    parts = line.split(",")
    tag = parts[0]
    try:
        if tag == "S" and len(parts) >= 7:
            s = _sample(None, [_num(p) for p in parts[1:14]])
            return ("S", s) if s else None
        if tag == "SMP" and len(parts) >= 9:
            v = [_num(p) for p in parts[1:16]]
            s = _sample(v[0], v[2:])
            return ("S", s) if s else None
        if tag == "R" and len(parts) >= 6:  # standalone recording: R,boot,t_ms,ax_mg,ay_mg,az_mg
            v = [_num(p) for p in parts[1:6]]
            s = _sample(v[0], [v[1], None, None, v[2], v[3], v[4]])
            if s:
                s.partial = True
            return ("S", s) if s else None
        if tag == "EVT" and len(parts) >= 7:
            t_ms = _num(parts[3])
            if t_ms is None:
                return None
            return "EVT", Event(_num(parts[1]), int(t_ms), parts[4], _num(parts[5]), _num(parts[6]))
        if tag == "CYC" and len(parts) >= 11:
            return "CYC", Cycle(_num(parts[1]), [_num(p) for p in parts[3:]])
        if tag == "BOOT" and len(parts) >= 5:
            return "BOOT", (_num(parts[1]), parts[4])
    except (IndexError, ValueError, TypeError):
        return None
    return None


def assign_times(samples, events):
    """Build one continuous timeline across reboots (device time restarts at 0 on every boot)."""
    pseudo, last = 0, None
    for s in samples:  # live recordings carry no boot number: a clock jump back means a reset
        if s.boot is None:
            if last is not None and s.t_ms < last - 1000:
                pseudo += 1
            last = s.t_ms
            s.boot = pseudo
    span = {}
    for r in list(samples) + list(events):
        b = r.boot if r.boot is not None else 0
        span[b] = max(span.get(b, 0), r.t_ms)
    offset, acc = {}, 0
    for b in sorted(span):
        offset[b] = acc
        acc += span[b] + 1000  # 1 s gap between boots
    for r in list(samples) + list(events):
        b = r.boot if r.boot is not None else 0
        r.t = (offset[b] + r.t_ms) / 1000.0
    samples.sort(key=lambda s: s.t)
    events.sort(key=lambda e: e.t)


def load_file(path):
    samples, events, cycles, boots = [], [], [], []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            parsed = parse_line(line)
            if not parsed:
                continue
            kind, obj = parsed
            if kind == "S":
                samples.append(obj)
            elif kind == "EVT":
                events.append(obj)
            elif kind == "CYC":
                cycles.append(obj)
            elif kind == "BOOT":
                boots.append(obj)
    assign_times(samples, events)
    fill_partial_samples(samples)
    return samples, events, cycles, boots


def fill_partial_samples(samples):
    """Recording (R) samples carry only acceleration: copy state/phase/depth/battery/temp from the
    latest full sample before them, and compute motion the same way the firmware does."""
    last_full = None
    gravity, last_t = None, None
    for s in samples:
        if not s.partial:
            last_full = s
        elif last_full is not None:
            for f in ("state", "phase", "temp", "p_raw", "depth_cm", "mv", "soc", "flags"):
                setattr(s, f, getattr(last_full, f))
        mag = math.sqrt(s.ax ** 2 + s.ay ** 2 + s.az ** 2)
        if gravity is None or last_t is None or s.t < last_t:
            gravity = mag
        else:
            gravity += (mag - gravity) * min(1.0, (s.t - last_t) / GRAVITY_TAU_S)
        last_t = s.t
        if s.partial or s.motion is None:
            s.motion = int(round(abs(mag - gravity)))


# ---------------------------------------------------------------------------------------------
# orientation
# ---------------------------------------------------------------------------------------------
def rotation_to_up(g):
    """Rotation matrix taking the measured 'up' (the gravity reaction) onto world +Z."""
    n = float(np.linalg.norm(g))
    if n < 1e-6:
        return np.eye(3)
    u = g / n
    up = np.array([0.0, 0.0, 1.0])
    v = np.cross(u, up)
    s = float(np.linalg.norm(v))
    c = float(np.dot(u, up))
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))


class Kinematics:
    """Low-passes acceleration to estimate gravity (tilt); returns the tilt and the world-frame vector."""

    def __init__(self, tau_s=0.5):
        self.tau = tau_s
        self.g = None
        self.t = None

    def update(self, t, a_mg):
        a = np.asarray(a_mg, dtype=float)
        if self.g is None or self.t is None or t < self.t or t - self.t > 5 * self.tau:
            self.g = a.copy()
        else:
            self.g += (a - self.g) * (1.0 - math.exp(-(t - self.t) / self.tau))
        self.t = t
        r = rotation_to_up(self.g)
        return r, r @ (a / 1000.0)


def describe_flags(flags):
    if flags is None:
        return "-"
    names = [name for bit, name in FLAG_NAMES if flags & bit]
    return " ".join(names) if names else "none"


def fmt(v, spec="", none="-"):
    return none if v is None else format(v, spec)


def hud_text(s: Sample, last_event: Optional[Event]):
    mag = math.sqrt(s.ax ** 2 + s.ay ** 2 + s.az ** 2) / 1000.0
    depth = "-" if s.depth_cm is None else f"{s.depth_cm / 100:.2f} m ({s.depth_cm / 30.48:.1f} ft)"
    batt = "-" if s.mv is None else f"{s.mv / 1000:.3f} V   {fmt(None if s.soc is None else s.soc / 100, '.1f')} %"
    lines = [
        f"t      {s.t:10.2f} s   boot {fmt(s.boot)}   device t {s.t_ms} ms",
        f"state  {STATE_NAMES.get(s.state, fmt(s.state)):<11} phase {PHASE_NAMES.get(s.phase, fmt(s.phase))}",
        f"accel  x {s.ax / 1000:+.3f}  y {s.ay / 1000:+.3f}  z {s.az / 1000:+.3f} g   |a| {mag:.3f} g",
        f"motion {fmt(s.motion)} mg   temp_raw {fmt(s.temp)}",
        f"depth  {depth}   p_raw {fmt(s.p_raw)}",
        f"batt   {batt}",
        f"flags  {describe_flags(s.flags)}",
    ]
    if last_event is not None:
        lines.append(f"event  {last_event.t:.1f} s  {last_event.name} ({fmt(last_event.d0)}, {fmt(last_event.d1)})")
    return "\n".join(lines)


# ---------------------------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------------------------
def run_gui(args, on_ready=None):
    import pyqtgraph as pg
    import pyqtgraph.opengl as gl
    from PyQt6 import QtCore, QtGui, QtWidgets

    def box_item():
        x, y, z = 0.45, 0.7, 0.18
        verts = np.array([[-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z],
                          [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]])
        faces = np.array([[0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
                          [2, 3, 7], [2, 7, 6], [1, 2, 6], [1, 6, 5], [0, 3, 7], [0, 7, 4]])
        colors = np.array([(0.2, 0.2, 0.5, 0.5)] * 2 + [(0.3, 0.5, 1.0, 0.6)] * 2 +   # -Z, +Z
                          [(0.3, 0.3, 0.3, 0.4)] * 2 + [(0.2, 0.9, 0.2, 0.6)] * 2 +   # -Y, +Y
                          [(1.0, 0.25, 0.25, 0.6)] * 2 + [(0.3, 0.3, 0.3, 0.4)] * 2)  # +X, -X
        return gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=colors, smooth=False,
                             drawEdges=True, edgeColor=(1, 1, 1, 0.8), glOptions="translucent")

    class Viewer(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.mode = args.mode
            self.trail_len = args.trail
            self.kin = Kinematics()
            self.ser = None
            self.rec = None
            self.rx_buf = b""
            self.live_offset = 0.0
            self.live_last_ms = None
            self.last_event = None
            self.setWindowTitle("Whale Detangler viewer")
            self.resize(1500, 900)
            self._build_ui()
            if self.mode == "play":
                self._init_play()
            else:
                self._init_live()

        # ---------------- layout ----------------
        def _build_ui(self):
            mono = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)

            self.view = gl.GLViewWidget()
            self.view.setCameraPosition(distance=4.5, elevation=22, azimuth=45)
            grid = gl.GLGridItem()
            grid.setSize(4, 4)
            grid.setSpacing(0.5, 0.5)
            grid.translate(0, 0, -0.8)
            self.view.addItem(grid)
            self.view.addItem(gl.GLAxisItem(size=QtGui.QVector3D(1.2, 1.2, 1.2)))
            self.box = box_item()
            self.view.addItem(self.box)
            self.arrow = gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=(1, 1, 0, 1), width=4, mode="lines")
            self.view.addItem(self.arrow)
            self.tip = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=0.08, color=(1, 1, 0, 1), pxMode=False)
            self.view.addItem(self.tip)
            self.trail = gl.GLLinePlotItem(pos=np.zeros((1, 3)), width=2, mode="line_strip")
            self.view.addItem(self.trail)

            self.hud = QtWidgets.QLabel("waiting for data...")
            self.hud.setFont(mono)
            self.hud.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)

            self.console = QtWidgets.QPlainTextEdit()
            self.console.setReadOnly(True)
            self.console.setMaximumBlockCount(1000)
            self.console.setFont(mono)

            left = QtWidgets.QWidget()
            lv = QtWidgets.QVBoxLayout(left)
            lv.addWidget(self.view, 5)
            lv.addWidget(self.hud, 0)
            lv.addWidget(self.console, 2)

            self.plots = pg.GraphicsLayoutWidget()
            self.p_acc = self.plots.addPlot(row=0, col=0, title="acceleration (g)")
            self.p_acc.addLegend(offset=(5, 5))
            self.c_ax = self.p_acc.plot(pen=pg.mkPen((255, 80, 80), width=1), name="x")
            self.c_ay = self.p_acc.plot(pen=pg.mkPen((80, 255, 80), width=1), name="y")
            self.c_az = self.p_acc.plot(pen=pg.mkPen((90, 140, 255), width=1), name="z")
            self.c_mag = self.p_acc.plot(pen=pg.mkPen((230, 230, 230), width=1), name="|a|")
            self.p_depth = self.plots.addPlot(row=1, col=0, title="depth (m)")
            self.p_depth.invertY(True)
            self.c_depth = self.p_depth.plot(pen=pg.mkPen((0, 200, 255), width=2))
            self.p_motion = self.plots.addPlot(row=2, col=0, title="motion (mg)")
            self.c_motion = self.p_motion.plot(pen=pg.mkPen((255, 170, 0), width=1))
            self.p_batt = self.plots.addPlot(row=3, col=0, title="battery (V)")
            self.c_batt = self.p_batt.plot(pen=pg.mkPen((200, 120, 255), width=2))
            self.all_plots = [self.p_acc, self.p_depth, self.p_motion, self.p_batt]
            for p in self.all_plots:
                p.showGrid(x=True, y=True, alpha=0.25)
                if p is not self.p_acc:
                    p.setXLink(self.p_acc)
            self.p_batt.setLabel("bottom", "time (s)")

            split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
            split.addWidget(left)
            split.addWidget(self.plots)
            split.setSizes([650, 850])

            self.bar = QtWidgets.QHBoxLayout()
            central = QtWidgets.QWidget()
            cv = QtWidgets.QVBoxLayout(central)
            cv.addWidget(split, 1)
            cv.addLayout(self.bar)
            self.setCentralWidget(central)

        def _render_3d(self, rot, tips, colors):
            m = np.eye(4)
            m[:3, :3] = rot
            self.box.setTransform(pg.Transform3D(*m.flatten()))
            tip = tips[-1]
            self.arrow.setData(pos=np.array([[0, 0, 0], tip]))
            self.tip.setData(pos=tip.reshape(1, 3))
            if len(tips) > 1:
                self.trail.setData(pos=np.asarray(tips), color=np.asarray(colors))

        # ---------------- playback ----------------
        def _init_play(self):
            self.samples, self.events, cycles, boots = load_file(args.file)
            if not self.samples:
                QtWidgets.QMessageBox.critical(self, "No samples",
                                               "No S/SMP lines in this file. FRAM only stores samples while "
                                               "underwater unless 'logsurface on' was set.")
                raise SystemExit(1)
            self.times = [s.t for s in self.samples]
            kin = Kinematics()
            self.rots, self.tips = [], []
            for s in self.samples:  # precompute the 3D state of every sample once
                r, w = kin.update(s.t, (s.ax, s.ay, s.az))
                self.rots.append(r)
                self.tips.append(w)
            self.tip_colors = [STATE_COLORS.get(s.state, (0.3, 0.9, 1.0, 1)) for s in self.samples]
            self.event_times = [e.t for e in self.events]

            t = np.array(self.times)
            col = lambda attr: np.array([getattr(s, attr) for s in self.samples], dtype=float)
            ax, ay, az = col("ax") / 1000, col("ay") / 1000, col("az") / 1000
            self.c_ax.setData(t, ax)
            self.c_ay.setData(t, ay)
            self.c_az.setData(t, az)
            self.c_mag.setData(t, np.sqrt(ax ** 2 + ay ** 2 + az ** 2))
            depth = np.array([np.nan if s.depth_cm is None else s.depth_cm / 100 for s in self.samples])
            self.c_depth.setData(t, depth, connect="finite")
            motion = np.array([np.nan if s.motion is None else s.motion for s in self.samples])
            self.c_motion.setData(t, motion, connect="finite")
            batt = np.array([np.nan if s.mv is None or s.mv == 0 else s.mv / 1000 for s in self.samples])
            self.c_batt.setData(t, batt, connect="finite")
            if np.all(np.isnan(batt)):
                self.p_batt.setTitle("battery (V)  (not in this file)")
            if np.all(np.isnan(depth)):
                self.p_depth.setTitle("depth (m)  (no calibrated depth in this file)")
            self._shade_phases()
            for e in self.events:
                if e.name == "PHASE_CHANGE":
                    continue  # already visible as shading
                line = pg.InfiniteLine(pos=e.t, angle=90, movable=False,
                                       pen=pg.mkPen((255, 170, 0, 160), style=QtCore.Qt.PenStyle.DashLine),
                                       label=e.name, labelOpts={"position": 0.9, "color": (255, 190, 60)})
                self.p_depth.addItem(line)
            self.cursors = []
            for p in self.all_plots:
                c = pg.InfiniteLine(pos=self.times[0], angle=90, movable=False, pen=pg.mkPen("y", width=1))
                p.addItem(c)
                self.cursors.append(c)

            self.console.appendPlainText(
                f"{args.file}: {len(self.samples)} samples, {len(self.events)} events, "
                f"{len(cycles)} cycle summaries, {len(boots)} boot records, span {self.times[-1] - self.times[0]:.1f} s")
            for c in cycles:
                f = c.fields + [None] * 16
                self.console.appendPlainText(
                    f"cycle {f[0]}: bottom {fmt(None if f[3] is None else f[3] / 100, '.2f')} m, "
                    f"soak {fmt(None if f[6] is None else f[6] / 3.6e6, '.2f')} h, "
                    f"drop {fmt(None if f[7] is None else f[7] / 1000, '.0f')} s (peak {fmt(f[9])} mg), "
                    f"haul {fmt(None if f[11] is None else f[11] / 1000, '.0f')} s (peak {fmt(f[13])} mg)")
            for e in self.events:
                self.console.appendPlainText(f"{e.t:10.1f} s  {e.name} ({fmt(e.d0)}, {fmt(e.d1)})")

            self.play_btn = QtWidgets.QPushButton("Play")
            self.play_btn.clicked.connect(self._toggle_play)
            self.speed = QtWidgets.QComboBox()
            for sp in ["0.25", "0.5", "1", "2", "5", "10", "30", "100", "300", "1000"]:
                self.speed.addItem(sp + "x", float(sp))
            self.speed.setCurrentIndex(2)
            self.slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
            self.slider.setRange(0, len(self.samples) - 1)
            self.slider.valueChanged.connect(self._slider_moved)
            self.time_lbl = QtWidgets.QLabel()
            self.bar.addWidget(self.play_btn)
            self.bar.addWidget(QtWidgets.QLabel("speed"))
            self.bar.addWidget(self.speed)
            self.bar.addWidget(self.slider, 1)
            self.bar.addWidget(self.time_lbl)
            QtGui.QShortcut(QtGui.QKeySequence("Space"), self, self._toggle_play)

            self.playing = False
            self.play_t = self.times[0]
            self.idx = -1
            self.wall = time.monotonic()
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self._play_tick)
            self.timer.start(33)
            self._show_index(0)

        def _shade_phases(self):
            start, phase = None, None
            segments = []
            for s in self.samples:
                if s.phase != phase:
                    if phase in PHASE_BRUSHES and start is not None:
                        segments.append((start, s.t, phase))
                    start, phase = s.t, s.phase
            if phase in PHASE_BRUSHES and start is not None:
                segments.append((start, self.samples[-1].t, phase))
            for t0, t1, ph in segments[:2000]:
                region = pg.LinearRegionItem(values=(t0, t1), movable=False, brush=pg.mkBrush(*PHASE_BRUSHES[ph]))
                region.setZValue(-10)
                for line in region.lines:
                    line.setPen(pg.mkPen(None))
                self.p_depth.addItem(region)

        def _toggle_play(self):
            self.playing = not self.playing
            if self.playing and self.idx >= len(self.samples) - 1:
                self.play_t = self.times[0]
            self.play_btn.setText("Pause" if self.playing else "Play")
            self.wall = time.monotonic()

        def _slider_moved(self, value):
            if value != self.idx:
                self.play_t = self.times[value]
                self._show_index(value)

        def _play_tick(self):
            now = time.monotonic()
            dt, self.wall = now - self.wall, now
            if not self.playing:
                return
            self.play_t += dt * self.speed.currentData()
            idx = min(bisect.bisect_right(self.times, self.play_t) - 1, len(self.samples) - 1)
            if idx >= len(self.samples) - 1:
                self._toggle_play()
            if idx != self.idx and idx >= 0:
                self._show_index(idx)

        def _show_index(self, idx):
            self.idx = idx
            s = self.samples[idx]
            lo = max(0, idx - self.trail_len + 1)
            self._render_3d(self.rots[idx], self.tips[lo:idx + 1], self.tip_colors[lo:idx + 1])
            e_idx = bisect.bisect_right(self.event_times, s.t) - 1
            self.hud.setText(hud_text(s, self.events[e_idx] if e_idx >= 0 else None))
            for c in self.cursors:
                c.setPos(s.t)
            self.time_lbl.setText(f"{s.t:.1f} / {self.times[-1]:.1f} s")
            self.slider.blockSignals(True)
            self.slider.setValue(idx)
            self.slider.blockSignals(False)

        # ---------------- live ----------------
        def _init_live(self):
            import serial

            port = pick_port(args.port)
            if not port:
                QtWidgets.QMessageBox.critical(self, "No serial port", "Connect the board and pass --port COMx")
                raise SystemExit(1)
            try:
                self.ser = serial.serial_for_url(port, args.baud, timeout=0)  # also accepts loop:// for tests
            except serial.SerialException as exc:
                QtWidgets.QMessageBox.critical(self, "Serial port", f"{exc}\n\nClose the Arduino Serial Monitor first.")
                raise SystemExit(1)
            self.console.appendPlainText(f"connected to {port} @ {args.baud} (the board resets on connect)")

            n = args.window * 50
            self.buf = {k: deque(maxlen=n) for k in ("t", "ax", "ay", "az", "mag", "depth", "motion", "batt")}
            self.live_tips = deque(maxlen=self.trail_len)
            self.live_colors = deque(maxlen=self.trail_len)
            self.latest = None
            self.dirty = False

            self.rec_btn = QtWidgets.QPushButton("Record")
            self.rec_btn.setCheckable(True)
            self.rec_btn.toggled.connect(self._toggle_record)
            self.rec_lbl = QtWidgets.QLabel("not recording")
            self.cmd = QtWidgets.QLineEdit()
            self.cmd.setPlaceholderText("console command (info, live on, logsurface on, dump ...) then Enter")
            self.cmd.returnPressed.connect(self._send_cmd)
            self.bar.addWidget(self.rec_btn)
            self.bar.addWidget(self.rec_lbl)
            self.bar.addWidget(self.cmd, 1)
            if args.record:
                self.rec_btn.setChecked(True)

            if not args.no_live:
                QtCore.QTimer.singleShot(2500, lambda: self._write("live on"))
            QtCore.QTimer.singleShot(6000, self._warn_if_no_data)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self._live_tick)
            self.timer.start(20)

        def _warn_if_no_data(self):
            if self.latest is None and not args.no_live and not getattr(self, "_retried", False):
                self._retried = True  #the first command can land while the board is still in setup()
                self._write("live on")
                QtCore.QTimer.singleShot(3000, self._warn_if_no_data)
                return
            if self.latest is None:
                msg = ("no 'S,' data lines received yet. Text above means the board is talking but not "
                       "streaming: upload the CURRENT whale-detangler-2.ino or bringup/accel_spi (older "
                       "versions do not support 'live on'), then reconnect. Nothing above: check the port/baud.")
                self.console.appendPlainText(msg)
                self.hud.setText("no data lines yet: see the console below")

        def _write(self, text):
            if self.ser:
                self.ser.write((text + "\n").encode())
                self.console.appendPlainText(f"> {text}")

        def _send_cmd(self):
            text = self.cmd.text().strip()
            if text:
                self._write(text)
            self.cmd.clear()

        def _toggle_record(self, on):
            if on:
                path = args.record or datetime.now().strftime("recording_%Y%m%d_%H%M%S.csv")
                args.record = None  # a later toggle creates a new timestamped file
                self.rec = open(path, "w", buffering=1)
                self.rec_t0 = time.time()
                self.rec_lbl.setText(f"recording -> {path}")
            elif self.rec:
                self.rec.close()
                self.rec = None
                self.rec_lbl.setText("not recording")

        def _live_tick(self):
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception as exc:  # unplugged
                self.console.appendPlainText(f"serial error: {exc}")
                self.timer.stop()
                return
            self.rx_buf += data
            while b"\n" in self.rx_buf:
                raw, self.rx_buf = self.rx_buf.split(b"\n", 1)
                self._handle_line(raw.decode(errors="replace").rstrip("\r"))
            if self.dirty:
                self._render_live()
                self.dirty = False

        def _handle_line(self, line):
            if self.rec:
                self.rec.write(f"{time.time() - self.rec_t0:.3f},{line}\n")
            parsed = parse_line(line)
            if not parsed:
                if line.strip():
                    self.console.appendPlainText(line)
                return
            kind, obj = parsed
            if kind == "EVT":
                self.console.appendPlainText(line)
                return
            if kind != "S":
                self.console.appendPlainText(line)
                return
            s = obj
            if self.live_last_ms is not None and s.t_ms < self.live_last_ms - 1000:
                self.live_offset += self.live_last_ms / 1000.0 + 1.0  # board reset
            self.live_last_ms = s.t_ms
            s.t = self.live_offset + s.t_ms / 1000.0
            if self.latest is not None and s.t < self.latest.t:
                return  # out-of-order duplicate (debug line and live line from the same moment)
            rot, tip = self.kin.update(s.t, (s.ax, s.ay, s.az))
            self.rot = rot
            self.live_tips.append(tip)
            self.live_colors.append(STATE_COLORS.get(s.state, (0.3, 0.9, 1.0, 1)))
            b = self.buf
            b["t"].append(s.t)
            b["ax"].append(s.ax / 1000)
            b["ay"].append(s.ay / 1000)
            b["az"].append(s.az / 1000)
            b["mag"].append(math.sqrt(s.ax ** 2 + s.ay ** 2 + s.az ** 2) / 1000)
            b["depth"].append(np.nan if s.depth_cm is None else s.depth_cm / 100)
            b["motion"].append(np.nan if s.motion is None else s.motion)
            b["batt"].append(np.nan if not s.mv else s.mv / 1000)
            self.latest = s
            self.dirty = True

        def _render_live(self):
            s = self.latest
            self._render_3d(self.rot, list(self.live_tips), list(self.live_colors))
            self.hud.setText(hud_text(s, None))
            b = {k: np.array(v, dtype=float) for k, v in self.buf.items()}
            t = b["t"]
            self.c_ax.setData(t, b["ax"])
            self.c_ay.setData(t, b["ay"])
            self.c_az.setData(t, b["az"])
            self.c_mag.setData(t, b["mag"])
            self.c_depth.setData(t, b["depth"], connect="finite")
            self.c_motion.setData(t, b["motion"], connect="finite")
            self.c_batt.setData(t, b["batt"], connect="finite")
            self.p_acc.setXRange(max(t[0], s.t - args.window), s.t, padding=0)
            #say why a graph is empty instead of leaving it blank (e.g. accel_spi sends no battery/depth)
            no_batt = "  (not reported: run the main firmware)"
            no_depth = "  (no calibrated depth: main firmware + 'cal zero' / 'cal span')"
            self.p_batt.setTitle("battery (V)" + (no_batt if np.all(np.isnan(b["batt"])) else ""))
            self.p_depth.setTitle("depth (m)" + (no_depth if np.all(np.isnan(b["depth"])) else ""))

        def closeEvent(self, event):
            if self.ser:
                try:
                    self.ser.write(b"live off\n")
                    self.ser.close()
                except Exception:
                    pass
            if self.rec:
                self.rec.close()
            super().closeEvent(event)

    app = QtWidgets.QApplication(sys.argv)
    pg.setConfigOptions(antialias=True)
    win = Viewer()
    win.show()
    if on_ready:  # test hook
        on_ready(win, app)
    return app.exec()


def pick_port(requested=None):
    """The requested port, else the USB serial adapter (FTDI VID 0403) rather than Bluetooth ports."""
    import serial.tools.list_ports

    if requested:
        return requested
    ports = sorted(serial.tools.list_ports.comports(),
                   key=lambda p: (p.vid != 0x0403, "USB" not in (p.hwid or "")))
    return ports[0].device if ports else None


def fetch_dump(args):
    """Send 'dump' and save every line (host-time prefixed, like serial_capture.py) until END."""
    import serial

    port = pick_port(args.port)
    if not port:
        print("no serial port found: connect the board (close the Arduino Serial Monitor first)")
        return None
    path = args.output or datetime.now().strftime("fram_%Y%m%d_%H%M%S.csv")
    try:
        ser = serial.serial_for_url(port, args.baud, timeout=0.5)
    except serial.SerialException as exc:
        print(f"cannot open {port}: {exc}\nclose the viewer / Arduino Serial Monitor first")
        return None
    print(f"connected to {port}; waiting for the board to boot...")
    time.sleep(3.0)  # opening the port resets the Pro Mini
    ser.reset_input_buffer()
    ser.write(b"dump\n")
    t0 = time.time()
    last_rx = time.time()
    n = 0
    ended = False
    with open(path, "w") as out:
        while time.time() - last_rx < 20:  # the board goes quiet only if something is wrong
            raw = ser.readline()
            if not raw:
                continue
            last_rx = time.time()
            line = raw.decode(errors="replace").rstrip("\r\n")
            out.write(f"{time.time() - t0:.3f},{line}\n")
            n += 1
            if n % 2000 == 0:
                print(f"  {n} lines, {time.time() - t0:.0f} s...")
            if line == "END":
                ended = True
                break
            if line.startswith("ERR"):
                print("board says:", line)
                break
    ser.close()
    print(f"saved {n} lines to {path}" + ("" if ended else "  (no END received: dump may be incomplete)"))
    return path if n else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    lv = sub.add_parser("live", help="connect to the board")
    lv.add_argument("--port", help="e.g. COM5 (default: first serial port found)")
    lv.add_argument("--baud", type=int, default=38400)
    lv.add_argument("--record", help="start recording to this CSV immediately")
    lv.add_argument("--no-live", action="store_true", help="do not send 'live on' (use the firmware's own output)")
    lv.add_argument("--window", type=int, default=30, help="seconds shown in the live graphs")
    lv.add_argument("--trail", type=int, default=150, help="points in the 3D trail")
    fe = sub.add_parser("fetch", help="download the board's FRAM log/recording, save it, then play it")
    fe.add_argument("--port", help="e.g. COM6 (default: the USB serial adapter)")
    fe.add_argument("--baud", type=int, default=38400)
    fe.add_argument("-o", "--output", help="file to save (default: fram_<date>_<time>.csv)")
    fe.add_argument("--no-play", action="store_true", help="only download")
    fe.add_argument("--trail", type=int, default=150, help="points in the 3D trail")
    pl = sub.add_parser("play", help="play back a recording or a FRAM dump capture")
    pl.add_argument("file")
    pl.add_argument("--trail", type=int, default=150, help="points in the 3D trail")
    args = ap.parse_args()
    if args.mode == "fetch":
        path = fetch_dump(args)
        if not path or args.no_play:
            sys.exit(0 if path else 1)
        args = argparse.Namespace(mode="play", file=path, trail=args.trail)
    sys.exit(run_gui(args))


if __name__ == "__main__":
    main()
