#!/usr/bin/env python3
"""
Realtime 3D ADXL visualizer for whale-detangler alpha demo.

Reads serial lines in format:
  PLOT,<t_ms>,<x>,<y>,<z>,<state_code>,<activity_flag>

State codes are the firmware's DeviceState (types.h):
  0 = BOOT, 1 = SAFE_IDLE, 2 = MONITORING (green)
  3 = CHARGING (yellow)
  4 = FIRED (red)
  5 = FAULT (magenta)
"""

import argparse
import sys
from collections import deque

import numpy as np
import serial
import serial.tools.list_ports
from PyQt6 import QtCore, QtWidgets
import pyqtgraph.opengl as gl


def pick_default_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    # Prefer common Arduino-like device names.
    preferred_tokens = ("usbmodem", "usbserial", "ttyACM", "ttyUSB", "wchusb")
    for p in ports:
        name = (p.device or "").lower()
        if any(tok in name for tok in preferred_tokens):
            return p.device
    return ports[0].device


STATE_CHARGING = 3
STATE_FIRED = 4
STATE_FAULT = 5


def state_color(state_code):
    if state_code == STATE_FIRED:
        return (1.0, 0.1, 0.1, 1.0)
    if state_code == STATE_CHARGING:
        return (1.0, 1.0, 0.1, 1.0)
    if state_code == STATE_FAULT:
        return (1.0, 0.1, 1.0, 1.0)
    return (0.1, 1.0, 0.1, 1.0)  # BOOT / SAFE_IDLE / MONITORING


class Plot3D:
    def __init__(self, ser, trail_len):
        self.ser = ser
        self.trail = deque(maxlen=trail_len)
        self.latest_state = 0

        self.app = QtWidgets.QApplication([])
        self.view = gl.GLViewWidget()
        self.view.setWindowTitle("Whale Detangler - Realtime ADXL 3D")
        self.view.resize(1100, 800)
        self.view.setCameraPosition(distance=900, elevation=20, azimuth=35)

        # Axes and floor grid.
        grid = gl.GLGridItem()
        grid.scale(50, 50, 1)
        grid.setSize(x=20, y=20, z=1)
        self.view.addItem(grid)

        axis = gl.GLAxisItem(size=QtGuiVector3(300, 300, 300))
        self.view.addItem(axis)

        # Current sample point.
        self.point = gl.GLScatterPlotItem(
            pos=np.array([[0.0, 0.0, 0.0]], dtype=float),
            size=12.0,
            color=np.array([[0.1, 1.0, 0.1, 1.0]], dtype=float),
            pxMode=False,
        )
        self.view.addItem(self.point)

        # Trail path through recent samples.
        self.path = gl.GLLinePlotItem(
            pos=np.zeros((1, 3), dtype=float),
            color=(0.6, 0.8, 1.0, 0.6),
            width=2.0,
            antialias=True,
            mode="line_strip",
        )
        self.view.addItem(self.path)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_from_serial)
        self.timer.start(30)

        self.view.show()

    def parse_plot_line(self, line):
        if not line.startswith("PLOT,"):
            return None
        parts = line.strip().split(",")
        if len(parts) != 7:
            return None
        try:
            # parts[1] is timestamp; currently unused for plotting
            x = float(parts[2])
            y = float(parts[3])
            z = float(parts[4])
            state = int(parts[5])
            return x, y, z, state
        except ValueError:
            return None

    def update_from_serial(self):
        updated = False
        # Drain available lines for smooth realtime display.
        while self.ser.in_waiting:
            raw = self.ser.readline()
            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue

            parsed = self.parse_plot_line(line)
            if parsed is None:
                continue

            x, y, z, state = parsed
            self.latest_state = state
            self.trail.append((x, y, z))
            updated = True

        if not updated or not self.trail:
            return

        points = np.array(self.trail, dtype=float)
        current = points[-1].reshape(1, 3)
        color = np.array([state_color(self.latest_state)], dtype=float)

        self.point.setData(pos=current, color=color)
        self.path.setData(pos=points)


def QtGuiVector3(x, y, z):
    # Local helper avoids importing QtGui directly for one type.
    from PyQt6.QtGui import QVector3D

    return QVector3D(float(x), float(y), float(z))


def main():
    parser = argparse.ArgumentParser(description="Realtime 3D ADXL serial plot")
    parser.add_argument("--port", type=str, default=None, help="Serial port, e.g. /dev/tty.usbmodem14101")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--trail", type=int, default=120, help="Number of recent points in trail")
    args = parser.parse_args()

    port = args.port or pick_default_port()
    if not port:
        print("No serial ports found. Connect board and retry with --port.")
        sys.exit(1)

    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
    except Exception as exc:
        print(f"Failed to open serial port {port}: {exc}")
        sys.exit(1)

    print(f"Connected to {port} @ {args.baud}")
    print("Expecting lines: PLOT,<t_ms>,<x>,<y>,<z>,<state_code>,<activity_flag>")
    print("State color: monitoring=green, charging=yellow, fired=red, fault=magenta")

    plotter = Plot3D(ser=ser, trail_len=max(10, args.trail))
    sys.exit(plotter.app.exec())


if __name__ == "__main__":
    main()
