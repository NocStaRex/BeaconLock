"""
ui/plots_panel.py
=================
Real-time PyQtGraph telemetry plot panel for BeaconLock.

Three stacked plots:
  1. Tracking Error (px)   — rolling curve + red ISRO 10 px threshold line.
  2. Slew Velocity (°/s)   — pan (cyan) + tilt (purple) + ±5.0 °/s clamp lines.
  3. FPS gauge             — rolling FPS EMA + 30-fps floor & 40-fps target lines.

Theme: Deep Aerospace Obsidian (#0d1322) matching the telemetry cockpit.
Labels: Non-clipping horizontal positions with explicit anchors.
Data is stored in fixed-length deques (WINDOW frames) for O(1) updates.

Slot update_plots(dict) is connected to TrackingWorker.metrics_ready.

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

from collections import deque
from typing import Optional

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, Slot
from PySide6.QtWidgets import QLabel, QSizePolicy, QVBoxLayout, QWidget

# ── Plot configuration ────────────────────────────────────────────────────
WINDOW       = 300        # rolling data window (frames)
BG_COLOR     = "#0d1322"  # strictly matching console theme
FG_COLOR     = "#94a3b8"  # slate-400
GRID_COLOR   = "#1e293b"  # subtle slate-800 grid
LABEL_COLOR  = "#f8fafc"  # crisp slate-50

# Configure global PyQtGraph defaults before any PlotWidget is created
pg.setConfigOptions(
    background=BG_COLOR,
    foreground=FG_COLOR,
    antialias=True,
    useOpenGL=False,      # CPU-only rendering
)


class PlotsPanel(QWidget):
    """
    PyQtGraph real-time telemetry panel.

    Slots
    -----
    update_plots(metrics: dict)  — called from main thread on metrics_ready signal.
    reset()                      — clears all rolling buffers.
    """

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumWidth(320)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # Rolling data buffers
        self._x      = deque(maxlen=WINDOW)   # frame index
        self._err    = deque(maxlen=WINDOW)   # tracking error (px)
        self._pan    = deque(maxlen=WINDOW)   # pan rate (°/s)
        self._tilt   = deque(maxlen=WINDOW)   # tilt rate (°/s)
        self._fps    = deque(maxlen=WINDOW)   # FPS EMA
        self._frame  = 0

        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(6)

        # ── Plot 1: Tracking Error ─────────────────────────────────
        self.p_err = pg.PlotWidget()
        self._style_plot(self.p_err, "Tracking Error", "error (px)", y_range=(0, 40))
        self.c_err = self.p_err.plot(
            pen=pg.mkPen("#38bdf8", width=2),  # bright cyan
            name="Error",
        )
        # ISRO 10 px threshold line — anchored inside the plot view to prevent right-edge clipping
        self.p_err.addItem(pg.InfiniteLine(
            pos=10, angle=0,
            pen=pg.mkPen("#ef4444", width=1.5, style=Qt.PenStyle.DashLine),
            label="ISRO 10 px",
            labelOpts={
                "color": "#ef4444",
                "position": 0.80,
                "anchors": [(1.0, 1.2), (1.0, -0.2)],
            },
        ))
        layout.addWidget(self.p_err)

        # ── Plot 2: Slew Velocity ──────────────────────────────────
        self.p_slew = pg.PlotWidget()
        self._style_plot(self.p_slew, "Slew Velocity", "rate (°/s)", y_range=(-6, 6))
        self.c_pan  = self.p_slew.plot(pen=pg.mkPen("#f59e0b", width=2), name="Pan")
        self.c_tilt = self.p_slew.plot(pen=pg.mkPen("#a78bfa", width=2), name="Tilt")
        # ±5 °/s clamp lines
        for pos in (5.0, -5.0):
            self.p_slew.addItem(pg.InfiniteLine(
                pos=pos, angle=0,
                pen=pg.mkPen("#ef4444", width=1.2, style=Qt.PenStyle.DashLine),
            ))
        layout.addWidget(self.p_slew)

        # ── Plot 3: FPS ────────────────────────────────────────────
        self.p_fps = pg.PlotWidget()
        self._style_plot(self.p_fps, "Throughput", "FPS", y_range=(0, 200))
        self.c_fps = self.p_fps.plot(pen=pg.mkPen("#4ade80", width=2), name="FPS")
        # 30 FPS mandatory floor — placed at position 0.85
        self.p_fps.addItem(pg.InfiniteLine(
            pos=30, angle=0,
            pen=pg.mkPen("#f59e0b", width=1.5, style=Qt.PenStyle.DashLine),
            label="30 FPS min",
            labelOpts={
                "color": "#f59e0b",
                "position": 0.85,
                "anchors": [(1.0, 1.2), (1.0, -0.2)],
            },
        ))
        # 40 FPS target line — placed at position 0.55 to avoid overlapping 30 FPS label
        self.p_fps.addItem(pg.InfiniteLine(
            pos=40, angle=0,
            pen=pg.mkPen("#22c55e", width=1.5, style=Qt.PenStyle.DotLine),
            label="40 FPS target",
            labelOpts={
                "color": "#22c55e",
                "position": 0.55,
                "anchors": [(1.0, -0.2), (1.0, 1.2)],
            },
        ))
        layout.addWidget(self.p_fps)

    @staticmethod
    def _style_plot(
        plot: pg.PlotWidget,
        title: str,
        y_label: str,
        y_range: tuple[float, float] = (0, 100),
    ) -> None:
        """Apply consistent deep aerospace obsidian styling to a PlotWidget."""
        plot.setBackground(BG_COLOR)
        plot.setTitle(title, color=LABEL_COLOR, size="10pt")
        plot.setLabel("left", y_label, color=FG_COLOR)
        plot.showGrid(x=False, y=True, alpha=0.35)
        plot.setYRange(*y_range, padding=0.05)
        plot.setMenuEnabled(False)
        plot.setMouseEnabled(x=False, y=False)

        left_axis = plot.getPlotItem().getAxis("left")
        left_axis.setTextPen(FG_COLOR)
        left_axis.setPen(pg.mkPen(GRID_COLOR))
        left_axis.setWidth(42)  # reserve fixed width so tick labels never clip

        bottom_axis = plot.getPlotItem().getAxis("bottom")
        bottom_axis.setTextPen(FG_COLOR)
        bottom_axis.setPen(pg.mkPen(GRID_COLOR))
        bottom_axis.setStyle(showValues=False)

        plot.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding,
        )

    # ------------------------------------------------------------------
    # Slot — called from main thread via queued signal
    # ------------------------------------------------------------------

    @Slot(object)
    def update_plots(self, metrics: dict) -> None:
        """
        Receive a metrics dict from TrackingWorker and update all curves.

        Thread context: MAIN thread (Qt signal/slot queuing guarantees this).
        """
        self._frame += 1
        self._x.append(self._frame)
        self._err.append(float(metrics.get("error_px", 0.0)))
        self._pan.append(float(metrics.get("pan_deg_s", 0.0)))
        self._tilt.append(float(metrics.get("tilt_deg_s", 0.0)))
        self._fps.append(float(metrics.get("fps", 0.0)))

        xs = list(self._x)
        self.c_err.setData(xs, list(self._err))
        self.c_pan.setData(xs, list(self._pan))
        self.c_tilt.setData(xs, list(self._tilt))
        self.c_fps.setData(xs, list(self._fps))

    @Slot()
    def reset(self) -> None:
        """Clear all rolling buffers (connected to RESET button)."""
        self._x.clear()
        self._err.clear()
        self._pan.clear()
        self._tilt.clear()
        self._fps.clear()
        self._frame = 0
        self.c_err.setData([], [])
        self.c_pan.setData([], [])
        self.c_tilt.setData([], [])
        self.c_fps.setData([], [])
