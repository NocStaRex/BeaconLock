"""
ui/plots_panel.py
=================
Real-time PyQtGraph telemetry plot panel for BeaconLock.

Three stacked plots:
  1. Tracking Error (px)   — rolling curve + red ISRO 10 px threshold line.
  2. Slew Velocity (°/s)   — pan (cyan) + tilt (purple) + ±5.0 °/s clamp lines.
  3. FPS gauge             — rolling FPS EMA + 30-fps mandatory floor line.

All plots use a dark aerospace theme matching the main dashboard.
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
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QLabel, QSizePolicy, QVBoxLayout, QWidget

# ── Plot configuration ────────────────────────────────────────────────────
WINDOW       = 300      # rolling data window (frames)
BG_COLOR     = "#0f172a"
FG_COLOR     = "#94a3b8"
GRID_COLOR   = "#1e293b"
LABEL_COLOR  = "#e2e8f0"

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
        self.setMinimumWidth(310)
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
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        # ── Panel title ────────────────────────────────────────────
        title = QLabel("TELEMETRY")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet(
            "color: #94a3b8; font: bold 10px 'Consolas'; letter-spacing: 3px;"
        )
        layout.addWidget(title)

        # ── Plot 1: Tracking Error ─────────────────────────────────
        self.p_err = pg.PlotWidget()
        self._style_plot(self.p_err, "Tracking Error", "error (px)", y_range=(0, 40))
        self.c_err = self.p_err.plot(
            pen=pg.mkPen("#22d3ee", width=2),
            name="Error",
        )
        # ISRO 10 px threshold line
        self.p_err.addItem(pg.InfiniteLine(
            pos=10, angle=0,
            pen=pg.mkPen("#ef4444", width=1, style=Qt.PenStyle.DashLine),
            label="ISRO 10 px", labelOpts={"color": "#ef4444", "position": 0.95},
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
                pen=pg.mkPen("#ef4444", width=1, style=Qt.PenStyle.DashLine),
            ))
        layout.addWidget(self.p_slew)

        # ── Plot 3: FPS ────────────────────────────────────────────
        self.p_fps = pg.PlotWidget()
        self._style_plot(self.p_fps, "Throughput", "FPS", y_range=(0, 200))
        self.c_fps = self.p_fps.plot(pen=pg.mkPen("#4ade80", width=2), name="FPS")
        # 30 FPS mandatory floor
        self.p_fps.addItem(pg.InfiniteLine(
            pos=30, angle=0,
            pen=pg.mkPen("#f59e0b", width=1, style=Qt.PenStyle.DashLine),
            label="30 FPS min", labelOpts={"color": "#f59e0b", "position": 0.95},
        ))
        # 40 FPS target line
        self.p_fps.addItem(pg.InfiniteLine(
            pos=40, angle=0,
            pen=pg.mkPen("#22c55e", width=1, style=Qt.PenStyle.DotLine),
            label="40 FPS target", labelOpts={"color": "#22c55e", "position": 0.75},
        ))
        layout.addWidget(self.p_fps)

    @staticmethod
    def _style_plot(
        plot: pg.PlotWidget,
        title: str,
        y_label: str,
        y_range: tuple[float, float] = (0, 100),
    ) -> None:
        """Apply consistent dark-theme styling to a PlotWidget."""
        plot.setTitle(title, color=LABEL_COLOR, size="10pt")
        plot.setLabel("left", y_label, color=FG_COLOR)
        plot.showGrid(x=False, y=True, alpha=0.25)
        plot.setYRange(*y_range, padding=0.05)
        plot.setMenuEnabled(False)
        plot.setMouseEnabled(x=False, y=False)
        plot.getPlotItem().getAxis("left").setTextPen(FG_COLOR)
        plot.getPlotItem().getAxis("bottom").setTextPen(FG_COLOR)
        # Remove x-axis numbers (rolling window — absolute frame index not meaningful)
        plot.getPlotItem().getAxis("bottom").setStyle(showValues=False)
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
