"""
ui/dashboard.py
================
BeaconLock native desktop telemetry cockpit (PySide6 + PyQtGraph).

Layout (3-panel horizontal split)
----------------------------------
┌──────────────────────────────────────────────────────────────────────┐
│  LeftPanel (Scrollable 264px) │ CenterPanel (640px) │ RightPanel     │
│  Controls                     │ 640×480 Camera View │ Telemetry Plots│
├───────────────────────────────┼─────────────────────┼────────────────┤
│  Source toggle                │ Video feed label    │ Error curve    │
│  Motion selector              │ Crosshair overlay   │ Slew curve     │
│  Disturbance sliders          │ State / error HUD   │ FPS gauge      │
│  Action buttons (32px stack)  │                     │                │
│  State badge + FPS row        │                     │                │
└───────────────────────────────┴─────────────────────┴────────────────┘

Threading contract
------------------
All vision / control processing runs inside TrackingWorker (QThread).
The dashboard ONLY reads Qt signals and updates widgets — it NEVER calls
FrameSource, CentroidEstimator, KalmanTracker, or PDController directly.

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import logging
import math
import os
import time
from collections import deque
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from PySide6.QtCore import Qt, Slot
from PySide6.QtGui import QImage, QPixmap, QIcon
from PySide6.QtWidgets import (
    QButtonGroup, QComboBox, QFileDialog, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QRadioButton,
    QScrollArea, QSizePolicy, QSlider, QVBoxLayout, QWidget, QMainWindow,
    QMessageBox, QStatusBar, QFrame,
)

from app.orchestrator import TrackingWorker
from app.state_machine import TrackingState
from evaluation.logger import FrameLogger
from evaluation.report_generator import ReportGenerator
from ingestion.frame_source import FrameSource
from ingestion.sim_source import SimulatorSource
from simulation.disturbances import DisturbanceConfig
from ui.plots_panel import PlotsPanel

logger = logging.getLogger(__name__)

# ─────────────────────────────────────────────────────────────────────────────
# Cohesive Modern Aerospace Obsidian Theme
# Palette:
#   Main Window Background: #0b0f19 (Deep Aerospace Obsidian)
#   Panel Surfaces:        #111827 (Seamless slate)
#   Panel Borders:         1px solid #1f2937, 8px rounded corners
#   Primary Text:          #f8fafc (crisp slate-50)
#   Secondary Hints:       #94a3b8 (slate-400)
#   Inputs / Combos:       #1e293b with #334155 border
#   Accent:                #0284c7 (Vibrant Sky) / #38bdf8
# ─────────────────────────────────────────────────────────────────────────────

AEROSPACE_THEME_STYLESHEET = """
/* ── Root Window & Base Widgets ────────────────────────────────────── */
QMainWindow, QWidget {
    background-color: #0b0f19;
    color: #f8fafc;
    font-family: "Segoe UI", "Inter", -apple-system, sans-serif;
    font-size: 12px;
}

/* ── Panel Surfaces (QGroupBox) ────────────────────────────────────── */
QGroupBox {
    background-color: #111827;
    border: 1px solid #1f2937;
    border-radius: 8px;
    margin-top: 13px;
    padding: 8px 6px 6px 6px;
    font-size: 10px;
    font-weight: bold;
    letter-spacing: 1.5px;
    color: #94a3b8;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 8px;
    top: 1px;
    padding: 0 5px;
    background-color: #111827;
    border-radius: 3px;
    color: #38bdf8;
}

/* ── Radio Buttons ─────────────────────────────────────────────────── */
QRadioButton {
    color: #e2e8f0;
    spacing: 6px;
    font-size: 11px;
    padding: 1px 0;
}
QRadioButton::indicator {
    width: 14px;
    height: 14px;
}
QRadioButton::indicator:unchecked {
    border: 2px solid #475569;
    border-radius: 7px;
    background: #1e293b;
}
QRadioButton::indicator:checked {
    background: #0284c7;
    border: 2px solid #38bdf8;
    border-radius: 7px;
}

/* ── Dropdowns (QComboBox) ─────────────────────────────────────────── */
QComboBox {
    background-color: #1e293b;
    border: 1px solid #334155;
    border-radius: 5px;
    padding: 3px 8px;
    min-height: 30px;
    max-height: 30px;
    color: #f8fafc;
    font-size: 12px;
    font-weight: 600;
}
QComboBox:hover {
    border-color: #0284c7;
}
QComboBox:focus {
    border: 1px solid #38bdf8;
}
QComboBox::drop-down {
    border: none;
    width: 20px;
}
QComboBox::down-arrow {
    image: none;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-top: 5px solid #94a3b8;
    width: 0;
    height: 0;
    margin-right: 6px;
}
QComboBox QAbstractItemView {
    background-color: #1e293b;
    border: 1px solid #334155;
    color: #f8fafc;
    font-size: 12px;
    font-weight: 600;
    selection-background-color: #0284c7;
    selection-color: #ffffff;
    padding: 4px;
}

/* ── Sliders ───────────────────────────────────────────────────────── */
QSlider::groove:horizontal {
    height: 4px;
    background: #1f2937;
    border-radius: 2px;
}
QSlider::handle:horizontal {
    width: 14px;
    height: 14px;
    margin: -5px 0;
    border-radius: 7px;
    background: #38bdf8;
    border: 2px solid #0b0f19;
}
QSlider::handle:horizontal:hover {
    background: #7dd3fc;
}
QSlider::sub-page:horizontal {
    background: #0284c7;
    border-radius: 2px;
}

/* ── Buttons — Base ────────────────────────────────────────────────── */
QPushButton {
    background-color: #1e293b;
    color: #f8fafc;
    border: 1px solid #334155;
    border-radius: 5px;
    padding: 0 10px;
    min-height: 32px;
    max-height: 32px;
    font-size: 12px;
    font-weight: 600;
}
QPushButton:hover {
    background-color: #334155;
    border-color: #475569;
}
QPushButton:pressed {
    background-color: #0f172a;
}
QPushButton:disabled {
    color: #475569;
    background-color: #111827;
    border-color: #1f2937;
}

/* START button: Solid Vibrant Sky #0284c7, text #ffffff bold */
QPushButton#btn_start {
    background-color: #0284c7;
    color: #ffffff;
    font-weight: bold;
    border: 1px solid #0284c7;
}
QPushButton#btn_start:hover {
    background-color: #0369a1;
    border-color: #38bdf8;
}
QPushButton#btn_start:disabled {
    background-color: #1e293b;
    color: #475569;
    border-color: #1f2937;
}

/* PAUSE button: Amber #d97706, text #ffffff bold */
QPushButton#btn_pause {
    background-color: #d97706;
    color: #ffffff;
    font-weight: bold;
    border: 1px solid #d97706;
}
QPushButton#btn_pause:hover {
    background-color: #b45309;
    border-color: #f59e0b;
}
QPushButton#btn_pause:disabled {
    background-color: #1e293b;
    color: #475569;
    border-color: #1f2937;
}

/* RESET button: Slate #475569, text #ffffff */
QPushButton#btn_reset {
    background-color: #475569;
    color: #ffffff;
    border: 1px solid #475569;
}
QPushButton#btn_reset:hover {
    background-color: #334155;
    border-color: #64748b;
}

/* Inject Blind Spot: Subtle dark red container with #ef4444 border & text */
QPushButton#btn_occlude {
    background-color: #271418;
    color: #f87171;
    border: 1px solid #ef4444;
}
QPushButton#btn_occlude:hover {
    background-color: #3b141a;
    border-color: #fca5a5;
    color: #fca5a5;
}
QPushButton#btn_occlude:disabled {
    background-color: #181215;
    color: #5c242c;
    border-color: #381a20;
}

/* Export Audit PDF: Dark slate #1e293b with #cbd5e1 text */
QPushButton#btn_pdf {
    background-color: #1e293b;
    color: #cbd5e1;
    border: 1px solid #334155;
}
QPushButton#btn_pdf:hover {
    background-color: #334155;
    color: #f8fafc;
    border-color: #475569;
}
QPushButton#btn_pdf:disabled {
    background-color: #111827;
    color: #475569;
    border-color: #1f2937;
}

/* Browse MP4 button */
QPushButton#btn_browse {
    background-color: #1e293b;
    color: #38bdf8;
    border: 1px solid #0284c7;
    min-height: 26px;
    max-height: 26px;
    font-size: 11px;
}
QPushButton#btn_browse:hover {
    background-color: #0369a1;
    color: #ffffff;
}
QPushButton#btn_browse:disabled {
    color: #475569;
    border-color: #1f2937;
    background: #111827;
}

/* ── State Badge ───────────────────────────────────────────────────── */
QLabel#state_badge {
    border-radius: 6px;
    padding: 4px 8px;
    font-family: "Consolas", monospace;
    font-size: 12px;
    font-weight: bold;
    letter-spacing: 1.5px;
    text-align: center;
}

/* ── Status Bar ────────────────────────────────────────────────────── */
QStatusBar {
    background-color: #0b0f19;
    color: #94a3b8;
    border-top: 1px solid #1f2937;
    font-size: 11px;
    font-family: "Consolas", monospace;
}

/* ── Left ScrollArea & Minimal Scrollbar ───────────────────────────── */
QScrollArea#left_scroll {
    background: transparent;
    border: none;
}
QWidget#left_content {
    background: transparent;
}
QScrollBar:vertical {
    background-color: #0b0f19;
    width: 6px;
    margin: 0px;
    border-radius: 3px;
}
QScrollBar::handle:vertical {
    background-color: #374151;
    min-height: 24px;
    border-radius: 3px;
}
QScrollBar::handle:vertical:hover {
    background-color: #4b5563;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
    background: none;
}
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {
    background: none;
}
"""

# Aliases for compatibility
DARK_STYLESHEET = AEROSPACE_THEME_STYLESHEET
LIGHT_STYLESHEET = AEROSPACE_THEME_STYLESHEET

# ── State badge colours ─────────────────────────────────────────────────────
BADGE_STYLES: dict[str, str] = {
    "SEARCHING":   "background:#0f2238; color:#38bdf8; border:1px solid #0284c7;",
    "ACQUIRING":   "background:#341a06; color:#fb923c; border:1px solid #ea580c;",
    "TRACKING":    "background:#0a2918; color:#4ade80; border:1px solid #16a34a;",
    "LOST":        "background:#3a0d14; color:#f87171; border:1px solid #dc2626;",
    "REACQUIRING": "background:#332506; color:#facc15; border:1px solid #ca8a04;",
    "IDLE":        "background:#111827; color:#64748b; border:1px solid #1f2937;",
}

# ── Icon path ───────────────────────────────────────────────────────────────
_ICON_PATH = Path(__file__).parent / "assets" / "icon.ico"
PROJECT_ROOT = Path(__file__).resolve().parents[1]
LOGS_DIR = PROJECT_ROOT / "evaluation" / "logs"
REPORTS_DIR = PROJECT_ROOT / "evaluation" / "reports"

os.makedirs(LOGS_DIR, exist_ok=True)
os.makedirs(REPORTS_DIR, exist_ok=True)


# ── Helpers ──────────────────────────────────────────────────────────────────

def _ndarray_to_pixmap(frame: np.ndarray) -> QPixmap:
    """Convert a numpy BGR/Grey frame to a QPixmap (copy — safe across threads)."""
    h, w = frame.shape[:2]
    if frame.ndim == 3:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()
    else:
        qimg = QImage(frame.data, w, h, w, QImage.Format.Format_Grayscale8).copy()
    return QPixmap.fromImage(qimg)


def _sep() -> QFrame:
    """Horizontal separator line."""
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setStyleSheet("color: #1f2937;")
    return line


# ── Main Dashboard Window ────────────────────────────────────────────────────

class Dashboard(QMainWindow):
    """
    BeaconLock aerospace telemetry cockpit.

    Instantiate and call show() to display the window.
    Handles the full session lifecycle: source selection → start → log → PDF.
    """

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("BeaconLock — ISRO PAT Simulator  |  AlphaTrion  |  SIH-2026")
        self.setMinimumSize(1240, 740)
        self.resize(1280, 780)

        # Set window icon
        if _ICON_PATH.exists():
            self.setWindowIcon(QIcon(str(_ICON_PATH)))

        self._worker:    Optional[TrackingWorker] = None
        self._source:    Optional[FrameSource]    = None
        self._fl_logger: Optional[FrameLogger]    = None
        self._session_start = 0.0
        self._logs_dir = LOGS_DIR
        self._reports_dir = REPORTS_DIR
        os.makedirs(self._logs_dir, exist_ok=True)
        os.makedirs(self._reports_dir, exist_ok=True)

        # P95 error tracking
        self._error_history: deque[float] = deque(maxlen=300)

        self._build_ui()
        self._connect_signals()
        self._set_running_state(False)

    # ─────────────────────────────────────────────────────────────────────────
    # UI Construction
    # ─────────────────────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ── Left panel (Scrollable) ────────────────────────────────
        root.addWidget(self._build_left_panel())

        # ── Center panel (640x480 Camera Feed) ─────────────────────
        root.addWidget(self._build_center_panel(), stretch=1)

        # ── Right panel (Telemetry plots) ──────────────────────────
        root.addWidget(self._build_right_panel())

        # ── Status bar ─────────────────────────────────────────────
        self._status = QStatusBar()
        self.setStatusBar(self._status)
        self._status.showMessage("BeaconLock ready — configure source and press  ▶ START")

    # ── Left panel ───────────────────────────────────────────────────────────

    def _build_left_panel(self) -> QWidget:
        content = QWidget()
        content.setObjectName("left_content")
        vlay = QVBoxLayout(content)
        vlay.setContentsMargins(6, 4, 8, 4)
        vlay.setSpacing(5)

        # Logo & Subtitle
        logo = QLabel("🔒  BEACONLOCK")
        logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        logo.setStyleSheet(
            "font: bold 14px 'Segoe UI'; color: #38bdf8; "
            "padding: 2px 0 0 0;"
        )
        vlay.addWidget(logo)
        sub = QLabel("Virtual PAT Simulator — ISRO PS-26169")
        sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sub.setWordWrap(True)
        sub.setStyleSheet("font-size: 10px; color: #64748b; padding-bottom: 2px;")
        vlay.addWidget(sub)
        vlay.addWidget(_sep())

        # Input Source Group
        src_grp = QGroupBox("INPUT SOURCE")
        src_lay = QVBoxLayout(src_grp)
        src_lay.setContentsMargins(6, 8, 6, 6)
        src_lay.setSpacing(4)
        self._rb_mode_a = QRadioButton("Mode A — Synthetic Simulator")
        self._rb_mode_b = QRadioButton("Mode B — Evaluator MP4")
        self._rb_mode_a.setChecked(True)
        src_lay.addWidget(self._rb_mode_a)
        src_lay.addWidget(self._rb_mode_b)

        self._btn_browse = QPushButton("📂  Browse MP4…")
        self._btn_browse.setObjectName("btn_browse")
        self._btn_browse.setEnabled(False)
        self._btn_browse.setFixedHeight(26)
        self._mp4_path: Optional[str] = None
        self._lbl_mp4 = QLabel("No file selected")
        self._lbl_mp4.setStyleSheet("color:#64748b; font-size:10px;")
        self._lbl_mp4.setWordWrap(True)
        src_lay.addWidget(self._btn_browse)
        src_lay.addWidget(self._lbl_mp4)
        self._rb_mode_a.toggled.connect(lambda on: self._btn_browse.setEnabled(not on))
        self._rb_mode_a.toggled.connect(lambda on: self._cmb_motion_grp.setVisible(on))
        vlay.addWidget(src_grp)

        # Beacon Motion Group
        self._cmb_motion_grp = QGroupBox("BEACON MOTION")
        mot_lay = QVBoxLayout(self._cmb_motion_grp)
        mot_lay.setContentsMargins(6, 8, 6, 6)
        mot_lay.setSpacing(3)
        self._cmb_motion = QComboBox()
        self._cmb_motion.addItems(["straight_line", "circular", "figure_8", "random_walk"])
        self._cmb_motion.setCurrentIndex(1)
        self._cmb_motion.setFixedHeight(30)
        mot_lay.addWidget(self._cmb_motion)
        vlay.addWidget(self._cmb_motion_grp)

        # Disturbances Group
        dist_grp = QGroupBox("DISTURBANCES")
        dist_lay = QVBoxLayout(dist_grp)
        dist_lay.setContentsMargins(6, 8, 6, 6)
        dist_lay.setSpacing(4)

        # Noise σ
        row_n = QHBoxLayout()
        lbl_n = QLabel("Noise σ:")
        lbl_n.setStyleSheet("color:#cbd5e1; font-weight:600; font-size:11px;")
        row_n.addWidget(lbl_n)
        self._lbl_noise = QLabel("0 px")
        self._lbl_noise.setStyleSheet(
            "background:#0b0f19; color:#38bdf8; border:1px solid #1e3a5f; "
            "border-radius:4px; padding:2px 6px; font:bold 11px 'Consolas';"
        )
        self._lbl_noise.setFixedWidth(46)
        row_n.addStretch()
        row_n.addWidget(self._lbl_noise)
        dist_lay.addLayout(row_n)
        self._sld_noise = self._make_slider(0, 20, 0)
        dist_lay.addWidget(self._sld_noise)

        # Jitter px
        row_j = QHBoxLayout()
        lbl_j = QLabel("Jitter:")
        lbl_j.setStyleSheet("color:#cbd5e1; font-weight:600; font-size:11px;")
        row_j.addWidget(lbl_j)
        self._lbl_jitter = QLabel("0 px")
        self._lbl_jitter.setStyleSheet(
            "background:#0b0f19; color:#38bdf8; border:1px solid #1e3a5f; "
            "border-radius:4px; padding:2px 6px; font:bold 11px 'Consolas';"
        )
        self._lbl_jitter.setFixedWidth(46)
        row_j.addStretch()
        row_j.addWidget(self._lbl_jitter)
        dist_lay.addLayout(row_j)
        self._sld_jitter = self._make_slider(0, 20, 0)
        dist_lay.addWidget(self._sld_jitter)

        # Atmosphere Dropdown (proper vertical spacing and height)
        atm_lbl = QLabel("Atmosphere:")
        atm_lbl.setStyleSheet("font-weight:600; color:#cbd5e1; font-size:11px; margin-top:2px;")
        dist_lay.addWidget(atm_lbl)
        self._cmb_atmos = QComboBox()
        self._cmb_atmos.addItems(["clear", "haze", "fog", "rain", "low_light"])
        self._cmb_atmos.setFixedHeight(30)
        dist_lay.addWidget(self._cmb_atmos)

        vlay.addWidget(dist_grp)

        self._sld_noise.valueChanged.connect(lambda v: self._lbl_noise.setText(f"{v} px"))
        self._sld_jitter.valueChanged.connect(lambda v: self._lbl_jitter.setText(f"{v} px"))

        # Session Controls (Explicit 32px height buttons in clean 5px stack)
        act_grp = QGroupBox("SESSION CONTROL")
        act_lay = QVBoxLayout(act_grp)
        act_lay.setContentsMargins(6, 8, 6, 6)
        act_lay.setSpacing(5)

        self._btn_start   = QPushButton("▶  START")
        self._btn_start.setObjectName("btn_start")
        self._btn_start.setFixedHeight(32)

        self._btn_pause   = QPushButton("⏸  PAUSE")
        self._btn_pause.setObjectName("btn_pause")
        self._btn_pause.setFixedHeight(32)

        self._btn_reset   = QPushButton("↺  RESET")
        self._btn_reset.setObjectName("btn_reset")
        self._btn_reset.setFixedHeight(32)

        self._btn_occlude = QPushButton("⊘  Inject Blind Spot (30f)")
        self._btn_occlude.setObjectName("btn_occlude")
        self._btn_occlude.setFixedHeight(32)

        self._btn_pdf     = QPushButton("📄  Export Audit PDF")
        self._btn_pdf.setObjectName("btn_pdf")
        self._btn_pdf.setFixedHeight(32)

        for btn in (self._btn_start, self._btn_pause, self._btn_reset,
                    self._btn_occlude, self._btn_pdf):
            act_lay.addWidget(btn)

        vlay.addWidget(act_grp)

        # State Badge + Live FPS side-by-side row
        stat_row = QHBoxLayout()
        stat_row.setSpacing(6)

        self._badge = QLabel("IDLE")
        self._badge.setObjectName("state_badge")
        self._badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._badge.setFixedHeight(28)
        self._badge.setStyleSheet(BADGE_STYLES["IDLE"])

        self._lbl_fps = QLabel("FPS: —")
        self._lbl_fps.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_fps.setFixedHeight(28)
        self._lbl_fps.setStyleSheet(
            "color:#4ade80; font:bold 11px 'Consolas', monospace; "
            "background:#0b1a13; border:1px solid #14532d; "
            "border-radius:5px; padding:2px;"
        )

        stat_row.addWidget(self._badge, 2)
        stat_row.addWidget(self._lbl_fps, 1)
        vlay.addLayout(stat_row)

        # Session info caption
        self._lbl_session = QLabel("No active session")
        self._lbl_session.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_session.setWordWrap(True)
        self._lbl_session.setStyleSheet("color:#64748b; font-size:10px; margin-top:2px;")
        vlay.addWidget(self._lbl_session)

        # Wrap content in a clean QScrollArea
        scroll = QScrollArea()
        scroll.setObjectName("left_scroll")
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll.setFixedWidth(264)
        scroll.setWidget(content)

        return scroll

    # ── Center panel ─────────────────────────────────────────────────────────

    def _build_center_panel(self) -> QWidget:
        grp = QGroupBox("CAMERA VIEW  [640 × 480 px  |  FOV 4° × 3°  |  0.00625 °/px]")
        grp.setObjectName("center_panel")
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(10, 14, 10, 10)
        lay.setSpacing(10)
        lay.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._video_label = QLabel()
        self._video_label.setFixedSize(640, 480)
        self._video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._video_label.setStyleSheet("background:#050811; border:1px solid #1f2937; border-radius:6px;")
        self._show_placeholder()
        lay.addWidget(self._video_label)

        # Metrics bar
        metrics_row = QHBoxLayout()
        metrics_row.setSpacing(10)

        def _metric_lbl(text: str, color: str) -> QLabel:
            lbl = QLabel(text)
            lbl.setStyleSheet(
                f"color:{color}; font:bold 11px 'Consolas', monospace; "
                f"background:#0d1322; border:1px solid #1f2937; "
                f"border-radius:5px; padding:4px 10px;"
            )
            return lbl

        self._lbl_err    = _metric_lbl("Error: — px", "#38bdf8")
        self._lbl_mrad   = _metric_lbl("— mrad", "#94a3b8")
        self._lbl_p95    = _metric_lbl("P95: — px", "#a855f7")
        self._lbl_frames = _metric_lbl("Frame: 0", "#94a3b8")

        for w in (self._lbl_err, self._lbl_mrad, self._lbl_p95, self._lbl_frames):
            metrics_row.addWidget(w)
        metrics_row.addStretch()

        lay.addLayout(metrics_row)
        return grp

    # ── Right panel ──────────────────────────────────────────────────────────

    def _build_right_panel(self) -> QWidget:
        grp = QGroupBox("REAL-TIME TELEMETRY")
        grp.setObjectName("right_panel")
        grp.setMinimumWidth(330)
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(6, 12, 6, 6)
        lay.setSpacing(6)
        self._plots = PlotsPanel()
        lay.addWidget(self._plots)
        return grp

    @staticmethod
    def _make_slider(lo: int, hi: int, val: int) -> QSlider:
        s = QSlider(Qt.Orientation.Horizontal)
        s.setMinimum(lo); s.setMaximum(hi); s.setValue(val)
        return s

    # ─────────────────────────────────────────────────────────────────────────
    # Signal wiring
    # ─────────────────────────────────────────────────────────────────────────

    def _connect_signals(self) -> None:
        self._btn_start.clicked.connect(self._on_start)
        self._btn_pause.clicked.connect(self._on_pause_resume)
        self._btn_reset.clicked.connect(self._on_reset)
        self._btn_occlude.clicked.connect(self._on_inject_occlusion)
        self._btn_pdf.clicked.connect(self._on_export_pdf)
        self._btn_browse.clicked.connect(self._on_browse_mp4)

    # ─────────────────────────────────────────────────────────────────────────
    # Slots — session lifecycle
    # ─────────────────────────────────────────────────────────────────────────

    @Slot()
    def _on_start(self) -> None:
        if self._worker and self._worker.isRunning():
            return

        # Build source
        if self._rb_mode_a.isChecked():
            dist = DisturbanceConfig(
                gaussian_sigma = self._sld_noise.value(),
                jitter_enabled = self._sld_jitter.value() > 0,
                jitter_max_px  = float(self._sld_jitter.value()),
                atmospheric    = self._cmb_atmos.currentText(),
            )
            self._source = SimulatorSource(
                motion_type = self._cmb_motion.currentText(),
                disturbance = dist,
                fps         = 30.0,
            )
            mode_str = "A"
        else:
            if not self._mp4_path:
                QMessageBox.warning(self, "No file",
                                    "Please browse for an MP4 file before starting.")
                return
            from ingestion.frame_source import VideoFileSource
            # Mode B: load file, auto-resize to 640×480 if needed, greyscale
            self._source = VideoFileSource(
                path          = self._mp4_path,
                target_width  = 640,
                target_height = 480,
                greyscale     = True,
            )
            mode_str = "B"

        # Logger
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = self._logs_dir / f"session_{ts}.csv"
        self._fl_logger = FrameLogger(str(log_path), session_id=ts)

        # Worker
        self._worker = TrackingWorker(
            source       = self._source,
            frame_logger = self._fl_logger,
            input_mode   = mode_str,
        )
        self._worker.frame_ready.connect(self._on_frame_ready)
        self._worker.state_changed.connect(self._on_state_changed)
        self._worker.metrics_ready.connect(self._plots.update_plots)
        self._worker.error_occurred.connect(self._on_worker_error)
        self._worker.worker_stopped.connect(self._on_worker_stopped)

        self._error_history.clear()
        self._session_start = time.time()
        self._worker.start()
        self._set_running_state(True)
        try:
            rel_log = log_path.relative_to(PROJECT_ROOT).as_posix()
        except ValueError:
            rel_log = log_path.name
        self._lbl_session.setText(f"Log: {rel_log}")
        self._status.showMessage(
            f"▶ Session started — Mode {mode_str}  |  "
            f"{'Simulator' if mode_str == 'A' else Path(self._mp4_path).name}"
        )

    @Slot()
    def _on_pause_resume(self) -> None:
        if not self._worker:
            return
        if self._btn_pause.text().startswith("⏸"):
            self._worker.pause()
            self._btn_pause.setText("▶  RESUME")
        else:
            self._worker.resume()
            self._btn_pause.setText("⏸  PAUSE")

    @Slot()
    def _on_reset(self) -> None:
        if self._worker and self._worker.isRunning():
            self._worker.stop()
            self._worker.wait(2000)
        self._worker = None
        if self._fl_logger:
            self._fl_logger.close()
            self._fl_logger = None

        self._plots.reset()
        self._show_placeholder()
        self._error_history.clear()
        self._badge.setText("IDLE")
        self._badge.setStyleSheet(BADGE_STYLES["IDLE"])
        self._lbl_fps.setText("FPS: —")
        self._lbl_err.setText("Error: — px")
        self._lbl_mrad.setText("— mrad")
        self._lbl_p95.setText("P95: — px")
        self._lbl_frames.setText("Frame: 0")
        self._lbl_session.setText("No active session")
        self._set_running_state(False)
        self._status.showMessage("Session reset.")

    @Slot()
    def _on_inject_occlusion(self) -> None:
        if self._worker:
            self._worker.inject_occlusion(30)
            self._status.showMessage("⊘  30-frame blind spot injected.")

    @Slot()
    def _on_export_pdf(self) -> None:
        if not self._fl_logger:
            QMessageBox.information(self, "No session",
                                    "Start a session first to generate telemetry data.")
            return
        if not self._fl_logger._closed:
            self._fl_logger.flush()
        ts = time.strftime("%Y%m%d_%H%M%S")
        pdf_path = self._reports_dir / f"audit_{ts}.pdf"
        meta = {
            "motion_type":    self._cmb_motion.currentText(),
            "atmospheric":    self._cmb_atmos.currentText(),
            "gaussian_sigma": self._sld_noise.value(),
            "input_mode":     ("Mode A – Synthetic" if self._rb_mode_a.isChecked()
                               else "Mode B – Evaluator MP4"),
        }
        try:
            gen = ReportGenerator(self._fl_logger, meta)
            out = gen.generate(str(pdf_path))
            try:
                rel_pdf = out.relative_to(PROJECT_ROOT).as_posix()
            except ValueError:
                rel_pdf = str(out)
            self._status.showMessage(f"✅  PDF saved: {rel_pdf}")
            QMessageBox.information(
                self, "Audit Report Saved",
                f"ISRO-compliant audit report saved to:\n{rel_pdf}"
            )
        except Exception as exc:
            QMessageBox.critical(self, "PDF Error", str(exc))
            logger.exception("PDF generation error: %s", exc)

    @Slot()
    def _on_browse_mp4(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Select Evaluator Video", "",
            "Video files (*.mp4 *.avi *.mov *.mkv);;All files (*)"
        )
        if path:
            self._mp4_path = path
            name = Path(path).name
            self._lbl_mp4.setText(name)
            self._lbl_mp4.setStyleSheet("color:#38bdf8; font-size:10px;")
            self._status.showMessage(f"Mode B file: {name}")

    # ─────────────────────────────────────────────────────────────────────────
    # Slots — worker feedback
    # ─────────────────────────────────────────────────────────────────────────

    @Slot(object, object)
    def _on_frame_ready(self, frame: np.ndarray, metrics: dict) -> None:
        """Update video label, error readout, P95, and FPS display."""
        pix = _ndarray_to_pixmap(frame)
        self._video_label.setPixmap(pix)

        err  = float(metrics.get("error_px", 0.0))
        mrad = float(metrics.get("error_mrad", 0.0))
        fps  = float(metrics.get("fps", 0.0))
        fid  = int(metrics.get("frame_id", 0))

        self._error_history.append(err)

        # P95 calculation
        if len(self._error_history) >= 5:
            sorted_err = sorted(self._error_history)
            p95_idx = max(0, int(len(sorted_err) * 0.95) - 1)
            p95 = sorted_err[p95_idx]
            p95_color = "#a855f7" if p95 <= 10.0 else "#ef4444"
            self._lbl_p95.setText(f"P95: {p95:.1f} px")
            self._lbl_p95.setStyleSheet(
                f"color:{p95_color}; font:bold 11px 'Consolas', monospace; "
                f"background:#0d1322; border:1px solid #1f2937; "
                f"border-radius:5px; padding:4px 10px;"
            )

        err_color = "#38bdf8" if err <= 10.0 else "#ef4444"
        self._lbl_err.setText(f"Error: {err:.2f} px")
        self._lbl_err.setStyleSheet(
            f"color:{err_color}; font:bold 11px 'Consolas', monospace; "
            f"background:#0d1322; border:1px solid #1f2937; "
            f"border-radius:5px; padding:4px 10px;"
        )
        self._lbl_mrad.setText(f"{mrad:.3f} mrad")
        self._lbl_frames.setText(f"Frame: {fid}")

        fps_color = "#4ade80" if fps >= 30.0 else "#f87171"
        self._lbl_fps.setText(f"FPS: {fps:.1f}")
        self._lbl_fps.setStyleSheet(
            f"color:{fps_color}; font:bold 11px 'Consolas', monospace; "
            f"background:#0b1a13; border:1px solid #14532d; "
            f"border-radius:5px; padding:2px;"
        )

    @Slot(str)
    def _on_state_changed(self, state_name: str) -> None:
        self._badge.setText(state_name)
        style = BADGE_STYLES.get(state_name, BADGE_STYLES["IDLE"])
        self._badge.setStyleSheet(style)
        dur = time.time() - self._session_start
        self._status.showMessage(f"State → {state_name}   |   t = {dur:.1f} s")

    @Slot(str)
    def _on_worker_error(self, msg: str) -> None:
        QMessageBox.critical(self, "Worker Error", msg)
        self._set_running_state(False)

    @Slot()
    def _on_worker_stopped(self) -> None:
        self._set_running_state(False)
        if self._fl_logger and not self._fl_logger._closed:
            self._fl_logger.flush()
        self._status.showMessage("Session ended — press Export PDF to save audit report.")

    # ─────────────────────────────────────────────────────────────────────────
    # UI helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _set_running_state(self, running: bool) -> None:
        self._btn_start.setEnabled(not running)
        self._btn_pause.setEnabled(running)
        self._btn_reset.setEnabled(True)
        self._btn_occlude.setEnabled(running)
        self._btn_pdf.setEnabled(self._fl_logger is not None)
        self._cmb_motion.setEnabled(not running)
        self._cmb_atmos.setEnabled(not running)
        self._rb_mode_a.setEnabled(not running)
        self._rb_mode_b.setEnabled(not running)
        self._sld_noise.setEnabled(not running)
        self._sld_jitter.setEnabled(not running)
        self._btn_browse.setEnabled(not running and self._rb_mode_b.isChecked())
        if running:
            self._btn_pause.setText("⏸  PAUSE")

    def _show_placeholder(self) -> None:
        """Render a dark 'no signal' placeholder in the video label."""
        ph = np.zeros((480, 640, 3), dtype=np.uint8)
        ph[:] = (11, 15, 25)   # #0b0f19 aerospace obsidian
        cv2.putText(ph, "NO SIGNAL", (185, 225),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.6, (40, 60, 90), 2, cv2.LINE_AA)
        cv2.putText(ph, "Configure source and press  START", (115, 272),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (60, 90, 130), 1, cv2.LINE_AA)
        self._video_label.setPixmap(_ndarray_to_pixmap(ph))

    # ─────────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────────

    def closeEvent(self, event) -> None:
        """Graceful shutdown: stop worker, flush logger."""
        if self._worker and self._worker.isRunning():
            self._worker.stop()
            self._worker.wait(3000)
        if self._fl_logger and not self._fl_logger._closed:
            self._fl_logger.close()
        event.accept()
