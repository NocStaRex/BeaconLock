"""
ui/dashboard.py
================
BeaconLock native desktop telemetry cockpit (PySide6 + PyQtGraph).

Layout (3-panel horizontal split)
----------------------------------
┌──────────────────────────────────────────────────────────────────────┐
│  LeftPanel (240px)  │  CenterPanel (640px)  │  RightPanel (340px)   │
│  Controls           │  640×480 Camera View  │  Telemetry Plots       │
├────────────────────┤├──────────────────────┤├──────────────────────-┤
│  Source toggle      ││  Video feed label    ││  Error (px) curve     │
│  Motion selector    ││  Crosshair overlay   ││  Slew rate curves     │
│  Disturbance sliders││  State / error HUD   ││  FPS gauge            │
│  Action buttons     ││                      ││                       │
│  State badge        ││                      ││                       │
└────────────────────┘└──────────────────────┘└───────────────────────┘

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
import tempfile
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
    QSizePolicy, QSlider, QVBoxLayout, QWidget, QMainWindow,
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
# Industrial Clean Aerospace Theme
# High-contrast, crisp, mission-control palette
# Primary: #f8fafc (surface) / #0f172a (text) / #0284c7 (accent)
# ─────────────────────────────────────────────────────────────────────────────

LIGHT_STYLESHEET = """
/* ── Root ──────────────────────────────────────────────────────────── */
QMainWindow, QWidget {
    background-color: #f1f5f9;
    color: #0f172a;
    font-family: "Segoe UI", "Inter", "Helvetica Neue", sans-serif;
    font-size: 12px;
}

/* ── Group boxes ────────────────────────────────────────────────────── */
QGroupBox {
    background-color: #ffffff;
    border: 1px solid #cbd5e1;
    border-radius: 6px;
    margin-top: 18px;
    padding: 10px 8px 8px 8px;
    font-size: 10px;
    font-weight: bold;
    letter-spacing: 1.5px;
    color: #475569;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    top: 3px;
    padding: 0 5px;
    background-color: #f1f5f9;
}

/* ── Radio buttons ──────────────────────────────────────────────────── */
QRadioButton {
    color: #334155;
    spacing: 7px;
    font-size: 12px;
    padding: 2px 0;
}
QRadioButton::indicator {
    width: 15px; height: 15px;
}
QRadioButton::indicator:unchecked {
    border: 2px solid #94a3b8;
    border-radius: 8px;
    background: #ffffff;
}
QRadioButton::indicator:checked {
    background: #0284c7;
    border: 2px solid #0284c7;
    border-radius: 8px;
}

/* ── Combo boxes ────────────────────────────────────────────────────── */
QComboBox {
    background-color: #ffffff;
    border: 1px solid #94a3b8;
    border-radius: 5px;
    padding: 5px 10px;
    min-height: 34px;
    color: #0f172a;
    font-size: 12px;
    font-weight: 600;
    selection-background-color: #0284c7;
}
QComboBox:hover { border-color: #0284c7; }
QComboBox:focus { border: 2px solid #0284c7; }
QComboBox::drop-down {
    border: none;
    width: 22px;
}
QComboBox::down-arrow {
    image: none;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-top: 6px solid #64748b;
    width: 0; height: 0;
    margin-right: 6px;
}
QComboBox QAbstractItemView {
    background-color: #ffffff;
    border: 1px solid #cbd5e1;
    color: #0f172a;
    font-size: 12px;
    font-weight: 600;
    selection-background-color: #bfdbfe;
    selection-color: #1e3a8a;
    padding: 4px;
}

/* ── Sliders ────────────────────────────────────────────────────────── */
QSlider::groove:horizontal {
    height: 5px;
    background: #e2e8f0;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    width: 16px; height: 16px;
    margin: -6px 0;
    border-radius: 8px;
    background: #0284c7;
    border: 2px solid #ffffff;
}
QSlider::sub-page:horizontal {
    background: #0284c7;
    border-radius: 3px;
}

/* ── Buttons — base ─────────────────────────────────────────────────── */
QPushButton {
    background-color: #f8fafc;
    color: #334155;
    border: 1px solid #cbd5e1;
    border-radius: 5px;
    padding: 0 12px;
    min-height: 36px;
    font-size: 12px;
    font-weight: 600;
}
QPushButton:hover { background-color: #e2e8f0; border-color: #94a3b8; }
QPushButton:pressed { background-color: #cbd5e1; }
QPushButton:disabled { color: #94a3b8; background-color: #f1f5f9; border-color: #e2e8f0; }

/* START button — sky-600 */
QPushButton#btn_start {
    background-color: #0284c7;
    color: #ffffff;
    border: none;
}
QPushButton#btn_start:hover  { background-color: #0369a1; }
QPushButton#btn_start:pressed{ background-color: #075985; }

/* PAUSE button — amber-600 */
QPushButton#btn_pause {
    background-color: #d97706;
    color: #ffffff;
    border: none;
}
QPushButton#btn_pause:hover  { background-color: #b45309; }
QPushButton#btn_pause:disabled { background-color: #fef3c7; color: #a16207; border: none; }

/* RESET button — slate-500 */
QPushButton#btn_reset {
    background-color: #64748b;
    color: #ffffff;
    border: none;
}
QPushButton#btn_reset:hover { background-color: #475569; }

/* Inject blind spot — red outline */
QPushButton#btn_occlude {
    background-color: #fee2e2;
    color: #b91c1c;
    border: 1.5px solid #ef4444;
}
QPushButton#btn_occlude:hover { background-color: #fecaca; border-color: #dc2626; }

/* Export PDF — dark */
QPushButton#btn_pdf {
    background-color: #1e293b;
    color: #ffffff;
    border: none;
}
QPushButton#btn_pdf:hover { background-color: #0f172a; }
QPushButton#btn_pdf:disabled { background-color: #e2e8f0; color: #94a3b8; }

/* Browse button */
QPushButton#btn_browse {
    background-color: #f0f9ff;
    color: #0369a1;
    border: 1.5px solid #7dd3fc;
}
QPushButton#btn_browse:hover { background-color: #e0f2fe; }
QPushButton#btn_browse:disabled { color: #94a3b8; border-color: #e2e8f0; background: #f8fafc; }

/* ── State badge ─────────────────────────────────────────────────────── */
QLabel#state_badge {
    border-radius: 6px;
    padding: 7px 12px;
    font-family: "Consolas", "Courier New", monospace;
    font-size: 13px;
    font-weight: bold;
    letter-spacing: 2px;
    text-align: center;
}

/* ── Status bar ──────────────────────────────────────────────────────── */
QStatusBar {
    background-color: #0f172a;
    color: #94a3b8;
    font-size: 11px;
    font-family: "Consolas", monospace;
}

/* ── Separator ───────────────────────────────────────────────────────── */
QFrame[frameShape="4"], QFrame[frameShape="5"] {
    color: #e2e8f0;
}
"""

# ── State badge colours ─────────────────────────────────────────────────────
BADGE_STYLES: dict[str, str] = {
    "SEARCHING":   "background:#dbeafe; color:#1d4ed8; border:1.5px solid #3b82f6;",
    "ACQUIRING":   "background:#ffedd5; color:#c2410c; border:1.5px solid #f97316;",
    "TRACKING":    "background:#dcfce7; color:#15803d; border:1.5px solid #22c55e;",
    "LOST":        "background:#fee2e2; color:#b91c1c; border:1.5px solid #ef4444;",
    "REACQUIRING": "background:#fef9c3; color:#a16207; border:1.5px solid #eab308;",
}

# ── Icon path ───────────────────────────────────────────────────────────────
_ICON_PATH = Path(__file__).parent / "assets" / "icon.ico"


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
    line.setStyleSheet("color: #e2e8f0;")
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
        self.setMinimumSize(1240, 760)
        self.resize(1300, 820)

        # Set window icon
        if _ICON_PATH.exists():
            self.setWindowIcon(QIcon(str(_ICON_PATH)))

        self._worker:    Optional[TrackingWorker] = None
        self._source:    Optional[FrameSource]    = None
        self._fl_logger: Optional[FrameLogger]    = None
        self._session_start = 0.0
        self._log_dir = Path(tempfile.mkdtemp(prefix="beaconlock_"))

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
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        root.addWidget(self._build_left_panel())
        root.addWidget(self._build_center_panel(), stretch=1)
        root.addWidget(self._build_right_panel())

        self._status = QStatusBar()
        self.setStatusBar(self._status)
        self._status.showMessage("BeaconLock ready — configure source and press  ▶ START")

    # ── Left panel ───────────────────────────────────────────────────────────

    def _build_left_panel(self) -> QWidget:
        panel = QWidget()
        panel.setFixedWidth(248)
        panel.setStyleSheet("background: transparent;")
        vlay = QVBoxLayout(panel)
        vlay.setContentsMargins(0, 0, 0, 0)
        vlay.setSpacing(8)

        # Logo
        logo = QLabel("🔒  BEACONLOCK")
        logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        logo.setStyleSheet(
            "font: bold 15px 'Segoe UI'; color: #0284c7; "
            "padding: 8px 4px 4px 4px;"
        )
        vlay.addWidget(logo)
        sub = QLabel("Virtual PAT Simulator — ISRO SIH-2026 | PS-26169")
        sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sub.setWordWrap(True)
        sub.setStyleSheet("font-size: 10px; color: #64748b; padding-bottom: 2px;")
        vlay.addWidget(sub)
        vlay.addWidget(_sep())

        # Source
        src_grp = QGroupBox("INPUT SOURCE")
        src_lay = QVBoxLayout(src_grp)
        src_lay.setSpacing(6)
        self._rb_mode_a = QRadioButton("Mode A — Synthetic Simulator")
        self._rb_mode_b = QRadioButton("Mode B — Evaluator MP4")
        self._rb_mode_a.setChecked(True)
        src_lay.addWidget(self._rb_mode_a)
        src_lay.addWidget(self._rb_mode_b)
        self._btn_browse = QPushButton("📂  Browse MP4…")
        self._btn_browse.setObjectName("btn_browse")
        self._btn_browse.setEnabled(False)
        self._mp4_path: Optional[str] = None
        self._lbl_mp4 = QLabel("No file selected")
        self._lbl_mp4.setStyleSheet("color:#94a3b8; font-size:10px;")
        self._lbl_mp4.setWordWrap(True)
        src_lay.addWidget(self._btn_browse)
        src_lay.addWidget(self._lbl_mp4)
        self._rb_mode_a.toggled.connect(lambda on: self._btn_browse.setEnabled(not on))
        self._rb_mode_a.toggled.connect(lambda on: self._cmb_motion_grp.setVisible(on))
        vlay.addWidget(src_grp)

        # Motion
        self._cmb_motion_grp = QGroupBox("BEACON MOTION")
        mot_lay = QVBoxLayout(self._cmb_motion_grp)
        self._cmb_motion = QComboBox()
        self._cmb_motion.addItems(["straight_line", "circular", "figure_8", "random_walk"])
        self._cmb_motion.setCurrentIndex(1)
        mot_lay.addWidget(self._cmb_motion)
        vlay.addWidget(self._cmb_motion_grp)

        # Disturbances
        dist_grp = QGroupBox("DISTURBANCES")
        dist_lay = QVBoxLayout(dist_grp)
        dist_lay.setSpacing(5)

        # Noise
        row_n = QHBoxLayout()
        row_n.addWidget(QLabel("Noise σ:"))
        self._lbl_noise = QLabel("0 px")
        self._lbl_noise.setStyleSheet(
            "background:#f0f9ff; color:#0369a1; border:1px solid #7dd3fc; "
            "border-radius:4px; padding:2px 7px; font:bold 11px 'Consolas';"
        )
        self._lbl_noise.setFixedWidth(46)
        row_n.addStretch()
        row_n.addWidget(self._lbl_noise)
        dist_lay.addLayout(row_n)
        self._sld_noise = self._make_slider(0, 20, 0)
        dist_lay.addWidget(self._sld_noise)

        # Jitter
        row_j = QHBoxLayout()
        row_j.addWidget(QLabel("Jitter:"))
        self._lbl_jitter = QLabel("0 px")
        self._lbl_jitter.setStyleSheet(
            "background:#f0f9ff; color:#0369a1; border:1px solid #7dd3fc; "
            "border-radius:4px; padding:2px 7px; font:bold 11px 'Consolas';"
        )
        self._lbl_jitter.setFixedWidth(46)
        row_j.addStretch()
        row_j.addWidget(self._lbl_jitter)
        dist_lay.addLayout(row_j)
        self._sld_jitter = self._make_slider(0, 20, 0)
        dist_lay.addWidget(self._sld_jitter)

        # Atmosphere
        atm_lbl = QLabel("Atmosphere:")
        atm_lbl.setStyleSheet("font-weight:600; color:#334155;")
        dist_lay.addWidget(atm_lbl)
        self._cmb_atmos = QComboBox()
        self._cmb_atmos.addItems(["clear", "haze", "fog", "rain", "low_light"])
        dist_lay.addWidget(self._cmb_atmos)

        vlay.addWidget(dist_grp)

        self._sld_noise.valueChanged.connect(lambda v: self._lbl_noise.setText(f"{v} px"))
        self._sld_jitter.valueChanged.connect(lambda v: self._lbl_jitter.setText(f"{v} px"))

        # Session controls
        act_grp = QGroupBox("SESSION CONTROL")
        act_lay = QVBoxLayout(act_grp)
        act_lay.setSpacing(6)

        self._btn_start   = QPushButton("▶  START")
        self._btn_start.setObjectName("btn_start")
        self._btn_pause   = QPushButton("⏸  PAUSE")
        self._btn_pause.setObjectName("btn_pause")
        self._btn_reset   = QPushButton("↺  RESET")
        self._btn_reset.setObjectName("btn_reset")
        self._btn_occlude = QPushButton("⊘  Inject Blind Spot  (30f)")
        self._btn_occlude.setObjectName("btn_occlude")
        self._btn_pdf     = QPushButton("📄  Export Audit PDF")
        self._btn_pdf.setObjectName("btn_pdf")

        for btn in (self._btn_start, self._btn_pause, self._btn_reset,
                    self._btn_occlude, self._btn_pdf):
            act_lay.addWidget(btn)

        vlay.addWidget(act_grp)

        # State badge
        self._badge = QLabel("IDLE")
        self._badge.setObjectName("state_badge")
        self._badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._badge.setStyleSheet(
            "background:#f1f5f9; color:#64748b; border:1.5px solid #cbd5e1; "
            "border-radius:6px; font:bold 13px 'Consolas'; padding:7px; letter-spacing:2px;"
        )
        vlay.addWidget(self._badge)

        # FPS live readout
        self._lbl_fps = QLabel("FPS: —")
        self._lbl_fps.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_fps.setStyleSheet(
            "color:#15803d; font:bold 13px 'Consolas'; "
            "background:#f0fdf4; border:1px solid #bbf7d0; "
            "border-radius:4px; padding:4px;"
        )
        vlay.addWidget(self._lbl_fps)

        vlay.addStretch()

        # Session info
        self._lbl_session = QLabel("No active session")
        self._lbl_session.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_session.setWordWrap(True)
        self._lbl_session.setStyleSheet("color:#94a3b8; font-size:10px;")
        vlay.addWidget(self._lbl_session)

        return panel

    # ── Center panel ─────────────────────────────────────────────────────────

    def _build_center_panel(self) -> QWidget:
        grp = QGroupBox("CAMERA VIEW  [640 × 480 px  |  FOV 4° × 3°  |  0.00625 °/px]")
        lay = QVBoxLayout(grp)
        lay.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._video_label = QLabel()
        self._video_label.setFixedSize(640, 480)
        self._video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._video_label.setStyleSheet("background:#0f172a; border:1px solid #cbd5e1; border-radius:3px;")
        self._show_placeholder()
        lay.addWidget(self._video_label)

        # Metrics bar
        metrics_row = QHBoxLayout()
        metrics_row.setSpacing(12)

        def _metric_lbl(text: str, color: str) -> QLabel:
            lbl = QLabel(text)
            lbl.setStyleSheet(
                f"color:{color}; font:bold 11px 'Consolas'; "
                f"background:#f8fafc; border:1px solid #e2e8f0; "
                f"border-radius:4px; padding:3px 8px;"
            )
            return lbl

        self._lbl_err    = _metric_lbl("Error: — px", "#0284c7")
        self._lbl_mrad   = _metric_lbl("— mrad", "#475569")
        self._lbl_p95    = _metric_lbl("P95: — px", "#7c3aed")
        self._lbl_frames = _metric_lbl("Frame: 0", "#64748b")

        for w in (self._lbl_err, self._lbl_mrad, self._lbl_p95, self._lbl_frames):
            metrics_row.addWidget(w)
        metrics_row.addStretch()

        lay.addLayout(metrics_row)
        return grp

    # ── Right panel ──────────────────────────────────────────────────────────

    def _build_right_panel(self) -> QWidget:
        self._plots = PlotsPanel()
        grp = QGroupBox("REAL-TIME TELEMETRY")
        lay = QVBoxLayout(grp)
        lay.setContentsMargins(4, 12, 4, 4)
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
        log_path = self._log_dir / f"session_{ts}.csv"
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
        self._lbl_session.setText(f"Log: {log_path.name}")
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
        self._badge.setStyleSheet(
            "background:#f1f5f9; color:#64748b; border:1.5px solid #cbd5e1; "
            "border-radius:6px; font:bold 13px 'Consolas'; padding:7px;"
        )
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
        pdf_path = self._log_dir / f"audit_{ts}.pdf"
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
            self._status.showMessage(f"✅  PDF saved: {out}")
            QMessageBox.information(self, "Audit Report Saved",
                                    f"ISRO-compliant audit report saved to:\n{out}")
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
            self._lbl_mp4.setStyleSheet("color:#0369a1; font-size:10px;")
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
            p95_color = "#7c3aed" if p95 <= 10.0 else "#b91c1c"
            self._lbl_p95.setText(f"P95: {p95:.1f} px")
            self._lbl_p95.setStyleSheet(
                f"color:{p95_color}; font:bold 11px 'Consolas'; "
                f"background:#f8fafc; border:1px solid #e2e8f0; "
                f"border-radius:4px; padding:3px 8px;"
            )

        err_color = "#0284c7" if err <= 10.0 else "#b91c1c"
        self._lbl_err.setText(f"Error: {err:.2f} px")
        self._lbl_err.setStyleSheet(
            f"color:{err_color}; font:bold 11px 'Consolas'; "
            f"background:#f8fafc; border:1px solid #e2e8f0; "
            f"border-radius:4px; padding:3px 8px;"
        )
        self._lbl_mrad.setText(f"{mrad:.3f} mrad")
        self._lbl_frames.setText(f"Frame: {fid}")

        fps_color = "#15803d" if fps >= 30.0 else "#dc2626"
        self._lbl_fps.setText(f"FPS: {fps:.1f}")
        self._lbl_fps.setStyleSheet(
            f"color:{fps_color}; font:bold 13px 'Consolas'; "
            f"background:#{'f0fdf4' if fps >= 30.0 else 'fef2f2'}; "
            f"border:1px solid #{'bbf7d0' if fps >= 30.0 else 'fca5a5'}; "
            f"border-radius:4px; padding:4px;"
        )

    @Slot(str)
    def _on_state_changed(self, state_name: str) -> None:
        self._badge.setText(state_name)
        base = BADGE_STYLES.get(state_name,
                                "background:#f1f5f9;color:#334155;border:1px solid #cbd5e1;")
        self._badge.setStyleSheet(
            f"{base} border-radius:6px; font:bold 13px 'Consolas'; "
            f"padding:7px; letter-spacing:2px;"
        )
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
        ph[:] = (15, 23, 42)   # slate-900
        cv2.putText(ph, "NO SIGNAL", (185, 225),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.6, (30, 60, 100), 2, cv2.LINE_AA)
        cv2.putText(ph, "Configure source and press  START", (115, 272),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (45, 80, 130), 1, cv2.LINE_AA)
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
