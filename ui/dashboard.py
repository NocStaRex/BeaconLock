"""
ui/dashboard.py
================
BeaconLock native desktop telemetry cockpit (PySide6 + PyQtGraph).

Layout (3-panel horizontal split)
----------------------------------
┌──────────────────────────────────────────────────────────────────────┐
│  LeftPanel (220px)  │  CenterPanel (640px)  │  RightPanel (320px)   │
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
import tempfile
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from PySide6.QtCore import Qt, QTimer, Slot, Signal, QObject
from PySide6.QtGui import (
    QImage, QPixmap, QColor, QFont, QPalette,
    QIcon,
)
from PySide6.QtWidgets import (
    QButtonGroup, QComboBox, QFileDialog, QGroupBox,
    QHBoxLayout, QLabel, QPushButton, QRadioButton,
    QScrollArea, QSizePolicy, QSlider, QSplitter,
    QStatusBar, QVBoxLayout, QWidget, QMainWindow,
    QMessageBox, QProgressBar,
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

# ── Global dark-theme stylesheet (applied to QApplication) ────────────────
DARK_STYLESHEET = """
QMainWindow, QWidget {
    background-color: #0f172a;
    color: #e2e8f0;
    font-family: Consolas, "Courier New", monospace;
    font-size: 11px;
}
QGroupBox {
    border: 1px solid #1e3a5f;
    border-radius: 4px;
    margin-top: 14px;
    padding: 8px 6px 6px 6px;
    color: #60a5fa;
    font-size: 10px;
    letter-spacing: 2px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 8px;
    top: 2px;
    padding: 0 4px;
}
QPushButton {
    background-color: #1e293b;
    color: #e2e8f0;
    border: 1px solid #334155;
    border-radius: 4px;
    padding: 5px 10px;
    min-height: 26px;
}
QPushButton:hover  { background-color: #334155; }
QPushButton:pressed{ background-color: #0ea5e9; color: #fff; }
QPushButton:disabled { color: #475569; border-color: #1e293b; }
QPushButton#btn_start  { border-color: #22c55e; color: #22c55e; }
QPushButton#btn_start:hover   { background: #14532d; }
QPushButton#btn_pause  { border-color: #f59e0b; color: #f59e0b; }
QPushButton#btn_reset  { border-color: #94a3b8; }
QPushButton#btn_occlude{ border-color: #f43f5e; color: #f43f5e; }
QPushButton#btn_pdf    { border-color: #818cf8; color: #818cf8; }
QRadioButton { color: #94a3b8; spacing: 6px; }
QRadioButton::indicator { width: 14px; height: 14px; }
QRadioButton::indicator:unchecked { border: 2px solid #334155; border-radius: 7px; }
QRadioButton::indicator:checked   { background: #0ea5e9; border: 2px solid #0ea5e9; border-radius: 7px; }
QComboBox {
    background: #1e293b; border: 1px solid #334155;
    border-radius: 4px; padding: 3px 6px; color: #e2e8f0;
}
QComboBox::drop-down { border: none; }
QComboBox QAbstractItemView {
    background: #1e293b; border: 1px solid #334155; color: #e2e8f0;
    selection-background-color: #0ea5e9;
}
QSlider::groove:horizontal {
    height: 4px; background: #1e293b; border-radius: 2px;
}
QSlider::handle:horizontal {
    width: 14px; height: 14px; margin: -5px 0;
    border-radius: 7px; background: #0ea5e9;
}
QSlider::sub-page:horizontal { background: #0ea5e9; border-radius: 2px; }
QLabel#state_badge {
    border-radius: 4px; padding: 4px 10px;
    font: bold 13px Consolas; letter-spacing: 2px;
    text-align: center;
}
QStatusBar { background: #020617; color: #475569; font-size: 10px; }
"""

# ── State badge colours ────────────────────────────────────────────────────
BADGE_STYLES: dict[str, str] = {
    "SEARCHING":   "background: #1e3a5f; color: #38bdf8; border: 1px solid #38bdf8;",
    "ACQUIRING":   "background: #431407; color: #fb923c; border: 1px solid #fb923c;",
    "TRACKING":    "background: #14532d; color: #4ade80; border: 1px solid #4ade80;",
    "LOST":        "background: #450a0a; color: #f87171; border: 1px solid #f87171;",
    "REACQUIRING": "background: #422006; color: #fbbf24; border: 1px solid #fbbf24;",
}


# ── Helpers ───────────────────────────────────────────────────────────────

def _ndarray_to_pixmap(frame: np.ndarray) -> QPixmap:
    """Convert a numpy BGR/Grey frame to a QPixmap (copy — safe across threads)."""
    h, w = frame.shape[:2]
    if frame.ndim == 3:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()
    else:
        qimg = QImage(frame.data, w, h, w, QImage.Format.Format_Grayscale8).copy()
    return QPixmap.fromImage(qimg)


# ── Main Dashboard Window ──────────────────────────────────────────────────

class Dashboard(QMainWindow):
    """
    BeaconLock aerospace telemetry cockpit.

    Instantiate and call show() to display the window.
    Handles the full session lifecycle: source selection → start → log → PDF.
    """

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("BeaconLock — ISRO PAT Simulator | AlphaTrion SIH-2026")
        self.setMinimumSize(1220, 740)
        self.resize(1260, 800)

        self._worker:  Optional[TrackingWorker] = None
        self._source:  Optional[FrameSource]    = None
        self._fl_logger: Optional[FrameLogger]  = None
        self._session_start = 0.0
        self._log_dir = Path(tempfile.mkdtemp(prefix="beaconlock_"))

        self._build_ui()
        self._connect_signals()
        self._set_running_state(False)

    # ------------------------------------------------------------------
    # UI Construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QHBoxLayout(central)
        root_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.setSpacing(8)

        # ── Left panel ─────────────────────────────────────────────
        self._left_panel = self._build_left_panel()
        root_layout.addWidget(self._left_panel)

        # ── Center panel (video feed) ──────────────────────────────
        self._center_panel = self._build_center_panel()
        root_layout.addWidget(self._center_panel)

        # ── Right panel (telemetry plots) ──────────────────────────
        self._plots = PlotsPanel()
        plots_group = QGroupBox("TELEMETRY PLOTS")
        pg_layout = QVBoxLayout(plots_group)
        pg_layout.setContentsMargins(2, 10, 2, 2)
        pg_layout.addWidget(self._plots)
        root_layout.addWidget(plots_group)

        # Stretch ratios: left=fixed, center=expand, right=fixed
        root_layout.setStretch(0, 0)
        root_layout.setStretch(1, 1)
        root_layout.setStretch(2, 0)

        # ── Status bar ─────────────────────────────────────────────
        self._status = QStatusBar()
        self.setStatusBar(self._status)
        self._status.showMessage("BeaconLock ready — select source and press START.")

    def _build_left_panel(self) -> QWidget:
        panel = QWidget()
        panel.setFixedWidth(230)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # ── Logo label ─────────────────────────────────────────────
        logo = QLabel("🔒 BEACONLOCK")
        logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        logo.setStyleSheet(
            "font: bold 14px Consolas; color: #0ea5e9; "
            "padding: 6px; border-bottom: 1px solid #1e3a5f;"
        )
        layout.addWidget(logo)

        sub = QLabel("PAT Simulator — ISRO SIH-2026")
        sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sub.setStyleSheet("font-size: 9px; color: #475569; padding-bottom: 4px;")
        layout.addWidget(sub)

        # ── Source selector ────────────────────────────────────────
        src_grp = QGroupBox("INPUT SOURCE")
        src_lay = QVBoxLayout(src_grp)
        self._rb_mode_a = QRadioButton("Mode A — Synthetic Simulator")
        self._rb_mode_b = QRadioButton("Mode B — Evaluator MP4")
        self._rb_mode_a.setChecked(True)
        src_lay.addWidget(self._rb_mode_a)
        src_lay.addWidget(self._rb_mode_b)

        self._btn_browse = QPushButton("Browse MP4…")
        self._btn_browse.setEnabled(False)
        self._mp4_path: Optional[str] = None
        src_lay.addWidget(self._btn_browse)
        layout.addWidget(src_grp)

        self._rb_mode_a.toggled.connect(
            lambda on: self._btn_browse.setEnabled(not on)
        )

        # ── Motion selector ────────────────────────────────────────
        mot_grp = QGroupBox("BEACON MOTION")
        mot_lay = QVBoxLayout(mot_grp)
        self._cmb_motion = QComboBox()
        self._cmb_motion.addItems(
            ["straight_line", "circular", "figure_8", "random_walk"]
        )
        self._cmb_motion.setCurrentIndex(1)   # circular default
        mot_lay.addWidget(self._cmb_motion)
        layout.addWidget(mot_grp)

        # ── Disturbance controls ───────────────────────────────────
        dist_grp = QGroupBox("DISTURBANCES")
        dist_lay = QVBoxLayout(dist_grp)

        # Gaussian noise
        dist_lay.addWidget(QLabel("Noise σ (px):"))
        self._sld_noise = self._make_slider(0, 20, 0)
        self._lbl_noise = QLabel("0")
        dist_lay.addWidget(self._sld_noise)
        dist_lay.addWidget(self._lbl_noise)

        # Jitter
        dist_lay.addWidget(QLabel("Jitter (px):"))
        self._sld_jitter = self._make_slider(0, 20, 0)
        self._lbl_jitter = QLabel("0")
        dist_lay.addWidget(self._sld_jitter)
        dist_lay.addWidget(self._lbl_jitter)

        # Atmosphere
        dist_lay.addWidget(QLabel("Atmosphere:"))
        self._cmb_atmos = QComboBox()
        self._cmb_atmos.addItems(["clear", "haze", "fog", "rain", "low_light"])
        dist_lay.addWidget(self._cmb_atmos)

        layout.addWidget(dist_grp)

        self._sld_noise.valueChanged.connect(
            lambda v: self._lbl_noise.setText(str(v))
        )
        self._sld_jitter.valueChanged.connect(
            lambda v: self._lbl_jitter.setText(str(v))
        )

        # ── Action buttons ─────────────────────────────────────────
        act_grp = QGroupBox("SESSION CONTROL")
        act_lay = QVBoxLayout(act_grp)

        self._btn_start  = QPushButton("▶  START")
        self._btn_start.setObjectName("btn_start")
        self._btn_pause  = QPushButton("⏸  PAUSE")
        self._btn_pause.setObjectName("btn_pause")
        self._btn_reset  = QPushButton("↺  RESET")
        self._btn_reset.setObjectName("btn_reset")
        self._btn_occlude = QPushButton("⊘  Inject Blind Spot (30f)")
        self._btn_occlude.setObjectName("btn_occlude")
        self._btn_pdf    = QPushButton("📄  Export Audit PDF")
        self._btn_pdf.setObjectName("btn_pdf")

        for btn in (self._btn_start, self._btn_pause, self._btn_reset,
                    self._btn_occlude, self._btn_pdf):
            act_lay.addWidget(btn)

        layout.addWidget(act_grp)

        # ── State badge ────────────────────────────────────────────
        self._badge = QLabel("IDLE")
        self._badge.setObjectName("state_badge")
        self._badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._badge.setStyleSheet(
            "background:#1e293b; color:#475569; "
            "border:1px solid #334155; border-radius:4px; "
            "font: bold 13px Consolas; padding: 6px; letter-spacing:2px;"
        )
        layout.addWidget(self._badge)

        # ── Live FPS label ─────────────────────────────────────────
        self._lbl_fps = QLabel("FPS: —")
        self._lbl_fps.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_fps.setStyleSheet("color: #4ade80; font: bold 12px Consolas;")
        layout.addWidget(self._lbl_fps)

        layout.addStretch()

        # ── Session info ───────────────────────────────────────────
        self._lbl_session = QLabel("No active session")
        self._lbl_session.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_session.setWordWrap(True)
        self._lbl_session.setStyleSheet("color:#475569; font-size:9px;")
        layout.addWidget(self._lbl_session)

        return panel

    def _build_center_panel(self) -> QWidget:
        panel = QGroupBox("CAMERA VIEW  [640 × 480 px | FOV 4° × 3°]")
        layout = QVBoxLayout(panel)
        layout.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self._video_label = QLabel()
        self._video_label.setFixedSize(640, 480)
        self._video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._video_label.setStyleSheet(
            "background: #000; border: 1px solid #1e3a5f;"
        )
        # Default "no signal" frame
        self._show_placeholder()
        layout.addWidget(self._video_label)

        # ── Error / metric bar ─────────────────────────────────────
        info_row = QHBoxLayout()
        self._lbl_err  = QLabel("Error: — px")
        self._lbl_err.setStyleSheet("color:#22d3ee; font: bold 11px Consolas;")
        self._lbl_mrad = QLabel("— mrad")
        self._lbl_mrad.setStyleSheet("color:#94a3b8; font: 10px Consolas;")
        self._lbl_frames = QLabel("Frame: 0")
        self._lbl_frames.setStyleSheet("color:#475569; font: 10px Consolas;")
        info_row.addWidget(self._lbl_err)
        info_row.addStretch()
        info_row.addWidget(self._lbl_mrad)
        info_row.addStretch()
        info_row.addWidget(self._lbl_frames)
        layout.addLayout(info_row)

        return panel

    @staticmethod
    def _make_slider(lo: int, hi: int, val: int) -> QSlider:
        s = QSlider(Qt.Orientation.Horizontal)
        s.setMinimum(lo); s.setMaximum(hi); s.setValue(val)
        return s

    # ------------------------------------------------------------------
    # Signal wiring
    # ------------------------------------------------------------------

    def _connect_signals(self) -> None:
        self._btn_start.clicked.connect(self._on_start)
        self._btn_pause.clicked.connect(self._on_pause_resume)
        self._btn_reset.clicked.connect(self._on_reset)
        self._btn_occlude.clicked.connect(self._on_inject_occlusion)
        self._btn_pdf.clicked.connect(self._on_export_pdf)
        self._btn_browse.clicked.connect(self._on_browse_mp4)

    # ------------------------------------------------------------------
    # Slots — session lifecycle
    # ------------------------------------------------------------------

    @Slot()
    def _on_start(self) -> None:
        if self._worker and self._worker.isRunning():
            return

        # Build source
        if self._rb_mode_a.isChecked():
            dist = DisturbanceConfig(
                gaussian_sigma  = self._sld_noise.value(),
                jitter_enabled  = self._sld_jitter.value() > 0,
                jitter_max_px   = float(self._sld_jitter.value()),
                atmospheric     = self._cmb_atmos.currentText(),
            )
            self._source = SimulatorSource(
                motion_type = self._cmb_motion.currentText(),
                disturbance = dist,
                fps         = 30.0,
            )
            mode_str = "A"
        else:
            if not self._mp4_path:
                QMessageBox.warning(self, "No file", "Please browse for an MP4 file.")
                return
            from ingestion.frame_source import VideoFileSource
            self._source = VideoFileSource(self._mp4_path, target_fps=30.0)
            mode_str = "B"

        # Build logger
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = self._log_dir / f"session_{ts}.csv"
        self._fl_logger = FrameLogger(str(log_path), session_id=ts)

        # Build worker
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

        self._session_start = time.time()
        self._worker.start()
        self._set_running_state(True)
        self._lbl_session.setText(f"Log: {log_path.name}")
        self._status.showMessage(f"Session started — Mode {mode_str} | "
                                 f"{self._cmb_motion.currentText()}")

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
        self._badge.setText("IDLE")
        self._badge.setStyleSheet(
            "background:#1e293b; color:#475569; border:1px solid #334155; "
            "border-radius:4px; font: bold 13px Consolas; padding:6px;"
        )
        self._lbl_fps.setText("FPS: —")
        self._lbl_err.setText("Error: — px")
        self._lbl_mrad.setText("— mrad")
        self._lbl_frames.setText("Frame: 0")
        self._lbl_session.setText("No active session")
        self._set_running_state(False)
        self._status.showMessage("Session reset.")

    @Slot()
    def _on_inject_occlusion(self) -> None:
        if self._worker:
            self._worker.inject_occlusion(30)
            self._status.showMessage("⊘ 30-frame blind spot injected.")

    @Slot()
    def _on_export_pdf(self) -> None:
        if not self._fl_logger:
            QMessageBox.information(self, "No session",
                                    "Start a session first to generate telemetry data.")
            return
        self._fl_logger.flush()
        ts = time.strftime("%Y%m%d_%H%M%S")
        pdf_path = self._log_dir / f"audit_{ts}.pdf"
        meta = {
            "motion_type":     self._cmb_motion.currentText(),
            "atmospheric":     self._cmb_atmos.currentText(),
            "gaussian_sigma":  self._sld_noise.value(),
            "input_mode":      "Mode A – Synthetic" if self._rb_mode_a.isChecked()
                               else "Mode B – Evaluator MP4",
        }
        try:
            gen = ReportGenerator(self._fl_logger, meta)
            out = gen.generate(str(pdf_path))
            self._status.showMessage(f"✅ PDF saved: {out}")
            QMessageBox.information(self, "Report saved", str(out))
        except Exception as exc:
            QMessageBox.critical(self, "PDF error", str(exc))
            logger.exception("PDF generation error: %s", exc)

    @Slot()
    def _on_browse_mp4(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Select Evaluator MP4", "", "Video files (*.mp4 *.avi *.mov)"
        )
        if path:
            self._mp4_path = path
            self._status.showMessage(f"MP4: {path}")

    # ------------------------------------------------------------------
    # Slots — worker feedback
    # ------------------------------------------------------------------

    @Slot(object, object)
    def _on_frame_ready(self, frame: np.ndarray, metrics: dict) -> None:
        """Update video label, error readout, and FPS display."""
        pix = _ndarray_to_pixmap(frame)
        self._video_label.setPixmap(pix)

        err  = metrics.get("error_px", 0.0)
        mrad = metrics.get("error_mrad", 0.0)
        fps  = metrics.get("fps", 0.0)
        fid  = metrics.get("frame_id", 0)

        err_color = "#4ade80" if err <= 10.0 else "#f87171"
        self._lbl_err.setText(f"Error: {err:.2f} px")
        self._lbl_err.setStyleSheet(f"color:{err_color}; font: bold 11px Consolas;")
        self._lbl_mrad.setText(f"{mrad:.3f} mrad")
        self._lbl_fps.setText(f"FPS: {fps:.1f}")
        self._lbl_frames.setText(f"Frame: {fid}")

    @Slot(str)
    def _on_state_changed(self, state_name: str) -> None:
        self._badge.setText(state_name)
        style = BADGE_STYLES.get(state_name,
                    "background:#1e293b;color:#e2e8f0;border:1px solid #334155;")
        self._badge.setStyleSheet(
            f"{style} border-radius:4px; font: bold 13px Consolas; "
            f"padding:6px; letter-spacing:2px;"
        )
        dur = time.time() - self._session_start
        self._status.showMessage(
            f"State → {state_name}  |  t={dur:.1f}s"
        )

    @Slot(str)
    def _on_worker_error(self, msg: str) -> None:
        QMessageBox.critical(self, "Worker error", msg)
        self._set_running_state(False)

    @Slot()
    def _on_worker_stopped(self) -> None:
        self._set_running_state(False)
        if self._fl_logger:
            self._fl_logger.flush()
        self._status.showMessage("Session ended.")

    # ------------------------------------------------------------------
    # UI helpers
    # ------------------------------------------------------------------

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
        if running:
            self._btn_pause.setText("⏸  PAUSE")

    def _show_placeholder(self) -> None:
        """Render a dark 'no signal' placeholder in the video label."""
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, "NO SIGNAL", (180, 230),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.8, (30, 40, 60), 3, cv2.LINE_AA)
        cv2.putText(placeholder, "Press START to begin", (160, 280),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (40, 60, 80), 2, cv2.LINE_AA)
        self._video_label.setPixmap(_ndarray_to_pixmap(placeholder))

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:
        """Graceful shutdown: stop worker, flush logger."""
        if self._worker and self._worker.isRunning():
            self._worker.stop()
            self._worker.wait(3000)
        if self._fl_logger:
            self._fl_logger.close()
        event.accept()
