"""
app/orchestrator.py
====================
TrackingWorker — QThread-based vision + control processing worker.

Architecture (Benchmark-2 compliant)
--------------------------------------
The worker runs entirely in a QThread and communicates with the GUI via
Qt signals.  It NEVER modifies any GUI widget directly.

Per-frame loop
--------------
1. Read frame from FrameSource (Mode A or B — identical interface).
2. CentroidEstimator  → sub-pixel CoG detection.
3. KalmanTracker      → 6-state prediction + coast on miss.
4. PDController       → rate-limited pan/tilt commands.
5. StateMachine.update() → state transition.
6. FrameLogger.log()  → CSV row.
7. Render overlay     → BGR frame with crosshair, KF ellipse, state box.
8. Emit Qt signals    → main thread updates video + plots.

Signals emitted (all cross-thread safe via Qt queue)
------------------------------------------------------
frame_ready(np.ndarray, dict)  — annotated 640×480 BGR frame + metrics dict
state_changed(str)             — new state name whenever FSM transitions
metrics_ready(dict)            — same metrics dict (for plots panel decoupling)
error_occurred(str)            — unrecoverable exception message
worker_stopped()               — emitted when run() exits cleanly

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import logging
import math
import time
from typing import Optional, TYPE_CHECKING

import cv2
import numpy as np

from PySide6.QtCore import QThread, Signal, QMutex, QMutexLocker

from app.state_machine import StateMachine, TrackingState
from control.pid_controller import PDController
from ingestion.frame_source import FrameSource
from perception.centroid import CentroidEstimator
from perception.kalman_tracker import KalmanTracker

if TYPE_CHECKING:
    from evaluation.logger import FrameLogger

logger = logging.getLogger(__name__)

# ── Overlay colour map (BGR) ──────────────────────────────────────────────
STATE_COLORS: dict[TrackingState, tuple[int, int, int]] = {
    TrackingState.SEARCHING:   (0,  191, 255),   # deep sky blue
    TrackingState.ACQUIRING:   (0,  165, 255),   # orange
    TrackingState.TRACKING:    (0,  255, 0),     # green
    TrackingState.LOST:        (0,  0,   255),   # red
    TrackingState.REACQUIRING: (0,  255, 255),   # yellow
}

FRAME_CX = 320
FRAME_CY = 240


class TrackingWorker(QThread):
    """
    Vision + control processing worker running in a dedicated QThread.

    Parameters
    ----------
    source       : FrameSource  Mode A SimulatorSource or Mode B VideoFileSource.
    frame_logger : FrameLogger  Telemetry CSV logger (must outlive this worker).
    input_mode   : str          'A' (simulator) or 'B' (video) — logged in CSV.
    kp, kd       : float        PD controller gains.
    target_fps   : float        Desired processing rate (soft limit via sleep).
    """

    # ── Qt signals ────────────────────────────────────────────────────
    frame_ready    = Signal(object, object)   # (np.ndarray BGR, dict metrics)
    state_changed  = Signal(str)              # new TrackingState.value
    metrics_ready  = Signal(object)           # dict metrics (for plots decoupling)
    error_occurred = Signal(str)              # exception message
    worker_stopped = Signal()                 # clean exit

    def __init__(
        self,
        source: FrameSource,
        frame_logger: "FrameLogger",
        input_mode: str = "A",
        kp: float = 2.5,
        kd: float = 0.3,
        target_fps: float = 30.0,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._source   = source
        self._logger   = frame_logger
        self._mode     = input_mode
        self._kp       = kp
        self._kd       = kd
        self._dt       = 1.0 / max(target_fps, 1.0)
        self._fps_nom  = target_fps

        # Per-frame processing objects (created fresh on run())
        self._centroid: Optional[CentroidEstimator] = None
        self._kalman:   Optional[KalmanTracker]     = None
        self._ctrl:     Optional[PDController]      = None
        self._fsm:      Optional[StateMachine]      = None

        # Control flags
        self._mutex    = QMutex()
        self._running  = False
        self._paused   = False

        # Diagnostics
        self._frame_id = 0
        self._t_start  = 0.0
        self._fps_ema  = target_fps   # exponential moving average

    # ------------------------------------------------------------------
    # QThread entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Main processing loop — executes in the worker thread."""
        try:
            self._init_pipeline()
            self._running = True
            self._t_start = time.perf_counter()
            prev_state    = TrackingState.SEARCHING

            while self._running:
                # ── Pause handling ─────────────────────────────────
                with QMutexLocker(self._mutex):
                    paused = self._paused
                if paused:
                    time.sleep(0.05)
                    continue

                t0 = time.perf_counter()

                # ── 1. Read frame ──────────────────────────────────
                ok, frame = self._source.read()
                if not ok or frame is None:
                    logger.warning("FrameSource returned no frame — stopping.")
                    break

                # ── 2. Centroid estimation ─────────────────────────
                roi_centre = None
                if self._kalman.is_initialised:
                    st = self._kalman.state
                    roi_centre = (float(st[0]), float(st[1]))

                cog = self._centroid.estimate(frame, roi_centre=roi_centre)

                # ── 3. Kalman filter step ──────────────────────────
                if cog.detected:
                    kr = self._kalman.step(cog.cx, cog.cy, cog.confidence)
                else:
                    kr = self._kalman.step(None, None, confidence=0.0)

                px = kr.predicted_x
                py = kr.predicted_y

                # ── 4. Error ───────────────────────────────────────
                error_px   = math.hypot(px - FRAME_CX, py - FRAME_CY)
                error_mrad = error_px * 0.00625 * (math.pi / 180.0) * 1000.0

                # ── 5. State machine ───────────────────────────────
                fsm_result = self._fsm.update(cog.detected, error_px)
                cur_state  = fsm_result.state

                # ── 6. PD control ──────────────────────────────────
                err_x = px - FRAME_CX
                err_y = py - FRAME_CY
                cmd = self._ctrl.update(err_x, err_y, dt=self._dt)

                # ── 7. Actuate camera (Mode A only) ────────────────
                if hasattr(self._source, "apply_camera_control"):
                    self._source.apply_camera_control(
                        cmd.pan_rate_deg_s, cmd.tilt_rate_deg_s, self._dt
                    )

                # ── 8. FPS EMA ────────────────────────────────────
                elapsed = time.perf_counter() - t0
                inst_fps = 1.0 / elapsed if elapsed > 1e-6 else self._fps_nom
                self._fps_ema = 0.9 * self._fps_ema + 0.1 * inst_fps

                # ── 9. Log ─────────────────────────────────────────
                ts = time.perf_counter() - self._t_start
                self._logger.log(
                    frame_id    = self._frame_id,
                    timestamp_s = ts,
                    input_mode  = f"Mode {self._mode}",
                    centroid_x  = cog.cx if cog.detected else px,
                    centroid_y  = cog.cy if cog.detected else py,
                    error_px    = error_px,
                    error_mrad  = error_mrad,
                    pan_deg_s   = cmd.pan_rate_deg_s,
                    tilt_deg_s  = cmd.tilt_rate_deg_s,
                    fps         = self._fps_ema,
                    lock_state  = cur_state.value,
                )

                # ── 10. Overlay rendering ──────────────────────────
                display = self._render_overlay(
                    frame, cog, px, py, error_px, cur_state
                )

                # ── 11. Build metrics dict ─────────────────────────
                metrics = {
                    "frame_id":   self._frame_id,
                    "timestamp_s":ts,
                    "error_px":   error_px,
                    "error_mrad": error_mrad,
                    "pan_deg_s":  cmd.pan_rate_deg_s,
                    "tilt_deg_s": cmd.tilt_rate_deg_s,
                    "fps":        self._fps_ema,
                    "state":      cur_state.value,
                    "centroid_x": cog.cx if cog.detected else px,
                    "centroid_y": cog.cy if cog.detected else py,
                    "detected":   cog.detected,
                    "confidence": cog.confidence if cog.detected else 0.0,
                }

                # ── 12. Emit Qt signals ────────────────────────────
                self.frame_ready.emit(display, metrics)
                self.metrics_ready.emit(metrics)

                if cur_state != prev_state:
                    self.state_changed.emit(cur_state.value)
                    prev_state = cur_state

                # ── 13. Rate-limit to target FPS ──────────────────
                sleep_s = self._dt - elapsed
                if sleep_s > 0.001:
                    time.sleep(sleep_s)

                self._frame_id += 1

        except Exception as exc:
            logger.exception("TrackingWorker error: %s", exc)
            self.error_occurred.emit(str(exc))
        finally:
            self._running = False
            self.worker_stopped.emit()
            logger.info("TrackingWorker stopped after %d frames.", self._frame_id)

    # ------------------------------------------------------------------
    # Control methods (called from GUI thread — thread-safe via mutex)
    # ------------------------------------------------------------------

    def stop(self) -> None:
        """Request the worker loop to exit."""
        with QMutexLocker(self._mutex):
            self._running = False

    def pause(self) -> None:
        with QMutexLocker(self._mutex):
            self._paused = True

    def resume(self) -> None:
        with QMutexLocker(self._mutex):
            self._paused = False

    def inject_occlusion(self, frames: int = 30) -> None:
        """Inject beacon occlusion for N frames (Mode A only)."""
        if hasattr(self._source, "inject_occlusion"):
            self._source.inject_occlusion(frames)

    def reset_fsm(self) -> None:
        """Reset FSM to SEARCHING (used by the RESET button)."""
        if self._fsm is not None:
            self._fsm.reset()

    @property
    def frame_id(self) -> int:
        return self._frame_id

    @property
    def fps(self) -> float:
        return self._fps_ema

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _init_pipeline(self) -> None:
        """Construct all processing objects (called from worker thread)."""
        self._centroid = CentroidEstimator()
        self._kalman   = KalmanTracker(dt=self._dt)
        self._ctrl = PDController(
            kp=self._kp, kd=self._kd,
            max_rate_deg_s=5.0,
            deadband_deg=0.05,   # 0.05° ≈ 8 px — allows convergence within 10px lock zone
        )
        self._fsm      = StateMachine()
        self._frame_id = 0
        logger.debug("TrackingWorker pipeline initialised (kp=%.2f kd=%.2f dt=%.4f)",
                     self._kp, self._kd, self._dt)

    def _render_overlay(
        self,
        frame:   np.ndarray,
        cog,
        pred_x:  float,
        pred_y:  float,
        error_px:float,
        state:   TrackingState,
    ) -> np.ndarray:
        """
        Draw telemetry overlays onto the camera frame.

        Returns a BGR uint8 copy (never modifies the original).
        """
        # Convert greyscale → BGR for coloured overlays
        if frame.ndim == 2:
            display = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            display = frame.copy()

        color = STATE_COLORS.get(state, (255, 255, 255))

        # ── Frame-centre crosshair (lock reference) ────────────────
        cv2.line(display, (FRAME_CX - 24, FRAME_CY), (FRAME_CX + 24, FRAME_CY),
                 (80, 80, 80), 1)
        cv2.line(display, (FRAME_CX, FRAME_CY - 24), (FRAME_CX, FRAME_CY + 24),
                 (80, 80, 80), 1)

        # ── Sub-pixel centroid marker (cyan cross) ─────────────────
        if cog.detected:
            ccx = int(round(cog.cx))
            ccy = int(round(cog.cy))
            cv2.drawMarker(display, (ccx, ccy),
                           (255, 255, 0), cv2.MARKER_CROSS, 16, 2)

        # ── Kalman predicted position (coloured circle) ────────────
        pkx = int(round(pred_x))
        pky = int(round(pred_y))
        cv2.circle(display, (pkx, pky), 10, color, 2)

        # ── Tracking bounding box ──────────────────────────────────
        bs = 36
        cv2.rectangle(display,
                      (pkx - bs, pky - bs),
                      (pkx + bs, pky + bs),
                      color, 1)

        # ── State badge (top-left) ─────────────────────────────────
        badge_text = state.value
        cv2.rectangle(display, (4, 4), (160, 28), (0, 0, 0), -1)
        cv2.putText(display, badge_text, (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

        # ── Error readout (top-right) ──────────────────────────────
        err_text = f"Err:{error_px:5.1f}px"
        cv2.rectangle(display, (480, 4), (636, 28), (0, 0, 0), -1)
        err_color = (0, 255, 0) if error_px <= 10.0 else (0, 0, 255)
        cv2.putText(display, err_text, (484, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, err_color, 1, cv2.LINE_AA)

        # ── 10-px spec ring ───────────────────────────────────────
        cv2.circle(display, (FRAME_CX, FRAME_CY), 10,
                   (0, 0, 180), 1, cv2.LINE_AA)

        return display
