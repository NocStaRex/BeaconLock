"""
ingestion/frame_source.py
=========================
Abstract FrameSource interface for BeaconLock.

ARCHITECTURAL CONTRACT (MANDATORY):
    - The TrackingPipeline MUST NEVER import from simulation/ or scene modules.
    - All frame ingestion is channelled exclusively through this interface.
    - Mode A (SimulatorSource) and Mode B (VideoFileSource) are interchangeable
      at runtime — the tracking core cannot distinguish them.

Mode A — Synthetic simulator frames with closed-loop virtual gimbal PTZ feedback.
Mode B — External evaluator-supplied .mp4 ingestion (Benchmark-2 compliance).
         PTZ loop is cleanly bypassed; tracking operates on raw video frames.

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import abc
import logging
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Abstract Base
# ---------------------------------------------------------------------------

class FrameSource(abc.ABC):
    """
    Abstract base class for all frame ingestion sources.

    Implementors must provide frames as greyscale or BGR numpy arrays.
    The tracking pipeline calls only this interface — never simulator internals.
    """

    @abc.abstractmethod
    def read(self) -> tuple[bool, np.ndarray]:
        """
        Read the next frame.

        Returns
        -------
        ok : bool
            True if a valid frame was retrieved; False on EOF or error.
        frame : np.ndarray
            HxW (greyscale) or HxWx3 (BGR) uint8 array.
            Shape is guaranteed to match get_resolution() when ok is True.
        """

    @abc.abstractmethod
    def get_fps(self) -> float:
        """Nominal frames-per-second of this source."""

    @abc.abstractmethod
    def get_resolution(self) -> tuple[int, int]:
        """
        Return (width, height) in pixels.

        Note: width × height follows OpenCV convention (cols × rows).
        """

    def release(self) -> None:
        """Optional cleanup — override if the source holds resources."""

    # ------------------------------------------------------------------ helpers
    def __repr__(self) -> str:
        w, h = self.get_resolution()
        return f"{self.__class__.__name__}(res={w}x{h}, fps={self.get_fps():.1f})"


# ---------------------------------------------------------------------------
# Mode A — Simulator Source (stub, fully implemented in simulation/)
# ---------------------------------------------------------------------------

class SimulatorSource(FrameSource):
    """
    Mode A: Wraps the synthetic 2000×2000 disturbance scene generator.

    The virtual gimbal PTZ feedback loop is active in this mode.
    The simulator pushes pre-rendered 640×480 viewport frames into the
    tracking pipeline.  This stub is replaced by the full implementation
    in Phase 2 (simulation/scene.py integration).

    Parameters
    ----------
    width, height : int
        Camera viewport resolution (default: 640×480 per ISRO spec).
    fps : float
        Target update rate (must be ≥ 30 Hz per spec).
    """

    # ISRO-specified defaults
    DEFAULT_WIDTH: int = 640
    DEFAULT_HEIGHT: int = 480
    DEFAULT_FPS: float = 30.0

    def __init__(
        self,
        width: int = DEFAULT_WIDTH,
        height: int = DEFAULT_HEIGHT,
        fps: float = DEFAULT_FPS,
    ) -> None:
        if fps < 30.0:
            raise ValueError(f"Camera update rate must be ≥ 30 Hz (got {fps})")
        self._width = width
        self._height = height
        self._fps = fps
        self._frame_count: int = 0
        logger.info("SimulatorSource initialised (stub) — %dx%d @ %.1f Hz", width, height, fps)

    # ------------------------------------------------------------------
    def read(self) -> tuple[bool, np.ndarray]:
        """
        Stub: returns a blank (black) greyscale frame.
        Replaced in Phase 2 when simulation/scene.py is wired in.
        """
        frame = np.zeros((self._height, self._width), dtype=np.uint8)
        self._frame_count += 1
        return True, frame

    def get_fps(self) -> float:
        return self._fps

    def get_resolution(self) -> tuple[int, int]:
        return self._width, self._height

    @property
    def frame_count(self) -> int:
        """Total frames produced since construction."""
        return self._frame_count


# ---------------------------------------------------------------------------
# Mode B — Video File Source (Benchmark-2 compliance)
# ---------------------------------------------------------------------------

class VideoFileSource(FrameSource):
    """
    Mode B: Wraps cv2.VideoCapture for evaluator-supplied .mp4 files.

    BENCHMARK-2 COMPLIANCE NOTES:
    - The PTZ / virtual camera loop is completely bypassed in this mode.
    - If the evaluator's video resolution differs from 640×480, a scale
      factor is recorded so centroid errors can be reported in both
      native-pixel and normalised coordinate systems.
    - Centroid errors are logged in both pixel spaces (see §7, blueprint).

    Parameters
    ----------
    path : str | Path
        Path to the .mp4 (or any OpenCV-readable video) file.
    target_width, target_height : int | None
        If provided, frames are resized to this resolution and the
        `scale_factor` attribute is populated for error normalisation.
        Leave None to use native video resolution.
    greyscale : bool
        Convert frames to greyscale before returning (default True).
    """

    SPEC_WIDTH: int = 640
    SPEC_HEIGHT: int = 480

    def __init__(
        self,
        path: str | Path,
        target_width: Optional[int] = None,
        target_height: Optional[int] = None,
        greyscale: bool = True,
    ) -> None:
        self._path = Path(path)
        if not self._path.exists():
            raise FileNotFoundError(f"Video file not found: {self._path}")

        self._cap = cv2.VideoCapture(str(self._path))
        if not self._cap.isOpened():
            raise RuntimeError(f"cv2.VideoCapture failed to open: {self._path}")

        # Native properties
        self._native_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self._native_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._fps = float(self._cap.get(cv2.CAP_PROP_FPS)) or 30.0
        self._total_frames = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT))

        # Resize target
        self._target_w = target_width or self._native_w
        self._target_h = target_height or self._native_h
        self._greyscale = greyscale

        # Scale factors for centroid error normalisation (blueprint §7)
        self.scale_x: float = self._target_w / self._native_w if self._native_w else 1.0
        self.scale_y: float = self._target_h / self._native_h if self._native_h else 1.0
        self._needs_resize: bool = (
            self._target_w != self._native_w or self._target_h != self._native_h
        )

        logger.info(
            "VideoFileSource opened: %s | native=%dx%d | target=%dx%d | fps=%.2f | frames=%d",
            self._path.name,
            self._native_w, self._native_h,
            self._target_w, self._target_h,
            self._fps, self._total_frames,
        )
        if self._needs_resize:
            logger.warning(
                "VideoFileSource: native resolution %dx%d ≠ target %dx%d — "
                "scale factors (sx=%.4f, sy=%.4f) will be applied to centroid errors.",
                self._native_w, self._native_h,
                self._target_w, self._target_h,
                self.scale_x, self.scale_y,
            )

    # ------------------------------------------------------------------
    def read(self) -> tuple[bool, np.ndarray]:
        ok, frame = self._cap.read()
        if not ok:
            return False, np.empty((0,), dtype=np.uint8)

        if self._needs_resize:
            frame = cv2.resize(
                frame, (self._target_w, self._target_h), interpolation=cv2.INTER_LINEAR
            )
        if self._greyscale and frame.ndim == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        return True, frame

    def get_fps(self) -> float:
        return self._fps

    def get_resolution(self) -> tuple[int, int]:
        return self._target_w, self._target_h

    def release(self) -> None:
        if self._cap.isOpened():
            self._cap.release()
            logger.info("VideoFileSource released: %s", self._path.name)

    @property
    def total_frames(self) -> int:
        return self._total_frames

    @property
    def native_resolution(self) -> tuple[int, int]:
        """Original video resolution before any resizing."""
        return self._native_w, self._native_h

    def __del__(self) -> None:
        if hasattr(self, "_cap"):
            self.release()
