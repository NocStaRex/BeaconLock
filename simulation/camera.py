"""
simulation/camera.py
====================
Virtual Pan-Tilt Camera for BeaconLock.

Extracts a 640×480 viewport from a 2000×2000 world canvas and integrates
rate-limited pan/tilt control commands.

Coordinate conventions
----------------------
- World coords:    (wx, wy) ∈ [0, 2000), origin top-left.
- Camera centre:   (cx_world, cy_world) — where the camera is pointing.
- Camera frame:    (col, row) ∈ [0, 640) × [0, 480), origin viewport top-left.
- Frame centre:    (320, 240) in camera frame coords.

World ↔ Camera transform:
    col = wx - (cx_world - FRAME_W/2)  →  col = wx - cx_world + 320
    row = wy - (cy_world - FRAME_H/2)  →  row = wy - cy_world + 240

Pan/tilt convention:
    Positive pan  → camera moves RIGHT  → beacon appears to move LEFT in frame.
    Positive tilt → camera moves DOWN   → beacon appears to move UP in frame.

Rate limits (ISRO PS-26169 §3):
    Max pan/tilt speed: 5–10 °/s (default 5 °/s)
    Angular resolution: 0.00625 °/px (4° FOV / 640 px)

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import logging
from typing import Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# ISRO-specified constants
WORLD_W: int   = 2000
WORLD_H: int   = 2000
FRAME_W: int   = 640
FRAME_H: int   = 480
FRAME_CX: int  = FRAME_W // 2    # 320
FRAME_CY: int  = FRAME_H // 2    # 240

# Angular resolution (px → deg)
DEG_PER_PX: float = 4.0 / FRAME_W   # 0.00625 °/px (horizontal = vertical)

# Default rate limit: 5 °/s (ISRO spec: 5–10 °/s)
DEFAULT_MAX_RATE_DEG_S: float = 5.0

# Camera position bounds (keeps viewport fully within canvas)
CAM_X_MIN: float = float(FRAME_CX)           # 320
CAM_X_MAX: float = float(WORLD_W - FRAME_CX) # 1680
CAM_Y_MIN: float = float(FRAME_CY)           # 240
CAM_Y_MAX: float = float(WORLD_H - FRAME_CY) # 1760


class VirtualCamera:
    """
    Virtual pan-tilt camera: rate-limited position integration + viewport crop.

    Parameters
    ----------
    init_x, init_y : float
        Initial camera centre in world coords (default: 1000, 1000 = canvas centre).
    max_rate_deg_s : float
        Pan/tilt rate limit in °/s (ISRO: 5–10 °/s).
    """

    def __init__(
        self,
        init_x: float = 1000.0,
        init_y: float = 1000.0,
        max_rate_deg_s: float = DEFAULT_MAX_RATE_DEG_S,
    ) -> None:
        self._cx: float = float(np.clip(init_x, CAM_X_MIN, CAM_X_MAX))
        self._cy: float = float(np.clip(init_y, CAM_Y_MIN, CAM_Y_MAX))
        self.max_rate = max_rate_deg_s

        # Accumulated pan/tilt angles from initial position
        self._pan_deg: float = 0.0
        self._tilt_deg: float = 0.0

        # Diagnostics
        self._last_pan_applied: float = 0.0
        self._last_tilt_applied: float = 0.0
        self._n_saturated: int = 0

        logger.debug(
            "VirtualCamera initialised at world (%.1f, %.1f) | max_rate=%.1f°/s",
            init_x, init_y, max_rate_deg_s,
        )

    # ------------------------------------------------------------------
    # Control actuation
    # ------------------------------------------------------------------

    def apply_control(
        self,
        pan_rate_deg_s: float,
        tilt_rate_deg_s: float,
        dt: float = 1.0 / 30.0,
    ) -> None:
        """
        Integrate rate commands into camera position for one timestep.

        Parameters
        ----------
        pan_rate_deg_s  : float  Commanded pan  rate (°/s). +ve = pan right.
        tilt_rate_deg_s : float  Commanded tilt rate (°/s). +ve = tilt down.
        dt              : float  Timestep (seconds). Default 1/30 s.
        """
        # Clamp to rate limit (hardware saturation)
        clamped_pan  = float(np.clip(pan_rate_deg_s,  -self.max_rate, self.max_rate))
        clamped_tilt = float(np.clip(tilt_rate_deg_s, -self.max_rate, self.max_rate))

        saturated = (abs(pan_rate_deg_s) > self.max_rate
                     or abs(tilt_rate_deg_s) > self.max_rate)
        if saturated:
            self._n_saturated += 1

        # Convert deg/s → px/s via angular resolution, then integrate
        px_per_s = 1.0 / DEG_PER_PX       # = 160 px per degree
        dx = clamped_pan  * px_per_s * dt  # px moved this frame (horizontal)
        dy = clamped_tilt * px_per_s * dt  # px moved this frame (vertical)

        # Update camera centre, bounded to valid viewport range
        self._cx = float(np.clip(self._cx + dx, CAM_X_MIN, CAM_X_MAX))
        self._cy = float(np.clip(self._cy + dy, CAM_Y_MIN, CAM_Y_MAX))

        # Track cumulative angles for telemetry
        self._pan_deg  += clamped_pan  * dt
        self._tilt_deg += clamped_tilt * dt

        self._last_pan_applied  = clamped_pan
        self._last_tilt_applied = clamped_tilt

    # ------------------------------------------------------------------
    # Viewport extraction
    # ------------------------------------------------------------------

    def get_viewport(
        self,
        world_canvas: np.ndarray,
        jitter_dx: float = 0.0,
        jitter_dy: float = 0.0,
    ) -> np.ndarray:
        """
        Extract 640×480 camera viewport from the world canvas.

        Parameters
        ----------
        world_canvas : np.ndarray
            2000×2000 (or HxW) greyscale canvas.
        jitter_dx, jitter_dy : float
            Camera jitter displacement (px) for this frame.
            Applied as an additional viewport origin offset.

        Returns
        -------
        viewport : np.ndarray
            640×480 uint8 greyscale image.
        """
        h, w = world_canvas.shape[:2]

        # Compute viewport origin with jitter applied
        origin_x = int(round(self._cx - FRAME_CX + jitter_dx))
        origin_y = int(round(self._cy - FRAME_CY + jitter_dy))

        # Clamp origin so viewport stays within canvas
        origin_x = int(np.clip(origin_x, 0, w - FRAME_W))
        origin_y = int(np.clip(origin_y, 0, h - FRAME_H))

        crop = world_canvas[origin_y:origin_y + FRAME_H, origin_x:origin_x + FRAME_W]

        # Handle edge case where crop is smaller than viewport (shouldn't happen
        # after clamping, but defensive resize just in case)
        if crop.shape[0] != FRAME_H or crop.shape[1] != FRAME_W:
            crop = cv2.resize(crop, (FRAME_W, FRAME_H), interpolation=cv2.INTER_LINEAR)

        return crop.astype(np.uint8)

    # ------------------------------------------------------------------
    # Coordinate transforms
    # ------------------------------------------------------------------

    def world_to_camera(self, wx: float, wy: float) -> tuple[float, float]:
        """
        Convert world coordinates to camera frame coordinates.

        Returns (col, row) in [0, 640) × [0, 480). May be outside if target
        is not in the current viewport.
        """
        col = wx - self._cx + FRAME_CX
        row = wy - self._cy + FRAME_CY
        return col, row

    def camera_to_world(self, col: float, row: float) -> tuple[float, float]:
        """Convert camera frame coordinates to world coordinates."""
        wx = col + self._cx - FRAME_CX
        wy = row + self._cy - FRAME_CY
        return wx, wy

    def is_in_viewport(self, wx: float, wy: float) -> bool:
        """Check if world point (wx, wy) is within the current viewport."""
        col, row = self.world_to_camera(wx, wy)
        return 0 <= col < FRAME_W and 0 <= row < FRAME_H

    # ------------------------------------------------------------------
    # Properties / diagnostics
    # ------------------------------------------------------------------

    @property
    def position(self) -> tuple[float, float]:
        """Camera centre in world coordinates."""
        return self._cx, self._cy

    @property
    def pan_deg(self) -> float:
        """Cumulative pan angle from initial position (degrees)."""
        return self._pan_deg

    @property
    def tilt_deg(self) -> float:
        """Cumulative tilt angle from initial position (degrees)."""
        return self._tilt_deg

    @property
    def last_command(self) -> tuple[float, float]:
        """(pan_rate, tilt_rate) actually applied last frame (°/s)."""
        return self._last_pan_applied, self._last_tilt_applied

    @property
    def saturation_count(self) -> int:
        """Total number of frames where rate saturation occurred."""
        return self._n_saturated

    def reset(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
    ) -> None:
        """Reset camera to given world position (or initial centre)."""
        if x is not None:
            self._cx = float(np.clip(x, CAM_X_MIN, CAM_X_MAX))
        if y is not None:
            self._cy = float(np.clip(y, CAM_Y_MIN, CAM_Y_MAX))
        self._pan_deg  = 0.0
        self._tilt_deg = 0.0
        self._n_saturated = 0
