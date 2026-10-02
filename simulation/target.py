"""
simulation/target.py
====================
BeaconTarget — manages beacon position, rendering, and occlusion injection.

Responsibilities
----------------
- Maintains beacon world position via a MotionModel delegate.
- Renders the beacon as a configurable square spot onto a world canvas.
- Supports temporary occlusion injection (for Kalman coasting tests).

Beacon spot specification (ISRO PS-26169 §3):
  Shape  : Square (default)
  Size   : 5–20 × 5–20 px (default 10 × 10)
  Brightness : 255 (monochrome, saturated)

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import logging
from typing import Optional

import numpy as np

from simulation.motion_models import (
    MotionModel,
    MotionType,
    create_motion_model,
    WORLD_W,
    WORLD_H,
)

logger = logging.getLogger(__name__)


class BeaconTarget:
    """
    Beacon spot target that moves on a 2000×2000 world canvas.

    Parameters
    ----------
    motion_model : MotionModel | str
        Pre-constructed MotionModel instance, or a string like 'circular'.
        If a string, motion is created with `center_x/y` as origin.
    center_x, center_y : float
        Logical centre for the trajectory (used when motion_model is a string).
    size : int
        Beacon square side length in pixels (5–20). Default 10.
    brightness : int
        Peak pixel intensity 0–255. Default 255.
    motion_kwargs : dict
        Extra kwargs forwarded to create_motion_model() when constructing from string.
    """

    MIN_SIZE: int = 5
    MAX_SIZE: int = 20

    def __init__(
        self,
        motion_model: MotionModel | str = "straight_line",
        center_x: float = 1000.0,
        center_y: float = 1000.0,
        size: int = 10,
        brightness: int = 255,
        motion_kwargs: Optional[dict] = None,
    ) -> None:
        # Resolve motion model
        if isinstance(motion_model, str):
            self._motion = create_motion_model(
                motion_model, center_x, center_y, **(motion_kwargs or {})
            )
        elif isinstance(motion_model, MotionModel):
            self._motion = motion_model
        else:
            raise TypeError(f"motion_model must be a MotionModel or string, got {type(motion_model)}")

        # Validate and store beacon parameters
        if not (self.MIN_SIZE <= size <= self.MAX_SIZE):
            logger.warning("Beacon size %d outside spec [5,20]; clamping.", size)
            size = max(self.MIN_SIZE, min(self.MAX_SIZE, size))
        self._size = size
        self._brightness = int(max(0, min(255, brightness)))

        # Occlusion state
        self._occluded: bool = False
        self._occlude_frames_remaining: int = 0

        # Cache initial position
        self._x, self._y = self._motion.position

        logger.debug(
            "BeaconTarget created — size=%dpx, brightness=%d, motion=%s",
            size, brightness, type(self._motion).__name__,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(self, dt: float) -> tuple[float, float]:
        """
        Advance beacon position by dt seconds.

        Returns
        -------
        (world_x, world_y) after update.  Position advances even when occluded
        (beacon keeps moving behind the occluder; tracker must coast via Kalman).
        """
        # Advance motion model regardless of occlusion
        self._x, self._y = self._motion.update(dt)
        return self._x, self._y

    def render(self, canvas: np.ndarray) -> None:
        """
        Draw the beacon square onto *canvas* in-place.

        Occlusion counter is managed here (post-render) so exactly N render
        calls are dark when inject_occlusion(N) is used.

        Parameters
        ----------
        canvas : np.ndarray
            Greyscale world canvas (WORLD_H × WORLD_W, uint8 or float32).
        """
        if self._occluded:
            # Decrement counter AFTER this frame is confirmed blank,
            # so N calls to render() produce exactly N dark frames.
            if self._occlude_frames_remaining > 0:
                self._occlude_frames_remaining -= 1
            if self._occlude_frames_remaining == 0:
                self._occluded = False   # next render will be visible
                logger.debug("Occlusion lifted.")
            return   # beacon is blanked this frame

        h, w = canvas.shape[:2]
        half = self._size // 2

        x0 = int(round(self._x)) - half
        y0 = int(round(self._y)) - half
        x1 = x0 + self._size
        y1 = y0 + self._size

        # Clamp to canvas bounds
        x0c = max(0, x0)
        y0c = max(0, y0)
        x1c = min(w, x1)
        y1c = min(h, y1)

        if x0c < x1c and y0c < y1c:
            canvas[y0c:y1c, x0c:x1c] = self._brightness

    def inject_occlusion(self, duration_frames: int) -> None:
        """
        Blank the beacon for *duration_frames* frames.

        Used to test Kalman coasting / LOST-state re-acquisition.
        The beacon continues moving while occluded (behind an occluder).

        Parameters
        ----------
        duration_frames : int
            Number of frames for which the beacon will not be rendered.
        """
        self._occluded = True
        self._occlude_frames_remaining = max(1, duration_frames)
        logger.debug("Occlusion injected: %d frames.", duration_frames)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def position(self) -> tuple[float, float]:
        """Current beacon world position (x, y) in pixels."""
        return self._x, self._y

    @property
    def size(self) -> int:
        return self._size

    @size.setter
    def size(self, value: int) -> None:
        if not (self.MIN_SIZE <= value <= self.MAX_SIZE):
            raise ValueError(f"Beacon size {value} outside [5, 20]")
        self._size = value

    @property
    def brightness(self) -> int:
        return self._brightness

    @property
    def is_occluded(self) -> bool:
        return self._occluded

    @property
    def motion_model(self) -> MotionModel:
        return self._motion
