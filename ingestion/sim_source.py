"""
ingestion/sim_source.py
=======================
Full SimulatorSource — Mode A FrameSource implementation.

This module is the ONLY place that imports from simulation/.
The TrackingPipeline (perception/ + control/) never touches simulation/.
The decoupled architecture (Benchmark-2 compliance) is preserved.

Pipeline per read() call
------------------------
1. Advance beacon target position (motion model step).
2. Render 2000×2000 world canvas.
3. Apply platform motion to camera position.
4. Extract 640×480 viewport (with camera jitter offset).
5. Apply atmospheric attenuation + image noise to viewport.
6. Return (True, distorted_viewport).

Ground-truth exposure (evaluation layer only)
---------------------------------------------
sim_source.ground_truth_camera : (col, row) beacon position in camera frame,
    WITHOUT jitter, for RMSE computation.  The TrackingPipeline must NOT access
    this attribute — it is for evaluation/test code only.

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ingestion.frame_source import FrameSource
from simulation.camera import VirtualCamera, FRAME_W, FRAME_H
from simulation.disturbances import DisturbanceConfig
from simulation.motion_models import MotionModel, MotionType, create_motion_model
from simulation.scene import WorldScene
from simulation.target import BeaconTarget

logger = logging.getLogger(__name__)


@dataclass
class GroundTruth:
    """Per-frame ground truth for evaluation/benchmarking."""
    world_x: float = 0.0
    world_y: float = 0.0
    cam_col: float = 0.0      # beacon column in camera frame (no jitter)
    cam_row: float = 0.0      # beacon row in camera frame (no jitter)
    in_viewport: bool = True
    occluded: bool = False
    frame_id: int = 0


class SimulatorSource(FrameSource):
    """
    Mode A FrameSource — fully synthetic simulation feed.

    Parameters
    ----------
    motion_type : str | MotionType
        Beacon motion mode. Default 'circular'.
    motion_kwargs : dict | None
        Extra kwargs for the motion model constructor.
    beacon_size : int
        Beacon square side length in px (5–20). Default 10.
    disturbance : DisturbanceConfig | None
        Disturbance configuration bundle. None = no disturbances.
    camera_init_x, camera_init_y : float
        Initial camera centre in world coords (default: 1000, 1000).
    max_rate_deg_s : float
        Pan/tilt rate limit (5–10 °/s per ISRO spec).
    fps : float
        Nominal simulation framerate (≥ 30 Hz).
    seed : int | None
        RNG seed for reproducibility.
    """

    def __init__(
        self,
        motion_type: str | MotionType = "circular",
        motion_kwargs: Optional[dict] = None,
        beacon_size: int = 10,
        disturbance: Optional[DisturbanceConfig] = None,
        camera_init_x: float = 1000.0,
        camera_init_y: float = 1000.0,
        max_rate_deg_s: float = 5.0,
        fps: float = 30.0,
        seed: Optional[int] = None,
    ) -> None:
        if fps < 30.0:
            raise ValueError(f"FPS must be ≥ 30 Hz (got {fps})")

        self._fps = fps
        self._dt  = 1.0 / fps
        self._frame_id = 0

        # Build motion model & beacon
        motion_model = create_motion_model(
            motion_type,
            center_x=camera_init_x,
            center_y=camera_init_y,
            **(motion_kwargs or {}),
        )
        self._target = BeaconTarget(
            motion_model=motion_model,
            size=beacon_size,
        )

        # Scene renderer
        self._scene = WorldScene(background=0, targets=[self._target])

        # Virtual pan-tilt camera
        self._camera = VirtualCamera(
            init_x=camera_init_x,
            init_y=camera_init_y,
            max_rate_deg_s=max_rate_deg_s,
        )

        # Disturbance engine (no-disturbance default)
        self._dist = disturbance or DisturbanceConfig()

        # Ground truth (updated every read())
        self._gt = GroundTruth()

        logger.info(
            "SimulatorSource ready — motion=%s size=%dpx fps=%.0f dist=%s",
            motion_type, beacon_size, fps,
            type(disturbance).__name__ if disturbance else "none",
        )

    # ------------------------------------------------------------------
    # FrameSource interface
    # ------------------------------------------------------------------

    def read(self) -> tuple[bool, np.ndarray]:
        """
        Generate and return the next 640×480 simulation frame.

        Returns
        -------
        (True, frame) : frame is a greyscale uint8 ndarray (480, 640).
        """
        # 1. Advance platform motion (updates camera position)
        pdx, pdy = self._dist.platform_step(self._dt)
        # Apply platform drift to camera (camera moves with platform)
        if pdx != 0.0 or pdy != 0.0:
            cx, cy = self._camera.position
            self._camera.reset(cx + pdx, cy + pdy)

        # 2. Advance beacon position
        wx, wy = self._target.update(self._dt)

        # 3. Record ground truth (no jitter — ideal camera coords)
        cam_col, cam_row = self._camera.world_to_camera(wx, wy)
        self._gt = GroundTruth(
            world_x=wx,
            world_y=wy,
            cam_col=cam_col,
            cam_row=cam_row,
            in_viewport=self._camera.is_in_viewport(wx, wy),
            occluded=self._target.is_occluded,
            frame_id=self._frame_id,
        )

        # 4. Render world canvas
        canvas = self._scene.render()

        # 5. Extract viewport with camera jitter offset, then apply image disturbances
        jx, jy = self._dist.jitter.sample()                     # sample jitter for this frame
        raw_viewport = self._camera.get_viewport(canvas, jitter_dx=jx, jitter_dy=jy)
        distorted_viewport = self._dist.apply_to_frame(raw_viewport)  # returns ndarray directly

        self._frame_id += 1
        return True, distorted_viewport

    def get_fps(self) -> float:
        return self._fps

    def get_resolution(self) -> tuple[int, int]:
        return FRAME_W, FRAME_H

    # ------------------------------------------------------------------
    # Closed-loop camera control (Mode A only)
    # ------------------------------------------------------------------

    def apply_camera_control(
        self,
        pan_rate_deg_s: float,
        tilt_rate_deg_s: float,
        dt: Optional[float] = None,
    ) -> None:
        """
        Feed tracker output back to the virtual camera.

        Called by the closed-loop after each read() + TrackingPipeline step.
        The camera integrates the rate commands and updates its world position
        for the NEXT read() call.

        Parameters
        ----------
        pan_rate_deg_s  : float  Pan  rate command from PDController (°/s).
        tilt_rate_deg_s : float  Tilt rate command from PDController (°/s).
        dt : float | None  Timestep — defaults to 1/fps.
        """
        self._camera.apply_control(
            pan_rate_deg_s,
            tilt_rate_deg_s,
            dt=dt if dt is not None else self._dt,
        )

    # ------------------------------------------------------------------
    # Ground truth & diagnostics (evaluation layer only — NOT tracking)
    # ------------------------------------------------------------------

    @property
    def ground_truth(self) -> GroundTruth:
        """
        Last frame ground truth (beacon position in camera frame, no jitter).

        ⚠ Evaluation / test code ONLY.  The TrackingPipeline must NOT use this.
        """
        return self._gt

    @property
    def ground_truth_camera(self) -> tuple[float, float]:
        """Shorthand: (cam_col, cam_row) of beacon in last frame."""
        return self._gt.cam_col, self._gt.cam_row

    @property
    def camera(self) -> VirtualCamera:
        return self._camera

    @property
    def target(self) -> BeaconTarget:
        return self._target

    @property
    def frame_id(self) -> int:
        return self._frame_id

    def inject_occlusion(self, duration_frames: int) -> None:
        """Delegate to beacon target for occlusion testing."""
        self._target.inject_occlusion(duration_frames)
