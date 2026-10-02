"""
simulation/scene.py
===================
WorldScene — renders the 2000×2000 world canvas each frame.

Responsibilities
----------------
- Maintains the persistent world canvas (dark background).
- Places beacon spot(s) from BeaconTarget instances.
- Provides the raw canvas to the camera viewport extractor.

This module is intentionally minimal — it is purely a renderer.
All physics (motion, disturbances) are handled by their own modules.

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import numpy as np

from simulation.target import BeaconTarget

WORLD_W: int = 2000
WORLD_H: int = 2000


class WorldScene:
    """
    2000×2000 greyscale world canvas renderer.

    Parameters
    ----------
    background : int
        Background pixel value (0–255). Default 0 (black).
    targets : list[BeaconTarget] | None
        Beacon targets managed by this scene.
    """

    def __init__(
        self,
        background: int = 0,
        targets: list[BeaconTarget] | None = None,
    ) -> None:
        self._background = int(max(0, min(255, background)))
        self._targets: list[BeaconTarget] = targets or []
        # Persistent canvas buffer (reused every frame to avoid allocation)
        self._canvas = np.full((WORLD_H, WORLD_W), self._background, dtype=np.uint8)

    def add_target(self, target: BeaconTarget) -> None:
        self._targets.append(target)

    def render(self) -> np.ndarray:
        """
        Render the world canvas for the current frame.

        Returns
        -------
        canvas : np.ndarray  (WORLD_H × WORLD_W) uint8 — DO NOT MODIFY the
                 returned array; it is a view of the internal buffer. Copy if needed.
        """
        # Clear to background
        self._canvas[:] = self._background

        # Render all beacon targets
        for t in self._targets:
            t.render(self._canvas)

        return self._canvas

    @property
    def targets(self) -> list[BeaconTarget]:
        return self._targets
