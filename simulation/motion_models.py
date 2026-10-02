"""
simulation/motion_models.py
============================
All 4 mandatory beacon motion models for BeaconLock.

Coordinate system: World canvas (2000 × 2000 px). Origin top-left.
All positions are (x, y) floats within [0, 2000).

Motion models are stateful objects — call update(dt) each frame to advance
the internal clock and retrieve the new (x, y) position.

Models
------
StraightLineMotion  — Constant velocity with configurable boundary reflection/wrap.
CircularMotion      — Orbital path at fixed radius around a configurable centre.
Figure8Motion       — Lissajous figure-8: x=A·sin(ωt), y=B·sin(2ωt+φ).
RandomWalkMotion    — Ornstein–Uhlenbeck smooth random walk with bounded velocity.

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import abc
import math
import random
from enum import Enum, auto
from typing import Optional

import numpy as np

# World canvas bounds (px)
WORLD_W: int = 2000
WORLD_H: int = 2000


class MotionType(Enum):
    """String-to-enum mapping for motion model selection."""
    STRAIGHT_LINE = auto()
    CIRCULAR      = auto()
    FIGURE_8      = auto()
    RANDOM_WALK   = auto()

    @classmethod
    def from_string(cls, s: str) -> "MotionType":
        mapping = {
            "straight_line": cls.STRAIGHT_LINE,
            "circular":      cls.CIRCULAR,
            "figure_8":      cls.FIGURE_8,
            "figure8":       cls.FIGURE_8,
            "random_walk":   cls.RANDOM_WALK,
            "random":        cls.RANDOM_WALK,
        }
        key = s.lower().strip()
        if key not in mapping:
            raise ValueError(f"Unknown motion type '{s}'. Valid: {list(mapping.keys())}")
        return mapping[key]


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------

class MotionModel(abc.ABC):
    """Abstract base for all beacon motion models."""

    def __init__(self, start_x: float, start_y: float) -> None:
        self._x = float(start_x)
        self._y = float(start_y)
        self._t = 0.0          # accumulated time (seconds)

    @abc.abstractmethod
    def update(self, dt: float) -> tuple[float, float]:
        """
        Advance internal state by dt seconds.

        Returns
        -------
        (x, y) : beacon position in world coordinates after the update.
        """

    def reset(self, x: Optional[float] = None, y: Optional[float] = None) -> None:
        """Reset position (and time) to given coords (or initial position)."""
        if x is not None:
            self._x = float(x)
        if y is not None:
            self._y = float(y)
        self._t = 0.0

    @property
    def position(self) -> tuple[float, float]:
        return self._x, self._y

    @property
    def elapsed_time(self) -> float:
        return self._t

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    @staticmethod
    def _reflect(v: float, lo: float, hi: float, vel: float) -> tuple[float, float]:
        """Reflect v off boundary [lo, hi]; returns (new_v, new_vel)."""
        if v < lo:
            return 2 * lo - v, -vel
        if v > hi:
            return 2 * hi - v, -vel
        return v, vel


# ---------------------------------------------------------------------------
# 1. Straight-Line Motion
# ---------------------------------------------------------------------------

class StraightLineMotion(MotionModel):
    """
    Constant-velocity motion with boundary reflection.

    Parameters
    ----------
    start_x, start_y : float
        Initial position (world px).
    vx, vy : float
        Velocity in px/s.  Default 80 px/s horizontal.
    reflect : bool
        If True (default), velocity reverses at canvas edges.
        If False, position wraps around (torus topology).
    margin : float
        Boundary margin (px) from canvas edge.
    """

    def __init__(
        self,
        start_x: float = 500.0,
        start_y: float = 1000.0,
        vx: float = 80.0,
        vy: float = 0.0,
        reflect: bool = True,
        margin: float = 50.0,
    ) -> None:
        super().__init__(start_x, start_y)
        self._vx = vx
        self._vy = vy
        self._do_reflect = reflect        # renamed: avoid collision with _reflect() staticmethod
        self._margin = margin
        self._x_lo = margin
        self._x_hi = WORLD_W - margin
        self._y_lo = margin
        self._y_hi = WORLD_H - margin

    def update(self, dt: float) -> tuple[float, float]:
        self._t += dt
        self._x += self._vx * dt
        self._y += self._vy * dt

        if self._do_reflect:
            self._x, self._vx = self._reflect(self._x, self._x_lo, self._x_hi, self._vx)
            self._y, self._vy = self._reflect(self._y, self._y_lo, self._y_hi, self._vy)
        else:
            # Wrap (torus)
            self._x = self._x_lo + (self._x - self._x_lo) % (self._x_hi - self._x_lo)
            self._y = self._y_lo + (self._y - self._y_lo) % (self._y_hi - self._y_lo)

        return self._x, self._y


# ---------------------------------------------------------------------------
# 2. Circular (Orbital) Motion
# ---------------------------------------------------------------------------

class CircularMotion(MotionModel):
    """
    Orbital circular trajectory around a fixed centre.

    Parameters
    ----------
    center_x, center_y : float
        Orbit centre in world coordinates.
    radius : float
        Orbit radius in px (spec: 200–600 px).
    omega : float
        Angular velocity in rad/s (positive = counter-clockwise).
    start_angle : float
        Initial angle in radians (0 = right of centre).
    """

    def __init__(
        self,
        center_x: float = 1000.0,
        center_y: float = 1000.0,
        radius: float = 300.0,
        omega: float = 0.5,
        start_angle: float = 0.0,
    ) -> None:
        x0 = center_x + radius * math.cos(start_angle)
        y0 = center_y + radius * math.sin(start_angle)
        super().__init__(x0, y0)
        self._cx = center_x
        self._cy = center_y
        self._radius = radius
        self._omega = omega
        self._angle = start_angle

    def update(self, dt: float) -> tuple[float, float]:
        self._t += dt
        self._angle += self._omega * dt
        self._x = self._cx + self._radius * math.cos(self._angle)
        self._y = self._cy + self._radius * math.sin(self._angle)
        return self._x, self._y

    @property
    def angle(self) -> float:
        return self._angle


# ---------------------------------------------------------------------------
# 3. Figure-8 (Lissajous) Motion
# ---------------------------------------------------------------------------

class Figure8Motion(MotionModel):
    """
    Figure-8 via Lissajous parametric equations:
        x(t) = cx + Ax · sin(ω·t + φx)
        y(t) = cy + Ay · sin(2ω·t + φy)

    The 2:1 frequency ratio produces a figure-8. Continuously changing
    acceleration vectors stress-test the Kalman 6-state model.

    Parameters
    ----------
    center_x, center_y : float
        Centre of the figure-8 in world coords.
    amp_x, amp_y : float
        Horizontal / vertical amplitude in px.
    omega : float
        Base angular velocity (rad/s). Horizontal period = 2π/ω.
    phi_y : float
        Phase offset for y oscillation (radians). π/2 gives symmetric figure-8.
    """

    def __init__(
        self,
        center_x: float = 1000.0,
        center_y: float = 1000.0,
        amp_x: float = 350.0,
        amp_y: float = 175.0,
        omega: float = 0.4,
        phi_y: float = math.pi / 2,
    ) -> None:
        x0 = center_x + amp_x * math.sin(0.0)
        y0 = center_y + amp_y * math.sin(phi_y)
        super().__init__(x0, y0)
        self._cx = center_x
        self._cy = center_y
        self._amp_x = amp_x
        self._amp_y = amp_y
        self._omega = omega
        self._phi_y = phi_y

    def update(self, dt: float) -> tuple[float, float]:
        self._t += dt
        self._x = self._cx + self._amp_x * math.sin(self._omega * self._t)
        self._y = self._cy + self._amp_y * math.sin(2 * self._omega * self._t + self._phi_y)
        return self._x, self._y

    @property
    def max_speed_px_s(self) -> float:
        """Theoretical maximum speed (px/s) — used for slew rate planning."""
        return max(self._amp_x * self._omega, 2 * self._amp_y * self._omega)


# ---------------------------------------------------------------------------
# 4. Random Walk Motion (Ornstein–Uhlenbeck)
# ---------------------------------------------------------------------------

class RandomWalkMotion(MotionModel):
    """
    Smooth bounded random walk using an Ornstein–Uhlenbeck (OU) process.

    The OU process produces smooth, mean-reverting trajectories that avoid
    leaving the canvas while still being unpredictable. Velocity is bounded
    to ±max_speed px/s at all times.

    Parameters
    ----------
    start_x, start_y : float
        Initial position.
    max_speed : float
        Maximum speed in px/s (clamp applied each step).
    theta : float
        Mean-reversion rate (toward canvas centre). Higher = tighter confinement.
    sigma : float
        Noise diffusion coefficient controlling randomness.
    seed : int | None
        Optional RNG seed for reproducibility.
    """

    def __init__(
        self,
        start_x: float = 1000.0,
        start_y: float = 1000.0,
        max_speed: float = 120.0,
        theta: float = 0.15,
        sigma: float = 60.0,
        seed: Optional[int] = None,
    ) -> None:
        super().__init__(start_x, start_y)
        self._max_speed = max_speed
        self._theta = theta
        self._sigma = sigma
        self._vx = 0.0
        self._vy = 0.0
        self._rng = np.random.default_rng(seed)
        self._margin = 100.0

    def update(self, dt: float) -> tuple[float, float]:
        self._t += dt
        sqrt_dt = math.sqrt(dt)

        # Ornstein-Uhlenbeck velocity update
        # dv = -θ·v·dt + σ·dW
        self._vx += (-self._theta * self._vx * dt
                     + self._sigma * sqrt_dt * float(self._rng.standard_normal()))
        self._vy += (-self._theta * self._vy * dt
                     + self._sigma * sqrt_dt * float(self._rng.standard_normal()))

        # Clamp speed
        speed = math.hypot(self._vx, self._vy)
        if speed > self._max_speed:
            self._vx *= self._max_speed / speed
            self._vy *= self._max_speed / speed

        # Boundary repulsion: push velocity away from edges
        m = self._margin
        if self._x < m:
            self._vx += 200.0 * dt
        elif self._x > WORLD_W - m:
            self._vx -= 200.0 * dt
        if self._y < m:
            self._vy += 200.0 * dt
        elif self._y > WORLD_H - m:
            self._vy -= 200.0 * dt

        # Integrate position
        self._x = self._clamp(self._x + self._vx * dt, 0, WORLD_W - 1)
        self._y = self._clamp(self._y + self._vy * dt, 0, WORLD_H - 1)
        return self._x, self._y

    @property
    def velocity(self) -> tuple[float, float]:
        return self._vx, self._vy


# ---------------------------------------------------------------------------
# Factory function
# ---------------------------------------------------------------------------

def create_motion_model(
    motion_type: str | MotionType,
    center_x: float = 1000.0,
    center_y: float = 1000.0,
    **kwargs,
) -> MotionModel:
    """
    Factory: create a MotionModel from a string name or MotionType enum.

    Parameters
    ----------
    motion_type : str | MotionType
        e.g. 'straight_line', 'circular', 'figure_8', 'random_walk'
    center_x, center_y : float
        Logical centre/start for the trajectory.
    **kwargs
        Passed to the selected motion model constructor.
    """
    if isinstance(motion_type, str):
        motion_type = MotionType.from_string(motion_type)

    if motion_type == MotionType.STRAIGHT_LINE:
        return StraightLineMotion(start_x=center_x, start_y=center_y, **kwargs)
    elif motion_type == MotionType.CIRCULAR:
        return CircularMotion(center_x=center_x, center_y=center_y, **kwargs)
    elif motion_type == MotionType.FIGURE_8:
        return Figure8Motion(center_x=center_x, center_y=center_y, **kwargs)
    elif motion_type == MotionType.RANDOM_WALK:
        return RandomWalkMotion(start_x=center_x, start_y=center_y, **kwargs)
    else:
        raise ValueError(f"Unknown MotionType: {motion_type}")
