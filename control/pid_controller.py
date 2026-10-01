"""
control/pid_controller.py
=========================
Rate-limited PD controller with anti-windup, deadband, and saturation
for BeaconLock virtual pan-tilt camera actuation.

Design choices (per MASTER_BLUEPRINT.md §6-F):
    - PD (no integral term) as primary — beacon tracker doesn't need Ki
      for the bounded-motion target types.  Avoids integral windup on
      sustained tracking loss.
    - Integral term is OPTIONAL and defaults to disabled (Ki = 0).
      Enable only if a persistent steady-state offset is measured.
    - Output saturation: ±5.0 °/s default (ISRO spec: 5–10 °/s).
    - Deadband (~0.2°): prevents high-frequency jitter-hunting when beacon
      is already centred within noise margin.
    - Anti-windup: integral state is clamped to ±windup_limit at every
      step regardless of whether integral is active.
    - Pixel-to-degree conversion: 0.00625 °/px (FOV 4°×3° / 640×480).

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# ISRO spec-derived constants
# ---------------------------------------------------------------------------

# Angular resolution (pixel-to-degree scale)
DEG_PER_PX_H: float = 4.0 / 640.0   # = 0.00625 °/px  (horizontal)
DEG_PER_PX_V: float = 3.0 / 480.0   # = 0.00625 °/px  (vertical)

# Maximum pan/tilt rate (ISRO hard limit: 5–10 °/s)
DEFAULT_MAX_RATE_DEG_S: float = 5.0

# Deadband in degrees — commands below this magnitude are zeroed
DEFAULT_DEADBAND_DEG: float = 0.2

# Default PD gains (tuned conservatively — operator can override)
DEFAULT_KP: float = 0.8
DEFAULT_KD: float = 0.15
DEFAULT_KI: float = 0.0            # disabled by default

# Anti-windup clamp (°/s · s accumulated)
DEFAULT_WINDUP_LIMIT: float = 2.0


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------

@dataclass
class ControlOutput:
    """Output from one PDController.update() call."""

    pan_rate_deg_s: float = 0.0
    """Commanded pan  rate in °/s (positive = right).  Clamped to ±max_rate."""

    tilt_rate_deg_s: float = 0.0
    """Commanded tilt rate in °/s (positive = down).  Clamped to ±max_rate."""

    error_x_px: float = 0.0
    error_y_px: float = 0.0
    """Raw pixel error fed into the controller this step."""

    error_x_deg: float = 0.0
    error_y_deg: float = 0.0
    """Error converted to angular units (°)."""

    in_deadband: bool = False
    """True if both axes were within the deadband — command is zeroed."""

    saturated: bool = False
    """True if either axis hit the rate limit this step."""

    integral_x: float = 0.0
    integral_y: float = 0.0
    """Current integral accumulator values — diagnostic."""


# ---------------------------------------------------------------------------
# PD (+ optional I) Controller
# ---------------------------------------------------------------------------

class PDController:
    """
    Rate-limited PD controller for virtual gimbal pan/tilt actuation.

    Usage
    -----
    controller = PDController()

    # Each control frame:
    output = controller.update(
        error_x_px = centroid_x - frame_centre_x,
        error_y_px = centroid_y - frame_centre_y,
        dt = 1.0 / 30.0,
    )
    # Apply output.pan_rate_deg_s, output.tilt_rate_deg_s to virtual camera.
    """

    def __init__(
        self,
        kp: float = DEFAULT_KP,
        kd: float = DEFAULT_KD,
        ki: float = DEFAULT_KI,
        max_rate_deg_s: float = DEFAULT_MAX_RATE_DEG_S,
        deadband_deg: float = DEFAULT_DEADBAND_DEG,
        windup_limit: float = DEFAULT_WINDUP_LIMIT,
        deg_per_px_h: float = DEG_PER_PX_H,
        deg_per_px_v: float = DEG_PER_PX_V,
    ) -> None:
        """
        Parameters
        ----------
        kp : float
            Proportional gain.
        kd : float
            Derivative gain.
        ki : float
            Integral gain (default 0 — disabled).  Enable only if steady-state
            offset persists after PD tuning.
        max_rate_deg_s : float
            Output saturation limit in °/s (ISRO: 5–10 °/s).
        deadband_deg : float
            Errors below this threshold (in degrees) produce zero command.
        windup_limit : float
            Anti-windup clamp for integral accumulator in °/s·s.
        deg_per_px_h, deg_per_px_v : float
            Pixel-to-degree scale factors (horizontal and vertical).
        """
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.max_rate = max_rate_deg_s
        self.deadband = deadband_deg
        self.windup_limit = windup_limit
        self.deg_per_px_h = deg_per_px_h
        self.deg_per_px_v = deg_per_px_v

        # Internal state
        self._prev_error_x: float = 0.0
        self._prev_error_y: float = 0.0
        self._integral_x: float = 0.0
        self._integral_y: float = 0.0
        self._first_step: bool = True

        logger.debug(
            "PDController created — Kp=%.3f Kd=%.3f Ki=%.3f | max=%.1f°/s | "
            "deadband=%.3f° | windup=%.2f",
            kp, kd, ki, max_rate_deg_s, deadband_deg, windup_limit,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update(
        self,
        error_x_px: float,
        error_y_px: float,
        dt: float = 1.0 / 30.0,
    ) -> ControlOutput:
        """
        Compute pan/tilt rate commands for the given pixel error.

        The error convention is:
            error_x_px = measured_cx − frame_centre_x
            error_y_px = measured_cy − frame_centre_y
        Positive error_x → beacon is to the right → positive pan command.
        Positive error_y → beacon is below centre → positive tilt command.

        Parameters
        ----------
        error_x_px, error_y_px : float
            Centroid offset from frame centre in pixels.
        dt : float
            Time since last call in seconds.

        Returns
        -------
        ControlOutput
        """
        output = ControlOutput(
            error_x_px=error_x_px,
            error_y_px=error_y_px,
        )

        # ----------------------------------------------------------------
        # Convert pixel error to angular error
        # ----------------------------------------------------------------
        err_x = error_x_px * self.deg_per_px_h
        err_y = error_y_px * self.deg_per_px_v

        output.error_x_deg = err_x
        output.error_y_deg = err_y

        # ----------------------------------------------------------------
        # Deadband check
        # ----------------------------------------------------------------
        err_mag = (err_x ** 2 + err_y ** 2) ** 0.5
        if err_mag < self.deadband:
            output.in_deadband = True
            self._prev_error_x = err_x
            self._prev_error_y = err_y
            self._first_step = False
            return output   # zero command

        # ----------------------------------------------------------------
        # Derivative term (skip on first step to avoid derivative kick)
        # ----------------------------------------------------------------
        if self._first_step:
            d_err_x = 0.0
            d_err_y = 0.0
            self._first_step = False
        else:
            d_err_x = (err_x - self._prev_error_x) / max(dt, 1e-6)
            d_err_y = (err_y - self._prev_error_y) / max(dt, 1e-6)

        # ----------------------------------------------------------------
        # Integral term with anti-windup clamp
        # ----------------------------------------------------------------
        self._integral_x += err_x * dt
        self._integral_y += err_y * dt

        # Anti-windup: clamp integral accumulator
        self._integral_x = float(
            max(-self.windup_limit, min(self.windup_limit, self._integral_x))
        )
        self._integral_y = float(
            max(-self.windup_limit, min(self.windup_limit, self._integral_y))
        )

        # ----------------------------------------------------------------
        # PD(I) law
        # ----------------------------------------------------------------
        raw_x = (
            self.kp * err_x
            + self.kd * d_err_x
            + self.ki * self._integral_x
        )
        raw_y = (
            self.kp * err_y
            + self.kd * d_err_y
            + self.ki * self._integral_y
        )

        # ----------------------------------------------------------------
        # Rate saturation clamp (ISRO: ±5–10 °/s)
        # ----------------------------------------------------------------
        clamped_x = max(-self.max_rate, min(self.max_rate, raw_x))
        clamped_y = max(-self.max_rate, min(self.max_rate, raw_y))

        output.saturated = (abs(raw_x) > self.max_rate) or (abs(raw_y) > self.max_rate)
        output.pan_rate_deg_s  = clamped_x
        output.tilt_rate_deg_s = clamped_y
        output.integral_x      = self._integral_x
        output.integral_y      = self._integral_y

        # Update previous error for next derivative computation
        self._prev_error_x = err_x
        self._prev_error_y = err_y

        return output

    def reset(self) -> None:
        """Reset internal state (call on state machine transition to SEARCHING)."""
        self._prev_error_x = 0.0
        self._prev_error_y = 0.0
        self._integral_x   = 0.0
        self._integral_y   = 0.0
        self._first_step   = True
        logger.debug("PDController reset.")

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def max_rate_deg_s(self) -> float:
        return self.max_rate

    @max_rate_deg_s.setter
    def max_rate_deg_s(self, value: float) -> None:
        if not (5.0 <= value <= 10.0):
            logger.warning("max_rate_deg_s=%.2f outside ISRO spec [5,10] °/s", value)
        self.max_rate = value

    @staticmethod
    def px_to_deg_horizontal(px: float) -> float:
        """Convert a pixel distance to degrees (horizontal axis)."""
        return px * DEG_PER_PX_H

    @staticmethod
    def px_to_deg_vertical(px: float) -> float:
        """Convert a pixel distance to degrees (vertical axis)."""
        return px * DEG_PER_PX_V

    @staticmethod
    def deg_to_mrad(deg: float) -> float:
        """Convert degrees to milliradians."""
        return deg * (1000.0 * 3.141592653589793 / 180.0)
