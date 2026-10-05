"""
perception/kalman_tracker.py
============================
6-State Constant-Acceleration Kalman Filter for BeaconLock.

State vector:  x = [x, y, vx, vy, ax, ay]^T
               (position, velocity, acceleration — 2D)

Why 6 states (not 4)?
    The ISRO PS mandates Figure-8 and Random motion modes — both involve
    continuously changing velocity (non-constant-velocity motion).  A 4-state
    [x,y,vx,vy] filter assumes near-constant velocity and will lag/overshoot
    on curved trajectories.  The 6-state model tracks acceleration explicitly
    and predicts through turns and direction reversals far more accurately.
    This matches practice in real optical/radar tracking (star trackers, PAT).

Adaptive measurement covariance R:
    When detection confidence is low (noisy/occluded frames), R is inflated so
    the filter trusts its own prediction more than the noisy measurement.
    R_eff = R_base / max(confidence, ε)
    This is the "disturbance-aware Kalman" innovation from §12 of the blueprint.

Coasting (predict-only) during LOST state:
    predict() runs every frame regardless of detection.
    update() is called ONLY when a valid centroid is available.
    During LOST state the filter coasts forward on its motion model and the
    predicted position is used to aim the camera — directly improving the
    ≤1s re-acquisition target.

References
----------
    filterpy.kalman.KalmanFilter  (R. Labbe, filterpy library)
    ISRO PS-26169 §3, §6-G (blueprint MASTER_BLUEPRINT.md)

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

import numpy as np
try:
    from filterpy.kalman.kalman_filter import KalmanFilter
except ImportError:
    from filterpy.kalman import KalmanFilter

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Default covariance tuning constants
# ---------------------------------------------------------------------------

# Process noise Q — how fast we expect state to change between frames
# Tuned for a ≤20 px/frame disturbance environment
Q_POSITION_VAR: float = 1.0        # px²  — position process noise
Q_VELOCITY_VAR: float = 10.0       # (px/s)²  — velocity process noise
Q_ACCEL_VAR:    float = 50.0       # (px/s²)² — acceleration process noise

# Base measurement noise R — sensor noise floor in calm conditions
R_BASE_VAR: float = 4.0            # px²  (≈ 2 px std dev)

# Maximum R inflation factor when confidence → 0
R_MAX_INFLATION: float = 100.0

# Minimum confidence clamp to avoid division by zero
CONFIDENCE_EPS: float = 0.01

# Initial uncertainty P — large to allow fast convergence from any position
P_INITIAL_VAR: float = 500.0       # px²


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------

@dataclass
class KalmanResult:
    """Output from one KalmanTracker.step() call."""

    predicted_x: float = 0.0
    predicted_y: float = 0.0
    """Post-update (or coast) position estimate — best current position."""

    velocity_x: float = 0.0
    velocity_y: float = 0.0
    """Current velocity estimate (px/frame)."""

    accel_x: float = 0.0
    accel_y: float = 0.0
    """Current acceleration estimate (px/frame²)."""

    updated: bool = False
    """True if a measurement update was performed this step."""

    innovation_x: float = 0.0
    innovation_y: float = 0.0
    """Measurement residual (measurement − prior prediction).  Zero if coasting."""

    uncertainty_x: float = 0.0
    uncertainty_y: float = 0.0
    """Posterior standard deviation of position estimate (√P[0,0], √P[1,1])."""

    effective_R: float = 0.0
    """Actual R diagonal value used this step (for telemetry logging)."""


# ---------------------------------------------------------------------------
# KalmanTracker
# ---------------------------------------------------------------------------

class KalmanTracker:
    """
    6-State Constant-Acceleration Kalman Filter tracker.

    Usage
    -----
    tracker = KalmanTracker(dt=1/30)

    # On first valid detection — initialise state
    tracker.initialise(cx, cy)

    # Every frame — regardless of detection
    result = tracker.step(cx=measured_cx, cy=measured_cy, confidence=conf)
    # OR during occlusion (coast):
    result = tracker.step(cx=None, cy=None, confidence=0.0)
    """

    def __init__(
        self,
        dt: float = 1.0 / 30.0,
        q_pos: float = Q_POSITION_VAR,
        q_vel: float = Q_VELOCITY_VAR,
        q_acc: float = Q_ACCEL_VAR,
        r_base: float = R_BASE_VAR,
        r_max_inflation: float = R_MAX_INFLATION,
        p_initial: float = P_INITIAL_VAR,
    ) -> None:
        """
        Parameters
        ----------
        dt : float
            Nominal time step between frames in seconds (1/FPS).
        q_pos, q_vel, q_acc : float
            Process noise variances for position, velocity, acceleration.
        r_base : float
            Base measurement noise variance (calm-conditions floor).
        r_max_inflation : float
            Maximum R inflation factor at zero confidence.
        p_initial : float
            Initial diagonal variance of the covariance matrix P.
        """
        self.dt = dt
        self.r_base = r_base
        self.r_max_inflation = r_max_inflation
        self._initialised: bool = False

        # ----------------------------------------------------------------
        # Build filterpy KalmanFilter
        # State:       [x, y, vx, vy, ax, ay]^T   (dim_x = 6)
        # Measurement: [x, y]^T                    (dim_z = 2)
        # ----------------------------------------------------------------
        self._kf = KalmanFilter(dim_x=6, dim_z=2)

        # State transition matrix F (constant-acceleration kinematics)
        self._kf.F = self._build_F(dt)

        # Measurement matrix H — we observe only x, y
        self._kf.H = np.array([
            [1., 0., 0., 0., 0., 0.],
            [0., 1., 0., 0., 0., 0.],
        ], dtype=np.float64)

        # Measurement noise covariance R (updated adaptively each step)
        self._kf.R = np.eye(2, dtype=np.float64) * r_base

        # Process noise covariance Q
        self._kf.Q = self._build_Q(q_pos, q_vel, q_acc)

        # Initial state covariance P
        self._kf.P = np.eye(6, dtype=np.float64) * p_initial

        # State vector initialised to zero; overwritten by initialise()
        self._kf.x = np.zeros((6, 1), dtype=np.float64)

        logger.debug(
            "KalmanTracker created — dt=%.4f s | R_base=%.2f | Q_pos=%.2f Q_vel=%.2f Q_acc=%.2f",
            dt, r_base, q_pos, q_vel, q_acc,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def initialise(self, cx: float, cy: float, vx: float = 0.0, vy: float = 0.0) -> None:
        """
        Seed the filter with a known position (and optional velocity guess).
        Must be called at least once before step().
        """
        self._kf.x = np.array([[cx], [cy], [vx], [vy], [0.], [0.]], dtype=np.float64)
        # Reset P to allow fast initial convergence
        self._kf.P = np.eye(6, dtype=np.float64) * P_INITIAL_VAR
        self._initialised = True
        logger.debug("KalmanTracker initialised at (%.2f, %.2f)", cx, cy)

    def step(
        self,
        cx: Optional[float],
        cy: Optional[float],
        confidence: float = 1.0,
        dt: Optional[float] = None,
    ) -> KalmanResult:
        """
        Advance the filter by one frame.

        Parameters
        ----------
        cx, cy : float | None
            Measured centroid position.  Pass None to coast (occlusion/LOST).
        confidence : float
            Detection confidence ∈ [0.0, 1.0] from CentroidEstimator.
            Low confidence inflates R, making the filter trust prediction more.
        dt : float | None
            Override the default timestep (use when frame timing is irregular).

        Returns
        -------
        KalmanResult
            Post-predict / post-update state estimate.
        """
        if not self._initialised:
            if cx is not None and cy is not None:
                self.initialise(cx, cy)
            else:
                # Cannot step without initialisation
                return KalmanResult()

        # Update F if dt changed
        effective_dt = dt if dt is not None else self.dt
        if dt is not None:
            self._kf.F = self._build_F(effective_dt)

        # ----------------------------------------------------------------
        # Predict step — runs every frame
        # ----------------------------------------------------------------
        prior_x = float(self._kf.x[0])
        prior_y = float(self._kf.x[1])
        self._kf.predict()

        result = KalmanResult()

        # ----------------------------------------------------------------
        # Update step — only when valid measurement available
        # ----------------------------------------------------------------
        if cx is not None and cy is not None and confidence > 0.0:
            # Adaptive R: inflate when confidence is low
            conf_clamped = max(confidence, CONFIDENCE_EPS)
            inflation = 1.0 + (1.0 - conf_clamped) * (self.r_max_inflation - 1.0)
            R_eff = self.r_base * inflation
            self._kf.R = np.eye(2, dtype=np.float64) * R_eff

            z = np.array([[cx], [cy]], dtype=np.float64)
            self._kf.update(z)

            result.updated = True
            result.innovation_x = cx - prior_x
            result.innovation_y = cy - prior_y
            result.effective_R = R_eff
        else:
            # Coasting: no update, rely on prediction
            result.updated = False
            result.effective_R = self.r_base * self.r_max_inflation

        # ----------------------------------------------------------------
        # Pack result from posterior state
        # ----------------------------------------------------------------
        x = self._kf.x
        P = self._kf.P

        result.predicted_x = float(x[0])
        result.predicted_y = float(x[1])
        result.velocity_x  = float(x[2])
        result.velocity_y  = float(x[3])
        result.accel_x     = float(x[4])
        result.accel_y     = float(x[5])
        result.uncertainty_x = float(np.sqrt(max(P[0, 0], 0.0)))
        result.uncertainty_y = float(np.sqrt(max(P[1, 1], 0.0)))

        return result

    def update_dt(self, dt: float) -> None:
        """Update the nominal timestep and rebuild F matrix in-place."""
        self.dt = dt
        self._kf.F = self._build_F(dt)

    @property
    def is_initialised(self) -> bool:
        return self._initialised

    @property
    def state(self) -> np.ndarray:
        """Current 6-element state vector as a flat 1D array (read-only copy).
        
        Returns a flattened copy so state[0] yields a Python float, not a
        (1,)-shaped array — prevents numpy comparison ambiguity in ROI code.
        """
        return self._kf.x.flatten().copy()

    @property
    def covariance(self) -> np.ndarray:
        """Current 6×6 covariance matrix P (read-only copy)."""
        return self._kf.P.copy()

    # ------------------------------------------------------------------
    # Private matrix builders
    # ------------------------------------------------------------------

    @staticmethod
    def _build_F(dt: float) -> np.ndarray:
        """
        Constant-acceleration state transition matrix.

        x_k+1 = F · x_k
        F = [[1,  0, dt,  0,  0.5*dt², 0      ],
             [0,  1,  0, dt,  0,       0.5*dt²],
             [0,  0,  1,  0,  dt,      0      ],
             [0,  0,  0,  1,  0,       dt     ],
             [0,  0,  0,  0,  1,       0      ],
             [0,  0,  0,  0,  0,       1      ]]
        """
        dt2 = 0.5 * dt ** 2
        return np.array([
            [1., 0., dt,  0., dt2,  0.],
            [0., 1.,  0., dt,  0., dt2],
            [0., 0.,  1.,  0., dt,  0.],
            [0., 0.,  0.,  1.,  0., dt],
            [0., 0.,  0.,  0.,  1.,  0.],
            [0., 0.,  0.,  0.,  0.,  1.],
        ], dtype=np.float64)

    @staticmethod
    def _build_Q(q_pos: float, q_vel: float, q_acc: float) -> np.ndarray:
        """Diagonal process noise covariance Q."""
        return np.diag([q_pos, q_pos, q_vel, q_vel, q_acc, q_acc]).astype(np.float64)

    def reset(self) -> None:
        """Reset to uninitialised state (use on SEARCHING after full loss)."""
        self._kf.x[:] = 0.0
        self._kf.P = np.eye(6, dtype=np.float64) * P_INITIAL_VAR
        self._initialised = False
        logger.debug("KalmanTracker reset.")
