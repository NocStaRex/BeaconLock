"""
app/state_machine.py
====================
5-State Finite State Machine for BeaconLock PAT tracking.

States
------
SEARCHING    No target detected. Full-frame centroid scan active.
ACQUIRING    Target detected; counting consecutive sub-threshold frames to lock.
TRACKING     Locked. ROI-based centroid + Kalman + PD control active.
LOST         Lock broken. Kalman coasting. Short re-acquisition window.
REACQUIRING  Was TRACKING; target redetected. Fast re-lock path.

Transitions (deterministic)
---------------------------
SEARCHING   + detected(conf>0)          → ACQUIRING
ACQUIRING   + 5× consecutive lock       → TRACKING
ACQUIRING   + 3× consecutive miss       → SEARCHING
TRACKING    + 5× consecutive miss       → LOST
LOST        + detected                  → REACQUIRING
LOST        + 90 frames coast timeout   → SEARCHING   (1-sec ISRO re-acq spec)
REACQUIRING + 3× consecutive lock       → TRACKING
REACQUIRING + 5× consecutive miss       → LOST

Lock criterion:   detected=True AND error_px ≤ LOCK_THRESHOLD_PX (10 px)
Miss  criterion:  detected=False OR error_px > LOCK_THRESHOLD_PX

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class TrackingState(Enum):
    """Canonical 5-state FSM states (string value used as UI badge text)."""
    SEARCHING    = "SEARCHING"
    ACQUIRING    = "ACQUIRING"
    TRACKING     = "TRACKING"
    LOST         = "LOST"
    REACQUIRING  = "REACQUIRING"


# ── Tuneable thresholds ────────────────────────────────────────────────────
LOCK_THRESHOLD_PX: float = 10.0    # ISRO tracking error ceiling
LOCK_CONSECUTIVE:  int   = 5       # frames needed to declare TRACKING
RELOCK_CONSECUTIVE:int   = 3       # frames needed for REACQUIRING → TRACKING
MISS_TO_LOST:      int   = 5       # TRACKING miss frames before LOST
MISS_TO_SEARCH:    int   = 3       # ACQUIRING miss frames before SEARCHING
COAST_TIMEOUT:     int   = 90      # LOST frames before giving up (3 s at 30 fps)


# ── Per-frame telemetry from the FSM ──────────────────────────────────────

@dataclass
class FSMResult:
    """Output from StateMachine.update() for one frame."""
    state:                TrackingState
    prev_state:           TrackingState
    state_changed:        bool
    consecutive_locks:    int
    consecutive_misses:   int
    coast_frames:         int
    lock_threshold_px:    float = LOCK_THRESHOLD_PX


# ── State machine ─────────────────────────────────────────────────────────

class StateMachine:
    """
    BeaconLock 5-state PAT tracking FSM.

    Call update() once per frame with the perception result.

    Parameters
    ----------
    lock_threshold_px : float
        Per-frame error below which the beacon is considered "locked".
    """

    def __init__(self, lock_threshold_px: float = LOCK_THRESHOLD_PX) -> None:
        self._state: TrackingState = TrackingState.SEARCHING
        self._lock_threshold = lock_threshold_px

        # Counters
        self._consecutive_locks:  int = 0
        self._consecutive_misses: int = 0
        self._coast_frames:       int = 0
        self._acq_frames:         int = 0   # frames spent in ACQUIRING/REACQUIRING

        # Transition history (most-recent first)
        self._transitions: list[tuple[TrackingState, TrackingState]] = []

    # ------------------------------------------------------------------
    # Core update
    # ------------------------------------------------------------------

    def update(
        self,
        detected: bool,
        error_px: float,
        in_viewport: bool = True,
    ) -> FSMResult:
        """
        Process one frame and advance the FSM.

        Parameters
        ----------
        detected     : bool   Centroid estimator returned a valid detection.
        error_px     : float  Euclidean distance of centroid from frame centre.
        in_viewport  : bool   Target is geometrically within viewport (for Mode A GT).

        Returns
        -------
        FSMResult with the new state and all counter values.
        """
        prev_state = self._state
        locked = detected and (error_px <= self._lock_threshold)

        # ── Advance counters ───────────────────────────────────────────────
        # IMPORTANT: In ACQUIRING / REACQUIRING the "miss" counter should only
        # increment when the beacon is *truly not detected* (not when it is
        # detected but error > threshold — that just means the controller is still
        # centering the beacon).  Using lock-fail as a miss in these states causes
        # the FSM to retreat to SEARCHING before the controller can converge.
        in_acq_state = self._state in (
            TrackingState.ACQUIRING, TrackingState.REACQUIRING
        )

        if locked:
            self._consecutive_locks  += 1
            self._consecutive_misses  = 0
        else:
            self._consecutive_locks   = 0
            # In ACQUIRING/REACQUIRING: only count misses on detection loss
            if in_acq_state and detected:
                pass   # detected but error > threshold — controller still converging
            else:
                self._consecutive_misses += 1

        # ── State transitions ─────────────────────────────────────────
        if self._state == TrackingState.SEARCHING:
            if detected:
                self._state = TrackingState.ACQUIRING
                self._consecutive_locks = 1 if locked else 0
                self._consecutive_misses = 0
                self._acq_frames = 0   # fresh acquisition attempt

        elif self._state == TrackingState.ACQUIRING:
            self._acq_frames += 1
            if self._consecutive_locks >= LOCK_CONSECUTIVE:
                self._state = TrackingState.TRACKING
                self._consecutive_misses = 0
                self._acq_frames = 0
            elif self._consecutive_misses >= MISS_TO_SEARCH:
                # Only triggered on actual detection loss (see counter logic above)
                self._state = TrackingState.SEARCHING
                self._consecutive_locks = 0
                self._acq_frames = 0
            elif self._acq_frames >= COAST_TIMEOUT:
                # Safety: give up acquisition if controller can't converge in 3s
                self._state = TrackingState.SEARCHING
                self._consecutive_locks = 0
                self._acq_frames = 0

        elif self._state == TrackingState.TRACKING:
            if self._consecutive_misses >= MISS_TO_LOST:
                self._state = TrackingState.LOST
                self._coast_frames = 0

        elif self._state == TrackingState.LOST:
            self._coast_frames += 1
            if detected:
                self._state = TrackingState.REACQUIRING
                self._consecutive_locks = 1 if locked else 0
                self._consecutive_misses = 0
                self._coast_frames = 0
            elif self._coast_frames >= COAST_TIMEOUT:
                self._state = TrackingState.SEARCHING
                self._coast_frames = 0
                self._consecutive_misses = 0

        elif self._state == TrackingState.REACQUIRING:
            if self._consecutive_locks >= RELOCK_CONSECUTIVE:
                self._state = TrackingState.TRACKING
                self._consecutive_misses = 0
            elif self._consecutive_misses >= MISS_TO_LOST:
                self._state = TrackingState.LOST
                self._coast_frames = 0

        # Record transition
        if self._state != prev_state:
            self._transitions.append((prev_state, self._state))

        return FSMResult(
            state=self._state,
            prev_state=prev_state,
            state_changed=(self._state != prev_state),
            consecutive_locks=self._consecutive_locks,
            consecutive_misses=self._consecutive_misses,
            coast_frames=self._coast_frames,
        )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def state(self) -> TrackingState:
        return self._state

    @property
    def consecutive_locks(self) -> int:
        return self._consecutive_locks

    @property
    def consecutive_misses(self) -> int:
        return self._consecutive_misses

    @property
    def coast_frames(self) -> int:
        return self._coast_frames

    @property
    def transitions(self) -> list[tuple[TrackingState, TrackingState]]:
        """Read-only list of (from_state, to_state) transitions so far."""
        return list(self._transitions)

    def reset(self) -> None:
        """Reset all state to SEARCHING."""
        self._state = TrackingState.SEARCHING
        self._consecutive_locks = 0
        self._consecutive_misses = 0
        self._coast_frames = 0
        self._acq_frames = 0
        self._transitions.clear()
