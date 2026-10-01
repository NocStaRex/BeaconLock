"""
perception/centroid.py
======================
Radiometric beacon centroid estimator for BeaconLock.

Algorithm
---------
1. **Background estimation** (radiometric thresholding):
       I_th = μ_bg + k · σ_bg        (default k = 3.0)
   Background statistics are computed from a border-annulus of the ROI to
   avoid biasing the estimate with the beacon itself.

2. **Intensity-weighted Centre of Gravity (CoG)**:
       c_x = Σ(x_i · I_i) / Σ(I_i)
       c_y = Σ(y_i · I_i) / Σ(I_i)
   Operating on the thresholded foreground mask; achieves 0.2–0.5 px
   sub-pixel accuracy on a clean beacon spot.

3. **Dynamic 64×64 ROI tracking**:
   During TRACKING state, detection runs only inside a 64×64 px window
   centred on the last known/predicted position, with the full-frame
   centroid coordinate recovered via offset addition.

4. **Confidence scoring** (used by Kalman adaptive-R and state machine):
       confidence ∈ [0.0, 1.0]  based on SNR and foreground pixel count.

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Default ROI half-size in pixels (64×64 window → half = 32)
DEFAULT_ROI_HALF: int = 32

# Sigma multiplier for radiometric threshold:  I_th = μ_bg + K_SIGMA · σ_bg
K_SIGMA: float = 3.0

# Border fraction of ROI used to estimate background (avoids beacon pixels)
BG_BORDER_FRACTION: float = 0.20     # 20% border ring

# Minimum foreground pixels to call a valid detection
MIN_FOREGROUND_PX: int = 3

# Minimum peak intensity (absolute, 0-255) to call a valid detection
MIN_PEAK_INTENSITY: int = 20


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------

@dataclass
class CentroidResult:
    """Output of a single centroid estimation call."""

    detected: bool = False
    """True if a valid beacon centroid was found in this frame."""

    cx: float = 0.0
    cy: float = 0.0
    """Full-frame centroid coordinates in pixel space (float, sub-pixel)."""

    confidence: float = 0.0
    """
    Detection confidence ∈ [0.0, 1.0].
    Used by KalmanTracker to scale measurement covariance R:
        R_eff = R_base / max(confidence, ε)
    """

    foreground_px: int = 0
    """Number of foreground (above-threshold) pixels — diagnostic."""

    peak_intensity: float = 0.0
    """Peak pixel intensity inside the detected region — diagnostic."""

    snr: float = 0.0
    """Signal-to-Noise Ratio: (peak - μ_bg) / max(σ_bg, 1) — diagnostic."""

    roi_used: bool = False
    """True if a restricted ROI was used (TRACKING mode); False = full frame."""

    roi_origin: tuple[int, int] = field(default_factory=lambda: (0, 0))
    """(col_offset, row_offset) of the ROI top-left corner in full-frame coords."""


# ---------------------------------------------------------------------------
# Core estimator
# ---------------------------------------------------------------------------

class CentroidEstimator:
    """
    Intensity-weighted Centre-of-Gravity beacon centroid estimator.

    Usage
    -----
    estimator = CentroidEstimator()

    # Full-frame search (SEARCHING / ACQUIRING states)
    result = estimator.estimate(frame)

    # ROI-restricted search (TRACKING / LOST states)
    result = estimator.estimate(frame, roi_centre=(cx_pred, cy_pred))
    """

    def __init__(
        self,
        roi_half: int = DEFAULT_ROI_HALF,
        k_sigma: float = K_SIGMA,
        min_foreground_px: int = MIN_FOREGROUND_PX,
        min_peak_intensity: int = MIN_PEAK_INTENSITY,
        bg_border_fraction: float = BG_BORDER_FRACTION,
    ) -> None:
        self.roi_half = roi_half
        self.k_sigma = k_sigma
        self.min_foreground_px = min_foreground_px
        self.min_peak_intensity = min_peak_intensity
        self.bg_border_fraction = bg_border_fraction

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def estimate(
        self,
        frame: np.ndarray,
        roi_centre: Optional[tuple[float, float]] = None,
    ) -> CentroidResult:
        """
        Estimate the beacon centroid in *frame*.

        Parameters
        ----------
        frame : np.ndarray
            Greyscale uint8 frame (HxW).  BGR frames are auto-converted.
        roi_centre : (cx, cy) | None
            If provided, restrict detection to a roi_half*2 × roi_half*2
            window centred here (TRACKING mode for CPU efficiency).
            None → full-frame search (SEARCHING mode).

        Returns
        -------
        CentroidResult
        """
        gray = self._ensure_grey(frame)
        h, w = gray.shape

        if roi_centre is not None:
            return self._estimate_roi(gray, w, h, roi_centre)
        else:
            return self._estimate_full(gray, w, h)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _ensure_grey(frame: np.ndarray) -> np.ndarray:
        if frame.ndim == 3:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return frame

    def _estimate_full(self, gray: np.ndarray, w: int, h: int) -> CentroidResult:
        """Full-frame centroid search (SEARCHING / ACQUIRING / REACQUIRING)."""
        result = self._compute_centroid_on_patch(
            patch=gray,
            col_offset=0,
            row_offset=0,
        )
        result.roi_used = False
        result.roi_origin = (0, 0)
        return result

    def _estimate_roi(
        self,
        gray: np.ndarray,
        w: int,
        h: int,
        roi_centre: tuple[float, float],
    ) -> CentroidResult:
        """64×64 ROI-restricted centroid search (TRACKING / LOST)."""
        cx_pred, cy_pred = roi_centre
        rh = self.roi_half

        # Clamp ROI to frame bounds
        col0 = int(max(0, cx_pred - rh))
        row0 = int(max(0, cy_pred - rh))
        col1 = int(min(w, col0 + 2 * rh))
        row1 = int(min(h, row0 + 2 * rh))

        # Snap col0/row0 in case col1/row1 were clamped
        col0 = max(0, col1 - 2 * rh)
        row0 = max(0, row1 - 2 * rh)

        patch = gray[row0:row1, col0:col1]
        if patch.size == 0:
            return CentroidResult(detected=False, roi_used=True, roi_origin=(col0, row0))

        result = self._compute_centroid_on_patch(
            patch=patch,
            col_offset=col0,
            row_offset=row0,
        )
        result.roi_used = True
        result.roi_origin = (col0, row0)
        return result

    def _compute_centroid_on_patch(
        self,
        patch: np.ndarray,
        col_offset: int,
        row_offset: int,
    ) -> CentroidResult:
        """
        Core radiometric centroid computation on an arbitrary image patch.

        Steps:
          1. Estimate background from border annulus.
          2. Compute adaptive threshold  I_th = μ_bg + k·σ_bg.
          3. Mask foreground pixels above I_th.
          4. If mask is valid, compute intensity-weighted CoG.
          5. Map local-patch coordinates back to full-frame coordinates.
        """
        result = CentroidResult()

        ph, pw = patch.shape

        # ----------------------------------------------------------------
        # Step 1 — Background estimation from border annulus
        # ----------------------------------------------------------------
        border_px = max(1, int(min(ph, pw) * self.bg_border_fraction))
        bg_mask = np.zeros((ph, pw), dtype=bool)
        bg_mask[:border_px, :] = True          # top
        bg_mask[-border_px:, :] = True         # bottom
        bg_mask[:, :border_px] = True          # left
        bg_mask[:, -border_px:] = True         # right

        bg_pixels = patch[bg_mask].astype(np.float64)
        if bg_pixels.size < 4:
            # Patch too small to estimate background reliably
            mu_bg = float(np.mean(patch))
            sigma_bg = float(np.std(patch)) + 1.0
        else:
            mu_bg = float(np.mean(bg_pixels))
            sigma_bg = float(np.std(bg_pixels)) + 1.0   # +1 avoids zero-div

        # ----------------------------------------------------------------
        # Step 2 — Adaptive radiometric threshold
        # ----------------------------------------------------------------
        I_th = mu_bg + self.k_sigma * sigma_bg

        # ----------------------------------------------------------------
        # Step 3 — Foreground mask
        # ----------------------------------------------------------------
        fg_mask = patch.astype(np.float64) > I_th
        fg_pixels = np.count_nonzero(fg_mask)

        peak_val = float(np.max(patch))

        result.foreground_px = fg_pixels
        result.peak_intensity = peak_val
        result.snr = (peak_val - mu_bg) / sigma_bg

        # Guard: insufficient foreground or too dim
        if fg_pixels < self.min_foreground_px or peak_val < self.min_peak_intensity:
            result.detected = False
            return result

        # ----------------------------------------------------------------
        # Step 4 — Intensity-weighted Centre of Gravity (sub-pixel)
        # ----------------------------------------------------------------
        # Build coordinate grids
        cols = np.arange(pw, dtype=np.float64)
        rows = np.arange(ph, dtype=np.float64)
        col_grid, row_grid = np.meshgrid(cols, rows)

        # Subtract background before weighting (improves accuracy under noise)
        weighted = np.where(fg_mask, patch.astype(np.float64) - mu_bg, 0.0)
        weighted = np.clip(weighted, 0.0, None)
        weight_sum = weighted.sum()

        if weight_sum < 1e-9:
            result.detected = False
            return result

        local_cx = float((col_grid * weighted).sum() / weight_sum)
        local_cy = float((row_grid * weighted).sum() / weight_sum)

        # ----------------------------------------------------------------
        # Step 5 — Map to full-frame coordinates
        # ----------------------------------------------------------------
        result.cx = local_cx + col_offset
        result.cy = local_cy + row_offset

        # ----------------------------------------------------------------
        # Confidence scoring
        # ----------------------------------------------------------------
        # Normalised SNR component (saturates at SNR ≥ 10)
        snr_conf = min(result.snr / 10.0, 1.0)
        # Foreground coverage component (saturates at 200 px)
        cov_conf = min(fg_pixels / 200.0, 1.0)
        result.confidence = float(0.6 * snr_conf + 0.4 * cov_conf)
        result.detected = True

        return result


# ---------------------------------------------------------------------------
# Module-level convenience function
# ---------------------------------------------------------------------------

_default_estimator: Optional[CentroidEstimator] = None


def estimate_centroid(
    frame: np.ndarray,
    roi_centre: Optional[tuple[float, float]] = None,
    **kwargs,
) -> CentroidResult:
    """
    Module-level convenience wrapper.  Uses a shared default CentroidEstimator.
    Pass keyword args to override defaults on first call only.
    """
    global _default_estimator
    if _default_estimator is None or kwargs:
        _default_estimator = CentroidEstimator(**kwargs)
    return _default_estimator.estimate(frame, roi_centre=roi_centre)
