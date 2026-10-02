"""
simulation/disturbances.py
===========================
Disturbance & Environmental Engine for BeaconLock.

All disturbances are independently configurable per the ISRO PS-26169 spec.

Disturbance types
-----------------
IMAGE NOISE (applied to viewport frame):
  Gaussian   — additive Gaussian noise, configurable σ (max 20 px std dev).
  Salt&Pepper— random pixel corruption at ~10% density.
  Poisson    — photon shot noise (signal-dependent).

SENSOR / PLATFORM DISTURBANCES:
  Camera Jitter   — high-frequency per-frame viewport displacement ±20 px.
                    Separate from platform motion (blueprint §6-C).
  Platform Motion — low-frequency structural platform drift ±20 px/frame.
                    Two sub-modes: Linear ramp and Circular oscillation.

ATMOSPHERIC ATTENUATION (applied to viewport before noise):
  clear      — no effect.
  haze       — mild contrast reduction + slight brightness boost.
  fog        — strong contrast/brightness reduction + Gaussian blur.
  rain       — vertical streak artifacts + contrast reduction.
  low_light  — strong brightness reduction, lowered contrast.

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import logging
import math
from enum import Enum, auto
from typing import Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# Maximum ISRO-specified disturbance magnitudes
MAX_JITTER_PX: int  = 20
MAX_PLATFORM_PX: int = 20
SP_NOISE_DENSITY: float = 0.10   # 10% salt-and-pepper


class AtmosphericMode(Enum):
    CLEAR     = auto()
    HAZE      = auto()
    FOG       = auto()
    RAIN      = auto()
    LOW_LIGHT = auto()

    @classmethod
    def from_string(cls, s: str) -> "AtmosphericMode":
        mapping = {
            "clear":     cls.CLEAR,
            "haze":      cls.HAZE,
            "fog":       cls.FOG,
            "rain":      cls.RAIN,
            "low_light": cls.LOW_LIGHT,
            "lowlight":  cls.LOW_LIGHT,
        }
        key = s.lower().strip()
        if key not in mapping:
            raise ValueError(f"Unknown atmospheric mode '{s}'")
        return mapping[key]


class PlatformMotionMode(Enum):
    NONE     = auto()
    LINEAR   = auto()
    CIRCULAR = auto()
    RANDOM   = auto()


# ---------------------------------------------------------------------------
# Image noise functions
# ---------------------------------------------------------------------------

def apply_gaussian_noise(
    frame: np.ndarray,
    sigma: float = 10.0,
    rng: Optional[np.random.Generator] = None,
) -> np.ndarray:
    """
    Additive Gaussian noise with std dev σ (max 20 per ISRO spec).

    Parameters
    ----------
    frame : np.ndarray  HxW uint8
    sigma : float       Noise std dev in intensity units [0, 20].
    rng   : np.random.Generator  Optional RNG for reproducibility.
    """
    if sigma <= 0:
        return frame
    sigma = min(sigma, 20.0)
    rng = rng or np.random.default_rng()
    noise = rng.normal(0.0, sigma, frame.shape).astype(np.float32)
    noisy = frame.astype(np.float32) + noise
    return np.clip(noisy, 0, 255).astype(np.uint8)


def apply_salt_pepper_noise(
    frame: np.ndarray,
    density: float = SP_NOISE_DENSITY,
    rng: Optional[np.random.Generator] = None,
) -> np.ndarray:
    """
    Salt-and-pepper noise: randomly sets pixels to 0 or 255.

    Parameters
    ----------
    density : float  Fraction of corrupted pixels (default 0.10 = 10%).
    """
    if density <= 0:
        return frame
    rng = rng or np.random.default_rng()
    out = frame.copy()
    h, w = frame.shape[:2]
    n_total = h * w
    n_corrupt = int(n_total * density)

    # Indices of corrupted pixels (flat)
    idx = rng.choice(n_total, n_corrupt, replace=False)
    flat = out.ravel()
    # Half salt (255), half pepper (0)
    flat[idx[:n_corrupt // 2]] = 255
    flat[idx[n_corrupt // 2:]] = 0
    return out


def apply_poisson_noise(
    frame: np.ndarray,
    scale: float = 1.0,
) -> np.ndarray:
    """
    Poisson (shot) noise — signal-dependent noise proportional to √I.

    The frame is treated as photon counts (scaled). Scale factor controls
    the SNR: lower scale = more noise.

    Parameters
    ----------
    scale : float  Photon count scale factor. 1.0 = moderate noise.
    """
    if scale <= 0:
        return frame
    # Treat pixel values as photon counts
    photons = frame.astype(np.float32) / 255.0 * scale
    photons = np.clip(photons, 0, None)
    noisy_photons = np.random.poisson(photons).astype(np.float32)
    noisy = (noisy_photons / scale * 255.0)
    return np.clip(noisy, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# Atmospheric attenuation
# ---------------------------------------------------------------------------

def apply_atmospheric(
    frame: np.ndarray,
    mode: AtmosphericMode | str = AtmosphericMode.CLEAR,
    rng: Optional[np.random.Generator] = None,
) -> np.ndarray:
    """
    Apply atmospheric contrast/brightness model to a viewport frame.

    Parameters
    ----------
    frame : np.ndarray  HxW uint8 greyscale viewport frame.
    mode  : AtmosphericMode | str
    rng   : RNG for stochastic effects (rain streaks, etc.)
    """
    if isinstance(mode, str):
        mode = AtmosphericMode.from_string(mode)

    if mode == AtmosphericMode.CLEAR:
        return frame

    out = frame.astype(np.float32)

    if mode == AtmosphericMode.HAZE:
        # Contrast decay α=0.85, lift β=15 (brightens background slightly)
        out = out * 0.85 + 15.0

    elif mode == AtmosphericMode.FOG:
        # Physical fog model: I_fog = α·I_clear + β (Koschmieder's law)
        # α=0.50 (transmission), β=55 (ambient sky radiance)
        # → I_fog(0)=55, I_fog(128)=119, I_fog(180)=145, I_fog(255)=182.5
        # Strong contrast reduction across all input ranges.
        out = out * 0.50 + 55.0
        out = cv2.GaussianBlur(out, (7, 7), sigmaX=3.0)

    elif mode == AtmosphericMode.RAIN:
        # Contrast reduction + vertical streak artifacts
        out = out * 0.75 + 20.0
        rng = rng or np.random.default_rng()
        h, w = frame.shape[:2]
        n_streaks = int(w * 0.04)   # ~4% of columns have streaks
        streak_cols = rng.integers(0, w, n_streaks)
        for col in streak_cols:
            streak_len = rng.integers(h // 8, h // 3)
            row_start  = rng.integers(0, h - streak_len)
            # Bright streak (simulates rain droplet blur)
            out[row_start:row_start + streak_len, col] = np.minimum(
                out[row_start:row_start + streak_len, col] + 60.0, 255.0
            )

    elif mode == AtmosphericMode.LOW_LIGHT:
        # Strong brightness reduction, moderate contrast decay
        out = out * 0.50 - 30.0

    return np.clip(out, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# Camera Jitter
# ---------------------------------------------------------------------------

class CameraJitter:
    """
    High-frequency per-frame camera vibration (sensor/mount jitter).

    Produces a random frame-to-frame image displacement ±max_px.
    Applied by shifting the viewport crop origin each frame.

    ⚠ This is DISTINCT from platform motion (blueprint §6-C).

    Parameters
    ----------
    max_px : float   Maximum per-frame displacement magnitude (spec: ±20 px).
    sigma  : float   Gaussian std dev for per-frame jitter distribution.
    seed   : int | None  For reproducibility.
    """

    def __init__(
        self,
        max_px: float = 20.0,
        sigma: float = 8.0,
        enabled: bool = True,
        seed: Optional[int] = None,
    ) -> None:
        self.max_px  = min(float(max_px), float(MAX_JITTER_PX))
        self.sigma   = sigma
        self.enabled = enabled
        self._rng    = np.random.default_rng(seed)
        self._dx: float = 0.0
        self._dy: float = 0.0

    def sample(self) -> tuple[float, float]:
        """
        Sample a new jitter displacement for the current frame.

        Returns
        -------
        (dx, dy) : signed pixel offsets, each clamped to ±max_px.
        """
        if not self.enabled:
            self._dx = self._dy = 0.0
            return 0.0, 0.0

        dx = float(np.clip(self._rng.normal(0.0, self.sigma), -self.max_px, self.max_px))
        dy = float(np.clip(self._rng.normal(0.0, self.sigma), -self.max_px, self.max_px))
        self._dx, self._dy = dx, dy
        return dx, dy

    @property
    def last_displacement(self) -> tuple[float, float]:
        return self._dx, self._dy


# ---------------------------------------------------------------------------
# Platform Motion
# ---------------------------------------------------------------------------

class PlatformMotion:
    """
    Low-frequency structural platform drift.

    Models platform motion as either:
      LINEAR   — constant or slowly changing drift velocity.
      CIRCULAR — sinusoidal oscillation (simulates satellite attitude wobble).
      RANDOM   — slow random walk with bounded displacement.

    The resulting (dx, dy) per-frame displacement is applied to the camera
    centre position — making the *entire scene* appear to drift.

    Parameters
    ----------
    mode       : PlatformMotionMode
    max_px_per_frame : float  Maximum displacement magnitude per frame (spec: ±20).
    """

    def __init__(
        self,
        mode: PlatformMotionMode | str = PlatformMotionMode.LINEAR,
        max_px_per_frame: float = 10.0,
        seed: Optional[int] = None,
    ) -> None:
        if isinstance(mode, str):
            mode_map = {
                "none":     PlatformMotionMode.NONE,
                "linear":   PlatformMotionMode.LINEAR,
                "circular": PlatformMotionMode.CIRCULAR,
                "random":   PlatformMotionMode.RANDOM,
            }
            mode = mode_map.get(mode.lower(), PlatformMotionMode.LINEAR)

        self.mode = mode
        self.max_px = min(float(max_px_per_frame), float(MAX_PLATFORM_PX))
        self._t: float = 0.0
        self._rng = np.random.default_rng(seed)
        # Linear mode drift direction
        self._drift_x = float(self._rng.uniform(-self.max_px, self.max_px))
        self._drift_y = float(self._rng.uniform(-self.max_px, self.max_px))
        # Circular mode parameters
        self._circ_omega_x = float(self._rng.uniform(0.3, 0.8))
        self._circ_omega_y = float(self._rng.uniform(0.3, 0.8))
        self._circ_phase_x = float(self._rng.uniform(0, 2 * math.pi))
        self._circ_phase_y = float(self._rng.uniform(0, 2 * math.pi))
        # Random walk state
        self._rw_vx: float = 0.0
        self._rw_vy: float = 0.0

    def step(self, dt: float) -> tuple[float, float]:
        """
        Compute the platform displacement for this frame.

        Returns
        -------
        (dx, dy) : pixel displacement to apply to camera centre.
        """
        self._t += dt

        if self.mode == PlatformMotionMode.NONE:
            return 0.0, 0.0

        elif self.mode == PlatformMotionMode.LINEAR:
            # Constant drift; magnitude already bounded to max_px/frame
            # Scale drift_x/y so they represent per-frame displacement at dt
            dx = self._drift_x * dt
            dy = self._drift_y * dt
            return (
                float(np.clip(dx, -self.max_px, self.max_px)),
                float(np.clip(dy, -self.max_px, self.max_px)),
            )

        elif self.mode == PlatformMotionMode.CIRCULAR:
            # Sinusoidal oscillation (satellite attitude wobble model)
            dx = self.max_px * math.sin(self._circ_omega_x * self._t + self._circ_phase_x) * dt
            dy = self.max_px * math.cos(self._circ_omega_y * self._t + self._circ_phase_y) * dt
            return dx, dy

        elif self.mode == PlatformMotionMode.RANDOM:
            # Slow random walk in velocity space
            self._rw_vx += float(self._rng.normal(0, 2.0))
            self._rw_vy += float(self._rng.normal(0, 2.0))
            self._rw_vx = float(np.clip(self._rw_vx, -self.max_px, self.max_px))
            self._rw_vy = float(np.clip(self._rw_vy, -self.max_px, self.max_px))
            return self._rw_vx * dt, self._rw_vy * dt

        return 0.0, 0.0


# ---------------------------------------------------------------------------
# Disturbance Config bundle
# ---------------------------------------------------------------------------

class DisturbanceConfig:
    """
    Unified disturbance configuration for one simulation run.

    All fields are independently togglable for scenario presets.
    """

    def __init__(
        self,
        # Image noise
        gaussian_sigma: float = 0.0,
        salt_pepper: bool = False,
        salt_pepper_density: float = SP_NOISE_DENSITY,
        poisson: bool = False,
        poisson_scale: float = 1.0,
        # Atmospheric
        atmospheric: str | AtmosphericMode = AtmosphericMode.CLEAR,
        # Jitter
        jitter_enabled: bool = False,
        jitter_max_px: float = 10.0,
        jitter_sigma: float = 4.0,
        # Platform
        platform_mode: str | PlatformMotionMode = PlatformMotionMode.NONE,
        platform_max_px: float = 5.0,
        # RNG seed (None = random)
        seed: Optional[int] = None,
    ) -> None:
        self.gaussian_sigma     = gaussian_sigma
        self.salt_pepper        = salt_pepper
        self.salt_pepper_density = salt_pepper_density
        self.poisson            = poisson
        self.poisson_scale      = poisson_scale
        self.atmospheric        = (AtmosphericMode.from_string(atmospheric)
                                   if isinstance(atmospheric, str) else atmospheric)
        self.jitter             = CameraJitter(jitter_max_px, jitter_sigma, jitter_enabled, seed)
        self.platform           = PlatformMotion(platform_mode, platform_max_px, seed)
        self._rng               = np.random.default_rng(seed)

    def apply_to_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        Apply all enabled image-space disturbances to a viewport frame.

        Atmospheric attenuation → Poisson → Gaussian → Salt&Pepper.
        Camera jitter is NOT applied here — it is applied to the viewport
        crop origin in VirtualCamera.get_viewport() before this method is called.

        Parameters
        ----------
        frame : np.ndarray  HxW uint8 greyscale viewport.

        Returns
        -------
        distorted_frame : np.ndarray  HxW uint8.
        """
        # 1. Atmospheric attenuation
        out = apply_atmospheric(frame, self.atmospheric, rng=self._rng)

        # 2. Poisson noise (photon shot, before Gaussian readout noise)
        if self.poisson:
            out = apply_poisson_noise(out, self.poisson_scale)

        # 3. Gaussian noise (readout / thermal noise)
        if self.gaussian_sigma > 0:
            out = apply_gaussian_noise(out, self.gaussian_sigma, self._rng)

        # 4. Salt & Pepper
        if self.salt_pepper:
            out = apply_salt_pepper_noise(out, self.salt_pepper_density, self._rng)

        return out

    def platform_step(self, dt: float) -> tuple[float, float]:
        """Advance platform motion and return (dx, dy) camera displacement."""
        return self.platform.step(dt)
