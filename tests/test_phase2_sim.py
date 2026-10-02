"""
tests/test_phase2_sim.py
========================
Phase 2 Headless Verification Suite — BeaconLock Simulation Engine
===================================================================

Tests the complete simulation stack:
    simulation/motion_models.py   — all 4 mandatory motion models
    simulation/target.py          — beacon rendering + occlusion injection
    simulation/disturbances.py    — Gaussian, S&P, Poisson, atmospheric, jitter
    simulation/camera.py          — virtual PTZ camera
    simulation/scene.py           — world canvas renderer
    ingestion/sim_source.py       — SimulatorSource (Mode A FrameSource)

Test groups
-----------
T0  Import & instantiation smoke test
T1  Motion models — 100-frame trajectory verification (all 4 types)
T2  Beacon rendering — spot visible in viewport, correct position
T3  Disturbance engine — each disturbance type modifies frames measurably
T4  Camera kinematics — slew rate saturation, coordinate transforms
T5  Occlusion injection — beacon disappears for N frames then reappears
T6  Closed-loop tracking — 300 frames, Figure-8, Gaussian noise:
        SimulatorSource → CentroidEstimator → KalmanTracker → PDController
        → apply_camera_control() loop.
        Asserts: RMSE ≤ 10 px, FPS ≥ 30, rate saturation ≤ 5 °/s.

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import math
import sys
import time
import traceback
from pathlib import Path
from typing import Optional

import numpy as np

# ------------------------------------------------------------------ path setup
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# ------------------------------------------------------------------ imports
# Simulation layer
from simulation.motion_models import (
    StraightLineMotion, CircularMotion, Figure8Motion, RandomWalkMotion,
    create_motion_model, MotionType,
)
from simulation.target import BeaconTarget
from simulation.disturbances import (
    DisturbanceConfig, AtmosphericMode,
    apply_gaussian_noise, apply_salt_pepper_noise, apply_poisson_noise, apply_atmospheric,
)
from simulation.camera import VirtualCamera, FRAME_W, FRAME_H, WORLD_W, WORLD_H
from simulation.scene import WorldScene

# Ingestion layer
from ingestion.sim_source import SimulatorSource

# Perception & control (decoupled — never import from simulation directly)
from perception.centroid import CentroidEstimator
from perception.kalman_tracker import KalmanTracker
from control.pid_controller import PDController

# ===========================================================================
# Terminal helpers
# ===========================================================================

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

GRN = "\033[92m"; RED = "\033[91m"; YEL = "\033[93m"
CYN = "\033[96m"; BLD = "\033[1m";  RST = "\033[0m"

_passed = 0; _failed = 0; _results: list[dict] = []

FPS = 30.0; DT = 1.0 / FPS
FRAME_CX = FRAME_W // 2   # 320
FRAME_CY = FRAME_H // 2   # 240


def _header(title: str) -> None:
    print(f"\n{BLD}{CYN}{'─'*70}{RST}\n{BLD}{CYN}  {title}{RST}\n{BLD}{CYN}{'─'*70}{RST}")

def _pass(test: str, detail: str = "") -> None:
    global _passed; _passed += 1
    print(f"  {GRN}[PASS]{RST}  {test}" + (f"  — {detail}" if detail else ""))
    _results.append({"test": test, "status": "PASS", "detail": detail})

def _fail(test: str, detail: str = "") -> None:
    global _failed; _failed += 1
    print(f"  {RED}[FAIL]{RST}  {test}" + (f"  — {detail}" if detail else ""))
    _results.append({"test": test, "status": "FAIL", "detail": detail})

def _assert(cond: bool, name: str, pd: str = "", fd: str = "") -> bool:
    if cond: _pass(name, pd)
    else:    _fail(name, fd)
    return cond

def _rmse(errs: list[float]) -> float:
    if not errs: return float("inf")
    return math.sqrt(sum(e**2 for e in errs) / len(errs))


# ===========================================================================
# T0 — Smoke / imports
# ===========================================================================

def test_t0_imports() -> None:
    _header("T0 — Import & Instantiation Smoke Test")
    try:
        mm = StraightLineMotion()
        _assert(True, "T0.1 StraightLineMotion instantiated")

        t = BeaconTarget("figure_8")
        _assert(True, "T0.2 BeaconTarget('figure_8') instantiated")

        dc = DisturbanceConfig(gaussian_sigma=5.0, salt_pepper=True)
        _assert(True, "T0.3 DisturbanceConfig instantiated")

        cam = VirtualCamera()
        _assert(cam.position == (1000.0, 1000.0), "T0.4 Camera init at (1000,1000)",
                f"pos={cam.position}")

        src = SimulatorSource(motion_type="circular", fps=30.0)
        _assert(isinstance(src, SimulatorSource), "T0.5 SimulatorSource instantiated")

        ok, frame = src.read()
        _assert(ok and frame.shape == (480, 640), "T0.6 SimulatorSource.read() → (480,640)",
                f"shape={frame.shape}")

    except Exception as exc:
        _fail("T0 — Exception", str(exc)); traceback.print_exc()


# ===========================================================================
# T1 — Motion models: 100-frame trajectory verification
# ===========================================================================

def test_t1_motion_models() -> None:
    _header("T1 — Motion Models: 100-Frame Trajectory Verification")

    configs = [
        ("straight_line", StraightLineMotion(start_x=500, start_y=1000, vx=80, vy=20)),
        ("circular",      CircularMotion(center_x=1000, center_y=1000, radius=300, omega=0.5)),
        ("figure_8",      Figure8Motion(center_x=1000, center_y=1000, amp_x=300, amp_y=150, omega=0.4)),
        ("random_walk",   RandomWalkMotion(start_x=1000, start_y=1000, seed=42)),
    ]

    for name, model in configs:
        positions = []
        for _ in range(100):
            x, y = model.update(DT)
            positions.append((x, y))

        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]

        # All positions within world canvas
        all_in_world = all(0 <= x <= WORLD_W and 0 <= y <= WORLD_H for x, y in positions)
        _assert(all_in_world, f"T1.{name}: all 100 positions within 2000×2000 canvas",
                "✓ in bounds", f"out of bounds: min_x={min(xs):.0f} max_x={max(xs):.0f}")

        # Check motion actually moves (not frozen)
        total_displacement = sum(
            math.hypot(positions[i][0]-positions[i-1][0], positions[i][1]-positions[i-1][1])
            for i in range(1, len(positions))
        )
        _assert(total_displacement > 10.0, f"T1.{name}: beacon actually moves (total disp > 10px)",
                f"disp={total_displacement:.1f} px", f"disp={total_displacement:.1f} px (frozen?)")

        print(f"       {name}: x∈[{min(xs):.0f},{max(xs):.0f}] y∈[{min(ys):.0f},{max(ys):.0f}]"
              f"  total_disp={total_displacement:.0f} px")


# ===========================================================================
# T2 — Beacon rendering: spot visible in viewport
# ===========================================================================

def test_t2_beacon_rendering() -> None:
    _header("T2 — Beacon Rendering: Spot Visibility & Position")

    # Place beacon at canvas centre, camera at canvas centre
    model = CircularMotion(center_x=1000, center_y=1000, radius=0, omega=0)
    target = BeaconTarget(motion_model=model, size=10, brightness=255)
    target.update(DT)   # advance once to set position

    scene = WorldScene(background=0, targets=[target])
    camera = VirtualCamera(init_x=1000, init_y=1000)

    canvas = scene.render()
    viewport = camera.get_viewport(canvas)

    # Beacon should be at frame centre (320, 240) — within ±10px
    peak_row, peak_col = np.unravel_index(np.argmax(viewport), viewport.shape)
    err = math.hypot(peak_col - FRAME_CX, peak_row - FRAME_CY)
    _assert(err < 10.0, "T2.1 Beacon peak within ±10px of frame centre",
            f"err={err:.2f} px at ({peak_col},{peak_row})",
            f"err={err:.2f} px — beacon not centred")

    _assert(int(viewport.max()) == 255, "T2.2 Beacon peak brightness = 255",
            f"max={int(viewport.max())}")

    # Check beacon occupies expected area (~100 px for 10×10)
    bright_px = int(np.sum(viewport > 200))
    _assert(bright_px >= 80, "T2.3 Beacon occupies ≥80 bright pixels (10×10 spot)",
            f"bright_px={bright_px}", f"bright_px={bright_px} < 80")

    # Ground truth coordinate transform
    bx, by = target.position
    col_gt, row_gt = camera.world_to_camera(bx, by)
    _assert(abs(col_gt - FRAME_CX) < 5, "T2.4 world_to_camera() maps beacon to frame centre",
            f"col_gt={col_gt:.1f} row_gt={row_gt:.1f}")

    print(f"       Beacon at world ({bx:.1f},{by:.1f}) → camera ({col_gt:.1f},{row_gt:.1f})"
          f"  peak at ({peak_col},{peak_row})  bright_px={bright_px}")


# ===========================================================================
# T3 — Disturbance engine
# ===========================================================================

def test_t3_disturbances() -> None:
    _header("T3 — Disturbance Engine: All Disturbance Types")

    rng = np.random.default_rng(0)
    # Reference clean frame with a bright spot
    ref = np.zeros((480, 640), dtype=np.uint8)
    ref[230:250, 310:330] = 200   # 20×20 beacon proxy

    # --- Gaussian noise
    noisy_g = apply_gaussian_noise(ref.copy(), sigma=15.0, rng=rng)
    diff_g = float(np.abs(noisy_g.astype(int) - ref.astype(int)).mean())
    _assert(diff_g > 2.0, "T3.1 Gaussian σ=15 changes mean pixel value by >2",
            f"mean_diff={diff_g:.2f}", f"mean_diff={diff_g:.2f} ≤ 2 (no effect?)")

    # --- Salt & Pepper
    noisy_sp = apply_salt_pepper_noise(ref.copy(), density=0.10, rng=rng)
    # Use direct pixel diff: old method (n_salt + n_pepper) fails because pepper pixels
    # landing on already-zero background don't change the zero-count, cancelling salt count.
    sp_changed = int(np.sum(noisy_sp != ref))
    # With 10% corruption on 640×480 = 30720 corrupted px; half are salt (0→255),
    # half are pepper (may land on 0→0 background). Salt pixels always differ from ref.
    # Expect ≥ 12000 reliably (conservatively half of 30720 accounting for overlap).
    _assert(sp_changed >= 12000,
            "T3.2 S&P 10%: ≥12000 pixels changed (640×480×0.10=30720 total corrupt)",
            f"changed={sp_changed}", f"changed={sp_changed} < 12000")

    # --- Poisson noise
    bright_ref = np.full((480, 640), 200, dtype=np.uint8)
    noisy_p = apply_poisson_noise(bright_ref, scale=1.0)
    diff_p = float(np.std(noisy_p.astype(float) - bright_ref.astype(float)))
    _assert(diff_p > 0.5, "T3.3 Poisson noise introduces std dev >0.5 on uniform bright frame",
            f"std={diff_p:.3f}", f"std={diff_p:.3f} ≤ 0.5")

    # --- Fog: reduces contrast and brightness
    bright_frame = np.full((480, 640), 180, dtype=np.uint8)
    fogged = apply_atmospheric(bright_frame, AtmosphericMode.FOG)
    fog_mean = float(fogged.mean())
    _assert(fog_mean < 175.0, "T3.4 Fog reduces pixel mean from 180",
            f"mean={fog_mean:.1f}", f"mean={fog_mean:.1f} ≥ 175")

    # --- Haze
    hazed = apply_atmospheric(bright_frame, AtmosphericMode.HAZE)
    haze_mean = float(hazed.mean())
    _assert(haze_mean < 180.0, "T3.5 Haze modifies pixel mean from 180",
            f"mean={haze_mean:.1f}", f"mean={haze_mean:.1f} unchanged")

    # --- Low light
    ll = apply_atmospheric(bright_frame, AtmosphericMode.LOW_LIGHT)
    ll_mean = float(ll.mean())
    _assert(ll_mean < 90.0, "T3.6 Low-light reduces mean significantly (<90)",
            f"mean={ll_mean:.1f}", f"mean={ll_mean:.1f} ≥ 90")

    # --- Rain
    rainy = apply_atmospheric(ref.copy(), AtmosphericMode.RAIN, rng=np.random.default_rng(7))
    _assert(rainy is not None and rainy.shape == (480, 640), "T3.7 Rain returns 480×640 frame")

    # --- Camera Jitter
    dc_jitter = DisturbanceConfig(jitter_enabled=True, jitter_max_px=20.0, seed=1)
    jx, jy = dc_jitter.jitter.sample()
    _assert(abs(jx) <= 20.0 and abs(jy) <= 20.0,
            "T3.8 Jitter displacement within ±20 px spec",
            f"jx={jx:.1f} jy={jy:.1f}", f"jx={jx:.1f} jy={jy:.1f} out of spec")

    print(f"       Gaussian Δmean={diff_g:.2f}  S&P changed={sp_changed}  "
          f"Poisson σ={diff_p:.3f}  Fog={fog_mean:.1f} (Koschmieder: 180→~145)  LL={ll_mean:.1f}")


# ===========================================================================
# T4 — Camera kinematics
# ===========================================================================

def test_t4_camera_kinematics() -> None:
    _header("T4 — Camera Kinematics: Slew Rate, Saturation, Coordinate Transforms")

    cam = VirtualCamera(init_x=1000.0, init_y=1000.0, max_rate_deg_s=5.0)

    # Apply max-rate pan command for 10 frames
    for _ in range(10):
        cam.apply_control(pan_rate_deg_s=5.0, tilt_rate_deg_s=0.0, dt=DT)

    # Expected x displacement: 5°/s × 1/30s × 160px/° × 10 frames ≈ 266.7 px
    expected_dx = 5.0 * DT * 160.0 * 10
    actual_dx = cam.position[0] - 1000.0
    _assert(abs(actual_dx - expected_dx) < 1.0, "T4.1 Pan 5°/s × 10 frames = correct displacement",
            f"actual={actual_dx:.1f} expected={expected_dx:.1f} px")

    # Saturation: request 20°/s → should be clamped to 5°/s
    cam2 = VirtualCamera(init_x=1000.0, init_y=1000.0, max_rate_deg_s=5.0)
    cam2.apply_control(pan_rate_deg_s=20.0, tilt_rate_deg_s=0.0, dt=DT)
    dx_clamped = cam2.position[0] - 1000.0
    dx_unclamped = 20.0 * DT * 160.0  # what it would be without clamping
    dx_clamped_expected = 5.0 * DT * 160.0
    _assert(abs(dx_clamped - dx_clamped_expected) < 0.5,
            "T4.2 20°/s command clamped to 5°/s output",
            f"dx={dx_clamped:.2f} (clamped) vs unclamped={dx_unclamped:.2f}")
    _assert(cam2.saturation_count == 1, "T4.3 Saturation counter incremented",
            f"count={cam2.saturation_count}")

    # Coordinate transform round-trip
    cam3 = VirtualCamera(init_x=1200.0, init_y=900.0)
    wx_test, wy_test = 1300.0, 800.0
    col, row = cam3.world_to_camera(wx_test, wy_test)
    wx_back, wy_back = cam3.camera_to_world(col, row)
    _assert(abs(wx_back - wx_test) < 0.01 and abs(wy_back - wy_test) < 0.01,
            "T4.4 world→camera→world round-trip error < 0.01 px",
            f"Δ=({abs(wx_back-wx_test):.4f},{abs(wy_back-wy_test):.4f})")

    # Camera bounds: should not go outside [320,1680]×[240,1760]
    cam4 = VirtualCamera(init_x=1000.0, init_y=1000.0)
    for _ in range(500):
        cam4.apply_control(10.0, 10.0, DT)  # keep commanding beyond bounds
    cx, cy = cam4.position
    _assert(320 <= cx <= 1680 and 240 <= cy <= 1760,
            "T4.5 Camera position clamped within valid viewport bounds",
            f"pos=({cx:.0f},{cy:.0f})")

    print(f"       Pan 10 frames: Δx={actual_dx:.1f}px (expected {expected_dx:.1f}px) "
          f"| Sat clamped to {dx_clamped:.2f}px")


# ===========================================================================
# T5 — Occlusion injection
# ===========================================================================

def test_t5_occlusion() -> None:
    _header("T5 — Occlusion Injection: Beacon Blanking & Re-appearance")

    src = SimulatorSource(motion_type="straight_line", fps=30.0, seed=0)

    # Warm up 5 frames
    for _ in range(5):
        ok, frame = src.read()

    # Inject 8-frame occlusion
    src.inject_occlusion(8)

    occluded_frames = 0
    visible_frames  = 0
    for i in range(15):
        ok, frame = src.read()
        gt = src.ground_truth

        if gt.occluded:
            occluded_frames += 1
            # Frame should be dark (no bright beacon)
            bright_px = int(np.sum(frame > 200))
            _assert(bright_px < 50 or i == 0,
                    f"T5.occlude_f{i} Frame is dark during occlusion",
                    f"bright={bright_px}", f"bright={bright_px} (beacon still visible!)")
        else:
            visible_frames += 1

    _assert(occluded_frames == 8, "T5.1 Exactly 8 occluded frames",
            f"occluded={occluded_frames}", f"occluded={occluded_frames} ≠ 8")
    _assert(visible_frames == 7, "T5.2 Exactly 7 visible frames (5 pre + 2 post)",
            f"visible={visible_frames}", f"visible={visible_frames} ≠ 7")
    print(f"       Occluded={occluded_frames} frames | Visible={visible_frames} frames")


# ===========================================================================
# T6 — Closed-loop tracking: Figure-8, Gaussian noise
# ===========================================================================

def test_t6_closed_loop() -> None:
    _header("T6 — Closed-Loop Tracking: Figure-8 + Gaussian σ=8, 300 Frames")

    # Build simulation source: figure-8 around canvas centre
    dist = DisturbanceConfig(
        gaussian_sigma=8.0,
        atmospheric="clear",
        seed=42,
    )
    src = SimulatorSource(
        motion_type="figure_8",
        motion_kwargs={
            "amp_x": 200.0,
            "amp_y": 100.0,
            "omega": 0.35,
        },
        beacon_size=10,
        disturbance=dist,
        camera_init_x=1000.0,
        camera_init_y=1000.0,
        fps=30.0,
        seed=42,
    )

    # Perception + control stack (decoupled — no sim imports)
    est  = CentroidEstimator()
    kf   = KalmanTracker(dt=DT)
    ctrl = PDController(kp=2.5, kd=0.3, max_rate_deg_s=5.0)   # aggressive gains for fast tracking

    N_FRAMES   = 300
    N_WARMUP   = 20    # frames before RMSE measurement starts
    errors_px  = []    # Euclidean centroid error from frame centre (= tracking error)
    gt_errors  = []    # error vs ground truth beacon position (for RMSE)
    fps_times  = []
    max_cmd    = 0.0   # max control command seen
    n_detected = 0

    for i in range(N_FRAMES):
        t0 = time.perf_counter()

        # --- Simulate
        ok, frame = src.read()
        gt_col, gt_row = src.ground_truth_camera   # eval-layer use only

        # --- Perceive
        roi = (float(kf.state[0]), float(kf.state[1])) if kf.is_initialised else None
        cog = est.estimate(frame, roi_centre=roi)

        if cog.detected:
            n_detected += 1
            kr = kf.step(cog.cx, cog.cy, cog.confidence)
        else:
            kr = kf.step(None, None, confidence=0.0)

        # --- Error in camera frame (centroid distance from frame centre)
        track_err = math.hypot(kr.predicted_x - FRAME_CX, kr.predicted_y - FRAME_CY)

        # --- Ground-truth RMSE: predicted vs actual beacon in camera frame
        gt_err = math.hypot(kr.predicted_x - gt_col, kr.predicted_y - gt_row)

        if i >= N_WARMUP:
            errors_px.append(track_err)
            gt_errors.append(gt_err)

        # --- Control
        err_x = kr.predicted_x - FRAME_CX
        err_y = kr.predicted_y - FRAME_CY
        cmd = ctrl.update(err_x, err_y, dt=DT)
        max_cmd = max(max_cmd, abs(cmd.pan_rate_deg_s), abs(cmd.tilt_rate_deg_s))

        # --- Actuate camera
        src.apply_camera_control(cmd.pan_rate_deg_s, cmd.tilt_rate_deg_s, DT)

        fps_times.append(time.perf_counter() - t0)

    # --- Metrics
    rmse_track = _rmse(errors_px)
    rmse_gt    = _rmse(gt_errors)
    detection_rate = n_detected / N_FRAMES
    avg_fps = 1.0 / (sum(fps_times) / len(fps_times))
    total_ms = sum(fps_times) * 1000.0

    _assert(rmse_gt <= 10.0, "T6.1 Ground-truth RMSE ≤ 10 px (ISRO tracking error spec)",
            f"RMSE = {rmse_gt:.2f} px", f"RMSE = {rmse_gt:.2f} px > 10")
    _assert(max_cmd <= 5.0, "T6.2 Max control command ≤ 5.0 °/s (rate saturation enforced)",
            f"max_cmd = {max_cmd:.3f} °/s", f"max_cmd = {max_cmd:.3f} °/s > 5.0")
    _assert(avg_fps >= 30.0, "T6.3 Simulation+perception FPS ≥ 30 (ISRO camera update rate)",
            f"{avg_fps:.1f} FPS  ({total_ms/N_FRAMES:.2f} ms/frame)",
            f"{avg_fps:.1f} FPS < 30")
    _assert(detection_rate >= 0.85, "T6.4 Detection rate ≥ 85% during closed-loop",
            f"{detection_rate*100:.1f}%", f"{detection_rate*100:.1f}% < 85%")
    _assert(src.camera.saturation_count < N_FRAMES * 0.30,
            "T6.5 Rate saturation in < 30% of frames",
            f"sat_frames={src.camera.saturation_count}/{N_FRAMES}",
            f"sat_frames={src.camera.saturation_count}/{N_FRAMES} ≥ 30%")

    print(f"       RMSE_gt={rmse_gt:.2f} px  RMSE_track={rmse_track:.2f} px  "
          f"detected={detection_rate*100:.1f}%  max_cmd={max_cmd:.3f}°/s  "
          f"FPS={avg_fps:.1f}  sat={src.camera.saturation_count}/{N_FRAMES}")


# ===========================================================================
# Summary
# ===========================================================================

def _print_summary() -> bool:
    total = _passed + _failed
    print(f"\n{BLD}{'='*70}{RST}")
    print(f"{BLD}  BEACONLOCK PHASE 2 — TEST SUMMARY{RST}")
    print(f"{BLD}{'='*70}{RST}")
    print(f"  Total:   {total}")
    print(f"  {GRN}Passed:  {_passed}{RST}")
    print(f"  {'Failed:  '+str(_failed) if _failed else 'Failed:  0'}")
    pass_rate = (_passed / total * 100) if total else 0.0
    colour = GRN if _failed == 0 else (YEL if pass_rate >= 80 else RED)
    print(f"\n  {colour}{BLD}Pass rate: {pass_rate:.1f}%{RST}")
    print(f"\n{'─'*70}")
    print(f"  {'Test':<56}  {'Status':>6}")
    print(f"{'─'*70}")
    for r in _results:
        sc = GRN if r["status"] == "PASS" else RED
        print(f"  {r['test']:<56}  {sc}{r['status']:>6}{RST}")
    print(f"{'─'*70}\n")
    return _failed == 0


# ===========================================================================
# Entry point
# ===========================================================================

if __name__ == "__main__":
    print(f"{BLD}")
    print("╔══════════════════════════════════════════════════════════════════════╗")
    print("║       BEACONLOCK — PHASE 2 SIMULATION ENGINE VERIFICATION SUITE     ║")
    print("║       ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)            ║")
    print("╚══════════════════════════════════════════════════════════════════════╝")
    print(RST)

    test_t0_imports()
    test_t1_motion_models()
    test_t2_beacon_rendering()
    test_t3_disturbances()
    test_t4_camera_kinematics()
    test_t5_occlusion()
    test_t6_closed_loop()

    all_passed = _print_summary()
    sys.exit(0 if all_passed else 1)
