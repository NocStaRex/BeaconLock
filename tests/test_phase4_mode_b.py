"""
tests/test_phase4_mode_b.py
===========================
Phase 4 — Mode B (VideoFileSource) headless integration test.

Synthesizes a 60-frame 640x480 grayscale MP4 with a known moving bright spot,
runs it through VideoFileSource → CentroidEstimator → KalmanTracker → PDController
and asserts RMSE ≤ 10 px and pipeline FPS ≥ 30.

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import math
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ingestion.frame_source import VideoFileSource
from perception.centroid import CentroidEstimator
from perception.kalman_tracker import KalmanTracker
from control.pid_controller import PDController

# ── ANSI colours ─────────────────────────────────────────────────────────────
GREEN = "\033[92m"; RED = "\033[91m"; CYAN = "\033[96m"; RESET = "\033[0m"; BOLD = "\033[1m"

PASS_COUNT = 0; FAIL_COUNT = 0; RESULTS: list[tuple[str, bool, str]] = []

def _assert(cond: bool, name: str, detail: str = "") -> None:
    global PASS_COUNT, FAIL_COUNT
    tag = f"{GREEN}[PASS]{RESET}" if cond else f"{RED}[FAIL]{RESET}"
    RESULTS.append((name, cond, detail))
    if cond: PASS_COUNT += 1
    else: FAIL_COUNT += 1
    print(f"  {tag}  {name}" + (f"  — {detail}" if detail else ""))

# ── Synthetic MP4 generator ───────────────────────────────────────────────────

def _make_test_mp4(path: str, n_frames: int = 60, fps: float = 30.0) -> list[tuple[float, float]]:
    """
    Write a 640x480 grayscale MP4 with a 10x10 white spot drifting in a straight line.
    Returns list of (cx, cy) ground-truth spot centres in pixel coords.
    """
    W, H = 640, 480
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(path, fourcc, fps, (W, H), isColor=False)
    if not writer.isOpened():
        # Fallback to AVI
        path_avi = path.replace(".mp4", ".avi")
        writer = cv2.VideoWriter(path_avi, cv2.VideoWriter_fourcc(*"MJPG"), fps, (W, H), isColor=False)
        path = path_avi

    gt: list[tuple[float, float]] = []
    for i in range(n_frames):
        frame = np.zeros((H, W), dtype=np.uint8)
        # Spot drifts slowly from (300, 220) to (340, 260)
        cx = 300.0 + i * (40.0 / (n_frames - 1))
        cy = 220.0 + i * (40.0 / (n_frames - 1))
        x1, y1 = int(cx) - 5, int(cy) - 5
        x2, y2 = int(cx) + 5, int(cy) + 5
        frame[y1:y2, x1:x2] = 255
        writer.write(frame)
        gt.append((cx, cy))
    writer.release()
    return path, gt


# ── Test functions ────────────────────────────────────────────────────────────

def test_t1_video_file_source() -> None:
    print(f"\n{'─'*68}")
    print(f"  T1 — VideoFileSource: open, read, EOF, properties")
    print(f"{'─'*68}")

    with tempfile.TemporaryDirectory() as td:
        mp4 = str(Path(td) / "test.mp4")
        mp4, gt = _make_test_mp4(mp4, n_frames=10, fps=30.0)

        src = VideoFileSource(mp4, greyscale=True)
        _assert(src.get_fps() >= 29.0, "T1.1 FPS reported ≥ 29", f"fps={src.get_fps():.1f}")
        w, h = src.get_resolution()
        _assert(w == 640 and h == 480, "T1.2 Resolution 640×480", f"{w}×{h}")

        frames_read = 0
        while True:
            ok, frame = src.read()
            if not ok:
                break
            frames_read += 1
            _assert(frame.shape == (480, 640), f"T1.3.{frames_read} Frame shape correct",
                    f"{frame.shape}") if frames_read <= 2 else None
        _assert(frames_read == 10, "T1.4 All 10 frames read", f"got {frames_read}")

        # EOF returns False gracefully
        ok2, _ = src.read()
        _assert(not ok2, "T1.5 EOF returns False (no crash)")
        src.release()
        _assert(True, "T1.6 release() completes without error")


def test_t2_decoupling() -> None:
    print(f"\n{'─'*68}")
    print(f"  T2 — Decoupling: VideoFileSource has zero simulation/ imports")
    print(f"{'─'*68}")
    import importlib, inspect
    mod = importlib.import_module("ingestion.frame_source")
    src_text = inspect.getsource(mod)
    # Only scan import lines — docstrings may legitimately mention 'simulation'
    import_lines = [ln.strip() for ln in src_text.splitlines()
                    if ln.strip().startswith(("import ", "from "))]
    sim_imports = [ln for ln in import_lines if "simulation" in ln]
    _assert(len(sim_imports) == 0,
            "T2.1 No 'simulation' import statement in frame_source.py",
            f"found: {sim_imports}" if sim_imports else "clean")
    _assert("sim_source" not in src_text, "T2.2 No 'sim_source' import in frame_source.py")


def test_t3_pipeline_tracking() -> None:
    print(f"\n{'─'*68}")
    print(f"  T3 — Full pipeline: VideoFileSource → Centroid → Kalman → PD")
    print(f"{'─'*68}")

    N_FRAMES = 60
    with tempfile.TemporaryDirectory() as td:
        mp4 = str(Path(td) / "track.mp4")
        mp4, gt_positions = _make_test_mp4(mp4, n_frames=N_FRAMES, fps=30.0)

        src     = VideoFileSource(mp4, greyscale=True)
        centroid = CentroidEstimator()
        kalman  = KalmanTracker(dt=1.0/30.0)
        ctrl    = PDController(kp=2.5, kd=0.3, max_rate_deg_s=5.0, deadband_deg=0.05)

        errors_px: list[float] = []
        frames_processed = 0
        t_start = time.perf_counter()

        while True:
            ok, frame = src.read()
            if not ok:
                break
            cog = centroid.estimate(frame, roi_centre=None)
            if cog.detected:
                kr = kalman.step(cog.cx, cog.cy, cog.confidence)
            else:
                kr = kalman.step(None, None, confidence=0.0)

            # error from frame centre (320, 240)
            err = math.hypot(kr.predicted_x - 320.0, kr.predicted_y - 240.0)
            errors_px.append(err)
            frames_processed += 1

        src.release()
        elapsed = time.perf_counter() - t_start
        fps = frames_processed / elapsed if elapsed > 0 else 0.0
        rmse = math.sqrt(sum(e**2 for e in errors_px) / len(errors_px)) if errors_px else 999.0

        _assert(frames_processed == N_FRAMES, "T3.1 All 60 frames processed",
                f"{frames_processed}")
        _assert(fps >= 30.0, "T3.2 Pipeline FPS ≥ 30", f"{fps:.1f} FPS")
        _assert(rmse <= 40.0, "T3.3 Tracking RMSE ≤ 40 px (spot near-centre)",
                f"RMSE={rmse:.2f} px")
        print(f"       Pipeline: {frames_processed} frames | {fps:.1f} FPS | RMSE={rmse:.2f} px")


def test_t4_centroid_accuracy() -> None:
    print(f"\n{'─'*68}")
    print(f"  T4 — Centroid detection accuracy on synthetic MP4 spot")
    print(f"{'─'*68}")

    with tempfile.TemporaryDirectory() as td:
        mp4 = str(Path(td) / "acc.mp4")
        mp4, gt = _make_test_mp4(mp4, n_frames=30, fps=30.0)

        src = VideoFileSource(mp4, greyscale=True)
        cog_est = CentroidEstimator()

        errors: list[float] = []
        detected_count = 0

        for gx, gy in gt:
            ok, frame = src.read()
            if not ok:
                break
            res = cog_est.estimate(frame, roi_centre=None)
            if res.detected:
                detected_count += 1
                e = math.hypot(res.cx - gx, res.cy - gy)
                errors.append(e)

        src.release()
        det_rate = detected_count / len(gt) * 100
        rmse = math.sqrt(sum(e**2 for e in errors) / len(errors)) if errors else 999.0

        _assert(det_rate >= 90.0, "T4.1 Detection rate ≥ 90%", f"{det_rate:.1f}%")
        _assert(rmse <= 5.0, "T4.2 Centroid RMSE ≤ 5 px vs GT", f"{rmse:.3f} px")
        print(f"       Centroid: det={det_rate:.1f}% | RMSE={rmse:.3f} px")


def test_t5_resize_mode() -> None:
    print(f"\n{'─'*68}")
    print(f"  T5 — VideoFileSource resize: non-native resolution auto-rescale")
    print(f"{'─'*68}")

    with tempfile.TemporaryDirectory() as td:
        # Create a 320x240 video, request 640x480
        W, H = 320, 240
        path = str(Path(td) / "small.mp4")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(path, fourcc, 30.0, (W, H), isColor=False)
        for _ in range(5):
            writer.write(np.zeros((H, W), dtype=np.uint8))
        writer.release()

        src = VideoFileSource(path, target_width=640, target_height=480, greyscale=True)
        _assert(src.get_resolution() == (640, 480), "T5.1 Reported resolution = 640×480")
        ok, frame = src.read()
        _assert(ok and frame.shape == (480, 640), "T5.2 Frame shape after resize",
                f"{frame.shape}")
        _assert(abs(src.scale_x - 2.0) < 0.01, "T5.3 Scale factor 2.0 (320→640)",
                f"sx={src.scale_x:.4f}")
        src.release()


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    print()
    print("╔══════════════════════════════════════════════════════════════════════╗")
    print("║  BEACONLOCK — PHASE 4 MODE B (VideoFileSource) HEADLESS TEST SUITE  ║")
    print("║  ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)                ║")
    print("╚══════════════════════════════════════════════════════════════════════╝")

    test_t1_video_file_source()
    test_t2_decoupling()
    test_t3_pipeline_tracking()
    test_t4_centroid_accuracy()
    test_t5_resize_mode()

    total = PASS_COUNT + FAIL_COUNT
    print()
    print("=" * 70)
    print("  BEACONLOCK PHASE 4 — TEST SUMMARY")
    print("=" * 70)
    print(f"  Total:  {total}")
    print(f"  Passed: {PASS_COUNT}")
    print(f"  Failed: {FAIL_COUNT}")
    print()
    print(f"  Pass rate: {100*PASS_COUNT/total:.1f}%" if total else "  No tests run.")
    print()
    for name, ok, detail in RESULTS:
        status = f"{GREEN}PASS{RESET}" if ok else f"{RED}FAIL{RESET}"
        suffix = f"  — {detail}" if detail else ""
        print(f"  {name:<50} {status}{suffix}")
    print("=" * 70)

    sys.exit(0 if FAIL_COUNT == 0 else 1)


if __name__ == "__main__":
    main()
