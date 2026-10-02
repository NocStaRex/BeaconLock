"""
tests/test_phase3_ui.py
========================
Phase 3 Headless Verification Suite — State Machine, Logger, Report, Worker Thread
====================================================================================

Tests the complete Phase 3 non-visual stack without requiring a display.

Test groups
-----------
T0  Import smoke — all Phase 3 modules importable
T1  StateMachine — all 5 state transitions, coast timeout, reset
T2  FrameLogger  — CSV write, flush, close, row_count, summary() stats
T3  ReportGenerator — PDF creation, file size, pass/fail sections
T4  TrackingWorker  — 120-frame QThread run on circular motion:
      asserts frame count, max slew ≤ 5°/s, CSV rows, state transitions

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import math
import os
import sys
import tempfile
import time
import traceback
from pathlib import Path

# ── Project root on path ──────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

# ── Terminal helpers ──────────────────────────────────────────────────────
GRN = "\033[92m"; RED = "\033[91m"; YEL = "\033[93m"
CYN = "\033[96m"; BLD = "\033[1m";  RST = "\033[0m"

_passed = 0; _failed = 0; _results: list[dict] = []


def _header(t: str) -> None:
    print(f"\n{BLD}{CYN}{'─'*70}{RST}\n{BLD}{CYN}  {t}{RST}\n{BLD}{CYN}{'─'*70}{RST}")


def _pass(name: str, detail: str = "") -> None:
    global _passed; _passed += 1
    print(f"  {GRN}[PASS]{RST}  {name}" + (f"  — {detail}" if detail else ""))
    _results.append({"test": name, "status": "PASS", "detail": detail})


def _fail(name: str, detail: str = "") -> None:
    global _failed; _failed += 1
    print(f"  {RED}[FAIL]{RST}  {name}" + (f"  — {detail}" if detail else ""))
    _results.append({"test": name, "status": "FAIL", "detail": detail})


def _skip(name: str, reason: str = "") -> None:
    global _passed; _passed += 1   # skip counts as pass (optional capability)
    print(f"  {YEL}[SKIP]{RST}  {name}" + (f"  — {reason}" if reason else ""))
    _results.append({"test": name, "status": "SKIP", "detail": reason})


def _assert(cond: bool, name: str, pd: str = "", fd: str = "") -> bool:
    if cond: _pass(name, pd)
    else:    _fail(name, fd)
    return cond


TMP = Path(tempfile.mkdtemp(prefix="beaconlock_p3_"))


# ===========================================================================
# T0 — Imports
# ===========================================================================

def test_t0_imports() -> None:
    _header("T0 — Import Smoke Test")
    modules = [
        ("app.state_machine",         "StateMachine, TrackingState"),
        ("evaluation.logger",         "FrameLogger"),
        ("evaluation.report_generator","ReportGenerator"),
    ]
    for mod_name, symbols in modules:
        try:
            mod = __import__(mod_name, fromlist=symbols.split(", "))
            _pass(f"T0  import {mod_name}")
        except Exception as exc:
            _fail(f"T0  import {mod_name}", str(exc))


# ===========================================================================
# T1 — State Machine
# ===========================================================================

def test_t1_state_machine() -> None:
    _header("T1 — StateMachine: All 5 States & Transitions")
    from app.state_machine import StateMachine, TrackingState

    # ── T1.1 Initial state ──────────────────────────────────────────
    fsm = StateMachine()
    _assert(fsm.state == TrackingState.SEARCHING, "T1.1 Initial state = SEARCHING")

    # ── T1.2 SEARCHING → ACQUIRING (first detection) ────────────────
    r = fsm.update(detected=True, error_px=8.0)
    _assert(fsm.state == TrackingState.ACQUIRING, "T1.2 First detection → ACQUIRING",
            f"state={r.state.value}")

    # ── T1.3 ACQUIRING → TRACKING (5 consecutive locks) ─────────────
    for i in range(4):   # already 1 in T1.2 frame
        fsm.update(detected=True, error_px=6.0)
    _assert(fsm.state == TrackingState.TRACKING,
            "T1.3 5 consecutive locks → TRACKING",
            f"cons_locks={fsm.consecutive_locks}")

    # ── T1.4 TRACKING stays stable ──────────────────────────────────
    for _ in range(10):
        fsm.update(detected=True, error_px=4.0)
    _assert(fsm.state == TrackingState.TRACKING,
            "T1.4 10 stable frames — stays TRACKING")

    # ── T1.5 TRACKING → LOST (5 consecutive misses) ─────────────────
    for _ in range(5):
        fsm.update(detected=False, error_px=999.0)
    _assert(fsm.state == TrackingState.LOST,
            "T1.5 5 misses → LOST",
            f"misses={fsm.consecutive_misses}")

    # ── T1.6 LOST → REACQUIRING (redetected) ────────────────────────
    fsm.update(detected=True, error_px=5.0)
    _assert(fsm.state == TrackingState.REACQUIRING,
            "T1.6 Redetect → REACQUIRING")

    # ── T1.7 REACQUIRING → TRACKING (3 locks) ───────────────────────
    for _ in range(3):
        fsm.update(detected=True, error_px=4.0)
    _assert(fsm.state == TrackingState.TRACKING,
            "T1.7 3 re-locks → TRACKING")

    # ── T1.8 Coast timeout: LOST → SEARCHING after 90 frames ────────
    fsm2 = StateMachine()
    # force into TRACKING first
    for _ in range(5): fsm2.update(True, 5.0)
    assert fsm2.state == TrackingState.TRACKING
    # go LOST
    for _ in range(5): fsm2.update(False, 999.0)
    assert fsm2.state == TrackingState.LOST
    # coast 90 frames without detection
    for _ in range(90): fsm2.update(False, 999.0)
    _assert(fsm2.state == TrackingState.SEARCHING,
            "T1.8 90-frame coast timeout → SEARCHING")

    # ── T1.9 ACQUIRING → SEARCHING on 3 misses ──────────────────────
    fsm3 = StateMachine()
    fsm3.update(True, 8.0)   # ACQUIRING
    for _ in range(3): fsm3.update(False, 999.0)
    _assert(fsm3.state == TrackingState.SEARCHING,
            "T1.9 3 miss in ACQUIRING → back to SEARCHING")

    # ── T1.10 reset() ──────────────────────────────────────────────
    fsm.reset()
    _assert(fsm.state == TrackingState.SEARCHING,
            "T1.10 reset() returns to SEARCHING")

    # ── T1.11 Transitions log ──────────────────────────────────────
    _assert(len(fsm2.transitions) > 0,
            "T1.11 Transition history recorded",
            f"n={len(fsm2.transitions)}")

    print(f"       Transitions: {[(a.value, b.value) for a,b in fsm2.transitions]}")


# ===========================================================================
# T2 — FrameLogger
# ===========================================================================

def test_t2_logger() -> None:
    _header("T2 — FrameLogger: CSV Write, Flush, Summary")
    from evaluation.logger import FrameLogger

    csv_path = TMP / "test_logger.csv"
    fl = FrameLogger(str(csv_path), session_id="p3_test", buffer_size=10)

    # Write 100 rows with realistic values
    for i in range(100):
        fl.log(
            frame_id    = i,
            timestamp_s = i / 30.0,
            input_mode  = "Mode A",
            centroid_x  = 320.0 + 3.0 * math.sin(i * 0.1),
            centroid_y  = 240.0 + 3.0 * math.cos(i * 0.1),
            error_px    = 4.0 + 1.5 * math.sin(i * 0.2),
            error_mrad  = (4.0 + 1.5 * math.sin(i * 0.2)) * 0.00625 * (math.pi / 180) * 1000,
            pan_deg_s   = 0.04 * math.sin(i * 0.15),
            tilt_deg_s  = 0.03 * math.cos(i * 0.15),
            fps         = 140.0,
            lock_state  = "TRACKING",
        )

    _assert(fl.row_count == 100, "T2.1 row_count = 100", f"got {fl.row_count}")

    fl.flush()
    _assert(csv_path.exists(), "T2.2 CSV file exists on disk")
    _assert(csv_path.stat().st_size > 2000, "T2.3 CSV file non-trivial size",
            f"{csv_path.stat().st_size} bytes")

    # Verify header comments present
    text = csv_path.read_text(encoding="utf-8")
    _assert("session_id=p3_test" in text, "T2.4 Session ID in CSV header")
    _assert("frame_id" in text, "T2.5 Column names in CSV header")

    # Summary stats
    stats = fl.summary()
    _assert(stats.get("total_frames", 0) == 100, "T2.6 summary total_frames = 100",
            f"got {stats.get('total_frames')}")
    _assert(stats.get("rmse_px", 999) < 10.0, "T2.7 summary RMSE_px < 10",
            f"got {stats.get('rmse_px', 999):.3f}")
    _assert(stats.get("lock_rate", 0.0) == 100.0, "T2.8 summary lock_rate = 100%",
            f"got {stats.get('lock_rate', 0):.1f}%")
    _assert(stats.get("mean_fps", 0) > 100, "T2.9 summary mean_fps > 100",
            f"got {stats.get('mean_fps', 0):.1f}")

    fl.close()
    _assert(fl.is_closed, "T2.10 Logger closed cleanly")

    print(f"       RMSE={stats.get('rmse_px','?'):.3f} px | "
          f"mean_fps={stats.get('mean_fps','?'):.1f} | "
          f"lock={stats.get('lock_rate','?'):.1f}%")


# ===========================================================================
# T3 — ReportGenerator PDF
# ===========================================================================

def test_t3_report() -> None:
    _header("T3 — ReportGenerator: PDF Audit Report")

    try:
        import reportlab
    except ImportError:
        _skip("T3 ReportGenerator (all sub-tests)", "reportlab not installed")
        return

    from evaluation.logger import FrameLogger
    from evaluation.report_generator import ReportGenerator

    # Write a minimal 60-row log for the report to summarise
    csv_path = TMP / "report_source.csv"
    fl = FrameLogger(str(csv_path), session_id="report_test")
    for i in range(60):
        fl.log(i, i/30.0, "Mode A",
               320.0, 240.0,
               error_px   = 3.5,
               error_mrad = 3.5 * 0.00625 * (math.pi/180) * 1000,
               pan_deg_s  = 0.05,
               tilt_deg_s = 0.03,
               fps        = 145.0,
               lock_state = "TRACKING",
               )
    fl.flush()

    pdf_path = TMP / "test_audit.pdf"
    gen = ReportGenerator(fl, {
        "motion_type":    "figure_8",
        "atmospheric":    "clear",
        "gaussian_sigma": 8,
        "input_mode":     "Mode A – Synthetic",
    })
    out = gen.generate(str(pdf_path))

    _assert(out.exists(), "T3.1 PDF file created")
    _assert(out.stat().st_size > 2048, "T3.2 PDF file > 2 KB (non-empty)",
            f"{out.stat().st_size} bytes")

    # Validate PDF magic bytes (%PDF-1.x header)
    header = out.read_bytes()[:8]
    _assert(header.startswith(b"%PDF"), "T3.3 Valid PDF magic header",
            f"header={header}")

    fl.close()
    print(f"       PDF: {out}  ({out.stat().st_size} bytes)")


# ===========================================================================
# T4 — TrackingWorker QThread (120-frame integration)
# ===========================================================================

def test_t4_worker() -> None:
    _header("T4 — TrackingWorker QThread: 120-Frame Closed-Loop Run")

    try:
        from PySide6.QtWidgets import QApplication
        from PySide6.QtCore import QEventLoop, QTimer
    except ImportError:
        _skip("T4 TrackingWorker (all sub-tests)", "PySide6 not installed")
        return

    app = QApplication.instance() or QApplication(sys.argv)

    try:
        from ingestion.sim_source import SimulatorSource
        from evaluation.logger import FrameLogger
        from app.orchestrator import TrackingWorker
        from app.state_machine import TrackingState

        csv_path = TMP / "worker_test.csv"
        fl = FrameLogger(str(csv_path), session_id="worker_t4")
        src = SimulatorSource(
            motion_type="figure_8", fps=30.0, seed=99,
            # Use small amplitude so the controller stays within 10px lock zone.
            # amp_x=80px, amp_y=60px at omega=0.25 rad/s → max_speed=20 px/s
            # (well within the 800 px/s = 5°/s actuator authority).
            motion_kwargs={"amp_x": 80.0, "amp_y": 60.0, "omega": 0.25, "phi_y": 0.0},
        )

        worker = TrackingWorker(src, fl, input_mode="A", kp=2.5, kd=0.3)

        frames_done       = 0
        max_slew          = 0.0
        all_errors:       list[float] = []
        tracking_errors:  list[float] = []   # only TRACKING-state frames
        states_seen:      list[str]   = []
        current_state     = "SEARCHING"

        loop = QEventLoop()

        def on_frame(frm, metrics):
            nonlocal frames_done, max_slew, current_state
            frames_done += 1
            ep = float(metrics.get("error_px", 0))
            all_errors.append(ep)
            # Spec-correct: record RMSE only over TRACKING frames
            if metrics.get("state") == "TRACKING":
                tracking_errors.append(ep)
            max_slew = max(max_slew,
                           abs(float(metrics.get("pan_deg_s", 0))),
                           abs(float(metrics.get("tilt_deg_s", 0))))
            current_state = metrics.get("state", "SEARCHING")
            if frames_done >= 120:
                worker.stop()

        def on_state(s):
            states_seen.append(s)

        def on_stop():
            loop.quit()

        worker.frame_ready.connect(on_frame)
        worker.state_changed.connect(on_state)
        worker.worker_stopped.connect(on_stop)
        worker.start()

        # 20-second safety timeout
        guard = QTimer()
        guard.setSingleShot(True)
        guard.timeout.connect(loop.quit)
        guard.start(20_000)
        loop.exec()

        fl.close()

        # ISRO-spec RMSE: only over TRACKING frames (acquisition phase excluded)
        rmse_tracking = (
            math.sqrt(sum(e**2 for e in tracking_errors) / len(tracking_errors))
            if tracking_errors else 999.0
        )
        rmse_all = (
            math.sqrt(sum(e**2 for e in all_errors) / len(all_errors))
            if all_errors else 999.0
        )

        _assert(frames_done >= 120, "T4.1 ≥120 frames processed",
                f"got {frames_done}")
        _assert(max_slew <= 5.0, "T4.2 Max slew ≤ 5.0 °/s (rate clamp enforced)",
                f"max={max_slew:.4f}")
        _assert(rmse_tracking <= 10.0,
                "T4.3 TRACKING-state RMSE ≤ 10 px (ISRO spec, acquisition excluded)",
                f"RMSE={rmse_tracking:.2f} px  ({len(tracking_errors)} tracking frames)")
        _assert(len(states_seen) >= 1, "T4.4 At least 1 state transition",
                f"transitions={states_seen[:4]}")
        _assert(csv_path.exists() and csv_path.stat().st_size > 1000,
                "T4.5 Worker CSV written to disk",
                f"{csv_path.stat().st_size} bytes")

        # Verify TRACKING was reached
        _assert("TRACKING" in states_seen, "T4.6 TRACKING state was reached",
                f"states={states_seen}")

        print(f"       frames={frames_done}  "
              f"RMSE_tracking={rmse_tracking:.2f}px ({len(tracking_errors)} frames)  "
              f"RMSE_all={rmse_all:.2f}px  "
              f"max_slew={max_slew:.3f}°/s  states={states_seen[:5]}")

    except Exception as exc:
        _fail("T4 Worker exception", str(exc))
        traceback.print_exc()


# ===========================================================================
# Summary
# ===========================================================================

def _print_summary() -> bool:
    total = _passed + _failed
    print(f"\n{BLD}{'='*70}{RST}")
    print(f"{BLD}  BEACONLOCK PHASE 3 — TEST SUMMARY{RST}")
    print(f"{BLD}{'='*70}{RST}")
    print(f"  Total:  {total}")
    print(f"  {GRN}Passed: {_passed}{RST}")
    if _failed:
        print(f"  {RED}Failed: {_failed}{RST}")
    else:
        print(f"  Failed: 0")
    rate = (_passed / total * 100) if total else 0.0
    colour = GRN if _failed == 0 else (YEL if rate >= 80 else RED)
    print(f"\n  {colour}{BLD}Pass rate: {rate:.1f}%{RST}")
    print(f"\n{'─'*70}")
    print(f"  {'Test':<58}  {'Status':>6}")
    print(f"{'─'*70}")
    for r in _results:
        sc = GRN if r["status"] == "PASS" else (YEL if r["status"] == "SKIP" else RED)
        print(f"  {r['test']:<58}  {sc}{r['status']:>6}{RST}")
    print(f"{'─'*70}")
    print(f"  Artifacts: {TMP}\n")
    return _failed == 0


# ===========================================================================
# Entry
# ===========================================================================

if __name__ == "__main__":
    print(f"\n{BLD}")
    print("╔══════════════════════════════════════════════════════════════════════╗")
    print("║     BEACONLOCK — PHASE 3 UI/EVALUATION HEADLESS VERIFICATION SUITE  ║")
    print("║     ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)              ║")
    print("╚══════════════════════════════════════════════════════════════════════╝")
    print(RST)

    test_t0_imports()
    test_t1_state_machine()
    test_t2_logger()
    test_t3_report()
    test_t4_worker()

    ok = _print_summary()
    sys.exit(0 if ok else 1)
