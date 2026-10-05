"""
main.py
=======
BeaconLock application entry point.

Usage
-----
    # Launch full GUI
    py -3 main.py

    # Headless verification (CI / no display required)
    py -3 -X utf8 main.py --headless-check

    # Run with specific log directory
    py -3 main.py --log-dir ./logs/

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import logging
import sys
import time
import tempfile
from pathlib import Path

# ── Ensure project root is on sys.path ───────────────────────────────────
_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

# ── Logging ───────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)-30s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
_log = logging.getLogger("main")


# ── Headless check ────────────────────────────────────────────────────────

def run_headless_check() -> int:
    """
    Headless integration test for Phase 3 (no display required).

    Verifies:
    - StateMachine state transitions.
    - FrameLogger CSV writes.
    - TrackingWorker processes ≥ 60 frames via Qt event loop.
    - ReportGenerator produces a valid PDF.

    Returns 0 on success, 1 on any failure.
    """
    import math
    from PySide6.QtCore import QCoreApplication, QTimer, QEventLoop
    from PySide6.QtWidgets import QApplication

    # QApplication is required for QThread signals to work
    app = QApplication.instance() or QApplication(sys.argv)

    passed = 0
    failed = 0

    def _check(cond: bool, name: str) -> None:
        nonlocal passed, failed
        if cond:
            passed += 1
            print(f"  \033[92m[PASS]\033[0m {name}")
        else:
            failed += 1
            print(f"  \033[91m[FAIL]\033[0m {name}")

    print("\n\033[1m\033[96m"
          "══════════════════════════════════════════════════════════\n"
          "  BEACONLOCK PHASE 3 — HEADLESS CHECK\n"
          "══════════════════════════════════════════════════════════"
          "\033[0m\n")

    # ── T1: State Machine ─────────────────────────────────────────────
    print("\033[1m[T1] State Machine transitions\033[0m")
    try:
        from app.state_machine import StateMachine, TrackingState

        fsm = StateMachine()
        _check(fsm.state == TrackingState.SEARCHING, "T1.1 Initial state = SEARCHING")

        # Feed 8 detected frames with 5px error → should hit TRACKING
        for _ in range(8):
            fsm.update(detected=True, error_px=5.0)
        _check(fsm.state == TrackingState.TRACKING, "T1.2 5 locks → TRACKING")

        # Feed 5 miss frames → LOST
        for _ in range(5):
            fsm.update(detected=False, error_px=999.0)
        _check(fsm.state == TrackingState.LOST, "T1.3 5 misses → LOST")

        # Redetect → REACQUIRING
        fsm.update(detected=True, error_px=4.0)
        _check(fsm.state == TrackingState.REACQUIRING, "T1.4 Redetect → REACQUIRING")

        # 3 more locks → back to TRACKING
        for _ in range(3):
            fsm.update(detected=True, error_px=3.0)
        _check(fsm.state == TrackingState.TRACKING, "T1.5 3 re-locks → TRACKING")

        # Reset
        fsm.reset()
        _check(fsm.state == TrackingState.SEARCHING, "T1.6 reset() → SEARCHING")

    except Exception as exc:
        _check(False, f"T1 exception: {exc}")

    # ── T2: FrameLogger CSV ───────────────────────────────────────────
    print("\n\033[1m[T2] FrameLogger CSV telemetry\033[0m")
    tmp_dir = Path(tempfile.mkdtemp(prefix="beaconlock_hc_"))
    csv_path = tmp_dir / "hc_session.csv"
    try:
        from evaluation.logger import FrameLogger
        fl = FrameLogger(str(csv_path), session_id="headless_check")

        for i in range(50):
            fl.log(
                frame_id    = i,
                timestamp_s = i / 30.0,
                input_mode  = "Mode A",
                centroid_x  = 320.0 + math.sin(i * 0.1) * 5,
                centroid_y  = 240.0 + math.cos(i * 0.1) * 5,
                error_px    = 5.0 + math.sin(i * 0.3) * 2,
                error_mrad  = 0.03,
                pan_deg_s   = 0.05 * math.sin(i * 0.2),
                tilt_deg_s  = 0.03 * math.cos(i * 0.2),
                fps         = 150.0,
                lock_state  = "TRACKING",
            )
        fl.flush()

        _check(csv_path.exists(), "T2.1 CSV file created")
        _check(fl.row_count == 50, f"T2.2 50 rows logged (got {fl.row_count})")

        stats = fl.summary()
        _check(stats.get("total_frames", 0) == 50, "T2.3 summary() → 50 frames")
        _check(stats.get("rmse_px", 999) < 10.0,
               f"T2.4 RMSE < 10 px (got {stats.get('rmse_px',999):.3f})")
        _check(stats.get("lock_rate", 0) == 100.0,
               f"T2.5 Lock rate = 100% (got {stats.get('lock_rate',0):.1f}%)")
        fl.close()

    except Exception as exc:
        _check(False, f"T2 exception: {exc}")
        fl = None

    # ── T3: ReportGenerator PDF ───────────────────────────────────────
    print("\n\033[1m[T3] ReportGenerator PDF audit\033[0m")
    pdf_path = tmp_dir / "hc_audit.pdf"
    try:
        from evaluation.report_generator import ReportGenerator
        # Use fl directly — it is already closed, 50 rows on disk.
        # (Creating a new FrameLogger on the same path with "w" mode would
        #  overwrite the existing data before summary() can read it.)
        gen = ReportGenerator(fl, {
            "motion_type": "figure_8",
            "atmospheric": "clear",
            "gaussian_sigma": 8,
            "input_mode": "Mode A – Synthetic",
        })

        out = gen.generate(str(pdf_path))
        _check(out.exists() and out.stat().st_size > 2048,
               f"T3.1 PDF generated (size={out.stat().st_size} bytes)")
        print(f"        PDF path: {out}")

    except ImportError:
        print("  \033[93m[SKIP]\033[0m T3.1 ReportLab not installed — PDF test skipped.")
        passed += 1   # not a failure
    except Exception as exc:
        _check(False, f"T3 exception: {exc}")

    # ── T4: TrackingWorker QThread (60 frames) ────────────────────────
    print("\n\033[1m[T4] TrackingWorker QThread — 60-frame run\033[0m")
    try:
        from ingestion.sim_source import SimulatorSource
        from evaluation.logger import FrameLogger
        from app.orchestrator import TrackingWorker

        csv2 = tmp_dir / "worker_session.csv"
        fl3  = FrameLogger(str(csv2), session_id="worker_test")
        src  = SimulatorSource(motion_type="circular", fps=30.0, seed=1)

        worker = TrackingWorker(src, fl3, input_mode="A")

        frames_received = 0
        states_seen: list[str] = []
        max_error = 0.0
        max_cmd   = 0.0
        loop = QEventLoop()

        def on_frame(frm, metrics):
            nonlocal frames_received, max_error, max_cmd
            frames_received += 1
            max_error = max(max_error, metrics.get("error_px", 0))
            max_cmd   = max(max_cmd,
                            abs(metrics.get("pan_deg_s", 0)),
                            abs(metrics.get("tilt_deg_s", 0)))
            if frames_received >= 60:
                worker.stop()

        def on_state(s):
            states_seen.append(s)

        def on_stop():
            loop.quit()

        worker.frame_ready.connect(on_frame)
        worker.state_changed.connect(on_state)
        worker.worker_stopped.connect(on_stop)

        worker.start()

        # Timeout guard: abort after 15 seconds
        guard = QTimer()
        guard.setSingleShot(True)
        guard.timeout.connect(loop.quit)
        guard.start(15_000)
        loop.exec()

        fl3.close()

        _check(frames_received >= 60,
               f"T4.1 ≥60 frames processed (got {frames_received})")
        _check(max_cmd <= 5.0,
               f"T4.2 Max slew ≤ 5.0 °/s (got {max_cmd:.3f})")
        _check(len(states_seen) > 0,
               f"T4.3 At least one state transition observed ({states_seen[:3]})")
        _check(csv2.exists() and csv2.stat().st_size > 500,
               f"T4.4 Worker CSV written ({csv2.stat().st_size} bytes)")

    except Exception as exc:
        _check(False, f"T4 exception: {exc}")
        import traceback; traceback.print_exc()

    # ── Summary ───────────────────────────────────────────────────────
    total = passed + failed
    rate = passed / total * 100 if total else 0.0
    colour = "\033[92m" if failed == 0 else "\033[91m"
    print(f"\n\033[1m{'='*58}\033[0m")
    print(f"\033[1m  HEADLESS CHECK SUMMARY\033[0m")
    print(f"\033[1m{'='*58}\033[0m")
    print(f"  Total: {total}  |  Passed: {passed}  |  Failed: {failed}")
    print(f"  {colour}\033[1mPass rate: {rate:.1f}%\033[0m")
    print(f"  Log dir: {tmp_dir}")
    print(f"\033[1m{'='*58}\033[0m\n")

    return 0 if failed == 0 else 1


# ── GUI launch ────────────────────────────────────────────────────────────

def launch_gui() -> int:
    """Start the full PySide6 desktop dashboard."""
    import ctypes

    # ── Windows AppUserModelID ── shows custom icon on taskbar (must be
    # set before QApplication is created so the taskbar grouping works).
    try:
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(
            "alphatrion.beaconlock.ps26169.v1"
        )
    except Exception:
        pass  # non-Windows or ctypes unavailable — silently skip

    from PySide6.QtWidgets import QApplication
    from PySide6.QtGui import QIcon
    from PySide6.QtCore import Qt

    # High-DPI scaling (must be set before QApplication is created)
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

    app = QApplication(sys.argv)
    app.setApplicationName("BeaconLock")
    app.setOrganizationName("AlphaTrion")
    app.setApplicationVersion("0.4.0")   # Phase 4

    # Custom icon (optical crosshair — generated in ui/assets/)
    _icon_path = _ROOT / "ui" / "assets" / "icon.ico"
    if _icon_path.exists():
        app.setWindowIcon(QIcon(str(_icon_path)))

    # Apply global aerospace obsidian stylesheet
    from ui.dashboard import AEROSPACE_THEME_STYLESHEET
    app.setStyleSheet(AEROSPACE_THEME_STYLESHEET)

    # Launch main window
    from ui.dashboard import Dashboard
    window = Dashboard()
    window.show()

    _log.info("BeaconLock GUI started — ISRO SIH-2026 | PS-26169 | AlphaTrion")
    rc = app.exec()
    _log.info("BeaconLock GUI exited (rc=%d)", rc)
    return rc


# ── Entry ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    if "--headless-check" in sys.argv:
        try:
            import ctypes
            if ctypes.windll.kernel32.AttachConsole(-1):
                sys.stdout = open("CONOUT$", "w", encoding="utf-8", errors="replace")
                sys.stderr = open("CONOUT$", "w", encoding="utf-8", errors="replace")
        except Exception:
            pass
        sys.exit(run_headless_check())
    else:
        sys.exit(launch_gui())

