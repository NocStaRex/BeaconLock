"""
evaluation/logger.py
=====================
Per-frame CSV telemetry logger for BeaconLock.

Streams one row per processed frame to a CSV file.  Writes are buffered
in a deque and flushed in batches so the hot tracking loop is never
blocked by disk I/O.  Thread-safe: can be called from the QThread worker.

CSV Schema (ISRO PS-26169 §11 log spec):
    frame_id, timestamp_s, input_mode, centroid_x, centroid_y,
    error_px, error_mrad, pan_deg_s, tilt_deg_s, fps, lock_state

ISRO SIH-2026 | PS-26169 | AlphaTrion (ID: 176697)
"""

from __future__ import annotations

import csv
import io
import logging
import threading
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

# CSV column names (canonical order)
COLUMNS = [
    "frame_id",
    "timestamp_s",
    "input_mode",
    "centroid_x",
    "centroid_y",
    "error_px",
    "error_mrad",
    "pan_deg_s",
    "tilt_deg_s",
    "fps",
    "lock_state",
]

# Flush buffer to disk every N rows (keeps I/O overhead low)
FLUSH_INTERVAL: int = 30


class FrameLogger:
    """
    Thread-safe per-frame CSV telemetry logger.

    Parameters
    ----------
    output_path : str | Path
        Destination CSV file path.  Parent directory is created if missing.
    session_id : str
        Human-readable session identifier embedded in the file header.
    buffer_size : int
        Number of rows to buffer before flushing to disk.
    """

    def __init__(
        self,
        output_path: str | Path,
        session_id: str = "",
        buffer_size: int = FLUSH_INTERVAL,
    ) -> None:
        self._path = Path(output_path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._session_id = session_id
        self._buffer_size = buffer_size

        self._lock = threading.Lock()
        self._buffer: deque[dict] = deque()
        self._row_count = 0
        self._closed = False
        self._t0: float = datetime.now().timestamp()

        # Open file and write header
        self._file = open(self._path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=COLUMNS, extrasaction="ignore")

        # Session metadata header comment
        self._file.write(
            f"# BeaconLock telemetry log\n"
            f"# session_id={session_id}\n"
            f"# started={datetime.now().isoformat()}\n"
            f"# columns={','.join(COLUMNS)}\n"
        )
        self._writer.writeheader()
        self._file.flush()
        logger.info("FrameLogger opened: %s", self._path)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def log(
        self,
        frame_id: int,
        timestamp_s: float,
        input_mode: str,
        centroid_x: float,
        centroid_y: float,
        error_px: float,
        error_mrad: float,
        pan_deg_s: float,
        tilt_deg_s: float,
        fps: float,
        lock_state: str,
    ) -> None:
        """
        Buffer one telemetry row.  Non-blocking unless buffer flush is due.

        Parameters match the ISRO PS-26169 §11 CSV schema exactly.
        """
        if self._closed:
            return

        row = {
            "frame_id":    frame_id,
            "timestamp_s": f"{timestamp_s:.4f}",
            "input_mode":  input_mode,
            "centroid_x":  f"{centroid_x:.2f}",
            "centroid_y":  f"{centroid_y:.2f}",
            "error_px":    f"{error_px:.3f}",
            "error_mrad":  f"{error_mrad:.4f}",
            "pan_deg_s":   f"{pan_deg_s:.4f}",
            "tilt_deg_s":  f"{tilt_deg_s:.4f}",
            "fps":         f"{fps:.1f}",
            "lock_state":  lock_state,
        }

        with self._lock:
            self._buffer.append(row)
            self._row_count += 1
            if len(self._buffer) >= self._buffer_size:
                self._flush_locked()

    def flush(self) -> None:
        """Force-flush the internal buffer to disk."""
        with self._lock:
            self._flush_locked()

    def close(self) -> None:
        """Flush remaining rows and close the file."""
        with self._lock:
            if not self._closed:
                self._flush_locked()
                self._file.close()
                self._closed = True
                logger.info("FrameLogger closed: %d rows → %s", self._row_count, self._path)

    def summary(self) -> dict:
        """
        Compute summary statistics from the on-disk CSV.

        Returns a dict with: total_frames, duration_s, rmse_px, rmse_mrad,
        mean_fps, max_slew_deg_s, lock_rate.
        """
        import math
        if not self._path.exists() or self._path.stat().st_size < 100:
            return {}

        # Flush before reading (skip if already closed — buffer is empty after close())
        if not self._closed:
            self.flush()

        errors_px: list[float] = []
        errors_mrad: list[float] = []
        fps_vals: list[float] = []
        slew_vals: list[float] = []
        lock_frames: int = 0
        total_rows: int = 0
        t_start: Optional[float] = None
        t_end: Optional[float] = None

        with open(self._path, "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("#"):
                    continue
            # Re-read properly
            f.seek(0)
            rows = [r for r in f if not r.startswith("#")]

        # Parse via csv.DictReader from the non-comment rows
        reader = csv.DictReader(io.StringIO("".join(rows)))
        for row in reader:
            try:
                ts = float(row["timestamp_s"])
                if t_start is None:
                    t_start = ts
                t_end = ts

                ep = float(row["error_px"])
                em = float(row["error_mrad"])
                errors_px.append(ep)
                errors_mrad.append(em)

                fps_vals.append(float(row["fps"]))
                slew = max(abs(float(row["pan_deg_s"])), abs(float(row["tilt_deg_s"])))
                slew_vals.append(slew)

                if row["lock_state"] in ("TRACKING", "REACQUIRING"):
                    lock_frames += 1
                total_rows += 1
            except (KeyError, ValueError):
                continue

        if total_rows == 0:
            return {}

        rmse_px   = math.sqrt(sum(e**2 for e in errors_px) / len(errors_px)) if errors_px else 0.0
        rmse_mrad = math.sqrt(sum(e**2 for e in errors_mrad) / len(errors_mrad)) if errors_mrad else 0.0
        duration  = (t_end - t_start) if (t_start is not None and t_end is not None) else 0.0

        return {
            "total_frames":    total_rows,
            "duration_s":      round(duration, 2),
            "rmse_px":         round(rmse_px, 3),
            "rmse_mrad":       round(rmse_mrad, 4),
            "mean_fps":        round(sum(fps_vals) / len(fps_vals), 1) if fps_vals else 0.0,
            "max_slew_deg_s":  round(max(slew_vals), 3) if slew_vals else 0.0,
            "lock_rate":       round(lock_frames / total_rows * 100, 1),
            "lock_frames":     lock_frames,
            "session_id":      self._session_id,
            "csv_path":        str(self._path),
        }

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def path(self) -> Path:
        return self._path

    @property
    def row_count(self) -> int:
        return self._row_count

    @property
    def is_closed(self) -> bool:
        return self._closed

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _flush_locked(self) -> None:
        """Write buffered rows to disk (must be called with self._lock held)."""
        while self._buffer:
            self._writer.writerow(self._buffer.popleft())
        self._file.flush()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
