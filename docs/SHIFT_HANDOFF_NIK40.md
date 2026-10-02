# BeaconLock — Engineering Shift Handoff
## Document ID: SHIFT_HANDOFF_NIK40
## Classification: INTERNAL TECHNICAL REFERENCE — SINGLE SOURCE OF TRUTH

---

```
╔══════════════════════════════════════════════════════════════════════════════╗
║  SYSTEM      : BeaconLock — Virtual Camera PAT Simulator (FSOC Digital Twin)║
║  CHALLENGE   : Smart India Hackathon (SIH) 2026 | Problem Statement PS-26169 ║
║  ORGANISATION: Department of Space / ISRO                                    ║
║  TEAM        : AlphaTrion — ID 176697                                        ║
║  SHIFT LEAD  : nik40                                                          ║
║  DOC CREATED : 2026-10-02T20:22:46+05:30                                     ║
║  STATUS      : Phases 1, 2, 3 — 100% COMPLETE & VERIFIED (Zero regressions)  ║
╚══════════════════════════════════════════════════════════════════════════════╝
```

> [!IMPORTANT]
> **Incoming engineer/agent: Read ONLY this document before touching any code.**
> All architectural decisions in Phases 1–3 are locked. Do not alter them unless
> a specific regression is observed. Refer to §5 for run commands and §6 for your
> exact Phase 4 deliverables.

---

## TABLE OF CONTENTS

1. [Header & Metadata](#1-header--metadata)
2. [Executive Kinematics & Benchmark Registry](#2-executive-kinematics--benchmark-registry)
3. [Architectural Map & Decoupling Rules](#3-architectural-map--decoupling-rules)
4. [Critical Bug Fixes & Lessons Learned](#4-critical-bug-fixes--lessons-learned-locked)
5. [Verification & Run Commands](#5-verification--run-commands)
6. [Shift 2 Actionable Roadmap](#6-shift-2-actionable-roadmap)
7. [Shift 2 Log (Incoming Engineer Fills This)](#7-shift-2-log-incoming-engineer-fills-this)

---

## 1. Header & Metadata

| Field | Value |
|---|---|
| **System** | BeaconLock — Virtual Camera PAT Simulator for Free Space Optical Communications |
| **Challenge** | Smart India Hackathon (SIH) 2026 |
| **Problem Statement ID** | PS-26169 |
| **Issuing Organisation** | Department of Space / Indian Space Research Organisation (ISRO) |
| **Team Name** | AlphaTrion |
| **Team ID** | 176697 |
| **Shift Author** | nik40 |
| **Handoff Timestamp** | 2026-10-02T20:22:46+05:30 |
| **Repository Root** | `d:\CODING_NOC\BeaconLock\` |
| **Python Interpreter** | `py -3` → Python 3.13.x (64-bit) at `C:\Users\OX_Ni\AppData\Local\Programs\Python\Python313\python.exe` |
| **Phase 1 Status** | ✅ COMPLETE — 31/31 PASS |
| **Phase 2 Status** | ✅ COMPLETE — 46/46 PASS |
| **Phase 3 Status** | ✅ COMPLETE — 33/33 PASS + HC 16/16 PASS |
| **Total Green Assertions** | **126+ across all three phases** |
| **Regressions** | **0** |

### 1.1 Master Blueprint Reference

The canonical system specification lives at:

```
docs/MASTER_BLUEPRINT.md
```

Every non-negotiable ISRO constraint, mathematical model, and competitive evaluation
strategy is recorded there. **It supersedes this document for spec questions.** This
document supplements it with *implementation reality* — what was actually built,
measured, and fixed.

---

## 2. Executive Kinematics & Benchmark Registry

All numbers below are **measured ground truth** from CI test runs on a standard
Windows CPU (Intel/AMD, no GPU).

### 2.1 Throughput (CPU-Only, Zero GPU)

| Stage | Measured FPS | Latency/Frame | ISRO Min |
|---|---|---|---|
| Pure Math Core (Centroid + Kalman + PD) | **2,370 – 4,345 FPS** | 0.23 – 0.42 ms | ≥ 30 FPS |
| Closed-Loop Sim + Noise + Perception (Phase 2) | **~159 FPS** | ~6.3 ms | ≥ 30 FPS |
| QThread Worker + Overlay Render (Phase 3) | **~140 FPS** (mean) | ~7.1 ms | ≥ 30 FPS |

> [!NOTE]
> All FPS numbers are measured without any GUI paint calls. The Qt render thread
> adds real-world overhead but never drops below 30 FPS in testing.

### 2.2 Tracking Accuracy

| Scenario | RMSE | ISRO Ceiling | Note |
|---|---|---|---|
| Phase 1 — Centroid, clean frame | **0.707 px** | — | Theoretical floor (√½ px) |
| Phase 1 — Centroid, σ=15 Gaussian, 64×64 ROI | **0.755 px** | — | Sub-pixel across noise |
| Phase 2 — Closed-loop Figure-8 (GT comparison) | **2.51 px** | ≤ 10 px ✓ | Full noise + jitter |
| Phase 3 — QThread Worker TRACKING-state RMSE | **8.31 px** | ≤ 10 px ✓ | 60 TRACKING frames |

### 2.3 Actuator Slew Performance

| Scenario | Max Slew Observed | ISRO Ceiling | Clamp Violations |
|---|---|---|---|
| Phase 2 closed-loop (300 frames) | **0.995 °/s** | ≤ 5.0 °/s | **0** |
| Phase 3 QThread worker (120 frames) | **0.190 °/s** | ≤ 5.0 °/s | **0** |

### 2.4 FSM State Cycle (Phase 3 T4 Verified)

```
SEARCHING → ACQUIRING → TRACKING → LOST → REACQUIRING → TRACKING
```

All five states reached, all transitions validated. Exactly matches ISRO PS-26169
acquisition latency specification (≤ 2.0 s) and re-acquisition specification (≤ 1.0 s).

### 2.5 PDF Audit Report

- Generated size: **~4,090 bytes** for 50-row session (valid ReportLab PDF 1.4)
- Magic header: `%PDF-1.4` confirmed
- Sections: (1) Session Summary, (2) ISRO Benchmark Matrix with PASS/FAIL, (3) Failure Boundary Documentation

---

## 3. Architectural Map & Decoupling Rules

### 3.1 Repository Layout

```
BeaconLock/
├── app/
│   ├── __init__.py
│   ├── state_machine.py        ← 5-state FSM (SEARCHING/ACQUIRING/TRACKING/LOST/REACQUIRING)
│   └── orchestrator.py         ← TrackingWorker(QThread) + _render_overlay()
│
├── control/
│   ├── __init__.py
│   └── pid_controller.py       ← Rate-limited PD, 0.05° deadband, ±5.0°/s clamp
│
├── evaluation/
│   ├── __init__.py
│   ├── logger.py               ← Thread-safe 11-col CSV logger (ring-buffer flush)
│   └── report_generator.py     ← ReportLab 3-section PDF audit engine
│
├── ingestion/                  ← *** THE ONLY BRIDGE TO simulation/ ***
│   ├── __init__.py
│   ├── frame_source.py         ← Abstract FrameSource ABC (Mode A + Mode B interface)
│   └── sim_source.py           ← SimulatorSource (Mode A concrete impl)
│
├── perception/
│   ├── __init__.py
│   ├── centroid.py             ← Radiometric CoG (border-annulus BG, 64×64 ROI)
│   └── kalman_tracker.py       ← 6-state KF [x,y,vx,vy,ax,ay]ᵀ with adaptive R
│
├── simulation/
│   ├── __init__.py
│   ├── camera.py               ← VirtualCamera (640×480 viewport, rate-clamped PTZ)
│   ├── disturbances.py         ← Gaussian/S&P/Poisson noise, Jitter, Platform, Atmos
│   ├── motion_models.py        ← StraightLine, Circular, Figure8, RandomWalk models
│   ├── scene.py                ← WorldScene (2000×2000 canvas renderer)
│   └── target.py               ← BeaconTarget (5–20 px square, occlusion injection)
│
├── ui/
│   ├── __init__.py
│   ├── dashboard.py            ← PySide6 QMainWindow cockpit (#0f172a dark theme)
│   └── plots_panel.py          ← PyQtGraph 3-plot real-time telemetry panel
│
├── tests/
│   ├── test_phase1_headless.py ← 31 assertions: CoG, Kalman, PD math
│   ├── test_phase2_sim.py      ← 46 assertions: sim engine, all 4 motions, disturbances
│   └── test_phase3_ui.py       ← 33 assertions: FSM, CSV, PDF, QThread worker
│
├── docs/
│   ├── MASTER_BLUEPRINT.md     ← Canonical ISRO spec + algo decisions (READ FIRST)
│   └── SHIFT_HANDOFF_NIK40.md  ← THIS FILE
│
├── main.py                     ← Entry point: `launch_gui()` + `run_headless_check()`
└── requirements.txt            ← Pinned Python 3.13-compatible deps
```

### 3.2 Module Roles — Detailed

#### `perception/centroid.py` — `CentroidEstimator`

- **Algorithm**: Radiometric background estimation using border-annulus pixels.
  - Background threshold: $I_{th} = \mu_{bg} + 3\sigma_{bg}$
  - Intensity-weighted Centre of Gravity (CoG):
    $$x_{cog} = \frac{\sum_i (I_i - I_{th}) \cdot x_i}{\sum_i (I_i - I_{th})}$$
- **ROI Mode**: 64×64 px sub-window around last known position (used once Kalman is locked).
- **Full-frame Mode**: Searches entire 640×480 viewport (used during SEARCHING/first detection).
- **Output**: `CoGResult(cx, cy, detected, confidence)` — sub-pixel float coordinates.
- **Imports**: `numpy`, `cv2` only. **Zero simulation imports.**

#### `perception/kalman_tracker.py` — `KalmanTracker`

- **Library**: `filterpy.kalman.KalmanFilter`
- **State vector**: $\mathbf{x} = [x, y, v_x, v_y, a_x, a_y]^T$ — 6-state constant-acceleration model.
- **Measurement**: $(x, y)$ pixel position from centroid, 2D.
- **Adaptive R**: Measurement noise covariance $R$ scales inversely with detection confidence: $R = R_{base} / \max(\text{confidence}, 0.01)$
- **Coasting**: When `detected=False`, propagates forward with prediction only (no measurement update). Kalman coasting covers LOST-state Kalman tracking.
- **Output**: `KalmanResult(predicted_x, predicted_y, velocity_x, velocity_y, covariance_norm)`.
- **Imports**: `filterpy`, `numpy`. **Zero simulation imports.**

#### `control/pid_controller.py` — `PDController`

- **Type**: Proportional-Derivative (no integrator — anti-windup not needed at these scales).
- **Gains**: `Kp = 2.5`, `Kd = 0.3` (default, tunable via constructor).
- **Scale**: `DEG_PER_PX = 0.00625 deg/px` (ISRO-locked: 4° FOV / 640 px).
- **Deadband**: `0.05°` (≈ 8 px) inside orchestrator. This is tuned BELOW the 10 px ISRO lock threshold to allow the controller to converge into the lock zone.

  > [!WARNING]
  > The Phase 1 test suite uses `deadband=0.2°` (32 px) as the **default** value for
  > unit-test isolation. The orchestrator **overrides this to 0.05°** in
  > `_init_pipeline()`. Do NOT change the orchestrator's deadband without re-running
  > the Phase 3 T4 worker test — convergence depends on it.

- **Saturation**: Output rate clamped to `±5.0 °/s` (hard ISRO ceiling).
- **Output**: `PDCommand(pan_rate_deg_s, tilt_rate_deg_s)`.
- **Imports**: `numpy`, `math`. **Zero simulation imports.**

#### `simulation/scene.py` — `WorldScene`

- **Canvas**: 2000 × 2000 pixels (≥ ISRO-specified 2000×2000 world coordinate space).
- Renders all `BeaconTarget` objects onto the canvas each frame.
- Returns raw `ndarray` (uint8, grayscale).

#### `simulation/target.py` — `BeaconTarget`

- **Beacon Spot**: Default 10×10 px square, configurable 5–20 px, brightness 255 (saturated white).
- Delegates position update to a `MotionModel` each frame.
- **Occlusion Injection**: `inject_occlusion(n_frames)` blanks the beacon for exactly `n_frames` frames to test Kalman coasting. Counter lives in `render()` (not `update()`) to guarantee exactly N dark frames.

#### `simulation/motion_models.py` — All 4 ISRO-Required Motion Models

| Class | Key Parameters | Notes |
|---|---|---|
| `StraightLineMotion` | `vx`, `vy`, `reflect` | Boundary reflection (default) or wrap |
| `CircularMotion` | `center_x/y`, `radius` (200–600 px), `angular_velocity_deg_s` | Starts at `center + (radius, 0)` |
| `Figure8Motion` | `amp_x`, `amp_y`, `omega`, `phi_y` | `phi_y=0.0` default → starts at canvas centre |
| `RandomWalkMotion` | `max_speed`, `bounds_x/y`, `seed` | Ornstein–Uhlenbeck smooth bounded walk |

> [!CAUTION]
> `Figure8Motion` defaults: `amp_x=350.0`, `amp_y=175.0`, `omega=0.4 rad/s`.
> These are large amplitudes. At full amplitude, the beacon moves at ~140 px/s,
> which requires the controller to actively chase it. For test scenarios where
> immediate TRACKING lock is needed, pass `motion_kwargs={"amp_x": 80.0, "amp_y": 60.0,
> "omega": 0.25, "phi_y": 0.0}` — as done in Phase 3 T4.

#### `simulation/disturbances.py` — `DisturbanceEngine`

Applies disturbances **to the rendered viewport frame** (not the world canvas). This is architecturally correct — it models sensor-plane noise, not world-space noise.

| Disturbance | Model | ISRO Param |
|---|---|---|
| Gaussian Noise | `np.random.normal(0, σ)` added per pixel | σ up to 20 |
| Salt & Pepper | Random pixel replacement to 0 or 255 | ~10% corrupted |
| Poisson Noise | `np.random.poisson(frame_px)` | Natural photon noise |
| Camera Jitter | Frame-to-frame viewport offset `±jitter_px` | ≤ ±20 px/frame |
| Platform Drift | Low-frequency camera displacement (Linear/Circular modes) | ±20 px/frame max |
| Atmospheric | `clear`, `haze`, `fog`, `rain`, `low_light` presets | Koschmieder law for fog |

**Fog formula (Koschmieder)**: $I_{out} = 0.50 \cdot I_{in} + 55$ — calibrated so
$I(180) = 145 < 175$ (threshold). Earlier buggy formula `0.60·I + 80` could give
$I(180) = 188 > 175$ (false detection). **Do not revert this coefficient.**

#### `ingestion/frame_source.py` — `FrameSource` (Abstract Base Class)

```python
class FrameSource(ABC):
    @abstractmethod
    def read(self) -> tuple[bool, np.ndarray]: ...
    @abstractmethod
    def get_fps(self) -> float: ...
    @abstractmethod
    def get_resolution(self) -> tuple[int, int]: ...
```

This is the **Mode A / Mode B interface contract**. Everything above `ingestion/`
in the import stack talks only to `FrameSource` — never to `simulation/` directly.

**Mode A** → `SimulatorSource(FrameSource)` in `sim_source.py`  
**Mode B** → `VideoFileSource(FrameSource)` in `frame_source.py` (stub — to be fully
implemented in Phase 4 for 30-FPS MP4 evaluator ingestion)

#### `ingestion/sim_source.py` — `SimulatorSource`

**This is the ONLY file that imports from `simulation/`.** It bundles:

1. `WorldScene` rendering (2000×2000 canvas → grayscale ndarray)
2. `BeaconTarget` position update (`target.update(dt)`)
3. `VirtualCamera` viewport crop (`camera.get_viewport(world_frame, cx, cy)`)
4. `DisturbanceEngine` application
5. Camera control integration (`apply_camera_control(pan, tilt, dt)`)

Exposes `read() → (bool, ndarray)` returning a clean 640×480 uint8 frame to the
tracking pipeline. Also exposes `ground_truth_camera` property (returns `(cx, cy)`
in frame-pixel coordinates — used only by tests for GT comparison, never by the
tracking core).

#### `evaluation/logger.py` — `FrameLogger`

- **Schema** (11 columns): `frame_id, timestamp_s, input_mode, centroid_x, centroid_y, error_px, error_mrad, pan_deg_s, tilt_deg_s, fps, lock_state`
- **Thread-safe**: `threading.Lock` guards all writes.
- **Buffer**: `deque` ring-buffer, flushes to disk every 30 frames (configurable).
- **`summary()`**: Reads full CSV from disk, computes RMSE (px & mrad), lock_rate, mean_fps, max_slew, session_duration. Guards: file must exist **and** be > 100 bytes (not `row_count == 0` — that was the bug).

#### `evaluation/report_generator.py` — `ReportGenerator`

Three-section ReportLab PDF:

1. **Session Summary**: Tabular metadata (motion type, atmospheric mode, duration, total frames).
2. **ISRO Performance Benchmark Matrix**: Computes PASS/FAIL live from logger stats vs. ISRO thresholds.
   ```python
   ISRO_THRESHOLDS = {
       "rmse_px":    ("Tracking Error",         10.0,  "≤ 10 px"),
       "lock_rate":  ("Lock Retention Rate",    95.0,  "≥ 95%"),
       "mean_fps":   ("System Throughput",      30.0,  "≥ 30 FPS"),
       "max_slew":   ("Slew Rate Ceiling",       5.0,  "≤ 5.0 °/s"),
   }
   ```
3. **Honest Failure Boundary Documentation**: Hard-coded edge cases (extreme fog below CoG threshold, full-frame occlusion > coast timeout) — competitive compliance requirement.

#### `app/state_machine.py` — `StateMachine`

**5-state FSM** with the following transitions:

```
                   detected
SEARCHING ──────────────────────► ACQUIRING
                                      │
                       5 consecutive  │  ≥3 det-misses
                       locked frames  │  OR acq_timeout
                                      ▼
                                  TRACKING ◄──────────────────────┐
                                      │                            │
                             5 misses │                            │ 3 re-locks
                                      ▼                            │
                                    LOST ──── detected ──► REACQUIRING
                                      │
                          90-frame    │
                          coast TMO   │
                                      ▼
                                  SEARCHING
```

**Constants (locked):**

| Constant | Value | Purpose |
|---|---|---|
| `LOCK_THRESHOLD_PX` | 10.0 px | Error ≤ this = "locked" |
| `LOCK_CONSECUTIVE` | 5 frames | ACQUIRING → TRACKING |
| `RELOCK_CONSECUTIVE` | 3 frames | REACQUIRING → TRACKING |
| `MISS_TO_LOST` | 5 frames | TRACKING → LOST |
| `MISS_TO_SEARCH` | 3 frames | (detection-miss only — see §4.1) |
| `COAST_TIMEOUT` | 90 frames | LOST → SEARCHING (3 s @ 30 FPS) |

**Miss-counting rule (CRITICAL — see §4.1 for bug history):**
- In `TRACKING`: any frame where `not locked` (error > 10px OR not detected) increments `consecutive_misses`.
- In `ACQUIRING` / `REACQUIRING`: only frames where `not detected` (actual signal loss) increment `consecutive_misses`. Frames where beacon is **detected but error > 10px** do NOT count as misses — the controller is still converging.

#### `app/orchestrator.py` — `TrackingWorker(QThread)`

Per-frame loop (runs at native CPU speed, not rate-limited during test):

```
1.  source.read()                    → (ok, raw_frame)
2.  centroid.estimate(raw_frame)     → CoGResult
3.  kalman.step(cx, cy, conf)        → KalmanResult (predicted position)
4.  error_px = hypot(pred_x-320, pred_y-240)
5.  fsm.update(detected, error_px)   → FSMResult (state, lock status)
6.  pd_ctrl.update(err_x, err_y, dt) → PDCommand (pan/tilt rate)
7.  source.apply_camera_control(...)  (Mode A only — SimulatorSource)
8.  logger.log(frame_id, ...)
9.  _render_overlay(raw_frame, ...)  → display_frame (with crosshair, ellipse, bbox)
10. emit frame_ready(display_frame, metrics_dict)
11. emit state_changed(state_name)   (only on transition)
12. emit metrics_ready(metrics_dict)
13. rate-limit sleep if needed
```

**Signals** (all use `object` type for numpy array safety):
- `frame_ready(object, object)` → `(ndarray, dict)`
- `state_changed(str)` → new state name
- `metrics_ready(object)` → metrics `dict`
- `error_occurred(str)` → error message
- `worker_stopped()` → QThread finished

#### `ui/dashboard.py` — `Dashboard(QMainWindow)`

- **Theme**: `background: #0f172a` (slate-900), white text, accent `#38bdf8` (sky-400).
- **Layout**: 3-panel — Left (controls), Center (camera feed + mini-map), Right (PyQtGraph plots).
- **Mode Selector**: Toggles `SimulatorSource` (Mode A) vs `VideoFileSource` (Mode B).
- **Motion Selector**: Dropdown → Straight / Circular / Figure-8 / Random.
- **Disturbance Controls**: Sliders for noise sigma, jitter, S&P %, atmosphere preset.
- **Action Buttons**: START / PAUSE / RESET / Inject Occlusion / Export PDF.
- **State Badge**: Live SEARCHING / ACQUIRING / TRACKING / LOST / REACQUIRING badge.

#### `ui/plots_panel.py` — `PlotsPanel(QWidget)`

Three stacked `PlotWidget` (PyQtGraph, OpenGL-backed):
1. **Error Plot**: Tracking error (px) with persistent red threshold line at 10 px.
2. **Slew Plot**: Pan/Tilt rate (°/s) with ±5.0°/s red ceiling lines.
3. **FPS Plot**: System throughput with green 30 FPS floor line.

All plots use `deque(maxlen=300)` ring buffers (10 s at 30 FPS).

### 3.3 Decoupling Rules — HARD ENFORCEMENT

```
simulation/          ← imports only numpy, opencv, math (pure world model)
    │
    └── ingestion/sim_source.py   ← THE ONLY file importing simulation/
                                     (Mode A concrete impl)
ingestion/frame_source.py        ← Abstract ABC only (numpy)
    │
    └── perception/              ← imports ingestion.frame_source (ABC) + numpy/cv2
    └── control/                 ← imports numpy, math only
    └── app/orchestrator.py      ← imports perception/ + control/ + evaluation/ +
                                     ingestion/ (via FrameSource ABC)
    └── ui/                      ← imports app/ + evaluation/ only
```

**The tracking core (`perception/` + `control/`) has ZERO `simulation/` imports.**
This guarantees 100% compliance for Benchmark-2 evaluation (Mode B judge MP4 video
ingestion). If a judge supplies an external 640×480 30-FPS MP4, `VideoFileSource`
plugs into the identical pipeline with zero changes to perception or control.

---

## 4. Critical Bug Fixes & Lessons Learned (LOCKED)

> [!WARNING]
> The bugs below are FIXED. If a future regression reintroduces any of them,
> the test suite will catch it. Do not "simplify" code that appears redundant
> here — each guard exists for a reason.

### 4.1 StateMachine — ACQUIRING Miss-Counting (Phase 3, app/state_machine.py)

**Bug**: In ACQUIRING state, `consecutive_misses` was incremented whenever
`error_px > 10px`, even if the beacon WAS detected. After 3 such frames
(`MISS_TO_SEARCH = 3`), the FSM retreated to SEARCHING before the PD controller
could converge. This caused `ACQUIRING → SEARCHING → ACQUIRING → ...` infinite
cycling with RMSE ~120 px.

**Root Cause**: The lock criterion `locked = detected AND error_px ≤ 10` is correct
for TRACKING, but WRONG for ACQUIRING miss-counting. In ACQUIRING, the beacon is
visible and the camera is closing in — the high error is a controller-convergence
artefact, not a target-loss event.

**Fix** (`app/state_machine.py`, `update()` counter block):
```python
in_acq_state = self._state in (TrackingState.ACQUIRING, TrackingState.REACQUIRING)

if locked:
    self._consecutive_locks  += 1
    self._consecutive_misses  = 0
else:
    self._consecutive_locks   = 0
    if in_acq_state and detected:
        pass   # detected but error > threshold — controller still converging
    else:
        self._consecutive_misses += 1
```

**Added Safety**: `_acq_frames` counter — if ACQUIRING exceeds 90 frames without
locking, retreat to SEARCHING to avoid infinite hang:
```python
elif self._acq_frames >= COAST_TIMEOUT:   # 90 frames = 3s at 30fps
    self._state = TrackingState.SEARCHING
    self._acq_frames = 0
```

### 4.2 Figure8Motion — Initial Position Offset (Phase 3, simulation/motion_models.py)

**Bug**: `Figure8Motion` default `phi_y = π/2`. The Lissajous y-formula:
$$y(0) = center\_y + amp\_y \cdot \sin(\phi_y)$$
With `phi_y = π/2` and `amp_y = 175 px`: $y(0) = 1000 + 175 = 1175$.
Camera coords: `(320, 415)` — **175 px below frame centre.** This made the
figure-8 start identically broken to the circular orbit start problem.

**Fix**: `phi_y` default changed from `math.pi / 2` to `0.0`:
```python
# simulation/motion_models.py — Figure8Motion.__init__
phi_y: float = 0.0,   # 0.0 → beacon starts at canvas centre (sin(0)=0)
```

Now: $y(0) = 1000 + 175 \cdot \sin(0) = 1000$. Camera: `(320, 240)`. Error: **0 px.**

**Impact**: Phase 2 tests (46/46) still pass — the figure-8 trajectory shape is
identical, only the start phase differs.

### 4.3 Actuator Deadband — Lock-Zone Penetration (Phase 3, app/orchestrator.py)

**Bug**: The PD controller Phase 1 default deadband is `0.2°` (calibrated for unit
tests). Converting: `0.2° / 0.00625 deg/px = 32 px`. Since the ISRO lock threshold
is 10 px, the controller stopped correcting at 32 px — **22 px outside the lock zone.**
The FSM could never accumulate 5 consecutive locked frames; TRACKING was unreachable.

**Fix**: `_init_pipeline()` in `orchestrator.py` overrides deadband:
```python
self._ctrl = PDController(
    kp=self._kp, kd=self._kd,
    max_rate_deg_s=5.0,
    deadband_deg=0.05,   # 0.05° ≈ 8 px — allows convergence within 10px lock zone
)
```

`0.05° = 8 px < 10 px lock threshold`. Controller now converges inside the lock
zone → FSM can accumulate locks → TRACKING reached.

### 4.4 FrameLogger.summary() — Guards (Phase 3, evaluation/logger.py)

**Bug 1 (row_count guard)**: `summary()` previously returned `{}` if `self._row_count == 0`. When a `FrameLogger` was re-opened on an existing CSV (or used after close), `_row_count = 0` caused premature early return even when the file had hundreds of valid rows.

**Fix**: Replace `_row_count == 0` with file-size check:
```python
if not self._path.exists() or self._path.stat().st_size < 100:
    return {}
```

**Bug 2 (flush on closed file)**: `summary()` called `self.flush()` unconditionally.
After `close()`, the underlying file handle is gone — `flush()` raises `I/O operation
on closed file`.

**Fix**: Guard the flush call:
```python
if not self._closed:
    self.flush()
```

**Bug 3 (float cast in summary loop)**: `abs(row["tilt_deg_s"])` failed because
CSV values are strings. Fixed to `abs(float(row["tilt_deg_s"]))`.

### 4.5 sim_source.py — apply_to_frame() Signature (Phase 2, ingestion/sim_source.py)

**Bug**: `disturbances.py` was changed to return `ndarray` directly (not a tuple)
from `apply_to_frame()`. The caller had `[0]` indexing: `apply_to_frame(frame)[0]`
which indexed the first ROW of the returned ndarray instead of the frame.

**Fix**: Remove `[0]` — call `apply_to_frame(raw_viewport)` directly.

### 4.6 StraightLineMotion — Name Collision (Phase 2, simulation/motion_models.py)

**Bug**: `self._reflect = reflect` (boolean attribute) shadowed `_reflect()`
staticmethod, causing `TypeError: bool is not callable` on boundary reflection.

**Fix**: Renamed boolean attribute to `self._do_reflect`.

### 4.7 Fog Koschmieder Coefficient (Phase 2, simulation/disturbances.py)

**Bug**: Original formula `I_out = 0.60 * I_in + 80` — for `I(180) = 188 > 175`
(the detection threshold). Fog was not actually attenuating bright beacons.

**Fix**: `I_out = 0.50 * I_in + 55` — for `I(180) = 145 < 175`. Fog now correctly
suppresses detection under heavy atmospheric conditions.

---

## 5. Verification & Run Commands

> [!IMPORTANT]
> Always use `py -3 -X utf8` on Windows to avoid codec errors in UTF-8 log output.
> All commands assume CWD = `d:\CODING_NOC\BeaconLock\`.

### 5.1 Full Regression Suite (Run All Phases)

```powershell
# Phase 1 — Pure Math Core (31 assertions)
py -3 -X utf8 tests/test_phase1_headless.py

# Phase 2 — Simulation Engine (46 assertions)
py -3 -X utf8 tests/test_phase2_sim.py

# Phase 3 — FSM, UI, QThread Worker (33 assertions)
py -3 -X utf8 tests/test_phase3_ui.py

# Headless Integration Check (16 assertions, includes PDF + QThread)
py -3 -X utf8 main.py --headless-check
```

### 5.2 Expected Results (Current Green Baseline)

| Command | Expected Exit Code | Expected Output |
|---|---|---|
| `test_phase1_headless.py` | 0 | `31/31 PASS  Pass rate: 100.0%` |
| `test_phase2_sim.py` | 0 | `46/46 PASS  Pass rate: 100.0%` |
| `test_phase3_ui.py` | 0 | `33/33 PASS  Pass rate: 100.0%` |
| `main.py --headless-check` | 0 | `16/16 PASS  Pass rate: 100.0%` |

If ANY exit code is non-zero, **do not proceed** with Phase 4 work. Diagnose the
regression using the bug-fix history in §4.

### 5.3 Launch Interactive GUI

```powershell
py -3 main.py
```

This opens the PySide6 cockpit dashboard. Default mode: Mode A (Synthetic Simulator),
figure-8 motion, no disturbances. Use the left panel to change settings, then hit
**START**.

### 5.4 Python Environment

```powershell
# Verify all required packages
py -3 -m pip show numpy opencv-python filterpy pyyaml reportlab pyqtgraph PySide6

# Installed versions (Phase 3 baseline)
# numpy           : 2.2.6
# opencv-python   : 5.0.0.93
# filterpy        : 1.4.5
# pyyaml          : 6.0.2
# reportlab       : (confirmed by PDF generation)
# pyqtgraph       : (installed)
# PySide6         : 6.11.2
# shiboken6       : 6.11.2
# PySide6_Addons  : 6.11.2
# PySide6_Essentials: 6.11.2
```

### 5.5 Pycache / Bytecode Anomalies

If test results appear stale (same RMSE numbers across runs with different parameters),
clear Python bytecode cache:

```powershell
Get-ChildItem -Recurse -Path "d:\CODING_NOC\BeaconLock" -Filter "*.pyc" | Remove-Item -Force
```

---

## 6. Shift 2 Actionable Roadmap

> **To incoming engineer (Shift 2 / next account):**
> Phases 1–3 are sealed. Your mandate is Phase 4 only. Do not restructure,
> rename, or refactor any file in `perception/`, `control/`, `app/`, or the
> existing test files. If you observe a regression in a Phase 1–3 test, document
> it here (§7) and escalate to nik40 before patching.

### 6.1 Deliverable 1 — Mode B Video File Ingestion (Priority: HIGH)

**File**: `ingestion/frame_source.py` → `VideoFileSource` (stub exists, needs full implementation)

**Specification**:
- Accept any 640×480 30-FPS grayscale or colour MP4/AVI evaluator video.
- Implement `read()` using `cv2.VideoCapture`.
- Handle end-of-file gracefully (return `(False, None)` — do not crash).
- Convert colour frames to grayscale (`cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)`) before returning.
- Implement `get_fps()` from `cap.get(cv2.CAP_PROP_FPS)`.
- Implement `get_resolution()` from `cap.get(cv2.CAP_PROP_FRAME_WIDTH/HEIGHT)`.
- Wire `VideoFileSource` into `Dashboard`'s Mode B selector (already has a file-picker placeholder).

**Test Coverage Needed**:
- `tests/test_phase4_mode_b.py`: Load a synthetically generated 30-FPS 640×480 MP4 (use `cv2.VideoWriter` to create it in the test), run the full tracking pipeline, assert RMSE ≤ 10 px and FPS ≥ 30.

**Architecture Reminder**: `VideoFileSource` must NOT import from `simulation/`. It reads external pixels — it is the Mode B analogue of `SimulatorSource`.

### 6.2 Deliverable 2 — Edge-Case Audit Report (Priority: HIGH)

**Purpose**: Demonstrate honest failure-boundary documentation to ISRO evaluators. The PDF report must show real, measured failure conditions.

**Scenarios to Run and Capture**:

| Scenario | Settings | Expected Outcome to Document |
|---|---|---|
| Extreme Fog | `atmospheric='fog'` + `gaussian_sigma=20` | Centroid fails when I < threshold; RMSE diverges |
| Max Jitter | `jitter_px=20` | Re-acquisition latency measured (should be ≤ 1.0 s = 30 frames) |
| Full Occlusion (10 s) | `inject_occlusion(300)` | Kalman coasting accuracy over 300 frames |
| Salt & Pepper 50% | `sp_prob=0.50` | CoG degrades below ISRO threshold |

**Deliverable**: Run each scenario via the GUI or a headless script, collect CSV logs,
generate PDFs via `ReportGenerator`. Commit the generated PDFs to `docs/audit_reports/`.

### 6.3 Deliverable 3 — PyInstaller Standalone Packaging (Priority: MEDIUM)

**Goal**: Ship a single-file `.exe` that evaluators can run without Python installed.

```powershell
# Install PyInstaller
py -3 -m pip install pyinstaller

# Build single-file executable
py -3 -m PyInstaller --onefile --windowed --name BeaconLock main.py

# Test the executable (headless-check only)
dist\BeaconLock.exe --headless-check
```

**Known Risks**:
- PySide6 + PyQtGraph + OpenCV + filterpy all need to be discovered by PyInstaller's
  analysis. You may need a `.spec` file with explicit `hiddenimports`.
- On Windows, `--windowed` suppresses the console — use `--console` for debugging.
- ReportLab font files may need `--add-data` to be included.

**Commit**: The working `BeaconLock.spec` file to `packaging/BeaconLock.spec`.

### 6.4 Deliverable 4 — YAML Configuration Profiles (Priority: LOW)

**Files to Create** (paths from MASTER_BLUEPRINT.md):
- `configs/baseline.yaml` — Standard evaluation config (figure_8, clear atmosphere, sigma=5, no jitter).
- `configs/dynamic_jitter.yaml` — Jitter stress test (jitter_px=15, sigma=12, haze atmosphere).
- `configs/evaluator_stress.yaml` — Maximum challenge (fog, sigma=20, jitter_px=18, random_walk motion).

Each YAML profile should be loadable by `SimulatorSource` via a `--config` argument
in `main.py`. Use `pyyaml` (already installed).

### 6.5 Protocol for Rotating Back to nik40

When Shift 2 completes their work:

1. Run the full regression suite (all 4 commands from §5.1). Capture terminal output.
2. Fill in §7 (Shift 2 Log) in this file with measured benchmarks, commit hashes, and any open issues.
3. If new tests were added, state the new total assertion count.
4. If ANY Phase 1–3 test FAILS, escalate **before** committing and describe the regression in §7.
5. Push all changes. Create a git commit message: `feat(phase4): [brief description] — Shift2 complete`.
6. Notify nik40 with the final pass/fail matrix and §7 contents.

---

## 7. Shift 2 Log (Incoming Engineer Fills This)

> **Instructions**: Fill in this section when your shift is complete.
> Do not delete any rows — append a new sub-section for each sub-deliverable.

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  SHIFT 2 HANDOFF LOG
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Engineer/Account  : [ TO BE FILLED ]
  Shift Start       : [ TO BE FILLED — ISO 8601 timestamp ]
  Shift End         : [ TO BE FILLED — ISO 8601 timestamp ]
  Git Commit Hash   : [ TO BE FILLED ]

  ── Regression Check (Run BEFORE starting any Phase 4 work) ──

  test_phase1_headless.py : [ PASS / FAIL ] [ __ / 31 ]
  test_phase2_sim.py      : [ PASS / FAIL ] [ __ / 46 ]
  test_phase3_ui.py       : [ PASS / FAIL ] [ __ / 33 ]
  main.py --headless-check: [ PASS / FAIL ] [ __ / 16 ]

  ── Deliverable 1 — Mode B VideoFileSource ──

  Status          : [ NOT STARTED / IN PROGRESS / COMPLETE ]
  Test file       : [ path to test_phase4_mode_b.py ]
  Test results    : [ __ / __ assertions ]
  Notes           : [ any edge cases or failures ]

  ── Deliverable 2 — Edge-Case Audit Report ──

  Status          : [ NOT STARTED / IN PROGRESS / COMPLETE ]
  PDF paths       : [ paths to generated audit PDFs ]
  Fog failure Px  : [ measured pixel threshold where detection fails ]
  Reacq latency   : [ measured frames from loss to re-lock ]
  Notes           : [ ]

  ── Deliverable 3 — PyInstaller Packaging ──

  Status          : [ NOT STARTED / IN PROGRESS / COMPLETE ]
  Spec file       : [ packaging/BeaconLock.spec ]
  .exe size       : [ MB ]
  HC test on .exe : [ PASS / FAIL ] [ __ / 16 ]
  Notes           : [ hidden imports needed, etc. ]

  ── Deliverable 4 — YAML Config Profiles ──

  Status          : [ NOT STARTED / IN PROGRESS / COMPLETE ]
  Files created   : [ list ]
  Notes           : [ ]

  ── Open Issues / Escalations for nik40 ──

  [ None at handoff ] OR [ describe any regressions or blockers ]

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

---

*Document end — SHIFT_HANDOFF_NIK40.md — nik40 — 2026-10-02T20:22:46+05:30*
