# 🔒 BeaconLock — Virtual Camera PAT Simulator

<div align="center">

[![SIH 2026](https://img.shields.io/badge/Smart%20India%20Hackathon-2026-orange?style=for-the-badge)](https://www.sih.gov.in/)
[![PS-26169](https://img.shields.io/badge/Problem%20Statement-PS--26169-blue?style=for-the-badge)]()
[![Team AlphaTrion](https://img.shields.io/badge/Team-AlphaTrion%20%7C%20ID%20176697-green?style=for-the-badge)]()
[![Organisation](https://img.shields.io/badge/Organisation-ISRO%20%7C%20Dept%20of%20Space-navy?style=for-the-badge)](https://www.isro.gov.in/)
[![Python](https://img.shields.io/badge/Python-3.13%2B-blue?style=for-the-badge&logo=python)](https://python.org)
[![Platform](https://img.shields.io/badge/Platform-CPU%20Only%20%7C%20Zero%20GPU-success?style=for-the-badge)]()
[![Tests](https://img.shields.io/badge/Tests-126%2B%20Assertions%20%7C%20100%25%20Pass-brightgreen?style=for-the-badge)]()

</div>

---

## System Overview

**BeaconLock** is a high-fidelity **digital twin** of the Coarse Acquisition / Pointing-Acquisition-Tracking (PAT) subsystem used in **Free Space Optical (FSO) Communication** links. It is developed as a research and evaluation tool for **ISRO Problem Statement PS-26169** in SIH 2026.

The system simulates a rate-limited virtual pan-tilt camera tracking a configurable optical beacon across a synthetic 2000 × 2000 px world canvas, applying realistic atmospheric disturbances, sensor noise, and platform jitter. All algorithms run at ≥ 30 FPS on a standard CPU — **no GPU required**.

### Key Capabilities

| Capability | Implementation |
|---|---|
| Beacon tracking | Radiometric CoG + 6-state Kalman filter (constant-acceleration) |
| Camera control | Rate-limited PD controller, ±5.0 °/s saturation |
| State machine | 5-state FSM: SEARCHING → ACQUIRING → TRACKING → LOST → REACQUIRING |
| World canvas | 2000 × 2000 px, 640 × 480 px focal plane array, FOV 4° × 3° |
| Beacon motion | Straight-line, Circular, Figure-8 (Lissajous), Random Walk (Ornstein–Uhlenbeck) |
| Disturbances | Gaussian, Salt & Pepper, Poisson, Camera Jitter ±20px, Platform Drift, Atmosphere |
| Ingestion modes | Mode A (synthetic simulator) / Mode B (evaluator MP4 video file) |
| Audit report | Automated ISRO-compliant ReportLab PDF with pass/fail matrix |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         BEACONLOCK ARCHITECTURE                              │
│                  ISRO SIH-2026 | PS-26169 | AlphaTrion (176697)             │
└─────────────────────────────────────────────────────────────────────────────┘

 ┌─────────────────────────── MODE A (Synthetic) ─────────────────────────────┐
 │                                                                              │
 │  simulation/scene.py  ←─ simulation/target.py ←─ simulation/motion_models.py│
 │  (2000×2000 canvas)        (5–20px beacon spot)   (4 motion models)         │
 │          │                                                                   │
 │          ├── simulation/disturbances.py (Gaussian/S&P/Poisson/Fog/Jitter)   │
 │          │                                                                   │
 │          └── simulation/camera.py (640×480 viewport, rate-limited PTZ)      │
 │                         │                                                    │
 │                ingestion/sim_source.py  ◄─── THE ONLY sim/ BRIDGE           │
 └──────────────────────────────────────────┼───────────────────────────────────┘
                                            │
 ┌───────────── MODE B (Evaluator MP4) ─────┘
 │                                          │
 │  ingestion/frame_source.py               │
 │  VideoFileSource(cv2.VideoCapture)  ─────┘
 └──────────────────────────────────────────┤
                                            │
                          ┌─────────────────▼────────────────────┐
                          │  FrameSource.read() → (bool, ndarray) │
                          │  (640×480 greyscale uint8)            │
                          └─────────────────┬────────────────────┘
                                            │
        ┌───────────────────────────────────▼───────────────────────────────┐
        │             app/orchestrator.py  (TrackingWorker: QThread)         │
        │                                                                     │
        │  perception/centroid.py         perception/kalman_tracker.py        │
        │  ┌─────────────────────┐        ┌──────────────────────────────┐   │
        │  │ Background:         │        │ State: [x, y, vx, vy, ax, ay]│   │
        │  │  I_th = μ + 3σ      │───────▶│ 6-state constant-accel KF    │   │
        │  │ CoG: Σ(I·x)/Σ(I)   │        │ Adaptive R (confidence gate) │   │
        │  └─────────────────────┘        └─────────────┬────────────────┘   │
        │                                               │                     │
        │  control/pid_controller.py                    │                     │
        │  ┌───────────────────────────────────────────▼─────┐               │
        │  │ error_px = hypot(pred_x - 320, pred_y - 240)    │               │
        │  │ rate = Kp·err_deg + Kd·d(err_deg)/dt            │               │
        │  │ clamp(rate, ±5.0 °/s)  |  deadband = 0.05°      │               │
        │  └─────────────────────────────────────────────────┘               │
        │                                                                     │
        │  app/state_machine.py (5-state FSM)   evaluation/logger.py          │
        │  SEARCHING → ACQUIRING → TRACKING      11-col CSV telemetry         │
        │  TRACKING  → LOST → REACQUIRING        (thread-safe, buffered)      │
        └────────────────────────────┬────────────────────────────────────────┘
                                     │ Qt signals (frame_ready, state_changed)
                          ┌──────────▼───────────────────────────────────────┐
                          │  ui/dashboard.py  (PySide6 QMainWindow)           │
                          │  3-panel industrial cockpit                        │
                          │  Left: controls  │ Center: 640×480 feed  │ Right: │
                          │  ui/plots_panel.py (PyQtGraph real-time curves)   │
                          │  evaluation/report_generator.py (ReportLab PDF)   │
                          └──────────────────────────────────────────────────┘
```

---

## Verified Benchmark Registry

All numbers are **measured ground truth** on a standard Windows CPU (no GPU):

| Metric | Measured | ISRO Specification | Status |
|---|---|---|---|
| **Pure math throughput** (Centroid+Kalman+PD) | **2,370 – 4,345 FPS** | ≥ 30 FPS | ✅ |
| **Closed-loop sim + noise + perception** | **~159 FPS** | ≥ 30 FPS | ✅ |
| **QThread worker mean FPS** | **~140 FPS** | ≥ 30 FPS | ✅ |
| **Mode B (VideoFileSource) pipeline** | **~146 FPS** | ≥ 30 FPS | ✅ |
| **Centroid RMSE (clean)** | **0.707 px** | — | Sub-pixel ✅ |
| **Centroid RMSE (σ=15 noise, 64×64 ROI)** | **0.755 px** | — | Sub-pixel ✅ |
| **Phase 2 Figure-8 tracking RMSE** | **2.51 px** | ≤ 10 px | ✅ |
| **Phase 3 QThread TRACKING-state RMSE** | **8.31 px** | ≤ 10 px | ✅ |
| **Max actuator slew (Phase 2)** | **0.995 °/s** | ≤ 5.0 °/s | ✅ |
| **Max actuator slew (Phase 3 worker)** | **0.190 °/s** | ≤ 5.0 °/s | ✅ |
| **Slew saturation violations** | **0** | 0 | ✅ |
| **Mode B centroid RMSE vs GT (MP4)** | **1.430 px** | ≤ 10 px | ✅ |
| **Mode B detection rate** | **100%** | ≥ 85% | ✅ |
| **FSM all 5 states cycled** | ✅ | Required | ✅ |
| **Acquisition latency** | ≤ 1.0 s | ≤ 2.0 s | ✅ |
| **Target loss rate** | < 5% | < 5% | ✅ |

---

## Directory Structure

```
BeaconLock/
│
├── app/
│   ├── orchestrator.py         # TrackingWorker(QThread): 13-step per-frame loop
│   └── state_machine.py        # 5-state FSM with detection-aware miss counting
│
├── control/
│   └── pid_controller.py       # Rate-limited PD, 0.05° deadband, ±5.0°/s clamp
│
├── evaluation/
│   ├── logger.py               # Thread-safe 11-column CSV telemetry logger
│   └── report_generator.py     # ReportLab PDF: session + ISRO matrix + boundaries
│
├── ingestion/
│   ├── frame_source.py         # Abstract FrameSource ABC + VideoFileSource (Mode B)
│   └── sim_source.py           # SimulatorSource (Mode A) — ONLY sim/ bridge
│
├── perception/
│   ├── centroid.py             # Radiometric background CoG (border-annulus method)
│   └── kalman_tracker.py       # 6-state [x,y,vx,vy,ax,ay] Kalman with adaptive R
│
├── simulation/
│   ├── camera.py               # VirtualCamera: 640×480 viewport + rate-clamped PTZ
│   ├── disturbances.py         # Gaussian/S&P/Poisson noise, Jitter, Platform, Atmos
│   ├── motion_models.py        # StraightLine, Circular, Figure8, RandomWalk models
│   ├── scene.py                # WorldScene: 2000×2000 canvas renderer
│   └── target.py               # BeaconTarget: 5–20px spot + occlusion injection
│
├── ui/
│   ├── assets/
│   │   ├── icon.ico            # Optical crosshair application icon (Windows taskbar)
│   │   └── icon.png            # Same icon as PNG (64×64 RGBA)
│   ├── dashboard.py            # PySide6 QMainWindow: 3-panel industrial cockpit
│   └── plots_panel.py          # PyQtGraph: Error / Slew / FPS real-time plots
│
├── tests/
│   ├── test_phase1_headless.py # 31 assertions: CoG, Kalman, PD math verification
│   ├── test_phase2_sim.py      # 46 assertions: simulation engine + disturbances
│   ├── test_phase3_ui.py       # 33 assertions: FSM, logger, PDF, QThread worker
│   └── test_phase4_mode_b.py   # 17+ assertions: Mode B ingestion + pipeline
│
├── docs/
│   ├── MASTER_BLUEPRINT.md     # Canonical ISRO spec + algorithm decisions
│   └── SHIFT_HANDOFF_NIK40.md  # Engineering shift handoff document
│
├── main.py                     # Entry point: GUI launch + --headless-check
├── requirements.txt            # Pinned Python 3.13-compatible dependencies
└── README.md                   # This file
```

---

## Installation

### Prerequisites

- **Python 3.13+** (64-bit, Windows recommended)
- **pip** package manager

### Step 1 — Clone the repository

```powershell
git clone https://github.com/AlphaTrion/BeaconLock.git
cd BeaconLock
```

### Step 2 — Install dependencies

```powershell
py -3 -m pip install -r requirements.txt
```

**`requirements.txt` contents (pinned for Python 3.13):**
```
numpy==2.2.6
opencv-python==5.0.0.93
filterpy==1.4.5
PySide6==6.11.2
pyqtgraph==0.14.0
reportlab
pandas
pyyaml
Pillow
```

### Step 3 — Verify installation

```powershell
py -3 -X utf8 main.py --headless-check
```

Expected output: `16/16 PASS  Pass rate: 100.0%`

---

## Quickstart

### Launch Interactive GUI

```powershell
py -3 main.py
```

This opens the 3-panel aerospace telemetry cockpit:
- **Left panel**: Source selector (Mode A simulator / Mode B MP4), motion & disturbance controls, session buttons
- **Center panel**: Live 640×480 camera feed with CoG crosshair, Kalman prediction ellipse, tracking bounding box
- **Right panel**: Real-time PyQtGraph plots — tracking error (px), slew rate (°/s), FPS

### Mode A — Synthetic Simulator

1. Select **Mode A – Synthetic Simulator** (default)
2. Choose beacon motion: `straight_line`, `circular`, `figure_8`, `random_walk`
3. Set disturbances (noise σ, jitter, atmosphere preset)
4. Press **▶ START**
5. Click **⊘ Inject Blind Spot** to test re-acquisition
6. Click **📄 Export Audit PDF** at any time

### Mode B — Evaluator MP4 Ingestion

1. Select **Mode B – Evaluator MP4**
2. Click **📂 Browse MP4…** → select any 640×480 (or arbitrary resolution) MP4/AVI
3. Press **▶ START** — the video streams through the identical CoG + Kalman tracking core
4. PTZ simulation is cleanly bypassed; only perception and control run

---

## Running Test Suites

```powershell
# Phase 1 — Pure Math Core (31 assertions)
py -3 -X utf8 tests/test_phase1_headless.py

# Phase 2 — Simulation Engine, Disturbances, Closed-Loop (46 assertions)
py -3 -X utf8 tests/test_phase2_sim.py

# Phase 3 — FSM, CSV Logger, PDF, QThread Worker (33 assertions)
py -3 -X utf8 tests/test_phase3_ui.py

# Phase 4 — Mode B VideoFileSource (17 assertions)
py -3 -X utf8 tests/test_phase4_mode_b.py

# Full integration check (16 assertions, no display required)
py -3 -X utf8 main.py --headless-check
```

**Total assertions: 126+ across all phases — 100% pass rate.**

---

## Evaluator Audit & PDF Report Generation

BeaconLock automatically generates an ISRO-compliant PDF audit report at the end of each session:

1. Start and run a tracking session (Mode A or Mode B)
2. Click **📄 Export Audit PDF** in the dashboard
3. The report includes:
   - **Section 1**: Session summary (motion type, atmospheric condition, duration, total frames)
   - **Section 2**: ISRO Performance Benchmark Matrix — RMSE, Lock Rate, FPS, Max Slew vs. specifications
   - **Section 3**: Honest failure boundary documentation (conditions under which specs are not met)

Reports are saved to a timestamped temp directory and the path is displayed in the status bar.

---

## Algorithm Reference

### Radiometric Centre of Gravity (CoG)

$$
I_{th} = \mu_{bg} + 3\sigma_{bg}
$$

$$
x_{cog} = \frac{\sum_i \max(I_i - I_{th},\, 0) \cdot x_i}{\sum_i \max(I_i - I_{th},\, 0)}
\quad
y_{cog} = \frac{\sum_i \max(I_i - I_{th},\, 0) \cdot y_i}{\sum_i \max(I_i - I_{th},\, 0)}
$$

Background estimated from border-annulus pixels; 64×64 ROI used when tracking is locked.

### 6-State Constant-Acceleration Kalman Filter

$$
\mathbf{x} = \begin{bmatrix} x \\ y \\ v_x \\ v_y \\ a_x \\ a_y \end{bmatrix}
\quad
\mathbf{F} = \begin{bmatrix}
1 & 0 & dt & 0 & \frac{dt^2}{2} & 0 \\
0 & 1 & 0 & dt & 0 & \frac{dt^2}{2} \\
0 & 0 & 1 & 0 & dt & 0 \\
0 & 0 & 0 & 1 & 0 & dt \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

Measurement noise covariance adapts to detection confidence: $R = R_{base} / \max(c, 0.01)$.

### Rate-Limited PD Controller

$$
e_{deg} = e_{px} \times 0.00625 \;\text{deg/px}
$$

$$
u(t) = K_p \cdot e_{deg}(t) + K_d \cdot \dot{e}_{deg}(t)
$$

$$
u_{out} = \text{clamp}\bigl(u(t),\, -5.0°/\text{s},\, +5.0°/\text{s}\bigr)
\quad \text{if } |e_{deg}| > 0.05°
$$

Angular pointing error ceiling: $10\;\text{px} \times 0.00625\;\text{deg/px} = 0.0625° \approx 1.09\;\text{mrad}$

---

## FSM State Diagram

```
              detected
SEARCHING ─────────────────► ACQUIRING
                                  │
                    5 consecutive  │  ≥3 detection-misses
                    locked frames  │  OR 90-frame timeout
                                  ▼
                              TRACKING ◄────────────────────────┐
                                  │                              │
                         5 misses │                              │ 3 re-locks
                                  ▼                              │
                                LOST ──── detected ──► REACQUIRING
                                  │
                       90-frame   │
                       coast TMO  │
                                  ▼
                              SEARCHING
```

**Lock criterion**: `detected AND error_px ≤ 10.0 px`  
**Miss criterion (TRACKING)**: `not detected OR error_px > 10.0 px`  
**Miss criterion (ACQUIRING)**: `not detected` only (high error during convergence is not a miss)

---

## Architectural Decoupling Rule

```
simulation/           ← pure world model, no UI imports
    │
    └── ingestion/sim_source.py   ← THE ONLY file that imports simulation/
                                     (Mode A concrete implementation)
ingestion/frame_source.py        ← Abstract FrameSource ABC (numpy only)
    │
perception/, control/            ← ZERO simulation/ imports (verified by test T2)
    │
app/orchestrator.py              ← imports perception + control + evaluation
    │
ui/                              ← imports app/ only
```

This guarantees **Benchmark-2 compliance**: replacing Mode A with a judge-supplied MP4 (Mode B) requires zero changes to any perception, control, or app layer.

---

## Team

| Role | Name |
|---|---|
| Principal Aerospace Software Architect & Lead CV Engineer | nik40 |
| Team | AlphaTrion |
| Team ID | 176697 |
| Challenge | Smart India Hackathon 2026 |
| Problem Statement | PS-26169 — Department of Space / ISRO |

---

<div align="center">
<sub>BeaconLock v0.4.0 — ISRO SIH-2026 | PS-26169 | AlphaTrion (Team ID 176697)</sub>
</div>
