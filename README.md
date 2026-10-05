# 🔒 BeaconLock — Virtual Camera PAT Simulator for Free Space Optical Communications

<div align="center">

[![SIH 2026](https://img.shields.io/badge/Smart%20India%20Hackathon-2026-orange?style=for-the-badge)](https://www.sih.gov.in/)
[![PS-26169](https://img.shields.io/badge/Problem%20Statement-PS--26169-blue?style=for-the-badge)](https://www.sih.gov.in/)
[![Organisation](https://img.shields.io/badge/Organisation-ISRO%20%7C%20Dept.%20of%20Space-navy?style=for-the-badge)](https://www.isro.gov.in/)
![Team AlphaTrion](https://img.shields.io/badge/Team-AlphaTrion%20%7C%20ID%20176697-green?style=for-the-badge)
![Platform](https://img.shields.io/badge/Hardware-CPU%20Only%20%7C%20Zero%20GPU-success?style=for-the-badge)
![Tests](https://img.shields.io/badge/Test%20Suite-143%20%2F%20143%20PASS%20(100%25)-brightgreen?style=for-the-badge)
[![Release](https://img.shields.io/badge/Release-v1.0.0%20Production-blueviolet?style=for-the-badge)](https://github.com/NocStaRex/BeaconLock/releases/tag/v1.0.0)

</div>

---

## Executive Summary

**BeaconLock** is a real-time, high-fidelity digital twin of the **Coarse Pointing, Acquisition, and Tracking (PAT)** subsystem designed for **Free Space Optical Communications (FSOC)** terminals. Developed for **ISRO Problem Statement PS-26169** at the **Smart India Hackathon (SIH) 2026**, the system emulates a closed-loop optical transceiver tracking an agile beacon target across a 2000 × 2000 px virtual space.

The software features a strictly decoupled four-tier architecture, combining radiometric sub-pixel centroiding, a 6-state constant-acceleration Kalman filter, rate-saturated kinematic actuation, and an aerospace telemetry dashboard. The entire tracking and control loop executes at **over 140 FPS on standard multi-core CPUs with zero GPU dependency**, exceeding ISRO's mandatory 20 FPS threshold by more than 700%.

<div align="center">
  <br/>
  <img src="ui/assets/dashboard_live_tracking.png" alt="BeaconLock Live Telemetry Cockpit" width="100%">
  <p><em>BeaconLock live telemetry cockpit running closed-loop coarse acquisition on a stochastic random-walk beacon (~78 FPS throughput, < 10 px tracking error lock).</em></p>
  <br/>
</div>

---

## 4-Tier Decoupled System Architecture

BeaconLock implements a unidirectional, interface-driven architecture that completely isolates sensor simulation from perception and control algorithms. This guarantees seamless compliance for both synthetic closed-loop testing (Mode A) and evaluator video ingestion (Mode B).

```
╔═══════════════════════════════════════════════════════════════════════════════════════════════════╗
║                                     TIER 1: INGESTION LAYER                                       ║
╚═══════════════════════════════════════════════════════════════════════════════════════════════════╝
               │                                                                    │
      [ MODE A: SYNTHETIC ]                                                [ MODE B: EVALUATOR ]
   simulation/scene.py (2000×2000)                                       ingestion/frame_source.py
   simulation/target.py (5–20 px)                                          VideoFileSource (OpenCV)
   simulation/disturbances.py (Fog/Jitter)                                 30-FPS MP4 / AVI Stream
   simulation/camera.py (Rate-Limited PTZ)                                 Resolution Auto-Rescaling
               │                                                                    │
               └───► ingestion/sim_source.py (SimulatorSource) ◄────────────────────┘
                                         │
                          FrameSource ABC: read() -> (ok, ndarray)
                          640 × 480 px Greyscale Viewport (FOV 4° × 3°)
                                         │
                                         ▼
╔═══════════════════════════════════════════════════════════════════════════════════════════════════╗
║                                     TIER 2: PERCEPTION LAYER                                      ║
╚═══════════════════════════════════════════════════════════════════════════════════════════════════╝
   perception/centroid.py (CentroidEstimator)
   ├── Radiometric Background Thresholding:  I_th = μ_bg + 3·σ_bg (Border-Annulus Estimation)
   ├── Sub-Pixel Center of Gravity (CoG):    x_cog = Σ(I·x) / Σ(I),  y_cog = Σ(I·y) / Σ(I)
   └── Adaptive Multi-Scale ROI:             Full-frame Search  <───►  Dynamic 64×64 Tracking ROI
                                         │
                                         ▼
╔═══════════════════════════════════════════════════════════════════════════════════════════════════╗
║                             TIER 3: KINEMATIC TRACKING & CONTROL                                  ║
╚═══════════════════════════════════════════════════════════════════════════════════════════════════╝
   perception/kalman_tracker.py (KalmanTracker)
   ├── 6-State Constant-Acceleration Filter: x = [x, y, v_x, v_y, a_x, a_y]^T
   ├── Adaptive Measurement Covariance:      R(c) = R_base / max(confidence, 0.01)
   └── Target Occlusion Coasting:            Kalman Propagation during Signal Blanking
                                         │
   control/pid_controller.py (PDController)
   ├── Proportional-Derivative Controller:   u(t) = Kp · e_deg + Kd · (de_deg / dt)
   ├── Actuator Rate Slew Clamp:             Bounded strictly to ±5.0 °/s (0.00625 °/px scale)
   └── Pointing Deadband:                    0.05° (8 px) Convergence Window
                                         │
   app/state_machine.py (StateMachine)
   └── 5-State FSM: SEARCHING ──► ACQUIRING ──► TRACKING ──► LOST ──► REACQUIRING
                                         │
                                         ▼
╔═══════════════════════════════════════════════════════════════════════════════════════════════════╗
║                           TIER 4: ACTUATION & TELEMETRY COCKPIT                                   ║
╚═══════════════════════════════════════════════════════════════════════════════════════════════════╝
   app/orchestrator.py (TrackingWorker: QThread)
   ├── Closed-Loop Gimbal Feedback:          simulation/camera.py (Mode A only)
   ├── Live Reticle Rendering:               Sub-pixel crosshair, Kalman covariance ellipse, ROI box
   ├── Telemetry Serialization:              evaluation/logs/session_YYYYMMDD_HHMMSS.csv
   │
   ui/dashboard.py & ui/plots_panel.py
   ├── Aerospace Obsidian Cockpit:           PySide6 Dark Console (#0b0f19, #111827)
   ├── Real-Time PyQtGraph Plots:            Tracking Error (10px line), Slew (±5°/s), FPS (30/40)
   └── Automated Compliance Reporting:       evaluation/reports/audit_YYYYMMDD_HHMMSS.pdf
```

---

## Verified Benchmark Registry (Measured Ground Truth)

All benchmark metrics below represent empirical ground-truth measurements captured across standardized multi-run headless suites and closed-loop QThread sessions on standard commodity x86-64 CPUs:

| Performance Metric | Measured Value | ISRO Specification | Compliance Status |
|---|---|---|:---:|
| **Pure Mathematical Throughput** (Perception + KF + PD) | **2,370 – 4,345 FPS** | ≥ 30 FPS | **PASS** (144× headroom) |
| **Full Closed-Loop Simulation** (World + Noise + Tracker) | **~159 FPS** | ≥ 30 FPS | **PASS** (5.3× headroom) |
| **QThread Worker Throughput** (Tracking + UI Signals) | **~140 FPS** | ≥ 30 FPS | **PASS** (4.6× headroom) |
| **Mode B Video Ingestion Pipeline** (`VideoFileSource`) | **~129 FPS** | ≥ 30 FPS | **PASS** (4.3× headroom) |
| **Centroid Sub-Pixel Accuracy** (Zero Noise) | **0.707 px** | — | **PASS** (Theoretical floor) |
| **Centroid Sub-Pixel Accuracy** (σ=15 Noise, 64×64 ROI) | **0.755 px** | — | **PASS** (Robust CoG) |
| **Closed-Loop Figure-8 Tracking RMSE** (Phase 2 Sim) | **2.51 px** | ≤ 10 px | **PASS** (75% error margin) |
| **Full Worker TRACKING-State RMSE** (Phase 3 Loop) | **8.31 px** | ≤ 10 px | **PASS** (≤ 1.09 mrad spec) |
| **Mode B Ingestion Tracking RMSE** vs GT MP4 | **1.430 px** | ≤ 10 px | **PASS** (85% error margin) |
| **Maximum Actuator Slew Rate** (Phase 2 & Phase 3) | **0.190°/s – 0.995°/s** | ≤ 5.0 °/s | **PASS** (Zero clamp violations) |
| **Lock Retention Rate** (Stable Target) | **100.0%** | ≥ 95.0% | **PASS** (Zero unexpected drops) |
| **Target Loss Rate** | **< 5.0%** | < 5.0% | **PASS** (Compliant) |
| **Acquisition Latency** (Initial Target Lock) | **< 1.0 s** | ≤ 2.0 s | **PASS** (2× faster than spec) |
| **Re-acquisition Latency** (Post 30-Frame Occlusion) | **< 0.5 s** | ≤ 1.0 s | **PASS** (2× faster than spec) |

---

## Directory Structure

```
BeaconLock/
├── app/
│   ├── __init__.py
│   ├── orchestrator.py         # Dedicated TrackingWorker QThread: 13-stage execution pipeline
│   └── state_machine.py        # 5-state FSM with convergence-aware miss accounting
│
├── configs/                    # Simulation profile presets and stress scenarios
│
├── control/
│   ├── __init__.py
│   └── pid_controller.py       # Slew-clamped PD controller, 0.05° deadband, ±5.0 °/s saturation
│
├── evaluation/
│   ├── __init__.py
│   ├── logs/                   # Auto-generated CSV telemetry logs (session_YYYYMMDD_HHMMSS.csv)
│   ├── reports/                # Auto-generated ReportLab PDF compliance audits
│   ├── logger.py               # Thread-safe 11-column telemetry logger with ring-buffer flush
│   └── report_generator.py     # ISRO benchmark audit engine with pass/fail threshold matrix
│
├── ingestion/                  # Sole architectural bridge between world and perception
│   ├── __init__.py
│   ├── frame_source.py         # Abstract FrameSource ABC & Mode B VideoFileSource
│   └── sim_source.py           # Concrete Mode A SimulatorSource bridging simulation/
│
├── packaging/
│   └── BeaconLock.spec         # Lean single-file PyInstaller build specification
│
├── perception/
│   ├── __init__.py
│   ├── centroid.py             # Radiometric background estimation & sub-pixel CoG
│   └── kalman_tracker.py       # 6-state constant-acceleration KF with confidence-adaptive R
│
├── simulation/
│   ├── __init__.py
│   ├── camera.py               # Virtual pan-tilt camera (640×480 FPA, rate-clamped gimbal)
│   ├── disturbances.py         # Gaussian, Poisson, S&P noise, platform jitter, atmosphere
│   ├── motion_models.py        # Straight-line, Circular, Lemniscate Figure-8, Random Walk
│   ├── scene.py                # 2000×2000 px world coordinate space canvas generator
│   └── target.py               # 5–20 px beacon spot rendering & programmable occlusion
│
├── tests/
│   ├── test_phase1_headless.py # Phase 1: Mathematical core verification (31 tests)
│   ├── test_phase2_sim.py      # Phase 2: World simulation & disturbance engine (46 tests)
│   ├── test_phase3_ui.py       # Phase 3: FSM, CSV logger, PDF generator & QThread (33 tests)
│   └── test_phase4_mode_b.py   # Phase 4: Mode B evaluator MP4 video ingestion (17 tests)
│
├── ui/
│   ├── assets/
│   │   ├── icon.ico            # Vector-rendered crosshair icon (Windows Taskbar hook)
│   │   └── icon.png            # 64×64 RGBA optical reticle asset
│   ├── dashboard.py            # PySide6 3-panel command dashboard (#0b0f19 obsidian theme)
│   └── plots_panel.py          # PyQtGraph real-time curves with non-clipping threshold lines
│
├── main.py                     # Primary entry point: GUI launcher & --headless-check runner
├── requirements.txt            # Pinned dependencies for Python 3.10–3.13 environments
└── README.md                   # System documentation and engineering specifications
```

---

## Installation & Execution

### Option 1: Quick Evaluation (.EXE — Zero Dependencies)

For standalone evaluation on Windows systems without a Python environment:

1. Download `BeaconLock.exe` from the repository **Releases** tab (or locate `dist/BeaconLock.exe` in the packaged build).
2. Double-click `BeaconLock.exe` to launch the aerospace cockpit.
3. The executable is a self-contained ~121 MB single-file binary with all graphics, computer vision, and reporting libraries bundled.

To run automated verification via command prompt:
```powershell
BeaconLock.exe --headless-check
```

---

### Option 2: Developer Setup (Python 3.10 – 3.13)

#### Prerequisites
- Windows 10/11 (or Linux with X11)
- Python 3.10, 3.11, 3.12, or 3.13 (64-bit)
- `pip` package manager

#### Setup Commands

```powershell
# 1. Clone repository
git clone https://github.com/AlphaTrion/BeaconLock.git
cd BeaconLock

# 2. Install pinned dependencies
py -3 -m pip install -r requirements.txt

# 3. Execute full headless integration verification
py -3 -X utf8 main.py --headless-check

# 4. Launch interactive desktop telemetry cockpit
py -3 main.py
```

---

## Operational Guide

### Mode A: Closed-Loop Synthetic Simulation

1. Launch `py -3 main.py` or `BeaconLock.exe`.
2. Keep **Mode A — Synthetic Simulator** selected in the left panel.
3. Select desired target kinematic trajectory:
   - `straight_line`: Constant velocity with boundary reflection.
   - `circular`: Orbital motion around configurable radius.
   - `figure_8`: Lemniscate of Bernoulli testing continuous acceleration vector transitions.
   - `random_walk`: Ornstein–Uhlenbeck smooth bounded stochastic drift.
4. Adjust disturbance parameters dynamically via sliders:
   - **Noise σ**: Injects zero-mean Gaussian image noise up to 20 px intensity.
   - **Jitter**: Induces high-frequency platform vibrations up to ±20 px/frame.
   - **Atmosphere**: Toggles between `clear`, `haze`, `fog` (Koschmieder attenuation), `rain`, and `low_light`.
5. Click **▶ START** to engage closed-loop tracking.
6. Click **⊘ Inject Blind Spot (30f)** to blackout the beacon for 1 second and evaluate Kalman coasting and re-acquisition latency.
7. Click **📄 Export Audit PDF** to compile the formal compliance report.

---

### Mode B: Evaluator Video Ingestion (Benchmark-2 Compliance)

1. Select **Mode B — Evaluator MP4** in the left panel.
2. Click **📂 Browse MP4…** and select any standard 30-FPS MP4, AVI, or MKV recording.
3. Click **▶ START**.
4. The pipeline switches frame sources to `VideoFileSource`, cleanly bypasses virtual camera actuation commands, and processes the raw video directly through the CoG + Kalman tracking core.
5. Centroid coordinates, Kalman predictions, and tracking errors are calculated and displayed on the live feed in real time.

---

## Automated Verification & Test Matrix

The codebase includes an exhaustive test suite covering unit mathematical algorithms, disturbance physics, UI threading, and external video ingestion. All 143 assertions pass with 100% success rate:

```powershell
# Run Phase 1: Mathematical core (CoG, Kalman, PD kinematics)
py -3 -X utf8 tests/test_phase1_headless.py

# Run Phase 2: Simulation canvas, motion trajectories & disturbances
py -3 -X utf8 tests/test_phase2_sim.py

# Run Phase 3: State machine, CSV logging, PDF generation & QThread loop
py -3 -X utf8 tests/test_phase3_ui.py

# Run Phase 4: Mode B external MP4 ingestion pipeline
py -3 -X utf8 tests/test_phase4_mode_b.py

# Run Full Headless End-to-End System Check
py -3 -X utf8 main.py --headless-check
```

### Test Suite Summary

| Test Module | Coverage Scope | Assertions | Result |
|---|---|:---:|:---:|
| `tests/test_phase1_headless.py` | Radiometric CoG, 6-state Kalman Filter, Slew-limited PD controller | 31 | **31 / 31 PASS** |
| `tests/test_phase2_sim.py` | 4 motion models, noise models, camera PTZ, closed-loop tracking | 46 | **46 / 46 PASS** |
| `tests/test_phase3_ui.py` | 5-state FSM transitions, CSV logging, ReportLab PDF generation | 33 | **33 / 33 PASS** |
| `tests/test_phase4_mode_b.py` | `VideoFileSource` ingestion, resolution rescaling, pipeline FPS | 17 | **17 / 17 PASS** |
| `main.py --headless-check` | End-to-end integration check across worker, FSM, and report engine | 16 | **16 / 16 PASS** |
| **Total Cumulative Assertions** | **Complete System Verification** | **143** | **143 / 143 PASS (100%)** |

---

## Audit Telemetry & PDF Compliance Reports

BeaconLock guarantees reproducible, audit-grade verification for academic and defense evaluations:

1. **Continuous Telemetry Logging**:
   Every processed frame is streamed to `evaluation/logs/session_YYYYMMDD_HHMMSS.csv` with the following 11-column schema:
   ```csv
   frame_id, timestamp_s, input_mode, centroid_x, centroid_y, error_px, error_mrad, pan_deg_s, tilt_deg_s, fps, lock_state
   ```
2. **Automated PDF Audit Generation**:
   Clicking **Export Audit PDF** generates an ISRO-formatted benchmark summary at:
   ```
   evaluation/reports/audit_YYYYMMDD_HHMMSS.pdf
   ```
   The report compiles:
   - **Session Configuration**: Trajectory model, atmospheric profile, duration, total frames.
   - **Performance Compliance Matrix**: Live statistical computation of RMSE (px & mrad), mean throughput (FPS), maximum slew rate (°/s), and lock retention (%) against hard ISRO ceilings.
   - **Honest Failure Boundary Documentation**: Empirical boundary declarations detailing environmental extremes where sub-pixel optical tracking diverges.

---

## Algorithmic Reference

### 1. Radiometric Background-Subtracted Center of Gravity (CoG)

To eliminate false detections caused by low-frequency ambient illumination and high-frequency atmospheric haze, the background intensity threshold $I_{th}$ is dynamically estimated from the outer border annulus of the search region:

$$
I_{th} = \mu_{bg} + 3\sigma_{bg}
$$

The intensity-weighted sub-pixel centroid coordinates $(x_{cog}, y_{cog})$ are computed as:

$$
x_{cog} = \frac{\sum_{i \in \Omega} \max(I_i - I_{th},\, 0) \cdot x_i}{\sum_{i \in \Omega} \max(I_i - I_{th},\, 0)}, \quad
y_{cog} = \frac{\sum_{i \in \Omega} \max(I_i - I_{th},\, 0) \cdot y_i}{\sum_{i \in \Omega} \max(I_i - I_{th},\, 0)}
$$

---

### 2. 6-State Constant-Acceleration Kalman Filter

The optical target state vector incorporates 2D position, velocity, and acceleration:

$$
\mathbf{x} = \begin{bmatrix} x & y & v_x & v_y & a_x & a_y \end{bmatrix}^T
$$

The state transition matrix $\mathbf{F}$ for frame interval $\Delta t = \frac{1}{30}\text{ s}$ is defined as:

$$
\mathbf{F} = \begin{bmatrix}
1 & 0 & \Delta t & 0 & \frac{1}{2}\Delta t^2 & 0 \\
0 & 1 & 0 & \Delta t & 0 & \frac{1}{2}\Delta t^2 \\
0 & 0 & 1 & 0 & \Delta t & 0 \\
0 & 0 & 0 & 1 & 0 & \Delta t \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

Measurement noise covariance $\mathbf{R}$ scales dynamically based on detection confidence $c \in [0, 1]$:

$$
\mathbf{R}(c) = \frac{\mathbf{R}_{base}}{\max(c,\, 0.01)}
$$

During target occlusions ($c = 0$), the filter bypasses measurement updates and coasts on internal kinematic extrapolation, preventing track divergence.

---

### 3. Slew-Limited Proportional-Derivative (PD) Actuation

Pixel tracking errors are mapped to gimbal angular coordinates using the optical sensor scale factor ($0.00625\text{ deg/px}$):

$$
e_{deg}(t) = \bigl(\mathbf{x}_{pred}(t) - \mathbf{x}_{center}\bigr) \times 0.00625\text{ deg/px}
$$

$$
u(t) = K_p \cdot e_{deg}(t) + K_d \cdot \frac{d e_{deg}(t)}{dt}
$$

Actuator rate commands are clamped to ISRO's hard mechanical velocity envelope:

$$
u_{clamped}(t) = \begin{cases}
0, & \text{if } |e_{deg}| \le 0.05^\circ \text{ (Deadband)} \\
\text{clamp}\bigl(u(t),\, -5.0^\circ/\text{s},\, +5.0^\circ/\text{s}\bigr), & \text{otherwise}
\end{cases}
$$

---

## Competition Credentials

- **Challenge**: Smart India Hackathon (SIH) 2026
- **Problem Statement ID**: PS-26169
- **Issuing Organisation**: Department of Space / Indian Space Research Organisation (ISRO)
- **Team**: AlphaTrion (Team ID: 176697)

---

<div align="center">
<sub>BeaconLock v1.0.0 — Production Release | Department of Space / ISRO | Smart India Hackathon 2026</sub>
</div>
