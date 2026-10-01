# PS 26169 — MASTER CONSOLIDATED BLUEPRINT
### (Merged from Claude + Perplexity + ChatGPT research — contradictions resolved — ready for GPT to build the PPT from)

---

## 0. HOW TO USE THIS DOCUMENT
This is the single source of truth. It merges independent AI research passes (Claude, Perplexity, ChatGPT, and a later Gemini pass) on ISRO SIH 2026 PS-26169, resolves architectural contradictions between them, and keeps only the highest-value, verified content from each. Build the PPT and any further planning directly from this document — do not re-merge the raw reports again.

**Revision note:** A later Gemini research pass was reviewed and added genuine value in three places (6-state Kalman/EKF, optional sub-pixel Gaussian centroid refinement, ROI-cropping for speed) — these are now integrated below. However, Gemini's stated evaluation weightage ("Technical Complexity 40%, Prototype Functionality 30%, Presentation 20%, UX 10%") and its "ISRO Target Benchmark" performance numbers (Acquisition <1.5s, RMSE <0.5mrad, Lock retention >98.5%, FPS≥60) were REJECTED — these do not match the official PDF (§4 and §3 below are the verified source of truth for evaluation weightage and performance targets; treat any tighter numbers only as an internal stretch goal, never as the official requirement). Its OpenGL/3D/C++ rendering stack suggestion was also rejected — §2's Python-first decision stands.

---

## 1. THE PROBLEM — PLAIN UNDERSTANDING

Free Space Optical Communication (FSOC) sends data via laser beam instead of radio. The beam is extremely narrow — even a fraction-of-a-degree pointing error breaks the link. Before fine laser pointing can lock in, a **coarse alignment stage** must:
1. Search the scene
2. Detect the remote beacon
3. Estimate its position
4. Move a pan-tilt camera to keep it centered
5. Hand over to fine-pointing hardware (out of scope for this project)

**You are building a pure-software Digital Twin / simulator of this coarse alignment stage** — no real hardware. ISRO wants this because real cameras + gimbals + optics cost lakhs; a software platform lets engineers develop and validate tracking algorithms cheaply.

**Core mental model:** A CCTV camera that can pan/tilt, hunting for a small glowing dot moving on a big screen, keeping it centered despite fog/noise/vibration — entirely in code.

---

## 2. RESOLVED ARCHITECTURAL DECISION: PYTHON-FIRST, NOT UNITY-FIRST

Three sources disagreed here. Resolution and reasoning:

**Decision: Python + OpenCV + PySide6/PyQt5 as the core. NOT Unity.**

Reason: Benchmark-2 (30% of total marks) requires your detection/tracking pipeline to process an **externally-supplied .mp4 file directly**, bypassing your own virtual camera entirely. If the simulator is built in Unity, the perception pipeline still has to exist independently in Python/OpenCV to handle this mode — meaning a Unity-first approach forces you to build and maintain **two separate perception stacks**. That's wasted effort and added risk for a hackathon timeline, and visual polish is not in the marking scheme (which rewards RMSE, FPS, lock retention, architecture, and algorithm choices — not graphics).

**Unity/Godot is optional and comes LAST (Phase 8)** — only if your team has spare time and a strong Unity developer, purely as a cosmetic front-end layered on top of the already-working Python core. Never let the core algorithm depend on it.

---

## 3. EXACT SPECIFICATIONS (FROM OFFICIAL PDF — NON-NEGOTIABLE)

### Camera
| Parameter | Value |
|---|---|
| Scene/screen size | ≥ 2000×2000 px |
| Camera type | Monochrome FPA (colour optional) |
| Camera resolution | 640×480 px (default, user-configurable) |
| Camera FOV | 4°×3° (default) |
| Camera update rate | ≥ 30 Hz |
| Initial camera position | Centre of scene |

### Target / Beacon
| Parameter | Value |
|---|---|
| Type | Beacon spot |
| Count | 1 mandatory, multiple optional |
| Shape | User-defined (default square) |
| Size | 5–20 × 5–20 px (default 10×10) |
| Initial location | User-defined (default random) |
| Motion (mandatory, all 4) | Straight line, Circular, Figure-8, Random |
| Motion (optional) | Spiral, Sinusoidal, custom |

### Camera Motion Constraints
| Parameter | Value |
|---|---|
| Max pan speed | 5–10 °/s (default 5°/s) |
| Max tilt speed | 5–10 °/s (default 5°/s) |
| Camera/control update interval | ≥ 20 Hz |

### Hard Performance Targets (graded directly)
| Metric | Requirement | Internal margin target |
|---|---|---|
| Acquisition time | ≤ 2 s | aim ≤ 1.0 s |
| Tracking error | ≤ 10 px | aim RMSE ≤ 4 px |
| Target loss rate | < 5% | aim ≤ 2% |
| Re-acquisition time | ≤ 1 s | aim ≤ 0.5 s |
| Processing speed | ≥ 20 FPS | aim ≥ 40 FPS |
| Camera update | ≥ 30 Hz | — |

### Disturbances (all must be independently configurable)
| Type | Detail |
|---|---|
| Noise | Salt & Pepper (~10%), Gaussian, Poisson — user selectable, one or more; max std dev 20px |
| Camera jitter | ± 20 px/frame (a SEPARATE concept from platform motion — see §6) |
| Atmospheric | Clear, Haze, Fog, Rain, Low light — contrast/brightness reduction |
| Platform motion | ± 20 px/frame max; linear mandatory, circular/random/spiral optional |

---

## 4. EVALUATION — EXACT WEIGHTAGE

| Stage | % | What's checked |
|---|---|---|
| Functional Verification | 20% | 10-15 min live demo — all mandatory features, GUI quality, operational success |
| Benchmark Performance-1 | 30% | Judge-given scenarios on YOUR simulator; centroiding error log; auto-generated performance report |
| **Benchmark Performance-2** | **30%** | **Judges supply .mp4 (30fps, full screen, noise+beacon). Your software bypasses its own PTZ camera and processes the video directly. Centroiding error compared to predefined ground truth. RMSE, acquisition/re-acquisition time, lock retention, FPS all scored.** |
| Technical Evaluation | 20% | Architecture, algorithm selection, AI/CV methods, innovation, documentation, Q&A |

**⚠ THE #1 RISK FOR ALL TEAMS: Benchmark-2.** Most teams couple their tracker tightly to their virtual camera and cannot process an external video. Your `TrackingPipeline` class must accept ANY frame source (simulator OR video file) with zero dependency on how the frame was generated.

---

## 5. FINAL ARCHITECTURE — MERGED BEST-OF-THREE

```
Input Manager (simulator frames OR evaluator .mp4 — same interface)
        ↓
Disturbance Engine (noise / jitter / atmospheric — only active in sim mode)
        ↓
Preprocessing (grayscale, light denoise, adaptive background estimate)
        ↓
Perception Layer
   ├── Classical adaptive blob/centroid detector (PRIMARY — fast, reliable, explainable)
   └── Optional YOLOv8-nano detector (SECONDARY — robust re-acquisition, difficult conditions)
        ↓
Intensity-Weighted Centroid Estimator (sub-pixel accuracy — NOT bounding-box centre)
        ↓
Kalman Filter (state = [x, y, vx, vy]; predicts through occlusion/noise)
        ↓
State Machine (SEARCHING → ACQUIRING → TRACKING → LOST → REACQUIRING)
        ↓
Pixel-to-Angle Conversion (using FOV 4°×3° over 640×480)
        ↓
PID/PD Controller (rate-limited to 5-10°/s, anti-windup, deadband)
        ↓
Virtual Camera (pan/tilt viewport over 2000×2000 scene; respects speed limits — NOT a teleporting crop)
        ↓
Metrics / GUI / CSV+JSON Log / Auto PDF Report
```

**PERFORMANCE OPTIMIZATION — ROI cropping (recommended, low-risk, real FPS gain):**
Once the beacon is first acquired (or once the Kalman filter has a confident predicted position), don't run detection/centroiding on the full 640×480 frame every frame. Instead crop a small sub-window (e.g., 64×64px) centred on the last-known/predicted position and run detection only inside that crop. This is a standard tracking optimization — full-frame search only during SEARCHING/LOST states; cropped-ROI processing during TRACKING state. Meaningfully reduces per-frame processing time and gives comfortable headroom above the ≥20 FPS requirement. Low implementation risk — just index the frame array with the crop bounds before running the centroid/detector logic, then add the crop offset back to get full-frame coordinates.

**Why classical detector is PRIMARY, not YOLO:** The target is tiny (5-20px), bright, and monochrome — exactly what classical blob/centroid detection is built for. YOLO adds latency, can lose very small targets to bounding-box quantization, and needs training data that may not match the evaluator's unknown video. Use YOLO as a **periodic robustness layer** (every 5-10 frames, or only when classical detector confidence is low) — not the main-loop detector. This is a stronger technical story: "we chose the right tool for a small, controlled optical target rather than defaulting to deep learning everywhere."

---

## 6. KEY TECHNICAL CONCEPTS ALL THREE SOURCES AGREE ARE CRITICAL

### A. Centroiding (not bounding-box centre)
ISRO's evaluation explicitly measures "centroiding error." Use intensity-weighted centre of mass:
```python
cx = sum(x_i * intensity_i) / sum(intensity_i)   # for all pixels in beacon ROI
cy = sum(y_i * intensity_i) / sum(intensity_i)
```
Subtract local background before weighting (adaptive threshold: `I_th = background_mean + 3*background_std`). This is far more accurate than bbox centre, especially under blur/noise.

**OPTIONAL REFINEMENT (adds sub-pixel accuracy, not mandatory — add only if core pipeline is stable and time remains):**
After computing the CoG centroid above, refine it further by fitting a 2D Gaussian paraboloid over a small 5×5 window centred at the CoG estimate:
```
ln I(u,v) = a·u² + b·u·v + c·v² + d·u + e·v + f
```
Solving this linear system for the peak of the paraboloid gives a refined sub-pixel centroid (down to ~0.05px accuracy vs ~0.2-0.5px for CoG alone). This is a real technique used in optical star-tracker/centroiding systems and is a strong technical-evaluation talking point, but it is pure polish on top of an already-working CoG centroid — build and validate CoG first, add this only as a stretch goal. Do not let this block the core pipeline.

### B. Three coordinate systems — define explicitly, never mix them
1. **Screen/world coordinates** — the 2000×2000 canvas
2. **Camera/image coordinates** — the 640×480 frame, (320,240) = centre
3. **Angular coordinates** — degrees, using FOV 4°×3°

Pixel-to-angle conversion:
```
deg_per_px_horizontal = FOV_H / RES_W   # 4/640 = 0.00625 °/px
deg_per_px_vertical   = FOV_V / RES_H   # 3/480 = 0.00625 °/px
```
So the ≤10px tracking-error spec = **≈0.0625° ≈ 1.09 milliradians** of angular error. State this explicitly in your report/pitch — it shows you understand this is a real pointing-accuracy problem, not just "keep dot near centre."

### C. Camera jitter ≠ Platform motion (commonly confused — keep separate)
- **Camera jitter**: rapid unwanted image displacement (sensor/mount vibration) — affects the image directly, ±20px/frame
- **Platform motion**: physical motion of the observing platform (e.g., satellite drift) — changes apparent target position, may need control compensation, ±20px/frame
Give them separate UI controls and separate disturbance models. Judges may test them independently.

### D. Adaptive thresholding (not fixed threshold)
Atmospheric disturbance reduces contrast/brightness, so a fixed `gray > 200` threshold will fail under Fog/Low-light. Use:
```python
threshold = max(fixed_min_threshold, background_mean + k * background_std)
```

### E. Candidate scoring (for multi-target / distractor rejection)
```
score = w1*normalized_intensity + w2*circularity + w3*blink_consistency + w4*proximity_to_prediction
circularity = 4π*Area / Perimeter²   # 1.0 = perfect circle
```
Optional: encode a blink signature (e.g., ON-ON-OFF-ON-OFF-OFF) on the true beacon to distinguish it from bright distractors — a genuinely novel touch few teams will include.

### F. PD may beat full PID
For a beacon tracker, a PD controller (no integral term) can be simpler and less prone to windup than full PID. Start Kp-only, tune until oscillation appears, back off slightly, add Kd to damp. Add Ki only if persistent steady-state offset remains. Always include: output saturation (max 5-10°/s), anti-windup, deadband (~0.2°) to prevent jitter-hunting near lock.

### G. Kalman filter — survives occlusion, smooths jitter, speeds re-acquisition

**UPGRADED STATE MODEL (recommended): 6-state instead of basic 4-state.**
State: `[x, y, vx, vy, ax, ay]` — position, velocity, AND acceleration. This is an upgrade over a basic 4-state `[x,y,vx,vy]` filter because the PS mandates Figure-8 and Random motion modes, both of which involve continuously changing velocity (non-constant-velocity motion). A 4-state filter assumes near-constant velocity between updates and will lag/overshoot on curved or erratic trajectories; the 6-state model tracks acceleration directly and predicts through turns and direction changes far more accurately. This is standard practice in real optical/radar tracking systems (e.g., satellite PAT, star trackers) and is a strong, defensible choice for the Technical Evaluation stage.

```python
# 6-state Kalman: x = [x, y, vx, vy, ax, ay]^T
# Constant-acceleration motion model, dt = time between frames
F = [[1, 0, dt, 0, 0.5*dt**2, 0],
     [0, 1, 0, dt, 0, 0.5*dt**2],
     [0, 0, 1, 0, dt, 0],
     [0, 0, 0, 1, 0, dt],
     [0, 0, 0, 0, 1, 0],
     [0, 0, 0, 0, 0, 1]]
H = [[1,0,0,0,0,0],   # we only measure x, y
     [0,1,0,0,0,0]]
```

Runs `predict()` every frame regardless of detection; `update()` only when a valid centroid is found (measurement update is skipped/bypassed during occlusion — the filter "coasts" forward on its motion model). During LOST state, keep predicting — point the camera toward the Kalman-predicted position rather than freezing, which directly helps hit the ≤1s re-acquisition target.

*(A simpler 4-state `[x,y,vx,vy]` filter is an acceptable fallback if the team is short on time — it still meets the spec, just with slightly less accurate prediction on curved trajectories. Note in the report which one was used and why.)*

### H. State machine — full 5-state version (merged)
```
SEARCHING     → full-frame classical scan (+ raster/spiral scan pattern if target unseen for a while)
   ↓ candidate found
ACQUIRING     → validate (confidence + N consecutive stable frames); start acquisition timer (≤2s)
   ↓ confirmed & error ≤10px
TRACKING      → classical detector/optical-flow + Kalman every frame; log centroiding error
   ↓ confidence drops / no detection for 5 frames
LOST          → Kalman predict-only; point camera at predicted position; start re-acq timer (≤1s)
   ↓ timeout exceeded
REACQUIRING / AI_REACQUIRE → wider search, optionally invoke YOLO detector here
   ↓ found again
TRACKING (resume)
```
Define your **lock condition** explicitly and be ready to state it to judges, e.g.: `centroid_error ≤ 10px AND detected-or-validly-predicted for N consecutive frames`.

---

## 7. BENCHMARK-2 — THE DECOUPLED VIDEO PIPELINE (MOST IMPORTANT ENGINEERING DECISION)

```python
class FrameSource:
    def read(self): raise NotImplementedError

class SimulatorSource(FrameSource): ...
class VideoFileSource(FrameSource): ...   # cv2.VideoCapture wrapper

class TrackingPipeline:
    # Must NEVER import from Scene/Camera/Simulator modules.
    # Accepts only: a numpy frame array.
    # Returns: centroid, error_px, error_deg, state, confidence.
    def process_frame(self, frame: np.ndarray) -> TrackResult: ...
```
Both `simulator.py` (Mode A) and `video_processor.py` (Mode B, Benchmark-2) call the exact same `TrackingPipeline` instance. This single design decision protects 30% of your marks.

**Important nuance from research**: if the evaluator's video resolution ≠ 640×480, do NOT silently resize without recording the scale factor — centroid error must be reported in the coordinate system the evaluator expects. Log both native-pixel error and normalized error.

---

## 8. TECH STACK (FINAL — RESOLVED)

| Layer | Choice | Why |
|---|---|---|
| Language | Python 3.11+ | Best CV/ML ecosystem, fast iteration |
| Simulation/rendering | pygame or OpenCV canvas | 2D world, fast, no unnecessary complexity |
| Computer vision | OpenCV | blob detection, CSRT/optical flow, VideoCapture for B2 |
| Noise generation | scikit-image (`random_noise`) | covers S&P/Gaussian/Poisson in one API |
| Tracking/filtering | filterpy (Kalman), Lucas-Kanade optical flow (OpenCV) | prediction + cheap frame-to-frame tracking |
| Deep learning (optional layer) | Ultralytics YOLOv8-nano | small, fast, periodic re-acquisition only |
| Model packaging | ONNX Runtime (export from YOLO) | avoids full PyTorch dependency in the packaged .exe — lighter, fewer PyInstaller headaches |
| Controller | simple-pid or custom PD | rate-limited pan/tilt output |
| GUI | PySide6 (preferred) or PyQt5 | professional desktop UI |
| Live plots | PyQtGraph | faster than matplotlib for real-time data |
| Logging | pandas + Python `logging` | CSV/JSON frame logs |
| Reports | ReportLab (PDF) or Jinja2+HTML | auto-generated performance report |
| Packaging | PyInstaller (`--onedir`, not `--onefile` with torch/onnx) | standalone .exe |
| Config | YAML | scenario presets (easy/medium/difficult) |
| Optional visual layer (Phase 8 only) | Unity or Godot | cosmetic front-end AFTER core Python pipeline works — never load-bearing |

---

## 9. PROJECT FOLDER STRUCTURE

```
fsoc_tracker/
├── app/                  main.py, config.py, state_machine.py
├── simulation/           scene.py, target.py, camera.py, motion_models.py, disturbances.py
├── perception/           classical_detector.py, yolo_detector.py, optical_flow_tracker.py,
│                         kalman_tracker.py, centroid.py, data_association.py
├── control/               pid_controller.py, angle_controller.py, search_controller.py, camera_limits.py
├── io/                    frame_source.py (SimulatorSource / VideoFileSource abstraction)
├── evaluation/            metrics.py, logger.py, plots.py, report_generator.py
├── ui/                    dashboard.py, plots_panel.py, replay_viewer.py
├── models/                beacon_detector.onnx
├── configs/               easy.yaml, medium.yaml, difficult.yaml
├── training/              synthetic_data_gen.py, dataset.yaml, train.py
└── tests/
```

---

## 10. TEST MATRIX (USE DIRECTLY IN REPORT + PPT)

| ID | Motion | Noise | Atmospheric | Platform Motion | Target Size | Purpose |
|---|---|---|---|---|---|---|
| T1 | Straight | None | Clear | Linear | 10px | Baseline |
| T2 | Circular | Gaussian | Clear | Linear | 10px | Tracking stability |
| T3 | Figure-8 | Salt & Pepper | Haze | Linear | 5px | Small-target robustness |
| T4 | Random | Poisson | Low light | Random | 20px | Detection robustness |
| T5 | Straight | Combined | Fog | Circular | 10px | Reacquisition |
| T6 | Figure-8 | Combined | Rain | Spiral | 5px | Stress test |

Report format per scenario: Avg FPS | Acquisition time | Mean error | Max error | Lock retention | Reacquisition time. **Never fabricate these numbers — generate them by actually running the app.**

---

## 11. PERFORMANCE LOG SCHEMA (EXACT)

**Per-frame CSV columns:**
`frame_id, timestamp_s, input_width, input_height, tracker_state, detected, confidence, centroid_x, centroid_y, predicted_x, predicted_y, error_x_px, error_y_px, euclidean_error_px, error_total_deg, cam_pan_deg, cam_tilt_deg, processing_ms, fps`

**Auto-generated run summary (JSON):**
`scenario, duration_s, average_fps, min_fps, max_fps, acquisition_time_s, avg_reacquisition_s, rmse_px, rmse_deg, avg_error_px, max_error_px, p95_error_px, lock_retention_percent, target_loss_count, target_loss_rate, avg_processing_ms, max_processing_ms`

---

## 12. INNOVATION IDEAS WORTH PRESENTING (MERGED, PRIORITIZED)

1. **Disturbance-aware Kalman filtering** — increase measurement noise covariance (R) dynamically when detector confidence drops or noise level rises, so the filter leans more on prediction under poor conditions. (Technically strongest — genuinely improves robustness, easy to demonstrate with a plot.)
2. **Adaptive detector selection** — high SNR → simple threshold centroiding; medium SNR → matched filter + Kalman; low SNR → invoke YOLO / widen search. (Efficient, explainable, ties directly to FPS requirement.)
3. **Intelligent spiral/raster search on loss** — instead of a blind full-frame rescan, search outward from last-known/predicted position first. (Directly improves re-acquisition time metric.)
4. **Blink-pattern beacon signature** — optional coded ON/OFF pattern to reject distractors in multi-target scenarios. (Novel, cheap to implement, good Q&A talking point.)
5. **Adaptive FOV / digital zoom concept** — wide search mode → narrower effective FOV once locked, mirroring how real coarse→fine PAT systems operate. (Good conceptual bridge to "fine alignment is the next stage" narrative even though out of scope.)

Do NOT lead with cosmetic ideas (3D visuals, mission-console skin) — they don't map to marking criteria. Mention GUI polish only after the above.

---

## 13. LIKELY JURY QUESTIONS + STRONG ANSWERS

| Question | Answer |
|---|---|
| Why not YOLO for everything? | Beacon is tiny, bright, monochrome — classical adaptive centroiding is faster, sub-pixel accurate, and doesn't depend on training data matching the evaluator's unknown video. YOLO is used selectively for robust re-acquisition. |
| Why Kalman filter? | Predicts target position through noise/occlusion, smooths jitter, and directly improves our re-acquisition time against the ≤1s spec. |
| Why PID/PD and not something fancier (MPC)? | Well-tuned rate-limited PD is easier to validate, explain, and tune within hackathon time, and meets all spec constraints (5-10°/s, ≥20Hz) reliably. MPC is future work. |
| How do you handle the evaluator's unknown .mp4? | Our perception pipeline is fully decoupled from the simulator — it operates on any frame source through a common interface, so the same code that tracks in our simulation processes the benchmark video with zero modification. |
| What does "tracking error ≤10px" mean physically? | With our default 4°×3° FOV over 640×480px, 10px error corresponds to ≈0.0625° (≈1.09 mrad) angular pointing error — a genuinely meaningful coarse-alignment accuracy. |
| Is this AI-assisted enough? | The PDF says "AI methods if used" — ours is a hybrid system: a deterministic, explainable centroiding baseline for reliability and speed, with an optional learned detector for difficult conditions/re-acquisition. This is a stronger and more defensible architecture than an all-neural pipeline for a small controlled optical target. |

---

## 14. ONE-PARAGRAPH ARCHITECTURE PITCH (memorize/adapt this for Q&A)

> "Our system is a closed-loop coarse PAT simulator that separates the virtual scene, camera model, disturbance model, perception, tracking, control, and evaluation layers. The beacon is detected using adaptive bright-spot segmentation and intensity-weighted centroiding, because the benchmark target is a small optical spot where centroid accuracy matters more than generic bounding-box detection. A Kalman filter predicts target motion during noisy or missing measurements, while a rate-limited PD controller commands the virtual pan-tilt camera within the official 5-10°/s constraints. For difficult conditions, an optional YOLOv8-nano detector supports candidate validation and re-acquisition. Critically, the same processing pipeline accepts both our generated simulator frames and evaluator-provided 30-FPS MP4 files with zero code changes. Every frame is logged, and the application automatically computes centroid RMSE, acquisition time, re-acquisition time, lock retention, target loss rate, FPS, and processing latency into an auto-generated report."

---

## 15. 6-WEEK BUILD ORDER (unchanged, validated by all 3 sources)

**Week 1** — Scene generator + virtual camera (manual control), straight-line beacon motion. Prove coordinate system works.
**Week 2** — All 4 mandatory motion modes + full disturbance engine (noise, jitter, atmospheric, platform motion) with config controls.
**Week 3** — Classical detector + centroid estimator + CSRT/optical-flow + Kalman + PD controller + state machine. Working end-to-end system (~75-80% score tier).
**Week 4** — Synthetic dataset generation (from your own scene, 3000-5000 images) → fine-tune YOLOv8-nano → export ONNX → integrate as periodic re-acquisition layer. Build `VideoFileSource` + decoupled `TrackingPipeline` for Benchmark-2.
**Week 5** — Full GUI (PySide6 + PyQtGraph), auto CSV/JSON logging, auto PDF report generation, test Benchmark-2 mode end-to-end on sample videos.
**Week 6** — Tune PID/Kalman gains to margin targets, PyInstaller packaging (test on clean machine), write technical report (10-15pg) + user manual, record demo video.

---

## 16. WHAT TO EXPLICITLY DOCUMENT AS LIMITATIONS (for the report/Q&A — shows maturity)

- This is a 2D angular-coordinate approximation, not full physical optical propagation
- Atmospheric disturbance is a configurable contrast/brightness model, not a true phase-screen turbulence simulation — call it a "disturbance approximation," don't overclaim physical accuracy
- Fine pointing stage is explicitly out of scope per the PDF
- No hardware-in-the-loop validation (future work)
- Camera jitter and platform motion models are simplified relative to real sensor/orbital dynamics

---

*End of consolidated blueprint. This document merges and resolves Claude, Perplexity, and ChatGPT's independent research passes on ISRO SIH 2026 PS-26169. Use this as the sole reference for building the PPT and remaining implementation plan.*
