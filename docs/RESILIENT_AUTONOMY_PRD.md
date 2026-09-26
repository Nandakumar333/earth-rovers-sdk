# PRD — Resilient Autonomous Exploration for Earth Rovers SDK

**Repository:** `frodobots-org/earth-rovers-sdk`  
**Proposed file:** `docs/RESILIENT_AUTONOMY_PRD.md`  
**Status:** Draft — implementation-ready baseline  
**Date:** 2026-09-26  
**Target branch:** feature/resilient-autonomy

---

## 1. Executive Summary

This PRD defines a production-oriented autonomy layer for the Frodobots Earth Rovers SDK.

The system is designed around a core constraint:

> The existing SDK is a remote rover communications and mission-control layer. It is not, by itself, a hard-real-time rover safety controller.

The proposed system therefore separates autonomy into two execution domains:

1. **Edge Rover Autonomy Agent**
   - Runs on compute physically close to the rover.
   - Owns safety-critical state estimation, local terrain understanding, local trajectory generation, command arbitration, and network-loss behavior.
   - Must remain safe when the remote operator, cloud service, or semantic AI process is unavailable.

2. **Remote Semantic Mission Layer**
   - Runs on a workstation, server, or GPU host.
   - Performs expensive visual reasoning, visual place recognition (VPR), topological-semantic memory, object search, next-best-view planning, mission reasoning, and global exploration.
   - Sends goals, route corridors, or bounded motion intents rather than becoming the rover's emergency-control loop.

The resulting architecture is:

```text
                           REMOTE / GPU HOST
┌───────────────────────────────────────────────────────────────────────────┐
│ Mission Manager                                                          │
│   ├─ Search objective                                                    │
│   ├─ Global exploration                                                 │
│   ├─ Semantic reasoning                                                 │
│   └─ Mission/checkpoint coordination                                    │
│                                                                           │
│ Semantic World Model                                                     │
│   ├─ VPR / place recognition                                             │
│   ├─ Topological graph                                                  │
│   ├─ Object memory                                                      │
│   ├─ Explored/unexplored coverage                                       │
│   └─ Global risk / information gain                                     │
└────────────────────────────────┬──────────────────────────────────────────┘
                                 │
                           LTE / Internet
                                 │
┌────────────────────────────────▼──────────────────────────────────────────┐
│ EDGE ROVER AUTONOMY AGENT                                                │
│                                                                           │
│ Camera / IMU / wheel telemetry                                           │
│          │                                                                │
│          ▼                                                                │
│ Time Synchronization + Sensor Health                                     │
│          │                                                                │
│          ▼                                                                │
│ State Estimator                                                          │
│          │                                                                │
│          ├──────────────► Local Terrain / Traversability                 │
│          │                     │                                         │
│          ▼                     ▼                                         │
│ Local World State ───────► Local MPPI / Trajectory Planner               │
│                                    │                                     │
│                                    ▼                                     │
│                           SAFETY COMMAND ARBITER                         │
│                           ├─ roll/pitch                                  │
│                           ├─ slip / stuck                                │
│                           ├─ command age                                 │
│                           ├─ sensor confidence                           │
│                           ├─ network health                              │
│                           └─ emergency stop                              │
│                                    │                                     │
│                                    ▼                                     │
│                              SDK /control                                │
└───────────────────────────────────────────────────────────────────────────┘
```

### Core product principle

**No remote AI component is safety-critical.**

A VPR model can fail. A detector can fail. Depth estimation can become uncertain. The LTE connection can disappear. The remote planner can crash.

In all of these cases, the rover must enter a defined safe state rather than continue executing stale motion commands.

---

# 2. Current Repository Baseline

The current public repository provides a Python/FastAPI/Hypercorn SDK with live video, telemetry, control, mission, checkpoint, intervention, and status functionality.

As of 2026-09-26, the repository documents:

- SDK v6.3.
- Python 3.9+ support.
- FastAPI + Hypercorn runtime.
- Browser/Playwright integration for rover sessions.
- `POST /control` accepting normalized `linear`, `angular`, and `lamp` values in `[-1, 1]` / `{0,1}`.
- Persistent rover command semantics: the rover continues executing the last command until a new command arrives.
- Recommended continuous motion command streaming at approximately 10 Hz.
- A control watchdog based on confirmed delivery, configurable with `CONTROL_WATCHDOG_S`.
- `GET /data` for latest telemetry, including battery, GPS, speed, vibration, accelerometer samples, gyroscope samples, magnetometer samples, and RPM samples.
- `GET /v2/screenshot`, `/v2/front`, `/v2/rear` for cached camera frames.
- `GET /feed` for MJPEG streaming at configurable FPS.
- `GET /status` exposing session, telemetry, and camera health.
- Mission, checkpoint, mission-history, and intervention APIs.
- A ROS2 bridge example exposing command, camera, GPS, IMU and battery data.

The repository also documents MINI+ hardware characteristics including a 4 km/h top speed, 18° maximum slope, front/rear camera streams, wide front-camera field of view, and an MPU6050 IMU whose telemetry is aggregated into `/data`.

### Architectural consequence

The SDK's existing watchdog can protect against a failed controller command path, but the documented watchdog cannot guarantee safety if the rover itself loses connectivity. That means the proposed autonomy layer must distinguish:

```text
SDK command-path safety
        from
true rover-local safety
```

The PRD therefore treats rover-local/edge safety as a separate component.

---

# 3. Problem Statement

The target use case is autonomous search and exploration over outdoor terrain with unreliable communications and uncertain visual conditions.

The system must operate under:

### 3.1 Network latency and jitter

Remote command/control may experience:

- 150–600 ms round-trip latency.
- Variable jitter.
- Packet loss.
- Temporary disconnects.
- Delayed command confirmation.
- Video frame staleness.

A remote closed loop cannot assume the observed image corresponds to the current rover state.

### 3.2 Severe visual disturbance

Outdoor rovers can encounter:

- Camera shake.
- High angular velocity.
- Motion blur.
- Rolling-shutter distortion.
- Sun glare.
- Lens flare.
- Dust.
- Low texture.
- Repetitive rock/soil patterns.
- Shadows.
- Dynamic vegetation.

Pure ORB/FAST-style feature pipelines are not a sufficient long-term localization strategy under all such conditions.

### 3.3 Wheel slip and terrain uncertainty

Wheels may rotate without equivalent vehicle motion due to:

- Gravel.
- Mud.
- Loose soil.
- Ruts.
- High-center situations.
- Uneven traction across wheels.

Wheel RPM therefore cannot be treated as direct ground velocity.

### 3.4 Rollover risk

The rover has a documented maximum slope capability. A fixed threshold equal to the published limit must not be treated as the safety trigger.

Rollover prediction must account for:

- Pitch.
- Roll.
- Angular rate.
- Terrain geometry.
- Vehicle geometry.
- Current speed.
- Slip.
- Sensor confidence.
- Transient disturbances.

### 3.5 Mission failure from stale autonomy

The most dangerous system state is:

```text
remote planner is unavailable
+
last motion command remains active
+
operator does not receive timely feedback
```

The new system must make this failure mode bounded and observable.

---

# 4. Goals

## 4.1 Primary goals

1. Provide bounded-risk autonomous driving over rough outdoor terrain.
2. Keep all safety-critical control decisions local to the rover/edge computer.
3. Make remote connectivity an optimization, not a safety dependency.
4. Support semantic object search using open-vocabulary vision models.
5. Provide drift-bounded global navigation using a topological-semantic memory graph.
6. Preserve local metric geometry for collision and terrain reasoning.
7. Support latency-aware local trajectory optimization using MPPI or an equivalent constrained local planner.
8. Detect and handle:
   - rollover risk,
   - high-centering/stuck conditions,
   - stale commands,
   - camera degradation,
   - telemetry loss,
   - network degradation,
   - perception uncertainty.
9. Reuse the existing SDK control, telemetry and streaming infrastructure.
10. Produce deterministic logs and replayable autonomy traces for debugging.

## 4.2 Secondary goals

1. Keep perception modules replaceable.
2. Allow CPU-only degraded operation.
3. Allow GPU acceleration where available.
4. Support ROS2 integration without requiring ROS2 for the core SDK.
5. Support simulation and recorded-data testing before physical rover deployment.
6. Expose autonomy health through SDK APIs/dashboard.
7. Create a clean extension point for future sensors such as stereo cameras, ToF, LiDAR, wheel encoders, or improved IMUs.

---

# 5. Non-Goals

The first production release will not attempt to:

- Guarantee global centimeter-level localization.
- Claim “zero drift.”
- Replace firmware-level emergency shutdown behavior.
- Infer safe terrain from monocular depth alone.
- Perform end-to-end neural motor control.
- Allow an LLM to directly emit motor commands.
- Make remote AI a safety authority.
- Guarantee operation on terrain outside the rover's documented mechanical envelope.
- Automatically reverse a fixed distance after every safety event.
- Treat a single-frame detector confidence threshold as object confirmation.
- Require a specific foundation model vendor.

---

# 6. Design Principles

## 6.1 Safety authority hierarchy

The command path MUST follow:

```text
Hardware / motor safety
        >
Edge safety arbiter
        >
Local trajectory planner
        >
Mission objective
        >
Remote semantic planner
        >
Operator suggestion
```

A lower-priority layer may request a command, but a higher-priority layer can veto it.

## 6.2 Local safety, remote intelligence

The following must be local:

- command freshness validation,
- emergency stop,
- rollover risk decision,
- minimum obstacle clearance enforcement,
- local terrain hazard rejection,
- network-loss behavior,
- sensor health gating,
- recovery-state transitions.

The following may be remote:

- semantic object recognition,
- global exploration,
- VPR retrieval,
- topological graph maintenance,
- mission strategy,
- next-best-view selection.

## 6.3 Uncertainty is a first-class signal

Every perception output SHOULD have:

- timestamp,
- confidence,
- covariance or uncertainty proxy,
- age,
- frame/source identifier.

Unknown must never silently become safe.

## 6.4 Local metric + global topology

The system MUST maintain two complementary world representations:

```text
Global:
  topological + semantic

Local:
  metric + geometric + probabilistic
```

## 6.5 No single-model dependency

Models must implement interfaces so they can be replaced.

Examples:

```text
DepthProvider
PlaceRecognitionProvider
ObjectDetector
Segmenter
TraversabilityEstimator
TrajectoryPlanner
```

## 6.6 Graceful degradation

When capability is lost:

```text
Full autonomy
   ↓
Reduced autonomy
   ↓
Slow / inspect
   ↓
Safe stop
   ↓
Human intervention
```

There must be no implicit transition from “degraded perception” to “continue at full speed.”

---

# 7. Target Users / Actors

| Actor | Responsibility |
|---|---|
| Remote Operator | Monitor mission, take control, intervene |
| Mission Planner | Define mission/search objective |
| Autonomy Agent | Execute local and global autonomy |
| Safety Controller | Veto unsafe motion |
| SDK Server | Manage rover session, transport, APIs |
| Perception Runtime | Produce geometry/semantic observations |
| ROS2 Client | Consume and publish standardized robotics topics |
| Developer | Configure, test and replay autonomy |
| Mission Backend | Checkpoint/mission state |
| Observability System | Store metrics, events and traces |

---

# 8. High-Level User Stories

### US-01 — Autonomous exploration

As a mission operator, I want to define a search target so the rover can autonomously explore a bounded area without requiring continuous steering.

### US-02 — LTE degradation

As a mission operator, if the LTE connection degrades, I want the rover to continue executing bounded local autonomy or safely stop rather than execute stale commands.

### US-03 — Hazard rejection

As a rover operator, I want the system to detect high-risk terrain before entering it.

### US-04 — Object search

As a mission operator, I want to search for arbitrary natural-language objects such as “red canister” or “metal pipe” without retraining a detector.

### US-05 — Candidate verification

As a mission operator, I want suspected objects to be tracked over time and verified from better viewpoints before they are reported as found.

### US-06 — Stuck detection

As a rover operator, I want the autonomy system to detect loss of traction and enter a controlled recovery behavior.

### US-07 — Rollover prevention

As a rover operator, I want the system to prevent commands that increase rollover risk.

### US-08 — Operator takeover

As a remote operator, I want to immediately take control and know the current autonomy state.

### US-09 — Replay

As a developer, I want to replay a mission from recorded inputs to reproduce a planner or perception failure.

---

# 9. Proposed Repository Architecture

## 9.1 New top-level directories

Recommended structure:

```text
earth-rovers-sdk/
├── autonomy/
│   ├── __init__.py
│   ├── config.py
│   ├── runtime.py
│   │
│   ├── contracts/
│   │   ├── __init__.py
│   │   ├── commands.py
│   │   ├── state.py
│   │   ├── perception.py
│   │   ├── planning.py
│   │   └── mission.py
│   │
│   ├── sensors/
│   │   ├── camera.py
│   │   ├── telemetry.py
│   │   ├── imu.py
│   │   ├── time_sync.py
│   │   └── health.py
│   │
│   ├── perception/
│   │   ├── base.py
│   │   ├── depth.py
│   │   ├── objects.py
│   │   ├── segmentation.py
│   │   ├── visual_quality.py
│   │   └── temporal_fusion.py
│   │
│   ├── localization/
│   │   ├── state_estimator.py
│   │   ├── visual_odometry.py
│   │   ├── vpr.py
│   │   ├── topology.py
│   │   └── loop_closure.py
│   │
│   ├── mapping/
│   │   ├── local_grid.py
│   │   ├── elevation.py
│   │   ├── traversability.py
│   │   └── coverage.py
│   │
│   ├── planning/
│   │   ├── global_explorer.py
│   │   ├── next_best_view.py
│   │   ├── local_mppi.py
│   │   ├── trajectory.py
│   │   ├── recovery.py
│   │   └── cost.py
│   │
│   ├── safety/
│   │   ├── arbiter.py
│   │   ├── rollover.py
│   │   ├── slip.py
│   │   ├── command_watchdog.py
│   │   ├── network_watchdog.py
│   │   ├── perception_gate.py
│   │   └── state_machine.py
│   │
│   ├── semantic/
│   │   ├── object_memory.py
│   │   ├── search.py
│   │   └── evidence.py
│   │
│   ├── transport/
│   │   ├── sdk_client.py
│   │   ├── command_stream.py
│   │   ├── telemetry_stream.py
│   │   └── reconnect.py
│   │
│   ├── observability/
│   │   ├── metrics.py
│   │   ├── events.py
│   │   ├── recorder.py
│   │   └── replay.py
│   │
│   └── providers/
│       ├── depth_anything.py
│       ├── dino_vpr.py
│       ├── megalloc_vpr.py
│       ├── yoloe.py
│       ├── yoloworld.py
│       └── mppi_torch.py
│
├── examples/
│   ├── autonomy/
│   │   ├── local_demo.py
│   │   ├── remote_semantic_demo.py
│   │   └── replay_demo.py
│   └── ros2/
│       └── ...
│
├── tests/
│   ├── autonomy/
│   ├── integration/
│   ├── replay/
│   └── safety/
│
├── configs/
│   ├── autonomy.yaml
│   ├── providers.yaml
│   └── safety.yaml
│
└── docs/
    └── RESILIENT_AUTONOMY_PRD.md
```

### Important implementation constraint

Do not turn `main.py` into a monolithic autonomy engine.

`main.py` should remain the HTTP/session boundary and delegate into `autonomy.runtime`.

---

# 10. Deployment Model

## 10.1 Mode A — Remote-only development

Useful for initial development:

```text
Laptop / GPU
  ├─ autonomy agent
  └─ SDK client
          ↓
      internet/LTE
          ↓
       rover
```

This mode can validate:

- VPR,
- semantics,
- exploration,
- latency modeling,
- command streaming,
- replay.

It MUST NOT be advertised as hard-real-time safety autonomy.

## 10.2 Mode B — Edge autonomy

Production target:

```text
Rover
 ├─ camera
 ├─ IMU
 ├─ wheel/motor telemetry where available
 └─ edge compute
       └─ autonomy agent
             └─ SDK /control transport
```

Remote host:

```text
Remote GPU
 └─ semantic mission layer
```

## 10.3 Mode C — Hybrid

Edge runs:

- state estimator,
- terrain hazard detection,
- local planner,
- safety arbiter.

Remote runs:

- VPR,
- object search,
- semantic graph,
- mission strategy.

This is the default production configuration.

---

# 11. Runtime State Machine

The autonomy runtime MUST use explicit states.

```text
                 ┌──────────────┐
                 │   DISABLED   │
                 └──────┬───────┘
                        │ start
                        ▼
                 ┌──────────────┐
                 │ INITIALIZING │
                 └──────┬───────┘
                        │ healthy
                        ▼
                 ┌──────────────┐
                 │     READY    │
                 └──────┬───────┘
                        │ mission
                        ▼
                 ┌──────────────┐
                 │ AUTONOMOUS   │
                 └───┬──────┬───┘
                     │      │
             degraded│      │hazard
                     ▼      ▼
             ┌──────────┐ ┌─────────┐
             │ DEFERRED │ │  STOP   │
             └────┬─────┘ └────┬────┘
                  │             │
                  │ recovery    │ recoverable
                  ▼             ▼
             AUTONOMOUS      RECOVERY
                                │
                         impossible/unsafe
                                ▼
                           MANUAL_ONLY
```

Additional terminal state:

```text
EMERGENCY_STOP
```

State transitions must be logged with timestamp, previous state, next state, reason, and triggering sensor/event.

---

# 12. Canonical Data Contracts

The autonomy stack must use typed data contracts.

## 12.1 RoverState

```python
class RoverState:
    timestamp: float
    pose: PoseEstimate
    velocity: VelocityEstimate
    acceleration: Vector3
    orientation: Quaternion
    roll_deg: float
    pitch_deg: float
    yaw_deg: float

    wheel_rpm: list[float]
    slip_ratio: float | None

    battery_pct: float | None
    gps_fix_quality: float | None

    command_age_ms: float
    telemetry_age_ms: float
    camera_age_ms: float

    network_rtt_ms: float | None
    network_jitter_ms: float | None

    state_confidence: float
```

## 12.2 MotionCommand

```python
class MotionCommand:
    linear: float
    angular: float

    valid_from: float
    expires_at: float

    source: str
    priority: int
    safety_token: str | None
```

The edge arbiter MUST reject expired commands.

## 12.3 PerceptionObservation

```python
class PerceptionObservation:
    timestamp: float
    sensor_id: str
    frame_id: str

    payload: object

    confidence: float
    age_ms: float
    quality: float

    uncertainty: dict[str, float]
```

## 12.4 TerrainCell

```python
class TerrainCell:
    x: float
    y: float

    elevation_m: float | None
    slope_deg: float | None
    roughness: float | None
    curvature: float | None

    obstacle_cost: float
    dropoff_cost: float
    slip_cost: float
    rollover_cost: float
    uncertainty_cost: float

    traversability: float
    confidence: float
```

---

# 13. Sensor Ingestion

## 13.1 Camera ingestion

Use the existing `/feed` endpoint for continuous consumption.

Default design:

```text
/ feed?view=front&fps=N
```

Requirements:

- Preserve capture timestamps.
- Do not process stale frames.
- Track frame age.
- Track duplicate frames.
- Track decode failures.
- Track frame sequence IDs.
- Track processing latency.

The existing `/v2/*` polling path remains supported for ROS2-style clients and diagnostics but should not be the primary high-rate video path when `/feed` is available.

## 13.2 Telemetry ingestion

Use `/data` and/or the SDK telemetry WebSocket path.

Requirements:

- Latest-wins cache.
- Monotonic age calculation.
- Original sensor timestamps retained.
- Telemetry sequence tracking.
- Telemetry dropout counters.

## 13.3 IMU

Do not assume the SDK's externally aggregated IMU feed is sufficient for a 50–100 Hz safety loop.

Production edge deployments MUST provide a rover-local IMU path if high-rate rollover protection is required.

The autonomy abstraction shall therefore support:

```text
IMUProvider
 ├─ SDKTelemetryIMU
 ├─ ROS2Imu
 ├─ SerialImu
 └─ CustomHardwareImu
```

If only low-rate SDK IMU data is available, the autonomy state must explicitly mark high-rate rollover protection as unavailable and switch to reduced-capability mode.

## 13.4 Time synchronization

All sensor measurements MUST carry:

- source timestamp,
- local receive timestamp,
- estimated clock offset where available.

The autonomy stack MUST distinguish:

```text
measurement time
from
processing time
from
command delivery time
```

This is required for latency compensation.

---

# 14. State Estimation

## 14.1 Objective

Provide a local state estimate robust to:

- wheel slip,
- camera shake,
- intermittent visual tracking,
- GPS degradation,
- delayed telemetry.

## 14.2 Recommended fusion hierarchy

```text
High-rate IMU
    +
Wheel / RPM
    +
Visual odometry
    +
GPS when healthy
    +
VPR correction
```

The baseline can use an EKF/UKF-style estimator.

Long term, the interface must allow factor-graph or invariant-EKF implementations.

## 14.3 State estimate requirements

State estimator MUST expose:

```text
pose
velocity
angular velocity
linear acceleration
covariance/uncertainty
source health
last update age
```

## 14.4 Visual odometry

Visual odometry is a local aid, not the global truth source.

Use temporal image sequences and IMU where available.

The VO component MUST expose:

```text
tracking_quality
feature_count
inlier_ratio
estimated_motion
covariance
camera_health
```

When visual quality drops:

```text
VO state → degraded
```

not:

```text
VO state → valid with stale pose
```

---

# 15. Visual Place Recognition and Topological Memory

## 15.1 Objective

Provide global scene identity that is robust to moderate viewpoint, illumination, and local motion changes.

## 15.2 VPR abstraction

```python
class PlaceRecognitionProvider(Protocol):
    def embed(self, image) -> Embedding:
        ...

    def retrieve(self, embedding, k: int) -> list[PlaceCandidate]:
        ...

    def verify(self, current, candidate) -> VerificationResult:
        ...
```

## 15.3 Candidate retrieval

Candidate selection may use:

- DINO-family embeddings,
- CosPlace,
- MegaLoc,
- future VPR models.

The system MUST NOT encode a universal hard threshold such as “cosine similarity > 0.88 means loop closure.”

Instead use:

```text
appearance similarity
+
heading compatibility
+
temporal context
+
geometric verification
+
candidate consistency
```

## 15.4 Topological node

Each node stores:

```text
node_id
embedding
capture timestamp
approximate GPS
local metric pose
heading
terrain signature
scene quality
objects observed
search coverage
risk score
parent/neighbor edges
```

## 15.5 Edge

Each edge stores:

```text
from_node
to_node
estimated distance
estimated travel time
heading delta
terrain difficulty
risk
slip evidence
successful traversals
failed traversals
```

## 15.6 Loop closure

A loop closure must be classified:

```text
CANDIDATE
VERIFIED
REJECTED
```

Only `VERIFIED` loop closures may modify global topology.

---

# 16. Local Terrain Understanding

## 16.1 Objective

Build a local probabilistic terrain model sufficient for:

- collision avoidance,
- slope estimation,
- roughness estimation,
- drop-off detection,
- traversability scoring,
- local trajectory optimization.

## 16.2 Depth provider

Initial providers:

```text
DepthAnythingProvider
CustomStereoProvider
ExternalDepthProvider
```

The architecture must treat foundation-model depth as a probabilistic estimate.

## 16.3 Terrain features

For each local cell estimate:

```text
height
slope
surface roughness
curvature
obstacle height
drop-off likelihood
slip likelihood
rollover risk
perception confidence
```

## 16.4 Ground/obstacle model

Do not depend exclusively on:

```text
surface normal angle > threshold
```

Instead combine:

```text
surface orientation
height discontinuity
local curvature
free-space continuity
temporal consistency
sensor confidence
```

## 16.5 Negative obstacle detection

Detect:

- drop-offs,
- holes,
- trenches,
- unobservable shadow zones.

A negative obstacle must increase cost even when the raw depth image is incomplete.

Unknown geometry near a driving corridor must produce a conservative score.

---

# 17. Traversability Model

Define total terrain cost:

\[
C_T =
w_s C_{slope}
+
w_r C_{roughness}
+
w_o C_{obstacle}
+
w_d C_{dropoff}
+
w_{slip} C_{slip}
+
w_{roll} C_{rollover}
+
w_u C_{uncertainty}
\]

Where each component is normalized to `[0,1]`.

Default policy:

```text
0.0 → preferred
0.0–0.4 → traversable
0.4–0.7 → caution
0.7–0.9 → high risk
>0.9 → prohibited
```

These values are initial engineering defaults only and MUST be calibrated using actual rover data.

---

# 18. Rollover Risk

## 18.1 Objective

Detect unsafe vehicle attitude before entering or remaining in unstable terrain.

## 18.2 Inputs

Use:

- pitch,
- roll,
- pitch rate,
- roll rate,
- linear acceleration,
- terrain slope,
- vehicle speed,
- slip estimate,
- local geometry,
- uncertainty.

## 18.3 Dynamic risk

Rollover risk MUST NOT be represented only as:

```text
abs(pitch) > fixed_angle
```

Instead compute:

```text
risk =
f(
  pitch,
  roll,
  angular_rate,
  terrain_angle,
  acceleration,
  speed,
  slip,
  uncertainty
)
```

## 18.4 Safety bands

Use configurable bands:

```text
NORMAL
CAUTION
HIGH_RISK
EMERGENCY
```

The actual numeric thresholds MUST be determined by hardware testing.

The documented 18° maximum slope is a vehicle capability constraint, not a universal software emergency threshold.

## 18.5 Emergency action

On `EMERGENCY`:

1. veto all forward motion commands;
2. publish zero motion immediately;
3. enter `SAFETY_STOP`;
4. capture diagnostic snapshot;
5. attempt local stabilization only if a validated recovery controller exists;
6. otherwise request manual intervention.

Do NOT perform an unconditional “reverse 1.5 m” maneuver.

---

# 19. Slip and Stuck Detection

## 19.1 Objective

Detect when commanded motion is not producing expected vehicle motion.

## 19.2 Inputs

```text
commanded velocity
wheel RPM
IMU acceleration
visual odometry velocity
GPS velocity if valid
motor current where available
```

## 19.3 Slip estimate

Approximate:

\[
Slip = 1 - \frac{v_{actual}}{v_{wheel}}
\]

with saturation and uncertainty handling.

The system MUST NOT divide by a near-zero denominator.

## 19.4 Stuck conditions

Candidate stuck condition:

```text
commanded_motion > threshold
AND
actual_motion < threshold
FOR duration > configured threshold
```

Confidence must be increased by corroborating evidence such as wheel RPM and motor current.

## 19.5 Recovery state machine

```text
STUCK_SUSPECTED
      ↓
STOP
      ↓
VERIFY
      ↓
RECOVERY_CANDIDATES
      ↓
SELECT_LOWEST_RISK
      ↓
EXECUTE_SHORT_MANEUVER
      ↓
VERIFY_PROGRESS
   ┌──┴──┐
 success  fail
   │       │
   ▼       ▼
 RESUME  ESCALATE
```

Recovery must be short-horizon and continuously safety-arbitrated.

---

# 20. Camera Health

## 20.1 Inputs

Compute:

- brightness histogram,
- saturation ratio,
- blur estimate,
- contrast,
- entropy,
- edge/feature count,
- detector stability,
- depth confidence.

## 20.2 Camera health score

```text
camera_health ∈ [0,1]
```

Suggested interpretation:

```text
>0.75 healthy
0.50–0.75 degraded
0.25–0.50 poor
<0.25 unusable
```

These thresholds are calibration targets, not final values.

## 20.3 Behavior

```text
DEGRADED:
  reduce speed
  increase perception redundancy
  prefer known terrain

POOR:
  stop exploring new terrain
  attempt camera recovery / viewpoint adjustment

UNUSABLE:
  safe stop unless another validated navigation source exists
```

A blind 45° rotation MUST NOT be the universal recovery action.

---

# 21. Network and Latency Model

## 21.1 Measurements

Maintain rolling estimates for:

```text
RTT
RTT p50/p95/p99
jitter
packet loss
command acknowledgement age
telemetry age
video age
disconnect duration
```

## 21.2 Command age

Every remote intent must have:

```text
created_at
expires_at
sequence_id
```

The edge agent MUST refuse to execute expired remote commands.

## 21.3 Latency-compensated planning

The local planner should use:

```text
predicted_state =
state_mean propagated to effective command-delivery time
+
state covariance growth
```

rather than:

```text
state + velocity * nominal RTT
```

## 21.4 Uncertainty-aware cost

MPPI must penalize commands that become unsafe when state uncertainty increases.

Example:

\[
J_{uncertainty}
=
\alpha \cdot
trace(\Sigma_{pose})
\]

or another implementation-specific uncertainty metric.

---

# 22. Local Planner — MPPI

## 22.1 Objective

Generate a short-horizon motion plan that respects:

- terrain cost,
- collision risk,
- rollover risk,
- slip risk,
- current state uncertainty,
- command age,
- velocity limits,
- angular limits,
- acceleration/curvature constraints.

## 22.2 Planning horizon

Initial target:

```text
1–2 seconds
```

This is a **planning horizon**, not a command duration.

## 22.3 Command streaming

The local controller must continuously issue short-lived commands through `/control`.

Initial target:

```text
control update: 10 Hz
command TTL: 200–500 ms
planner refresh: 5–20 Hz
```

Actual values must be benchmarked on hardware.

## 22.4 Candidate trajectory representation

Candidate controls may use:

```text
[v, omega]
```

or spline/arc primitives.

The final SDK command remains:

```json
{
  "command": {
    "linear": 0.0,
    "angular": 0.0,
    "lamp": 0
  }
}
```

## 22.5 Cost terms

Minimum cost terms:

```text
trajectory tracking
obstacle
terrain
slope
roughness
drop-off
rollover
slip
uncertainty
command age
control effort
reverse penalty
recovery penalty
```

---

# 23. Global Exploration

## 23.1 Principle

The global planner should output:

```text
goal
route corridor
preferred viewpoint
semantic objective
```

It should NOT directly emit 10 Hz motor commands.

## 23.2 Frontier/objective selection

For each candidate node or frontier:

\[
Utility =
\frac{
ExpectedInformationGain
+
ObjectDetectionGain
+
CoverageGain
}{
TravelCost
+
RiskCost
+
EnergyCost
}
\]

## 23.3 Next-best-view

The planner should consider:

- visibility,
- expected object scale,
- occlusion,
- terrain risk,
- energy,
- network quality,
- distance,
- return/recovery options.

Do not assume that “high ground always wins.”

## 23.4 Exploration policy

Prioritize candidate viewpoints that maximize:

```text
information gain per unit risk
```

---

# 24. Semantic Object Search

## 24.1 Input

Natural language objective:

```text
"Find a red plastic canister"
```

## 24.2 Detection pipeline

```text
Open-vocabulary detector
        ↓
multi-frame association
        ↓
candidate track
        ↓
geometric consistency
        ↓
viewpoint optimization
        ↓
close-range verification
        ↓
object evidence record
```

## 24.3 Object evidence

```json
{
  "object_id": "candidate-17",
  "query": "red plastic canister",
  "first_seen": 0.0,
  "last_seen": 42.2,
  "frames_seen": 16,
  "bbox_history": [],
  "track_confidence": 0.89,
  "appearance_confidence": 0.93,
  "geometric_confidence": 0.82,
  "verified": true,
  "location_node": "node-42",
  "local_pose": {}
}
```

## 24.4 Verification

Object confirmation MUST use multiple observations.

Never use:

```text
single frame confidence > X
```

as the only acceptance condition.

---

# 25. Semantic Memory

Maintain:

```text
ObjectMemory
PlaceMemory
TerrainMemory
FailureMemory
ExplorationCoverage
```

### Failure memory

Every failed traversal must affect future route planning.

Example:

```text
node A → node B
failed due to:
  high slip
  wheel spin
  excessive slope
```

Future route cost for this edge increases until evidence indicates the route is safe again.

---

# 26. Safety Command Arbiter

## 26.1 Objective

Guarantee that every motion command passes safety validation before `/control`.

## 26.2 Inputs

```text
remote mission command
local planner command
manual command
rollover state
terrain state
sensor health
network state
command age
battery state
mission state
```

## 26.3 Priority

```text
EMERGENCY_STOP
    1000

SAFETY_RESTRICTION
     900

MANUAL_OPERATOR
     700

LOCAL_AUTONOMY
     500

REMOTE_MISSION
     300
```

## 26.4 Output

```python
SafetyDecision(
    allowed: bool,
    command: MotionCommand,
    reason: str,
    constraints_applied: list[str],
)
```

## 26.5 Hard invariants

Examples:

```text
expired command → reject
unsafe terrain → reject
emergency state → reject non-zero motion
stale state → clamp speed or stop
camera unusable + no alternate navigation → stop
critical telemetry missing → stop or restricted mode
```

---

# 27. Command Watchdog

The existing SDK watchdog must remain enabled for autonomy streaming.

Recommended initial production configuration:

```text
CONTROL_WATCHDOG_S = 0.5–1.0
```

Subject to validation.

The autonomy command stream must:

1. publish continuously;
2. maintain sequence IDs;
3. measure dispatch latency;
4. measure delivery latency where observable;
5. treat missed confirmations as a health event;
6. fail closed.

The autonomy layer MUST NOT attempt to disable the SDK watchdog as a way to make motion smoother.

---

# 28. New SDK Endpoints

The following endpoints are proposed.

## `GET /autonomy/status`

Returns:

```json
{
  "enabled": true,
  "mode": "hybrid",
  "state": "AUTONOMOUS",
  "safety_state": "NORMAL",
  "planner": {
    "healthy": true,
    "hz": 10.0,
    "last_plan_age_ms": 21
  },
  "perception": {
    "camera_health": 0.91,
    "depth_health": 0.88,
    "vpr_health": 0.95
  },
  "network": {
    "rtt_ms": 285,
    "jitter_ms": 42,
    "command_age_ms": 74
  },
  "terrain": {
    "risk": 0.21,
    "uncertainty": 0.08
  }
}
```

## `POST /autonomy/start`

Starts autonomy after validating configuration and sensor health.

## `POST /autonomy/stop`

Transitions autonomy to safe stopped mode.

## `POST /autonomy/mode`

Supported modes:

```text
MANUAL
ASSISTED
AUTONOMOUS
SAFE_STOP
RECOVERY
```

## `POST /autonomy/mission`

Example:

```json
{
  "objective": {
    "type": "object_search",
    "query": "red plastic canister"
  },
  "search_boundary": {
    "polygon": []
  },
  "risk_budget": 0.35
}
```

## `GET /autonomy/events`

Returns recent autonomy safety/planning events.

## `GET /autonomy/graph`

Returns a summarized topological graph for visualization/debugging.

## `GET /autonomy/metrics`

Exports autonomy metrics.

---

# 29. Backward Compatibility

The existing SDK endpoints MUST remain functional.

No autonomy feature may break:

```text
POST /control
GET /data
GET /status
GET /feed
GET /v2/*
POST /start-mission
POST /checkpoint-reached
POST /end-mission
interventions/*
```

Autonomy must be opt-in.

Default startup:

```text
AUTONOMY_ENABLED=false
```

---

# 30. Configuration

Proposed `.env` configuration:

```env
# Core
AUTONOMY_ENABLED=false
AUTONOMY_MODE=hybrid

# Control
AUTONOMY_CONTROL_HZ=10
AUTONOMY_COMMAND_TTL_MS=400
CONTROL_WATCHDOG_S=0.8

# Planning
AUTONOMY_PLANNER_HZ=10
AUTONOMY_PLAN_HORIZON_S=2.0
AUTONOMY_MPPI_SAMPLES=1024

# Safety
AUTONOMY_MAX_SPEED=0.35
AUTONOMY_CAUTION_SPEED=0.20
AUTONOMY_STOP_SPEED=0.0

AUTONOMY_TERRAIN_STOP_COST=0.90
AUTONOMY_UNKNOWN_TERRAIN_COST=0.80

# State freshness
AUTONOMY_MAX_TELEMETRY_AGE_MS=500
AUTONOMY_MAX_CAMERA_AGE_MS=500
AUTONOMY_MAX_STATE_AGE_MS=250

# Network
AUTONOMY_RTT_CAUTION_MS=250
AUTONOMY_RTT_STOP_MS=1000
AUTONOMY_JITTER_STOP_MS=500

# Perception
AUTONOMY_DEPTH_PROVIDER=depth_anything
AUTONOMY_VPR_PROVIDER=megalloc
AUTONOMY_OBJECT_PROVIDER=yoloe

# Persistence
AUTONOMY_DATA_DIR=./data/autonomy
AUTONOMY_RECORDING_ENABLED=true
```

All safety thresholds must be validated and documented before hardware deployment.

---

# 31. Observability

## 31.1 Metrics

Minimum metrics:

### Control

```text
autonomy_control_commands_total
autonomy_control_rejections_total
autonomy_command_age_ms
autonomy_command_dispatch_ms
autonomy_command_delivery_ms
autonomy_watchdog_trips_total
```

### Network

```text
autonomy_rtt_ms
autonomy_jitter_ms
autonomy_packet_loss_ratio
autonomy_disconnect_total
```

### Perception

```text
autonomy_camera_fps
autonomy_camera_age_ms
autonomy_camera_health
autonomy_depth_latency_ms
autonomy_depth_confidence
autonomy_vpr_latency_ms
autonomy_vpr_confidence
autonomy_detector_latency_ms
```

### Planner

```text
autonomy_planner_hz
autonomy_planning_latency_ms
autonomy_mppi_iterations
autonomy_candidate_count
autonomy_trajectory_rejections
```

### Safety

```text
autonomy_rollover_warnings_total
autonomy_emergency_stop_total
autonomy_stuck_events_total
autonomy_recovery_attempts_total
autonomy_recovery_success_total
autonomy_perception_gate_stops_total
```

### Mission

```text
autonomy_distance_m
autonomy_search_area_m2
autonomy_coverage_ratio
autonomy_objects_seen_total
autonomy_objects_verified_total
autonomy_false_candidate_total
```

---

# 32. Event Model

All safety-critical events MUST be structured.

Example:

```json
{
  "event": "SAFETY_STOP",
  "timestamp": 1760000000.123,
  "severity": "CRITICAL",
  "reason": "ROLL_RISK",
  "state": "AUTONOMOUS",
  "roll_deg": 16.2,
  "roll_rate_deg_s": 9.1,
  "terrain_risk": 0.93,
  "command": {
    "linear": 0.28,
    "angular": 0.05
  }
}
```

---

# 33. Recording and Replay

## 33.1 Recording requirements

Record:

```text
camera timestamps / optional frames
telemetry
IMU
commands
planner outputs
safety decisions
network stats
perception outputs
VPR candidates
mission events
```

## 33.2 Replay requirements

Replay must support:

```text
sensor replay
planner-only replay
safety-only replay
full stack replay
```

## 33.3 Determinism

Given:

```text
same sensor stream
same configuration
same random seed
```

the deterministic planner/safety components SHOULD produce materially equivalent results.

---

# 34. Testing Strategy

## 34.1 Unit tests

Required for:

- command expiration,
- command priority,
- safety arbiter,
- terrain cost normalization,
- rollover state transitions,
- slip math,
- watchdog state transitions,
- topology node/edge operations,
- event serialization,
- metric calculation.

## 34.2 Property-based tests

Examples:

### Safety

For every possible command:

```text
if emergency_state:
    output.linear == 0
    output.angular == 0
```

### Expiry

```text
if now > command.expires_at:
    command is rejected
```

### Priority

```text
SAFETY_STOP always dominates all lower priorities
```

## 34.3 Integration tests

Test:

```text
SDK /control
autonomy command stream
telemetry stream
camera feed
watchdog
disconnect/reconnect
mission lifecycle
```

## 34.4 Replay tests

Maintain recorded datasets for:

1. smooth terrain;
2. gravel;
3. high camera shake;
4. glare;
5. dust;
6. network latency;
7. network outage;
8. wheel slip;
9. drop-off;
10. high pitch;
11. high roll;
12. false object candidate;
13. repeated visual scene.

## 34.5 Hardware-in-the-loop

At minimum:

```text
motor controller mock
IMU injection
camera replay
network impairment
command loss
```

---

# 35. Network Impairment Test Matrix

The test harness MUST support:

| Condition | Target behavior |
|---|---|
| 50 ms RTT | normal autonomy |
| 150 ms RTT | normal with latency compensation |
| 300 ms RTT | bounded-speed autonomy |
| 600 ms RTT | local loop dominates |
| 1 s RTT | no dependency on remote feedback |
| 2 s RTT | remote commands expire; local controller/safe-stop |
| 10% packet loss | degraded |
| 30% packet loss | safe degraded mode |
| Complete outage | local continuation or safe stop |
| intermittent outage | no stale-command execution |

---

# 36. Acceptance Criteria

The first release is accepted only when the following are demonstrated.

## AC-01 — No stale remote command

Given a remote command that expires before delivery:

```text
the edge autonomy agent MUST NOT execute it.
```

## AC-02 — Network loss

When the remote semantic channel disconnects:

```text
the rover MUST NOT continue indefinitely on a stale semantic command.
```

## AC-03 — Local safety

An emergency safety decision MUST be possible without a round trip to the remote planner.

## AC-04 — Watchdog

A continuous motion stream interrupted at the SDK layer MUST cause the SDK watchdog to engage according to configured timeout.

## AC-05 — Terrain uncertainty

Unknown terrain in the planned footprint must increase cost or cause stop behavior, never silently classify as safe.

## AC-06 — Topological loop closure

A candidate loop closure must not alter global topology until verification succeeds.

## AC-07 — Multi-frame object confirmation

A single low-confidence detection cannot terminate a search mission as “object found.”

## AC-08 — Operator takeover

Manual stop/control must override autonomy within the configured command path.

## AC-09 — Replay

Every safety stop in field testing must be reconstructable from a recording.

## AC-10 — Backward compatibility

All existing SDK tests must remain passing with autonomy disabled.

---

# 37. Performance Targets

These are engineering targets, NOT guaranteed hardware specifications.

## Edge

| Component | Target |
|---|---:|
| Safety arbiter | ≥ 50 Hz |
| State estimation | ≥ 50 Hz |
| Local safety checks | ≥ 50 Hz |
| Local planner | 5–20 Hz |
| Control command stream | 10 Hz |
| Camera processing | ≥ 10 FPS minimum |
| Command arbitration latency | < 20 ms |
| Safety decision latency | < 50 ms target |

## Remote

| Component | Target |
|---|---:|
| VPR inference | < 300 ms |
| Object detection | < 200 ms |
| Global replanning | 1–5 Hz |
| Topological update | < 500 ms |
| Semantic search decision | < 2 s |

Exact values depend on edge GPU, remote GPU, model choice, image size, and transport conditions.

---

# 38. Safety Requirements

## SR-01

The safety arbiter MUST be fail-closed.

## SR-02

A crashed planner MUST NOT cause indefinite motion.

## SR-03

An expired command MUST NOT be accepted.

## SR-04

A missing critical sensor MUST force reduced autonomy or stop.

## SR-05

The system MUST expose the reason for every safety veto.

## SR-06

No machine-learning output may directly bypass the safety arbiter.

## SR-07

Remote mission objectives must not bypass local terrain constraints.

## SR-08

Recovery maneuvers must be bounded and continuously safety-checked.

## SR-09

Emergency stop handling must be deterministic.

## SR-10

Production hardware must have a separate validated firmware/hardware fail-safe path when available; the SDK software layer must not claim to substitute for it.

---

# 39. Security Requirements

## Authentication

Reuse existing SDK authentication.

Do not expose:

- raw API tokens,
- model credentials,
- debug credentials

through telemetry or dashboard payloads.

## Command authorization

Only authenticated clients with explicit autonomy control permission can:

```text
start autonomy
stop autonomy
change autonomy mode
upload mission
```

## Audit

Record:

```text
who changed autonomy mode
when
from where
what mission
what override
```

---

# 40. Dashboard Integration

Extend the existing dashboard with an autonomy panel.

## Required tiles

```text
AUTONOMY
  Mode: AUTONOMOUS

SAFETY
  NORMAL / CAUTION / STOP

NETWORK
  RTT
  jitter
  command age

PERCEPTION
  camera health
  depth confidence
  object tracker
  VPR health

PLANNER
  planner Hz
  planning latency
  current objective

TERRAIN
  risk
  uncertainty

MISSION
  coverage
  current frontier
  target status
```

## Visualization

Add:

- local risk map,
- trajectory candidates,
- selected trajectory,
- topological nodes,
- current semantic target,
- object candidate tracks,
- safety events.

---

# 41. ROS2 Integration

Use ROS2 as an adapter, not as a requirement for the Python core.

Recommended topics:

```text
/camera/front/image_raw
/camera/rear/image_raw

/imu/data
/gps/fix
/wheel/rpm

/odom
/local_costmap
/terrain/risk

/cmd_vel
/autonomy/cmd_vel_safe

/autonomy/state
/autonomy/events
/autonomy/object_tracks
/autonomy/topology
```

The final motor command must still pass through the safety arbiter.

The ROS2 bridge should not create a bypass around safety.

---

# 42. Provider Abstraction

## Depth

```text
DepthProvider
```

Initial providers:

```text
DepthAnything
ExternalDepthAPI
StereoDepth
```

## VPR

```text
PlaceRecognitionProvider
```

Initial providers:

```text
DINO
CosPlace
MegaLoc
```

## Object detection

```text
ObjectDetector
```

Initial providers:

```text
YOLOE
YOLO-World
CustomDetector
```

## Planner

```text
LocalPlanner
```

Initial:

```text
MPPI
FallbackReactivePlanner
```

---

# 43. Dependency Strategy

The existing repository intentionally has a lightweight runtime dependency set.

Do not immediately add heavyweight ML libraries to the core SDK requirements.

Use optional extras:

```text
requirements.txt
requirements-autonomy.txt
requirements-edge.txt
requirements-dev.txt
```

Suggested conceptual split:

```text
core:
  existing SDK dependencies

autonomy:
  torch
  scipy
  pydantic
  faiss-cpu or faiss-gpu
  networkx

vision:
  model-specific dependencies

edge:
  GPU/runtime-specific packages
```

The goal is to keep:

```text
pip install -r requirements.txt
```

working for users who do not enable autonomy.

---

# 44. Model Runtime Strategy

Models SHOULD be loaded lazily.

Example:

```python
provider = DepthProviderFactory.create(config.depth_provider)

if autonomy_enabled:
    provider.load()
```

Avoid loading:

- Torch,
- CUDA,
- FAISS,
- model weights

when autonomy is disabled.

---

# 45. Memory Management

The system must explicitly bound memory usage.

## Video

Use:

```text
latest-frame cache
bounded frame queue
drop-oldest policy
```

Never allow unbounded camera backlog.

## Telemetry

Use:

```text
latest-wins
bounded history window
```

## Topological graph

Use configurable retention:

```text
active graph
compressed historical graph
persistent keyframes
```

## Object tracks

Expire tracks after configurable inactivity.

---

# 46. Failure Mode and Effects Analysis

| Failure | Detection | Action |
|---|---|---|
| Remote planner crash | heartbeat timeout | local planner continues or stop |
| Remote connection lost | network watchdog | local mode / stop |
| Camera stale | frame age | slow/stop |
| Camera unusable | camera-health score | slow/stop |
| IMU unavailable | health monitor | restricted mode/stop |
| Depth unavailable | provider health | safer planner / stop |
| VPR unavailable | provider health | local-only navigation |
| Object detector unavailable | provider health | exploration pause |
| Planner timeout | loop deadline | safe command / stop |
| Command stale | TTL check | reject |
| Telemetry stale | age threshold | restrict/stop |
| Roll risk high | safety estimator | stop |
| Slip high | traction estimator | recovery |
| Recovery failed | retry limit | manual |
| Memory full | recorder health | rotate/degrade |
| GPU OOM | runtime exception | unload/recover/stop |
| SDK session lost | `/status` / transport | watchdog |
| Mission backend unavailable | mission API failure | keep local safety; mission pause |

---

# 47. Implementation Phases

## Phase 0 — Foundations

Deliver:

- autonomy package layout;
- typed contracts;
- configuration;
- runtime lifecycle;
- event model;
- metrics;
- feature flag;
- unit test foundation.

Exit criteria:

```text
AUTONOMY_ENABLED=false
```

has zero behavioral change.

---

## Phase 1 — Command Safety

Deliver:

- command TTL;
- safety arbiter;
- command priority;
- local watchdog;
- network health;
- SDK command integration.

Exit criteria:

- stale commands are rejected;
- safety stop overrides autonomy;
- command stream is observable;
- existing SDK watchdog remains operational.

---

## Phase 2 — Sensor and State Layer

Deliver:

- camera adapter;
- telemetry adapter;
- IMU abstraction;
- time synchronization;
- local state estimator;
- health scoring.

Exit criteria:

- state age visible;
- stale sensor detection works;
- replay input supported.

---

## Phase 3 — Local Terrain

Deliver:

- depth provider interface;
- local terrain grid;
- slope/roughness;
- drop-off detection;
- uncertainty map;
- traversability cost.

Exit criteria:

- terrain grid reproducible from replay;
- unsafe/unknown terrain receives appropriate penalty.

---

## Phase 4 — Local Planner

Deliver:

- MPPI;
- trajectory generation;
- latency-aware state propagation;
- risk-aware cost;
- local `/control` integration.

Exit criteria:

- no stale command execution;
- planner maintains target frequency;
- command output passes safety arbiter.

---

## Phase 5 — Stuck / Recovery

Deliver:

- slip estimation;
- stuck state machine;
- bounded recovery;
- recovery telemetry.

Exit criteria:

- simulated stuck cases recover or escalate safely;
- recovery never bypasses safety.

---

## Phase 6 — VPR / Topology

Deliver:

- embeddings;
- vector index;
- node/edge graph;
- loop closure verification;
- topology persistence.

Exit criteria:

- repeated scenes can be recognized;
- false loop closures are rejected in replay tests;
- topology survives session restart.

---

## Phase 7 — Semantic Search

Deliver:

- open-vocabulary detector;
- object tracking;
- object memory;
- viewpoint refinement;
- evidence-based confirmation.

Exit criteria:

- no single-frame object event ends a mission;
- target candidates are persisted and rechecked.

---

## Phase 8 — Global Exploration

Deliver:

- frontier generation;
- next-best-view;
- information-gain utility;
- semantic search strategy.

Exit criteria:

- planner produces bounded-risk exploration goals;
- global planner does not directly command motors.

---

## Phase 9 — Dashboard

Deliver:

- autonomy panel;
- risk visualization;
- planner state;
- topology visualization;
- object search visualization;
- safety event timeline.

---

## Phase 10 — Hardware Pilot

Run:

1. static sensor validation;
2. low-speed controlled driving;
3. network impairment tests;
4. terrain tests;
5. recovery tests;
6. semantic search tests;
7. mixed autonomy tests.

No outdoor unsupervised testing until all Phase 1–9 acceptance criteria pass.

---

# 48. Recommended GitHub Issue Breakdown

## Epic A — Autonomy Foundation

- A-001 Add autonomy package and configuration
- A-002 Define typed autonomy contracts
- A-003 Add feature flag
- A-004 Add autonomy lifecycle
- A-005 Add metrics/event framework

## Epic B — Safety

- B-001 Safety command arbiter
- B-002 Command TTL enforcement
- B-003 Network watchdog
- B-004 Rollover estimator
- B-005 Sensor health gate
- B-006 Stuck detector
- B-007 Recovery state machine

## Epic C — Perception

- C-001 Camera provider
- C-002 Depth provider interface
- C-003 DepthAnything adapter
- C-004 Camera health
- C-005 Temporal fusion

## Epic D — Localization / Topology

- D-001 State estimator
- D-002 VO provider
- D-003 VPR interface
- D-004 Vector index
- D-005 Topological graph
- D-006 Loop verification

## Epic E — Planning

- E-001 Terrain grid
- E-002 Traversability
- E-003 MPPI
- E-004 Latency compensation
- E-005 Global explorer
- E-006 Next-best-view
- E-007 Recovery planner

## Epic F — Semantic Search

- F-001 Open-vocabulary detector
- F-002 Multi-frame object tracker
- F-003 Object memory
- F-004 Candidate verification
- F-005 Search mission state machine

## Epic G — Transport / SDK

- G-001 Autonomy SDK client
- G-002 Command stream
- G-003 Autonomy REST endpoints
- G-004 `/status` extension
- G-005 Dashboard integration

## Epic H — Replay / Test

- H-001 Recorder
- H-002 Replay engine
- H-003 Network impairment harness
- H-004 Sensor replay harness
- H-005 Safety regression suite

---

# 49. Definition of Done

A feature is complete only when:

- implementation exists;
- public interface is typed/documented;
- unit tests exist;
- failure path is tested;
- metrics exist where operationally relevant;
- logs/events exist for safety behavior;
- replay coverage exists for complex autonomy logic;
- configuration is documented;
- feature can be disabled cleanly;
- existing SDK behavior remains intact.

A safety-critical feature additionally requires:

- simulation test;
- failure-injection test;
- hardware-in-the-loop evidence;
- documented safety assumptions.

---

# 50. Recommended Initial Technology Choices

These are recommendations, not mandatory dependencies.

| Capability | Initial option | Reason |
|---|---|---|
| Language | Python | Fits existing SDK |
| API | FastAPI | Existing repository |
| Runtime | Hypercorn/asyncio | Existing repository |
| Camera | OpenCV/PyAV | Existing ecosystem |
| Tensor runtime | PyTorch | Flexible model support |
| Acceleration | TensorRT where available | Edge latency |
| Vector index | FAISS | Fast VPR retrieval |
| Graph | NetworkX initially | Simple prototype |
| Local map | NumPy/SciPy | Lightweight baseline |
| Planner | PyTorch MPPI | GPU-friendly |
| Config | Pydantic | Existing dependency |
| Robotics adapter | ROS2 | Existing repository example |
| Metrics | Prometheus-compatible abstraction | Operational visibility |
| Replay | JSONL + image/video artifacts | Simple initial implementation |

---

# 51. Architecture Decisions

## ADR-001 — Do not replace all metric localization with topology

Decision:

```text
global topology + local metric geometry
```

Reason:

- topology handles global place identity;
- metric geometry handles local collision avoidance and trajectory generation.

## ADR-002 — Do not run safety-critical control remotely

Decision:

```text
safety and local control on edge
```

Reason:

- communication delay and loss are unavoidable.

## ADR-003 — Keep SDK compatibility

Decision:

Autonomy must be layered over existing SDK interfaces, not replace them.

## ADR-004 — Provider-based ML architecture

Decision:

Model implementations are plugins behind stable interfaces.

## ADR-005 — Fail closed

Decision:

Unknown, stale, or unsafe state reduces capability.

---

# 52. Open Decisions

The following must be resolved before final hardware rollout:

1. What exact rover model is the first supported autonomy target?
2. Is a local Jetson/edge computer physically available?
3. Is direct high-rate IMU access available outside the current SDK telemetry feed?
4. Are wheel RPM/encoder measurements exposed with sufficient update rate?
5. Is motor current available?
6. Is the rover firmware able to perform a local hardware fail-safe stop?
7. What exact camera timestamp semantics are available?
8. Which GPU will host the edge perception stack?
9. Is a second camera available on the target rover?
10. Is stereo or LiDAR hardware permissible?
11. What terrain classes will be considered supported?
12. What maximum autonomous operating speed will be approved?
13. What recovery maneuvers are mechanically validated?
14. What constitutes a mission-level “object found” acceptance event?
15. How long should topological memory persist?
16. What remote semantic compute environment will be used?
17. What data can be persisted due to privacy/retention constraints?

---

# 53. Recommended MVP Scope

Do not implement the entire architecture in one milestone.

The first useful MVP should be:

```text
┌──────────────────────────────────────────────┐
│ MVP                                          │
├──────────────────────────────────────────────┤
│ Existing SDK camera + telemetry              │
│ Command TTL                                  │
│ Safety arbiter                               │
│ Network watchdog                             │
│ Local state estimator                        │
│ Depth provider                               │
│ Terrain risk map                             │
│ Local reactive planner                       │
│ Basic MPPI                                   │
│ Rollover gate                                │
│ Stuck detector                               │
│ Replay recorder                              │
└──────────────────────────────────────────────┘
```

Then add:

```text
VPR → topology → semantic object search → global exploration
```

This order matters because a beautiful semantic planner is not useful if the command/safety layer is still unsafe.

---

# 54. What Not to Build First

Do NOT start by implementing:

- LLM-based mission planner;
- large semantic graph UI;
- open-vocabulary object search;
- complex frontier algorithms;
- 1024/2048-sample MPPI GPU optimization;
- multiple VPR backends.

Start with:

```text
command freshness
+
safety arbiter
+
sensor health
+
local geometry
+
replay
```

These provide the foundation on which all higher-level intelligence depends.

---

# 55. MVP End-to-End Flow

Example mission:

> Find a red plastic canister in the search region.

Flow:

```text
Operator
   │
   ▼
POST /autonomy/mission
   │
   ▼
Mission Manager
   │
   ▼
Semantic Search Objective
   │
   ▼
Remote Object Detector
   │
   ▼
Candidate region
   │
   ▼
Next-Best-View
   │
   ▼
Goal / route corridor
   │
   ▼
EDGE LOCAL PLANNER
   │
   ├─ state
   ├─ terrain
   ├─ risk
   ├─ uncertainty
   └─ latency
   │
   ▼
Safety Arbiter
   │
   ▼
/control @ 10 Hz
   │
   ▼
Rover
   │
   ▼
new observations
   │
   ├─ VPR update
   ├─ terrain update
   ├─ object tracker
   └─ coverage update
   │
   ▼
Object Verification
   │
   ▼
MISSION_COMPLETE
```

---

# 56. Success Metrics

The project should be evaluated using measurable outcomes.

## Safety

```text
unsafe command execution = 0
stale command execution = 0
unexplained safety stop = minimize
```

## Control

```text
command update stability
planner deadline miss rate
command latency
```

## Perception

```text
terrain false-safe rate
camera degradation detection rate
object verification precision
VPR false-loop rate
```

## Autonomy

```text
distance traveled autonomously
coverage achieved
mission completion rate
intervention rate
recovery success rate
```

## Network resilience

```text
performance under RTT
performance under jitter
performance under packet loss
recovery after disconnect
```

---

# 57. Final Architecture Principle

The production implementation should be understood as five cooperating systems:

```text
1. EDGE SAFETY
   Keeps the rover alive.

2. LOCAL GEOMETRIC AUTONOMY
   Decides where the rover can safely move now.

3. GLOBAL TOPOLOGICAL MEMORY
   Decides where the rover has been and how places relate.

4. SEMANTIC SEARCH INTELLIGENCE
   Decides what is worth observing next.

5. SDK TRANSPORT + OPERATIONS
   Provides rover connectivity, mission lifecycle,
   video, telemetry, monitoring and remote control.
```

The correct dependency direction is:

```text
Mission Intelligence
        ↓
Global Planner
        ↓
Local Planner
        ↓
Safety Arbiter
        ↓
SDK Transport
        ↓
Rover
```

Never:

```text
LLM / VLM
   ↓
motor command
```

and never:

```text
LTE
   ↓
safety decision
```

---

# 58. Repository Integration Checklist

Before opening the implementation PR:

- [ ] Add `autonomy/` package.
- [ ] Add `docs/RESILIENT_AUTONOMY_PRD.md`.
- [ ] Add `AUTONOMY_ENABLED=false` to configuration documentation.
- [ ] Add autonomy endpoints behind feature flag.
- [ ] Preserve current `/control` behavior.
- [ ] Preserve current watchdog.
- [ ] Add command TTL layer above SDK transport.
- [ ] Add sensor-health abstractions.
- [ ] Add replay framework.
- [ ] Add safety event logging.
- [ ] Add unit test suite.
- [ ] Add network impairment tests.
- [ ] Add dashboard status indicators.
- [ ] Add optional ML dependencies separate from core SDK dependencies.
- [ ] Add ROS2 topic mappings.
- [ ] Document edge-compute requirements.
- [ ] Document that SDK-side safety is not a substitute for firmware/hardware safety.
- [ ] Complete hardware validation before enabling autonomous outdoor operation.

---

# 59. Reference Links

## Frodobots SDK

- Repository: https://github.com/frodobots-org/earth-rovers-sdk
- README: https://github.com/frodobots-org/earth-rovers-sdk/blob/main/README.md
- Main application: https://github.com/frodobots-org/earth-rovers-sdk/blob/main/main.py
- Video feed: https://github.com/frodobots-org/earth-rovers-sdk/blob/main/video_feed.py
- Telemetry hub: https://github.com/frodobots-org/earth-rovers-sdk/blob/main/telemetry_hub.py
- Requirements: https://github.com/frodobots-org/earth-rovers-sdk/blob/main/requirements.txt
- Docker: https://github.com/frodobots-org/earth-rovers-sdk/blob/main/Dockerfile

## Candidate autonomy components

These are implementation candidates and must be benchmarked on the target hardware/environment:

- Depth Anything: https://github.com/DepthAnything/Depth-Anything-V2
- Depth Anything 3: https://github.com/ByteDance-Seed/Depth-Anything-3
- YOLOE: https://github.com/THU-MIG/yoloe
- YOLO-World: https://github.com/AILab-CVC/YOLO-World
- FAISS: https://github.com/facebookresearch/faiss
- DINOv2: https://github.com/facebookresearch/dinov2
- CosPlace: https://github.com/gmberton/cosPlace
- ROS2: https://github.com/ros2/ros2

---

# 60. Final Product Requirement

The product is successful when the rover can execute a bounded-risk autonomous exploration mission in an outdoor environment where:

- network latency changes over time;
- remote semantic compute is intermittently unavailable;
- images contain motion blur and illumination changes;
- terrain contains roughness, slopes and uncertain geometry;
- wheels can slip;
- the rover can become stuck;
- the camera can become degraded;
- semantic object candidates may be false positives;

and the system still:

1. keeps safety decisions local;
2. avoids stale remote commands;
3. degrades gracefully;
4. retains a useful topological memory;
5. uses local metric geometry for immediate motion;
6. can search for arbitrary semantic objects;
7. records enough evidence to reconstruct failures;
8. remains backward-compatible with the existing Frodobots SDK.

**Target architectural statement:**

> **Semantic Global Intelligence + Metric Local Autonomy + Edge Safety + Uncertainty-Aware Planning + Graceful Network Degradation**

This is the guiding architecture for the implementation in `earth-rovers-sdk`.
