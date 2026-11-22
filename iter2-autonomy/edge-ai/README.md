# QNX edge AI with an independent LiDAR guard

This demo runs native YOLOX object detection and MiDaS relative-depth estimation
on the Raspberry Pi 5 under QNX, using **QNX's ONNX Runtime 1.24.4 APK**. The
phone supplies camera, LiDAR, inertial readings and ARKit pose; it no longer runs
the object detector. The mounted phone is the display; the Pi runs headless.

## Independent processes

- `lidar_guard.cpp`: a measured-LiDAR clearance decision plus an optional
  conservative inferred-obstacle veto, a 100 Hz monotonic watchdog, bounded
  input parsing and a 20 Hz status heartbeat. It requests SCHED_RR priority 20
  on CPU 3 and reports whether those settings were actually applied.
- `rgb_broker.py`: retains one latest raw RGB frame. Each AI consumer pulls it
  independently; a slow consumer does not create a frame backlog.
- `ai_detector.cpp`: YOLOX-Nano, one inference thread, ONNX Runtime CPU backend.
- `ai_depth.cpp`: MiDaS-small, a separate one-thread inference process. Its
  output is **relative inverse depth, not meters**.
- `inference_worker.py`: bounded native-worker input/results, restart on failure,
  and latest-frame selection. YOLO and MiDaS run in separate adapter processes.
- `headless.py`: telemetry, accumulated mapping and the phone return channel.
  It cannot grant clearance when the native guard is stale, disconnected or
  denies permission. A separate bounded TCP worker sends leased phone commands
  to the native motor server.
- `tars_motor`: the repository's Uno/Pico/stub motor server, with a native
  LiDAR gate in its 50 Hz control loop. The installed service explicitly uses
  **stub** mode while no motor controller is attached. The guard sends a separate
  100 Hz localhost heartbeat; an absent heartbeat expires after 100 ms.

The guard never waits for the Python dashboard, RGB broker, ONNX Runtime,
YOLO or MiDaS. Expired inference falls back to the measured-LiDAR decision;
missing or stale LiDAR still denies motion. It shares physical USB, OS and power with them; this does not make
the complete robot a certified or failure-proof system. The command path is
integrated, but physical motor delivery/braking, a balancing loop, and validated
stopping-distance parameters are not established by the stub tests.

## Guard behavior

Input is calibrated world-space measured depth with phone pose. A provisional
forward envelope checks obstacles 0–0.6 m ahead, within ±0.45 m sideways and
0.08–1.2 m above the floor. CLEAR additionally requires measured floor returns
in each of three lateral bands across the forward corridor. Missing calibration, tracking, coverage, invalid
packets, excessive growing input backlog or a disconnect deny permission.
Samples older than 150 ms by **Pi receipt time** become STALE. Receipt time is
not absolute sensor acquisition latency; phone/Pi clocks are not synchronized.
The native loop records tick lateness and missed periods. It never spins while
waiting for data. No inferred points enter the measured LiDAR binary protocol.
A separate short-lived inferred-obstacle message can veto CLEAR; it cannot
override a measured stop, establish free space, or compensate for missing LiDAR.

**Corridor size flags.** `--half-width <m>` (0.05–1.0, default 0.45) and
`--stop-distance <m>` (>0.15–2.0, default 0.6) set the envelope above. The
three floor bands are thirds of the corridor and the map-supported floor cells
span 0.15 m..stop distance, so both shrink with it. The inferred-obstacle stop
uses the same stop distance. The status JSON reports `half_width_m` and
`stop_distance_m`. Without the flags behaviour is unchanged. The values are meant
to come from the car size in `iter2-autonomy/goto/goto.conf` (`car_length_m`,
`car_width_m`, `safety_margin_m`, `guard_stop_m`), so the planner and the guard
agree. `goto -c goto.conf --guard-flags` prints the matching flags.

These dimensions and coverage rules are prototype values. Mounting extrinsics,
robot footprint, blind spots, slopes, drop-offs and stopping dynamics still need
validation before hardware motion can use a clearance decision.

## Build and run

All sources and model artifacts are saved locally before transfer to QNX.
On the Pi, from the repository/deployment root:

```sh
sh iter2-autonomy/edge-ai/setup-qnx.sh
sh iter2-autonomy/edge-ai/ai-build.sh
sh iter2-autonomy/edge-ai/build-guard.sh
make -C iter1-wasd/pi native
python3 iter2-autonomy/dashboard/service.py start
```

`service.py start` is detached from SSH and defaults to headless. It starts the
USB daemon, six local USB proxies, native motor server and guard, RGB broker,
both AI workers and headless dashboard. It must be started again after a Pi reboot. Stop it with
`python3 iter2-autonomy/dashboard/service.py stop`. A monitor and laptop are unnecessary
while the service runs. Keep the phone app open and sensors started.

The dependency installer uses the configured signed QNX package repository,
not Linux packages. Installed inference/runtime headers came from
`https://repo.oss.qnx.com/8.0.5/extra` (`onnxruntime` and `onnxruntime-dev`,
version `1.24.4-r0`). The current packaged runtime emits duplicate ONNX schema
registration diagnostics at startup; observed inference succeeds despite these
messages. Tests capture stderr to a file to avoid a QNX pipe deadlock under the
large diagnostic burst. Global libraries have not been patched or replaced.

## Phone views and 3D reconstruction

- **Camera:** smooth local RGB/LiDAR pair, independent of AI; a separate QNX
  detection card uses the exact cached RGB/depth frame corresponding to results.
  A dedicated **QNX RGB depth estimation** card shows the source-matched RGB and
  model heatmap, model time, source age and calibration quality. Uncalibrated
  colors show relative ordering only; an accepted LiDAR fit gives the model
  predictions a meter scale. The diagnostic heatmap remains model output;
  measured LiDAR overrides predictions in the fused map.
- **All tabs:** guard-driven background and banner. Green means fresh measured
  clearance; amber warns of a forward obstacle within 1.2 m or unknown clearance;
  red means blocked at the provisional 0.6 m stop zone; gray means stale or
  disconnected. The phone display is telemetry and never controls stop timing.
- **3D map:** warm near points through cool far points (0–6+ m from the phone),
  bright measured points and faint inferred points, estimated phone pose,
  remembered object markers, and a level floor plane/grid with orbit/zoom controls.
  Before floor calibration a visual-only plane sits 0.25 m below the phone and is
  labelled assumed. It does not set floor readiness, free space or guard permission.
  A top-down measured-only planning map
  remains available under the disclosure. The render is a bounded spatial sample
  of stored geometry, not a Gaussian splat or photorealistic reconstruction.
  The minimap supports pinch zoom, drag pan, zoom buttons and Recenter. Purple
  cells are temporary inferred obstacles; orange cells are measured obstacles.
- **Pi status:** native guard verdict, sample age, tick timing and actual scheduler
  settings; model timing; Pi-received motion and stream metrics.

ARKit supplies pose in a local gravity-aligned coordinate frame. It can drift,
lose tracking or need relocalization; it is not surveyed/global robot position.
Camera-to-robot extrinsics remain uncalibrated. Saved maps require returning to
the same room and successful ARKit relocalization.

The map remembers up to 100 object locations for the current AR session, even
before floor calibration. Labels show last-seen age. Association is nearest
same-class detection within 0.6 m, not reliable object identity or motion tracking.
Old locations can therefore remain after an object moves. Saved objects return
only with the matching saved AR coordinate frame and remain stale until observed
again. Repeated cached detector results do not refresh their observation age.
Apple describes the environment-dependent restore requirements in
[Managing session life cycle and tracking quality](https://developer.apple.com/documentation/ARKit/managing-session-life-cycle-and-tracking-quality).

## From scanning to robot navigation

The current obstacle response is **stop**: blocked, unknown, stale or missing
guard data cancels drive intent. It does not choose an unobserved reverse or
turn. Selecting a remembered object still previews a route; it does not execute
one. A level display plane is not evidence that every surface is traversable.

To make target-directed motion functional on the chassis:

1. Mount the phone rigidly and measure its pose/height relative to the robot,
   including which camera heading is the robot's forward direction. Set the
   actual floor and validate pose tracking through turns and relocalization.
2. Connect and flash the intended Uno/controller, verify direction and stop
   commands with wheels lifted, and calibrate wheel diameter/track and travel.
   The pulled branch supports 28BYJ-48 stepper wheels; verify that configuration
   against the actual hardware before enabling the hardware backend.
3. Add a path-following controller that turns a selected route into short leased
   commands, tracks progress using pose/odometry, and replans or stops when
   measured geometry changes. The native guard remains the final stop gate.
4. Measure braking distance, latency and footprint at the intended speed. Validate
   blind spots, floor edges, stairs, slopes, moving obstacles and tracking loss;
   tune the provisional corridor and threshold accordingly. A two-wheel balancing
   chassis additionally needs a separate fast attitude/balance controller.

These steps are chassis integration and validation work, not capabilities
implied by a green dashboard background.

For RGB depth, the phone matches MiDaS results to the original cached frame and
fits `1 / measured_depth = scale * relative_inverse_depth + offset`. It requires
sufficient high-confidence LiDAR samples, non-degenerate variation, a positive
scale, and a held-out 90th-percentile error of at most 0.25 m. Poor fits are
rejected. Direct LiDAR wins; inference fills only missing depth, within 0.2–8 m,
with at most 256 points per result. Results older than two seconds are rejected.
Estimated points are displayed separately and are removed near measured returns.
A measured ray passing through a prior inferred point to a farther surface can
also remove that estimate. This does not clear an entire ground column. Direct
measured geometry and persistent occupancy retain their own evidence.

Fresh calibrated estimates above the floor add a temporary blocked-cell layer
to path previews and may send a forward-obstacle veto to the native guard.
They never create free cells, remove measured obstacles, or authorize movement.
Only frames explicitly marked `usage=conservative_obstacle_veto` participate.
The active layer expires two seconds after its original source observation;
repeated cached results cannot renew it. Historical inferred display points can
remain up to 30 seconds and are distinguished from currently active blockers.
Estimated geometry is not saved as measured geometry. Extrapolation outside
LiDAR coverage remains uncertain even when the near-range fit passes.

`SceneFeed` publishes a bounded, nonblocking FUS1 advisory on UDP5569. Its lease
is at most 400 ms and never outlives the original observation. The native guard
polls at most eight messages per tick without waiting. It combines a fresh
inferred obstacle within 0.6 m with its LiDAR result by adding a stop; if the AI
or publisher stops, the advisory expires and LiDAR-only operation continues.
The motor still requires a new command after a stop clears. This is conservative
fusion, not a calibrated replacement for the LiDAR clearance requirement.

## Ports and bounded contracts

All Pi listeners/proxies bind localhost.

| Port | Purpose |
|---|---|
| 5555 | Local native motor command server, guarded stub by default |
| 5557 | Phone → Pi motion NDJSON |
| 5558 | Optional paired preview JPEG; headless validates and discards |
| 5559 | Pi → phone status/results; phone commands back |
| 5560 | Pi service administration |
| 5561 | Phone → Pi world observations at up to 5 Hz |
| 5562 | Phone → native guard measured LiDAR, target 20 Hz |
| 5563 | Phone → broker unannotated RGB JPEG, no fixed FPS timer; one packet outstanding |
| 5564 | Native guard → dashboard UDP status |
| 5565 | YOLO worker → dashboard UDP result |
| 5566 | Local latest-frame broker for both AI consumers |
| 5567 | MiDaS worker → dashboard UDP result |
| 5568 | Native guard → native motor gate, independent of dashboard/AI |
| 5569 | Mapping → native guard, optional inferred-obstacle veto |
| 5573 | TARS speech (goto/voice → phone): `SAY <text>` lines, phone speaks them; see `iter2-autonomy/iphone/README.md` |
| 5577 | Robot mic (phone → `tars_voice`): AirPods audio as MIC1 packets; the phone listens, `tars_voice` dials; see `iter2-autonomy/iphone/README.md` |

5562 uses a four-byte big-endian length, then `LDR1`, sequence u32, source uptime
microseconds u64, flags u32, count u32, pose `[x,y,z,yaw]` f32, floor_y f32 and
count XYZ f32 triples; all values big-endian, max 1,024 points / 12,332 bytes.
Flags: normal tracking=1, floor known=2, valid horizontal heading=4. ACK 0x01
follows validation; the phone allows one outstanding packet with a 150 ms ACK
timeout. Source sequence/time must increase within each connection.

5563 carries a big-endian length followed by `RGB1`, source-frame ID u64 and
JPEG bytes. Source-frame ID is phone uptime microseconds. The bounded phone
cache holds up to three seconds of frames, capped at 90. Broker ACK means accepted into the latest-frame slot;
it does not wait for inference. Native worker contracts are in `ai-protocol.md`.

5569 carries 24 bytes: `FUS1`, generated Pi `CLOCK_MONOTONIC` microseconds u64,
expiry on that same clock u64, and nearest forward inferred obstacle f32; all
numbers are big-endian. A nearest value of -1 means no obstacle. Expiry zero
with nearest -1 explicitly invalidates the contribution. The guard rejects
future/replayed/old timestamps, nonfinite values and leases longer than 500 ms.
Private tests disable fusion publishing by default or use their own ports.

## Validation and demo

```sh
python3 iter2-autonomy/edge-ai/ai-test.py
python3 iter2-autonomy/edge-ai/ai-depth-test.py
python3 iter2-autonomy/edge-ai/test_guard.py
python3 iter2-autonomy/edge-ai/test_edge_transport.py
python3 -m unittest discover -s iter2-autonomy/dashboard -p 'test_*.py'
python3 iter2-autonomy/edge-ai/test_isolation.py
python3 iter2-autonomy/edge-ai/test_isolation.py --with-depth
MOTOR_BINARY=iter1-wasd/pi/build/tars_motor python3 iter2-autonomy/edge-ai/test_robot_stop.py
```

66 automated tests passed on QNX (4 YOLO, 3 MiDaS, 11 guard, 17 transport,
31 dashboard/map). The phone's pure calibration helper also passed six synthetic
checks. Device and simulator builds succeed; the 3D fixture was visually checked.
The native YOLO test image produced dog/bicycle/car detections in 147–155 ms per
inference, with one inference thread; these are sample timings, not WCET bounds.

The first actual QNX isolation run used **synthetic 20 Hz LiDAR** and the real
YOLO model. The guard remained CLEAR and advanced while inference ran, was
SIGSTOPped and was SIGKILLed. It reported priority 20, CPU mask 8, zero scheduler
errors, zero missed ticks and 0.205 ms maximum tick lateness in that run. Stopping
sensor packets (without closing TCP) produced an observed STALE heartbeat after
206 ms, including status publication/observation delay. This proves the tested
process-separation behavior, not physical LiDAR accuracy or a hard deadline for
all possible workloads.

## Challenge provenance

- [QNX open-source ports](https://github.com/qnx-ports/build-files) includes the
  [ONNX Runtime port](https://github.com/qnx-ports/build-files/tree/main/ports/onnxruntime).
- [QNX open-source delivery guidance](https://github.com/qnx) documents QNX APKs
  and QNX-maintained ports.
- `models/README.md` records official YOLOX/MiDaS sources, exact downloads,
  checksums, preprocessing and upstream licenses. Both model workers run on QNX;
  there is no cloud or Mac inference in the running demo.

### Combined-load and live-transport results

The combined QNX isolation run also passed with real YOLO and MiDaS processes
running together. Each was separately SIGSTOPped and SIGKILLed while synthetic
20 Hz LiDAR continued. The guard reported zero missed ticks, 0.528 ms maximum
tick lateness, and an observed STALE heartbeat 196 ms after LiDAR silence.
The actual report is saved in `validation/qnx-isolation-with-depth.json`.

Live phone feeds then reached both QNX models: representative YOLO inference
was 142–162 ms and MiDaS inference 789–849 ms, with the native guard reporting
100 Hz, roughly 20 new sensor frames/s and no missed ticks in that observation.
These are observed timings, not a guarantee under all workloads. Pose tracking,
floor calibration, point availability and RGB depth-fit quality still depend on
the actual scene and must be checked on the phone.

A QNX UDP default of 9,216 bytes initially rejected the larger depth results.
`probe_udp.py` reproduced EMSGSIZE at 40/60 KB. Per-socket SO_SNDBUF/SO_RCVBUF
increases delivered both sizes intact without changing global networking
settings. Both workers and the collector now use bounded 256 KiB socket buffers,
retain a 60,000-byte message cap, and report send failures instead of silently
ignoring them. The depth result is live on the real return channel after this fix.

### Motor integration validation

The motor integration was built with native QNX clang and deployed in guarded
stub mode. The updated phone app was built, installed and its amber/red map
fixtures visually checked. The imported motor suite passed 241 checks on macOS,
including the Uno serial backend against a fake controller; 21 focused serial
checks also passed after the final serial-stop ordering change.

On QNX, 63 targeted tests passed: 8 native motor gate/phone bridge tests, 3 real
guard-to-motor integration tests, 12 guard tests, and 40 dashboard/mapping tests.
The saved [QNX stop report](validation/qnx-robot-stop.json) uses synthetic LiDAR
and the actual native stub backend. Observed zero-output times were 55 ms for an
obstacle, 165 ms for sensor silence, 22 ms for unknown tracking/calibration,
44 ms for sensor disconnect, and 99 ms for a SIGSTOPped guard. Old leased drive
commands remained rejected after clearance returned. These are individual
software observations, not physical braking times or worst-case bounds.

See [motor guard setup](../../iter1-wasd/pi/MOTOR_GUARD.md) for the independent heartbeat,
generation-aware command contract, explicit hardware configuration, and the
remaining motor/controller bench work. The phone's background updates from
telemetry at up to 5 Hz; the 100 Hz guard and 50 Hz motor loop do not wait for it.

### Fused-depth validation

The updated relative/metric depth card was built and installed on the iPhone;
both synthetic display modes and the inferred-stop banner were visually checked.
The Pi rebuilt the native guard and passed **78 targeted tests**: 19 guard/fusion,
54 dashboard/mapping/transport, and 5 native guard-to-motor integration tests.
The tests cover original-observation expiry, LiDAR endpoint/ray overrides,
temporary inferred route blockers, invalid/replayed messages, measured-stop
precedence, inference-only stops, and no automatic resumption of old commands.

In [the QNX fusion stop report](validation/qnx-depth-fusion-stop.json), a
synthetic inferred obstacle caused zero native stub output after 22 ms. This
measures advisory arrival to commanded stop; it excludes camera acquisition and
the roughly 0.8-second model inference. It is not a physical braking time.
Real-scene depth-fit quality and inferred obstacles must still be checked with
the mounted phone in the intended environment.

After the updated phone reconnected, live QNX telemetry showed normal tracking,
a calibrated floor, 4,665 measured voxels, 354 inferred display voxels and eight
remembered objects in one snapshot. The optional inferred channel became live,
and MiDaS timings were approximately 802–841 ms. The guard reported zero missed
ticks in that observation. This verifies the live calibrated transport/fusion
path, not scene-wide depth accuracy or physical obstacle stopping.

### Remembered calibration and distant visual memory

The distant mapping experiment separates longer-lived visual geometry from the
short-lived obstacle input. Measured LiDAR landmarks can anchor later relative
depth frames after reprojection into the current camera. Reuse is approximate:
viewpoint, texture agreement, occlusion checks, tracking and anchor age limit it.
A brief cached-calibration fallback is explicitly uncertain and visual only.

The visualization accepts finite positive predictions out to 20 m rather than
clamping invalid/infinite predictions onto a wall. A separate bounded distant
layer retains observations for up to 120 seconds, with fading display confidence.
LiDAR endpoints and measured clear rays override it. This layer never creates
free space, updates path planning, or grants motor clearance. It is not stored
as measured geometry. A longer display range is not an accuracy specification.

To seed the experiment, slowly scan nearby textured surfaces at several depths,
then look toward more distant surfaces while maintaining tracking. Pure rotation
cannot reveal hidden surfaces, and textureless walls or major viewpoint changes
may leave too few reliable anchors. The phone reports the calibration source
and uncertainty; fresh LiDAR still takes priority as the robot approaches.
