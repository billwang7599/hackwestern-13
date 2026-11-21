# Mounted iPhone sensors and robot dashboard

The current architecture uses the iPhone as a sensor/display and runs YOLO and
MiDaS inference on the QNX Pi. See [the edge-AI guide](../pi/edge-ai/README.md)
for protocols, separation, build instructions, model provenance and validation.
The prior bundled Apple Core ML model remains a local prototype artifact but is
not loaded or run by the current camera pipeline.

## Install

Build `MotionUSB.xcodeproj`, select the existing signing team and install from
the Mac to the unlocked, trusted phone. The target is iOS 17 or later; LiDAR
requires a supported Pro device. Then connect USB to the Pi, open MotionUSB and
tap **Start sensors**. Keep the app foregrounded; backgrounding stops sensors.

```sh
DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer xcodebuild \
  -project iphone/MotionUSB.xcodeproj -scheme MotionUSB -sdk iphoneos \
  -derivedDataPath /tmp/hw13-motion-build -allowProvisioningUpdates build
```

## Use

**Camera** shows a fast local RGB/depth pair, and a separate source-matched QNX
object-detection result. Warm depth colors are near, cool colors far, and black
is missing or low-confidence depth. Green detection boxes have valid measured
depth; yellow boxes are 2D detections without usable depth. The local view targets
10 Hz without waiting for AI. Its measured rate is shown; actual throughput is
hardware/load dependent.

The RGB sender for QNX AI has no fixed FPS timer. It offers the newest camera
frame whenever encoding and the previous USB acknowledgement permit; busy
frames are dropped. Camera acquisition prefers 30 Hz. The Pi keeps only the
latest frame, and each model processes it at its own speed. This does not imply
30 FPS inference. The local preview remains separately limited to 10 Hz.

The separate **QNX RGB depth estimation** card always shows the exact RGB frame
used by MiDaS beside its 64×64 depth prediction. Without a usable LiDAR fit, it
shows **relative depth, no meter scale**: warm pixels are relatively nearer and
cool pixels farther, normalized independently for each frame. Missing LiDAR,
limited tracking or a rejected fit does not hide that diagnostic output. Flat
predictions display gray. Relative output alone produces no metric map points
or obstacle veto.

The depth card reports its scale mode and heuristic confidence:

- **Direct:** a robust fit to source-matched current LiDAR samples. This is the
  only fit that can produce fresh conservative obstacle-veto points, limited to
  8 m. It also supplies visualization points out to 20 m when the fitted inverse
  depth is positive and within range.
- **Anchors:** old measured LiDAR points are reprojected into the current RGB
  frame using its own intrinsics and pose. A fit needs at least 40 distinct
  model pixels spanning multiple depths; one nearby anchor cannot determine
  scale and offset. A textured RGB patch must agree, the viewpoint must remain
  within 0.5 m/20°, and any current LiDAR disagreement rejects the anchor. These
  are approximate correspondences, not verified static scene matches. The
  result is **visual memory only**, never obstacle clearance or motor input.
- **Cached:** an accepted direct/anchor fit can be reused for at most three
  seconds from the original fitted frame, within 0.12 m/6°, while RGB appearance
  and the full relative-prediction distribution/layout remain consistent. A
  cached frame never renews the original fit timestamp. This mode is also
  **visual memory only** and its heuristic confidence decays.
- **Relative:** no accepted scale is available. The actual model output remains
  visible without inventing metric positions.

Start by slowly scanning nearby textured surfaces at varied distances, then
look farther while keeping some nearby features in view. Measured anchor memory
is bounded to 4096 spatial cells and 30 seconds; collection runs at most 5 Hz on
its own RGB queue. Tracking loss, session changes, reset, stop and relocalization
clear anchor and cached-fit state. Old anchors beyond those viewpoint/appearance
limits cannot provide scale just because the robot saw them earlier.

The diagnostic heatmap shows actual model predictions from 0.2–20 m when a scale
is accepted; this display range is not a validated 20 m sensor range. Black
means invalid or out of range. Zero/negative inverse depths and values beyond
20 m are rejected, never clamped into a distant wall. Measured LiDAR overrides
inferred geometry in the fused map, not this diagnostic raster. The card shows
model inference time, source-frame age, fit sample count/held-out error, scale
mode, heuristic confidence and anchor age. Frames older than two seconds are
visibly stale and do not produce new geometry. Source matching retains up to
three seconds of source frames, with a hard limit of 90; unmatched results are
discarded instead of being painted onto the live camera.

**3D map** shows accumulated measured and calibrated inferred points, with
distance coloring and a near/far legend. Drag to orbit, pinch to zoom and use
Recenter to return to the phone pose. Object markers show remembered world
positions, labels and observation age; they are not predictions of where a
moving object is now. Violet distant-memory points are uncertain geometry from
`persistent_points`, separate from safety input. The Pi retains at most 5000
such cells for 120 seconds, decreases confidence with age, and requires fresh
source timestamps for corroboration. This is temporary geometric memory, not a
surveyed map or a surface-reconstruction guarantee. The floor map can be zoomed
and panned separately.

The floor is modeled as one horizontal plane in ARKit's gravity-aligned world.
Until a classified floor or **Set floor** measurement is available, the display
shows an explicitly assumed plane 0.25 m below the phone. That provisional
mounting height is only a visual reference: it never marks space free, enables
path planning or permits motor commands. Slowly scan the room and floor. If
floor classification does not complete, aim the center at clear ground and tap
**Set floor**. The optional target selector previews routes only; selecting a
remembered object does not drive the robot to it.

The Pi remembers up to 100 located objects for the current AR session, matching
the same class within 0.6 m and retaining the latest measured position. This is
simple spatial association, not reliable instance identification. Labels remain
visible after the camera turns away. A target becomes stale for planning after
5 seconds without a new observation; repeating a cached inference result does
not reset that age. Loss of tracking or the scene connection also marks targets
stale. New AR sessions, clearing the map, or a substantial floor recalibration
clear that memory. Inferred depth never supplies object positions: located
objects require source-matched measured depth.

RGB-derived depth is approximate. Bad LiDAR calibration fits are rejected; a
passing near-range fit does not prove accuracy far away. Fresh, accepted
**direct-fit** inferred points may add conservative blocked cells to the minimap/path preview and veto
motor motion; they never mark space free or grant clearance. Reliable measured
LiDAR replaces inferred geometry. Purple minimap cells are active inferred
obstacles; orange cells are measured obstacles. Faint points retained in the 3D
view are visual history and are not necessarily active obstacles. ARKit pose is
a drifting local estimate, not absolute surveyed robot position. Mounting
extrinsics remain uncalibrated.

**Pi status** shows headless mode, all Pi-received motion values, rates, counts,
gaps and receipt age, plus native guard timing and QNX model inference timing.
The phone local preview and Pi-received data are deliberately identified.
Receipt age does not measure absolute sensor-to-Pi delay. Debug commands go to
the native Pi motor gate; the deployed stub backend exercises commands without
physical output. Hardware mode must be configured explicitly. Drive is denied
when the measured LiDAR guard is stale/not CLEAR, and a fresh calibrated inferred
obstacle may also stop it. AI can never override a LiDAR denial. The background
and banner report clear, caution, blocked, or unavailable status, with inferred
stops explicitly identified; they are a display of the decision, not the safety
control loop.

Save surroundings saves a phone ARWorldMap and asks the Pi to save measured
geometry and remembered object locations. Check both messages. The Pi only
restores a saved map after the phone reports the matching saved AR session with
normal tracking. Loading requires looking at the original room until ARKit
relocalizes; restored object labels are historical and must be reobserved before
planning. Reset scan changes the AR
coordinate frame, which clears Pi geometry. Pi stream Stop/Start preserves the
map in its receiver process, but process restart or GUI/headless switching loses
unsaved observations. Estimated geometry is not saved as reliable structure.

## Headless operation

Run `python3 pi/dashboard/service.py start` on the Pi. It detaches from SSH and
needs no monitor or laptop afterward, but must be restarted after a Pi reboot.
The phone remains USB-connected to the Pi. The six phone TCP listeners have no
application authentication; the intended transport is local USB forwarding.

## Visual checks

Build for the simulator with `SWIFT_ACTIVE_COMPILATION_CONDITIONS=DEBUG`, then
launch with `--preview-fixture` (camera) or `--preview-fixture --map` (3D map).
Add `--blocked`, `--inferred-stop`, `--clear`, `--unknown`, or `--stale` to
exercise obstacle colors. The camera fixture includes a synthetic relative-depth
card; `--depth-only` brings that card to the top, `--depth-metric` switches it to
a synthetic calibrated example; `--depth-anchors` and `--depth-cached` show the
visual-memory modes. `--depth-stale` verifies stale styling. Fixtures are synthetic and do not validate
hardware. Normal installed builds and launches do not activate them.

Run `python3 iphone/test_depth_geometry.py` on the Mac for pure projection,
calibration, occlusion, texture correspondence, cache-range and RGB-orientation
checks against the actual Swift helper code. These checks do not establish
real-scene correspondence or far-range accuracy.

### Manual driving and keyboard diagnostics

The Pi status tab has W/A/S/D buttons and STOP. Each direction tap explicitly
selects phone control, then requests one 500 ms pulse at 25% command speed.
The native LiDAR gate still has final authority. Buttons do not auto-repeat.
The input check panel echoes Pi keyboard held keys, last press/release, event
count, control owner, requested wheel commands, native output commands, stop
reason, serial device presence, and Arduino READY count. Last press persists
across release so a quick tap remains visible at the 5 Hz status update rate.
A serial node or historical READY count does not prove a working motor or
wheel movement; there are no encoder measurements in this diagnostic.
