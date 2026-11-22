# Mounted iPhone sensors and robot dashboard

The current architecture uses the iPhone as a sensor/display and runs YOLO and
MiDaS inference on the QNX Pi. See [the edge-AI guide](../edge-ai/README.md)
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
  -project iter2-autonomy/iphone/MotionUSB.xcodeproj -scheme MotionUSB -sdk iphoneos \
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
Until a classified floor or **Set floor** calibration is available, the display
uses the mounted phone to estimate the plane. The power-button edge is uppermost and its
midpoint is about 152.4 mm above the ground. In the upright mount the main camera
is about 138 mm high and the LiDAR scanner about 88 mm high. Pitch and roll project
the in-plane offsets onto gravity. These estimates use the housing dimensions and
omit the small out-of-plane optical offsets and case thickness. Tap **Set floor** with the rover resting on the intended floor and the phone in its
normal driving position. The app subtracts the tilt-adjusted camera height from the
current tracked camera altitude and locks that world-space floor elevation. The
button works while looking forward, without a center-pixel depth sample. Save map
stores that elevation. Automatic floor detection operates until a floor is explicitly set. The Controls panel displays both heights.
[Apple mechanical drawing, sheet 1, detail D](https://developer.apple.com/download/files/accessories/dimensional-drawings/iphone-17-pro-max.pdf).

The phone's YOLO and measured LiDAR tiles and the USB preview use a virtual level
camera at the captured camera position. Each image and its annotations share the
original frame's gravity-aligned ARKit pose and intrinsics. Looking steeply down
keeps the source view for inspection. Detection, depth backprojection and
the QNX guard use their original registered observations. The provisional
mounting height is only a visual reference: it never marks space free, enables
path planning or permits motor commands. Once confirmed, the floor is available to mapping and the guard; obstacle checks still require fresh observations. The optional target selector previews routes only; selecting a
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

Run `python3 iter2-autonomy/dashboard/service.py start` on the Pi. It detaches from SSH and
needs no monitor or laptop afterward, but must be restarted after a Pi reboot.
The phone remains USB-connected to the Pi. The eight phone TCP listeners (including the
speech link on 5573 and the robot mic on 5577) have no application authentication; the intended transport is local USB forwarding.

## Speech link (5573)

The robot has no speaker, so the phone speaks for it. `SpeechLink.swift` listens
on TCP 5573. `service.py` forwards Pi `127.0.0.1:5573` to the phone over USB, so
goto/voice on the Pi just connect to localhost. It accepts up to 8 clients at
once; each sends newline-delimited UTF-8 lines:

| Line | Effect |
|---|---|
| `SAY <text>` | queue `<text>` (normal priority); `SAY` is case-insensitive |
| `{"type":"say","text":"...","priority":"normal\|urgent"}` | same; `priority` is optional (default normal) |
| `HUSH` or `{"type":"hush"}` | stop speaking now and clear the queue |

Each line gets a one-line reply: `OK say`, `OK hush` or `ERR <reason>`. Senders
may ignore it. Blank lines get no reply.

- **Queue:** normal requests wait their turn; past 5 pending, the oldest is
  dropped. **Urgent** cuts off the current sentence and replaces everything
  pending (a stale sentence never follows "Stopping.").
- **Limits:** lines over 1024 bytes are dropped whole (`ERR line too long`);
  text is cut to 300 characters on a word boundary; control characters become
  spaces. Invalid UTF-8, unknown JSON types and other garbage get `ERR`.
- **Audio:** `AVSpeechSynthesizer`, `en-US` on-device voice, no network. The
  audio session is `.playback`/`.spokenAudio` with `.duckOthers`, so it should
  speak with the silent switch on; it is deactivated when the queue empties.
- **UI:** the last spoken line shows as a `TARS: …` caption under the guard
  banner on every tab. Like the other listeners it stops when the app
  backgrounds and restarts when it becomes active.

Try it from the Pi (or the Mac with the app in the Simulator):

```sh
python3 iter2-autonomy/iphone/tools/say.py "I remember 2 things: a trash can to my left, and a chair behind me."
python3 iter2-autonomy/iphone/tools/say.py --urgent "Stopping."
python3 iter2-autonomy/iphone/tools/say.py --hush
```

`python3 iter2-autonomy/iphone/test_speech_link.py` compiles the parser, line framer and queue
policy straight out of `SpeechLink.swift` and runs 47 checks on the Mac.
Not yet verified: speech on a real iPhone, the silent-switch behaviour, and the
USB forward from the Pi.

## Robot mic (5577, AirPods)

The robot has no microphone. `PhoneMic.swift` makes the phone on the car its
ear, through your AirPods: their mic goes to the Pi's `tars_voice` (whisper,
"TARS" wake word, intent classifier, stop-first), exactly like a wired mic.

**Turn it on:** pair the AirPods with the phone, open the app, tap the
controls button (sliders icon) and switch on **Robot mic (AirPods)**. The
first time, allow microphone access. The switch is remembered. On the Pi,
`voice.conf` has `input = phone` (the default) and `service.py` forwards
`127.0.0.1:5577` to the phone. Then say "TARS, go to the trash can".

**Status line** (under the TARS caption): listening / muted (TARS talking) /
interrupted, the input device (e.g. "Bill's AirPods Pro"), Pi linked or
waiting, a level meter, and two switch times: `→spk` (mute to speaker ready)
and `→mic` (end of TARS's speech to the first AirPods audio again). An orange
line warns when the AirPods aren't connected (the phone's own mic is used
then), on a call/Siri interruption, or if permission was denied.

**Half duplex:** iOS keeps a Bluetooth headset's mic and output together, and
forcing the phone speaker also switches the input to the phone mic. So while
TARS speaks a `SAY`, the stream is muted (`M`), the reply plays from the phone
speaker (`overrideOutputAudioPort(.speaker)`), then the AirPods route and the
stream (`U`) come back. A multi-sentence reply stays muted throughout. A
spoken "stop" during TARS's speech is not heard; ✋, the STOP button, the Uno
button and the watchdog still work. When the robot mic is off, the speech link
behaves as before (`.playback` session).

**Wire format** (the phone listens, `tars_voice` dials it, like 5573; one Pi at
a time, the newest connection wins; the Pi never writes back):

| Bytes | Meaning |
|---|---|
| `MIC1 16000 1 s16le\n` | once per connection, then the current state (`U` or `M`) |
| `A` len16LE payload | audio: 16 kHz mono s16le, len even, 2–8192 bytes (the app sends 20 ms = 640) |
| `M` 0 0 / `U` 0 0 | mute (TARS talking, ignore audio) / unmute |
| `K` 0 0 | keepalive, at least once a second when no audio flows |

The Pi drops the link on anything else, or after 3 s with no packet, and
redials (0.5–5 s backoff). Capture: `AVAudioEngine` input tap at the hardware
rate → `AVAudioConverter` (downmix) → 16 kHz s16 → 20 ms packets; a linear
resampler is the fallback if no converter can be made. If the link backs up,
audio past ~2 s in flight is dropped (counted as `drop` in the status).
Session: `.playAndRecord`, `.allowBluetoothHFP` + `.defaultToSpeaker` +
`.duckOthers`, preferred input = Bluetooth HFP. Like the other listeners it
stops when the app backgrounds and restarts when it becomes active.

Without the phone: `python3 iter2-autonomy/iphone/tools/mic_stream.py cmd.wav`
pretends to be the phone (WAV or raw stdin; on the Pi stop the 5577 iproxy
first or use `--port`).

`python3 iter2-autonomy/iphone/test_phone_mic.py` compiles the framing, the
Pi-parser mirror, the PCM/resampler/level math and the half-duplex state
machine straight out of `PhoneMic.swift` and runs 66 checks on the Mac, plus a
byte-for-byte cross-check against the encoder the C parser test uses
(`make -C iter2-autonomy/voice test-phone`). **Not yet verified:** anything on
a real iPhone (AirPods HFP capture, the speaker switch and its timing, route
changes, interruptions, permission prompt) and the 5577 USB forward on the Pi.

## Visual checks

Build for the simulator with `SWIFT_ACTIVE_COMPILATION_CONDITIONS=DEBUG`, then
launch with `--preview-fixture` (camera) or `--preview-fixture --map` (3D map).
Add `--blocked`, `--inferred-stop`, `--clear`, `--unknown`, or `--stale` to
exercise obstacle colors. The camera fixture includes a synthetic relative-depth
card; `--depth-only` brings that card to the top, `--depth-metric` switches it to
a synthetic calibrated example; `--depth-anchors` and `--depth-cached` show the
visual-memory modes. `--depth-stale` verifies stale styling. Fixtures are synthetic and do not validate
hardware. Normal installed builds and launches do not activate them.

Run `python3 iter2-autonomy/iphone/test_depth_geometry.py` on the Mac for pure projection,
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

### Room recognition with AprilTags

Print [room tag 0](room-tag-0.svg) at actual size and measure the black square from outer edge to outer edge. It should be **160 mm** wide. Keep the white margin. Fix it flat on a vertical wall. For a different room, use a different tag36h11 ID printed at the same black-border width. Leave each marker in its original location.

Start sensors and point the rear camera at the wall tag until the dashboard shows its ID. Once ARKit has mapped the room, **Save map** stores the landmark with the existing iPhone world map and requests the usual Pi map save. **Load map** restores that saved room. Revisit familiar features for ARKit relocalization, then look at the same tag for position and heading correction. The app retains one saved room at a time. Older saves without a landmark still load.

The iPhone runs the vendored AprilRobotics AprilTag 3.4.5 detector offline on a separate serial worker, at most 2.5 observations per second. Two consistent reads are required. Matching a tag can correct translation and yaw after normal tracking resumes. Large discrepancies (at least 75 cm or 30 degrees) are reported for inspection. A different ID does not register a correction. The Pi continues receiving poses and geometry in the room frame.
