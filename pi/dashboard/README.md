# Pi dashboard / headless service

The current QNX AI and independent LiDAR architecture is documented in
[../edge-ai/README.md](../edge-ai/README.md). That guide supersedes the earlier
phone-inference architecture described in the hardware history below. The phone
is now the main display, including a 3D point cloud and all Pi telemetry.

Current mapping keeps the measured occupancy grid separate from a temporary
inferred obstacle layer. Only source-matched metric fits explicitly tagged
`usage: conservative_obstacle_veto`, with at least 40 calibration samples,
positive scale and held-out 90th-percentile error at most 0.25 m, can contribute.
They require normal tracking and a measured floor. Their two-second lifetime
starts at the original phone observation, not receipt of repeated cached results.
Snapshot grid cell `3` is an inferred obstacle; persisted grid data remains
measured-only (`0` unknown, `1` observed floor, `2` measured obstacle). Inference
can block an otherwise available path but never create free space.

Current LiDAR endpoints replace nearby estimates. A measured ray may also remove
an estimate within 2.5 cm of that ray when the measured endpoint lies at least
10 cm farther away; this does not clear entire ground columns or measured
obstacles. These tolerances are provisional and need physical validation.

Production headless/GUI scene feeds explicitly enable the optional native-guard
veto on localhost UDP 5569. `SceneFeed` and `FusionPublisher` default to port 0
(disabled), so standalone tests cannot affect a running robot. The binary packet
is `!4sQQf`: `FUS1`, Pi monotonic send microseconds, expiry microseconds, nearest
inferred forward distance in meters. Expiry is capped by both original
observation age and a 400 ms publisher lease. Expiry zero and distance -1 clear
only the inferred contribution; they never grant measured clearance. A valid
fit with no inferred obstacle uses distance -1 with a future expiry.

Run `python3 -m unittest discover -s pi/dashboard -p 'test_*.py'` for mapping,
fusion, transport and command tests. UDP transport tests bind private ports.

# Pi dashboard and mounted-phone display

The native SDL dashboard runs on the Pi's QNX display, with the iPhone camera
on the left and motion readings on the right. It reads the iPhone's
existing MotionUSB stream through localhost port 5557. It shows acceleration,
angular velocity, receipt rate, sequence gaps, and time since the latest receipt.
It marks data stale after 500 ms and reconnects after a stream failure. Old
values remain visible with the stale indicator. This is telemetry, not a motor
controller. Receipt age is not end-to-end sensor latency.

All source is saved in this repository before deployment. Install the updated
MotionUSB app, allow Camera access, and leave it open with Start pressed.
The combined rear-camera and motion stream was hardware verified at about
12 fps video and 100 Hz motion with no motion sequence gaps during the test.
The user confirmed the scene was visible. The selfie-camera build was then
installed and received 109 frames and 897 motion samples with zero gaps in a
10.01-second test including connection startup. The phone status/control and Pi-returned motion display have also been installed.
These earlier video rates are not measurements of the new ARKit pipeline.

## Run on QNX

First start the USB bridge as documented in `../iphone-usb/README.md`.
Do not run `receive_motion.py` at the same time: the phone accepts one motion
client, and a second receiver replaces the first.

From `~/projects/iphone-usb`:

```sh
# In a separate terminal, in addition to the motion proxy on 5557:
pi/iphone-usb/install/bin/iproxy -l -s 127.0.0.1 5558:5558
# Then in another terminal:
python3 pi/dashboard/dashboard.py
```

Dependencies already present on the tested image: Python 3, SDL2 compatibility
library, SDL2_ttf, SDL2_image, and `/usr/share/fonts/dejavu/DejaVuSans.ttf`.
Use `--font PATH` to select another font or `--seconds 10` for a timed test.
Click Start/Stop streams or press Space to connect/disconnect motion, video and scene feeds.
Escape closes the dashboard. It does not stop the USB bridge.
`sh pi/dashboard/run.sh` opens the dashboard paused. The phone camera remains
active while Pi reception is paused; use Stop in MotionUSB to stop capture.

As an alternative to the integrated iPhone view, plug the Logitech into a spare Pi USB port, then run in another
terminal:

```sh
camera_example3_viewfinder -u 5
```

Unit 5 is the USB camera entry in this Pi's
`/etc/config/sensor/sensor_rpi5.conf`; unit 1 is simulated color bars.
The camera uses QNX's existing native viewfinder in a separate window.
Use **Alt+Tab** on the Pi keyboard to switch between camera and telemetry.
Escape closes the focused app. No camera recording is configured.

## Verified hardware

- Logitech C270, USB VID:PID `046d:0825`, detected by QNX and showing a real
  image in the native camera viewfinder (verified by display screenshot).
- iPhone 17 Pro Max motion over direct USB: dashboard received 1,004 samples
  in a 10-second smoke test with no skipped sequence numbers.
- Dashboard display verified by screenshot with live axis values and ~100 Hz.
- The Logitech viewfinder was stopped when switching to the iPhone-camera
  dashboard. No boot services added.

Logitech delivered frame rate and glass-to-glass latency have not been measured. There is
no measured latency advantage over the new iPhone camera stream. A direct webcam avoids app-side iPhone image encoding and
forwarding, but exposure, camera encoding, buffering, and display timing still
determine the actual delay. Comparing both paths requires the same scene and
a repeatable visual timing test, not just comparing nominal frame rates.

## iPhone camera protocol

Port 5558 carries a 4-byte big-endian JPEG length followed by that JPEG.
Frames are limited to 2,000,000 bytes. The Pi acknowledges a decoded/uploaded
frame with byte 0x01; the phone permits only one outstanding frame and cancels
clients that do not acknowledge within two seconds. The Pi retains only one
pending image. A stale image stays visible with a prominent stale label.
The phone now sends a composite of rear-camera ARKit RGB plus its matching
LiDAR heatmap, with labeled detection boxes on the RGB panel. Both panels rotate
to portrait within the side-by-side raster; JPEG quality is 0.7 and the maximum
send rate targets 10 Hz, with source-matched inference and bounded processing. It captures no audio
and records no video. Encoding runs separately from motion sampling. Combined
LiDAR/detector performance and absolute end-to-end latency remain unmeasured.

Run framing tests with `python3 -m unittest discover -s pi/dashboard -p 'test_*.py'`.

QNX setup reference:
https://www.qnx.com/developers/docs/qnxeverywhere/com.qnx.doc.target_images/topic/qsti/next-steps.html

## Mounted phone / no Pi display

The phone stays on the robot and connected to the Pi over USB. MotionUSB shows
a local rear-camera preview plus motion values returned by the Pi, receive rates,
sequence gaps, and connection/stale status. Status updates arrive about twice
per second; motion sampling remains requested at 100 Hz. Port 5559 carries
newline-delimited JSON status and commands, separate from motion and video.

From `~/projects/iphone-usb`:

```sh
python3 pi/dashboard/service.py start
python3 pi/dashboard/service.py status
python3 pi/dashboard/service.py gui       # open Pi window; keep phone status active
python3 pi/dashboard/service.py headless  # close Pi window; keep phone status active
python3 pi/dashboard/service.py stop
```

The gui/headless commands switch the single active receiver. Closing the Pi
window returns the managed service to headless mode. Pi Start/Stop and Space
remain available in the window; phone stream commands work in either mode.

The service runs detached from SSH and manages usbmuxd, six local-only iproxy
ports, and the headless receiver. Stop separate proxies and the graphical
dashboard before starting it; the phone accepts one client per data stream.
The launcher reapplies the runtime USB exception and needs passwordless sudo
as configured on this Pi. Logs are in `pi/dashboard/state/headless-service.log`.
It checks for the phone every ten seconds and restarts the old USB daemon when
no device is found; physical unplug recovery is still experimental. This is
not a boot service: after a Pi reboot, run the start command again.

Headless mode acknowledges JPEG frames after checking their framing/envelope
and discards them; it does not decode, record, or display them. Its measured
video rate therefore is not a display FPS benchmark. No monitor or persistent
laptop connection is needed once the service has started. Keep the phone app
foregrounded; opening another app stops its streams.

Phone Pi Start/Stop buttons control reception while the status channel remains
connected. Debug drive buttons send simulated left/right intent only. There is
no motor backend attached to this service. Intents expire after 500 ms or on
status-link disconnect; Halt immediately zeros them. The repo's separate motor
server expects a Pico/UART backend, not the previously described Grove I2C
driver, so actual hardware control remains separate work.

28 tests passed locally and on QNX: framing limits/truncation, reconnects, large
map status packets, command validation, debug-intent timeout, world projection,
clearance, unknown-space blocking, stale data, session reset and persistence.
Run `python3 pi/dashboard/benchmark_mapping.py` for a synthetic Pi timing check;
it is not a hardware sensor or end-to-end latency benchmark.

## Surroundings mapping

`mapping.py` receives scenes over localhost 5561 and runs in either receiver mode.
The iPhone performs rear RGB capture, ARKit tracking/LiDAR projection and YOLO
inference; the Pi fuses observations and plans previews. The surroundings view
is rendered on the phone from Pi-returned status, not as a mirrored Pi screen.

Each scene is a 4-byte big-endian JSON byte count followed by UTF-8 JSON, with a
512 KiB maximum and ACK byte 0x01. Version 1 contains session ID, monotonic AR
frame timestamp, tracking state, pose `[x,y,z,yaw]`, floor_y, up to 1500 world
points, and up to 20 located object detections. All world coordinates are meters;
y is up. Yaw points along `(sin(yaw), cos(yaw))` in the x/z plane. Timestamps are
phone uptime, not Pi time. Vision results use their source frame's depth and pose;
overtaken inference is dropped. Confidence and range filtering omit unreliable
depth, and do not turn missing returns into free space.

The Pi stores up to 30,000 10 cm voxels and a 200×200 grid covering 20×20 m around
the AR origin. Grid row increases with z: 0 unknown, 1 observed ground, 2 obstacle.
Only ground endpoints within 6 cm of the calibrated floor mark free cells.
Points 8 cm–1.2 m above the floor mark obstacles. Obstacle occupancy is persistent
until clear/reset; moving objects can leave old occupied cells. This conservative
prototype does not detect stairs/drop-offs or guarantee traversability.

Path previews use A*, a provisional 35 cm robot radius and a 60–85 cm approach
ring around a recently observed target. The full conservative square footprint must fit within known free space.
Unknown space, invalid tracking, stale
scenes, missing floor calibration and stale targets prevent valid previews.
No output connects to motors. The phone pose still needs calibration to the
robot center and forward axis before any autonomous motion can be added.

Map commands arrive on 5559: map_save, map_load, map_clear, select_target with
string id. Status `surroundings` contains grid, resolution, origin, size, pose,
objects, path, tracking, freshness and messages. Snapshot publication is about
2 Hz. Heading coverage counts observed yaw bins; it is not map completeness.

Save writes `state/map.json` atomically on the Pi, while the phone separately
stores an ARWorldMap. Loading waits for the matching saved session and normal
relocalized tracking. Objects restored from disk are stale until reobserved.
Both saves must succeed for useful recall. New session IDs clear old geometry.
Unsaved geometry survives Pi stream Stop/Start but not process restart or a
GUI/headless switch. Runtime maps are ignored by git and remain on the devices.

### QNX timing and first hardware check

On this Pi 5/QNX image, a synthetic 1500-point benchmark measured median
12.12 ms ingestion, 47.02 ms forced clearance/path planning and 13.65 ms snapshot
serialization (50 iterations; 40,549-byte snapshot). These are component timings
for one test map, not a worst-case bound or transport latency.

The installed ARKit build delivered 60 scene packets, 141 camera frames and
1402 motion samples during an initial receiver run (8 motion sequence gaps).
ARKit reported limited tracking and no calibrated floor, so geometry stayed
empty. This proves scene transport, not successful physical mapping or target
recognition. Scan/relocalization and actual detection performance still need
field verification.

After the rear camera was exposed to the room, live diagnostics reported normal
tracking, a calibrated floor, 247 accumulated voxels, 64 occupied cells and two
located targets by scene frame 44. The user confirmed “YOLO ready.” No known-free
floor cells were observed at that point, so a real path preview and Save/Load
relocalization remain unverified on hardware. The Pi display showed about
10.5 fps video and 100.9 Hz motion in a separate snapshot; these instantaneous
rates are not a sustained mapping benchmark.

### Phone as the main display

The service has been switched to headless mode. The current phone UI exposes Pi
mode, all motion values/bars, rates, counts, gaps, receipt age and mapping state.
Its paired camera/LiDAR preview is local to the phone, while telemetry and the
accumulated surroundings map come back from the Pi. A monitor or persistent SSH
connection is unnecessary. Keep the phone app foregrounded. The annotated JPEG
still reaches the headless receiver for framing validation and receipt metrics;
no Pi image decode or display is needed.
