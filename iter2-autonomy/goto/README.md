# iter2-autonomy/goto/: "TARS, go to the trash can"

Object name in, wheels out. Everything that runs on the robot is **QNX native C/C++** built with `qcc`/`q++`. Python is used only on the laptop: model export and the Mac simulator tests.

```
voice app ──"GOTO trash can"──► goto (C, SCHED_RR 20) ──D / S / TURN / BEEP / LED──► tars_motor (C, SCHED_RR 30) ──► Uno
                                   ▲  TCP 5570 control                         TCP 5555
webcam ──► detect (C++, ONNX Runtime) ──JSON lines, UDP 127.0.0.1:5571──┤
iPhone LiDAR/ARKit ──LDR1 packets (pose + world points), TCP/UDP 127.0.0.1:5572──┘  (nav.c grid + A*)
```

**One detector.** `iter2-autonomy/goto/detect` (YOLO-World, ONNX Runtime) is the only object detector that feeds `goto`: it's the one that knows "trash can". Andrew's YOLOX/MiDaS (`iter2-autonomy/edge-ai`) serve the dashboard, depth fusion and the guard's veto only. Anything else that wants to feed `goto` must speak `DETECTIONS.md` on 5571, but run one detector at a time on the Pi.

**Ports** (Andrew's stack owns 5555 and 5557–5569): `goto` control **5570**, detections **5571**, nav feed **5572**; voice → phone speech **5573** (Andrew's SpeechLink). `goto` never binds 5562 (lidar_guard's LiDAR) or 5568 (guard → motor gate).

| File | What |
|---|---|
| `src/goto.c`, `src/goto_core.[ch]` | the controller: control port, detection input, nav feed, state machine, `tars_motor` client |
| `src/nav.[ch]` | occupancy grid + A* (C port of `iter2-autonomy/dashboard/mapping.py`), LDR1 parser, pose rule |
| `src/detect.cpp`, `src/camera.hpp`, `src/yolo_post.hpp` | the detector: QNX camera, YOLO via ONNX Runtime, JSON out |
| `objects.txt` | object names + aliases (shared with `iter2-autonomy/voice/`) |
| `goto.conf` | every threshold, with comments |
| `export_world.py`, `get_model.sh` | laptop: YOLO-World with our classes, exported to ONNX |
| `DETECTIONS.md` | the detector → goto interface |
| `tests/` | C unit tests, YOLO post-processing tests, Python 2D simulator + socket tests, `test_fullstack.py` (real tars_motor + goto + voice, fake phone/YOLO/guard) |

## How it behaves

1. **GOTO \<object\>** (`objects.txt` aliases: "bin", "garbage", "me" → person, …). It beeps once and looks at fresh frames **before** turning, since the object may already be in view.
2. **Search:** if it doesn't see the object, it does an exact `TURN 30`, waits for the turn to finish plus `settle_s`, then looks at 4 fresh frames (it needs 2 hits). After 360° with nothing, it stops with `BEEP 3` and the result `failed:not_found`. It turns toward the side where the object was last seen.
3. **Align:** if the object's bearing is more than 12°, it does one exact `TURN` of 0.9 × the bearing. The steppers are exact, so this beats steering with 150 ms camera lag.
4. **Approach:** it sends `D l r` at 10 Hz (the watchdog is 500 ms): a P controller on the bearing (EMA-smoothed box centre, 2° deadband), with forward speed reduced when it's off-centre. If the bearing passes 20° it stops and re-aligns.
5. **Arrived:** the box height is ≥ 70% of the frame (person: 80%), or the box bottom touches the frame bottom. That has to hold for 2 frames. Then it sends `S`, `BEEP 1` and `LED ok`.
6. **Lost:** no target for 1.2 s → `S`, then search again, starting toward where it was last seen.
7. **Stop always wins:** `CANCEL` on 5570, or **an `S` from any other client** (voice "stop", teleop, ✋), cancels the GOTO. `goto` watches `tars_motor`'s `stops=` counter. If `goto` dies, `tars_motor` sees the disconnect and stops the wheels. Tipped, bumped, obstacle or a refused `D` all → `S` + `BEEP 3`.

**Arrival thresholds depend on how the webcam is mounted.** The defaults assume it's about 12 cm up and level, and they stop about 0.5–0.7 m from a 0.4–0.8 m tall object. On the robot, drive up to the target by hand, read `h=` from `STATUS` and set `arrive_h_frac` in `goto.conf`.

## Control port (TCP 5570)

```
GOTO <words>    -> OK goto trash can | ERR unknown object '...' (robot beeps 3)
CANCEL|STOP|S   -> OK                 (S goes to tars_motor first, even when idle)
STATUS          -> OK state=approach target=trash_can result=running seen=1 bearing=-3.2 h=0.31 searched=0 fps=7.9 ...
OBJECTS         -> OK trash can,chair,person,...
RESOLVE <words> -> OK trash can | ERR unknown object
RECALL <words>  -> OK recall trash can | ERR not remembered: ...   ("go back to where the X was": any remembered
                   instance, even stale/gone/old, routed over the map; YOLO still confirms; no blind start)
NAV             -> OK pose=arkit|dr|stale|none,x,z,yaw_deg free=.. occ=.. version=.. packets=.. bad=.. feed_age=..
NAV ROUTE       -> OK route=x,z;x,z;... i=<next waypoint>  (or route=- i=0; nav/ARKit frame, metres)
NAV GRID        -> OK grid v=<version> size=200 res=0.10 origin=-10.00,-10.00 box=ix0,iz0,w,h rle=<cell>x<n>,...
                   (0 unknown, 1 free, 2 occupied; box rows z-major; box=- rle=- when empty)
DETS            -> OK age=<s> <last accepted detection JSON line>   (age=-1.00 {} before any)
                   NAV ROUTE / NAV GRID / DETS are read-only views for the laptop mission control (iter2-autonomy/mission/)
SCAN            -> OK scan; 12 x TURN 30, then "EVENT scanned trash can,chair" (or "scanned nothing")
MEMORY          -> OK n=2 pose=x,y,hdg; trash can#1 fresh x=.. y=.. rel_bearing=.. rel_dist=.. est_dist=.. age=.. seen=.. conf=.. map=x,z; ...
MEMORY SAY      -> OK n=2 I remember 2 things: a trash can about 1.2 m to my left, and a chair about 2.0 m behind me.
FORGET [obj[#n]]-> OK forgot N        (no argument = everything; "trash can" = all its instances; "trash can#2" = one)
WATCH           -> OK, then async "EVENT searching|remembered <obj> #n|not_there <obj> #n|arrived|lost|cancelled|failed:<why>|scanning|scanned ..." lines
                   map-guided: route <obj> #n <corners> | reveal | explore | unmapped | no_map | no_route | blocked:<why> |
                   occluded | detour | next_candidate | guard_face | guard_stop <obj> <why>
PING            -> OK PONG
```

**For the voice app:** on intent `goto` + object, send `GOTO <object words>` to 5570 and speak or beep based on the `EVENT` lines. On "stop", send `S` to 5555 (the fastest path). `goto` notices it by itself.

## Object memory

`goto` remembers where it has seen things, so the next `GOTO` turns straight at the object instead of searching 30° at a time.

- **Pose.** There are no encoders, but the 28BYJ-48 wheels move what they're told to. `tars_motor` integrates the motion that actually ran: the slewed `D` output every tick, plus each `MOVE`/`TURN` when the Uno reports `DONE`/`ABORT` with the amount done. That includes WASD teleop and voice moves. It's in `STATUS` as `pose=x,y,hdg pose_epoch=` and in `POSE`. Axes: x forward from where `tars_motor` started, y right, hdg + = clockwise. `goto` polls it at 4 Hz, even when idle. **Drift is unknown**: skipped steps, slip and being picked up are invisible to it.
- **What's stored.** Every detection frame updates the memory, for every label, not just the target (`memory_min_conf` 0.40). Each entry is an **instance**: `label#id`, world x/y, the world bearing of the last sighting, an estimated distance, conf, times seen, last seen (wall clock), and a status of **fresh / stale / gone**. A sighting merges into an instance of the same label if it's within `memory_merge_mm` (500), or on the same ray within the range noise. Otherwise it becomes a new instance (`trash can#2`). Two boxes of one label in one frame are two instances. Nothing is remembered while an exact move runs or the robot is spinning (the pose lags then).
- **Distance** comes from where the box bottom meets the floor, for a level camera at `cam_height_mm` (120). If the bottom edge is cut off by the frame, it falls back to box height vs `arrive_dist_mm`. In the sim it reads 2.1 m for a can 2.0 m away. **It hasn't been checked with the real C270 mount.**
- **GOTO with memory.** It picks the best usable instance: fresh before stale, then nearest. It skips gone instances and anything older than `memory_max_age_s` (30 min). It does one exact `TURN` to the instance's bearing from the current pose, then LOOKs. If it sees the target, it aligns and approaches as usual. If not, it optionally does `MOVE` up to `memory_drive_mm` toward the spot (default **0 = off**), then marks the instance (fresh → stale → gone), tries the next instance (up to `memory_tries`), and falls back to the normal 360° search starting toward the remembered side. Arriving rewrites the instance's position from 0.55 m in front of the robot.
- **Saved** atomically (`.tmp` + fsync + rename) to `memory_path` (default `memory.txt` next to `goto.conf`, or `-M file`, or `-` for RAM only), at most every `memory_save_s` (2 s) when something changed, and on exit. It's loaded at startup. The file is text (`v 2`, one `pose` line, `inst` lines), and bad lines are skipped.
- **Restarts.** The file keeps `tars_motor`'s `pose_epoch` and the world↔motor offset. If only `goto` restarted (same epoch), nothing changes. If `tars_motor` restarted (new epoch, pose back to 0), the memory re-bases: it assumes the robot is still where it last knew. **If you carried the robot somewhere else, run `FORGET`** ("TARS, forget everything"). `POSE RESET` on 5555 also triggers a re-base.
- **Voice:** "TARS, look around" / "what do you see" → `SCAN`. "TARS, forget everything" → `FORGET`. "TARS, forget the bin" → `FORGET trash can`.
- **Adding states** (e.g. a lidar OCCLUDED check, or DETOUR around a wall): what happens after a failed LOOK is decided in one place, `look_failed()` in `goto_core.c`, in order (drive closer → next instance → search). New exact-move states go in `goto_sm_wants_status()` and in the turn branch of `goto_sm_tick()`. A lidar can set `MEM_GONE` or feed `dist` through `goto_mem_sight()`.

## Map-guided GOTO (iteration 2: voice + YOLO + iPhone LiDAR)

The phone's LDR1 feed on 5572 builds `nav.c`'s grid and gives an ARKit pose. When a GOTO target is in memory **and** the pose is usable, `goto` drives over the map to the object's approach ring and lets YOLO confirm and dock. The design and transition table are in [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md) §3. As built:

- **RECALL**: pick the instance (fresh before stale, nearest; `RECALL` also takes stale/gone/old ones). Its map position is `map=x,z`, stored when it was seen with a good nav pose. Older instances are projected from the memory frame through the current pose.
- **PLAN**: `nav_plan` from the fused pose to the 0.6–0.85 m ring. Before planning, the robot's own footprint is marked FREE (it's standing there; the phone can't see under itself). Outcomes:
  - path → FOLLOW_PATH
  - already on the ring → face the spot, LOOK
  - NO_PATH / NO_GOAL → **reveal**: spin in `nav_reveal_deg` (30°) steps, re-planning after each, up to 360°. The phone sees a 60° cone, and unknown blocks like a wall.
  - still NO_GOAL and the ring is mostly unmapped (LiDAR shadow) → up to 4 **explore** hops to a reachable mapped cell ≥ 0.4 m nearer the goal, each followed by another reveal, then re-plan. Cells whose line to the goal is known to be blocked (the middle of the wall in between) are tried last: from there the goal's ring can't be mapped.
  - still nothing → **NEXT_CANDIDATE** (`no_route`: next instance, else blind search).
  - no map at all / pose stale / unmapped after exploring → the **visual-only** memory turn.
- **FOLLOW_PATH**: each waypoint is an exact `TURN` (if off by more than `nav_turn_min_deg`) then a forward `MOVE` (never reverse: the guard only watches ahead). The last leg is a `TURN` to face the spot, then **LOOK** (YOLO, 2 of 4 frames).
  - After each leg (and each reveal turn), the next one waits for **2 ARKit poses taken after the move ended** (≤ `nav_pose_fresh_s`; DR needs no wait). The newest LDR1 packet at `done` was measured before the move ended, and planning from it turned by a stale heading.
  - `nav_version` change and an occupied cell now on the remaining route → `S`, re-plan.
  - A leg aborted by the guard / obstacle / deadman, or refused by the guard → **PLAN wait** (guard CLEAR + `nav_guard_wait_s`, or 3× that if it stays BLOCKED, since the new route starts with a pivot), max `nav_max_replans` (6), then `failed:blocked`.
  - A refused **face** turn → LOOK straight away: what's blocking is normally the target.
  - **Something inside the robot's own footprint** (≲ 0.45 m away, e.g. a person or a board stepping in right in front; also inside lidar_guard's stop zone): every plan from here would be `START_BLOCKED`. Instead the route starts with a **back-off cell** 0.2–0.4 m away (its footprint clear, the straight line to it observed free, in a direction pointing away from every obstacle cell in the footprint): an in-place pivot (allowed under a fresh BLOCKED, assumption 7) and a short **forward** MOVE that only increases the distance to the obstacle. Never a reverse move. Used by PLAN and DETOUR.
- **LOOK failed at the spot**:
  - `nav_line_clear == 0` (an occupied cell between us and the spot) → **OCCLUDED**, *not* downgraded → **DETOUR** once per instance to a vantage cell 0.7–1.3 m from the spot with a clear line of sight **and a clear drive lane** (no OCC on lines ±0.1–0.4 m beside it, up to 0.3 m from the spot: the visual approach drives straight at the target from there, and the guard would stop it on the occluder's edge), where the robot fits (footprint clear; up to 8 candidates are planned, nearest first, with `nav_plan_ring`, back-off first if needed), LOOK again.
  - Line clear (or no vantage) → **CHECK_OTHERS**: fresh → stale → gone, next same-label instance.
  - None left → **NEXT_CANDIDATE** → 360° search.
- **Guard hiccup during a search / align / memory TURN** (aborted with `lidar_guard`, or refused with `guard_*`): tars_motor allows a pivot under a fresh BLOCKED, so this is a guard that went stale for a moment. → `S`, `EVENT guard_stop`, LOOK again after `nav_guard_wait_s` and carry on (the interrupted search step doesn't count toward the 360°). Shares the `nav_max_replans` budget → `failed:blocked`. Before, the whole GOTO ended `failed:turn_lidar_guard`.
- **APPROACH** is the visual servo as before, but drives are **leased**: `DL <guard_epoch> l r` (epoch from `STATUS`, fixed per leg). A guard denial bumps the epoch, so the leg ends (`S`) and a new one starts only after a fresh LOOK. An old drive never resumes after the guard clears. If the guard stops us while the target is centred, tracked and at least half the arrive height, that's **arrived**: lidar_guard's ~0.6 m stop zone and our ~0.55 m docking distance overlap.
- **Pose rule** (nav.h): ARKit while LDR1 packets say tracking + heading OK and the last one is < `nav_pose_fresh_s` (0.5 s) old. Otherwise dead reckoning: `tars_motor` STATUS `pose=` → `nav_note_dr(-y, x, -hdg)` in metres/radians, carried into the ARKit frame from the last tracked moment. Stale/none → no map driving.
- **Stop wins**: `CANCEL`, or any `S` (the `stops=` counter), from any state including mid-leg and PLAN wait. Tip/bump/watchdog/guard live in `tars_motor` underneath all of it.

### Assumptions (made without asking; change them here if wrong)

1. **Ports**: ours move to 5570/5571/5572 to clear Andrew's 5557/5558 (phone motion, preview). *Needs Andrew's sign-off.* The LDR1 copy for `goto` comes on 5572 (TCP length-prefixed, or one UDP datagram ≤ ~9 KB on macOS). Who sends it (the phone directly, or Andrew's service teeing what it gets on 5562) isn't built yet; the sim plays the phone.
2. **Frames**: ARKit x/z, y up, forward = (sin yaw, cos yaw), + yaw = left (nav.h). `tars_motor` is x forward, y right, hdg + = clockwise (mm, deg), so DR → nav is `x = -y/1000, z = x/1000, yaw = -hdg`. The phone is assumed at the robot's centre, facing forward (extrinsics unmeasured; the ring margin absorbs a few cm).
3. **Pose freshness 0.5 s** (`nav_pose_fresh_s`), not the 150 ms in the design doc: LDR1 is ~20 Hz over USB/iproxy and 150 ms would flap. Tune on the robot.
4. **Robot footprint is free**: before each plan, the 9×9 cells under the robot are marked FREE (never over OCC). Without it, A* could never start, because the phone doesn't see the floor under or beside itself.
5. **Reveal + explore** (above) is our frontier-lite. Unknown stays blocked for A*, exactly as `mapping.py` (radius 0.35 m kept, per the lead).
6. **Guard stop at the target = docked** (see APPROACH). The approach ring (0.6–0.85 m) sits at the edge of lidar_guard's stop zone, so a face TURN refused by the guard goes to LOOK.
7. **tars_motor allows an in-place `TURN` while the guard reports a fresh `guard_blocked`** (not stale/missing/invalid). `D`/`DL`/`MOVE` stay refused. The guard watches a forward corridor only, so a pivot isn't what it guards. Without this, a robot facing a wall inside the zone could never turn away (found in the full-stack sim). The pivot is limited: speed ≤ 0.40 (clamped), |deg| ≤ 180 (`ERR guard_blocked pivot>180`, handled like any guard refusal), and a running TURN that's faster or longer aborts when the guard turns BLOCKED. So `goto` sends every TURN at `search_speed = 0.40` (was 1.0; on the DC car that's the Uno's slow turn at `deg_s_slow`, ~60°/s default guess, instead of D 1) and its turns are already shortest-way (≤ 180). The sweep beside/behind the robot is still unguarded and relies on the 0.35 m inflation; measure the SM11 + phone-mount swing radius on the floor. **This changes the safety floor: Andrew still needs to sign off.**
8. **Exact moves need a keepalive** on Andrew's main (the 500 ms deadman aborts MOVE/TURN): `goto` PINGs at 10 Hz while it waits on a move, and `tars_voice` has a keepalive thread for voice moves.
9. **MEMORY SAY** is one line, nearest first, at most 4 objects named ("and N more"), gone ones left out, directions relative to the current heading (ahead / ahead on my left / to my right / behind me …). Voice beeps `min(n, 4)` times (`LED no` for none) and sends `SAY <sentence>` (≤ 300 chars) to the phone on 5573. On a stop it sends the urgent JSON "Stopping." **after** `S` and `CANCEL`.
10. Memory file gains `navw <id> <x> <z> <label>` lines (map position). Old files load unchanged; old loaders skip the new lines as bad lines.
11. The full-stack sim speeds the stub up 4× (`WHEELCAL 260 400`): same turn rate, 4× linear speed. Not physical.
12. **DC smart car (Kuman SM11):** `TURN`/`MOVE` are timed, not exact, and `tars_motor`'s POSE drifts. Nothing in goto changed: it re-plans from the iPhone pose after every leg. `tests/test_dc_car.py` (in `make test`) and `make test-fullstack-dc` check it with ±25% MOVE/TURN error + 15°/m drift at 150 mm/s. Known gaps: (a) with TURN really doing < ~85%, the 12 × 30° search leaves a blind wedge (calibrate `dc-spin`; or raise `search_max_deg` to ~420); (b) object memory merges sightings by the dead-reckoned x/y, so on a DC car a moved object can refresh its old entry instead of downgrading it. Merge by `navw` when both sides have one (goto_core.c).

## Run order on the Pi

```sh
./tars_motor -b uno -d /dev/serusb1 &                                     # 1. motors (iter1-wasd/pi/)
./goto -c goto.conf -o objects.txt &                                      # 2. controller
./detect --camera camapi:1 --model models/yolo_world.onnx --labels models/labels.txt &   # 3. eyes
printf 'GOTO trash can\n' | nc 127.0.0.1 5570                             # or the voice app
```

## Build

```sh
make -C iter2-autonomy/goto host && make -C iter2-autonomy/goto test     # Mac: goto_host, unit tests, simulator (no QNX needed)
make -C iter2-autonomy/goto                               # QNX: build/goto (qcc via docker/run.sh, like iter1-wasd/pi/)
make -C iter2-autonomy/goto detect ORT_ROOT=<staging>     # QNX: build/detect (needs ONNX Runtime for QNX, below)
make -C iter2-autonomy/goto deploy PI_HOST=qnxuser@<pi-ip>
```

`goto` uses only libc, BSD sockets (`-lsocket`) and `-lm`. `detect` also needs `-lonnxruntime` and `-lcamapi`.

### Model (laptop)

```sh
iter2-autonomy/goto/get_model.sh            # venv + ultralytics, then export_world.py: YOLO-World v2 (s) with the
                             # objects.txt classes baked in -> iter2-autonomy/goto/models/yolo_world.onnx + labels.txt
iter2-autonomy/goto/get_model.sh --coco yolo11n.pt   # faster plain COCO model (no "trash can")
```

Models are never committed. Re-export after changing the **canonical** names in `objects.txt`. Adding an alias only needs a restart.

### ONNX Runtime for QNX

[`qnx-ports/build-files`](https://github.com/qnx-ports/build-files) has a QNX 8.0 port at `ports/onnxruntime` (source: `qnx-ports/onnxruntime`, branch `qnx-v1.23.2`). The port says it **builds on a Linux host only**, so use its Docker container, not macOS directly:

```sh
mkdir -p ~/qnx_workspace && cd ~/qnx_workspace
git clone https://github.com/qnx-ports/build-files.git
cd build-files/docker && ./docker-build-qnx-image.sh && ./docker-create-container.sh
# inside the container:
source ~/qnx800/qnxsdp-env.sh
cd ~/qnx_workspace && ./build-files/ports/onnxruntime/install_all.sh
git clone -b qnx-v1.23.2 https://github.com/qnx-ports/onnxruntime && (cd onnxruntime && git submodule update --init)
make -C build-files/ports/onnxruntime install -j4      # 32 GB RAM recommended for big -j
# -> $QNX_TARGET/aarch64le/usr/local/{include,lib}; then:
make -C <repo>/goto detect ORT_ROOT=$QNX_TARGET/aarch64le/usr/local
scp $QNX_TARGET/aarch64le/usr/local/lib/libonnxruntime.so* qnxuser@<pi-ip>:/data/home/qnxuser/goto/
# on the Pi: LD_LIBRARY_PATH=/data/home/qnxuser/goto ./detect ...
```

The same repo also has ports for **OpenCV** (`ports/opencv`; it needs `numpy` and `muslflt` first, plus the SDP package `com.qnx.qnx800.target.screen.img_codecs`) and **TensorFlow / TFLite**. I didn't find an **ncnn** port. ONNX Runtime was chosen because it's one library: `detect` does its own letterbox, YUV→RGB and NMS in `yolo_post.hpp`, so OpenCV isn't needed. The port's upstream tests were run on a Pi 4 QSTI image. **It hasn't been built or run by us yet.**

### Camera on QNX (no V4L2)

QNX has no V4L2. Cameras go through the **QNX Camera library** (`camera/camera_api.h`, `-lcamapi`) on top of the **Sensor Framework** service (`sensor`), which owns the device and is configured with a sensor config file listing camera units. The QNX docs say the camera library supports USB cameras, and the sensor config picks the connection type (USB, GigE, sensor) per unit. The QNX Everywhere Pi image ships camera samples. `detect --camera camapi:1` opens `CAMERA_UNIT_1` and uses the viewfinder callback. It handles YUYV (`CAMERA_FRAMETYPE_YCBYCR`, what UVC webcams usually send), UYVY, NV12, RGB8888 and BGR8888.

On the Pi, check in this order:
1. `pidin | grep -i sensor`: is the sensor service running? Find its config (`ps -ef` shows its `-c` argument) and check that a unit is the USB webcam.
2. Run the camera sample on the image, if there is one. If it shows video, `camapi:1` should work.
3. `./detect --camera camapi:1 --nomodel --stdout --no-udp --frames 20` prints 20 empty-detection lines with `w`/`h`. This tests the camera without the model.

The camapi code is **written from the API docs and hasn't been compiled against the SDP headers yet.** Fixes, if any, will be field names in `CamapiSource::on_frame`. Fallback if camapi won't cooperate: any program that can produce raw frames can feed `detect --camera raw:640x480:yuyv:-` through a pipe, or the teammate's detector can talk to `goto` directly over `DETECTIONS.md`.

### CPU notes (Pi 5, estimates, not measured)

- `goto` is nearly idle: it parses about 10 JSON lines per second and sends about 10 commands per second. It runs at SCHED_RR 20, below `tars_motor` (30) and above normal-priority ML.
- `detect` is all inference: YOLOv8s-worldv2 at 320 px is roughly 13 GFLOPs at 640, so about 3.3 at 320. Expect about 5–10 fps on 2 threads of a Pi 5 with ONNX Runtime. `--fps 8` caps it. `--threads 2` leaves cores for speech. `--coco yolo11n.pt` is about 3× cheaper if the World model is too slow (but has no "trash can").
- `detect` runs at **normal priority on purpose**, so it can never starve the motor loop.

## nav: room map + route planner (C, `src/nav.[ch]`)

The map side of map-guided GOTO. It's a C99 port of the grid and A* in `iter2-autonomy/dashboard/mapping.py`, with no deps beyond libc/libm, no threads and no I/O, so it builds with `qcc` like the rest of `goto`. Andrew's Python stays the **desktop visualization**; this module is what drives the robot (map-guided GOTO above).

- **Grid:** 200×200 cells of 0.1 m covering ARKit world x,z in [-10, 10) m, each UNKNOWN/FREE/OCC. A point 0.06 m or less from the floor marks a cell FREE, but never clears an obstacle. A point 0.08–1.2 m above the floor marks it OCC. `nav_carve_ray` frees unknown cells along a ray. It's optional, and LDR1 ingest doesn't use it, the same as the Python.
- **Feed:** `nav_ingest_ldr1()` parses one LDR1 packet (the phone's measured LiDAR, world-frame points + pose + floor_y, format in `iter2-autonomy/edge-ai/README.md`). The 4-byte length prefix is optional, and validation is the same as `lidar_guard`'s. It maps only when the tracking and floor flags are set. A floor jump over 0.1 m clears the grid. The JSON world-observation stream on 5561 is not parsed in C. Port 5562 is owned by `lidar_guard`; `goto` takes its own copy of the packets on 5572. `nav_plan_ring()` is `nav_plan` with a custom ring; DETOUR and explore use ring 0–0.11 m to reach one exact cell.
- **Plan:** `nav_plan(sx,sz → tx,tz)`. The robot's footprint square (2·ceil(r/0.1)+1 cells, r = 0.35 m by default, `nav_set_robot_radius`) must be entirely *observed* free, so **unknown blocks the same as a wall**. A* is 8-connected and never cuts a diagonal corner. Goals are the clear cells on the 0.6–0.85 m ring around the target. Collinear cells are dropped, and the output is the corner waypoints (start excluded, last = approach point). Errors: `START_BLOCKED`, `TARGET_OUTSIDE`, `NO_GOAL`, `NO_PATH`, `NO_MAP`, `TOO_LONG`. About 1.5 ms for a worst-case serpentine across the whole map on an M-series Mac (Pi 5 not measured).
- **Occluded vs gone:** `nav_line_clear()` returns 1 for a clear line, 0 if an OCC cell is in between (occluded) and -1 if some cells are unknown. It ignores cells within 0.3 m of the end, because the object itself is occupied.
- **Pose rule:** while tracking is OK, the ARKit pose comes from `nav_set_pose(..., 1)` or each LDR1 packet. When tracking is lost, `nav_get_pose()` maps the dead-reckoned motion (`nav_note_dr()`, from `tars_motor` STATUS `pose=`) since the last tracked moment into the ARKit frame. Always re-check the target with YOLO before declaring arrival.

Tests: `make test` runs `test_nav`, which covers routes, errors, inflation, diagonals, line of sight, pose fallback, LDR1 good/truncated/oversized/NaN and a 200k-packet fuzz. `make test-nav-asan` runs the same tests plus a 1M-packet fuzz under ASan/UBSan. `make test-nav-parity` runs the C planner and `mapping.py`'s `Map._plan`, imported directly, on the same fixtures (wall, door, U-trap, unknown strip, every error, plus seeded random clutter). It requires an **identical** result, either the same error or the same corner list cell for cell. Ties break the way Python's `heapq` breaks them.

## Mac tests (what has actually run)

```sh
make -C iter2-autonomy/goto test
```

- `test_core`: aliases, the JSON parser, config, and the state machine on scripted detection streams with a fake clock.
- `test_yolo`: YUV→RGB, letterbox, YOLO decode for both output layouts, NMS, and the `detect` JSON parsed back by `goto`'s C parser (the contract test).
- `test_goto_sim.py`:
  - the C `goto` over real sockets against a fake `tars_motor` with differential-drive kinematics, and a pinhole camera sim (60° HFOV, 12 cm mount, 150 ms latency, 10% misses, jitter, motion blur, a distractor chair)
  - CANCEL latency, and an external `S` from another client
  - the TCP and stdin inputs
  - the real `iter1-wasd/pi/build/tars_motor_host -b stub` end to end
  - random start poses

Results on the Mac (2026-11-21), host builds only:

| Suite | Result |
|---|---|
| `test_core` with map-guided GOTO (route around a wall without crossing OCC or reversing, occluded vs gone + DETOUR, multi-instance CHECK_OTHERS, no route → reveal → FAILED, STOP/CANCEL mid-path, guard wait + re-plan + map change + re-plan limit, DL lease, visual fallbacks, RECALL, MEMORY SAY) on top of object memory | **265/265**, also under ASan/UBSan (incl. phone-lag legs: every leg starts from the post-move pose; explore hop must see the goal; occluder 0.39 m ahead, inside the footprint: pivot away + forward back-off, DETOUR with a clear drive lane, ARRIVED, never closer to it, no reverse; guard hiccup during search/align TURN: LOOK again, full 360, bounded) |
| `test_fullstack.py`: real `tars_motor_host -b stub` (guard gate on, fake lidar_guard), real `goto`, real `tars_voice` + whisper; fake phone (LDR1 raycast, 60° cone, TCP 5572-style), fake YOLO (occlusion-aware), fake phone speech | **33/33**: obstacle (reveal → explore → route around the wall → docked, ≥ 0.45 m from walls), occlusion (board dropped at the spot → OCCLUDED → DETOUR → docked at the bin, < 0.8 m), occlusion_close (the same board 0.39 m in front of the robot, inside its footprint and the guard zone → pivot + forward back-off → DETOUR → docked at the bin), moved object (not_there → search → docked at the new spot, old spot downgraded), STOP mid-route (S from another client: cancelled, wheels still), visual-only (no phone), voice (MEMORY SAY printed + BEEP + SAY on the phone port; "go back to where the bin was" → RECALL → docked). Prints the fake guard's worst GAT1 gap per scenario and keeps tars_motor's log in `build/fs_motor_<port>.log`. With every TURN at 0.40 (pivot limit): 15/15 in a row (80–87 s each) on a Mac with load avg 4–10 from other work, incl. fake-guard stalls up to 192 ms that the TURN retry rides out; 3/3 with 7 busy loops on top. Before that, at speed 1.0, one run in ~15 failed in a machine-wide stall (several scenarios `turn_timed_out` at once; tars_motor accepted a TURN and never started it, unexplained, logs now kept). Earlier fixes: a stale ARKit pose at the end of each leg masking an explore hop into the wall's shadow (obstacle/voice `not_found`), and a board landing inside the footprint (occlusion `not_found`, ~1 in 16). |
| memory vs blind, 16 random scenes (SCAN, then a teleop TURN + MOVE 300, then GOTO; the fake motor's pose drifts like dead reckoning: ±3% turns, ±2% wheels) | memory 16/16 arrived, median **39 s / 1 turn**; blind 16/16, median 44 s / 3 turns. Driving at 4 cm/s dominates, so the gain is modest. |
| `test_core` with object memory only | 185/185, also under ASan/UBSan. |
| `test_core` before memory (C state machine, parser, aliases, config) | 92/92. It also passes under ASan/UBSan. The JSON parser was fuzzed with 300k mutated lines under ASan/UBSan, and one UB was found and fixed (huge `w`/`h`). |
| `test_yolo` (pre/post-processing, JSON contract) | 25/25 |
| `test_nav` (grid, A*, line of sight, pose rule, LDR1 parser + fuzz) | 84/84, also under ASan/UBSan with 1M fuzzed LDR1 packets |
| `nav_parity.py` (C `nav_plan` vs `mapping.py` `Map._plan`) | 1560/1560 identical over 5 seeds (12 hand fixtures + 300 random each; ~74% routed, the rest error cases) |
| `test_goto_sim.py` sockets + sim + real `tars_motor_host -b stub` | all checks pass. CANCEL → `S` at the motor server: median 0.2 ms, max 0.24 ms. An external `S` cancels within one STATUS poll. |
| 2D sim, 40 random start poses (0.8–3 m, any bearing, 0.4/0.6/0.8 m objects) | **40/40 arrived**. Median 46 s, max 77 s. Stopped 0.49–0.58 m away, facing within 2°. It never touched the object. |
| `detect` with ONNX Runtime 1.32 (macOS arm64) + exported YOLO-World 320 px | boxes match ultralytics' own ONNX predict to within 0.1 px and 0.001 confidence on `bus.jpg`. About 11 ms/frame on an M-series Mac with 2 threads. The Pi 5 will be several times slower. |
| detect → goto → `tars_motor_host` pipeline | real YOLO boxes of a person at −20.5° → `goto` → `TURN -18.5` running on `tars_motor`. |

To build `detect` with real inference on a Mac: get ONNX Runtime headers and a dylib, then `make host-detect ORT_HOST=<dir with include/ lib/>`. That produces `build/detect_host_ort`.

**Not done yet:**
- Nothing map-related has run with a real iPhone: no phone sends LDR1 to 5572 yet (Andrew's app sends it to lidar_guard on 5562). Frames, extrinsics and the 0.5 s pose freshness are assumptions (see above).
- `goto` and `detect` haven't been cross-compiled with `qcc`/`q++`: there's no SDP or Docker on this Mac.
- The camapi source has never been compiled.
- ONNX Runtime for QNX hasn't been built.
- Nothing has run on the Pi, with the C270 or on the real robot.
- The arrival threshold is tuned in the simulator only.
