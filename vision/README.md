# vision/: drive TARS with your hand (MediaPipe, on the Pi under QNX)

Two modes, switched with a held peace sign ✌️:

- **GESTURE**: ✋ stop · 👍/☝️ step forward · ✊/👇/👎 step back · 👈/👉 turn 30°
- **FOLLOW**: the rover steers toward your hand and drives until it's at
  arm's length. ✋ stops it, and losing the hand stops it.

The rules and every number are in **[GESTURES.md](GESTURES.md)** and
**[gesture.conf](gesture.conf)**, shared by all three apps:

```
camera ─▶ MediaPipe HandLandmarker ─▶ classify ─▶ Controller ─▶ TCP :5555 ─▶ tars_motor (pi/) ─▶ Uno ─▶ wheels
          (21 points, 1 hand)          (label)    (mode, debounce,        D l r / S / VIS
                                                   moves, follow, fail-safes)
```

| App | Runs on | Camera | Status |
|---|---|---|---|
| `desktop/gesture_drive.py` | Mac | OpenCV webcam | **tested** (unit, parity, end-to-end with real MediaPipe + stub server) |
| `qnx/python/tars_gesture.py` | Pi, QNX | libcamapi (QNX codelab helper) | written, **not run on QNX yet** |
| `qnx/cpp/tars_gesture.cc` | Pi, QNX | libcamapi (qnx-ports `qnx_defs`) | written + type-checked against stub headers, **not built yet** |

**Which Pi app?** Start with the **Python** one. QNX ships MediaPipe prebuilt
(`apk add mediapipe`), so it needs no compiling. The C++ app is the
"no Python in the loop" version for the judges. It has to be built on the Pi
with Bazel (hours), so start that build early and in parallel.

MediaPipe and the hand landmarker model are Apache 2.0, free to use.

---

## 1. Mac: try it now (tune your gestures here)

```sh
cd vision
uv venv --python 3.11 .venv && uv pip install --python .venv/bin/python -r desktop/requirements.txt
#   (or: python3.11 -m venv .venv && .venv/bin/pip install -r desktop/requirements.txt)
./get_model.sh

.venv/bin/python desktop/gesture_drive.py               # DRY RUN: prints commands, overlay window
```

macOS will ask for camera permission for your terminal the first time
(System Settings → Privacy & Security → Camera).

The window shows the mode (GESTURE blue / FOLLOW yellow), the label it sees,
the hand skeleton and the two wheel bars. In FOLLOW it also shows the centre
deadband and the current vs target hand size. The bottom line shows fps,
inference ms, camera→command ms and the last palm→STOP ms.
Keys: `q` quit · `space` stop · `m` switch mode · `p` pause.

Tune without editing files: `--set stable_frames=3 --set follow_target_size=0.25`.
Copy the values you like into `gesture.conf`.

**Against the motor server, no robot needed:**
```sh
make -C pi host && pi/build/tars_motor_host -b stub          # terminal 1: prints wheel commands
vision/.venv/bin/python vision/desktop/gesture_drive.py --host 127.0.0.1   # terminal 2
printf 'STATUS\n' | nc 127.0.0.1 5555                          # shows vis=… stop_ms=…
```

**Tests:**
```sh
python3 vision/tests/test_gesture.py            # 38 unit tests: classifier, steps, follow, mode toggle
python3 vision/tests/test_parity.py             # Python vs C++ (gesture.hpp): identical output
vision/tests/check_cpp_syntax.sh                # tars_gesture.cc type-checks (stub MediaPipe headers)
vision/.venv/bin/python vision/tests/e2e_desktop.py   # real MediaPipe on real hand photos -> stub server
make -C pi test                                  # server incl. hard-stop S + VIS (116 checks)
```
or all of them: `vision/tests/run_all.sh`.

---

## 2. Pi: Python app (fast path)

Needs the **QNX 8.0.5 QSTI** Pi 5 image (it has `apk`, QNX's MediaPipe package and Screen).
The C270 already works with `sensor -U usb_camera.conf` + `camera_example3_viewfinder`.

```sh
# Mac
vision/qnx/deploy.sh                            # PI_HOST=qnxuser@<ip> if tars.local doesn't resolve
make -C pi deploy                               # tars_motor with the new S/VIS hook

# Pi (ssh qnxuser@tars.local)
cd /data/home/qnxuser/vision
sh qnx/setup_pi.sh                              # once: apk add mediapipe, camera helper, model
python3 qnx/python/tars_gesture.py --list-units # which camera unit the C270 is
python3 qnx/python/tars_gesture.py --dry-run --save-every 30   # look at gesture_view.jpg (scp it back)

# drive: tars_motor in one ssh, the vision app in another
/data/home/qnxuser/tars_motor -b uno -d /dev/serusb1        # (or: make -C pi run from the Mac)
python3 qnx/python/tars_gesture.py --unit <N>
```

The log looks like this (from the Mac end-to-end run; Pi numbers will differ):
```
[cmd] D 0.50 0.50    forward        mode=gesture
[cmd] S              mode_follow    mode=follow
[status] mode=follow fps=26.9 infer=7ms cam->cmd=17ms palm->stop=- see=fist wheels=+0.07,+0.49 link_rtt=0.3ms
[cmd] S              palm           mode=follow
[lat] PALM -> STOP: camera frame -> S acked in 6 ms
```
and the server's `STATUS` shows `stop_ms=… vis=follow vis_fps=… vis_cam_ms=… vis_stop_ms=… vis_see=fist`.

**Expected fps on the Pi 5 CPU** (estimate, not measured): MediaPipe's hand
landmarker runs about 10–20 fps at 320 px wide. QNX's own face-detection
codelab reports 25 fps on the Pi 5 with a lighter model. If it's under 8 fps:
`--set infer_width=256`, and `--set stable_frames=3` so moves still feel snappy.

## 3. Pi: C++ app (`qnx/cpp`)

Built **on the Pi** inside QNX's MediaPipe port, exactly as in QNX's
[MediaPipe Camera Sample codelab](https://qnx.github.io/codelabs/mediapipe-camera-sample/).
It's not cross-compiled in our Docker toolchain, because the port carries its own QNX Bazel toolchains.

```sh
# Pi
cd /data/home/qnxuser/vision
bash qnx/cpp/build_on_pi.sh        # apk add bazel6 …; clones qnx-ports/mediapipe (qnx-v0.10.26) to ~/mediapipe;
                                   # copies tars_gesture.cc + BUILD + gesture.hpp into mediapipe/examples/qnx/;
                                   # runs QNX's build_qnx_examples.sh answering "y" only for ours
qnx/cpp/tars_gesture_cpu --model=models/hand_landmarker.task --conf=gesture.conf --dry_run
qnx/cpp/tars_gesture_cpu --model=models/hand_landmarker.task --conf=gesture.conf --camera_unit=<N>
```
`BAZEL_JOBS=2` if the build runs out of memory. Same log format, same `VIS` report.
The camera code is QNX's `qnx_defs.cc`, which accepts NV12, YCbYCr (YUYV), CbYCrY,
RGB888, RGB8888 and BGR8888, so the C270's packed YUV is covered.

---

## 4. Fallback: vision on the laptop, motors on the Pi

If neither Pi app works in time, run the Mac prototype against the robot over Wi-Fi.
It uses the same logic, commands and fail-safes, with only the camera moved:
```sh
vision/.venv/bin/python vision/desktop/gesture_drive.py --host tars.local
```
Point the Mac webcam (or an iPhone via Continuity Camera, `--camera 1`) at
yourself. In FOLLOW mode the camera should ride on the robot, so use the laptop
for GESTURE mode. Wi-Fi adds roughly 5–30 ms of `link_rtt`, which is shown in the log.

---

## 5. Troubleshooting

| Symptom | Fix |
|---|---|
| `setup_pi.sh`: no `apk` | Not the 8.0.5 QSTI image. Reflash, or use the fallback (§4) |
| `--list-units` is empty | the sensor service isn't running your USB config: `sudo pidin ar \| grep sensor`, then `sensor -U usb_camera.conf …` as in your viewfinder test |
| `frame timeout` lines | another app has the camera open (close `camera_example3_viewfinder`), or the wrong `--unit` |
| colours swapped / purple view | `--packed-order uyvy` (Python app; the first frame logs `frametype=…`) |
| `unexpected descriptor` error | the Python helper can't decode that frame type. Use the C++ app (QNX's decoder) or set the sensor config to NV12 |
| labels flicker | more light, plain background; `--set stable_frames=5`; check `gesture_view.jpg` |
| point left/right swapped | `point_left_is_left = 0` |
| follow steers away from the hand | `follow_invert_turn = 1` |
| follow overshoots / wobbles | lower `follow_kp_turn`, raise `follow_deadband` |
| creeps when the hand is still | raise `follow_size_deadband` |
| `[link] … ERR tipped over` | stand it up, send `RESET` (`python laptop/tars_client.py`) |
| turns aren't 30° | `desktop/calibrate_turn.py --host tars.local` (12 turns = 360°) |

## 6. Layout

```
gesture.conf          all tuning (both languages read it; unknown keys are an error)
GESTURES.md           the rules
get_model.sh          hand_landmarker.task (float16, 7.8 MB, sha256-checked)
common/gesture.py     classifier + controller (pure Python)
common/gesture.hpp    the same in C++17 (header-only)
common/app.py         MediaPipe runtime + ticker + status/VIS (Python apps)
common/link.py        TCP client with reconnect / dry run
desktop/              gesture_drive.py (Mac), calibrate_turn.py, requirements.txt
qnx/python/           tars_gesture.py, qnx_cam.py (+ vendor/qnx_camera_helper.py from setup_pi.sh)
qnx/cpp/              tars_gesture.cc, BUILD, build_on_pi.sh
qnx/deploy.sh         Mac → Pi copy;  qnx/setup_pi.sh  one-time Pi setup
tests/                unit, parity, C++ syntax, end-to-end (handgen.py makes synthetic hands)
```

### Server hook (pi/src/main.c)
- `S` is now a **hard stop**: the wheels go to 0 on the next tick with no slew (before, it slewed down over ~140 ms).
  `STATUS` reports `stop_ms=` (S received → wheels commanded 0).
- New `VIS <mode> <fps> <cam_ms> <stop_ms> <label>`: the vision app reports
  about once a second. `STATUS` shows `vis= vis_fps= vis_cam_ms= vis_stop_ms= vis_see= vis_age_ms=`.
  It does not feed the watchdog.
