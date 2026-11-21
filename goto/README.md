# goto/: "TARS, go to the trash can"

Object name in, wheels out. Everything that runs on the robot is **QNX native C/C++** built with `qcc`/`q++`. Python is used only on the laptop: model export and the Mac simulator tests.

```
voice app ──"GOTO trash can"──► goto (C, SCHED_RR 20) ──D / S / TURN / BEEP / LED──► tars_motor (C, SCHED_RR 30) ──► Uno
                                   ▲  TCP 5557 control                         TCP 5555
webcam ──► detect (C++, ONNX Runtime) ──JSON lines, UDP 127.0.0.1:5558──┘
           (or the teammate's YOLO: same format, see DETECTIONS.md)
```

| File | What |
|---|---|
| `src/goto.c`, `src/goto_core.[ch]` | the controller: control port, detection input, state machine, `tars_motor` client |
| `src/detect.cpp`, `src/camera.hpp`, `src/yolo_post.hpp` | the detector: QNX camera, YOLO via ONNX Runtime, JSON out |
| `objects.txt` | object names + aliases (shared with `voice/`) |
| `goto.conf` | every threshold, with comments |
| `export_world.py`, `get_model.sh` | laptop: YOLO-World with our classes, exported to ONNX |
| `DETECTIONS.md` | the detector → goto interface |
| `tests/` | C unit tests, YOLO post-processing tests, Python 2D simulator + socket tests |

## How it behaves

1. **GOTO \<object\>** (`objects.txt` aliases: "bin", "garbage", "me" → person, …). It beeps once and looks at fresh frames **before** turning, since the object may already be in view.
2. **Search:** if it doesn't see the object, it does an exact `TURN 30`, waits for the turn to finish plus `settle_s`, then looks at 4 fresh frames (it needs 2 hits). After 360° with nothing, it stops with `BEEP 3` and the result `failed:not_found`. It turns toward the side where the object was last seen.
3. **Align:** if the object's bearing is more than 12°, it does one exact `TURN` of 0.9 × the bearing. The steppers are exact, so this beats steering with 150 ms camera lag.
4. **Approach:** it sends `D l r` at 10 Hz (the watchdog is 500 ms): a P controller on the bearing (EMA-smoothed box centre, 2° deadband), with forward speed reduced when it's off-centre. If the bearing passes 20° it stops and re-aligns.
5. **Arrived:** the box height is ≥ 70% of the frame (person: 80%), or the box bottom touches the frame bottom. That has to hold for 2 frames. Then it sends `S`, `BEEP 1` and `LED ok`.
6. **Lost:** no target for 1.2 s → `S`, then search again, starting toward where it was last seen.
7. **Stop always wins:** `CANCEL` on 5557, or **an `S` from any other client** (voice "stop", teleop, ✋), cancels the GOTO. `goto` watches `tars_motor`'s `stops=` counter. If `goto` dies, `tars_motor` sees the disconnect and stops the wheels. Tipped, bumped, obstacle or a refused `D` all → `S` + `BEEP 3`.

**Arrival thresholds depend on how the webcam is mounted.** The defaults assume it's about 12 cm up and level, and they stop about 0.5–0.7 m from a 0.4–0.8 m tall object. On the robot, drive up to the target by hand, read `h=` from `STATUS` and set `arrive_h_frac` in `goto.conf`.

## Control port (TCP 5557)

```
GOTO <words>    -> OK goto trash can | ERR unknown object '...' (robot beeps 3)
CANCEL|STOP|S   -> OK                 (S goes to tars_motor first, even when idle)
STATUS          -> OK state=approach target=trash_can result=running seen=1 bearing=-3.2 h=0.31 searched=0 fps=7.9 ...
OBJECTS         -> OK trash can,chair,person,...
RESOLVE <words> -> OK trash can | ERR unknown object
WATCH           -> OK, then async "EVENT searching|arrived|lost|cancelled|failed:<why> <object>" lines
PING            -> OK PONG
```

**For the voice app:** on intent `goto` + object, send `GOTO <object words>` to 5557 and speak or beep based on the `EVENT` lines. On "stop", send `S` to 5555 (the fastest path). `goto` notices it by itself.

## Run order on the Pi

```sh
./tars_motor -b uno -d /dev/serusb1 &                                     # 1. motors (pi/)
./goto -c goto.conf -o objects.txt &                                      # 2. controller
./detect --camera camapi:1 --model models/yolo_world.onnx --labels models/labels.txt &   # 3. eyes
printf 'GOTO trash can\n' | nc 127.0.0.1 5557                             # or the voice app
```

## Build

```sh
make -C goto host && make -C goto test     # Mac: goto_host, unit tests, simulator (no QNX needed)
make -C goto                               # QNX: build/goto (qcc via docker/run.sh, like pi/)
make -C goto detect ORT_ROOT=<staging>     # QNX: build/detect (needs ONNX Runtime for QNX, below)
make -C goto deploy PI_HOST=qnxuser@<pi-ip>
```

`goto` uses only libc, BSD sockets (`-lsocket`) and `-lm`. `detect` also needs `-lonnxruntime` and `-lcamapi`.

### Model (laptop)

```sh
goto/get_model.sh            # venv + ultralytics, then export_world.py: YOLO-World v2 (s) with the
                             # objects.txt classes baked in -> goto/models/yolo_world.onnx + labels.txt
goto/get_model.sh --coco yolo11n.pt   # faster plain COCO model (no "trash can")
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

## Mac tests (what has actually run)

```sh
make -C goto test
```

- `test_core`: aliases, the JSON parser, config, and the state machine on scripted detection streams with a fake clock.
- `test_yolo`: YUV→RGB, letterbox, YOLO decode for both output layouts, NMS, and the `detect` JSON parsed back by `goto`'s C parser (the contract test).
- `test_goto_sim.py`:
  - the C `goto` over real sockets against a fake `tars_motor` with differential-drive kinematics, and a pinhole camera sim (60° HFOV, 12 cm mount, 150 ms latency, 10% misses, jitter, motion blur, a distractor chair)
  - CANCEL latency, and an external `S` from another client
  - the TCP and stdin inputs
  - the real `pi/build/tars_motor_host -b stub` end to end
  - random start poses

Results on the Mac (2026-11-21), host builds only:

| Suite | Result |
|---|---|
| `test_core` (C state machine, parser, aliases, config) | 92/92. It also passes under ASan/UBSan. The JSON parser was fuzzed with 300k mutated lines under ASan/UBSan, and one UB was found and fixed (huge `w`/`h`). |
| `test_yolo` (pre/post-processing, JSON contract) | 25/25 |
| `test_goto_sim.py` sockets + sim + real `tars_motor_host -b stub` | all checks pass. CANCEL → `S` at the motor server: median 0.2 ms, max 0.24 ms. An external `S` cancels within one STATUS poll. |
| 2D sim, 40 random start poses (0.8–3 m, any bearing, 0.4/0.6/0.8 m objects) | **40/40 arrived**. Median 46 s, max 77 s. Stopped 0.49–0.58 m away, facing within 2°. It never touched the object. |
| `detect` with ONNX Runtime 1.32 (macOS arm64) + exported YOLO-World 320 px | boxes match ultralytics' own ONNX predict to within 0.1 px and 0.001 confidence on `bus.jpg`. About 11 ms/frame on an M-series Mac with 2 threads. The Pi 5 will be several times slower. |
| detect → goto → `tars_motor_host` pipeline | real YOLO boxes of a person at −20.5° → `goto` → `TURN -18.5` running on `tars_motor`. |

To build `detect` with real inference on a Mac: get ONNX Runtime headers and a dylib, then `make host-detect ORT_HOST=<dir with include/ lib/>`. That produces `build/detect_host_ort`.

**Not done yet:**
- `goto` and `detect` haven't been cross-compiled with `qcc`/`q++`: there's no SDP or Docker on this Mac.
- The camapi source has never been compiled.
- ONNX Runtime for QNX hasn't been built.
- Nothing has run on the Pi, with the C270 or on the real robot.
- The arrival threshold is tuned in the simulator only.
