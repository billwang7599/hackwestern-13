# PLAN: QNXplorer rover, Hack Western 13

Shared plan for humans and coding agents. **Read [AGENTS.md](AGENTS.md) before changing code.**
Update the status table when you finish something.
Design for voice + YOLO + iPhone LiDAR fusion and the GOTO state machine: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).
Repo layout, one folder per iteration: `iter1-wasd/` (WASD: Uno, `tars_motor`, laptop teleop), `iter2-autonomy/` (voice, goto, iPhone + Andrew's edge-AI/dashboard/iphone-usb), `iter3-pet/` (MediaPipe vision), `archive/` (goalie, pico, Joy-Con). Old → new paths: [docs/MOVED.md](docs/MOVED.md).

## Priorities (set by the team, 2026-11-21)

**Everything on the robot is QNX native (C/C++, qcc).** Python only on the laptop and in tests. Blocker #0: get the QNX SDP 8.0 toolchain working (`docker/run.sh`, `~/qnx800`).

1. **Keyboard + motor control.** Laptop WASD over Wi-Fi to the Pi, then to the Uno and the wheels. Demo path: native C `tars_motor`.
2. **Voice to command to YOLO "go to".** "TARS, go to the trash can": speech, then intent `goto` + object, then YOLO finds it, then the robot searches, steers and stops on arrival.
3. *Back burner:* MediaPipe gestures, follow mode, teach-a-gesture. Keep the code, but don't spend time on it.
4. *Iteration 3 (after 1 + 2 work on the robot):* **pet mode**, see below.

## The robot

A two-wheel rover that runs **fully on the robot**. No laptop and no internet at runtime.

```
 USB mic ─┐            ┌─ USB webcam (C270)
          ▼            ▼
[Pi 5 · QNX 8.0]  "the brain"
   iter2-autonomy/voice/   speech → text → intent classifier → command
   iter3-pet/vision/  MediaPipe hand gestures · follow mode · teach-a-gesture
   yolo     object detection (person / cup / ball …), teammate's code
        │  each one is a TCP client of ↓  (port 5555, text lines)
   iter1-wasd/pi/  tars_motor (C, SCHED_RR): watchdog, tip-over lock, Uno reconnect
        │  USB serial 115200
        ▼
[Arduino Uno]  "the muscles"   iter1-wasd/uno/tars/tars.ino
   default: Kuman SM11 kit · DC TT motors on the kit driver (EN+IN) · speed cap ~150 mm/s
   · timed M <mm> / R <deg> (no encoders) · kit button D13 · LED D3 · buzzer D12
   (option: 2× 28BYJ-48 steppers · exact M/R · button D2 · LED D13 · buzzer D12)
   · accelerometer tip/bump (optional)
```

The Pi decides. The Uno only moves motors and reports events. Nothing on the Pi touches a motor pin.

## CPU budget (Pi 5, 4 cores): pick by mode

Running YOLO, MediaPipe and speech together at full rate won't fit. These are estimates, not measured:

| Mode | MediaPipe Hands | YOLO (nano, small input) | Speech |
|---|---|---|---|
| **Gesture** (default) | every frame | off, or ~2 Hz | idle until voice activity |
| **Follow / seek object** | ~3 Hz, only to catch ✋ stop | every frame | idle until voice activity |
| **Teach** | every frame | off | off |

Speech uses `threads=2` and only runs on a ≤3 s segment after voice activity detection, so it's nearly free between commands.
Switch modes with a gesture (✌️), a voice command ("follow me", "gestures") or the Uno button.

## Speech: model and classifier

- **Speech-to-text model:** [whisper.cpp](https://github.com/ggml-org/whisper.cpp) with **`ggml-tiny.en-q5_1.bin`** (~31 MB, English only).
  It's pure C/C++, so it builds with QNX `qcc`. It's downloaded by `iter2-autonomy/voice/get_model.sh` and not committed.
  - Fallback if accuracy is poor: `ggml-base.en-q5_1.bin` (~57 MB, roughly 2–3× slower).
  - The prompt is biased toward the command vocabulary ("TARS, forward, back, left, right, stop, …").
- **Wake word:** "TARS …" by default, so crowd noise doesn't trigger moves.
- **Text → control classifier** (`iter2-autonomy/voice/intent/`): whisper's output is noisy ("for word", "turn lift"), so text goes through a small trained classifier instead of exact string matching.
  1. **Safety pass first:** anything fuzzy-matching *stop / halt / freeze / no* sends `S` immediately. The classifier can't override it.
  2. **Intent classifier:** character n-gram features with a linear model (logistic regression). It's trained in Python on generated paraphrases plus simulated whisper mistakes, and the weights are exported to a C header, so it runs in microseconds on QNX with no ML runtime.
     Intents: `forward, back, left, right, turn_around, stop, follow, gestures, teach, spin, status, unknown`.
  3. **Slots:** a number parser ("ninety", "half a metre", "two steps") turns the intent into `TURN ±deg` or `MOVE ±mm`.
  4. **Confidence cutoff:** below the threshold the robot does nothing, then `LED no` + `BEEP 3`. A recognised command gets `BEEP 1`.

## Status

| Area | Branch | State |
|---|---|---|
| Mounted iPhone video and spatial geometry | `main` | Set floor uses the mounted camera height and tracked tilt while facing forward. 44 geometry checks and 44 map tests passed. Upright mount estimate is 13.8 cm for RGB and 8.8 cm for LiDAR. Pi modules staged for the next dashboard launch. Installed the signed build on the physical iPhone 17 Pro Max. |
| Uno stepper firmware, `tars_motor`, WASD teleop, `docs/PI_TEST.md` | `main` | Mac tests pass. **Not run on hardware yet.** |
| **Kuman SM11 smart car port** (firmware `CAR_SM11` default, `tars_motor` WHEELS DC/SPEEDCAL, `wheel_cal.py dc-move/dc-spin [--slow]`, `docs/PI_TEST.md` §2b; DC slow turn for TURN speed < 1, so goto's 0.40 turns and the guard_blocked pivot are slow on the car too: `DC_SLOW_TURN_DUTY` 0.36 / 60°/s, both guesses) | `kothar/dc-car` → `kothar/integrate` | Mac only: Uno host tests sm11 248/248 (+ stepper 238, arm 260, dc 278, dc en+in 275); tars_motor 340/340 + guard 8/8; teleop 36/36; goto 247/84/25/52 + DC sloppy-motion sim 8/8; full-stack DC 64/65 (known memory-merge miss). **Wired bring-up:** the SM11 firmware compiled for the ATmega328P Uno (16,964 bytes), and the current SM11 host suite passed 274/274. The QNX Pi is running the updated USB motor service with its guard enabled. Flashed and verified the USB Uno from the Mac. At 115200 baud it reported READY and answered V with WHEELS DC 150.0 120.0 and WHEELS DCSLOW 60.0. Verified the Pi-to-kit USB handshake on QNX: USB 2341:0043, READY received, DC firmware settings recognized (150 mm/s, 120°/s, 60°/s slow). Physical motion remains untested. Speed cap 150 mm/s from the sim sweep. |
| MediaPipe gestures + follow mode | `main` | Mac tests pass. Turns still timed for DC motors; must switch to `TURN`/`MOVE`. |
| Teach-a-gesture on the spot | `main` | Mac tests pass. Back burner. |
| Iteration 3: pet mode (`iter3-pet/vision/common/pet.py` + `pet.hpp`) | `kothar/pet` | Mac only: 38/38 pet unit tests, Python vs C++ parity identical (pet sessions included). C++ app type-checked against stub headers. **Not built on QNX, not run on the Pi or with a real camera/hand.** |
| Speech + intent classifier (`iter2-autonomy/voice/`) | `main` | Mac tests pass (stop recall 100%, 107/109 realistic transcripts, 0 wrong moves; crowd chatter ~4/21). **Native Pi build added** (`make -C iter2-autonomy/voice native`: whisper.cpp/ggml from source with the Pi's clang, no SDP/cmake; `service.py` runs it with `TARS_ENABLE_VOICE=1`). That exact source list, built with MinGW on Windows (x86 kernels), transcribed a TTS "Tars, go to the trash can." word for word through the project's `stt.c`, and `intent_cli` then gave `GOTO trash can`. **Not built or run on QNX/Pi yet.** Next: `make native` on the Pi, then `build/native/mic_test`. Use a close-talk mic or the Uno talk button on the demo floor. |
| **Iteration 2: map-guided GOTO** (voice + YOLO + iPhone LiDAR; `iter2-autonomy/goto/nav.c`, RECALL/PLAN/FOLLOW_PATH/OCCLUDED/CHECK_OTHERS/DETOUR/NEXT_CANDIDATE, DL lease, nav feed 5572, voice `memory`/`recall` + speech to the phone 5573) | `kothar/fusion-linear` | Mac only: goto 265/265 C unit (also ASan/UBSan), test_nav 84/84 (+1M-packet LDR1 fuzz under ASan/UBSan), parity vs `mapping.py` 212/212, goto sim 52/52, **full-stack sim 33/33** (obstacle, occlusion → detour, occlusion right in front of the robot → back-off + detour, moved object, STOP mid-route, visual-only, voice memory + recall); tars_motor 313/313; voice 149/149 (stop recall 1131/1131, 0 wrong motions), e2e 52/52. **No real iPhone feeds 5572 yet; nothing built with qcc or run on the Pi/QNX/iPhone.** Assumptions: `iter2-autonomy/goto/README.md`. |
| Object memory (`iter2-autonomy/goto/` memory + `tars_motor` POSE + voice `scan`/`forget`) | in iteration 2 above | Mac: goto 185/185 C unit (also ASan/UBSan), 52/52 sim/socket checks; memory vs blind in the sim (16 scenes, teleop move between SCAN and GOTO): median 39 s / 1 turn vs 44 s / 3 turns, 16/16 arrived; tars_motor 293/293; voice 141/141 (stop recall 100%), e2e 52/52. **Pose drift on the real floor is unknown; nothing has run on the Pi.** |
| YOLO go-to-object (`iter2-autonomy/goto/`: C `goto` + C++ `detect`) | `main` | Mac: 92/92 C unit (also under ASan/UBSan, parser fuzzed), 25/25 YOLO post, 34/34 socket+sim checks; 2D sim 40/40 random starts arrived (median 46 s, stops 0.49–0.58 m away); real `tars_motor_host -b stub` end to end; C++ `detect` + ONNX Runtime on the Mac matches ultralytics boxes to 0.1 px. **Not cross-compiled with qcc, camapi code never compiled, nothing run on the Pi.** |
| Laptop mission control (`iter2-autonomy/mission/`: `mission_relay` C + `laptop/mission.py`; voice net input 5574; goto `DETS`/`NAV ROUTE`/`NAV GRID`) | `kothar/mission` → `kothar/integrate` | **Relay key required** (`-k` / `MISSION_KEY`; `--no-key` demo only). Mac only: relay 58/58, laptop app 33/33 (after the key commit; earlier: relay 44/44, laptop app 32/32 (STOP = S then CANCEL, kill -9 stops only its own drive), voice net 25/25 (stop first; mic + laptop at once), goto sim 60/60). Opt-in phone tee in Andrew's `headless.py` (only with `TARS_MISSION_TEE_PORT` set; approved, Andrew will be told); ports 5574–5576 pending Andrew's sign-off. **Not built with qcc, not run on the Pi, real Wi-Fi, iPhone or a real browser mic.** |
| **Real robot mode** for the sim's stack: `detect --camera broker:5566` (YOLO-World on the iPhone camera via `rgb_broker.py`, `--rotate`, libjpeg), `goto/build-native.sh` builds it on the Pi, `service.py` starts `detect` (`TARS_ENABLE_GOTO=1`, YOLOX off), `tars_voice` (`TARS_ENABLE_VOICE=1`) and `mission_relay` (`MISSION_KEY`) | `hardware_impl` | Windows/MinGW only: test_yolo 30/30 (+5 rotation checks); `detect.cpp` with the broker source syntax-checked against stub POSIX/libjpeg headers; `service.py` compiles. **Not built with clang on QNX, not run with the broker or a phone. Rotation default (270) and the iPhone `hfov_deg` are unverified.** |
| Goalie / balancer experiments | `archive/goalie/`, `balancer/` | Parked. Don't touch. |

## Iteration 3: pet mode

QNXplorer reacts to your hand like a small pet, using MediaPipe on the Pi (QNX C++ app `iter3-pet/vision/qnx/cpp/tars_gesture.cc`
is the robot path). 👋 wave → wiggle hello · ✋ stay (stop) · ☝️ come here · 👍 happy spin (zoomies when
excited) · 👎 sad back-up · ✌️ flash = 360° trick, hold = leave · ✊ hold = play dead · no hand → bored
look-arounds, then sleep. A mood (`energy`) rises with 👍/👋 and decays, and scales speed. Enter with `--pet`
or by cycling ✌️ GESTURE → FOLLOW → PET. Rules + bounds: `iter3-pet/vision/GESTURES.md` §8.

- **Stop still wins:** ✋ → `S` on the first frame from every pet state, nothing moves while it's up;
  camera stall, lost hand and a `D` refused by `tars_motor` (another client's `S`, watchdog, tip, guard)
  all stop and never auto-restart.
- **Bounded:** every pet `D` ≤ `pet_max_speed`, every wiggle segment ≤ `pet_max_seg_ms`, come-here ≤ 5 s per command.
- **CPU:** it's the MediaPipe mode in the CPU table (YOLO idle). Don't run it alongside go-to.
- **Like the gesture moves, the body language is timed `D` segments, not `TURN`/`MOVE`.** Tune `pet_*` on the real robot.

## Hardware checks that gate everything (do these first)

1. Unplug the SM11's Bluetooth module, upload `iter1-wasd/uno/tars/tars.ino`, then run the first-flash checks in `docs/PI_TEST.md` §2b (`D 0.5 0.5` both forward, `M 300`, `R 90`).
2. Build and deploy the native C `tars_motor` (needs Docker + SDP 8.0 in `~/qnx800`). This is the demo path. Find the Uno's serial device name.
3. WASD drive from a laptop (`iter1-wasd/laptop/teleop.py`).
4. On the Pi: `apk search mediapipe`, and run the camera test.
5. On the Pi: `iter2-autonomy/voice/qnx/mic_test` to record 3 s from the USB mic and play it back. **If this fails, voice is off.**
6. Measure fps with YOLO + MediaPipe running together, then fill in the CPU table above with real numbers.

## Ports (all localhost on the Pi)

Andrew's stack: 5555 (`tars_motor`) and 5557–5569 (`iter2-autonomy/edge-ai/README.md`). Ours: **5570** `goto` control (was 5557), **5571** detections → `goto` (was 5558), **5572** LDR1 nav feed → `goto`, **5573** QNXplorer speech → phone (Andrew's SpeechLink; `tars_voice` is a client), **5574** laptop → `tars_voice` (HEAR/AUDIO), **5575** `mission_relay` (the only one on Wi-Fi), **5576** phone status tee → relay, **5577** phone mic (AirPods) → `tars_voice` (MIC1; phone listens, voice dials). 5570–5572 and 5574–5577 are pending Andrew's sign-off.

## Next up

- Iteration 2 on hardware: have the phone (or Andrew's service) send a copy of its LDR1 packets to `goto` on 5572, check the frame/yaw signs with `NAV`, and get Andrew's sign-off on the pivot-while-BLOCKED rule in `tars_motor`, now limited to 0.40 speed / 180° (`iter2-autonomy/goto/README.md` assumption 7). The sideways sweep is still unguarded: measure the SM11 + phone-mount swing radius against the 0.35 m inflation.

- Merge `voice-stt` once its tests pass, with the `goto <object>` intent added.
- `iter2-autonomy/goto/`: build ONNX Runtime for QNX (qnx-ports, Linux/Docker host), cross-compile `goto` + `detect`, test camapi with the C270 (`detect --nomodel`), then tune `arrive_h_frac` on the real camera mount.
- Demo script: "TARS, go to the trash can" → search → drive → stop on arrival → "TARS, stop".
- Back burner: merge `learn-gesture`, and switch gesture turns to `TURN`/`MOVE`.
