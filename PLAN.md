# PLAN: TARS rover, Hack Western 13

Shared plan for humans and coding agents. **Read [AGENTS.md](AGENTS.md) before changing code.**
Update the status table when you finish something.
Design for voice + YOLO + iPhone LiDAR fusion and the GOTO state machine: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

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
   voice/   speech → text → intent classifier → command
   vision/  MediaPipe hand gestures · follow mode · teach-a-gesture
   yolo     object detection (person / cup / ball …), teammate's code
        │  each one is a TCP client of ↓  (port 5555, text lines)
   pi/  tars_motor (C, SCHED_RR): watchdog, tip-over lock, Uno reconnect
        │  USB serial 115200
        ▼
[Arduino Uno]  "the muscles"   uno/tars/tars.ino
   2× 28BYJ-48 wheel steppers (ULN2003) · ramps · exact M <mm> / R <deg>
   button D2 · LED D13 · buzzer D12 · accelerometer tip/bump
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
  It's pure C/C++, so it builds with QNX `qcc`. It's downloaded by `voice/get_model.sh` and not committed.
  - Fallback if accuracy is poor: `ggml-base.en-q5_1.bin` (~57 MB, roughly 2–3× slower).
  - The prompt is biased toward the command vocabulary ("TARS, forward, back, left, right, stop, …").
- **Wake word:** "TARS …" by default, so crowd noise doesn't trigger moves.
- **Text → control classifier** (`voice/intent/`): whisper's output is noisy ("for word", "turn lift"), so text goes through a small trained classifier instead of exact string matching.
  1. **Safety pass first:** anything fuzzy-matching *stop / halt / freeze / no* sends `S` immediately. The classifier can't override it.
  2. **Intent classifier:** character n-gram features with a linear model (logistic regression). It's trained in Python on generated paraphrases plus simulated whisper mistakes, and the weights are exported to a C header, so it runs in microseconds on QNX with no ML runtime.
     Intents: `forward, back, left, right, turn_around, stop, follow, gestures, teach, spin, status, unknown`.
  3. **Slots:** a number parser ("ninety", "half a metre", "two steps") turns the intent into `TURN ±deg` or `MOVE ±mm`.
  4. **Confidence cutoff:** below the threshold the robot does nothing, then `LED no` + `BEEP 3`. A recognised command gets `BEEP 1`.

## Status

| Area | Branch | State |
|---|---|---|
| Uno stepper firmware, `tars_motor`, WASD teleop, `docs/PI_TEST.md` | `main` | Mac tests pass. **Not run on hardware yet.** |
| MediaPipe gestures + follow mode | `main` | Mac tests pass. Turns still timed for DC motors; must switch to `TURN`/`MOVE`. |
| Teach-a-gesture on the spot | `main` | Mac tests pass. Back burner. |
| Iteration 3: pet mode (`vision/common/pet.py` + `pet.hpp`) | `kothar/pet` | Mac only: 38/38 pet unit tests, Python vs C++ parity identical (pet sessions included). C++ app type-checked against stub headers. **Not built on QNX, not run on the Pi or with a real camera/hand.** |
| Speech + intent classifier (`voice/`) | `main` | Mac tests pass (stop recall 100%, 107/109 realistic transcripts, 0 wrong moves; crowd chatter ~4/21). **Not built or run on QNX/Pi yet.** Next: `voice/qnx/mic_test` on the Pi. Use a close-talk mic or the Uno talk button on the demo floor. |
| Object memory (`goto/` memory + `tars_motor` POSE + voice `scan`/`forget`) | `kothar/object-memory` (not pushed) | Mac: goto 185/185 C unit (also ASan/UBSan), 52/52 sim/socket checks; memory vs blind in the sim (16 scenes, teleop move between SCAN and GOTO): median 39 s / 1 turn vs 44 s / 3 turns, 16/16 arrived; tars_motor 293/293; voice 141/141 (stop recall 100%), e2e 52/52. **Pose drift on the real floor is unknown; nothing has run on the Pi.** |
| YOLO go-to-object (`goto/`: C `goto` + C++ `detect`) | `main` | Mac: 92/92 C unit (also under ASan/UBSan, parser fuzzed), 25/25 YOLO post, 34/34 socket+sim checks; 2D sim 40/40 random starts arrived (median 46 s, stops 0.49–0.58 m away); real `tars_motor_host -b stub` end to end; C++ `detect` + ONNX Runtime on the Mac matches ultralytics boxes to 0.1 px. **Not cross-compiled with qcc, camapi code never compiled, nothing run on the Pi.** |
| Goalie / balancer experiments | `goalie/`, `balancer/` | Parked. Don't touch. |

## Iteration 3: pet mode

TARS reacts to your hand like a small pet, using MediaPipe on the Pi (QNX C++ app `vision/qnx/cpp/tars_gesture.cc`
is the robot path). 👋 wave → wiggle hello · ✋ stay (stop) · ☝️ come here · 👍 happy spin (zoomies when
excited) · 👎 sad back-up · ✌️ flash = 360° trick, hold = leave · ✊ hold = play dead · no hand → bored
look-arounds, then sleep. A mood (`energy`) rises with 👍/👋 and decays, and scales speed. Enter with `--pet`
or by cycling ✌️ GESTURE → FOLLOW → PET. Rules + bounds: `vision/GESTURES.md` §8.

- **Stop still wins:** ✋ → `S` on the first frame from every pet state, nothing moves while it's up;
  camera stall, lost hand and a `D` refused by `tars_motor` (another client's `S`, watchdog, tip, guard)
  all stop and never auto-restart.
- **Bounded:** every pet `D` ≤ `pet_max_speed`, every wiggle segment ≤ `pet_max_seg_ms`, come-here ≤ 5 s per command.
- **CPU:** it's the MediaPipe mode in the CPU table (YOLO idle). Don't run it alongside go-to.
- **Like the gesture moves, the body language is timed `D` segments, not `TURN`/`MOVE`.** Tune `pet_*` on the real robot.

## Hardware checks that gate everything (do these first)

1. Upload `uno/tars/tars.ino`, then type `M 100` in Serial Monitor. Both wheels should turn.
2. Build and deploy the native C `tars_motor` (needs Docker + SDP 8.0 in `~/qnx800`). This is the demo path. Find the Uno's serial device name.
3. WASD drive from a laptop (`laptop/teleop.py`).
4. On the Pi: `apk search mediapipe`, and run the camera test.
5. On the Pi: `voice/qnx/mic_test` to record 3 s from the USB mic and play it back. **If this fails, voice is off.**
6. Measure fps with YOLO + MediaPipe running together, then fill in the CPU table above with real numbers.

## Next up

- Merge `voice-stt` once its tests pass, with the `goto <object>` intent added.
- `goto/`: build ONNX Runtime for QNX (qnx-ports, Linux/Docker host), cross-compile `goto` + `detect`, test camapi with the C270 (`detect --nomodel`), then tune `arrive_h_frac` on the real camera mount.
- Demo script: "TARS, go to the trash can" → search → drive → stop on arrival → "TARS, stop".
- Back burner: merge `learn-gesture`, and switch gesture turns to `TURN`/`MOVE`.
