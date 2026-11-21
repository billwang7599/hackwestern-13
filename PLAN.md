# PLAN: TARS rover, Hack Western 13

Shared plan for humans and coding agents. **Read [AGENTS.md](AGENTS.md) before changing code.**
Update the status table when you finish something.

## Priorities (set by the team, 2026-11-21)

**Everything on the robot is QNX native (C/C++, qcc).** Python only on the laptop and in tests. Blocker #0: get the QNX SDP 8.0 toolchain working (`docker/run.sh`, `~/qnx800`).

1. **Keyboard + motor control.** Laptop WASD over Wi-Fi to the Pi, then to the Uno and the wheels. Fast path: `pi/tars_bridge.py` (Python, no QNX build), branch `kothar/py-bridge`.
2. **Voice to command to YOLO "go to".** "TARS, go to the trash can": speech, then intent `goto` + object, then YOLO finds it, then the robot searches, steers and stops on arrival.
3. *Back burner:* MediaPipe gestures, follow mode, teach-a-gesture. Keep the code, but don't spend time on it.

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
| Uno stepper firmware, `tars_motor`, WASD teleop, `docs/PI_TEST.md` | `stepper-wheels` (GitHub) | Mac tests pass. **Not run on hardware yet.** |
| `pi/tars_bridge.py`: Python stand-in for `tars_motor`. **Bring-up/debug only** (test motors before the QNX build works); **not the demo path**, which is the native C `tars_motor` | `kothar/py-bridge` (local) | Passes all 240 `tars_motor` checks + 34 bridge checks + teleop 36/36 on Mac (stub + fake-Uno pty). **Not run on QNX/Pi/real Uno yet.** |
| MediaPipe gestures + follow mode | `stepper-wheels` (GitHub) | Mac tests pass. Turns still timed for DC motors; must switch to `TURN`/`MOVE`. |
| Teach-a-gesture on the spot | `kothar/learn-gesture` (local only) | Mac tests pass. Not merged. |
| Speech + intent classifier | `kothar/voice-stt` (local, in progress) | Being built. |
| Python bridge (no QNX build) | `kothar/py-bridge` (local, in progress) | Being built. |
| YOLO go-to-object | `kothar/goto` (local, in progress) | Being built. The teammate's YOLO plugs in via the detection interface. |
| Goalie / balancer experiments | `goalie/`, `balancer/` | Parked. Don't touch. |

## Hardware checks that gate everything (do these first)

1. Upload `uno/tars/tars.ino`, then type `M 100` in Serial Monitor. Both wheels should turn.
2. Build and deploy the native C `tars_motor` (needs Docker + SDP 8.0 in `~/qnx800`). This is the demo path. Find the Uno's serial device name.
   *Fallback while `qcc` isn't working:* `pi/tars_bridge.py` (docs/PI_TEST.md §1d). It's for bring-up and debugging only.
3. WASD drive from a laptop (`laptop/teleop.py`).
4. On the Pi: `apk search mediapipe`, and run the camera test.
5. On the Pi: `voice/qnx/mic_test` to record 3 s from the USB mic and play it back. **If this fails, voice is off.**
6. Measure fps with YOLO + MediaPipe running together, then fill in the CPU table above with real numbers.

## Next up

- Merge `py-bridge` once its tests pass (priority 1).
- Merge `voice-stt` once its tests pass, with the `goto <object>` intent added.
- `goto/` (branch `kothar/goto`): a YOLO detection interface (JSON lines: label, conf, box), the search-and-steer program, and the YOLO-World custom-vocabulary export.
- Demo script: "TARS, go to the trash can" → search → drive → stop on arrival → "TARS, stop".
- Back burner: merge `learn-gesture`, and switch gesture turns to `TURN`/`MOVE`.
