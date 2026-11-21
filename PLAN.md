# PLAN: TARS rover, Hack Western 13

Shared plan for humans and coding agents. **Read [AGENTS.md](AGENTS.md) before changing code.**
Update the status table when you finish something.

## Priorities (set by the team, 2026-11-21)

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
Voice no longer switches vision modes: follow/gestures/teach were dropped as voice intents. Voice drives the go-to controller instead.

## Speech: model and classifier

- **Speech-to-text model:** [whisper.cpp](https://github.com/ggml-org/whisper.cpp) with **`ggml-tiny.en-q5_1.bin`** (~31 MB, English only).
  It's pure C/C++, so it builds with QNX `qcc`. It's downloaded by `voice/get_model.sh` and not committed.
  - Fallback if accuracy is poor: `ggml-base.en-q5_1.bin` (~57 MB, roughly 2–3× slower).
  - The prompt is a plain vocabulary list ("forward, back, left, right, stop, …, trash can, chair …"). A prompt full of "TARS, turn left." made tiny.en hallucinate "TARS, turn." on a bare "TARS".
- **Wake word:** "TARS …" by default, so crowd noise doesn't trigger moves. It's fuzzy ("Stars", "Tires", "Tons"…), carries over a pause ("TARS" … "turn left"), and a press of the **Uno button** = 4 s of listening with no wake word (best fix for a loud floor).
- **Everything on the robot is C** (QNX native): io-audio capture, VAD, whisper.cpp, intent inference, slot/object parser, TCP clients. Python is only for training, `try.py` and tests.
- **Text → control classifier** (`voice/intent/`): whisper's output is noisy ("for word", "turn lift"), so text goes through a small trained classifier instead of exact string matching.
  1. **Safety pass first:** anything fuzzy-matching *stop / halt / freeze / whoa* (or, after the wake word, *no / wait / hold on…*) sends `S` to `tars_motor` and `CANCEL` to the go-to controller. The classifier can't override it.
  2. **Intent classifier:** character n-gram features with a linear model (logistic regression). It's trained in Python on generated paraphrases plus simulated whisper mistakes, and the weights are exported to a C header, so it runs in microseconds on QNX with no ML runtime.
     Intents: `forward, back, left, right, turn_around, stop, goto, spin, status, unknown`. Phrases live in `voice/intent/phrases.txt`; retrain with `python3 voice/intent/train.py`.
  3. **Slots:** a number parser ("ninety", "half a metre", "two steps") turns the intent into `TURN ±deg` or `MOVE ±mm`. For `goto`, an object parser maps synonyms to the canonical list in `voice/intent/objects.txt` ("bin" → `trash can`). Then `GOTO <object>` goes to TCP 5557. An unknown object gets `LED no` + `BEEP 3`. A manual `MOVE`/`TURN` sends `CANCEL` to 5557 first.
  4. **Confidence cutoff:** below the threshold the robot does nothing, then `LED no` + `BEEP 3`. A recognised command gets `BEEP 1`.

## Status

| Area | Branch | State |
|---|---|---|
| Uno stepper firmware, `tars_motor`, WASD teleop, `docs/PI_TEST.md` | `stepper-wheels` (GitHub) | Mac tests pass. **Not run on hardware yet.** |
| MediaPipe gestures + follow mode | `stepper-wheels` (GitHub) | Mac tests pass. Turns still timed for DC motors; must switch to `TURN`/`MOVE`. |
| Teach-a-gesture on the spot | `kothar/learn-gesture` (local only) | Mac tests pass. Not merged. |
| Speech + intent classifier | `kothar/voice-stt` (local, not pushed) | **Mac tests pass; not built or run on QNX/Pi yet.** Classifier: 88.5% on unseen phrasings (cautious: 1.3% wrong, 40% "say again"), 107/109 realistic transcripts with 0 wrong moves, stop recall 100%, Python=C parity. End to end (macOS `say` → whisper tiny.en → real `tars_motor` stub): clean 21/21 ×2 voices, white noise 20 dB 18/21 ×2, crowd chatter 15 dB ~4/21 (never a wrong move). whisper 153 ms/command on M4 (Pi est. 0.6–1.2 s). Next: `voice/qnx/mic_test` on the Pi, then the QNX build. Use a close-talk mic or the Uno talk button on the demo floor. |
| Python bridge (no QNX build) | `kothar/py-bridge` (local, in progress) | Being built. |
| YOLO go-to-object | `kothar/goto` (local, in progress) | Being built. The teammate's YOLO plugs in via the detection interface. |
| Goalie / balancer experiments | `goalie/`, `balancer/` | Parked. Don't touch. |

## Hardware checks that gate everything (do these first)

1. Upload `uno/tars/tars.ino`, then type `M 100` in Serial Monitor. Both wheels should turn.
2. Build and deploy `tars_motor` (needs Docker + SDP 8.0 in `~/qnx800`). Find the Uno's serial device name.
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
