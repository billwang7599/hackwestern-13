# TARS gesture rules (the one place they're written down)

Three implementations follow these rules exactly:

| Where | Code |
|---|---|
| Mac prototype (`desktop/gesture_drive.py`) and QNX Python app (`qnx/python/tars_gesture.py`) | `common/gesture.py` |
| QNX C++ app (`qnx/cpp/tars_gesture.cc`) | `common/gesture.hpp` (line-for-line port) |

`tests/test_parity.py` runs the Python and C++ versions on the same ~50,000
lines of random sessions and fails on the first differing label, mode or
command. **All numbers live in `gesture.conf`**, which both versions read.
Names in `code font` below are keys in that file.

## 1. What the camera sees → a label

MediaPipe HandLandmarker gives 21 points per hand. The frame is mirrored first
(`flip = 1`, selfie view: *your* left is image left) and downscaled to
`infer_width`. All distances are aspect-corrected, so they're in units of image
height.

- **palm size** = wrist → middle-finger knuckle (points 0 → 9)
- a finger (index/middle/ring/pinky) is **extended** if
  `|wrist→tip| > ext_ratio × |wrist→PIP joint|`, and **folded** if `< fold_ratio ×`.
  These are ratios, so they work at any distance and rotation.
- the thumb is **out** if `|thumb tip − index knuckle| > thumb_out × palm`
- a **direction** is up/down/left/right if the vector is within
  `dir_tolerance_deg` of that axis, otherwise it's undecided

Checked in this order:

| Label | Rule |
|---|---|
| `no_hand` | MediaPipe found no hand |
| `palm` | all 4 fingers extended |
| `peace` | index + middle extended, ring + pinky folded |
| `point_up/down/left/right` | only index extended (others folded); direction of index knuckle → tip (undecided → `unknown`) |
| `thumbs_up` / `thumbs_down` | all 4 folded, thumb out, thumb knuckle → tip points up / down |
| `fist` | all 4 folded (and not a thumbs up/down) |
| `unknown` | anything else (half-curled fingers, in-between poses) |

## 2. Modes

Starts in **GESTURE** mode. Holding a **peace sign ✌️** for `mode_frames`
consecutive frames toggles GESTURE ↔ FOLLOW and sends `S`. A second toggle needs
**both** a release (any non-peace frame) **and** `mode_cooldown_ms` since the
last toggle, so holding the sign never flips it back. The mode shows in the log
`[status] mode=…`, in the server's `STATUS` (`vis=…`), and on the desktop overlay.

## 3. GESTURE mode: discrete steps

| Gesture | Move |
|---|---|
| ✋ open palm | **STOP** (`S`) after `stop_frames` (default 1) frames, and cancels any move |
| 👍 thumbs up / ☝️ point up | **forward step**: `D +step_speed +step_speed` for `step_ms` |
| ✊ fist / 👇 point down / 👎 thumbs down | **back step**: same, reversed |
| 👈 point left / 👉 point right | **turn 30°**: wheels ±`turn_speed` for `turn_ms` (calibrated, see below) |

- **Debounce:** a move fires only after `stable_frames` identical consecutive
  frames. Any different label resets the count.
- **One at a time:** while a move runs, other moves are ignored (palm still stops it).
- **Cooldown:** after a move ends or is stopped, the next one waits `cooldown_ms`.
- **Holding:** with `hold_repeat = 1`, holding 👍 repeats the step after each
  cooldown. With `0`, you must release first.
- **Resend:** during a move, `D` is resent every `resend_ms` (the server's
  watchdog stops the wheels if `D` is older than 500 ms). When the move ends, one `S`.
- **"Point left":** by default the rover turns to *its* left
  (`point_left_is_left = 1`). Set it to 0 to swap, e.g. if it faces you and you
  want it to turn toward where you point.

**Calibrating the 30° turn:** run `desktop/calibrate_turn.py --host tars.local`.
It does 12 turns, which should add up to 360°. Then set
`turn_ms = turn_ms × 360 / measured`. The server's slew limit ramps the wheels
up over ~140 ms, so the turn isn't linear in `turn_ms`. Measure, don't compute.

## 4. FOLLOW mode: steer toward the hand

Any detected hand except ✋ and ✌️ is followed (a fist is the steadiest).
`flip` is undone first, so "left" means the rover's own left (the camera looks
where the rover looks).

```
x      = palm centre x in the rover's view (mean of wrist + 4 knuckles), 0..1
err    = x − 0.5                       (+ = hand is to the rover's right)
turn   = 0                             if |err| ≤ follow_deadband
       = follow_kp_turn·(|err| − follow_deadband)·sign(err), clamped ±follow_max_turn
size   = palm size (image heights; bigger = closer)
e      = follow_target_size − size     (+ = too far)
fwd    = follow_kp_fwd·(e − follow_size_deadband)  clamped [0, follow_max_fwd]   if e >  follow_size_deadband
       = follow_kp_fwd·(e + follow_size_deadband)  clamped [−follow_max_back, 0] if e < −follow_size_deadband
       = 0 otherwise                   (follow_max_back = 0: never reverse)
target = (fwd + turn, fwd − turn), clamped ±1
```

- **Stream:** while following, `D l r` goes out every `follow_send_ms` (15 Hz),
  even when it's `D 0.00 0.00`, so the watchdog stays fed and the robot holds still.
- **Slew:** each send moves the wheels toward the target by at most
  `follow_slew_per_s × dt`.
- **Hand lost:** after `follow_grace_frames` consecutive no-hand frames → `S`,
  and silence until a hand returns. Within the grace period it keeps going.
- **✋ held** for `follow_palm_frames` frames → `S`, and it stays stopped while the palm is up.
- **✌️** sets the target to 0 (it holds still while you switch modes).
- `follow_invert_turn = 1` if it steers away from your hand.

## 5. Fail-safes (both modes)

| Condition | Action |
|---|---|
| no camera frame for `frame_timeout_ms` (checked by a 20 ms timer thread, independent of the camera) | `S` |
| hand lost (gesture: `lost_ms`; follow: `follow_grace_frames`) | `S` |
| vision app dies / Wi-Fi drops | tars_motor's own 500 ms watchdog and disconnect-stop |
| robot tipped / bumped | tars_motor refuses `D` (`ERR …`), logged by the app |

`S` is a **hard stop** on the server: the wheels go to 0 on the next 50 Hz tick
with no slew. The server reports that time as `stop_ms=` in `STATUS`.

## 6. Latency (the demo number)

- `cam->cmd`: camera frame captured → command written **and acknowledged** by tars_motor
- `palm->stop`: the same, for the last ✋ → `S`
- server `stop_ms`: `S` received → wheels commanded 0 (≤ 1 tick = 20 ms)

So "**sees your palm → motors stopped**" ≈ `vis_stop_ms + stop_ms`. MediaPipe
inference time is reported separately (`infer=`). On the Mac (M-series,
localhost) the end-to-end test measured `palm->stop` ≈ 6 ms and server
`stop_ms` 2–16 ms. **The Pi numbers are still to be measured.**
