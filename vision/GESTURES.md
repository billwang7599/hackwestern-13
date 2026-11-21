# TARS gesture rules (the one place they're written down)

Three implementations follow these rules exactly:

| Where | Code |
|---|---|
| Mac prototype (`desktop/gesture_drive.py`) and QNX Python app (`qnx/python/tars_gesture.py`) | `common/gesture.py` + `common/learn.py` |
| QNX C++ app (`qnx/cpp/tars_gesture.cc`) | `common/gesture.hpp` + `common/learn.hpp` (line-for-line ports) |

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
| `teach` 🤙 | index + middle + ring folded, pinky extended (thumb ignored, any rotation) |
| `thumbs_up` / `thumbs_down` | all 4 folded, thumb out, thumb knuckle → tip points up / down |
| `fist` | all 4 folded (and not a thumbs up/down) |
| `unknown` | anything else (half-curled fingers, in-between poses) → then the **learned poses** are tried (§7) |

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

## 7. Teach a new pose on the spot (no screen, no laptop)

Code: `common/learn.py` / `learn.hpp` (`Brain`, which wraps the Controller).
Everything is done with hands and the robot's own body as the display.

### Trigger: 🤙 shaka, held `teach_hold_ms` (2 s)

Why shaka and not 🤘: the rule is "index, middle, ring folded + pinky out". No
built-in uses the pinky alone, and it doesn't involve the index finger. A sloppy
🤘 (index + pinky) degrades into ☝️ **point**, which would fire a move. A sloppy
🤙 degrades into ✊ (a back step, and only after 4 stable frames; the hold is 2 s).
Up to `learn_gap_frames` (3) flicker frames inside the hold are tolerated.
The robot holds still while 🤙 is up, in both modes. Teaching always returns to GESTURE mode.

Or press the **Uno button** (short press). See the interface below.

### Flow

| Step | You do | Robot does (feedback) |
|---|---|---|
| armed | hold 🤙 2 s (or press the button) | `S`, `LED teach`, `BEEP 1`, small left-right wiggle |
| countdown (`teach_countdown_ms` = 3 s) | drop the 🤙, get your new pose ready | one small forward-back **nudge** + `BEEP 1` per second (it's still at the end, so the camera is steady) |
| record | hold your **new** pose until it's done (`learn_frames` = 40 hand frames) | `LED rec`, `BEEP 2`, no motion |
| check | | reject → `LED no`, `BEEP 3`, **"no" shake** (left, right, left). Reasons: `too_similar <builtin>` if ≥ `learn_builtin_frac` (30%) of frames already read as a built-in (incl. 🤙 and ✋); `too_similar poseN` if an existing learned pose accepts ≥ `learn_similar_frac` of them; `full` (`learn_max_classes`); `no_hand` (no 40 frames within `learn_record_timeout_ms`) |
| assign | show an **existing** gesture for `learn_assign_frames` frames: 👉 turn right · 👈 turn left · 👍/☝️ forward · ✊/👎/👇 back · ✌️ spin · nothing for `learn_assign_ms` (5 s) = wiggle | on entry: `LED ok`, `BEEP 2`, **double wiggle** |
| confirm | | performs the chosen action **once**; the pose is saved as `pose1`, `pose2`, … |

- **✋ cancels** teaching in any step (`learn_cancel_frames` = 3 frames). The `S` goes out on the first palm frame, as always.
- **Forget all:** keep holding 🤙 through the whole countdown (2 + 3 = 5 s), or long-press the
  Uno button. Feedback: `LED forget`, `BEEP 4`, double back-forward "bow".
- A second short button press during teaching cancels it.
- After teaching, the gesture you used to pick the action is ignored until you change pose
  (so ✌️ for spin doesn't then toggle FOLLOW, and 👍 doesn't step forward). Then there's a
  `learn_cooldown_ms` pause before 🤙 can trigger again.
- `LED`/`BEEP` are optional tars_motor commands. If the server answers `ERR`, nothing
  breaks (the motions carry the meaning). `feedback_led_beep = 0` stops sending them.
- During teach feedback the hand-lost rule is paused (the motions are < 1 s and self-ending).
  ✋, the camera-stall stop and the server watchdog still apply.

### Recognition (few-shot k-NN, on the Pi)

1. **Features** (57 numbers): the 21 points (x aspect-corrected, z ignored) with the wrist at 0,
   scaled so wrist→middle knuckle = 1, rotated so that vector points up, mirrored so the
   index knuckle is left of the pinky knuckle (left hands = right hands; palm/back the same).
   Plus the 15 finger-joint bend angles × `learn_angle_weight`. So position, size, in-plane
   rotation and handedness don't matter.
2. **Examples:** the 40 recorded frames + `learn_augment` (2) jittered copies each
   (±`learn_jitter`, deterministic RNG so Python and C++ store the same thing).
3. **Accept radius per pose** = `learn_thr_mult` × (mean + 2 sd) of its own leave-one-out
   k-NN distances, clamped to [`learn_thr_min`, `learn_thr_max`].
4. **Per frame:** only when the built-in rules say `unknown`. Score each pose = mean distance
   to its `learn_k` (3) nearest examples; ratio = score / radius. Best ratio ≤ 1 → that pose,
   otherwise `unknown` (no action). Confidence = 1 / (1 + ratio²), so 0.5 = on the radius.
5. The pose label then goes through the **same debounce / cooldown / hold rules** as a
   built-in (§3). Its intent is `<action>@<pose>`, e.g. `spin@pose1`.

**Safety:** built-in rules run first and learned poses are only consulted on `unknown`, so a
learned pose can never shadow ✋ (or any built-in). In every teach state the Controller still
receives ✋ as ✋. Learned actions end with `S` and obey hand-lost, ✋ and camera-stall like any move.

**Actions:** `forward`/`back`/`turn_left`/`turn_right` (same as the built-ins); `spin` =
`turn_ms × spin_turns` (360° once the turn is calibrated); `wiggle` = 5 swings of `wiggle_ms`;
`arms` = `G wave` then wait `arms_ms` (set in JSON); `custom` = a list of segments in the JSON:
`["D", l, r, ms]` or `["X", "<line>", 0, ms]`.

**Storage:** `vision/learned.json` (`--learned`), written atomically on every change and
loaded at start: `{"version":1, "next_seed":N, "classes":[{"name","action","seed","thr","custom","examples":[[57 floats]...]}]}`.
Jittered copies are rebuilt from `seed` on load.

**Timing** (`tests/bench_learn.py`, this Mac, features + k-NN per frame, 120 examples/pose):
1 pose 6 µs C++ / 160 µs Python · 4 poses 16 µs / 640 µs · 8 poses 32 µs / 1.3 ms.
MediaPipe inference is milliseconds, so this is noise either way. The Pi 5 should be ~2–4× slower (not measured).

### Uno button interface (for the owners of `uno/` and `pi/`; not implemented by vision/)

- **Uno → tars_motor** (serial line): `BTN TEACH` on a short press, `BTN FORGET` on a long
  press (≥ 2 s, the Uno decides). Debounce on the Uno.
- **tars_motor**: count them and add two fields to `STATUS`: `btn_teach=<n> btn_forget=<n>`
  (cumulative counters since start, space-separated `key=value` like the other fields).
- **vision app**: polls `STATUS` every 200 ms. The first reply sets the baseline, and any
  increase = one press. If the fields are missing, it just doesn't poll buttons
  (`--no-buttons` turns polling off).
- Optional **feedback commands** the vision app sends (reply `OK`, or `ERR` if unsupported):
  `LED <teach|rec|ok|no|forget|off>` and `BEEP <n>` (n short beeps). A buzzer on a spare Uno pin is enough.
- The vision app's `VIS` mode field shows `teach:<state>` while teaching (≤ 15 chars).
