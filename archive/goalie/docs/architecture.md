# Beat the Goalie: architecture and protocol contract

This file is the **contract** between `vision/`, `pi/`, `arduino/` and `sim/`.
Change it only with a matching change on both sides of a link.

```
[Laptop]  webcam -> vision/track.py (ball + paddle, homography -> metres)
          ui.py scoreboard, taunt.py (Claude + say), switch_input.py
     │  TCP :5556, line protocol, ~30-60 Hz frames            goalie_link.py
     ▼
[Pi 5 · QNX 8.0]  goalie_motor: 200 Hz SCHED_RR loop
          predict ball crossing x on the goal line -> target paddle x
          motor-model observer (sent duties + late camera paddle x) + PD -> duty
          watchdog (no frame for WD ms -> park centre), save/goal counters,
          QNX/LINUX mode switch, jitter stats
     │  USB serial (or UART fallback) 115200 8N1, line protocol
     ▼
[Arduino Uno]  H-bridge -> DC motor #1 (goalie), DC motor #2 (flag),
               ULN2003 -> 28BYJ-48 (scoreboard needle), MPU6050 (hit detect)
```

## Field coordinates (shared by all components)

Units: metres, seconds. Origin at the **centre of the goal mouth on the goal line**.

- `x`: across the goal. `+x` = right as seen from the overhead camera with the
  goal at the bottom of the image. Goal spans `[-GOAL_HALF_W, +GOAL_HALF_W]`,
  default `GOAL_HALF_W = 0.11` (22 cm goal).
- `y`: distance out from the goal line towards the shooter, `+y` = into the field.
  The ball comes in with `vy < 0`. Field length default 0.60 m.
- Paddle position is reported as the `x` of the paddle centre (for a pivot arm,
  the x of the paddle tip/blade centre projected on the goal line).

## Link 1: laptop -> Pi (TCP, port 5556)

One command per line, ASCII, `\n` terminated. Every command gets exactly one
reply line: `OK ...` or `ERR <reason>`.

| Command | Meaning |
|---|---|
| `F <t_ms> <ball_seen> <bx> <by> <paddle_seen> <px>` | One camera frame. `t_ms` = laptop monotonic ms (integer). `*_seen` is `0`/`1`; when 0 the coordinate fields are ignored (send `0`). Feeds the watchdog. Reply: `OK <target_x> <duty>` |
| `MODE QNX` / `MODE LINUX` | Real-time mode vs emulated-Linux mode (see below). Reply `OK QNX` / `OK LINUX` |
| `HUMAN <x>` | Human-goalie mode (Switch controller): track this target x instead of the prediction. Must be resent like frames. |
| `AUTO` | Leave human mode, back to prediction. |
| `GOAL` | Vision saw the ball cross the goal line inside the goal: goals++. |
| `RESET` | Zero saves/goals/jitter stats. |
| `PING` | `OK PONG` (also feeds the watchdog). |
| `STATUS` | `OK key=value ...` (space-separated, order not guaranteed). Keys at least: `mode=QNX\|LINUX`, `ctl=AUTO\|HUMAN`, `saves=N`, `goals=N`, `target=<x>`, `paddle=<x>`, `duty=<d>`, `wd=0\|1` (1 = watchdog parked), `ms_since_frame=N`, `jit_avg_us=N`, `jit_max_us=N`, `loop_hz=<f>`, `hits=N` |

Clients must parse STATUS as key=value and ignore unknown keys.

Extra STATUS keys `goalie_motor` emits (informational, may change):
`sched=RR|OTHER` (actual policy of the control thread), `emulated=0|1` (1 in
LINUX mode), `pred=NONE|AWAY|INCOMING`, `armed=0|1` (save counter armed),
`stopped=0|1` (no paddle feedback -> `S` sent), `paddle_seen=0|1`,
`kp=<f> kd=<f>` (active controller gains), `accel_mg=N` (last Uno `A`, -1 if
none), `backend=arduino|stub`, `ctl_kind=observer|pd`, `paddle_est=<x>`
(controller's belief of the paddle x now), `model_rpm=<f>`, `uno_ready=N`
(READY lines seen; > 1 = the Uno rebooted).

Extensions (not needed by the laptop; reply format as shown):

| Command | Meaning |
|---|---|
| `GAINS <kp> <kd>` | Live tuning of the active controller (observer: duty/rad, duty/(rad/s); pd: duty/m, duty/(m/s)). Reply `OK <kp> <kd>`. |
| `SIMHIT <mag>` | **Test only**, stub backend: pretend the Uno sent `HIT <mag>`. `ERR` with the arduino backend. `mock_pi.py` implements it too. |
| `SIMUNO <line>` | **Test only**, stub backend: pretend the Uno sent any line. |

**Prediction (Pi, = `sim/predict.py`, checked by `pi/make parity`):** straight
line least-squares fit over the last 5 ball samples (laptop timestamps); a gap
> 0.15 s starts a new fit. Incoming when `vy < -0.15 m/s` and the crossing is
< 2 s away: target = crossing x. Otherwise the paddle shadows the ball half
way: target = `0.5 * x`. Ball unseen for 0.25 s: target 0. Targets are clamped
to the goal (and the paddle's reach). Side walls (bank shots) are **off by
default**; `goalie_motor --walls 0.25` folds the crossing between walls at
±0.25 m (the ball centre bounces one radius, 0.02 m, from the wall).

**Save logic (Pi):** a save is counted when the Uno reports `HIT` while a shot
is in flight (ball moving towards the goal, `vy < -0.15`, within the last
300 ms), or when the ball bounces back (`vy` goes from < -0.15 to > +0.15)
after coming within 0.10 m of the goal line. Debounce: one save per shot,
re-armed when the ball is > 0.25 m out or not seen for 1 s.

**Watchdog:** no `F`/`PING`/`HUMAN` for `WD_MS` (default 300 ms) -> target x = 0
(park centre) with the paddle-feedback loop running if the paddle was last seen
recently, otherwise send `S` (stop) to the Uno. `wd=1` in STATUS.

**Modes:** `QNX` = SCHED_RR high priority, absolute-time `clock_nanosleep` ticks.
`LINUX` = the same loop at normal priority **plus injected scheduling jitter**
(random 0-30 ms sleeps, configurable) to emulate a non-real-time OS under load.
This is an emulation for the demo and must be labelled as such in docs and UI.

## Link 2: Pi -> Uno (serial 115200 8N1)

ASCII lines, `\n` terminated. The Uno never replies to commands (fire and forget);
it only sends events.

| Pi -> Uno | Meaning |
|---|---|
| `M <duty>` | DC motor #1 (goalie), duty float in `[-1, 1]`, `+` moves the paddle towards `+x`. Sent every control tick (200 Hz). |
| `S` | Stop all motors. |
| `N <steps>` | Scoreboard needle: move the 28BYJ-48 to absolute step position (non-blocking). |
| `C <ms>` | Celebrate: spin DC motor #2 (flag) for ms, then stop. |
| `Z` | Zero the needle at its current position. |

| Uno -> Pi | Meaning |
|---|---|
| `READY` | Sent once after boot. Opening the USB port resets the Uno, so expect one at start. A `READY` **mid-run means the Uno rebooted** (brown-out, USB glitch): its needle zero is lost (re-centre the needle by hand, then `Z`). The Pi logs it and counts it in `uno_ready`. |
| `HIT <mag>` | Accelerometer spike above threshold; `mag` = peak deviation in milli-g. Rate-limited to one per 250 ms. |
| `A <mg>` | Optional low-rate (≤10 Hz) accel magnitude for debugging. Pi may ignore. |
| `# <text>` | Status/debug text (e.g. `# no accelerometer found`). **The Pi must ignore lines starting with `#`** (it only logs them). |

**Uno watchdog:** no `M` for 200 ms -> motor #1 stops (brakes).

**Static-friction boost lives on the Pi only.** The Pi's controller adds the
deadband compensation (observer `db_comp` 0.12, or 0.15 minimum duty in
`--ctl pd`). The Uno's own kick/boost setting stays **0** so the two don't add
up. Measure the duty where the paddle starts moving on the bench and set it in
one place (`goalie_motor --db-comp`).

## Defaults (keep in sync with `sim/params.py`)

| Name | Value |
|---|---|
| Control rate | 200 Hz |
| Laptop frame rate | 30-60 Hz |
| `WD_MS` laptop->Pi | 300 ms |
| Uno motor watchdog | 200 ms |
| Goal width | 0.22 m |
| Field length | 0.60 m |
| Paddle (pivot arm) length | 0.12 m |
| Paddle blade width (pivot) | 0.05 m (a still 5 cm blade already saves ~50%; wider makes the game too easy) |
| Side walls (bank-shot prediction) | off; ±0.25 m when enabled (sim `field_half_w`). Vision's calib default board width is 0.40 m: measure your board and pass its half width to `--walls` |
| Camera latency the Pi assumes | 50 ms (`--delay-ms`) |
| Motor no-load RPM | 150 placeholder: **measure it** (`docs/rpm_test.md`), pass `--rpm` |
| Controller (observer) | kp 3.0 duty/rad, kd 0.10, db_comp 0.12, min_duty 0.06, friction offset 0.10, L1 0.5, L2 2.0, tau 0.04 s |
