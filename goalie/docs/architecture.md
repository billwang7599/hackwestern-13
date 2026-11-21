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
          PD on (target - measured paddle x) -> motor duty
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

**Save logic (Pi):** a save is counted when the Uno reports `HIT` while a shot
is in flight (ball moving towards the goal, `vy < -0.05`), or when the ball
reverses direction (`vy` goes from < -0.05 to > +0.05) within 0.10 m of the
goal line. Debounce: one save per shot, re-armed when the ball is > 0.25 m out
or not seen for 1 s.

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
| `READY` | Sent once after boot. |
| `HIT <mag>` | Accelerometer spike above threshold; `mag` = peak deviation in milli-g. Rate-limited to one per 250 ms. |
| `A <mg>` | Optional low-rate (≤10 Hz) accel magnitude for debugging. Pi may ignore. |

**Uno watchdog:** no `M` for 200 ms -> motor #1 stops.

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
