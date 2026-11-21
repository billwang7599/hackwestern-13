# goalie/mv: single-webcam 3D ball tracking and goal-crossing prediction

From one webcam behind the goal: detect the ball, estimate its 3D position from
**where it is in the image (direction)** and **how big it looks (distance)**,
fit a **ballistic trajectory** (gravity known, drag optional), then predict
**where (x, height) and when** it crosses the goal plane.

```
frame ─► detect.py ─► geometry.py ─► tracker.py ─► trajectory.py ─► overlay.py / P lines
         HSV + static   pinhole:       track mgmt,    weighted LSQ
         rejection,     size→depth,    labels         ballistic fit,
         blur-robust    3D + covariance               KF, crossing
         sub-px size
```

## World frame

Origin at the goal centre **on the floor**. `X` = across the goal (+ = right as
seen from the camera/goalie looking out at the shooter). `Y` = height (+ up).
`Z` = distance out from the goal plane (+ toward the shooter). An incoming ball
has `vz < 0` and crosses at `Z = 0`. Heights are of the ball **centre**.

This matches `docs/architecture.md`: MV `X` = contract `x`, MV `Z` = contract
`y`, and MV `Y` (height) is new.

## Setup

```sh
cd archive/goalie/mv
python3.14 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python tools/probe_cam.py      # which index is the Logitech?
```

On this Mac **device 0 = Logitech C270, device 1 = MacBook camera**. The index
can change when devices are re-plugged. macOS asks for Camera permission for
your terminal app on first use. If frames are black or the camera does not open,
allow it under System Settings › Privacy & Security › Camera.

## Run it in stages, on the real webcam

```sh
.venv/bin/python calib.py ball tennis              # or pingpong / soccer / --diam 0.07
.venv/bin/python calib.py hsv --device 0           # click the ball, tweak sliders, press s
# Stage 1: detection only. Circle + (u,v), diameter px, conf/circularity, HSV mask inset
.venv/bin/python live.py --device 0 --stage detect
# Stage 2: per-ball SPEED / HEIGHT / DIST / DIR (az, el) label + velocity arrow
.venv/bin/python live.py --device 0 --stage track
# Stage 3 (default): ballistic fit + crossing point on the goal-face view
.venv/bin/python live.py --device 0
```

Keys: `c` clear track, `s` save the last ~4 s as `out/clip_*.mp4`, `m` toggle
mask inset, `q` quit.  Options: `--record` (raw video to `out/rec_*.mp4`),
`--video clip.mp4` (replay, same labels), `--rolling` (ball on the floor),
`--bg` (MOG2 motion mask), `--emit stdout|udp:HOST:PORT`,
`--headless N` (no window, saves `out/live_last.png`).

Labels show `--` until there is enough data. Speed and direction need ≥ 3
detections; a crossing prediction needs ≥ 5.

* **SPEED** is |v| in m/s and km/h.
* **HEIGHT** is the ball-centre Y.
* **DIST** is Z from the goal plane.
* **DIR az** is the left/right angle of travel relative to straight at the goal
  plane (0 = perpendicular, + = right).
* **DIR el** is up/down (+ = rising).

All of these come from a ballistic fit over the last 6 detections, which smooths
without lag. Speed and az are also median-filtered over 5 frames.

## Calibration (in order of value)

1. `calib.py ball <name>`: the ball diameter **is** the depth scale. A 5% error
   in size is a 5% error in distance.
2. `calib.py hsv --device 0`: colour range. The default orange range also
   matches skin.
3. Intrinsics:
   * `calib.py intrinsics --device 0 --board 9x6 --square 0.024`. Print a 9x6
     inner-corner chessboard, measure a square, and grab about 15 views tilted
     into the corners.
   * Or `calib.py nominal --hfov 55`, an FOV guess (C270 ≈ 55–60°).
4. Pose:
   * `calib.py pose --height 0.35 --back 0.25 --pitch 8`. Tape-measure the
     camera height and its distance behind the goal line, and estimate the
     downward tilt.
   * Or `calib.py pose-board --device 0 --origin X Z` with the chessboard flat
     on the floor. Its first inner corner sits at world (X, 0, Z), with rows
     running across the goal.
5. `calib.py ballcheck --device 0 --distance 1.00`: hold the ball on the
   optical axis at a measured distance. It prints the estimated distance and a
   suggested effective diameter if it is off.

Settings live in `camera.json` (gitignored). `calib.py show` prints them.

## Synthetic accuracy (no hardware)

```sh
MPLBACKEND=Agg .venv/bin/python synth.py --demo --ball tennis --dist 3 --speed 6 --stage track
MPLBACKEND=Agg .venv/bin/python synth.py --sweep            # -> out/sweep.md/.csv/.png
MPLBACKEND=Agg .venv/bin/python synth.py --raw-clip && .venv/bin/python live.py --video out/synth_raw.mp4
.venv/bin/python -m pytest -q tests
```

The renderer projects the sphere exactly (pixel rays inside the sphere's cone,
2×2 supersampled), averages sub-frames over the exposure (motion blur), and
adds optical blur, sensor noise and optional clutter. Ground truth includes
**quadratic air drag** and floor bounces. Demo frames show the live labels plus
grey `GT` labels and a grey `+` for the true crossing.

### What the sweep says (full tables: [RESULTS.md](RESULTS.md))

Metric: median 2D (X, height) error of the crossing prediction available
**150 ms before** the ball reaches the goal, over 24 random shots per cell.
Camera: 640x480, HFOV 55°, 0.35 m up and 0.25 m behind the goal, exposure
equal to the frame time (worst case blur, which is the C270's indoor default).

* **Use a tennis ball (or bigger), not a ping-pong ball.** Tennis at 30 fps,
  2–4 m, 5–10 m/s gives a median of 0.8–1.9 cm and 79–100 % of shots within
  3 cm. The one exception is slow 4 m shots (5 m/s lobs that leave the frame):
  6.8 cm. Ping-pong medians look similar, but at 30 fps the tails are bad: p90
  is 90–170 cm in several cells and one cell has a 25 cm median, from the tiny
  blob and heavy drag. Soccer-size is best (0.3–1.1 cm, except slow 4 m shots:
  3.6 cm).
* **60 fps roughly halves the error** (tennis, 4 m, 5 m/s: 6.8 → 1.4 cm). The
  C270 only does 30 fps.
* **Exposure / motion blur is the biggest lever.** With a 4 ms exposure:
  tennis, 3 m, 5 m/s, 30 fps goes from 2.4 to 0.4 cm. macOS/AVFoundation
  ignores exposure requests, so use bright light (it shortens auto exposure)
  or a UVC tool (e.g. `uvc-util`) to set manual exposure on the C270.
* **Calibrate the ball size.** If the pipeline assumes a ball 5 % smaller
  than it is, the median roughly doubles and timing drifts by up to 66 ms on
  slow shots (`calib.py ballcheck`).
* **Distance window:** 2–4 m works. From 1 m, shots faster than about 5 m/s
  are over before 150 ms + 5 frames of data. Slow lobs from ≥ 2 m leave the
  frame. The fit keeps predicting through the gap, but accuracy drops.
* **Rolling ball:** 0.0–0.4 cm. A floor ray is exact, so size doesn't matter.
* The **weighted ballistic fit beats the constant-acceleration Kalman filter**
  in almost every cell (KF is 1.5–5x worse), so it is the one used for
  prediction. The KF is kept for comparison.
* **Motion-blur size:** moments vs fitEllipse minor axis give similar medians;
  moments put more shots within 3 cm (70 % vs 55 %), so moments is the default.
* **Static clutter:** same-colour static objects cost nothing with static
  rejection on; MOG2 adds nothing on top.
* **Labels vs ground truth:** speed-label error is typically 1–7 %.

These are synthetic numbers with perfect intrinsics, uniform light and no lens
distortion or rolling shutter. Treat them as a lower bound on real error and
confirm with `--record` clips of real shots.

## Output hook / proposed contract extension

`live.py --emit udp:goalie.local:5557` (or `stdout`) sends one line per frame
with a valid prediction:

```
P <t_ms> <x_m> <height_m> <t_to_cross_ms> <conf 0..1>
```

Proposal for `docs/architecture.md`, not yet applied. Add one optional command
on the TCP :5556 link: `PRED <t_ms> <x> <h> <t_to_cross_ms> <conf>`, with reply
`OK <target_x> <duty>`. When a PRED with `conf ≥ 0.3` has arrived in the last
100 ms, the Pi uses its `x` as the target instead of its own 2D predictor. `F`
keeps carrying the paddle position and feeding the watchdog, and nothing
existing changes. `h` is informational for a 1-DOF goalie (or a "too high,
don't bother" flag). If the Pi should do the fitting itself, use a 3D frame
instead: `F3 <t_ms> <seen> <x> <y> <z> <sx> <sy> <sz> <paddle_seen> <px>`. The
per-axis sigmas are required, because depth is far noisier than x.

## Files

| file | what |
|---|---|
| `config.py` | Camera (intrinsics + pose), ball, detector settings; `camera.json` I/O |
| `detect.py` | HSV mask, static-clutter rejection, optional MOG2, blob scoring/gating, blur-robust size |
| `geometry.py` | exact off-axis sphere back-projection, uncertainty, ground-plane rays (rolling) |
| `trajectory.py` | weighted ballistic LSQ (+ linear drag, rolling mode, bounce), crossing + 1σ, 9-state KF |
| `tracker.py` | per-frame glue, track reset/gating, smoothed labels |
| `overlay.py` | drawing for all stages (shared by live, replay, synth) |
| `live.py` | webcam app (capture thread, record/replay, emit) |
| `calib.py` | nominal/chessboard intrinsics, pose (simple/board), HSV picker, ball check |
| `synth.py` | renderer, truth with drag, sweep → `out/sweep*.md/csv/png` |
| `tools/` | `probe_cam.py`, `exposure_test.py`, `debug_shot.py` |
