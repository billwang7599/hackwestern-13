# goalie/vision: laptop side

The webcam tracks the ball and paddle. Their positions go to the Pi as `F` frames over TCP :5556 (see
[../docs/architecture.md](../docs/architecture.md)). The laptop also draws the scoreboard and handles the
Switch controller and the Claude taunts.

```sh
cd goalie/vision
python3.14 -m venv .venv          # 3.11-3.14; pygame-ce has no 3.15 wheel yet
.venv/bin/pip install -r requirements.txt
```

## No hardware at all

```sh
.venv/bin/python mock_pi.py                                  # stand-in Pi server on :5556
.venv/bin/python track.py --fake --host 127.0.0.1            # window: synthetic shots, live scoreboard
.venv/bin/python fake_ball.py --host 127.0.0.1 --shots 20 --speed 1.0   # headless, prints results
.venv/bin/python -m pytest -q test_vision.py                 # full headless test suite
```

To test against the real C server, swap `mock_pi.py` for `goalie/pi`'s `build/goalie_motor_host -b stub -p 5556`.

## Real camera

1. Clamp the C270 60-80 cm above the field, with the goal at the **bottom** of the image. Lock the
   exposure if you can, and use even lighting with no hard shadows.
2. Put the coloured tape on the paddle **blade tip only**. Its centre is the x that the Pi controls.
3. `.venv/bin/python calibrate.py --field-w <measured wall-to-wall m>`:
   - Click the corners in order: far-left, far-right, goal-right, goal-left. The last two must be on
     the goal line.
   - Press `b` and click the ball. Press `p` and click the paddle.
   - Use the sliders to tune the masks, then press `s` to save `calib.json`.
   - macOS asks for camera permission for your terminal the first time.
4. `.venv/bin/python track.py --host goalie.local` (or the Pi's IP).

| Key | Action |
|---|---|
| `m` | Switch between QNX and LINUX (emulated jitter) mode |
| `h` | Toggle human goalie mode. Steer with the Switch left stick, or `a`/`d` without a controller |
| `r` | Reset the score |
| `t` | Taunts on/off |
| `q` | Quit |

Taunts use `ANTHROPIC_API_KEY` if it is set. Without it you get canned lines. They are spoken with `say`.

Find your controller's button and axis numbers with `.venv/bin/python switch_input.py`.

`mock_pi.py` also accepts `SIMHIT <mag>`, which fakes an accelerometer hit.
