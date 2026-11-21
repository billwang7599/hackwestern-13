# goalie/sim: is the motor fast enough?

```
cd goalie/sim
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python run.py                           # pass/fail: 60 casual 1.0 m/s rolls, exit 0 = pass
.venv/bin/python run.py --feasibility --sweep-rpm # the full report + out/feasibility.png (~1 min)
.venv/bin/python run.py --live                    # window: space shoot, arrows aim/speed, a auto
.venv/bin/python run.py --rpm 90                  # after the RPM test (docs/rpm_test.md)
.venv/bin/python tune.py --rpm 90                 # re-pick gains for your motor
```

Headless: `MPLBACKEND=Agg` (then `--live` saves `out/live_frame.png`).

| file | what |
|---|---|
| `params.py` | every physical / camera / controller number, with defaults |
| `plant.py` | ball roll + DC gearmotor (deadband, friction, backlash) + paddle geometry and block test |
| `predict.py` | crossing-point predictor. **Port to `pi/src/predict.c`**: algorithm spec in the docstring |
| `controller.py` | model-based observer + PD. **Port to the `pi/src/main.c` loop**: spec in the docstring |
| `simulate.py` | camera (fps, latency, noise, drops) + 200 Hz Pi loop + 1 kHz physics |
| `run.py` | scenario / feasibility / live |
| `tune.py` | gain grid search |

## Result (defaults: pivot paddle 12 cm x 5 cm, 22 cm goal, 30 fps, 50 ms)

The motor isn't the bottleneck. A pivot paddle on anything above ~30 RPM stops
every casual roll (0.8-1.5 m/s, corner shots included). At 150 RPM it holds
up to ~3.5 m/s, where the 30 fps camera becomes the limit. Two caveats:

- A paddle that **never moves** already saves 50% (the 12 cm rail blade: 87%).
  For a fun demo keep the paddle narrow or the goal wide (`--goal-w`, `--paddle-w`).
- The camera only sees the paddle 30-60 ms late, so the Pi has to run a small
  motor model and correct it from the camera (controller.py). A PD loop
  straight on camera data overshoots by a whole goal width.

Run `run.py --feasibility --sweep-rpm` for the current numbers.
