# The 5-minute RPM test

The sim and the Pi's controller both need one number: the goalie motor's
**no-load output RPM at the voltage you'll actually use**. Everything else
has sensible defaults.

## Measure it

1. Put a small tape flag on the output shaft (fold a strip of masking tape
   around it so a tab sticks out).
2. Power the motor **from the battery pack you'll use in the demo**.
   Quickest: wire it straight to the pack. Better: through the driver at full
   duty, because H-bridges like the L9110S drop ~1 V and the paddle will see
   that lower speed. With the sketch loaded on the Uno (it stops the motor if
   no `M` arrives for 200 ms, so keep resending):

   ```sh
   pip install pyserial
   python3 -c "import serial,time; s=serial.Serial('/dev/cu.usbmodemXXXX',115200); time.sleep(2); t=time.time()
   while time.time()-t<12: s.write(b'M 1\n'); time.sleep(0.05)
   s.write(b'S\n')"
   ```
3. Count the flag's turns for **10 seconds** (film it with your phone in slow
   motion if it's too fast to count; most phones do 240 fps).
4. `RPM = turns × 6`. Do it twice, in both directions, and take the lower one.

Optional, 2 more minutes: find the **starting duty** (the static-friction
deadband). Send `M 0.05`, `M 0.10`, ... and note the smallest duty that makes
the paddle move reliably, with the paddle mounted.

Sanity check: palm-sized plastic gearmotors (TT style) are usually 100-250 RPM
at 5-6 V. Under 30 RPM the pivot goalie still stops casual rolls, but not
corner shots at 1.5 m/s (see `sim/README.md`).

## Feed it to the sim

```sh
cd goalie/sim
.venv/bin/python run.py --rpm X                      # pass/fail at 1.0 m/s, out/scenario.png
.venv/bin/python run.py --rpm X --feasibility        # fastest shot it can stop (~1 min)
.venv/bin/python tune.py --rpm X                     # re-pick gains for this motor (prints best first)
```

`run.py` prints the motor time constant it assumes (`tau NN ms`). If you also
measured the starting duty, add `--deadband D`. If your camera latency is
known (vision UI shows it), add `--latency-ms L`.

## Feed it to the Pi

Put the same numbers on the `goalie_motor` command line (on the Pi, or
`make run` with an edited Makefile):

```sh
goalie_motor -b arduino -d /dev/serusb1 --rpm X --tau 0.0NN --delay-ms 50
# with tune.py's winners (kp, kd, obs_l1, obs_l2):
goalie_motor ... --rpm X --kp 3 --kd 0.1 --l1 0.5 --l2 2
# measured starting duty D -> deadband compensation:
goalie_motor ... --db-comp D
```

| Sim name | `goalie_motor` flag | Default |
|---|---|---|
| `motor.rpm` / `control.model_rpm` | `--rpm` | 150 |
| `control.model_tau` (printed `tau`) | `--tau` (seconds) | 0.04 |
| `control.latency_est` | `--delay-ms` | 50 |
| `control.kp`, `control.kd` | `--kp`, `--kd` | 3.0, 0.10 |
| `control.obs_l1`, `obs_l2` | `--l1`, `--l2` | 0.5, 2.0 |
| `control.db_comp` | `--db-comp` | 0.12 |
| `control.min_duty` | `--min-duty` | 0.06 |
| `control.model_uoff` | `--uoff` | 0.10 |
| `goalie.paddle_len`, `geometry` | `--paddle-len`, `--geom` | 0.12, pivot |

Live tuning without a restart: send `GAINS <kp> <kd>` over TCP (e.g.
`nc goalie.local 5556`), then `STATUS` to read back `kp=` / `kd=`.

**Set the static-friction boost only on the Pi** (`--db-comp`). Leave the
Uno's `MOTOR_A_MIN_DUTY` at 0 so they don't add up.
