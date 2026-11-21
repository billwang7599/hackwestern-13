# Self-balancing TARS: controller, simulator, tuning

TARS stands on two wheels under the middle of the slabs, which makes it an inverted pendulum. The controller keeps the wheels under the centre of mass. Push it and it rolls to catch itself, leans back, and returns home.

```
balance/                     portable controller (the code that runs on the robot)
  balance.h / balance.c      C99, no OS, no malloc, float only
  test_balance.c             unit tests            -> make -C balance test
  micropython/balance.py     same math for the Pico (also the sim's fallback)
sim/                         Python simulator
  run.py                     headless / --compare / --live
  tune.py                    gain search, --save -> gains.json
  check_parity.py            C library == MicroPython port?
  params.py                  robot + motor presets (edit this to match your build)
  plant.py simulate.py controller.py
```

## Quick start

```sh
make -C balance test                      # build libbalance + run unit tests
cd sim
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python run.py                   # headless: PASS/FAIL, max tilt, max push, out/nema17.png
.venv/bin/python run.py --compare         # which motors can balance this robot?
.venv/bin/python run.py --live            # window: ←/→ (or a/d) push it, [ ] push size, r reset
.venv/bin/python tune.py --motor nema17   # ~15 s gain search; add --save to keep the result
.venv/bin/python check_parity.py          # C vs MicroPython port
```

Other `run.py` flags: `--motor tt|dc25|pololu37d|nema17`, `--push 2:0.4` (time:impulse in N·s, repeatable), `--gains kp=80,kd=20`, `--params robot.json`, `--fused` (BNO085), `--slip`, `--no-outer`, `--backend c|py`, `--out file.png`.

## How the control works

```
                 ┌──────── outer, 50 Hz ───────┐       ┌───────── inner, 400 Hz ─────────┐
speed setpoint → │ PI on (speed_sp − speed)    │ → θ*  │ a = kp(θ − θ*) + ki∫ + kd·θ̇     │ → a → ∫ → wheel speed
                 └─────────────────────────────┘       └─────────────────────────────────┘      ± turn·track/2
                          ↑ speed (encoder, or our own                ↑ θ, θ̇ from IMU
                            command if we have none)
```

- **Estimator.** A complementary filter: `θ = α(θ + θ̇·dt) + (1−α)·atan2(−a_fwd, a_up)`, with `α = τ/(τ+dt)` and `τ = comp_tau`. The gyro handles fast motion and the accelerometer corrects slow drift. Accelerating the wheels shows up in `a_fwd` and fools the accelerometer, so `accel_comp` subtracts the controller's own last acceleration command first. Without it, the estimate lagged the true pitch by about 2° during manoeuvres. With a BNO085, set `use_fused_pitch = 1` and pass its angle in.
- **Gyro bias.** It's learned automatically while `enable = 0` and the robot sits still on the kickstand.
- **Inner loop (pitch to wheel acceleration).** If the robot leans forward, the wheels accelerate forward. D acts on the gyro rate directly (derivative on measurement), so it doesn't kick when the setpoint changes. The output acceleration is integrated into a wheel speed, which is exactly what a stepper wants.
- **Outer loop (speed to target lean).** To go faster it leans forward. If it's drifting forward it leans back, which, unintuitively, means first speeding up so it can tip back. The speed integral works as position hold, so after a shove it rolls home. It also finds the true balance angle when the CoM isn't exactly over the axle.
- **Turning.** Turn is added differentially: `left = v − turn·track/2`, `right = v + turn·track/2` (+turn = left).
- **Safety.** Anti-windup on both integrators, plus clamps on lean, acceleration and speed. Past `tilt_cutoff` (40°) the state goes to FALLEN and the motors switch off. To re-arm, hold it within ±3° for 0.5 s. States: `DISABLED → WAIT_UPRIGHT → RUNNING ⇄ FALLEN`.

Default gains (`balance_default_cfg`) are tuned on the default robot in `sim/params.py`: 1.3 kg, CoM 25 cm above the axle, 45 cm slab, 90 mm wheels, NEMA17.

| | value | units |
|---|---|---|
| kp | 84 | m/s² per rad |
| ki | 0 | |
| kd | 22 | m/s² per rad/s |
| speed_kp | 0.26 | rad per m/s |
| speed_ki | 0.02 | rad per m |

With NEMA17s, the biggest push it can recover from is set by `max_speed`, not by the gains. The limit is 0.70 N·s at 1.0 m/s, 1.0 N·s at 1.5 m/s (the default), and 1.22 N·s at 2.0 m/s. Above that, the stepper's pull-out torque runs out and it loses steps. Changing the control rate from 400 Hz to 100 Hz made no difference in the sim.

The tuner can score slightly better with very high gains (kp of 250 or more). I checked that against a pessimistic robot (10 ms latency, 5× gyro noise), and the moderate set held up better, so it's the default.

## Do our motors work? (`run.py --compare`)

The scenario: hold the robot upright, arm it, then push it with ±0.25 N·s at the top of the slab (a firm shove). "Max push" is the largest impulse it recovers from in **both** directions. Each motor uses its own tuned gains (`gains.json`).

| motor | result | max push | verdict |
|---|---|---|---|
| **NEMA17 + TMC2209** (default) | PASS | **1.00 N·s** | balances, takes big shoves |
| Pololu 37D 30:1 + encoder | PASS | 0.83 N·s | balances |
| 25 mm metal gearmotor, 6 V 200 RPM, no encoder (`dc25`) | PASS | 0.41 N·s | balances, smaller margin |
| **TT yellow gearmotor, 6 V 200 RPM, no encoder (`tt`)** | FAIL | **0.08 N·s** | barely stands, a light tap knocks it over |
| TT with no backlash/deadband (re-tuned) | FAIL | 0.14 N·s | still too weak |
| TT with 3× torque (re-tuned) | PASS | 0.45 N·s | torque is the problem |
| TT with 65 mm wheels (re-tuned) | FAIL | 0.18 N·s | still marginal |

**Verdict: the TT motors can't make the push demo work, and the limit is torque.** About 0.08 N·m stall per motor isn't enough, and backlash makes it worse. Order 2× NEMA17 with TMC2209 drivers, which is what the controller is tuned for. Second choice is encoder gearmotors. If you can only get 25 mm metal gearmotors, they work in the sim but with half the margin. Re-run `run.py --compare --ablate` after changing `params.py`.

How the DC model works (`plant.py`): torque falls linearly with speed (torque-speed line), and the rotor inertia is reflected through the gearbox. The gearbox has backlash, modelled as a gap plus a compliant contact. There's Coulomb/static friction, a PWM deadband with firmware compensation, and optionally no encoder. With no encoder, the outer loop runs on the controller's own commanded speed (`wheel_speed = NAN`). That's a guess about the real speed: the controller assumes duty is proportional to speed. Use `--no-outer` to turn the speed loop off; the robot still balances but drifts metres away.

## Tuning the real robot

Do this with the kickstand within reach and someone's hand around the top.

1. **Check signs first, with the motors off.** Lean the robot forward: `pitch` must go **positive** and `gyro_pitch` must be positive while it's moving forward. At rest tilted forward, `acc_fwd` must be **negative**. If any of these are wrong, swap or negate the axes in your IMU read function. Then enable it and give it a nudge forward: the wheels must drive **forward**. If they go backward, invert the motors, not the gains.
2. **Find the balance angle.** Hold it where it balances by hand and read `pitch`. Put that value in `pitch_offset`, or let the outer integral find it.
3. **Turn off the outer loop** (`speed_kp = speed_ki = 0`) and set `ki = 0`, `kd = 0`.
4. **Raise `kp`** from about 20 until it stands but oscillates (rocks back and forth). Real hardware usually lands within ±50% of the sim's value.
5. **Add `kd`** until the oscillation dies. Too much shows up as buzzing or jitter. Then raise `kp` about 20% more and repeat.
6. **`ki` (inner):** usually leave it at 0, or use a small value (≤ 5) only if it leans steadily.
7. **Outer loop:** `speed_kp` about 0.1 to 0.3. Too low and it wanders; too high and it surges back and forth slowly. Then add a small `speed_ki` (0.01 to 0.05) for position hold. If it makes the robot rock slowly, reduce it.
8. **Push test.** Push a bit harder each time. When it falls, check whether it hit `max_accel` or `max_speed` (motor limit) or whether the estimate lagged (raise `comp_tau` a little).

Live tuning over the laptop link uses these commands. They're a **proposal**; the server doesn't implement them yet.

| command | meaning |
|---|---|
| `T <kp> <ki> <kd>` | inner gains, calls `balance_set_inner_gains()` |
| `TO <speed_kp> <speed_ki>` | outer gains |
| `B <speed m/s> <turn rad/s>` | balance-mode setpoint; replaces `D` while balancing (watchdog still applies) |
| `BE 0/1` | enable or disable balancing (kickstand mode = 0) |
| `BS` | status: `OK <state> <pitch_deg> <target_deg> <v>`, good for live graphs |

## What the sim can't tell you

- **Backlash and friction.** It models them with simple, made-up numbers. Real gearboxes are worse and vary.
- **Mounting.** The IMU isn't perfectly square to the slab and the CoM isn't over the axle. The sim includes 0.5° and 4 mm of each, but your build will differ. Calibrate (tuning step 2).
- **Flex and vibration.** Slabs bend, and steppers vibrate the IMU. Mount the IMU firmly near the axle line, and use the MPU6050's digital low-pass filter (DLPF at about 42 Hz).
- **Stepper reality.** Missed steps are only flagged (the "stall" ms count) and not modelled after they happen. Torque also depends on supply voltage, current setting and microstepping. Use 12 V and set the TMC2209 current properly.
- **Timing on QNX and the Pi 5 RP1.** The sim uses 0.2 ms jitter and 2 ms latency. Actual I2C/GPIO latency on QNX isn't known yet; measure it. Robust to about 10 ms of latency in the sim, but margins shrink.
- **Planar model only.** There's no yaw dynamics, no uneven floor, and no wheel bounce. Traction limits apply only with `--slip`, which is approximate and stepper-only.

## Wiring the controller in

**Pico first (recommended).** Copy `balance/micropython/balance.py` to the Pico and run a 400 Hz loop:
1. Read the MPU6050 over I2C (e.g. GP20/GP21) and convert to rad/s and m/s².
2. Call `balance.step(...)`.
3. Convert the m/s output to step rates: `steps/s = v / (2π·r) · 200 · microsteps`. Generate them with PIO or PWM-frequency STEP pins (e.g. STEP/DIR on GP10–13, EN on GP14), not by bit-banging in Python.
4. Take `B`/`BE`/`T` setpoints from the Pi over the existing UART.

For DC motors, replace the existing `pico/main.py` deadband (which *ignores* small commands) with deadband *compensation*: add about 0.07–0.10 to |duty|. Otherwise the robot can't make small corrections.

If MicroPython can't keep 400 Hz, drop to 200 Hz. The sim passes unchanged at 200 Hz and even at 100 Hz (`RobotParams.control_hz`). Steady timing matters more than a high rate.

**QNX port (the pitch for judges).** Add `../balance/balance.c` to `SRCS` (and `-I../balance` to `CFLAGS`) in `pi/Makefile`. In `pi/src/main.c`, run a second `SCHED_FIFO` thread at 400–500 Hz using `clock_nanosleep(TIMER_ABSTIME)`. Each tick it should:
1. Read the IMU (I2C on the Pi 5; QNX RP1 support is new, so test early).
2. Call `balance_step()`.
3. Send `left`/`right` to the Pico as step rates or duty.

Keep the existing 500 ms command watchdog, but have it zero the *setpoint*, not cut the motors. Cutting power mid-balance means it falls. Log `dt` jitter and show it live: that's the hard real-time story.
