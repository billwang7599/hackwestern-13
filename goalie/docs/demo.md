# Demo run sheet

Three minutes on stage, plus setup. Rehearse once with `--fake` (no hardware)
so the laptop side is known good.

## Before the judges arrive (10 min)

1. **Build tip from the sim:** use the **pivot paddle with a narrow 5 cm
   blade**. A paddle that never moves already saves ~50% (a 12 cm rail blade:
   87%), so a wide blade makes the robot look good without doing anything.
   If it's saving everything, narrow the blade or widen the goal.
2. Power order: motor battery **off** -> Uno USB into the Pi -> boot the Pi ->
   motor battery **on** last (so a booting Uno never sees the motor supply
   floating).
3. Start the server on the Pi (ssh from the laptop):
   ```sh
   cd goalie/pi && make deploy && make run          # or on the Pi:
   ./goalie_motor -b arduino -d /dev/serusb1 --rpm <measured> --delay-ms 50
   ```
   The log should show `[uno] READY` within ~2 s and
   `MODE QNX (SCHED_RR prio 40 ...)`. If it says `[RR not granted: running SCHED_OTHER]`,
   run it as root (or give the user real-time priority).
4. Laptop:
   ```sh
   cd goalie/vision
   export GOALIE_HOST=goalie.local          # or the Pi's IP
   .venv/bin/python calibrate.py            # click the 4 field corners, then the ball, then the paddle
   .venv/bin/python track.py                # the game window
   ```
   Re-run `calibrate.py` whenever the camera or the lighting moves. Check: the
   ball dot follows the real ball, the paddle marker follows the paddle, the
   link light is green, `wd` is off.
5. Bench check: roll one slow ball. The paddle should move to the predicted
   crossing (the marker on the goal line in the UI) before the ball arrives.
   Tap the goal frame: the server log shows `HIT`; the scoreboard doesn't change
   unless a shot was in flight.

## On stage

| Time | Say / do | What they see |
|---|---|---|
| 0:00 | "An AI goalkeeper. The webcam tracks the ball, a QNX real-time loop on the Pi predicts where it will cross and moves the paddle." | Scoreboard, green **QNX real-time** banner |
| 0:20 | Roll 3-4 casual shots. | Saves, needle swings, flag waves, Claude taunts (`t` toggles them) |
| 0:50 | Invite a judge to shoot. | More saves (or a goal: the taunt changes tone) |
| 1:20 | **Jitter demo:** press `m`. "Same code, same loop, but scheduled like a normal desktop OS under load." | Red **LINUX (emulated jitter)** banner, `jit_max` jumps from <1 ms to ~30 ms, loop rate drops from 200 Hz to ~100-130 Hz |
| 1:40 | Shoot fast ones in LINUX mode, then press `m` back to QNX. | Jittery paddle; back to smooth. Be honest: it's an **emulation** (normal priority + injected 0-30 ms stalls), not real Linux. In our no-hardware runs the save-rate gap was small (within a shot or two), so sell the jitter numbers, not a save-rate difference. |
| 2:00 | **Crash demo:** on the Pi, `kill -9 $(pidof goalie_motor)` (or `slay goalie_motor`). "What happens if the control software dies mid-game?" | Uno stops the motor within **200 ms** (its own watchdog: no `M` for 200 ms). The paddle drops to rest, it doesn't keep spinning. Restart the server: play resumes; the laptop reconnects by itself. |
| 2:30 | **Human vs AI:** press `h` (or just use the Switch controller). A judge steers the goalie with the Switch stick / `a` `d`. | `ctl=HUMAN` on the UI; same real-time loop, human target |
| 2:50 | Press `h` again for AUTO. "The robot is better than you." `r` resets the score. | |

Keys in `track.py`: `m` QNX/LINUX, `h` human goalie (`a`/`d` steer without a
controller), `r` reset score, `t` taunts on/off, `q` quit.

## Failure modes and fixes

| Symptom | Fix |
|---|---|
| Paddle swings past the target and back | `--delay-ms` too small or `--rpm` too low: raise them. Or `GAINS 2 0.1` live. |
| Paddle creeps / never quite arrives | Starting duty too low: raise `--db-comp` (e.g. 0.18). |
| `stopped=1` in STATUS, motor does nothing | The camera isn't seeing the paddle (`paddle_seen=0`): redo the paddle colour in `calibrate.py`. With no paddle feedback for 500 ms the Pi sends `S` on purpose. |
| `wd=1` | No frames from the laptop for 300 ms: check `GOALIE_HOST` and the network. |
| `uno_ready` > 1 or "Uno rebooted" in the log | Brown-out: motors on the Uno 5 V or a missing common ground. See `wiring.md`. Needle zero is lost: centre it by hand. |
| No `/dev/serusb1` | USB serial not enumerating on QNX: UART fallback in `wiring.md`, run with `-d /dev/ser1`. |
| Taunts silent | No `ANTHROPIC_API_KEY` uses canned lines; check the Mac's volume and `say` works. |

## No-hardware rehearsal

```sh
make -C goalie/pi host
goalie/pi/build/goalie_motor_host -b stub -p 5556 --rpm 100 --tau 0.045 --delay-ms 40 &
cd goalie/vision && GOALIE_HOST=127.0.0.1 .venv/bin/python track.py --fake
# or the scripted version, written to docs/integration_run.txt:
cd goalie && ./integration_run.sh
```
