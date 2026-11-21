# AGENTS.md: rules for coding agents (and humans) in this repo

Read [PLAN.md](PLAN.md) first. It has the architecture, the CPU budget and the current status.

## Ground rules

- **QNX native is the selling point.** Everything that runs on the robot is C/C++ built with `qcc` for QNX 8.0 (aarch64le): `tars_motor`, voice, the intent classifier, go-to and the detector. Python is only for laptop tools, model export/training and Mac test harnesses. No Python fallbacks on the robot: if the native path breaks, fix it.
- **Hackathon, 30 h.** Prefer the smallest change that works on the real robot. No refactors for their own sake.
- **Fully local at runtime.** Nothing on the robot may need the laptop or the internet once it's booted. The laptop is only for building, deploying and WASD teleop.
- **Pi decides, Uno moves.** Pi code never drives motor pins. The Uno never decides where to go.
- **All motion goes through `tars_motor`** (TCP 5555). Vision, voice and YOLO are each a client. Don't open the Uno serial port from anything else.
- **Stop always wins.** `S` from any client, ✋, the spoken word "stop", tip-over and watchdog timeouts must stop the wheels. No learned pose, classifier or mode may block a stop.
- **Fail safe.** If a client crashes or goes quiet, `tars_motor`'s watchdog stops the wheels. Keep it that way.
- **Don't touch `goalie/` or `balancer/`.** They're parked experiments.
- **Branches:** work on your own branch/worktree. Commit often, and **don't push unless the lead says so**. The base branch is `main`.
- **Be honest about testing.** Say exactly what ran (Mac, simulator, stub server) and what has **not** run on the Pi/QNX/Uno.

## Layout

| Path | What | Language |
|---|---|---|
| `uno/tars/tars.ino` | Uno firmware: wheels, LED, buzzer, button, tip/bump | Arduino C++ |
| `uno/test/` | host-side firmware tests with mocks (`make -C uno/test`) | C++ |
| `pi/` | `tars_motor` QNX server + `test_server.py` (`make -C pi host && make -C pi test`) | C / Python |
| `vision/` | MediaPipe gestures, follow mode, teach-a-gesture (`GESTURES.md`) | Python (+ partial C++) |
| `voice/` | speech-to-text (whisper.cpp) + intent classifier | C/C++ (+ Python training) |
| `laptop/` | `tars_client.py`, WASD `teleop.py`, `wheel_cal.py` (dev only) | Python |
| `docker/` | QNX SDP 8.0 cross-compile container (`docker/run.sh`) | |
| `docs/PI_TEST.md` | hardware bring-up checklist for the Pi teammate | |

## Protocols (keep these in sync, and update the docs if you change them)

**Clients → `tars_motor` (TCP 5555, one line per command):**
`D <l> <r>` drive fraction −1..1 (must be repeated, watchdog) · `S` stop · `MOVE <mm>` · `TURN <deg>` (+ = right)
· `LED <teach|rec|ok|no|forget|off>` · `BEEP <1-4>` · `STATUS` (includes `btn_teach=` `btn_forget=` counters) · `PING`
· `WHO <name>` (label the client) · `MODE <gesture|follow|toggle|teach>` (request for the vision app).
Unsupported → `ERR`, with no change to motor state.
Several clients at once: `S` from any client stops everything and holds other clients' non-zero `D` for 1.5 s (`ERR held`); a disconnect only stops a drive or move that client started.

**voice → go-to controller (TCP 5557, one line each, reply optional):** `GOTO <canonical object>` (e.g. `GOTO trash can`) · `CANCEL` (on every stop, and before any manual `MOVE`/`TURN`).

**`tars_motor` → Uno (USB serial 115200):** `D l r`, `M <mm>`, `R <deg>`, `S`, `K diam track`, `L <0-3>`, `B <n>`.
**Uno → `tars_motor`:** `READY`, `DONE M|R <n>`, `ABORT …`, `T <deg>` tipped, `B <mg>` bump, `BTN TEACH`, `BTN FORGET`.
The full spec is in the header comment of `uno/tars/tars.ino`.

## Before you commit

Run the suites you touched, and preferably all of them:
```sh
make -C uno/test
make -C pi host && make -C pi test
python3 laptop/test_teleop.py
# plus the tests in vision/tests and voice/ if you changed those
```
Don't commit models (`*.task`, `*.bin`, `*.onnx`), `build/`, `.venv/` or `__pycache__/`. Use a `get_model.sh` script instead.

## Pi / QNX notes

- Cross-compile with `qcc` via `docker/run.sh` (needs `~/qnx800`). QNX 8.0 has Python and `apk`.
- Real-time loops use `SCHED_RR`. Keep heavy ML (YOLO, MediaPipe, whisper) at normal priority so they can't starve `tars_motor`.
- Speech uses `threads=2` so cores are left for vision.
