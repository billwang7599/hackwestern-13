# AGENTS.md: rules for coding agents (and humans) in this repo

Read [PLAN.md](PLAN.md) first. It has the architecture, the CPU budget and the current status.
The sensor-fusion design, state machine and port map are in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Ground rules

- **QNX native is the selling point.** Everything that runs on the robot is C/C++ built with `qcc` for QNX 8.0 (aarch64le): `tars_motor`, voice, the intent classifier, go-to and the detector. Python is only for laptop tools, model export/training and Mac test harnesses. No Python fallbacks on the robot: if the native path breaks, fix it.
- **Hackathon, 30 h.** Prefer the smallest change that works on the real robot. No refactors for their own sake.
- **Fully local at runtime.** Nothing on the robot may need the laptop or the internet once it's booted. The laptop is only for building, deploying, WASD teleop and the optional mission control (`iter2-autonomy/mission/`: shows what the Pi computed, sends commands, does no robot processing).
- **Pi decides, Uno moves.** Pi code never drives motor pins. The Uno never decides where to go.
- **All motion goes through `tars_motor`** (TCP 5555). Vision, voice and YOLO are each a client. Don't open the Uno serial port from anything else.
- **Stop always wins.** `S` from any client, ✋, the spoken word "stop", tip-over and watchdog timeouts must stop the wheels. No learned pose, classifier or mode may block a stop.
- **Fail safe.** If a client crashes or goes quiet, `tars_motor`'s watchdog stops the wheels. Keep it that way.
- **Don't touch `archive/goalie/` or `balancer/`.** They're parked experiments.
- **Branches:** work on your own branch/worktree. Commit often, and **don't push unless the lead says so**. The base branch is `main`.
- **Be honest about testing.** Say exactly what ran (Mac, simulator, stub server) and what has **not** run on the Pi/QNX/Uno.

## Layout

One folder per iteration (README "Iterations"; old → new paths in `docs/MOVED.md`). Iterations 2 and 3 build on iteration 1's `tars_motor`.

| Path | What | Language |
|---|---|---|
| **`iter1-wasd/`** | **Iteration 1: WASD driving** | |
| `iter1-wasd/uno/tars/tars.ino` | Uno firmware: wheels, LED, buzzer, button, tip/bump. Default build = **Kuman SM11 kit** (`CAR_SM11`: DC wheels, timed MOVE/TURN, speed cap); steppers = `-DWHEEL_TYPE=2` | Arduino C++ |
| `iter1-wasd/uno/test/` | host-side firmware tests with mocks (`make -C iter1-wasd/uno/test`) | C++ |
| `iter1-wasd/pi/` | `tars_motor` QNX server + `test_server.py` (`make -C iter1-wasd/pi host && make -C iter1-wasd/pi test`), `MOTOR_GUARD.md` | C / Python |
| `iter1-wasd/laptop/` | `tars_client.py`, WASD `teleop.py`, `wheel_cal.py` (dev only) | Python |
| **`iter2-autonomy/`** | **Iteration 2: voice + YOLO + iPhone LiDAR autonomy** (`docs/ARCHITECTURE.md`) | |
| `iter2-autonomy/voice/` | speech-to-text (whisper.cpp) + intent classifier | C/C++ (+ Python training) |
| `iter2-autonomy/goto/` | "go to the trash can": `goto` controller (C) + `detect` YOLO via ONNX Runtime (C++), `nav.c` A*, object memory, `objects.txt` aliases, `DETECTIONS.md` (`make -C iter2-autonomy/goto host && make -C iter2-autonomy/goto test`) | C / C++ (+ Python export & sim) |
| `iter2-autonomy/iphone/` | MotionUSB iOS app: ARKit pose, LiDAR, speech link (5573) | Swift |
| `iter2-autonomy/mission/` | laptop mission control: `mission_relay` (Pi, one Wi-Fi port 5575) + `laptop/mission.py` browser UI (`make -C iter2-autonomy/mission test`) | C / Python (laptop) |
| `iter2-autonomy/edge-ai/`, `dashboard/`, `iphone-usb/` | Andrew's stack: QNX edge AI + `lidar_guard`, Pi dashboard / `service.py`, iPhone-over-USB for QNX | C++ / Python / sh |
| **`iter3-pet/`** | **Iteration 3: MediaPipe pet** | |
| `iter3-pet/vision/` | MediaPipe gestures, follow mode, teach-a-gesture, pet mode (`GESTURES.md`) | Python + C++ ports (parity-tested) |
| `archive/` | parked: `goalie/`, `pico/` (legacy firmware; the `-b pico` backend still lives in `tars_motor`), `JOYCON.md` | |
| `hardware/` | 3D-printable stepper wheel | |
| `docker/` | QNX SDP 8.0 cross-compile container (`docker/run.sh`) | |
| `docs/PI_TEST.md` | hardware bring-up checklist for the Pi teammate | |

## Protocols (keep these in sync, and update the docs if you change them)

**Clients → `tars_motor` (TCP 5555, one line per command):**
`D <l> <r>` drive fraction −1..1 (must be repeated, watchdog) · `S` stop · `MOVE <mm>` · `TURN <deg>` (+ = right; exact on steppers, **timed open loop on the DC car**: the iPhone pose is the truth)
· `WHEELCAL [diam track]` (steppers) · `SPEEDCAL [mm_s deg_s [deg_s_slow]]` (DC: speeds at D 1 and the slow-turn rate → Uno `V`; no args → `OK dc <mm_s> <deg_s> [<deg_s_slow>]` / `OK stepper`). `STATUS` has `wheels=stepper|dc dc_mm_s= dc_deg_s= dc_deg_s_slow=` (0 = the Uno didn't report one). DC: `TURN` with speed < 0.99 is the Uno's **slow turn** (own duty `DC_SLOW_TURN_DUTY`, timed at `deg_s_slow`), so goto's 0.40 turns and the guard_blocked pivot clamp are slow on the SM11 too; `MOVE` speed is still ignored.
· `LED <teach|rec|ok|no|forget|off>` · `BEEP <1-4>` · `STATUS` (includes `btn_teach=` `btn_forget=` counters) · `PING` · `STATUS` also has `stops=` (S received from anyone)
· `WHO <name>` (label the client) · `MODE <gesture|follow|toggle|teach>` (request for the vision app)
· `POSE` → `OK <x_mm> <y_mm> <hdg_deg> <epoch> <moving>` · `POSE RESET`. `STATUS` also has `pose=x,y,hdg pose_epoch=`. The pose is dead-reckoned from the motion that ran (D output, MOVE/TURN DONE/ABORT amounts): x forward, y right, hdg + = clockwise. The epoch changes on restart or `POSE RESET`.
Unsupported → `ERR`, with no change to motor state.
Several clients at once: `S` from any client stops everything and holds other clients' non-zero `D` for 1.5 s (`ERR held`); a disconnect only stops a drive or move that client started.

**voice → go-to controller (TCP 5570, one line each, reply optional):** `GOTO <canonical object>` (e.g. `GOTO trash can`) · `CANCEL` (on every stop, and before any manual `MOVE`/`TURN`) · `SCAN` ("look around") · `FORGET [object]` ("forget everything" / "forget the bin") · `MEMORY SAY` ("what do you remember": voice prints the sentence, `BEEP min(n,4)`, and sends it to the phone) · `RECALL <object>` ("go back to where the bin was").

**`tars_motor` → Uno (USB serial 115200):** `D l r`, `M <mm>`, `R <deg>`, `S`, `K diam track` (steppers), `V [mm_s deg_s [deg_s_slow]]` (DC speed cal; two values keep the slow rate; sent bare after the first `S` as a query); `R <deg> <speed>` (DC: speed < 0.99 = slow turn), `L <0-3>`, `B <n>`.
**Uno → `tars_motor`:** `READY`, `WHEELS DC <mm_s> <deg_s>` (DC firmware: at boot before `READY`, and after `V`; switches `tars_motor`'s pose + move estimates to DC), then `WHEELS DCSLOW <deg_s_slow>` (separate line: older Pi code ignores it; a `WHEELS DC` without it means TURN speed is ignored), `DONE M|R <n>`, `ABORT …`, `T <deg>` tipped, `B <mg>` bump, `BTN TEACH`, `BTN FORGET`.
**SM11: the kit's Bluetooth module must be unplugged** (it's on D0/D1, the Pi link).
The full spec is in the header comment of `iter1-wasd/uno/tars/tars.ino`.

**Clients → `goto` (TCP 5570):** `GOTO <object words>` · `RECALL <object words>` · `CANCEL` · `STATUS` · `OBJECTS` · `RESOLVE <words>` · `SCAN` · `MEMORY` · `MEMORY SAY` · `NAV` · `NAV ROUTE` (`OK route=x,z;… i=<next>`) · `NAV GRID` (RLE of the planner grid) · `DETS` (`OK age=<s> <last detection JSON>`) · `FORGET [object[#n]]` · `WATCH` (async `EVENT searching|remembered|not_there|arrived|lost|failed:…|scanning|scanned … <object>`) · `PING`.
Object memory (`iter2-autonomy/goto/README.md`) stores instances (`trash can#1`, `#2`) with status fresh/stale/gone in `memory.txt` next to `goto.conf`.
**Phone / Mac fake → `goto` nav feed (TCP or UDP 127.0.0.1:5572):** `LDR1` packets, the same format as lidar_guard's 5562 (`iter2-autonomy/edge-ai/README.md`), with or without the 4-byte length prefix (TCP: with). Builds goto's occupancy grid + ARKit pose (`iter2-autonomy/goto/README.md` "Map-guided GOTO").
**voice → phone speech (TCP 127.0.0.1:5573, Andrew's SpeechLink via iproxy):** `SAY <text>` (≤ 300 chars) → `OK say`; urgent JSON `{"type":"say","text":"Stopping.","priority":"urgent"}` only *after* `S` + `CANCEL`. Best effort: never delays a stop. `say_port = 0` in `voice.conf` turns it off.
**Guard (Andrew's `lidar_guard` → `tars_motor` UDP 5568):** a denial zeroes `D`/`DL`, aborts `MOVE`, bumps `guard_epoch` (a leased `DL <epoch> l r` from before the denial is refused). An in-place `TURN` is still allowed while the reason is a fresh `guard_blocked` (forward-only guard; lets the robot pivot away), never when stale/missing/invalid, and only up to speed 0.40 (clamped; on DC that's the Uno's slow turn) and |deg| 180 (else `ERR guard_blocked pivot>180`); a faster/longer TURN already running aborts. *Pending Andrew's sign-off.* Exact `MOVE`/`TURN` also need a keepalive (`PING`/`D`) within the 500 ms deadman.

**Laptop / mission relay → `tars_voice` (TCP 127.0.0.1:5574, `net_port` in `voice.conf`, 0 = off):** `HEAR <text>` (same classifier + stop-first path as the mic, no wake word) · `AUDIO <bytes>` + 16 kHz mono s16le or WAV (≤ 15 s; whisper on the Pi) · `WATCH` (JSON events: `heard`/`sent`/`memory`/`say`) · `PING`.
**Laptop → `mission_relay` (TCP 5575, all interfaces, key required):** first line `HELLO <MOTOR|GOTO|VOICE|WATCH|CAM> <key>` (missing/wrong → `ERR key`, closed). The relay won't start without `-k <key>` or `MISSION_KEY`; `--no-key` is for the local demo stack only (loud warning). The laptop app takes `--key` / `MISSION_KEY`; MOTOR/GOTO/VOICE are byte pipes to 5555/5570/5574 (one service connection per laptop connection); WATCH = tagged status lines; CAM = phone frames from 5566 ≤ 5 fps. **Phone status tee → relay (TCP 127.0.0.1:5576):** JSON lines from `headless.py` when `TARS_MISSION_TEE_PORT` is set. Details: `iter2-autonomy/mission/README.md`.

**Ports.** Andrew's stack owns 5555 and 5557–5569 (`iter2-autonomy/edge-ai/README.md`). Ours: `goto` control **5570** (was 5557), detections **5571** (was 5558), nav feed **5572** (new); **5573** is TARS speech → phone (Andrew's SpeechLink; voice is a client); voice net input **5574**, mission relay **5575**, phone tee **5576**. *5570–5572 and 5574–5576 pending Andrew's sign-off.* Never bind 5562 (lidar_guard's LiDAR) or 5568 (guard → motor gate).
**Detectors → `goto` (UDP/TCP 127.0.0.1:5571):** one JSON line per frame, `{"w","h","dets":[{"label","conf","box":[x1,y1,x2,y2]}]}`. See `iter2-autonomy/goto/DETECTIONS.md`. `iter2-autonomy/goto/detect` (YOLO-World) is the only detector that feeds `goto`.

## Before you commit

Run the suites you touched, and preferably all of them (from the repo root):
```sh
make -C iter1-wasd/uno/test
make -C iter1-wasd/pi host && make -C iter1-wasd/pi test
python3 iter1-wasd/laptop/test_teleop.py
make -C iter2-autonomy/goto test test-nav-asan test-nav-parity test-fullstack
make -C iter2-autonomy/voice host && make -C iter2-autonomy/voice test
make -C iter2-autonomy/mission test
(cd iter3-pet/vision/tests && for t in test_pet test_gesture test_learn test_parity; do python3 $t.py; done)
```
Don't commit models (`*.task`, `*.bin`, `*.onnx`), `build/`, `.venv/` or `__pycache__/`. Use a `get_model.sh` script instead.

## Pi / QNX notes

- Cross-compile with `qcc` via `docker/run.sh` (needs `~/qnx800`). QNX 8.0 has Python and `apk`.
- Real-time loops use `SCHED_RR`. Keep heavy ML (YOLO, MediaPipe, whisper) at normal priority so they can't starve `tars_motor`.
- Speech uses `threads=2` so cores are left for vision.
