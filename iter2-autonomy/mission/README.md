# Mission control: the laptop as a separate dashboard + controls

**The robot never needs this.** Everything here is optional. With the laptop off, absent or
disconnected, the robot behaves exactly as before. The laptop does no robot work: no
perception, mapping, planning, intent classification or decisions. It shows what the Pi
computed and passes your commands to the Pi's own processes.

```
 browser (localhost:8765) ── laptop/mission.py (Python stdlib) ──Wi-Fi TCP 5575──► mission_relay (C, Pi)
                                                                                      │ 127.0.0.1 only
       MOTOR pipe ─► tars_motor 5555      GOTO pipe ─► goto 5570      VOICE pipe ─► tars_voice 5574
       WATCH ◄─ tars_motor STATUS, goto STATUS/DETS/MEMORY/NAV/ROUTE/GRID/EVENTs, voice events,
                phone status (tee 5576 from Andrew's headless.py)
       CAM   ◄─ latest phone RGB frame from Andrew's frame broker 5566 (≤ 5 fps, passthrough)
```

## Run it

**On the Pi** (after the normal robot stack is up; build with `make -C iter2-autonomy/mission`
via `docker/run.sh`, or `make native` on the Pi with its clang):

```sh
MISSION_KEY=<key> ./mission_relay  # laptops on 0.0.0.0:5575 (or: ./mission_relay -k <key>)
# optional, to mirror what the phone shows (guard verdict, YOLOX boxes, map, rates):
TARS_MISSION_TEE_PORT=5576 python3 iter2-autonomy/dashboard/service.py start
# tars_voice listens for the laptop on 127.0.0.1:5574 by default (voice.conf net_port; 0 = off)
```

**The key is required.** Without `-k <key>` or `MISSION_KEY` the relay refuses to start, because
anyone on the Wi-Fi could otherwise drive the robot. Every laptop connection must send it in
`HELLO`; a missing or wrong key gets `ERR key` and is closed. Pick something long (no spaces).
`--no-key` exists only for the local demo stack and tests on a Mac: it logs a loud warning.
Never use it on the robot.

Normal priority, never SCHED_RR. If the relay dies, nothing on the robot notices except that
any drive the laptop started stops (its tars_motor connection closes).

**On the laptop:**

```sh
MISSION_KEY=<key> python3 iter2-autonomy/mission/laptop/mission.py --relay <pi-ip>   # open http://localhost:8765
python3 iter2-autonomy/mission/laptop/mission.py --relay <pi-ip> --key <key> --laptop-stt   # see "Voice"
```

The same key as the relay (`--key` or `MISSION_KEY`; the app won't start without one). A
wrong key shows as `ERR key` on every link in the UI.

No robot? `python3 iter2-autonomy/mission/tests/demo_stack.py` runs a fake Pi on the Mac
(real tars_motor stub, goto, tars_voice; fake phone, camera, detections) plus the app. It
starts the relay with a test key and passes the same key to the app.

## Controls

| Control | Sends | Notes |
|---|---|---|
| **STOP** (always on top) / **Space** | `S` → tars_motor, then `CANCEL` → goto | same order as a spoken stop |
| hold **W A S D** | `D l r` at 20 Hz, `S` on release (`iter1-wasd/laptop/teleop.py` `HoldDrive`) | A/D spin, W+A/D arc. If goto is busy: `CANCEL` first. goto answers it with `S`, so tars_motor may hold the drive 1.5 s (`ERR held`) |
| GOTO / RECALL / FORGET / FORGET all / SCAN / CANCEL | the goto line | only these commands are allowed |
| MEMORY SAY | `HEAR what do you remember` → tars_voice | the robot prints it, beeps, and the phone says it |
| 🎤 hold (or hold **T**) | 16 kHz mono clip → `AUDIO` → tars_voice | whisper + classifier on the Pi |
| text box | `HEAR <text>` → tars_voice | same classifier, stop words first |
| RESET tip | `RESET` → tars_motor | clears the tip latch |

Safety: if the browser tab goes quiet while a key is held, the app sends `S` after 0.35 s.
If `mission.py` dies, the relay closes its tars_motor connection, and tars_motor stops only
the drive the laptop started (tested: 82 ms, not the watchdog). Another client's move keeps
running. On a clean exit the app sends no `S`, because `S` would stop other clients too.

## Voice

The laptop mic sits close to whoever is talking, which is the main fix for crowd noise
(robot mic: quiet room 21/21, crowd 3–4/21). By default the clip goes to the Pi
(`AUDIO <bytes>`), where tars_voice runs whisper and the classifier. `--laptop-stt` is the
"cheat": whisper runs on the laptop (`iter2-autonomy/voice/build/stt_cli`, same wrapper and
model) and only `HEAR <text>` goes to the robot. The robot still classifies and decides.
Laptop input needs no wake word (it's push-to-talk). The robot mic keeps working at the same time.
whisper tiny.en sometimes repeats on short clips ("turn left, turn left,"). The robot's own
mic path does it too, and the classifier handles it.

## What each panel shows, and where it comes from

| Panel / data | Source on the Pi | Port, process | Format | Reachable over Wi-Fi? |
|---|---|---|---|---|
| tars_motor: wheels, stop reason, `stops=`, tipped/tilt, bump, move, mode, clients, pose, guard gate (`guard_permit/reason/age`) | `tars_motor` `STATUS` | 5555, tars_motor | one text line | no: Andrew's service starts it with `-l` → relay MOTOR pipe / WATCH `M` |
| goto: state, target, result, route progress, guard, fps | `goto` `STATUS` | 5570, goto | text line | goto binds all interfaces, but its sends are blocking, so it's always read through the relay (WATCH `G`) |
| goto events (searching, arrived, failed:…) | `goto` `WATCH` | 5570 | `EVENT …` lines | WATCH `E` |
| detections (YOLO-World) for boxes | `goto` `DETS` (new) | 5570 (detector → goto on UDP 5571 can't be shared) | `OK age=s {json}` | WATCH `Y` |
| object memory | `goto` `MEMORY` | 5570 | text, `map=wx,wz` per object | WATCH `O` |
| robot pose (ARKit or dead reckoning) | `goto` `NAV` | 5570 | text | WATCH `N` |
| planned route | `goto` `NAV ROUTE` (new) | 5570 | `route=x,z;…` | WATCH `T` |
| planner grid | `goto` `NAV GRID` (new) | 5570 | RLE over the known box | WATCH `K` (1 Hz) |
| voice: transcript, intent, confidence, what it sent, what TARS said | `tars_voice` `WATCH` (new) | 5574, tars_voice (127.0.0.1) | JSON lines: heard/sent/memory/say | WATCH `V` |
| SAY text to the phone | tars_voice `say` events | (5573 is the phone's) | JSON | WATCH `V` |
| phone camera | Andrew's `rgb_broker.py` latest frame | 5566 (127.0.0.1), many readers allowed | `RGB1` + JPEG | relay CAM, ≤ 5 fps |
| phone camera (raw) | phone → Pi | 5558 / 5563 via iproxy | JPEG | **no**: the phone takes one client per stream |
| guard verdict CLEAR/CAUTION/BLOCKED/UNKNOWN/STALE, nearest obstacle | `lidar_guard` → headless `EdgeStatus` | UDP 5564 (bound by headless) | JSON | only via the phone tee (`P`) |
| YOLOX boxes on the phone frame, MiDaS timing | AI workers → headless | UDP 5565/5567 (bound by headless) | JSON | only via the phone tee |
| phone-shown map: grid, pose, path, objects, tracking, floor, voxels; motion rates | headless `status` → phone | 5559 via iproxy (phone accepts one client) | JSON | only via the phone tee |
| guard → motor gate | `lidar_guard` → tars_motor | UDP 5568 | `GAT1` | never touched (shown through tars_motor STATUS) |

Without the phone tee, the safety panel falls back to tars_motor's gate (`guard_permit` /
`guard_reason`), and the map uses goto's own grid. Data older than 1 s is greyed out
(2.5 s for the 1 Hz items: map, memory, phone). Every link shows its own connection pill.
The relay's `R` line reports which Pi services are up.

## Ports (new ones pending Andrew's sign-off, like 5570–5572)

| Port | Bind | Who | What |
|---|---|---|---|
| **5574** | 127.0.0.1 | tars_voice | network input: `HEAR <text>`, `AUDIO <bytes>` + PCM/WAV (16 kHz mono 16-bit, ≤ 15 s), `WATCH` (JSON events), `PING` |
| **5575** | 0.0.0.0 | mission_relay | the laptop's only port: `HELLO <MOTOR\|GOTO\|VOICE\|WATCH\|CAM> <key>` (key required) |
| **5576** | 127.0.0.1 | mission_relay | phone status tee: newline-delimited JSON from `headless.py` (`TARS_MISSION_TEE_PORT`) |
| 8765 | 127.0.0.1 (laptop) | mission.py | the browser page |

## Relay rules (`relay.c`)

- One thread, `poll()`, every socket non-blocking.
- Service replies are always read at once. goto's `send_str` blocks, and tars_motor
  answers inline, so a slow laptop must never back them up.
- A laptop that doesn't read misses whole WATCH lines (2 MB queue each). It skips CAM frames
  (it only gets a new frame once the last one has gone out), or has its pipe closed (1 MB).
  Closing a MOTOR pipe stops that laptop's drive.
- Rates: tars_motor/goto STATUS + DETS 5 Hz; MEMORY/NAV/ROUTE 1 Hz; NAV GRID ≤ 1 Hz; phone
  ≤ 1 Hz; camera ≤ 5 fps (hard caps). Pollers connect only while someone watches, with
  ≤ 8 polls outstanding. The motor poller sends only `WHO` and `STATUS`, and `STATUS` doesn't
  feed the watchdog, so it can't keep anyone's move alive.
- Each MOTOR/GOTO/VOICE laptop connection gets its own service connection, so tars_motor's
  "a disconnect stops only that client's drive" rule applies to each laptop.

## Changes outside this folder

- `iter2-autonomy/goto/src/goto.c`: `DETS`, `NAV ROUTE`, `NAV GRID` (read-only).
- `iter2-autonomy/voice/src/main.c`: network input + events. `src/stt_cli.c` is a laptop-only tool.
- `iter2-autonomy/dashboard/headless.py` (Andrew's code; approved, Andrew will be told): opt-in
  `MissionTee`, ~40 lines, off unless `TARS_MISSION_TEE_PORT` is set. It's the only way to see what the phone
  sees, because 5559 takes one client and the guard/AI UDP ports are bound by headless.

## Tests (Mac only)

```sh
make -C iter2-autonomy/mission test      # relay 44/44 + laptop app 32/32
make -C iter2-autonomy/voice test-net    # HEAR/AUDIO/WATCH 25/25
```

`test_relay.py` runs the real tars_motor stub plus fakes. It covers: the MOTOR pipe disconnect,
which stops only that laptop's drive (another client's MOVE survives); rate limits; byte-exact
CAM passthrough; a non-reading laptop not blocking the fast laptop or goto; clean disconnects;
the key (missing / wrong → `ERR key` + closed, right → works, `MISSION_KEY`, no key → won't
start, `--no-key` warns); service down; the headless tee. `test_laptop.py` runs the real tars_motor + goto + relay +
`mission.py` through its HTTP API. It covers: STOP = S then CANCEL; WASD; browser-quiet stop;
kill -9 stopping only its own drive; no key → app won't start, wrong key → every link `ERR key`; goto buttons; voice text/clip; `--laptop-stt`; MJPEG; stale data.

**Not run:** on the Pi, under QNX/qcc, over real Wi-Fi, with a real iPhone, a real laptop mic
in a browser (the JS was syntax-checked and the page logic exercised through the same HTTP
API, but not clicked in a real browser), or with the real headless service.
