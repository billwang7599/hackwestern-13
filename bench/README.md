# bench/: the laptop digital test bench

**The robot never needs this.** The bench is a laptop-only dev tool: one browser page
(`python3 bench/bench.py`, then open http://127.0.0.1:8770) for driving, debugging and
experimenting with the robot stack. Python stdlib on the laptop, plain HTML/CSS/JS, no
build step, and nothing loaded from the internet (venue Wi-Fi).

Every part runs either **simulated** or **real**, and you can mix them:

| Part | Simulated | Real |
|---|---|---|
| Car (Uno + motors) | virtual SM11: the real `iter1-wasd/uno/tars/tars.ino`, compiled for the Mac with the test mocks, driving a DC-motor physics model, on a pseudo-serial port (`bench/car/`) | the real Uno on this Mac's USB (`tars_motor -b uno -d /dev/cu.usbmodem…`) |
| Pi brain | the real `tars_motor`, `goto`, `tars_voice`, `mission_relay` host builds on the Mac (same C code as QNX) | the Pi, through `mission_relay` (TCP 5575, key) |
| Phone | simulated room: ARKit pose, LiDAR guard, YOLO detections (`bench/world/`) | the iPhone on this Mac's USB, Andrew's receiver/mapping (`bench/phone/`) |
| Voice | text box | laptop mic, whisper + the real C intent classifier on the laptop |

`bench.py --target sim|usb|pi` says where the robot stack is. `bench/run_sim.py` starts
the whole simulated stack and then the bench.

## Contract (read before adding a part)

**Plugins** (server side): `bench/plugins/<name>.py` with `setup(bench)` (and optional
`shutdown()`). Loaded at startup; a plugin that fails to load is logged and skipped.
- `bench.route("GET"|"POST", "/api/<name>/<what>", fn)`: `fn(query: dict, body: bytes)`
  returns `(status, payload)`; payload dict/list → JSON, str → text, bytes → binary,
  or `(bytes, content_type)`.
- `bench.publish(topic, data, sticky=True)`: server-sent event to every browser tab.
  Sticky topics are replayed to tabs that open later.
- `bench.log(source, text)`: a line in the shared log strip.
- `bench.tap(fn)`: `fn(topic, data)` on every publish, server side (the recorder uses it).
  The robot plugin publishes every line it sends and the reply as a non-sticky `cmd` event,
  so the recorder sees commands as well as telemetry.
- `bench.target`: `{"name": "sim"|"usb"|"pi", "motor", "goto", "voice", "relay"}`.
- `bench.plugins.get("robot")`: the robot link (owned by the `robot` plugin). It is the
  **only** thing that opens connections to tars_motor / goto / tars_voice (or the relay):
  `send(dest, line) -> reply` with dest `"motor"|"goto"|"voice"`, and `stop()` (`S` to
  tars_motor, then `CANCEL` to goto). Other plugins call it; if it isn't loaded, they
  disable their send buttons and say so.

**Panels** (browser side): `bench/web/panels/<name>.js` calls
`Bench.register({id, title, order, wide, mount(el, Bench)})`.
In a panel: `Bench.on(topic, fn)`, `await Bench.api("<plugin>/<what>", body?)`, `Bench.log(src, text)`.
**STOP** and **Space** live in the shell (`web/shell.js` → `POST /api/robot/stop`), so
stopping never depends on a panel.

**Event topics:** `status` (tars_motor STATUS, parsed), `goto` (goto STATUS + EVENTs),
`car` (virtual car truth + params), `world` (sim room + sensors), `voice` (voice lab
results), `map3d` (3D map data), `scenario` (scenario progress), `cmd` (every line sent
to the robot + reply), `log`.

**Tests use free ports.** Several parts' tests run at once on one Mac, so a test never
binds the robot's real port numbers or 5590–5599; every process takes its ports as flags.

### Ownership (one owner per file, so parts can be built in parallel)

| Part | Files |
|---|---|
| shell | `bench.py`, `web/index.html`, `web/shell.js`, `web/bench.css` |
| robot link + drive | `plugins/robot.py`, `plugins/scenarios.py`, `plugins/recorder.py`, `web/panels/{drive,status,scenarios,recorder}.js` |
| virtual car | `car/`, `plugins/car.py`, `web/panels/car.js` |
| sim world | `world/`, `run_sim.py`, `plugins/world.py`, `web/panels/world2d.js` |
| voice lab | `plugins/voicelab.py`, `web/panels/voice.js` |
| iPhone 3D map | `phone/`, `plugins/phone.py`, `web/panels/map3d.js` |

Tests go in `bench/tests/test_<part>*.py`.

## Ports (laptop only, 127.0.0.1)

The robot's own ports (tars_motor 5555, goto 5570, detections 5571, map feed 5572, say 5573,
voice 5574, relay 5575, tee 5576, and Andrew's 5557–5569) are unchanged; see
`docs/ARCHITECTURE.md`. In sim, the host builds listen on the same numbers on 127.0.0.1
unless a test picks free ports.

| Port | Who listens | What |
|---|---|---|
| 8770 | `bench.py` | the browser page |
| 5590 | world sim / bench (UDP) | **virtual car truth**, 50 Hz, one JSON per datagram: `{"t": ms, "x": m, "y": m, "th": rad, "v": m/s, "w": rad/s, "wl": m/s, "wr": m/s, "pwm_l": -255..255, "pwm_r": …, "batt_v": V, "contact": bool}` |
| 5591 | virtual car (UDP) | **car control**, one JSON per datagram: `{"set": {<profile key>: value}}`, `{"pose": {"x","y","th"}}` (teleport), `{"fault": "push"\|"lift_left"\|"lift_right"\|"tip"\|"bump"\|"unplug"\|"brownout"\|"clear", …}`, `{"walls": [[x1,y1,x2,y2], …]}` (the car stops against walls and the firmware feels a bump) |
| 5592 | world sim | world control (format in `world/README.md`) |
| 5593 | bench (UDP) | world state for the 2D view (format in `world/README.md`) |
| 5594 | phone part (HTTP) | Andrew's room view (`room_view.serve_room`, unchanged) showing the bench's map; the `map3d` panel iframes it (`TARS_BENCH_PHONE_ROOM_PORT`) |
| 5595 | phone part (TCP) | scene source in the phone's 5561 format (usbmux forwarder / your `iproxy` / replay / sim) that Andrew's `SceneFeed` reads (`TARS_BENCH_PHONE_SCENE_PORT`) |
| 5596–5599 | | spare: add a row here before using one |

**World frame** (car truth, world sim, 2D view): metres, right-handed, z up, x = the car's
forward direction at its start pose, y = left, `th` = heading in radians, counter-clockwise
from +x. Converting to ARKit (y up) or goto's frames is the world sim's job.

## Testing honesty

Say exactly what ran: the sim, a recording, the real Uno on USB, the iPhone. The bench
proving something in sim does not mean the car does it.

## iPhone 3D map (`phone/`)

Andrew's mapping (`mapping.SceneFeed` + `Map`) and room view run unchanged on the laptop.
The source can be the live iPhone on this Mac's USB (stdlib usbmux client, no install), a
recording, or a ray-cast sim room. The sim room follows topic `world` live (rooms in the
`bench/world/rooms/*.json` shape), and falls back to a built-in room when no world is loaded.
The sim takes its pose from topic `car`. `/api/phone/*` and topic `map3d` are documented in
`plugins/phone.py`; details, frames and the live-phone checklist are in `phone/README.md`.
Test: `python3 bench/tests/test_phone.py`.
