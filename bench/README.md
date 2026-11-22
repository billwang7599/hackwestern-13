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
  The header shows a **SIM** or **REAL ROBOT** badge from it. On `usb`/`pi`, `/panels.json` leaves
  out the sim-only panels (`SIM_ONLY_PANELS` in bench.py: car, world2d) unless `--all-panels`.
- `bench.plugins.get("robot")`: the robot link (owned by the `robot` plugin). It is the
  **only** thing that opens connections to tars_motor / goto / tars_voice (or the relay):
  `send(dest, line, data=b"") -> reply` with dest `"motor"|"goto"|"voice"` (`data`: raw
  bytes written right after the line, for tars_voice's `AUDIO <n>` + clip; the reply is the
  next line), and `stop()` (`S` to
  tars_motor, then `CANCEL` to goto). Other plugins call it; if it isn't loaded, they
  disable their send buttons and say so. `listen("watch", fn)` gets mission_relay's WATCH lines
  (pi) or `V <json>` tars_voice events (sim/usb); `listen("cam", fn)` gets relay CAM frames (pi).

**Panels** (browser side): `bench/web/panels/<name>.js` calls
`Bench.register({id, title, short, order, area, help, view, wide, tall, mount(el, Bench)})`.
`help` is one plain-English line saying what the panel is for: the shell puts a small **ⓘ** after the title
that shows/hides it (closed by default, remembered per panel in `localStorage` as `bench.help.<id>`), and it
is the tooltip on the panel's sub-tab or map-view button. Every panel should have one. The map view switch has
its own ⓘ that shows the current view's `help`. `short` is the label on a sub-tab (else `title`); `tall`
makes a grid panel span two rows (the Car page's virtual car beside safety + raw STATUS).
In a panel: `Bench.on(topic, fn)`, `await Bench.api("<plugin>/<what>", body?)`, `Bench.log(src, text)`,
`Bench.emit(topic, data)` (local event to this tab's `Bench.on` handlers), `Bench.openPage(id)` (`openTab` is
the old name), `Bench.page`, `Bench.on('page', fn)`, `Bench.openSub(page, panelId)` (pick a sub-tab), `Bench.showPanel(id)`
(its page, its sub-tab, scrolled into view), `Bench.showView(id)`, `Bench.setBadgeDetail(text)`, and
`Bench.mic.start()` / `await Bench.mic.stop()` (the one laptop-mic capture: a 16 kHz mono WAV, `mic` local
event while recording).
**STOP** and **Space** live in the shell (`web/shell.js` → `POST /api/robot/stop`), so
stopping never depends on a panel. The STOP button is in the top bar, which is on every page, and Space
stops on every page unless focus is in a text box.

**Pages** (a navbar under the top bar, one job per page; `area` = the page id). The top bar (STOP, SIM/REAL
badge, connection lights, what the robot is doing) and, while the guard blocks, a plain-words note under it
stay on every page. Pages marked *map* show the big centre view (`area: 'view'` panels: `world2d`, `mc-map`,
`map3d`, with a switch above it) beside their column; the others use the full width in a grid
(`wide: true` spans it).

Each page starts with a collapsible **What's this page for?** box (`ABOUT` in `shell.js`: what the page is
for, what you can do, one tip), full width under the navbar. It is open on the first visit; after that it
stays as the user left it (`bench.about.<page>`); closed, it shows a one-line reminder. Pages marked *sub*
show one panel at a time behind sub-tabs (the panels in `order`; the choice is remembered as
`bench.sub.<page>`, the default is the lowest order), so no page is one long scroll.

| Page (`#hash`, key) | Map | Panels |
|---|---|---|
| Drive `#drive` 1 (default) | yes | `command` (Talk to TARS), `drive` (WASD pad, speed, exact MOVE/TURN) |
| Room `#room` 2 | yes, edit mode on, *sub* | `room-objects` (every object and wall, 🗑 per row, Delete selected, Undo), `world-tools` (faults, sensor noise), `maps` (map demos) |
| Memory `#memory` 3 | yes, Robot's map | `mc-memory` (what goto remembers, Forget per row, Forget all with a confirm), `mc-goto` (goto status table, GOTO/RECALL/SCAN/CANCEL, events) |
| Voice lab `#voice` 4 | | `voice` |
| Car `#car` 5 | | `mc-safety` (guard), `car` (virtual car readings, faults, profile; `tall`), `status` (raw tars_motor STATUS) |
| Tests `#tests` 6 | *sub* | `scenarios` (table with results), `drive-debug` (wheel chart, raw console), `recorder`, `log` |
| Mission `#mission` 7 | | `mc-camera`, `mc-voice`, `mc-phone`, `mc-pi` |

The page is the URL hash (linkable), else the last one shown (`localStorage`), else Drive; keys 1–7 switch
pages outside text boxes. Each map page remembers its own map view. Old drawer ids still work as `area`
(`side`→drive, `robot`→memory, `sim`/`maps`→room, `pi`→mission, `debug`→tests); an unknown id makes a new
page. A page left with no panels (Room on the real robot: its panels are sim-only) drops out of the navbar.
Hidden pages stay in the DOM, so panels keep their state and their topic handlers; canvases are 0×0 while
hidden and redraw on the `resize` the shell fires after every page/view switch. `?nohelp` skips the
first-run help (screenshots). The last log line is in the footer (click: the Log sub-tab on Tests).

**Removing things:**
- *Sim room objects* (Room page): 🗑 on a row of "Things in the room" (walls are under "Walls"), **Delete
  selected** (click a row or the object on the map first), **Del** on the map, or **right-click → Delete**
  on the map (on any map page). All send `world/ctl {op: remove, id}`. **↶ Undo** (or ⌘Z on the Room page)
  re-adds the last one as it was (`{op: add, obj}` / `{op: add_wall, seg}`; a re-added wall gets the next
  wall id).
- *Resets* (map toolbar and Room page, two buttons):
  - **🚗 Car to start** (`POST /api/world/car_to_start`, world op `car_to_start`): S + CANCEL first, then only
    the car jumps to the room's start. Placed things, drawn walls, faults and goto's map and memory are
    kept, and goto is **not** restarted. goto's nav pose is the phone's ARKit pose, whose frame is fixed
    to the room, so the next LDR1 packet puts goto at the start on its own map. tars_motor's dead-reckoned
    pose doesn't see the jump, like carrying the real robot. goto's map pose re-anchors to ARKit, but
    memory sightings use that dead-reckoned frame until the next goto restart (map-mode GOTOs use each
    memory's ARKit `map=` position).
  - **🗺 Reset map** (`POST /api/world/reset_map`, world op `reset_map`, asks to confirm): the room goes
    back to how it was loaded or saved (removes things and walls you added, clears faults), and goto
    restarts with an empty map and the room's memory. The car stays where it is: goto's seeded memory
    file gets a `pose` line with the car's real pose and an epoch tars_motor never uses, so goto's memory
    frame re-bases onto it.
  - `POST /api/world/reset` (world op `reset`) is the old combined reset (room + car to start + goto
    restart). It's still there for scripts but has no button.
- *goto's memory* (Memory page): **Forget** on a row sends `FORGET <label>#<id>` and **🗑 Forget all…**
  (asks first) sends `FORGET all`, both through `POST /api/mission/goto` (mission plugin → robot link → goto).
  goto answers `OK forgot n`. Anything the camera still sees is remembered again at once, as a new sighting.

Test: `python3 bench/tests/test_pages.py` (static page/panel checks + both remove paths on a stub-car sim;
`--static` skips the sim).

**Event topics:** `status` (tars_motor STATUS, parsed), `goto` (goto STATUS + EVENTs),
`mission` / `mission_grid` / `mission_phone` / `journal` (mission control, below),
`car` (virtual car truth + params), `world` (sim room + sensors), `voice` (voice lab
results), `map3d` (3D map data), `scenario` (scenario progress), `cmd` (every line sent
to the robot + reply), `navgrid` (goto's `NAV GRID`, on change), `maps` (map demo / explorer state), `log`.

**Tests use free ports.** Several parts' tests run at once on one Mac, so a test never
binds the robot's real port numbers or 5590–5599; every process takes its ports as flags.

### Ownership (one owner per file, so parts can be built in parallel)

| Part | Files |
|---|---|
| shell | `bench.py`, `web/index.html`, `web/shell.js`, `web/bench.css`, `web/panels/{topbar,command}.js`, `tests/test_pages.py` |
| robot link + drive | `plugins/robot.py`, `plugins/scenarios.py`, `plugins/recorder.py`, `web/panels/{drive,status,scenarios,recorder}.js` |
| virtual car | `car/`, `plugins/car.py`, `web/panels/car.js` |
| sim world | `world/`, `run_sim.py`, `plugins/world.py`, `web/panels/world2d.js` |
| voice lab | `plugins/voicelab.py`, `web/panels/voice.js` |
| iPhone 3D map | `phone/`, `plugins/phone.py`, `web/panels/map3d.js` |
| mission control | `plugins/mission.py`, `web/panels/mission.js`, `mission.command` (imports `iter2-autonomy/mission/laptop/`, which Andrew owns) |
| map demos | `maps/`, `plugins/maps.py`, `web/panels/maps.js`, `world/prescans/` (plus small hooks: `world2d.js` layers/pick, `worldsim.py` `seed_nav`, `phone/sim.py` surfaces) |

Tests go in `bench/tests/test_<part>*.py`.

## Ports (laptop only, 127.0.0.1)

The robot's own ports (tars_motor 5555, goto 5570, detections 5571, map feed 5572, say 5573,
voice 5574, relay 5575, tee 5576, phone mic 5577, and Andrew's 5557–5569) are unchanged; see
`docs/ARCHITECTURE.md`. In sim, the host builds listen on the same numbers on 127.0.0.1
unless a test picks free ports.

| Port | Who listens | What |
|---|---|---|
| 8770 | `bench.py` | the browser page |
| 5590 | world sim / bench (UDP) | **virtual car truth**, 50 Hz, one JSON per datagram: `{"t": ms, "x": m, "y": m, "th": rad, "v": m/s, "w": rad/s, "wl": m/s, "wr": m/s, "pwm_l": -255..255, "pwm_r": …, "batt_v": V, "contact": bool}`, plus extra keys (additive, see "Virtual car" below). The car sends it to 5590 **and** to every subscriber (`{"sub": port}` on 5591), so the world sim and the bench both get it |
| 5591 | virtual car (UDP) | **car control**, one JSON per datagram: `{"set": {<profile key>: value}}`, `{"pose": {"x","y","th"}}` (teleport), `{"fault": "push"\|"lift_left"\|"lift_right"\|"tip"\|"bump"\|"unplug"\|"brownout"\|"clear", …}`, `{"walls": [[x1,y1,x2,y2], …]}` (the car stops against walls and the firmware feels a bump), `{"sub": <udp port>}` (send truth there too for 5 s; renew every ~2 s), `{"button": "tap"\|"hold"\|"down"\|"up"}`, `{"reset": true}`, `{"get": "profile"}`. **Every datagram gets a JSON reply** to its sender: `{"ok": true, …}` or `{"ok": false, "error": …}` |
| 5592 | world sim | world control (format in `world/README.md`) |
| 5593 | bench (UDP) | world state for the 2D view (format in `world/README.md`) |
| 5594 | phone part (HTTP) | Andrew's room view (`room_view.serve_room`, unchanged) showing the bench's map; the `map3d` panel iframes it (`TARS_BENCH_PHONE_ROOM_PORT`) |
| 5595 | phone part (TCP) | scene source in the phone's 5561 format (usbmux forwarder / your `iproxy` / replay / sim) that Andrew's `SceneFeed` reads (`TARS_BENCH_PHONE_SCENE_PORT`) |
| 5596–5599 | | spare: add a row here before using one |

**World frame** (car truth, world sim, 2D view): metres, right-handed, z up, x = the car's
forward direction at its start pose, y = left, `th` = heading in radians, counter-clockwise
from +x. Converting to ARKit (y up) or goto's frames is the world sim's job.

## Virtual car (`bench/car/`)

**What it is:** the real `iter1-wasd/uno/tars/tars.ino`, default `CAR_SM11` build (DC wheels,
EN+IN driver, kit pins), compiled **unchanged** for the Mac with the firmware test mocks
(`iter1-wasd/uno/test/mock/`), inside `build/vuno`, which adds:
- **Serial** on a pseudo-terminal. Opening it resets the "Uno" like DTR does (WHEELS DC,
  WHEELS DCSLOW, READY after `boot_ms`, bytes sent meanwhile are lost).
- **Time:** `millis()`/`micros()` from the real clock.
- **Pins → physics** (`physics.h`, 1 ms steps): L298N coast/brake/drive, a sagging battery,
  deadband (start vs keep-rolling), first-order lag, side gains, slip, drift, skid-steer
  track, walls (stall plus a bump spike). **4WD** (`profiles/sm11.json`, the real car): two TT
  motors in parallel per channel (twice the current and torque), the L298N drop growing with
  current, a per-channel current limit, tyre scrub in a spin (a 4-wheel skid-steer must drag
  its tyres sideways), a rectangular body whose corners sweep the half-diagonal in a spin,
  and a **motor-caused brownout** (battery on-phase sag under `uno_resets_below_v` for
  `brown_hold_ms` resets the Uno). `profiles/sm11_2wd.json` is the old 2WD car.
- **Accelerometer:** physics → MPU6050 registers, so tip/bump run the firmware's own code.
- **Button / LED / buzzer** on the kit pins (D13 / D3 / D12).

```sh
make -C bench/car                                 # build/vuno (host clang)
python3 bench/car/launch.py --motor 5555          # car + host tars_motor -b uno, ports 5590/5591
python3 bench/car/launch.py ctl '{"fault":"tip"}' # one control datagram
make -C bench/car test                            # physics.h unit tests + brownout grid (no firmware)
python3 bench/tests/test_car.py                   # through the real tars_motor + wheel_cal.py (2WD; --profile sm11 for 4WD)
python3 bench/tests/test_car_4wd.py               # 2WD vs 4WD: MOVE/TURN/slow TURN, uncalibrated and after wheel_cal.py
python3 bench/tests/test_car_plugin.py            # plugins/car.py routes + topic
bench/car/hil.sh                                  # the REAL Uno on this Mac's USB (untested)
```

**For run_sim.py:**
`sys.path.insert(0, "bench/car"); import launch as car`, then
`proc, dev = car.start("sm11", truth_port, ctl_port, seed)`,
optionally `car.start_motor(dev, port, guard_port=…)`, and finally `car.stop()`.
- Give tars_motor **`dev`** (a symlink, default `<tempdir>/tars-vuno-<ctl_port>`), not the
  `/dev/ttys…` path: "unplug" re-creates the pty and moves the link, which is how tars_motor's
  normal 1 s reconnect finds it.
- `start` prints nothing; the car logs to the `log=` file. `--trace` (in `extra=`) logs every
  serial line both ways.

**Truth extras** (beyond the 5590 row): `batt_a`, `mode_l`/`mode_r` (0 coast, 1 brake,
2 drive), `odo` (m driven), `roll` (deg), `imu` [x,y,z] g, `led`, `buzz`, `btn`,
`fw` ("run"/"reset"), `boots`, `plugged`, `host_open`, `pty`, `faults` {tip, lift_left,
lift_right, brownout, unplug}, `stalls`/`stall_max_ms` (main-loop gaps over 50 ms, meaning a busy laptop).
`wl`/`wr` are wheel **surface** speeds: a lifted wheel spins, a stalled one reads ~0.
4WD extras: `batt_v_min` (now, PWM on-phase: both channels conducting), `batt_v_low` (lowest
since the last `{"stats_reset": true}` on 5591), `ch_a` [l, r] channel amps, `ilim` [l, r]
(at `ch_limit_a`), `stall` [l, r] (driven but under the start/keep voltage: buzzing in place),
`sag_resets` (brownouts the motors caused), `motors_per_side`.

**Faults:**
- `push` (`vx`,`vy` body m/s, default 0.3 sideways)
- `bump` (`g`, default 2.5)
- `tip` (roll 75°, until clear)
- `lift_left` / `lift_right` (that wheel loses the floor, 12° roll)
- `unplug` (`ms`, default 1500, 0 = until clear; the firmware keeps running on the battery)
- `brownout` (`v` default 4.0, `ms` default 400; below `uno_resets_below_v` the Uno resets
  and says READY again)
- `clear`

**Profile:** `profiles/sm11.json`. **Every value is `measured: false`**, with its source;
the file's `measure_when_the_car_arrives` lists how to replace each one (wheel_cal.py
dc-move/dc-spin, docs/PI_TEST.md §2b and Step 5). They are tuned so the firmware's own
defaults make sense: duty 0.45 (`DC_MAX_DUTY`) gives 150 mm/s, duty 0.30 (`DC_MIN_DUTY`)
keeps a rolling wheel going but can't start one, so the 0.8 × 40 ms kick does that. The
skid-steer track (2WD: 0.20 m, from `test_dc_car.py`) makes an **uncalibrated** `TURN` under-turn
(2WD: D 1 spins ~68°/s here vs the firmware's default 120), as a real car would before
`wheel_cal.py dc-spin`. Live changes: `{"set": …}` or the panel sliders; nothing is saved.
The firmware's EEPROM survives resets and unplugs, but not a restart of `vuno`.

**4WD parameters** (all `measured: false`; defaults keep a 2WD circle car exactly as before):
`motors_per_side` 2, `driver_drop_v` 1.3 + `driver_r_ohm` 1.2 × channel A (L298N VCEsat vs
current), `ch_limit_a` 2.0 (L293D-style board: 0.6), `track_m` 0.13, `wheelbase_m` 0.10,
`mass_kg` 0.85 (kit + Pi + iPhone), `scrub_mu` 0.4, `motor_n_per_v` 0.55, `length_m` 0.20,
`width_m` 0.175 (track, wheelbase and body estimated from the user's photo with the 65 mm TT
wheel as scale), `brown_hold_ms` 5, `tau_s` 0.14 (÷ motors per side). `track_eff_m` 0 =
derive `track_m × (1 + (wheelbase/track)²)` = 0.207 m. Spin scrub per motor =
`scrub_mu × mass × g × wheelbase / (2 × track) / (motors_per_side × motor_n_per_v)` volts on
top of `v_keep`/`v_start` (1.17 V here), scaled by how much of a spin the command is.

**What 4WD predicts, with these guesses** (`make -C bench/car test`, `test_car_4wd.py`):
- **The firmware can't spin the loaded 4WD car in place.** `TURN 90` at D 1 (kick 0.8 × 40 ms,
  then `DC_MAX_DUTY` 0.45) turns ~1°, the slow turn (0.36) the same; the lowest spin duty that
  turns from rest is ~0.84 (duty 1.0: ~133°/s). The photo geometry (shorter wheelbase)
  only lowered that from ~0.88. Straight driving is fine. `wheel_cal.py dc-spin`
  can't calibrate it: at 0° it says "measured amount must be > 0" (slow: "the slow turn
  stalled: raise DC_SLOW_TURN_DUTY"); a few degrees ends in a SPEEDCAL usage traceback (the
  scaled deg/s is under SPEEDCAL's 20). A bare kit on a slick floor (0.45 kg,
  mu 0.2) turns at D 1 (~22°/s) but its slow turn still stalls. Measure this first when the
  car arrives.
- **Taped rear tyres** (`profiles/sm11_taped.json`, rear `scrub_mu` 0.15: with grippy fronts
  the pivot moves to the front axle and only the taped rears slide, so the model's symmetric
  scrub with mu = the rears' is the right resistance; the bench still spins about the centre,
  not the front axle). It turns, but only just: lowest spin duty ~0.46 vs the 0.45 cap,
  `TURN 90` creeps ~16° (uncalibrated) and the slow turn stalls; `dc-spin` can't save it (the
  rate is under SPEEDCAL's 20°/s). Tape alone is not enough with these guesses; tape plus a
  higher spin duty would be.
- **Brownout:** none on a 7.4 V pack with 0.30 Ω (lowest 6.64 V on a duty-1 spin start,
  6.73 V on a firmware TURN; 4WD stall 1.2 A per channel, under the L298N's 2 A). A flat pack
  in a weak holder (7.0 V, 0.5 Ω) resets the Uno on a duty-1 spin start (5.91 V); the
  firmware's TURN on it sits at 6.01 V, right on the 6.0 V guess. With the Pi's USB plugged
  in, the Uno R3 switches to USB power below ~6.6 V on VIN and should not reset at all
  (`uno_resets_below_v` 0).
- **Corners:** a spin 3 cm from a wall clips it with a corner (shoved ~4 mm, a ~2.0 g spike:
  over the firmware's 1.2 g bump threshold). The old circle body never touched it.
- `sim GOTO` on `sm11` inherits the spin stall, so `run_sim.py` defaults to `--profile sm11_2wd`
  for the software demo (user decision); `--profile sm11` runs the 4WD model.

**Determinism:** noise comes from `--seed`, but the firmware runs on the real clock, so two
runs agree statistically, not bit for bit. A busy laptop shows up in `stalls`.

**What it can't tell you:**
- The real kit's numbers (all guesses until measured).
- PWM-frequency effects (490 vs 980 Hz), beyond the gain.
- Serial at 115200 baud (the pty is instant).
- The AVR's speed or RAM.
- The Bluetooth module on D0/D1.
- Whether the real Uno resets on open (CH340 clones do too; `--no-reset-on-open` models one that doesn't).

## Voice lab (`plugins/voicelab.py`, `web/panels/voice.js`)

Text or laptop-mic speech goes through the robot's **real** voice code on the laptop. Needs
`iter2-autonomy/voice/get_model.sh` and `make -C iter2-autonomy/voice host`.

| Shown | Comes from |
|---|---|
| decision (action, intent, prob, why) | `build/intent_cli`: the C classifier, the same `intent.c` as on QNX |
| top-3, amount, object, wake word | `intent/intent.py` (the Python reference parsing the same `intent_model.h`). The test checks that its top-1 matches intent_cli on every phrase |
| **lines the robot sends** | a **shadow `tars_voice_host`** (the same `main.c`) whose tars_motor / go-to / phone-speech links go to fake listeners on free ports. The lab sends it `HEAR <text>` and records what arrives, in order (e.g. `S`, `CANCEL`, `BEEP 1`, the urgent "Stopping." SAY). The fakes answer `OK`, so `MEMORY SAY` shows "no memory" |
| transcript (🎤 or batch audio) | `build/stt_cli` (whisper tiny.en, as mission `--laptop-stt`) |

- **Modes:** *push-to-talk* is the robot's `HEAR`/`AUDIO` input, with no wake word.
  *Robot mic* requires "TARS". Wake carry-over ("TARS" … pause … "turn left") is not modelled.
- **Mic:** hold 🎤 or **T**. The browser records, decimates to 16 kHz mono and POSTs a WAV
  (the same capture code as mission control). Clips are 0.1–15 s.
- **Send switch** (off = dry run; it is disabled without the robot plugin). When on, text
  goes as `HEAR <text>` and the clip as `AUDIO <n>` + bytes, via `robot.send("voice", …)`. The
  robot's own classifier decides. If `robot.send` has no `data=`, audio falls back to the
  laptop transcript as `HEAR`, and the panel says so.
- **Batch:** `realistic.tsv` (labelled; scored like `test_intent.py`), `phrases.txt` (`{n}`
  → "ninety degrees", `{d}` → "half a meter", `{o}` → "trash can"; training data, so it
  isn't a held-out score), or a WAV folder (`labels.tsv` `file<TAB>intent[<TAB>first line]`,
  or names like `goto__far.wav`).
  - *Speak it* synthesises each line with macOS `say` (cached in `$TMPDIR/tars-bench-voice`)
    and runs it through whisper.
  - Noise: white or babble (a WAV, or `say` crowd sentences), at a chosen SNR.
  - Output: accuracy, wrong motions, stop recall, a confusion table, the worst misses (wrong
    motions first), median/p95 latency and the load average.
- **Latency caveat:** the stt/classify times include starting the process (and whisper's ~30
  ms model load). The robot keeps the model loaded.
- API: `GET /api/voicelab/info`, `POST /api/voicelab/{text,audio,send,batch,batch_cancel}`,
  `GET /api/voicelab/batch_last`. Topic `voice` carries `{kind: result|batch|send}`.
- Test: `python3 bench/tests/test_voice.py`.

## Sim world + `run_sim.py` (`bench/world/`)

`python3 bench/run_sim.py [--room NAME] [--free-ports] [--car auto|virtual|stub] [--profile sm11_2wd|sm11] [--relay]` starts
the virtual car (`bench/car/launch.py`, if present) or the **stub car**, the real `tars_motor`
(`-b uno -d <car link>` or `-b stub`, guard on `-g`), `goto` (memory seeded from the room),
`tars_voice` (if built and its model exists), `mission_relay` (`--relay`), the world sim and
`bench.py --target sim`, and prints what runs on which port. Ctrl-C: `S` + `CANCEL`, then
every child stops. Logs go to a temp run directory (printed).

run_sim hands the bench these environment variables: `BENCH_WORLD_CTL` (5592), `BENCH_WORLD_STATE`
(5593), `BENCH_SUPERVISOR` (an ephemeral run_sim port: `{"op": "restart_goto", "memory"}` gives a fresh
goto + `POSE RESET` when a room loads), `BENCH_CAR_CTL` (5591), `BENCH_CAR_TRUTH` (a copy of the truth
stream, for the car plugin with the stub car), `BENCH_CAR_KIND` (`virtual` | `stub`).

### Demo sim speed (`--speed N`, top-bar selector)

`run_sim.py --speed 4` (or the **1× / 2× / 4× speed** selector in the top bar, sim only) makes a go-to
play N times faster, for demos. Bench only: the robot always runs at 1×, and 1× is the default.
- **What speeds up:** the virtual car (firmware `millis()`, physics, truth rate, boot, unplug and
  button timers: `vuno --speed`), the world sim (phone, LiDAR and camera rates and latencies, and
  fault durations in sim time: `worldsim.py --speed`) and goto (`-T N`, its existing time scale; its
  map clear-span timers follow it).
- **What stays real time:** `tars_motor` (unchanged; its 50 Hz D keepalive and 100 ms guard TTL are
  wall clock, so the world's GAT1 stays at 100 wall Hz), voice and the bench.
- **Host stalls:** because tars_motor is real time, a 125 ms laptop stall would look like 500 ms to the
  firmware at 4× and trip its mid-move link guard. So the car's clock runs at N× only while
  tars_motor's bytes are fresh (< 40 ms of wall time) and falls back to 1× during a stall. Car control
  `{"get":1}` reports `speed` and `slow_ms`, the wall time spent at 1× for this reason.
- **Live switch:** the top bar or `POST /api/world/speed {"speed": N}` sends S, sets the world and
  car clocks, then restarts goto with `-T N`. goto's memory is kept; its map starts empty.
- **Limits:** the virtual car only; the stub car's moves are timed by tars_motor, so it refuses. WASD
  driving is N× faster too, and tars_motor's dead-reckoned pose for plain `D` drives is N× short;
  MOVE and TURN use the firmware's own numbers, so they're right. Up to 8× matched 1× in
  `tests/test_speed.py`. Above about 8×, tars_motor's 20 ms D tick exceeds the firmware's 200 ms
  drive watchdog in sim time.
- **Tests:** `python3 bench/tests/test_speed.py [--speeds 1,4,8] [--only open_bin,pocket_mouth,live]`.

The 2D panel (`world2d`) is a course editor: place boxes, walls, people and goto objects, drag,
rotate (`[` `]`), delete, save/load layouts (edit mode is on while the **Room** page is shown; **✏ Edit room** on the Drive page goes there; outside it a
click only focuses the map for WASD). Faults, sensor noise and the sensor readout are the `world-tools` panel on
the **Room** page. The header badge always says **SIM** (plus the car kind) on the sim target. Room
format, world control (5592) and world state (5593): `bench/world/README.md`.

## Robot link, drive, status, scenarios, recorder

**Run it against a stub stack** (no car, no phone):
```sh
make -C iter1-wasd/pi host && make -C iter2-autonomy/goto host
iter1-wasd/pi/build/tars_motor_host -b stub -p 5555 -r 0 &
iter2-autonomy/goto/build/goto_host -m 127.0.0.1:5555 -p 5570 -r 0 -M - -o iter2-autonomy/goto/objects.txt &
python3 bench/bench.py            # http://127.0.0.1:8770
# real Pi: python3 bench/bench.py --target pi --relay <pi-ip> --key <MISSION_KEY>
```
Tests: `python3 bench/tests/test_ui.py` (needs the host builds above + `make -C iter2-autonomy/mission host`; free ports).

- **`plugins/robot.py`**: the only robot connection. sim/usb: direct TCP (`WHO bench`); pi:
  `mission.py`'s `RelayLink` (MOTOR/GOTO/VOICE/WATCH). Replies are matched FIFO on OK/ERR lines
  (tars_voice's JSON event lines go to `listen("voice", fn)`). `send(dest, line, data=b"",
  timeout=None, src=)`: `data` is written right after the line (`AUDIO <n>` + n bytes of 16 kHz
  PCM or WAV, as mission.py does; the relay's VOICE channel is a byte pipe, ≤ 1 MB up); the
  `cmd` event carries `bytes: n`, never the payload. STATUS polled at 10 Hz (`status`), goto
  STATUS ~3 Hz + WATCH EVENTs (`goto`). Polls and move keepalives are not `cmd` events.
  Keepalive: STATUS doesn't feed tars_motor's watchdog, so while a MOVE/TURN **we** started runs,
  the poller sends PING. Manual MOVE/TURN: CANCEL goto first **only if goto is busy** (goto answers
  CANCEL with S, which would abort the move), then wait for that S to land.
- **Drive** (Drive page; the chart and console are `drive-debug` on the Tests page): hold W A S D / arrows,
  or press the on-screen pad → `POST /api/robot/keys` (teleop.py's `HoldDrive` on the
  server, D at 20 Hz, S on release; the tab refreshes held keys every 100 ms and the server sends
  S if it stops for 0.35 s). Tab hidden/blurred → release. Chart: commanded l/r, tars_motor
  out_l/out_r, and the virtual car's `wl`/`wr` (topic `car`, m/s ÷ `dc_mm_s`) when present.
  **Guard BLOCKED** (obstacle in the LiDAR guard's forward corridor, e.g. parked at the bin after
  GOTO): tars_motor refuses every plain `D`, even reverse. The bench then sends Andrew's keyboard
  drive `KD <TARS_KEYBOARD_AUTH> <key>` (one key, fixed 0.25 duty, 150 ms lease; tars_motor takes
  it only for a GOB1-tagged obstacle) for a single held **S, A or D**; **W needs Shift** (body
  `{"held": [...], "speed": s, "shift": true}`). run_sim makes the key per run and gives it to
  tars_motor and the bench (sim/usb only, never the relay); it is redacted from every event. The
  sim guard tags obstacle blocks GOB1 like the robot (`guard_gob1`, lidar_guard
  `--identify-obstacles 1` in service.py). `status` events and `GET /api/robot/state` carry
  `guard: {state: clear|blocked|stale|missing|invalid|off|unknown, reason, override, help}`;
  `help` is plain words for the UI (`""` = nothing to explain), shown in the Drive panel's hint
  line. `status.drive.override` = the held key is going as KD right now.
- **Status**: every STATUS field grouped; changes flash; "STALE" after 1 s without STATUS.
- **Scenarios** (`plugins/scenarios.py`, one decorated function each): link, move_1000, turn_90,
  turn_-90, turn_slow, stop_latency, watchdog, stop_mid_move, wheel_direction, speedcal_move /
  _spin / _slow (wheel_cal.py's flow, asks before saving), tip, bump, goto_trash. Mapped to
  `docs/PI_TEST.md` §2b / Step 5 in the panel. `c.where` = `sim-car` (car truth flowing:
  measures against truth), `stub` (only command/safety paths; tip/bump via the stub's `SIM`
  lines, which tests tars_motor, not the firmware), `real` (measurements are prompts). What
  can't run there SKIPs with the reason. STOP aborts a run; every run ends with `S`.
  Car faults: `car` plugin's `control(msg)` / `send_control(msg)` if it has one, else UDP JSON
  to the car control port (5591, or the plugin's `CONTROL_PORT`).
- **Recorder**: every event (`bench.tap`) → `bench/recordings/*.jsonl` (gitignored), on from
  start. Replay re-publishes with timing, data tagged `_replay: true`; **it never sends to the
  robot** (only `bench.publish`; the test spies the wire), pauses the live status/goto publish,
  isn't re-recorded, and shows a banner. Stop the sim first for clean replays of other parts' topics.

## iPhone 3D map (`phone/`)

Andrew's mapping (`mapping.SceneFeed` + `Map`) and room view run unchanged on the laptop.
The source can be the live iPhone on this Mac's USB (stdlib usbmux client, no install), a
recording, or a ray-cast sim room. The sim room follows topic `world` live (rooms in the
`bench/world/rooms/*.json` shape), and falls back to a built-in room when no world is loaded.
The sim takes its pose from topic `car`. `/api/phone/*` and topic `map3d` are documented in
`plugins/phone.py`; details, frames and the live-phone checklist are in `phone/README.md`.
Test: `python3 bench/tests/test_phone.py`.

## Mission control (`plugins/mission.py`, `web/panels/mission.js`)

The bench is the one laptop UI. For the real robot:
```sh
MISSION_KEY=<key> python3 bench/bench.py --target pi --relay <pi-ip>   # or double-click bench/mission.command
```
The key is required (as in mission.py). The flags match mission.py's: `--speed`, `--laptop-stt`
(`--stt-cli`, `--model`). `$TARS_HOST` is the default relay, then `tars.local`.
`mission.command` takes the key from `$MISSION_KEY` or `~/.tars_mission_key`, or asks for it.

`iter2-autonomy/mission/laptop/mission.py` still works unchanged; it is not retired yet. Andrew owns
it, and the bench **imports** it:
- `mission.State` parses the WATCH/CAM streams.
- `Mission.goto_cmd/memory_say/hear/audio` run unchanged with an adapter that sends through the
  robot plugin.
- `follow_actions.py` and the `.command` ssh lines run as written.

| mission.py (8765) | bench (8770) |
|---|---|
| STOP / Space (S then CANCEL) | shell header STOP / Space → `robot/stop` |
| hold WASD, speed, CANCEL-if-busy, quiet-tab stop | Drive panel → `robot/keys` |
| `/state` 5 Hz SSE | topics `mission` (5 Hz), `mission_grid`, `mission_phone`, plus `status`, `goto` |
| `/cam.mjpg` + YOLOX/detect boxes | Camera panel, `GET /api/mission/cam.jpg` (polled per new frame id) |
| safety verdict, gate, RESET tip | Safety panel, `POST /api/mission/reset` |
| goto table, GOTO/RECALL/FORGET/FORGET all/SCAN/CANCEL/MEMORY SAY, event log | Memory page: Go-to controller + memory table (Forget, Forget all), `POST /api/mission/goto`, `memory_say` |
| object memory, phone table | Memory page (memory table), Mission page (Phone link) |
| map (NAV GRID / phone grid, route, pose, objects) | Map (goto) panel |
| voice: 🎤 hold / T, text box, voice log | **Talk to TARS** box (Drive page, via the voice lab + Send to robot; 🎤 / **T**), and the Robot voice log panel (Mission page: 🎤 hold / **V**, `text`, `audio`) |
| link pills, Pi service pills (relay R) | Pi panel |
| `actions.command` (journals over SSH) | Pi panel "follow journals", `GET/POST /api/mission/journal`, topic `journal` |
| `room-view.command` (SSH tunnel to 5580) | Pi panel "room view tunnel" + link / iframe, `GET/POST /api/mission/roomview` |

On sim/usb the same panels work. The bench polls goto at the relay's rates (DETS 5 Hz; MEMORY, NAV,
NAV ROUTE, NAV GRID 1 Hz), and tars_voice events come from a WATCH connection. There is no camera
in sim/usb. SSH runs with `BatchMode=yes` (it needs an ssh key and never prompts); `BENCH_SSH`
replaces the ssh binary (tests). The default `user@host` is read from Andrew's `actions.command`
(`$TARS_SSH` overrides it). Port 5580 on the laptop is Andrew's room-view tunnel, not a bench port.

Test: `python3 bench/tests/test_mission.py` runs mission's fake Pi (`demo_stack.py` pieces: real
tars_motor stub, goto, tars_voice, mission_relay with a key, fake camera, phone tee, detections) and a
fake ssh, then checks every row above through the bench's HTTP API, the sim and usb targets, and
kill -9 while driving. **Not run:** the real Pi, the relay over Wi-Fi, the robot's camera, real ssh.

## Whole bench end to end (`tests/test_e2e.py`)

`python3 bench/tests/test_e2e.py [--only scenarios,goto,route,stop,voice,box,collide,phone,moved,detour]`
runs `run_sim.py --free-ports --no-build` (virtual SM11, real host tars_motor/goto/tars_voice, world sim,
bench) and drives it only through the bench's HTTP API, like the browser. Build first:
`make -C bench/car && make -C iter1-wasd/pi host && make -C iter2-autonomy/goto host` (+ voice host and
model for the voice part). About 7.5 min. It runs scenarios in bring-up order (link, speedcal_* saved to the
**virtual** Uno's EEPROM, then the rest), car scenarios in room `empty`, GOTO in `default` and
`demo_moved_bin`, STOP mid-GOTO, voice lab send → GOTO, a box dropped mid-route, reversing into a dropped
box, the 3D map following the room, and `demo_detour`.

Glue notes (as merged):
- Only the world sim binds 5590. It copies truth to `BENCH_CAR_TRUTH`, which the car plugin binds (no
  `{"sub"}` then), so the stub car's truth reaches the bench too. Stub truth (`"stub": true`) and
  `alive: false` frames are never used by scenarios as measurements.
- Car faults from scenarios go through `plugins/car.py` `control(msg)`, so they follow `BENCH_CAR_CTL`.
- `run_sim.py` also passes the phone part's ports (`TARS_BENCH_PHONE_ROOM_PORT`/`_SCENE_PORT`), so a
  `--free-ports` stack never touches 5594/5595. Logs append (goto restarts on every room load).
- STOP (S + CANCEL) makes an **idle** goto send its own S, which holds other clients' motion for 1.5 s.
  Wait that out before a scripted MOVE/TURN after STOP (the robot voice path has the same issue).
- Under heavy load the Python world sim can miss tars_motor's 100 ms guard TTL (`guard_stale`): moves
  abort. That's the sim, not the robot; rerun.

## Map demos (`maps/`, SIMULATION)

Two demos of "build a map, then go find stuff", with the real tars_motor + goto host builds on
the virtual car (`python3 bench/run_sim.py`, panel **Map demos (SIM)**):
- **A. Explore live.** goto starts knowing nothing. Scan / Explore fills goto's map only from what
  the sim LiDAR sees (drawn under the 2D view), and the 3D view fills with splats. Then GOTO uses
  the map it just built.
- **B. Prescan** (simulation only). A saved map (mode A's result, an iPhone recording, or Andrew's
  map save) becomes the sim room (walls from its occupied cells), goto's prior map and memory, and
  the 3D backdrop. Drop the car anywhere, then GOTO.

Details, frames and what the real robot would need: `maps/README.md`. Test: `python3 bench/tests/test_maps.py`.

## Testing honesty

Say exactly what ran: the sim, a recording, the real Uno on USB, the iPhone. The bench
proving something in sim does not mean the car does it.
