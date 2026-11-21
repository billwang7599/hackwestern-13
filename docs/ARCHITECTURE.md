# TARS architecture: voice + YOLO + iPhone LiDAR, QNX native

This is the design for the whole robot. It covers how the three sensing layers split the work, the feature list with honest status, the one behaviour state machine, pose, ports, CPU and what has actually run.
Rules for changing code are in [AGENTS.md](../AGENTS.md). Status and priorities are in [PLAN.md](../PLAN.md).
Everything here lives in `iter2-autonomy/` (voice, goto, iphone, edge-ai, dashboard, iphone-usb) on top of `iter1-wasd/pi/` (`tars_motor`); see [MOVED.md](MOVED.md).

```
 USB mic        C270 / phone RGB            iPhone 17 Pro (ARKit + LiDAR, USB)
    │                 │                               │
    ▼                 ▼                               ▼
 voice (C)       detect (C++, YOLO-World)       phone feeds 5561 / 5562
 INTENT          RECOGNITION + DOCKING          GEOMETRY
 "go to the bin" label, bearing, box height     pose, occupancy grid, obstacles
    │ GOTO/SCAN/…     │ JSON dets                     │ LDR1 / world obs
    └────────────►  goto (C)  ◄───────────────────────┘ (via iter2-autonomy/goto/nav.c)
                    object memory + nav + behaviour state machine
                        │ TURN / MOVE / DL / S / BEEP / LED   (TCP 5555)
                        ▼
      lidar_guard (C++, SCHED_RR) ──GAT1 5568──► tars_motor (C, SCHED_RR 30)  ◄── SAFETY FLOOR
                                                     │ USB serial 115200
                                                     ▼
                                                 Uno: steppers, tip/bump, 200 ms watchdog
```

Everything in that picture runs on the Pi 5 under QNX 8.0 in C/C++. Andrew's Python dashboard and `mapping.py` are desktop/diagnostic visualization only (§6).

## 1. Sensor-fusion contract

Each layer answers one question and nothing else.

| Layer | Answers | Owns | Does **not** do |
|---|---|---|---|
| **Voice** (`iter2-autonomy/voice/`) | *What* to do, and *to which object* | intents, wake word, the stop-first safety pass | steering, geometry |
| **YOLO** (`iter2-autonomy/goto/detect`, YOLO-World) | *Which pixels are the object*, how centred, how close | label, confidence, fine bearing (box centre), arrival (box height ≥ `arrive_h_frac`) | metric position, obstacles |
| **iPhone LiDAR + ARKit** | *Where am I, where are the walls, where is the object in the room, is it hidden or gone* | robot pose, 10 cm occupancy grid, object world (x,z), line-of-sight checks | recognising objects, deciding arrival |
| **tars_motor + lidar_guard** | *May the wheels move right now?* | guard gate, watchdog, tip/bump, S-wins | anything above |

**The fusion point is the object-memory entry** (from `kothar/object-memory`, `goto` side):

```
instance  = label#id                      # from YOLO   ("trash can#2")
world     = (x, z) at the sighting        # from ARKit pose + bearing + range (dead-reckoning fallback)
status    = fresh | stale | gone          # from re-observation (YOLO) and line of sight (map)
bearing   = world bearing of last sighting
dist, conf, seen count, last_seen
```

**The map gets the robot near; YOLO confirms and docks.** The planner drives to a 0.6–0.85 m approach ring around the remembered `world`, then the existing visual servo takes over for the last metre. Nothing is ever declared ARRIVED on the map alone.

## 2. Feature set

| | Feature | Status | Where |
|---|---|---|---|
| A | WASD teleop + voice manual drive (`MOVE`/`TURN`) | **built** | `iter1-wasd/laptop/teleop.py`, Pi keyboard (`iter1-wasd/pi/MOTOR_GUARD.md`), `iter2-autonomy/voice/` |
| B | Voice wake-word intent layer, stop-first | **built**, incl. `scan`/`forget`, `memory` ("what do you remember" → `MEMORY SAY`, printed + `BEEP n` + `SAY` to the phone on 5573) and `recall` ("go back to where the X was" → `RECALL`) | `iter2-autonomy/voice/` |
| C | YOLO-World recognition, 10 canonical labels + aliases | **built** | `iter2-autonomy/goto/detect`, `iter2-autonomy/goto/objects.txt` |
| D | Visual-servo GOTO: search / align / approach / arrive | **built** | `iter2-autonomy/goto/src/goto_core.c` |
| E | ARKit pose + LiDAR occupancy grid | **built** in C (`iter2-autonomy/goto/nav.c`, LDR1 feed on 5572); Andrew's Python stays the preview | `iter2-autonomy/goto/nav.[ch]` |
| F | A* with robot-radius inflation + 0.6–0.85 m approach ring | **built** in C, parity-identical to `mapping.py` (212/212) | `iter2-autonomy/goto/nav.[ch]` |
| G | Persistent multi-instance object memory (fresh/stale/gone) | **built**, now with map positions (`navw`) | `iter2-autonomy/goto/memory.txt` |
| H | **Map-guided GOTO**: route to the remembered spot, then YOLO-dock | **built** (Mac/sim only) | `goto` state machine, §3 |
| I | Obstacle avoidance: lidar_guard hard gate + occlusion/detour | guard **built** (Andrew); OCCLUDED/DETOUR, re-plan on map change/guard **built** | `iter2-autonomy/edge-ai/`, `goto` |
| J | SCAN: 12 × `TURN 30`, fill memory (and map) | **built** on object-memory | `goto` |
| K | Safety floor: guard gate (5568), 500 ms watchdog, tip latch, bump hold, 0.6 m guard stop zone, `S` wins | **built** | `iter1-wasd/pi/src/main.c`, `iter2-autonomy/edge-ai/lidar_guard.cpp` |

## 3. State machine (two levels)

### 3.1 Top-level mode

One mode at a time, chosen by voice, the Uno button or teleop. Teleop and voice manual moves always send `CANCEL` to `goto` first.

| Mode | Entered by | Who drives |
|---|---|---|
| `IDLE` | boot, ARRIVED, FAILED, any stop | nobody |
| `TELEOP` | WASD key, phone D-pad, voice `MOVE`/`TURN` | the teleop client / voice |
| `GOTO` | "TARS, go to the X", "go back to where the X was" | `goto` |
| `SCAN` | "TARS, look around" | `goto` (12 × `TURN 30`) |
| `FOLLOW` | *parked* (MediaPipe, back burner) | — |

### 3.2 GOTO behaviour (C, inside `goto`)

```mermaid
stateDiagram-v2
    [*] --> RECALL : GOTO obj
    RECALL --> PLAN : usable instance + map + pose ok
    RECALL --> LOOK : instance, no map (visual fallback: MEM_TURN)
    RECALL --> LOOK : no instance (look before turning)
    PLAN --> FOLLOW_PATH : path found
    PLAN --> NEXT_CANDIDATE : NO_GOAL / NO_PATH
    FOLLOW_PATH --> FOLLOW_PATH : leg done, more waypoints
    FOLLOW_PATH --> PLAN : grid version changed / guard denied leg
    FOLLOW_PATH --> LOOK : at approach point (TURN to face spot)
    LOOK --> ALIGN_TURN : seen, |bearing| > 12 deg
    LOOK --> APPROACH : seen, centred
    LOOK --> OCCLUDED : not seen, line of sight blocked
    LOOK --> CHECK_OTHERS : not seen, line of sight clear
    LOOK --> SEARCH_TURN : not seen, no memory left
    OCCLUDED --> DETOUR : vantage cell exists
    OCCLUDED --> CHECK_OTHERS : no vantage cell
    DETOUR --> PLAN : plan to vantage cell
    CHECK_OTHERS --> PLAN : another same-label instance
    CHECK_OTHERS --> NEXT_CANDIDATE : none left
    NEXT_CANDIDATE --> RECALL : tries left
    NEXT_CANDIDATE --> SEARCH_TURN : tries exhausted
    SEARCH_TURN --> LOOK : turn done + settle
    SEARCH_TURN --> FAILED : 360 deg, nothing
    ALIGN_TURN --> APPROACH : turn done
    APPROACH --> ALIGN_TURN : |bearing| > 20 deg
    APPROACH --> SEARCH_TURN : lost > 1.2 s
    APPROACH --> ARRIVED : h >= arrive_h_frac, 2 frames
    ARRIVED --> [*]
    FAILED --> [*]
    note right of RECALL : CANCEL, S, guard, tip, bump or watchdog from ANY state wins (S, IDLE)
```

**States**

- **RECALL**: resolve the label to the best known instance: fresh before stale, then nearest; skip `gone` and anything older than `memory_max_age_s`. "Go back to where the X was" skips the visibility requirement and goes straight to PLAN with the stored world position.
- **PLAN**: `nav_plan(pose → approach ring around instance.world)`. Output: simplified waypoints, last one on the 0.6–0.85 m ring.
- **FOLLOW_PATH**: each waypoint is an exact `TURN <deg>` then `MOVE <mm>` leg. Re-plan if `nav_version()` changes, or if a leg is aborted by the guard (§3.4).
- **LOOK**: standing still, check 4 fresh frames (2 hits). This is the YOLO confirmation at the spot.
- **ALIGN_TURN / APPROACH / ARRIVED**: unchanged from the built visual servo (`iter2-autonomy/goto/README.md`): one `TURN 0.9 × bearing`, then 10 Hz `D` P-control, arrive at box height ≥ 70% (person 80%) for 2 frames.
- **SEARCH_TURN**: blind `TURN 30` steps up to 360°, starting toward the last-seen side.
- **OCCLUDED**: `nav_line_clear(robot, instance.world) == 0` (an occupied cell between us and the spot). The object may still be there; don't downgrade it.
- **DETOUR**: pick a free cell on the ring with clear line of sight to the spot (vantage cell), PLAN to it, LOOK again. One detour per instance.
- **CHECK_OTHERS**: line of sight was clear and YOLO saw nothing, so mark the instance fresh → stale → gone and try the next instance of the same label.
- **NEXT_CANDIDATE**: count tries (`memory_tries`, default 3), then fall back to SEARCH_TURN.
- **FAILED**: `S`, `BEEP 3`, `EVENT failed:<why> <object>`.

These states plug into `look_failed()` and the exact-move branch of `goto_sm_tick()` on the object-memory branch, which were left as the extension point.

### 3.3 Transition table

`(mm)`/`(deg)` are computed from the current pose. All actions go to `tars_motor` on TCP 5555.

| State | Event | Condition | Next | Action to `tars_motor` |
|---|---|---|---|---|
| any | `GOTO obj` | obj in `objects.txt` | RECALL | `BEEP 1` |
| any | `GOTO obj` | unknown obj | IDLE | `LED no`, `BEEP 3` |
| RECALL | instance found | map ready, pose ok | PLAN | — |
| RECALL | instance found | no map / no phone | LOOK (via MEM_TURN) | `TURN (deg)` to instance bearing |
| RECALL | none | — | LOOK | — |
| PLAN | `nav_plan ≥ 0` | — | FOLLOW_PATH | — |
| PLAN | `NAV_ERR_START_BLOCKED` | — | SEARCH_TURN | `TURN 30` (reveal map) |
| PLAN | `NAV_ERR_NO_GOAL` / `NO_PATH` | — | NEXT_CANDIDATE | — |
| PLAN | `NAV_ERR_NO_MAP` | — | LOOK (visual fallback) | `TURN (deg)` to instance bearing |
| FOLLOW_PATH | leg start | — | FOLLOW_PATH | `TURN (deg)`, then `MOVE (mm)` |
| FOLLOW_PATH | `DONE` | more waypoints | FOLLOW_PATH | next leg |
| FOLLOW_PATH | `DONE` | last waypoint | LOOK | `TURN (deg)` to face the spot |
| FOLLOW_PATH | `nav_version` changed | path now crosses a blocked cell | PLAN | `S` |
| FOLLOW_PATH | `ABORT lidar_guard` / guard epoch bumped | — | PLAN (after guard CLEAR) | none (already stopped) |
| LOOK | seen (2/4) | \|bearing\| > 12° | ALIGN_TURN | `TURN 0.9×bearing` |
| LOOK | seen (2/4) | centred | APPROACH | `D l r` @ 10 Hz |
| LOOK | not seen | `nav_line_clear == 0` | OCCLUDED | — |
| LOOK | not seen | line clear or unknown | CHECK_OTHERS | — (instance → stale/gone) |
| LOOK | not seen | no memory | SEARCH_TURN | `TURN 30` |
| OCCLUDED | vantage cell | not detoured yet | DETOUR → PLAN | — |
| OCCLUDED | no vantage cell | — | CHECK_OTHERS | — |
| CHECK_OTHERS | next same-label instance | — | PLAN | — |
| CHECK_OTHERS | none | — | NEXT_CANDIDATE | — |
| NEXT_CANDIDATE | tries left | — | RECALL | — |
| NEXT_CANDIDATE | tries exhausted | — | SEARCH_TURN | `TURN 30` |
| SEARCH_TURN | `DONE` + `settle_s` | — | LOOK | — |
| SEARCH_TURN | 360° done | — | FAILED | `S`, `BEEP 3` |
| ALIGN_TURN | `DONE` | — | APPROACH | `D l r` |
| APPROACH | \|bearing\| > 20° | — | ALIGN_TURN | `S`, `TURN` |
| APPROACH | target lost > 1.2 s | — | SEARCH_TURN | `S` |
| APPROACH | h ≥ `arrive_h_frac`, 2 frames | — | ARRIVED | `S`, `BEEP 1`, `LED ok`; memory rewritten from pose |
| any | timeout 240 s | — | FAILED | `S`, `BEEP 3` |
| **any** | **`CANCEL`, or `stops=` counter rose** | — | **IDLE** | **`S`** |
| **any** | **tip / bump / watchdog / guard deny** | — | per §3.4 | (`tars_motor` already zeroed) |

### 3.4 Safety-floor overlay

The floor lives in `tars_motor`'s 50 Hz loop and pre-empts every `goto` state. `goto` can't override it, only react.

| Trigger | What `tars_motor` does | What `goto` does |
|---|---|---|
| `S` from any client (voice "stop", teleop, phone) | zero wheels, abort MOVE/TURN, hold others' `D` 1.5 s | sees `stops=` rise → IDLE |
| guard deny (BLOCKED in 0.6 m corridor, UNKNOWN, STALE > 150 ms, gate heartbeat > 100 ms) | zero wheels, abort MOVE (`ABORT lidar_guard`), `guard_epoch++`. **As built:** an in-place `TURN` still runs/is accepted while the reason is a fresh `guard_blocked` (never when stale/missing/invalid) | FOLLOW_PATH: re-plan once CLEAR (or after 3× the wait while BLOCKED: the new route starts with a pivot). Face turn refused → LOOK. APPROACH: the leg ends; target centred and big → arrived, else LOOK again. Old `DL` refused. |
| 500 ms watchdog | zero, abort exact move | `goto` keeps feeding it (`D` at 10 Hz, `PING` during exact moves) |
| tip latch | zero, latched until `RESET` | FAILED, `BEEP 3` |
| bump | zero, refuse drive 1 s | FAILED, `BEEP 3` |
| `goto` dies / disconnects | stop whatever that client started | — |
| Uno silent 200 ms | Uno stops on its own | — |

**Leased drive (built).** APPROACH sends `DL <guard_epoch> l r` with the epoch read from `STATUS` and fixed for the leg. A guard denial bumps the epoch, so a stale drive from before the obstacle stays refused even after the guard clears. The leg ends, and a new one starts only after a fresh LOOK. Path legs are exact `TURN`/`MOVE` (aborted by the guard directly).

The guard checks a **forward** corridor only. Pivots and reverse are not covered, so the map's inflation (35 cm robot radius, kept) is what keeps `TURN` legs clear, and `goto` never plans a reverse `MOVE`. Because the 0.35 m inflation is smaller than the ~0.6 m stop zone, legs can end facing something inside the zone. That's why pivots are allowed under a fresh BLOCKED (see `iter2-autonomy/goto/README.md` assumption 7; **to agree with Andrew**).

**Pure visual GOTO stays the fallback.** With no phone, no map, or no pose, `goto` behaves exactly as it does on `main` today: memory turn (if any), LOOK, SEARCH, ALIGN, APPROACH. The guard gate is then disabled only if `tars_motor` was started without `-g` (bench mode).

## 4. Pose

One pose source at a time, in the ARKit world frame (x, z; y up; yaw along `(sin, cos)`):

1. **ARKit pose** when the phone reports normal tracking + heading (LDR1 flags 1|4) and the last such packet is < `nav_pose_fresh_s` old. **As built: 0.5 s**, not 150 ms (LDR1 is ~20 Hz over USB; tune on the robot).
2. **Dead reckoning** from `tars_motor` `STATUS pose=x,y,hdg` when tracking is limited/lost or the phone is absent: `nav_note_dr(-y/1000, x/1000, -hdg)` every poll; `nav_get_pose()` carries the motion since the last tracked moment into the ARKit frame (the anchor = the world↔motor offset).

As built, a DR → ARKit jump does not trigger a re-plan by itself; the next leg is computed from the new pose, and a changed grid re-plans via `nav_version`. The "one more leg, wait 2 s" rule isn't built.

**Handover rule.** ARKit → dead reckoning: freeze the last good offset, continue on odometry. Dead reckoning → ARKit: take ARKit as truth, recompute the offset, and if the jump is > 0.3 m or > 15°, re-PLAN. While on dead reckoning, FOLLOW_PATH runs at most one more leg and then waits up to 2 s for tracking before falling back to LOOK at the current spot.

**Relocalization risk.** ARKit can relocalize or reset and shift the whole frame, so remembered object positions can jump. New session ID → clear the grid (Andrew's mapping already does this) and mark all instances stale. **Always re-observe with YOLO before ARRIVED**; the map is never sufficient.

The phone-to-robot extrinsics (mount offset, forward axis) are uncalibrated. Until measured, the approach ring margin absorbs it.

## 5. Ports and processes

**Andrew's stack** (from `iter2-autonomy/edge-ai/README.md`, all localhost):

| Port | Purpose |
|---|---|
| 5555 | native motor command server (`tars_motor`) |
| 5557 | phone → Pi motion NDJSON |
| 5558 | optional paired preview JPEG |
| 5559 | Pi → phone status/results; phone commands back |
| 5560 | Pi service administration |
| 5561 | phone → Pi world observations, ≤ 5 Hz |
| 5562 | phone → native guard measured LiDAR (`LDR1`), ~20 Hz |
| 5563 | phone → broker RGB JPEG (`RGB1`) |
| 5564 | native guard → dashboard UDP status |
| 5565 | YOLO worker → dashboard UDP result |
| 5566 | latest-frame broker for AI consumers |
| 5567 | MiDaS worker → dashboard UDP result |
| 5568 | native guard → native motor gate (`GAT1`) |
| 5569 | mapping → native guard, inferred-obstacle veto (`FUS1`) |

**Ours moved** to avoid the clash (5557/5558 are taken). *5570–5572 pending Andrew's sign-off.*

| Port | Old | Purpose |
|---|---|---|
| 5570 | 5557 | clients → `goto` commands (`GOTO`, `RECALL`, `CANCEL`, `SCAN`, `MEMORY [SAY]`, `FORGET`, `NAV`, `STATUS`, `WATCH`, …) |
| 5571 | 5558 | detector → `goto` detection JSON lines (`iter2-autonomy/goto/DETECTIONS.md`) |
| 5572 | new | **as built: phone → `goto`**, LDR1 packets (the 5562 format, TCP length-prefixed or UDP); `goto` listens. A display feed back to the phone isn't built. |
| 5573 | (Andrew) | TARS speech → phone (SpeechLink via iproxy): `SAY <text>` → `OK say`, urgent JSON form, `HUSH`. `tars_voice` is a client. |

**Processes and priority**

| Process | Lang | Priority | Notes |
|---|---|---|---|
| `tars_motor` | C | SCHED_RR 30 | 50 Hz loop, safety floor |
| `lidar_guard` | C++ | SCHED_RR 20, CPU 3 | 100 Hz, independent of all ML |
| `goto` (+ nav, memory) | C | SCHED_RR 20 | nearly idle; A* ~tens of ms per re-plan (Python version: 47 ms on the Pi) |
| `tars_voice` | C/C++ | normal | whisper `threads=2`, only after VAD |
| `detect` (YOLO-World) | C++ | normal | ~5–10 fps est., `--fps 8` |
| MiDaS (`ai_depth`) | C++ | normal | ~0.8 s/frame measured on the Pi |
| YOLOX (`ai_detector`) | C++ | normal | dashboard only |

**Pi 5 CPU budget (estimates, not measured together).** YOLO-World + MiDaS + YOLOX + whisper won't all fit. Mode switching:

| Mode | YOLO-World | YOLOX | MiDaS | Speech |
|---|---|---|---|---|
| IDLE / TELEOP | ~2 Hz (memory) | off | on | VAD |
| GOTO | full | off | **off** (LiDAR guard is enough) | VAD |
| SCAN | full | off | off | VAD |

ML at normal priority means a stop can never be starved, whatever the fps.

## 6. One detector

- **`iter2-autonomy/goto/detect` (YOLO-World, ONNX Runtime) is the only object detector that feeds `goto`.** It's the one that knows "trash can"; COCO/YOLOX doesn't.
- Andrew's **YOLOX** (5565) and **MiDaS** (5567) serve the phone dashboard, depth fusion and the guard's inferred-obstacle veto (5569) only. In GOTO mode they're off (§5).
- Proposed, **not agreed or built**: `detect` reading frames from Andrew's latest-frame broker (5566). As built, `detect` keeps its own camera source (C270 via camapi). Either way the output is the same JSON on 5571.
- **`iter2-autonomy/dashboard/mapping.py` and the Python dashboard are visualization**, not robot runtime. The planner that moves the wheels is `iter2-autonomy/goto/nav.c` (C, same grid, same A*, parity-tested against `mapping.py`).

## 7. Honest status (2026-11-21)

**Run on the real Pi/QNX (Andrew's side, per `iter2-autonomy/edge-ai/README.md`):** YOLOX + MiDaS on QNX ONNX Runtime 1.24.4, `lidar_guard` with synthetic LiDAR, guard → `tars_motor` stub stop tests, live phone feeds to the Pi, a 1500-point mapping benchmark. **No physical drive** has been sent to the wheels.

**Mac only (our side, iteration 2 as built):** `tars_motor` 313/313 (guard gate, DL epoch, pivot-while-BLOCKED, pose), `goto` C unit 247/247 incl. ASan/UBSan, `nav` 84/84 + 1M-packet fuzz under ASan/UBSan, parity vs `mapping.py` 212/212, goto 2D sim 52/52, **scripted full-stack sim 27/27** (real `tars_motor_host` + `goto` + `tars_voice`/whisper; fake phone LDR1, fake YOLO, fake lidar_guard, fake phone speech: obstacle, occlusion → detour, moved object, STOP mid-route, visual-only, voice memory + recall), voice 149/149 (stop recall 1131/1131) and e2e 52/52, Uno 237/259/216/213, teleop 36/36. `detect` with ONNX Runtime on macOS only.

**Never run on the Pi/QNX/iPhone:** `goto`, `detect` (camapi never compiled), `tars_voice`, the object memory, `iter2-autonomy/goto/nav.c`, map-guided GOTO, the 5572 feed (no phone sends to it yet), the 5573 speech from voice. The blocker is still our `qcc` toolchain (Docker + SDP 8.0 in `~/qnx800`); Andrew builds with native QNX clang on the Pi, which may be the quicker route.

**Open items**
- Andrew's sign-off on the port move (§5).
- Who sends LDR1 to 5572 on the robot (the phone directly, or Andrew's service teeing 5562).
- The pivot-while-BLOCKED change in `tars_motor` (§3.4): agree it with Andrew.
- Frame/yaw signs on the real phone mount (`NAV` shows the pose).
- Phone-to-robot extrinsics, `arrive_h_frac` and `cam_height_mm` on the real mount.
- Repo hygiene, to raise with Andrew, not blocking: ~90 MB of committed model blobs (`iter2-autonomy/edge-ai/models/*.onnx` 87 MB, `iter2-autonomy/iphone/Models/YOLOv3TinyFP16.mlmodel`) and `.idea/` are in history.
