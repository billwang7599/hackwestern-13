# bench/world/: the simulated room and the phone/webcam sensors in it

**SIM, laptop only.** Nothing here runs on the robot or is needed by it. It exists so the
real `tars_motor` and `goto` host builds can be driven, debugged and demoed against a room
before (or instead of) the real car. A result here is a sim result, not a robot result.

| File | What |
|---|---|
| `worldsim.py` | the world sim process: room, LiDAR guard, phone LDR1 feed, YOLO detections, world control |
| `stub_car.py` | **STUB CAR** fallback (no physics): turns `tars_motor -b stub`'s dead-reckoned pose into car truth |
| `common.py` | rooms, ray casting, frame conversions, goto memory seeding (shared by run_sim and the plugin) |
| `rooms/*.json` | layouts. Saving from the 2D view writes here |
| `../run_sim.py` | one command for the whole sim stack + bench |
| `../plugins/world.py`, `../web/panels/world2d.js` | the 2D view and course editor in the bench |

```sh
python3 bench/run_sim.py                       # real port numbers, room "default", then open the bench URL
python3 bench/run_sim.py --room demo_detour     # any room in rooms/
python3 bench/run_sim.py --free-ports           # ephemeral ports (tests, several stacks at once)
python3 bench/tests/test_world.py               # end to end: GOTO, moved bin, dropped box, guard faults
```

## Room format (`rooms/<name>.json`)

World frame (same as `bench/README.md`): metres, x = the car's forward at its start pose, y = left,
z up, angles in radians counter-clockwise.

```json
{
  "name": "default",
  "about": "free text shown in the bench",
  "start": {"x": 0.0, "y": 0.0, "th": 0.0},
  "wall_h": 2.0,
  "walls": [[x1, y1, x2, y2], ...],
  "objects": [
    {"id": "bin",   "label": "trash can", "x": 2.4, "y": 0.0, "r": 0.15, "h": 0.40},
    {"id": "chair", "label": "chair", "x": 0.9, "y": 1.45, "w": 0.45, "d": 0.45, "h": 0.85, "yaw": 0.0},
    {"id": "door",  "label": "door", "x": 3.6, "y": -1.0, "w": 0.04, "d": 0.8, "h": 2.0, "solid": false},
    {"id": "obstacle1", "label": "", "x": 1.9, "y": 0.0, "w": 0.35, "d": 0.9, "h": 0.5}
  ],
  "memory": [{"id": "bin"}, {"id": "bin", "x": 2.2, "y": -0.4}]
}
```

- **walls**: segments, height `wall_h`. In world control and the plugin, wall `i` has id `"w<i>"`.
- **objects**: a cylinder (`r`) or a box (`w` along its own x, `d` along its own y, rotated by `yaw`),
  `h` tall, at centre `x, y`. `label` is a canonical name from `iter2-autonomy/goto/objects.txt`
  (YOLO sees it), or `""` for a plain obstacle YOLO ignores. `"solid": false` = LiDAR and the car pass
  through it (a doorway), YOLO still sees it. A person is `{"label": "person", "r": 0.2, "h": 1.7}`.
  Ids are unique; the editor names new ones `obstacle1`, `person1`, `trash_can1`…
- **memory** (optional): what goto remembers when the room starts, as if it had SCANned earlier.
  `{"id"}` = that object where it is; `{"id", "x", "y"}` = remembered somewhere else (the moved-bin demo).
  run_sim writes it into goto's memory file (`common.goto_memory_text`, the same format as
  `test_fullstack.mem_file`); loading a room from the bench restarts goto with it.

Shipped layouts: `default` (go to the trash can around a wall), `demo_moved_bin` (remembered at A, now
at B), `demo_detour` (drop a box on the route while it drives), `open_bin` (visual-only GOTO), `empty`.
Rooms must stay small (world state is a UDP datagram; macOS caps those at 9216 bytes): a few dozen
walls and objects is fine.

## World control (port 5592: TCP JSON lines, or UDP datagrams; one JSON reply each)

Every edit re-casts the guard rays, detections and occlusion, the phone's depth points (goto's map),
and **resends the car's collision walls** (5591 `{"walls": [...]}`: every wall + every solid object's
footprint, a box as 4 segments, a cylinder as an 8-gon). Walls are also resent every 2 s in case
the car restarted.

| Request | Reply |
|---|---|
| `{"op": "ping"}` | `{"ok", "rev"}` |
| `{"op": "get_room"}` / `{"op": "get_state"}` / `{"op": "rooms"}` | `{"ok", "room"}` / `{"ok", "state"}` / `{"ok", "rooms"}` |
| `{"op": "add", "obj": {...}}` (object as in the room format; `"kind": "person"` fills person defaults) | `{"ok", "id", "rev"}` |
| `{"op": "add_wall", "seg": [x1, y1, x2, y2]}` | `{"ok", "id": "w<i>", "rev"}` |
| `{"op": "update", "id", "x"?, "y"?, "yaw"?, "w"?, "d"?, "r"?, "h"?, "label"?, "solid"?}` (`"move"` = same) | `{"ok", "rev"}` |
| `{"op": "update", "id": "w3", "seg": [...]}` | `{"ok", "rev"}` |
| `{"op": "remove", "id"}` (object or `w<i>`) | `{"ok", "rev"}` |
| `{"op": "load", "name"}` or `{"op": "load", "room": {...}}` | `{"ok", "rev", "goto_memory"}` (car teleported to start, faults cleared) |
| `{"op": "reset"}` | reload the room as loaded/saved: same reply as load |
| `{"op": "save", "name", "overwrite"?, "about"?}` | `{"ok", "path"}` or `{"ok": false, "exists": true}` |
| `{"op": "set", "params": {...}}` (shorthand `{"set": {...}}`) | `{"ok", "params"}` |
| `{"op": "fault", "name", "on"?: true, "s"?: seconds}` (shorthand `{"fault": name}`) | `{"ok"}` |
| `{"op": "teleport", "x", "y", "th"}` / `{"op": "car", "msg": {...}}` | passed to the car's 5591 |

Removing walls renumbers the ones after it (ids are positions).

**Faults:** `person` (a person appears `dist` 0.45 m in front of the car: a real object in the room, so
guard, map and YOLO all see it), `guard_silent` (no GAT1), `guard_invalid` (malformed GAT1),
`guard_stale` (GAT1 with 300 ms old stamps), `pose_stale` (phone feed stops), `pose_lost` (LDR1 with
tracking flags 0; lidar_guard then can't permit either), `pose_frozen` (the phone keeps sending its last
pose), `reloc` (one-shot: the ARKit frame jumps `reloc_m`/`reloc_deg` and stays jumped), `false_det`
(a phantom detection, `label`, `u` = 0..1 across the frame), `cam_off` (no detection frames), `clear`.
Moving an object live is just `update` (the moved-bin case).

**Params** (live, all in `worldsim.DEFAULTS`): pose noise/latency/dropouts, phone rate and height,
LiDAR FOV/range/rays/noise, guard rate/stop distance/half width/GOB1, camera rate/latency/height,
detection range, confidence near/far, miss and false-positive rates, box jitter.

## World state (port 5593: UDP JSON, to the bench)

`{"type": "room", "rev", ...the room..., "labels", "hfov_deg", "params"}` on every change and every 2 s, and
`{"type": "state", "rev", "room": <name>, "truth", "phone", "guard", "dets", "hfov_deg", "lidar_hfov_deg",
"faults", "reloc", "counts"}` at 10 Hz:
- `truth`: the car's pose `{x, y, th, v, w, contact, stub, age_ms}`
- `phone`: what goto is told `{x, y, th, tracking, age_ms, points}` (world frame)
- `guard`: `{verdict: CLEAR|CAUTION|BLOCKED|UNKNOWN|STALE|INVALID, permit, reason, sent, nearest, bands, rays: [[angle_rel, dist, hit_id], ...], pose: [x, y, th]}`
- `dets`: what YOLO reported, `{id, label, conf, x, y, vis}` or `{label, fp: true, bearing}`
- `counts`: health (`gat`, `ldr1`, `det_frames`, `gat_max_gap_ms`, `sim_guard_late` and `sim_stale_ticks`
  = times the SIM fell behind tars_motor's 100 ms guard TTL; not 0 means a stop was the sim's fault).

The bench plugin republishes this as topic **`world`**, with the **full current room** under `"room"`
in every event (walls + all objects, including everything placed live), plus `route` (goto's planned
path in the world frame), `goto` (its state) and `car_kind`.

## What it models (and how)

All sensors look at the **true** pose (from the car), delayed by their latency:
- **Guard** (to tars_motor `-g`, real port 5568): `lidar_guard.cpp`'s rules on the same depth points
  the phone sends goto: obstacle points with |side| < 0.45 m and height 0.08–1.2 m within 0.6 m ahead
  → BLOCKED; CLEAR needs ≥ 15 points and ≥ 2 floor hits in each of three lateral bands 0.15–1.5 m ahead;
  else CAUTION; permit on CLEAR or CAUTION; STALE if the newest frame is ≥ 150 ms old; tracking must be
  OK (flags 7). GAT1 at 100 Hz (GOB1 for obstacle denials if `guard_gob1`).
- **Phone** (to goto `-n`, real 5572): LDR1, TCP length-prefixed, 20 Hz, `lidar_rays` rays over a 60°
  FOV to 5 m, floor points 0.25–3 m, obstacle points at 0.1/0.3/0.6/1.0 m up to the object's height.
  Pose and points are in the ARKit frame (`x_ark` = left, `z_ark` = forward, yaw CCW, origin = the
  room's start pose) **as the phone believes it**: truth + relocalisation offset + noise, so a
  noisy pose also smears the map, as on the robot.
- **YOLO** (to goto `-u`, real 5571): 8 Hz, 640×480 pinhole with goto.conf's `hfov_deg`, a level camera
  `cam_h` up; occlusion by 5 rays across each object (walls and other objects block); confidence by
  distance × visible fraction; misses, false positives, jitter.
- **Car**: the virtual SM11 (`bench/car/`) when present. It reads walls from `{"walls"}` on 5591, sends
  truth to 5590. Else the **stub car**: tars_motor `-b stub` (stepper timing, `WHEELCAL 260 400` ≈
  163 mm/s, about the SM11's cap), truth = its dead reckoning, stopped at walls (`contact`). It has no
  physics, no drift and no bumps, and tars_motor's pose never disagrees with the truth except at a wall.

Not modelled: camera blur and lighting, LiDAR on glass or black surfaces, vertical FOV, phone mount offset,
CPU load on the Pi.
