# bench/maps: map demo modes (SIMULATION, laptop only)

Two ways to demo "build a map, then go find stuff" on the bench, with the **real** `tars_motor`
and `goto` host builds driving the virtual SM11 in the sim room. Nothing here runs on the robot,
and no robot code was changed for it.

| File | What |
|---|---|
| `prescan.py` | grids (goto's `NAV GRID` run-length code), goto `MEMORY` parsing, prescan files, scan → sim room (walls from occupied cells), goto memory for a drop pose, seed points |
| `explore.py` | the bench-side explorer: `SCAN`, frontier hop, `SCAN`, … until goto remembers the target |
| `../plugins/maps.py` | routes `/api/maps/*`, topics `navgrid` and `maps` |
| `../web/panels/maps.js` | the **Map demos (SIM)** panel; draws goto's map under the 2D view (`window.World2DLayers`) |
| `../world/prescans/*.json` | saved prescans. `sample_default.json` (47 KB) was made by mode A in room `default` |

Start everything with `python3 bench/run_sim.py` and open the bench URL it prints. Forget, Load and
Drop restart goto, so they need run_sim's supervisor.

## A. Explore live: the car starts knowing nothing

1. Build or load a course in **Sim world (2D)**: walls, boxes and a trash can, anywhere.
2. **Map demos → 1. Forget everything**: the same room reloads with no `memory`, goto restarts
   (empty nav map, empty object memory), the iPhone 3D map clears, and the car goes back to the start.
3. **2. Scan (spin)**: goto `SCAN` turns the car 360° in steps and looks. goto's map
   (teal = floor it has seen, orange = obstacles, unshaded = unknown) and the 3D view fill in
   **only from what the sim LiDAR sees from the car**. The world sim ray-casts 25 rays over 60° to
   5 m from the car's true pose, and walls and boxes occlude. The test checks every mapped cell.
4. **3. Explore & find** (target `trash can`, then GOTO): `SCAN` → if goto's `MEMORY` has the
   target with a map position, stop → else pick the nearest **frontier** on goto's own grid (a seen
   floor cell next to unknown, reached through seen floor kept 0.45 m from obstacles, which is
   lidar_guard's corridor; 0.30 m if nothing else is reachable) and drive there with `TURN`/`MOVE` to
   tars_motor (≤ 1.6 m per hop), then `SCAN` again. The dashed magenta line is the hop. When found, it
   sends `GOTO trash can` and goto plans over the map it just built.
   - Stop always wins: the STOP button / Space (S, then CANCEL), **stop explore**, any refused or
     aborted move, or a STOP from anywhere (the robot plugin's stop counter) ends the run, and
     nothing new is sent after that. The guard can refuse a `MOVE`; the explorer then scans from
     where it is and picks another frontier.
5. Or say / type "tars go to the trash can" in the voice lab: same GOTO.

## B. Prescan (SIMULATION ONLY)

1. **Make one**: after mode A (or any run), **Save goto's map as a prescan** (name). That writes
   goto's `NAV GRID`, the objects in goto's `MEMORY` (one per label within 0.6 m: goto keeps
   repeat sightings of one object as separate entries until the fix on `kothar/goto-fixes` 466274c lands)
   and the 3D map's splats to `bench/world/prescans/<name>.json`.
   **Or import a real scan**: a phone3d recording (`bench/recordings/*.jsonl` or
   `bench/phone/recordings/*.jsonl`), replayed offline through Andrew's `mapping.Map`, or Andrew's
   mapper save file (`mapping.Map._saved()`, e.g. `iter2-autonomy/dashboard/state/map.json`).
2. **Load prescan**:
   - **Room**: the sim room is rebuilt from the scan, not copied from the original. Occupied cells
     become wall segments (rows of cells → one segment; ≤ 220, the room must fit a UDP datagram),
     and the virtual car collides with them. The scan's objects become room objects with default
     sizes (YOLO sees them); the cells under them are left to the object.
   - **goto's map**: seeded before the car moves. The world sim's new `seed_nav` op replays one LDR1
     point per mapped cell (free → floor hit, occupied → hit 0.3 m up) on its own phone feed. goto
     has no map-load command, so this is bench-only. The plugin checks that goto's free/occupied
     counts match.
   - **goto's memory**: every object, `navw` in the scan frame and `inst` relative to the drop pose.
   - **3D**: the 3D map is cleared and the prescan's splats go in as the backdrop (press
     **Surface** in Andrew's room view).
3. **Drop car (click in 2D)** + heading: the car goes there, goto restarts (memory relative to
   the drop, `POSE RESET`), and its map is re-seeded. The live LiDAR keeps updating it on top.
4. **GOTO target** (or voice): goto plans over the prescan map from wherever you dropped it.

**Frames.** The sim world frame **is** the scan's ARKit frame. A prescan room starts at the
origin: world x = z_ark (forward at the scan's start), world y = x_ark (left),
z = y − floor_y. Grid layout = goto's `nav.h`: 200 × 200 cells of 0.1 m, corner at (−10, −10).
A real recording's ARKit frame starts wherever the phone started its session.

**What it would take on the real robot.** This is Andrew's area and robot code; none of it exists yet.
- goto (C, QNX) would need a map load/save command (the grid is 40 kB), plus object memory with
  `navw` positions in the same frame. That memory part exists.
- The phone would have to **relocalise** into the scan's frame after a restart: ARKit
  `ARWorldMap` save/load on MotionUSB, or Andrew's AprilTags giving the frame offset. Until then,
  a new ARKit session has a new origin and the saved map lands in the wrong place.
- The map would need checking against fresh LiDAR before trusting it. Moved furniture stays
  occupied until the phone sees floor there for 5 frames over 0.6 s, which nav.c already does.

## What the tests show (and don't)

`python3 bench/tests/test_maps.py` (`--unit` for no stack) on the Mac with the virtual SM11:
- unit: grid code, walls, MEMORY parsing, seed points, the frontier planner, sim splats through
  Andrew's `validate_scene` + `Map` → `surface_splats`, recording → prescan → room.
- end to end, mode A:
  - A2 (one `SCAN`, with the sim's pose noise off so the check is exact): every mapped cell is
    in line of sight of where the car scanned, none behind the partition, none outside the room.
  - A3: explore finds the bin, then GOTO arrives about 0.74 m from it.
- end to end, mode B:
  - B1: the prescan rebuilds the room, and goto's map is seeded to the prescan's counts.
  - B2: the car pushed at an imported wall stops on it.
  - B3: GOTO from a dropped pose arrives.

Not run: a real iPhone scan (the recording import is tested with sim recordings only), the
browser panel (unless the report says so), the Pi, QNX, the real car.

**Known goto limits seen here (not changed):**
- goto keeps duplicate memory entries for one object (fixed on `kothar/goto-fixes`).
- goto can't route through a 0.8 m doorway: its planner wants a 0.7 m footprint of seen floor.
  A car dropped in the default room's back room gets `no_route`, then `failed:not_found`.
- A `SCAN` can end `failed:turn_timed_out` on a loaded Mac. The explorer carries on from the map it has.
