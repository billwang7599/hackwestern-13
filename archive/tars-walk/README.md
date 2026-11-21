# Walking TARS (dropped)

TARS walking on **two 28BYJ-48 stepper "legs"** (one crank per side) instead of wheels.

- **Uno** (`uno/tars/tars.ino`): crank gait with `W` walk, `R` turn and `Q` commands, foil crank-position
  switches on D2/D8, left/right sync, stall stop, timed fallback when a switch is missed,
  EEPROM calibration, rock count. Host tests in `uno/test/`.
- **Pi `tars_motor`** (`pi/src/main.c`): `WALK`, `TURN` and `CAL` commands, walk state in `STATUS`,
  parses the Uno's `DONE`/`ABORT`/`CAL` lines, stub walk sim, iPhone yaw turn correction (`-y`).
- **Laptop**: walk keys in `teleop.py`/`switch_teleop.py`/`voice.py`, client helpers, calibration
  and demo scripts.

## Why it was dropped

The 28BYJ-48 is too weak: about 300 gf·cm of torque. The feet dragged, and anything past a 4 cm leg
was over budget, so it barely shuffled. We switched to wheels (steppers first, now DC gearmotors /
the Kuman SM11 car), and those live in `iter1-wasd/`.

## What's here

| File | What |
|---|---|
| `0001-…patch`, `0002-…patch` | `git format-patch` of branch `kothar/tars` commits `6b52624` and `8581f7c` |
| `uncommitted.diff` | `git diff` of the `wt-tars` worktree on top of `8581f7c` (laptop walk controls, small `main.c` / `test_server.py` edits) |
| `untracked/laptop/` | three new files that were never committed: `calibrate_walk.py`, `iphone_yaw_bridge.py`, `walk_demo.py` |

## How to apply

The base is **`d30075f`** ("TARS: Uno co-processor with stepper arms + tip/bump safety…"), which is in
main's history but uses the **old, pre-reorg layout** (`uno/`, `pi/`, `laptop/` at the top level, before
`iter1-wasd/`). It won't apply to current main without hand-porting.

```sh
git switch -c tars-walk d30075f
git am archive/tars-walk/000*.patch          # copy the archive folder out first, it isn't in d30075f
git apply archive/tars-walk/uncommitted.diff
cp archive/tars-walk/untracked/laptop/*.py laptop/
```

## Test status

Mac host tests only (Uno host tests, `pi/test_server.py` against the stub backend). No pass counts
were recorded in the commits. Never run on the Uno, Pi or QNX, and never walked on real legs.
