# archive/: parked and dropped work

None of this is built, tested or deployed by the robot. Nothing at the top level (Makefiles, test
runners) reaches into `archive/`. Generated output (`build/`, `out/`, plots), virtualenvs, caches,
Xcode user data and models were left out. All test status below is **Mac only**: none of it ran on
the Pi, QNX, the Uno or the iPhone.

## Parked (moved here in the folder reorg)

| Path | What | Source |
|---|---|---|
| [`goalie/`](goalie/) | "Beat the Goalie", a separate project (sim, Pi, Arduino, vision) | was `goalie/` on main |
| [`pico/`](pico/) | legacy MicroPython Pico firmware, `tars_motor -b pico` | was `pico/` on main |
| [`JOYCON.md`](JOYCON.md) | right Joy-Con IMU relay notes | was `JOYCON.md` on main |

## Dropped, never pushed before (archived here)

| Path | What it was | Why dropped | Last known test status | Source |
|---|---|---|---|---|
| [`goalie/mv/`](goalie/mv/) | Goalie ball tracker: single-webcam 3D ball tracking (size → distance, gravity/drag fit) and goal-crossing prediction, calibration, synthetic renderer + sweep, `RESULTS.md` | the Goalie project was parked for TARS | unit tests in `tests/` (13 test functions), pass counts not recorded; synthetic accuracy sweep in [`RESULTS.md`](goalie/mv/RESULTS.md); never run with a real webcam on the robot | branch `kothar/goalie-mv` @ `059f4cf` (commits `6d8c367`, `602f668`, `059f4cf`) |
| [`tars-walk/`](tars-walk/) | Walking TARS: two 28BYJ-48 stepper crank legs, Uno gait (`W`/`R`/`Q`), Pi `WALK`/`TURN`/`CAL`, laptop walk controls. Stored as patches against the old layout | 28BYJ-48 too weak (~300 gf·cm), feet dragged; switched to wheels | Mac host tests (Uno + stub `tars_motor`), counts not recorded | branch `kothar/tars` @ `8581f7c` (`6b52624`, `8581f7c`, base `d30075f`) + uncommitted/untracked files in `wt-tars`. See [`tars-walk/README.md`](tars-walk/README.md) |
| [`balancer/`](balancer/) | Uno two-wheel self-balancer: MPU6050 → complementary filter → PID at 200 Hz → H-bridge, serial protocol for Pi steering/tuning, optional external (iPhone) tilt | self-balancing dropped: the motors on hand were too weak and time ran short | Uno host tests (`make -C uno/test`) 109/109 on the Mac (worker report); never flashed | untracked in `wt-balancer` (worktree at `61ccad4`), never committed |
| [`iphone-depth-wifi/`](iphone-depth-wifi/) | Our first iPhone app `TARSDepth`: ARKit depth scan → one JSON line per frame over UDP 5559 to the Pi (Wi-Fi) | replaced by Andrew's USB ARKit app in `iter2-autonomy/iphone` | Swift tests 27/27 on the Mac (worker report); never run on a phone with the Pi. `iphone/PROTOCOL.md` and the Pi-side `c/` receiver and `tools/` were **not** present (empty dirs), so only the app is archived | untracked in `wt-iphone` (worktree at `98b41da`), never committed |
| [`balance-sim/`](balance-sim/) | First self-balancing TARS study: portable C99 controller `balance/` (+ MicroPython port), Python simulator/tuner `sim/` comparing TT, 25 mm DC, Pololu 37D and NEMA17, write-up [`balance.md`](balance-sim/balance.md) | self-balancing dropped (no NEMA17s on hand; the DC motors failed in sim) | `make -C balance test` and C/MicroPython parity passed on the Mac; sim: NEMA17 + TMC2209 PASS (1.00 N·s max push), small DC motors FAIL | untracked `balance/`, `sim/`, `docs/balance.md` in the user's checkout `~/Documents/projects/hack-western-13` |

`balance-sim/` keeps the original `balance/` + `sim/` side by side, so the paths in `balance.md` still work
from inside `archive/balance-sim/`. The sim needs its own venv (`sim/requirements.txt`).
