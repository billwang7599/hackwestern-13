# MOVED: one folder per iteration (2026-11-21)

The repo was reorganized into one folder per iteration. Only paths changed: no behaviour,
ports, protocols or Pi-side install paths. Every file was moved with `git mv`, so git sees
each one as a rename.

| Iteration | Folder | What |
|---|---|---|
| 1 | `iter1-wasd/` | WASD driving: Uno firmware, `tars_motor`, laptop teleop |
| 2 | `iter2-autonomy/` | iteration 1 + voice, goto/YOLO/nav/memory, iPhone, edge AI, dashboard |
| 3 | `iter3-pet/` | iteration 1 + MediaPipe gestures and pet mode |
| — | `archive/` | parked: goalie, pico, Joy-Con relay doc |

## Old → new

| Old path | New path |
|---|---|
| `uno/` | `iter1-wasd/uno/` |
| `laptop/` | `iter1-wasd/laptop/` |
| `pi/` (`src/`, `Makefile`, `test_server.py`, `test_guard_gate.py`, `test_imu_receiver.py`, `MOTOR_GUARD.md`) | `iter1-wasd/pi/` |
| `pi/dashboard/` | `iter2-autonomy/dashboard/` |
| `pi/edge-ai/` | `iter2-autonomy/edge-ai/` |
| `pi/iphone-usb/` | `iter2-autonomy/iphone-usb/` |
| `voice/` | `iter2-autonomy/voice/` |
| `goto/` | `iter2-autonomy/goto/` |
| `iphone/` | `iter2-autonomy/iphone/` |
| `vision/` | `iter3-pet/vision/` |
| `goalie/` | `archive/goalie/` (content unchanged) |
| `pico/` | `archive/pico/` |
| `JOYCON.md` | `archive/JOYCON.md` |

Unchanged at the top: `AGENTS.md`, `CLAUDE.md`, `PLAN.md`, `README.md`, `docs/`, `docker/`,
`hardware/` (the current 28BYJ-48 wheel), `.clangd`, `.gitignore`, `.idea/`, and the
gitignored `vendor/iphone-usb/` source checkout.

Common commands, before → after:

| Before | After |
|---|---|
| `make -C pi host && make -C pi test` | `make -C iter1-wasd/pi host && make -C iter1-wasd/pi test` |
| `make -C pi native` / `keyboard-native` | `make -C iter1-wasd/pi native` / `keyboard-native` |
| `python3 pi/dashboard/service.py start` | `python3 iter2-autonomy/dashboard/service.py start` |
| `sh pi/edge-ai/build-guard.sh` | `sh iter2-autonomy/edge-ai/build-guard.sh` |
| `sh pi/iphone-usb/build-qnx.sh` | `sh iter2-autonomy/iphone-usb/build-qnx.sh` |
| `make -C goto test` | `make -C iter2-autonomy/goto test` |
| `make -C voice host test` | `make -C iter2-autonomy/voice host test` |
| `python3 vision/tests/test_pet.py` | `python3 iter3-pet/vision/tests/test_pet.py` |

Cross-folder paths that changed in code:
- `dashboard/service.py` starts `iter1-wasd/pi/build/tars_motor` (was `pi/build/tars_motor`).
- `dashboard/headless_keyboard.py` starts `iter1-wasd/pi/build/keyboard_hid`.
- `edge-ai/test_robot_stop.py` defaults to `iter1-wasd/pi/build/tars_motor_host`.
- `iphone-usb/build-qnx.sh` and `build-libusb-qnx.sh` install into `iter2-autonomy/iphone-usb/install`.
- `iter1-wasd/pi/test_guard_gate.py` imports `iter2-autonomy/dashboard/control.py`.
- The goto, voice and vision tests find `tars_motor_host` under `iter1-wasd/pi/build/`.

Deploy targets are unchanged: `make deploy` / `deploy.sh` still copy to `/data/home/qnxuser/`,
`/data/home/qnxuser/voice/` and `/data/home/qnxuser/vision/`, and `tars.env` is unchanged.

## Rebasing open work

```sh
git fetch origin
git -c merge.directoryRenames=true rebase origin/main
```

With `merge.directoryRenames=true`, new files you added under an old folder (for example
`pi/dashboard/new_thing.py`) land in the new folder too (`iter2-autonomy/dashboard/new_thing.py`).
Edits to existing files follow the renames automatically. Afterwards, `git grep` your new
code for `pi/`, `../pi`, `goto/`, `voice/` or `vision/` paths and point them at the table above.

## On a Pi that runs from a repo checkout (Andrew's setup)

Git doesn't move ignored files (build output, state, pairing records). They stay behind in the old
`pi/...` folders after you pull. In the checkout on the Pi:

```sh
mkdir -p iter1-wasd/pi iter2-autonomy/dashboard iter2-autonomy/edge-ai iter2-autonomy/iphone-usb
[ -d pi/build ] && mv pi/build iter1-wasd/pi/                         # tars_motor, keyboard_hid
[ -d pi/dashboard/state ] && mv pi/dashboard/state iter2-autonomy/dashboard/
for d in build state; do [ -d pi/edge-ai/$d ] && mv pi/edge-ai/$d iter2-autonomy/edge-ai/; done
[ -d pi/iphone-usb/state ] && mv pi/iphone-usb/state iter2-autonomy/iphone-usb/   # pairing records
sh iter2-autonomy/iphone-usb/build-libusb-qnx.sh && sh iter2-autonomy/iphone-usb/build-qnx.sh
```

Rebuild the iPhone USB tools instead of moving `pi/iphone-usb/install/`: `configure --prefix` bakes
the old absolute path into them. Then update any boot script or alias that calls
`pi/dashboard/service.py`. Once everything works, delete the leftover `pi/` folder.
