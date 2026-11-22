# TARS: four slabs on wheels

For the QNX self-balancing sensor prototype, see [Right Joy-Con IMU relay](archive/JOYCON.md).
For the wired iPhone experiment, see [MotionUSB](iter2-autonomy/iphone/README.md) and
[QNX USB transport setup](iter2-autonomy/iphone-usb/README.md). Rear-camera/LiDAR mapping and
the mounted-phone surroundings display are described in [the dashboard guide](iter2-autonomy/dashboard/README.md).

For the native QNX AI runtime, independent LiDAR guard, and 3D phone display,
see [QNX edge AI](iter2-autonomy/edge-ai/README.md).

> **Testing on the real Pi?** Start with **[docs/PI_TEST.md](docs/PI_TEST.md)**: build, wire,
> bring-up, WASD teleop, calibration and troubleshooting, step by step from a fresh clone.
> The wheels are now **two 28BYJ-48 steppers** (exact `MOVE`/`TURN`); the DC-motor build is still
> available with `WHEEL_TYPE WHEELS_DC` in `iter1-wasd/uno/tars/tars.ino`.

A TARS-inspired robot (from *Interstellar*) for Hack Western 13.

Four 30 cm foam-board slabs. The middle two stand on a wheeled base (two DC
gearmotors + a caster, no balancing). The outer two are arms swung by
28BYJ-48 steppers for gestures. An accelerometer stops everything if it
tips over or bumps into something. **Build guide: [docs/tars_build.md](docs/tars_build.md).**

```
[Mac laptop]  teleop.py (keys) · switch_teleop.py (Switch controller)  iter1-wasd/laptop/
              (sender/receiver only; voice runs on the Pi: iter2-autonomy/voice/)
     │  TCP :5555, line protocol over Wi-Fi
     ▼
[Pi 5 · QNX 8.0]  tars_motor: SCHED_RR control thread @50 Hz,          iter1-wasd/pi/
                  500 ms watchdog, tip-over latch, bump hold, slew,
                  gesture keyframes + arm model
     │  USB serial 115200: "D 0.5 0.5" every tick, "A 1 70 60", ...
     │  ← "T 63" tipped, "B 1500" bump, "READY"
     ▼
[Arduino Uno]  H-bridge → 2 DC wheels · 2× ULN2003 → 28BYJ-48 arms     iter1-wasd/uno/
               ADXL345 or MPU6050 → tip/bump · 200 ms watchdog
```

Why an Uno in between? Pi 5 GPIO/PWM goes through the RP1 chip, and QNX
support for it is new. Serial on QNX is reliable, and the Uno does PWM, stepper
timing and the accelerometer well. The Uno stays dumb. Gestures are keyframe
tables on the Pi (`iter1-wasd/pi/src/gesture.c`), so they can change without reflashing.

## Iterations

The repo has one folder per iteration. Iterations 2 and 3 both build on iteration 1:
all motion still goes through iteration 1's `tars_motor` (TCP 5555).

| Folder | What you get | Needs |
|---|---|---|
| [`iter1-wasd/`](iter1-wasd/) | WASD driving: Uno firmware, the QNX motor server `tars_motor`, laptop teleop | Pi + Uno + wheels |
| [`iter2-autonomy/`](iter2-autonomy/) | Iteration 1 plus voice commands, YOLO-World "go to the trash can", object memory, map-guided GOTO, and Andrew's iPhone LiDAR/ARKit + edge-AI + dashboard stack | iteration 1, mic, camera, iPhone |
| [`iter3-pet/`](iter3-pet/) | Iteration 1 plus MediaPipe hand gestures, follow mode, teach-a-gesture and pet mode | iteration 1, camera |
| [`archive/`](archive/) | Parked or legacy: the Goalie game, the Pico firmware, the Joy-Con IMU relay | nothing |

Paths moved on 2026-11-21; [docs/MOVED.md](docs/MOVED.md) has the old → new table.

```
iter1-wasd/
  pi/              tars_motor: C motor server for QNX (Makefile: make / deploy / run / host / test)
  uno/             Arduino Uno firmware (uno/tars/tars.ino) + host unit tests (uno/test)
  laptop/          tars_client.py (lib), teleop.py, switch_teleop.py, wheel_cal.py
iter2-autonomy/
  voice/           offline speech -> intent classifier (whisper.cpp, C)
  goto/            go-to controller + YOLO-World detector + object memory + nav.c A* (C/C++)
  iphone/          MotionUSB iOS app (ARKit, LiDAR, speech link on 5573)
  iphone-usb/      usbmuxd/libusb for QNX (iPhone over USB)
  edge-ai/         QNX ONNX Runtime YOLOX/MiDaS, lidar_guard
  dashboard/       Pi dashboard, mapping preview, service.py launcher
iter3-pet/
  vision/          MediaPipe hand gestures, follow, teach-a-gesture, pet mode (see iter3-pet/vision/README.md)
archive/           goalie/ (separate "Beat the Goalie" project), pico/ (legacy MicroPython, backend `-b pico`), JOYCON.md
hardware/          3D-printable 127 mm wheel for the 28BYJ-48 steppers
docker/            Ubuntu amd64 container with QNX Linux host tools (SDP mounted from Mac)
docs/              PI_TEST.md (hands-on checklist), ARCHITECTURE.md, MOVED.md, tars_build.md, tars_stub_run.txt
.clangd            clangd config for nvim
```

---

## 1. One-time setup (do this BEFORE the hackathon)

QNX SDP 8.0 has **no macOS host tools**, so the compiler runs in Docker. This
follows QNX's official guide ([blog](https://devblog.qnx.com/how-to-use-macos-to-develop-for-qnx-8-0/),
[repo](https://gitlab.com/qnx/projects/docker-mac)).

1. **License:** get the free non-commercial license at <https://qnx.com/getqnx>
   and deploy it to yourself in the myQNX License Manager. Approval can take time.
2. **QNX Software Center for macOS:** install it, log in, and activate the license.
3. **Install the SDP:** *Add Installation → QNX Software Development Platform 8.0
   (com.qnx.qnx800)* into the default **`~/qnx800`**.
4. **Linux host tools:** *Manage Installation → Available → "QNX SDP Host Tools
   (Linux 64-bit)"*. These are what run inside Docker.
5. **Pi 5 image:** also install `com.qnx.qnx800.quickstart.rpi5`. The `.img`
   ends up under `~/qnx800`. Run `find ~/qnx800 -name '*.img'` to find it.
6. **Docker Desktop:** turn on *Settings → General → Use Rosetta for
   x86_64/amd64 emulation*. Then:
   ```sh
   docker/run.sh build        # builds the tars-qnx800 image (no QNX bits inside)
   docker/run.sh qcc --version  # sanity check, should be able to find qcc
   ```
   The SDP is licensed, so it is **never baked into the image**. `run.sh`
   bind-mounts `~/qnx800` (override with `QNX_SDP_HOST=...`) and `~/.qnx`
   (license) into the container, and `entrypoint.sh` sources `qnxsdp-env.sh`.
   If macOS flags Docker as malware, see
   [docker/for-mac#7520](https://github.com/docker/for-mac/issues/7520).

### Flash and boot the Pi 5
1. Use **Raspberry Pi Imager → Use custom** and pick the QNX Pi 5 `.img`.
   Use a 32 GB+ microSD card.
2. Before ejecting: on the FAT32 boot partition, edit or create
   `wpa_supplicant.conf`:
   ```
   network={
       ssid="HackWesternWiFi"
       psk="password"
       key_mgmt=WPA-PSK
   }
   ```
   Venue Wi-Fi with a captive portal won't work, so use a phone hotspot.
3. Boot **with no USB keyboard or mouse plugged in** (known Pi 5 image bug).
   Use the official 27 W power supply.
4. Find its IP (hotspot client list, or a USB-UART cable on the serial console),
   then `ssh qnxuser@<ip>`. Check the QSTI docs for the default password.
   Add an entry to `~/.ssh/config` so `tars.local` works, or use
   `PI_HOST=qnxuser@<ip>` everywhere:
   ```
   Host tars.local
       HostName 192.168.x.y
       User qnxuser
   ```
   Then run `ssh-copy-id qnxuser@tars.local` so deploys don't prompt.

### Laptop (Python)
```sh
cd laptop && python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
export ANTHROPIC_API_KEY=...
```

For the Switch controller: `pip install pygame`, then pair the controller in
macOS Bluetooth settings.

### Uno
Arduino IDE: open `iter1-wasd/uno/tars/tars.ino`, board "Arduino Uno", upload. Or:
```sh
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:uno iter1-wasd/uno/tars
arduino-cli upload  --fqbn arduino:avr:uno -p /dev/cu.usbmodemXXXX iter1-wasd/uno/tars
```
Check the CONFIG block first (driver type, inverts). See docs/tars_build.md §5 and §7.

---

## 2. Dev loop

```sh
# edit in nvim on the Mac, then:
make -C iter1-wasd/pi                  # cross-compiles in Docker (auto) -> iter1-wasd/pi/build/tars_motor
make -C iter1-wasd/pi deploy           # scp to $PI_HOST (default qnxuser@tars.local)
make -C iter1-wasd/pi run              # ssh + run (BACKEND=uno UART=/dev/serusb1 PORT=5555)
make -C iter1-wasd/pi run BACKEND=stub # no hardware: just prints motor/arm commands

python iter1-wasd/laptop/tars_client.py tars.local     # ping + status
python iter1-wasd/laptop/teleop.py tars.local          # window: HOLD WASD to drive, release = stop (--terminal: old mode)
python iter1-wasd/laptop/wheel_cal.py move 1000         # exact moves / calibration (see docs/PI_TEST.md §5)
python iter1-wasd/laptop/switch_teleop.py tars.local   # Switch controller (--show to map buttons)
```

**No Pi yet?** Run `make -C iter1-wasd/pi host && iter1-wasd/pi/build/tars_motor_host -b stub`, then
point the laptop scripts at `127.0.0.1` (`TARS_HOST=127.0.0.1`). A captured
run is in `docs/tars_stub_run.txt`.

**Tests (no hardware):**
```sh
make -C iter1-wasd/pi test     # server: protocol, watchdog, gestures, tip/bump (stub)
                    # + the real uno backend against a fake Uno on a pty
make -C iter1-wasd/uno/test    # firmware on a mock Arduino core: stepper wheels (+/- one arm),
                    # DC wheels (both driver types), both accelerometers
python3 iter1-wasd/laptop/test_teleop.py   # hold-to-drive key logic, -> stub server, pygame window
```

Root may be needed for SCHED_RR priority > the user limit. If the thread can't
get it, the server warns and continues at default priority.

### nvim / clangd
- `make -C iter1-wasd/pi compdb` runs `bear` inside the container and writes
  `iter1-wasd/pi/compile_commands.json`. The repo and SDP are mounted at the **same
  absolute paths** as on the Mac, so the paths in it work for clangd on macOS.
- `--query-driver` can't help here, because qcc is a Linux binary and clangd on the
  Mac can't execute it. Instead, `.clangd` strips qcc's `-V…` flag and adds
  `-isystem /Users/bill/qnx800/target/qnx/usr/include`, `-D__QNXNTO__` and an
  aarch64 target. **Edit that path** if your SDP lives elsewhere.
- If headers still resolve to macOS ones, check `:LspInfo` and `:LspLog`, and confirm
  `compile_commands.json` is next to `iter1-wasd/pi/` (clangd searches parent dirs).

---

## 3. Protocol (TCP :5555, newline-terminated, ASCII)

| Command | Meaning | Reply |
|---|---|---|
| `D <l> <r>` | wheel speeds in [-1, 1] (clamped). **Resend at least every 500 ms** | `OK`, or `ERR tipped…` / `ERR bumped…` |
| `S` | stop wheels **now** (no slew), cancel gesture, halt arms | `OK` |
| `G <name>` | gesture: `wave` `point` `shrug` `nod` `stance` `fold` (non-blocking) | `OK <expected_ms>` |
| `A <arm> <deg> [dps]` | arm 0=left 1=right to an absolute angle (+ = forward, ±135, ≤60 °/s) | `OK <clamped_deg>` |
| `HOME` | both arms to 0 | `OK` |
| `ZERO` | current arm position := 0 (after straightening them by hand) | `OK` |
| `LEVEL` | "it's upright now": recapture the tilt reference | `OK` |
| `RESET` | clear the tip-over latch (refused while the Uno still reports tipped) | `OK` / `ERR still tipped` |
| `GESTURES` | | `OK wave point …` |
| `PING` | keepalive | `OK PONG` |
| `STATUS` | | `OK <l> <r> <ms_since_cmd> <dist_m> <estop> why=… arm0=… arm1=… gesture=… tipped=… tilt=… bump_ms=… bumps=… ready=… backend=… stop_ms=… vis=… vis_fps=… vis_cam_ms=… vis_stop_ms=… vis_see=… vis_age_ms=… move=… move_id=… move_kind=… move_req=… move_done=… move_why=… move_est_ms=… wheel_d=… wheel_t=… btn_teach=… btn_forget=… led=…` |
| `MOVE <mm> [speed]` | stepper wheels: drive exactly mm (+ fwd), speed 0–1 of max. Non-blocking; a non-zero `D`, `S`, tip, bump or disconnect aborts it | `OK <id> <est_ms>`; progress in STATUS `move=running\|done\|aborted` |
| `TURN <deg> [speed]` | stepper wheels: spin in place exactly, **+ = right** (clockwise) | `OK <id> <est_ms>` |
| `WHEELCAL [diam track]` | read / set (saved in the Uno's EEPROM) wheel diameter and track, mm | `OK <diam> <track>` |
| `LED <name\|0-3>` / `BEEP <n>` | on-robot LED: `off`=0 `teach`=2 slow blink, `rec`/`ok`=1 solid, `no`/`forget`=3 fast blink (iter3-pet/vision/GESTURES.md §7); n beeps (0–9) | `OK`, `ERR usage` / `ERR backend has no LED` (nothing changes) |
| `VIS <mode> <fps> <cam_ms> <stop_ms> <label>` | report from the hand-gesture app (`iter3-pet/vision/`); shown in STATUS, doesn't feed the watchdog | `OK` |
| `SIM T\|B\|READY [n]`, `SIM BTN TEACH\|FORGET` | **stub backend only**: inject an Uno event (tests) | `OK` |
| anything else | | `ERR <reason>` |

Safety, in layers:
- client disconnect → wheels stop now; no command for 500 ms → stop (`-w`)
- **tip-over** (Uno: tilt > 40° for 150 ms) → the Uno stops itself, the Pi stops
  wheels + gesture + arms and **latches**. Driving, arms and gestures are refused
  until `RESET`, which is only accepted 1 s after the last `T` report.
- **bump** (|accel| spike > 1.2 g) → wheels stop, driving refused for 1 s (`-k`)
- obstacle closer than 0.25 m blocks forward motion (`iter1-wasd/pi/src/obstacle.c`, a stub for now)
- the Uno stops the wheels on its own after 200 ms without a `D` line (exact moves are
  self-terminating, and abort if the Pi goes quiet for 500 ms mid-move)
- runs unattended: with no client the wheels stay stopped; a missing or unplugged Uno is
  reopened every second

Pi → Uno serial protocol: see the header of `iter1-wasd/uno/tars/tars.ino`. The Pi sends
`D` every tick (50 Hz). Lines from the Uno starting with `#` are logged, never acted on.

---

## 4. Wiring

See **[docs/tars_build.md](docs/tars_build.md)**: slab sizes, wheel/caster/stepper
mounting, power (separate motor and stepper supplies, common ground, **never
power motors or steppers from the Uno's 5 V**), the Uno pin tables for both
driver types, the accelerometer, and the Pi↔Uno link (USB, or header UART with a
5 V→3.3 V divider).

Quick reference (default 2-PWM driver): wheels D9/D10 (left) and D3/D11 (right),
left arm ULN2003 on A0–A3, right arm on D4–D7, accelerometer SDA/SCL on A4/A5.

The old Pi↔Pico UART + TB6612 wiring still works with `-b pico` (see `archive/pico/main.py`).

---

## 5. Hackathon checklist

- [ ] License deployed, SDP + Linux host tools in `~/qnx800`, `docker/run.sh qcc --version` works
- [ ] `make -C iter1-wasd/pi` produces an aarch64 binary (`file iter1-wasd/pi/build/tars_motor`)
- [ ] Pi 5 boots QNX, joins the hotspot, `ssh` works, `ssh-copy-id` done
- [ ] Plug the Uno into the Pi, `ls /dev/ser*` before/after: find its name
      (assumed `/dev/serusb1`) and set `UART=`. If nothing shows up, use the
      header-UART fallback (docs/tars_build.md §6)
- [ ] `make -C iter1-wasd/pi run BACKEND=stub` + `teleop.py`: watch the watchdog stop when you quit
- [ ] Uno flashed, bench bring-up done (docs/tars_build.md §7): arms swing the
      right way, wheels spin the right way, `T`/`B` events appear
- [ ] Chassis: middle slabs on the wheeled base + caster, arms pivot at mid-height,
      clear of the floor
- [ ] Arms straight before power-up (that's 0°); TARS standing when the Uno boots
- [ ] Voice on the Pi end to end (`make -C iter2-autonomy/voice native`, phone mic via AirPods)
- [ ] Obstacle sensor wired into `obstacle.c` (or read by the Uno and sent as a line)
- [ ] Demo script: "set humor to 0%" / "100%", "come here", "wave"
- [ ] **Batteries charged**, spare SD card with the working image, phone hotspot on

## Open questions / verify on the hardware
- **Uno USB serial on QNX:** does the Uno enumerate (CDC-ACM; CH340 clones need a
  different driver), and under what name (assumed `/dev/serusb1`)?
- **Header UART path** for GPIO14/15 if you use the fallback (assumed `/dev/ser1`).
  The console may own one UART. Check `ls /dev/ser*` and the BSP docs.
- **Driver chip:** which board you have decides `DRIVER_TYPE` in `tars.ino`.
- **Thresholds:** `BUMP_THRESHOLD_MG` (1200) and `TIP_DEG` (40°) are guesses. Tune on the desk.
- **Home dir / writable path** on the Pi (Makefile assumes `/data/home/qnxuser`,
  override with `PI_DIR=`).
- **Default login/hostname** of the Pi 5 QSTI. Check the QSTI docs.
- **SCHED_RR priority limits** for non-root on QNX 8 (default `-r 30`).
