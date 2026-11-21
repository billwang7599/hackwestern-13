# TARS: four slabs on wheels

A TARS-inspired robot (from *Interstellar*) for Hack Western 13.

```
[Mac laptop]  listen() → Claude (humor/honesty %) → speak()          laptop/
     │  TCP :5555, line protocol over Wi-Fi
     ▼
[Pi 5 · QNX 8.0]  tars_motor: SCHED_RR control thread @50 Hz,         pi/
                  500 ms watchdog, obstacle-stop hook, slew limiting
     │  UART 115200, "D 0.5 0.5\n"
     ▼
[Pico · MicroPython]  PWM → TB6612FNG → 2 DC motors (+ gesture servo)  pico/
```

Why a Pico in between? Pi 5 GPIO/PWM goes through the RP1 chip, and QNX
support for it is new. UART on QNX is reliable, and the Pico does PWM well. To
drive motors from the Pi directly, write a `motor_gpio.c` backend
(see `pi/src/motor.h` and the TODO in `motor_stub.c`).

```
docker/   Ubuntu amd64 container with QNX Linux host tools (SDP mounted from Mac)
pi/       C motor server for QNX (Makefile: make / deploy / run / compdb / host)
pico/     MicroPython firmware for the motor co-processor
laptop/   Python: tars_client.py (lib), teleop.py (WASD), voice.py (LLM brain)
.clangd   clangd config for nvim
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

### Pico
Flash MicroPython (hold BOOTSEL, drop the `.uf2`), then copy `pico/main.py`
to the board. Thonny is easiest, or run `mpremote cp pico/main.py :main.py`.

---

## 2. Dev loop

```sh
# edit in nvim on the Mac, then:
make -C pi                  # cross-compiles in Docker (auto) -> pi/build/tars_motor
make -C pi deploy           # scp to $PI_HOST (default qnxuser@tars.local)
make -C pi run              # ssh + run (BACKEND=pico UART=/dev/ser1 PORT=5555)
make -C pi run BACKEND=stub # no hardware: just prints motor commands

python laptop/tars_client.py tars.local   # ping + status
python laptop/teleop.py tars.local        # drive with WASD
python laptop/voice.py --no-robot         # brain only
python laptop/voice.py --host tars.local  # the whole thing
```

**No Pi yet?** Run `make -C pi host && pi/build/tars_motor_host -b stub`, then
point the laptop scripts at `127.0.0.1` (`TARS_HOST=127.0.0.1`).

Root may be needed for SCHED_RR priority > the user limit. If the thread can't
get it, the server warns and continues at default priority.

### nvim / clangd
- `make -C pi compdb` runs `bear` inside the container and writes
  `pi/compile_commands.json`. The repo and SDP are mounted at the **same
  absolute paths** as on the Mac, so the paths in it work for clangd on macOS.
- `--query-driver` can't help here, because qcc is a Linux binary and clangd on the
  Mac can't execute it. Instead, `.clangd` strips qcc's `-V…` flag and adds
  `-isystem /Users/bill/qnx800/target/qnx/usr/include`, `-D__QNXNTO__` and an
  aarch64 target. **Edit that path** if your SDP lives elsewhere.
- If headers still resolve to macOS ones, check `:LspInfo` and `:LspLog`, and confirm
  `compile_commands.json` is next to `pi/` (clangd searches parent dirs).

---

## 3. Protocol (TCP :5555, newline-terminated, ASCII)

| Command | Meaning | Reply |
|---|---|---|
| `D <l> <r>` | wheel speeds in [-1, 1] (clamped). **Resend at least every 500 ms** | `OK` |
| `S` | stop | `OK` |
| `G <name>` | gesture (`wave`, `nod`) | `OK` |
| `PING` | keepalive | `OK PONG` |
| `STATUS` | | `OK <l> <r> <ms_since_cmd> <dist_m> <estop>` |
| anything else | | `ERR <reason>` |

Safety, in layers: client disconnect → immediate stop; no command for 500 ms
→ stop (`-w` to change); obstacle closer than 0.25 m blocks forward motion
(`pi/src/obstacle.c`, a stub for now); the Pico stops on its own after 300 ms
without UART traffic.

Pi → Pico UART uses the same `D`/`S`/`G` lines. The Pi resends `D` every tick (50 Hz).

---

## 4. Wiring

**Pi 5 ↔ Pico (UART, 3.3 V on both sides, so wire them directly)**

| Pi 5 header | | Pico |
|---|---|---|
| pin 8 GPIO14 TXD | → | GP1 (UART0 RX), pin 2 |
| pin 10 GPIO15 RXD | ← | GP0 (UART0 TX), pin 1 |
| pin 6 GND | — | GND, pin 3 |
| pin 2 5V *(optional)* | → | VSYS, pin 39 (don't also plug in USB) |

**Pico → TB6612FNG** (pins set at the top of `pico/main.py`)

| Pico | TB6612FNG | |
|---|---|---|
| GP2 | PWMA | left speed |
| GP3 / GP4 | AIN1 / AIN2 | left direction |
| GP5 | STBY | high = enabled |
| GP6 / GP7 | BIN1 / BIN2 | right direction |
| GP8 | PWMB | right speed |
| 3V3(OUT) pin 36 | VCC | logic |
| GND | GND | **common ground with the motor battery** |
| — | VM | motor battery + (4.5–13.5 V) |
| — | AO1/AO2, BO1/BO2 | left motor, right motor |
| GP15 | — | gesture servo signal (power the servo from 5–6 V, not the Pico) |

A TB6612 handles about 1.2 A per channel. For larger motors, swap in a BTS7960 or
L298N and adjust `Motor` in `pico/main.py`. If a wheel spins backwards, set
`INVERT_LEFT/RIGHT`.

---

## 5. Hackathon checklist

- [ ] License deployed, SDP + Linux host tools in `~/qnx800`, `docker/run.sh qcc --version` works
- [ ] `make -C pi` produces an aarch64 binary (`file pi/build/tars_motor`)
- [ ] Pi 5 boots QNX, joins the hotspot, `ssh` works, `ssh-copy-id` done
- [ ] Find the header UART device on QNX (`ls /dev/ser*`), loopback-test it
      (jumper pin 8↔10, `echo hi > /dev/serN`), and set `UART=` accordingly
- [ ] `make -C pi run BACKEND=stub` + `teleop.py`: watch the watchdog stop when you quit
- [ ] Pico flashed, LED lights when driving, wheels spin the right way
- [ ] Chassis: 4 slabs, outer two = drive wheels, caster under the middle
- [ ] `voice.py` end-to-end; Bluetooth speaker on the robot paired to the laptop
- [ ] Real STT in `listen()` (or push-to-talk)
- [ ] Obstacle sensor wired into `obstacle.c` (or via the Pico)
- [ ] Demo script: "set humor to 0%" / "100%", "come here", "wave"
- [ ] **Batteries charged**, spare SD card with the working image, phone hotspot on

## Open questions / verify on the hardware
- **UART device path** for GPIO14/15 on the QNX Pi 5 image (assumed `/dev/ser1`).
  The console may own one UART. Check `ls /dev/ser*` and the BSP docs.
- **Home dir / writable path** on the Pi (Makefile assumes `/data/home/qnxuser`,
  override with `PI_DIR=`).
- **Default login/hostname** of the Pi 5 QSTI. Check the QSTI docs.
- **SCHED_RR priority limits** for non-root on QNX 8 (default `-r 30`).
