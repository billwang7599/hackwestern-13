# Beat the Goalie

An AI robot goalkeeper for Hack Western 13. An overhead webcam tracks a
ping-pong ball. A QNX real-time loop on the Pi 5 predicts where the ball will
cross the goal line and swings a DC-motor paddle there. An accelerometer
counts the saves.

Protocol and coordinates: [docs/architecture.md](docs/architecture.md).

```
sim/       feasibility + controller sim (run this first: is the motor fast enough?)
pi/        QNX C server goalie_motor (Makefile: make / host / test / parity / deploy / run)
arduino/   Uno sketch: motor driver, MPU6050 hit detect, 28BYJ-48 needle, flag
vision/    laptop: webcam tracking, scoreboard UI, Switch controller, Claude taunts
docs/      architecture (protocol contract), wiring, RPM test, demo run sheet,
           integration_run.txt (latest no-hardware end-to-end run)
integration_run.sh   fake ball -> real C server -> motor duties, writes docs/integration_run.txt
```

## Quickstart

### 0. No hardware: everything on the Mac (5 min)

```sh
cd goalie
(cd sim && python3 -m venv .venv && .venv/bin/pip install -r requirements.txt)
(cd vision && python3.14 -m venv .venv && .venv/bin/pip install -r requirements.txt)
make -C pi test              # builds the server for the Mac, 47 protocol checks
./integration_run.sh         # fake ball -> real C server -> fake paddle, 8 scenarios

# play it with a fake camera:
pi/build/goalie_motor_host -b stub -p 5556 &
(cd vision && GOALIE_HOST=127.0.0.1 .venv/bin/python track.py --fake)
```

### 1. Is the motor fast enough? (5 min)

Do the [RPM test](docs/rpm_test.md), then:

```sh
cd sim && .venv/bin/python run.py --rpm X        # PASS/FAIL, out/scenario.png
```

Short answer from the sim: a pivot paddle needs ~15 RPM for 1 m/s rolls and
~30 RPM for 1.5 m/s corner shots. Use a **narrow 5 cm blade**: a paddle that
never moves already saves ~50%.

### 2. Uno

Identify the driver chip and wire it up ([docs/wiring.md](docs/wiring.md),
details in [arduino/README.md](arduino/README.md)). Set `DRIVER_TYPE` in
`arduino/goalie/goalie.ino`, upload, then bench-test from the Mac:

```sh
python3 arduino/serial_test.py /dev/cu.usbmodemXXXX
```

### 3. Pi 5 (QNX 8.0)

```sh
cd pi
make                     # cross-compiles in the QNX Docker container (../../docker/run.sh)
make deploy PI_HOST=qnxuser@<pi>
ssh qnxuser@<pi> ./goalie_motor -b arduino -d /dev/serusb1 --rpm X
```

### 4. Laptop

```sh
cd vision
export GOALIE_HOST=<pi>
.venv/bin/python calibrate.py      # 4 field corners, then ball and paddle colours
.venv/bin/python track.py          # play; m = QNX/LINUX, h = human goalie, r = reset, t = taunts
```

Demo script, jitter demo and kill -9 demo: [docs/demo.md](docs/demo.md).

## Tests

| What | Command | Last result |
|---|---|---|
| sim pass/fail | `cd sim && .venv/bin/python run.py` | exit 0 (PASS) |
| C server | `make -C pi test` | 47/47 |
| C vs Python predictor + controller | `make -C pi parity` | exact to ~1e-15 |
| Uno sketch (host mock) | `make -C arduino/test` | 81/81 |
| vision (mock server) | `cd vision && .venv/bin/python -m pytest -q` | 16/16 |
| vision (real C server) | `GOALIE_SERVER_BIN=../pi/build/goalie_motor_host .venv/bin/python -m pytest -q` | 16/16 |
| end to end | `./integration_run.sh` | see `docs/integration_run.txt` |

Not tested yet: anything on the real Pi (QNX cross-compile, USB serial
naming, SCHED_RR rights), the real Uno with motors, the real webcam
calibration and the Switch controller.
