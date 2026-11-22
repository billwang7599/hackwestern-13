# Motor control with the iPhone LiDAR guard

The QNX sensor service now starts the repository's native `tars_motor` server and
routes the phone's short drive requests to it. Deployment defaults to **stub**:
commands and stopping are exercised, but no serial device or physical motor is
controlled. The source was integrated from `origin/stepper-wheels` at `39d55ea`.

## Build and run on the Pi

From the saved repository copy:

```sh
make -C iter1-wasd/pi native
sh iter2-autonomy/edge-ai/build-guard.sh
python3 iter2-autonomy/dashboard/service.py start
```

The service uses `tars_motor -b stub -l -g 5568`: TCP 5555 is loopback-only,
and UDP 5568 is an independent native clearance gate. Guard/AI/Python processes
are separate. The guard evaluates at 100 Hz; the motor outputs at 50 Hz. Guard
UDP datagrams never wait for AI or the phone UI. The service enables the guard's
`--motor-port 5568`; standalone tests default that option to 0, so synthetic
fixtures cannot accidentally authorize the production motor process.

The phone status contains `motor.backend`, `mode`, `requested`, `output`,
`inhibited`, `reason`, `guard_age_ms`, `guard_epoch`, and connection/age fields.
The phone return channel updates at up to 5 Hz; its background color is a
status display, not the 100 Hz guard or the native stopping loop.
Output values are **commands**, not encoder feedback or proof of movement.
A stub result is always labeled stub.

## Stop and command semantics

- The guard sends a 13-byte `GAT1` packet: four magic bytes, uint64 big-endian
  host `CLOCK_MONOTONIC` microseconds, and one permit byte (0 or 1).
- The motor accepts only increasing, non-future timestamps younger than 100 ms.
  It drains at most 16 datagrams per motor tick and expires missing heartbeats.
  A denial seen in a batch always stops for at least one motor tick, even if a
  CLEAR follows it in the same receive batch.
- LiDAR samples themselves expire at 150 ms. Therefore a stopped sensor and a
  stopped guard have different detection deadlines. These are software limits,
  not measured braking distances or certified worst-case response times.
- Any BLOCKED, UNKNOWN, stale/disconnected sensor, guard failure, or malformed
  gate packet denies motion. Denial hard-zeros wheel commands and cancels exact
  MOVE/TURN and gestures. It never blindly reverses or turns by itself.
- Exceptions while the reason is a **fresh `guard_blocked`** (never stale, missing,
  invalid or UNKNOWN): an in-place `TURN` (speed clamped to 0.40, |deg| <= 180) and a
  **reverse `MOVE`** (mm < 0, speed clamped to 0.50; DC wheels ignore MOVE speed) are
  accepted, so a client can pivot or back out the way it came. The guard only watches
  ahead, so these rely on the caller (goto retraces its own breadcrumb trail). A forward
  MOVE, D/DL and a faster running reverse/turn still abort. S, tip, bump, the watchdog
  and the deadman still stop them. Pending Andrew's sign-off.
- `DL <guard_epoch> <left> <right>` is the phone bridge's leased drive command.
  STATUS supplies the epoch; every clear-to-denied event increments it. The bridge
  pins the epoch for one phone button press. Old repeated commands remain refused
  after an obstacle clears; a new user command is required. Legacy `D` is an
  explicit new command each time, intended for standalone clients/bench tools.
- The bridge's mailbox expires in 150 ms if the dashboard stops updating it.
  A phone pulse expires in 500 ms. The native watchdog is 500 ms and covers
  exact MOVE/TURN as well as continuous drive. Exact moves require PING or D 0 0
  keepalives. TCP disconnect hard-stops and cancels even queued moves.
- The Uno backend gives a zero transition / move abort priority over queued
  drive lines. Its RX and output work is bounded. Uno firmware separately has
  its 200 ms watchdog. USB backpressure and physical braking still need bench
  validation with the actual board, power supply, and wheels.

Loopback is a trusted-process boundary, not authentication: another process on
the Pi can forge fresh guard packets. Do not expose the gate port on a network.

## Hardware bring-up later

The imported hardware path expects the matching `iter1-wasd/uno/tars/tars.ino` firmware
and a known serial device. It is not a Grove I2C driver implementation. Keep
wheels raised for initial direction and stop tests. Stop the service, select the
actual backend/device explicitly, then start it:

```sh
python3 iter2-autonomy/dashboard/service.py stop
TARS_MOTOR_BACKEND=uno TARS_MOTOR_DEVICE=/dev/ACTUAL_SERIAL_DEVICE \
  python3 iter2-autonomy/dashboard/service.py start
```

The placeholder must be replaced with the enumerated board's device; the service
never guesses or silently switches from stub to hardware. The phone must be
rigidly mounted and its forward direction, height, robot footprint, real stopping
distance, and LiDAR coverage must be calibrated. The current gate checks a
front corridor (0.6 m stop / 1.2 m warning), not a verified full-body swept volume.
It is an obstacle **stop** path, not autonomous route following or physical
collision-avoidance validation. The original two-wheel balancing loop is separate
and is not implemented by this bridge.

## Verification

```sh
make -C iter1-wasd/pi test                       # full native stub + fake Uno serial suite
python3 iter1-wasd/pi/test_guard_gate.py         # malformed/replayed gate, lease, stop races
python3 iter2-autonomy/edge-ai/test_robot_stop.py  # real guard + real native motor, synthetic LiDAR
python3 -m unittest discover -s iter2-autonomy/dashboard -p test_motor_link.py
```

For native QNX test binaries set `MOTOR_BINARY` as the tests describe. The actual
motor-server suite's fake Uno uses pseudo-terminals; it proves protocol bytes,
not real hardware movement. No production gate port is used by these tests.

## USB keyboard driving

Run `python3 iter2-autonomy/dashboard/service.py gui` and focus the Pi dashboard window.
Hold W for forward, S for reverse, A for an in-place left turn, D for right.
Commands use 25% wheel speed through the existing independent wheel protocol:
`D .25 .25`, `D -.25 -.25`, `D -.25 .25`, `D .25 -.25` respectively.
Release any movement key or press Space to stop; P toggles sensor streams.
Focus loss, minimization, missing native connection, or a denied LiDAR guard
clears the held command. Autorepeat cannot restart it: release and press again.
Keyboard use takes local ownership; F stops local motion and returns control
to the phone. Esc closes the window and stops the command bridge.

This requires `iter1-wasd/uno/tars/tars.ino`, not the older single-byte g/s sketch.
The default stepper build is 28BYJ-48 + ULN2003, left IN1..4 on 8,9,10,11 and
right on 4,5,6,7. It includes a 200 ms Arduino drive watchdog. Confirm wiring
and wheel direction with wheels lifted before floor testing. The guard remains
a forward-corridor stop sensor; reverse and pivot swept-volume coverage is not
validated. Motor output remains stub until the matching firmware is flashed
and an explicit `TARS_MOTOR_BACKEND=uno TARS_MOTOR_DEVICE=/dev/serusb1` service
start is performed. No firmware flasher is currently installed on the QNX Pi.

Bring-up on 2026-11-21: the Uno on Mac `/dev/cu.usbmodem101` was flashed
with the repository sketch (`arduino:avr:uno`, AVR core 1.8.8), upload verification
enabled. Build: 17882 bytes flash / 731 bytes static RAM. Serial stop + status
returned READY, wheels 0 0, move idle, no accelerometer. No physical drive test
was sent. The running Pi service was selected with backend `uno`, device
`/dev/serusb1`; this environment selection must be supplied again after stopping
or rebooting the service (an ordinary start still defaults to stub).

### Headless keyboard (no focus or monitor required)

Build `make -C iter1-wasd/pi keyboard-native`, then use `service.py headless`. The receiver
launches `keyboard_hid` via sudo (QNX HID access needs privilege). It observes
only the verified Compx 25a7:fa70 eight-byte boot keyboard interface. It neither
opens a window nor sends motor commands; Python routes WASD through MotorLink
and the independent native guard. Space/Esc stops, F returns control to phone.
No P/stream toggle is bound in headless mode; use the phone for that.

Each hold is limited to two seconds, then release and press again. This bounds
motion if the wireless link loses a release but its USB dongle remains present.
The receiver doesn't provide a verified radio-link heartbeat. USB removal,
helper failure/stale heartbeat (300 ms), conflicting keys, guard denial and
phone halt cancel the hold. Neutral input is required after connection/loss.
Initial input without a neutral report is ignored. Modifier+WASD stops.
The helper observes globally: avoid typing WASD elsewhere on the Pi while
headless keyboard control is active. It records only the control-key mask.

Phone `control_phone` cancels keyboard intent and requires a fresh keyboard
press after neutral to reclaim local control. Phone direction buttons send
this operation followed by a bounded drive pulse. Headless status now includes
`keyboard` (keys, last_press, last_keys, event_count, owner, connection state),
and motor serial device presence plus historical Arduino READY message count.
Keyboard input is reported even when the LiDAR gate refuses motor output.

### Physical keyboard obstacle override

Hold either Shift key and press one WASD direction on the Pi keyboard. Release Shift or the direction to stop. Each shifted press lasts at most two seconds. The manual bridge renews a 150 ms native lease at quarter speed. The supervisor gives only the dashboard and motor server a per-run credential for the KD command. An opted-in guard tags fresh classified obstacles with GOB1 and a zero permit byte. Ordinary drive and autonomous clients still see a denial. Missing or stale input, loss of tracking, stop, bump and tip-over still inhibit the shifted drive.
