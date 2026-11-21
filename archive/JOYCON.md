# Right Joy-Con IMU relay

Original Switch Joy-Con (R) -> Bluetooth -> Mac Python -> UDP 5556 -> QNX C++.
This is a telemetry bring-up tool. It does not command motors or balance the robot.
Joy-Con 2 uses a different protocol and is not supported by this implementation.

## Pair on the Mac

1. Charge the right Joy-Con and detach it from the Switch. Close Steam or other controller apps.
2. Hold the small SYNC button on the inner rail until the player lights cycle.
3. Open System Settings -> Bluetooth and connect **Joy-Con (R)**.
4. Keep the Mac and Pi on a network that allows devices to communicate.
5. From this repository root:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r iter1-wasd/laptop/requirements-joycon.txt
.venv/bin/python iter1-wasd/laptop/joycon_relay.py --list
```

If discovery is empty, reconnect the controller, check its battery, and close other controller software.
If macOS requests Input Monitoring access for your terminal, grant it in
System Settings -> Privacy & Security -> Input Monitoring and restart the terminal.
Do not run the relay as root. Pairing alone does not prove raw HID access works;
the relay checks setup acknowledgements and exits if IMU data stops for a second.

## Compile and run on QNX

All sources live in this repo. Copy the saved source and compile it on the Pi:

```sh
ssh qnxuser@172.23.43.185 'mkdir -p projects/joycon-imu'
scp iter1-wasd/pi/src/imu_receiver.cpp qnxuser@172.23.43.185:projects/joycon-imu/
ssh qnxuser@172.23.43.185 'clang++ -std=c++17 -Wall -Wextra -O2 projects/joycon-imu/imu_receiver.cpp -o projects/joycon-imu/imu_receiver -lsocket'
ssh -t qnxuser@172.23.43.185 './projects/joycon-imu/imu_receiver'
```

In another Mac terminal, first test the network without the controller:

```sh
.venv/bin/python iter1-wasd/laptop/joycon_relay.py --simulate --seconds 3
```

Then relay real readings:

```sh
.venv/bin/python iter1-wasd/laptop/joycon_relay.py
```

The receiver prints accelerometer and gyro **raw signed counts**, roughly four
times per second. Tilt and rotate the controller: the numbers should change.
Ctrl-C stops each program. `--host` and `--port` override the relay destination;
the receiver takes an optional port argument. A silence longer than 250 ms prints
STALE. This is a diagnostic message, not an integrated motor watchdog.

## Data and limitations

Each full HID report contains three IMU samples. The relay forwards all three as
individual ASCII UDP datagrams:

```text
IMU1 sequence mac_monotonic_ns device_timer sample_index ax ay az gx gy gz
```

Sequence counts samples from zero per relay run; sample_index is 0..2. The three
samples share a Mac receipt timestamp, not a sensor acquisition timestamp.
Mac and Pi monotonic clocks are unrelated; subtracting them does not measure latency.
Values are uncalibrated sensor counts, not g or degrees/second.
The receiver validates fields and ranges but does not authenticate senders, reorder
packets, or reject duplicates. Use it on a trusted network for telemetry only.

Before balancing: rigidly mount the sensor to the chassis, calibrate sensor scale
and gyro bias, map the mounting axes, measure end-to-end delay/jitter, and integrate
a stale-data motor stop. A handheld Joy-Con measures your hand, not robot tilt.
Bluetooth plus Wi-Fi introduces variable delay; the wired IMU should provide the
feedback for the final balancing loop on the Pi.

Python `hidapi` keeps this bring-up small and gives explicit report freshness.
`joycon-python` is another option, but its published environment is much older;
we use the HID transport directly here.

Sources: [Joy-Con protocol](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_notes.md),
[IMU notes](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/imu_sensor_notes.md),
[joycon-python](https://github.com/tocoteron/joycon-python),
[Apple Bluetooth controller setup](https://support.apple.com/en-gb/111099).

## Local verification

```sh
python3 -m unittest discover -s laptop -p 'test_joycon_relay.py'
clang++ -std=c++17 -Wall -Wextra -Werror iter1-wasd/pi/src/imu_receiver.cpp -o /tmp/hw13-imu-receiver
python3 iter1-wasd/pi/test_imu_receiver.py /tmp/hw13-imu-receiver
```
