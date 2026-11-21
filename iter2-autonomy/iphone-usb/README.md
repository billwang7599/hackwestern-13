# QNX iPhone USB transport bring-up

## Arduino USB serial bring-up

The attached Uno R3 enumerates as `2341:0043`, serial `95931323132351110242`.
This Pi image has no `devc-serusb` binary; the available `qnx-devc` package
contains the PL011 UART driver, not the USB serial driver.

The existing `uno_bridge.py` uses this project's patched libusb and exposes
a raw PTY through `pi/dashboard/state/uno-serial` at fixed CDC 115200 8N1.
The runtime launcher/HID exclusions reserve this Uno as well as the phone.
The service supervisor starts the bridge when the Uno backend and that
project-local serial path are selected. Do not start a second USB claimant:
`LIBUSB_ERROR_BUSY` is expected while the running bridge owns the device.

After reconnecting the Arduino, the motor process retained an unusable old
PTY and reported a full TX buffer. Restarting the motor process and bridge
restored communication. A fresh `S` / `?` exchange returned
`# arms 0 0 tilt 0 tipped 0 accel none wheels 0 0 move -` through the motor log.
This verifies a USB serial round trip with stopped wheels, not physical movement
or reliable unattended hotplug recovery. Reconnection handling still needs work.

## Phone transport

Goal: iPhone app -> USB multiplexing -> QNX localhost TCP -> sensor receiver.
USB enumeration alone does not prove that app data can flow.

The QNX `libusb` and `libusb-dev` packages are installed with `sudo apk add`.
The `qnx-usb-dev` package supplies headers for the patched native backend.
`curl-dev` supplies libtatsu's build dependency. Installing curl-dev also updated
curl/libcurl and curl-doc through the configured QNX repository.

Upstream sources are retained locally in `vendor/iphone-usb/` (gitignored).
`sources.lock` records their exact commits and URLs. No credentials are stored.
Our scripts and application code are tracked separately from upstream sources.

Recreate the source directories with `python3 pi/iphone-usb/fetch-sources.py`.
This checks pinned commits and applies the recorded QNX patch. Make a source
archive without .git directories or macOS extended attributes. Transfer sources plus
this directory while preserving their paths. Run on QNX:

```sh
sh pi/iphone-usb/build-qnx.sh
sh pi/iphone-usb/build-libusb-qnx.sh
```

Installation is under `pi/iphone-usb/install/` in the transferred project;
build output is under `pi/iphone-usb/logs/`. No boot service is configured.
The build script uses native clang, not the repo's Docker cross-compile workflow.

Compile the read-only USB probe:

```sh
clang pi/iphone-usb/probe.c -o pi/iphone-usb/probe $(pkg-config --cflags --libs libusb-1.0)
sudo ./pi/iphone-usb/probe
```

## Run on the Pi

From `/data/home/qnxuser/projects/iphone-usb`:

```sh
sudo sh pi/iphone-usb/configure-runtime.sh
sudo sh pi/iphone-usb/run-usbmuxd.sh
```

The first command restarts the HID driver and USB launcher with a specific
exception for Apple 05ac:12a8. Keyboard/mouse input briefly interrupts. The
launcher uses a local rules copy; system configuration and boot files are not
modified. The second command runs in the foreground; Ctrl-C stops it.

In a second SSH terminal, from the same directory:

```sh
pi/iphone-usb/install/bin/idevicepair validate
# Only if not already paired: idevicepair pair, then accept Trust on the phone.
pi/iphone-usb/install/bin/iproxy -l -s 127.0.0.1 5557:5557
```

Open MotionUSB on the iPhone and tap Start. In a third terminal:

```sh
python3 pi/iphone-usb/receive_motion.py --seconds 10
```

The receiver connects only to localhost and times out after one second of
silence. It is a diagnostic, not a motor safety controller. The iPhone app source
and signing instructions are in `iphone/`.

Observed on this image: the packaged libusb backend crashed in an unprivileged
probe. The same probe under sudo successfully opened Apple device 05ac:12a8 and
read configuration 2. USB access must run with appropriate privileges; the
receiver and iPhone app do not need elevated privileges.

## Verified status and limitations

- Native builds installed libplist, libimobiledevice-glue, libusbmuxd/iproxy,
  libtatsu, libimobiledevice/idevicepair, and usbmuxd in the project prefix.
- Patched libusb successfully exchanges USB bulk data with the iPhone.
- Pairing was accepted and `idevicepair validate` succeeded.
- `ideviceinfo -k ProductType` returned `iPhone18,2` over this USB transport.
- MotionUSB compiled, signed, and installed from the Mac; the user trusted the
  developer profile and started the app before connecting the phone to the Pi.
- Direct USB sensor streaming passed: the QNX receiver read 1,005 samples in
  10.04 seconds (about 100 Hz), with zero skipped sequence numbers. Acceleration
  was approximately [−0.2, −0.4, −9.8] m/s² with small gyro readings while the
  phone was mostly stationary. This verifies transport, not absolute latency,
  dynamic orientation accuracy, or suitability for closed-loop balancing.

QNX-specific fixes are recorded in the two `.patch` files. The backend now
claims the selected configuration explicitly and ignores removal notifications
for an old configuration during configuration switching. The daemon selects
the plain Apple data configuration (3) for this iPhone. Diagnostic logging is
retained at info level.

Automatic discovery is disabled (`-n`) because this older QNX libusb backend
leaks attachment references during polling; it caused a failure during the
first pairing attempt. **Stop the daemon before unplugging, and restart it after
reconnecting.** This is a bring-up workaround, not a production-quality driver.
Disconnect/recovery and long-running stability need more testing.

Pairing records stay on the Pi under `pi/iphone-usb/state/`, excluded from git.
To restore the original runtime USB behavior after stopping usbmuxd:

```sh
sudo sh pi/iphone-usb/configure-runtime.sh restore
```

The package-provided libusb remains installed; the patched library lives only
in the project prefix and is selected by the daemon launcher's library path.
