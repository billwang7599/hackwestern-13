#!/bin/sh
# Hardware in the loop, before the Pi/QNX works: the host tars_motor (the same C as
# on the Pi) on this Mac, talking to the REAL Uno on the Mac's USB. Everything that
# talks to tars_motor (bench, teleop.py, wheel_cal.py, goto host build) then drives
# the real car exactly as it would through the Pi.
#
#   bench/car/hil.sh                  # finds the Uno, tars_motor on 127.0.0.1:5555
#   PORT=5600 DEV=/dev/cu.usbserial-1410 bench/car/hil.sh
#   then: python3 bench/bench.py --target usb      (or iter1-wasd/laptop/teleop.py)
#
# !! UNTESTED: written without the car or an Uno on hand. First run: wheels in the air.
# !! SM11: unplug the kit's Bluetooth module (D0/D1 = this USB serial link).
#
# Which device:
#   genuine Uno R3 (16U2)          /dev/cu.usbmodem*
#   CH340 clone (most kit boards)  /dev/cu.usbserial-*     (Apple's built-in CH34x driver, macOS 11+)
#                                  /dev/cu.wchusbserial*   (if WCH's own driver is installed)
#   Nothing appears for a CH340 -> try another cable (charge-only cables are common), then
#   WCH's driver from wch-ic.com (needs a reboot + "allow" in System Settings > Privacy & Security).
# Use cu.*, not tty.*. Close the Arduino IDE serial monitor first (one opener at a time).
# Opening the port resets the Uno (DTR): READY comes ~1.5 s later, as on the Pi.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
PORT=${PORT:-5555}
WD=${WD:-500}

if [ -z "$DEV" ]; then
  for pat in '/dev/cu.usbmodem*' '/dev/cu.usbserial*' '/dev/cu.wchusbserial*'; do
    for d in $pat; do [ -e "$d" ] && DEV=$d && break 2; done
  done
fi
if [ -z "$DEV" ] || [ ! -e "$DEV" ]; then
  echo "no Uno found (looked for /dev/cu.usbmodem*, cu.usbserial*, cu.wchusbserial*)." >&2
  echo "plug it in, or set DEV=/dev/cu.<name>; see the CH340 notes at the top of $0" >&2
  ls /dev/cu.* 2>/dev/null >&2 || true
  exit 1
fi

make -s -C "$REPO/iter1-wasd/pi" host >/dev/null
echo "tars_motor (host build) -b uno -d $DEV on 127.0.0.1:$PORT, watchdog ${WD} ms. Ctrl-C stops the motors."
echo "check: STATUS shows ready=1 wheels=dc within ~2 s (python3 -c 'import socket;s=socket.create_connection((\"127.0.0.1\",$PORT));s.sendall(b\"STATUS\\n\");print(s.recv(4096).decode())')"
# -l: listen on localhost only. -g 0: no iPhone guard gate (there is no phone in this setup).
exec "$REPO/iter1-wasd/pi/build/tars_motor_host" -b uno -d "$DEV" -p "$PORT" -w "$WD" -r 0 -g 0 -l
