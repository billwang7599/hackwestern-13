#!/bin/sh
set -eu
cd /data/home/qnxuser/projects/iphone-usb/pi/dashboard
mkdir -p state
# Resume on each boot, even if the previous run was manually stopped.
rm -f state/paused
# Power-loss logger (state/powerlog.log, state/power-events.log).
/usr/bin/python3 powerlog.py >>state/powerlog.err 2>&1 &
exec /usr/bin/python3 watchdog.py >>state/watchdog.log 2>&1
