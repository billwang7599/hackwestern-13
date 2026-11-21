#!/bin/sh
set -eu
cd /data/home/qnxuser/projects/iphone-usb/pi/dashboard
mkdir -p state
# Resume on each boot, even if the previous run was manually stopped.
rm -f state/paused
exec /usr/bin/python3 watchdog.py >>state/watchdog.log 2>&1
