#!/bin/sh
# Run with sudo on QNX. Foreground, manual start only; Ctrl-C stops it.
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ "$(id -u)" != 0 ]; then
    echo 'Run this script with sudo for QNX USB access.' >&2
    exit 1
fi
export LD_LIBRARY_PATH="$HERE/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
mkdir -p "$HERE/state"
chmod 700 "$HERE/state"
# QNX's older libusb backend leaks attachment references during polling.
# Enumerate once; restart this daemon after reconnecting the phone.
exec "$HERE/install/sbin/usbmuxd" -n -v -P "$HERE/state/usbmuxd.pid" -C "$HERE/state/pairing"
