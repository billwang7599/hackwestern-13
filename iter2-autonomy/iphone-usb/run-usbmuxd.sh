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
# sudo can exit before its USB child. Reap a leftover project daemon
# before the supervisor replaces it, checking both PID and executable.
pidfile="$HERE/state/usbmuxd.pid"
old=$(cat "$pidfile" 2>/dev/null || true)
case "$old" in
    ''|*[!0-9]*) ;;
    *) if pidin -p "$old" ar 2>/dev/null | awk -v p="$old" -v exe="$HERE/install/sbin/usbmuxd" '$1==p && $2==exe {found=1} END {exit !found}'; then
           kill -TERM "$old" 2>/dev/null || true
           sleep 1
           if pidin -p "$old" ar 2>/dev/null | awk -v p="$old" -v exe="$HERE/install/sbin/usbmuxd" '$1==p && $2==exe {found=1} END {exit !found}'; then
               kill -KILL "$old" 2>/dev/null || true
           fi
       fi ;;
esac
exec "$HERE/install/sbin/usbmuxd" -n -v -P "$HERE/state/usbmuxd.pid" -C "$HERE/state/pairing"
