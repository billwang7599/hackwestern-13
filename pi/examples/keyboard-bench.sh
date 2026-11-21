#!/bin/sh
# Run on the Pi as qnxuser. F exits and returns the Uno to the dashboard.
set -eu
cd "$(dirname "$0")/.."
root=$(pwd)
state=$root/dashboard/state
scratch=$(mktemp -d /tmp/wasd-bench.XXXXXX)
bridge=
hid=
cleanup() {
    if [ -n "$hid" ]; then sudo -n kill -TERM "$hid" 2>/dev/null || true; wait "$hid" || true; fi
    if [ -n "$bridge" ]; then sudo -n kill -TERM "$bridge" 2>/dev/null || true; wait "$bridge" || true; fi
    rm -f "$scratch/input"
    rmdir "$scratch"
    rm -f "$state/paused"
}
[ -x build/serial_move ] && [ -x build/keyboard_hid ] || exit 1
python3 dashboard/service.py status >/dev/null
trap cleanup EXIT
trap 'exit 1' INT TERM HUP
python3 dashboard/service.py stop >/dev/null
i=0
while sudo -n pidin ar | grep -Eq '[t]ars_motor |[u]no_bridge.py|[s]ervice.py run'; do
    i=$((i+1))
    [ "$i" -lt 30 ] || exit 1
    sleep 1
done
sudo -n python3 iphone-usb/uno_bridge.py &
bridge=$!
i=0
while [ ! -c "$state/uno-serial" ]; do
    i=$((i+1))
    [ "$i" -lt 15 ] || exit 1
    sleep 1
done
mkfifo "$scratch/input"
sudo -n build/keyboard_hid >"$scratch/input" &
hid=$!
build/serial_move "$state/uno-serial" --wasd <"$scratch/input"
