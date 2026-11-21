#!/bin/sh
# Runtime-only: reserve this iPhone for usbmuxd. No boot files are changed.
# Restore with: sudo sh configure-runtime.sh restore
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ "$(id -u)" != 0 ]; then echo 'Run with sudo' >&2; exit 1; fi
if [ "${1:-}" = restore ]; then
    sh "$HERE/exclude-iphone-hid.sh" restore
    slay -f usblauncher_otg || true
    usblauncher_otg -0 -l
else
    test -f "$HERE/rules-iphone.lua"
    sh "$HERE/exclude-iphone-hid.sh"
    slay -f usblauncher_otg || true
    if ! usblauncher_otg -0 -l -c "$HERE/rules-iphone.lua"; then
        usblauncher_otg -0 -l
        sh "$HERE/exclude-iphone-hid.sh" restore
        exit 1
    fi
fi
