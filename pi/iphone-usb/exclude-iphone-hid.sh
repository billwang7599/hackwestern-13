#!/bin/sh
# QNX runtime-only test. Existing USB keyboard/mouse briefly disconnect.
# Restore default behavior with: sudo sh exclude-iphone-hid.sh restore
set -eu
if [ "$(id -u)" != 0 ]; then echo 'Run with sudo' >&2; exit 1; fi
HID=$(command -v io-hid)
slay -f io-hid || true
if [ "${1:-}" = restore ]; then
    "$HID" -dusb
else
    if ! "$HID" -dusb igndev=0x05ac:0x12a8,igndev=0x2341:0x0043; then
        "$HID" -dusb
        exit 1
    fi
fi
