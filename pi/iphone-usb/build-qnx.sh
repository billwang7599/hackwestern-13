#!/bin/sh
# Run on QNX after transferring this repo's vendor/iphone-usb sources.
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
PREFIX="$ROOT/pi/iphone-usb/install"
export CC=clang CXX=clang++
export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
export LD_LIBRARY_PATH="$PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export LIBS=-lsocket
mkdir -p "$PREFIX" "$ROOT/pi/iphone-usb/logs"
if [ "$#" -eq 0 ]; then
    set -- libplist libimobiledevice-glue libusbmuxd libtatsu libimobiledevice usbmuxd
fi
for project in "$@"; do
    case "$project" in
        libplist|libimobiledevice-glue|libusbmuxd|libtatsu|libimobiledevice|usbmuxd) ;;
        *) printf 'Unknown project: %s\n' "$project" >&2; exit 1 ;;
    esac
    case "$project" in
        libplist) options='--without-cython --without-tests' ;;
        libusbmuxd) options='--without-inotify' ;;
        libimobiledevice) options='--without-cython --without-readline --disable-wireless-pairing --with-openssl' ;;
        usbmuxd) options='--without-systemd --with-systemdsystemunitdir=no' ;;
        *) options='' ;;
    esac
    printf 'Building %s\n' "$project"
    (
        cd "$ROOT/vendor/iphone-usb/$project"
        NOCONFIGURE=1 ./autogen.sh &&
        ./configure --prefix="$PREFIX" $options &&
        make -j4 &&
        make install
    ) > "$ROOT/pi/iphone-usb/logs/$project.log" 2>&1 || {
        tail -60 "$ROOT/pi/iphone-usb/logs/$project.log"
        exit 1
    }
done
printf 'Installed iPhone USB tools in %s\n' "$PREFIX"
