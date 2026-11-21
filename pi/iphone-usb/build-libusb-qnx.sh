#!/bin/sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
PREFIX="$ROOT/pi/iphone-usb/install"
cd "$ROOT/vendor/iphone-usb/libusb"
NOCONFIGURE=1 ./autogen.sh
CC=clang CXX=clang++ LIBS='-lusbdi -lsocket' CFLAGS='-g -O1' ./configure --prefix="$PREFIX" --disable-static
make -j4
make install
