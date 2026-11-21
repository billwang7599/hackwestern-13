#!/bin/sh
# Compile iter3-pet/vision/qnx/cpp/tars_gesture.cc against stand-in headers (syntax/type
# check only; the real build is on the Pi). Run from anywhere.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
c++ -std=c++17 -fsyntax-only -Wall -Wextra -Werror \
    -I "$HERE/cxx_stubs" "$HERE/../qnx/cpp/tars_gesture.cc"
echo "tars_gesture.cc: syntax/type check OK (stub headers)"
