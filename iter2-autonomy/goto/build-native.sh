#!/bin/sh
# Native QNX image toolchain; preserves make/qcc cross-build as the SDP path.
set -eu
cd "$(dirname "$0")"
test "$(uname -s)" = QNX || { echo 'Run on the QNX board' >&2; exit 1; }
mkdir -p build
clang -std=gnu11 -O2 -Wall -Wextra -D_QNX_SOURCE \
    src/goto.c src/goto_core.c src/nav.c -o build/goto -lsocket -lm
echo 'Built native QNX goto: build/goto'
clang -shared -fPIC -O2 -Wall -Wextra loopback_bind.c -o build/loopback_bind.so
