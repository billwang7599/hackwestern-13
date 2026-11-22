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

# detect (YOLO-World on the iPhone camera via rgb_broker 5566): needs QNX's ONNX Runtime and
# libjpeg-turbo packages (sh iter2-autonomy/edge-ai/setup-qnx.sh). Skipped, not fatal, without them.
ORT_INCLUDE=${ORT_INCLUDE:-/usr/include/onnxruntime}
if [ ! -f "$ORT_INCLUDE/onnxruntime_cxx_api.h" ]; then
    for candidate in /usr/include/onnxruntime/core/session /usr/local/include/onnxruntime /usr/include; do
        if [ -f "$candidate/onnxruntime_cxx_api.h" ]; then ORT_INCLUDE=$candidate; break; fi
    done
fi
if [ -f "$ORT_INCLUDE/onnxruntime_cxx_api.h" ]; then
    ${CXX:-clang++} -std=gnu++17 -O2 -Wall -Wextra -D_QNX_SOURCE -DHAVE_ORT -DHAVE_JPEG -I"$ORT_INCLUDE" \
        src/detect.cpp -o build/detect -lonnxruntime -ljpeg -lsocket -pthread
    echo 'Built native QNX detect: build/detect (model: run get_model.sh on the laptop, copy models/ here)'
else
    echo 'detect skipped: ONNX Runtime headers not found (run iter2-autonomy/edge-ai/setup-qnx.sh, or set ORT_INCLUDE)' >&2
fi
