#!/bin/sh
# Native build on QNX. No changes to global libraries or toolchain.
set -eu
cd "$(dirname "$0")"
ORT_INCLUDE=${ORT_INCLUDE:-/usr/include/onnxruntime}
if [ ! -f "$ORT_INCLUDE/onnxruntime_cxx_api.h" ]; then
    for candidate in /usr/include/onnxruntime/core/session /usr/local/include/onnxruntime /usr/include; do
        if [ -f "$candidate/onnxruntime_cxx_api.h" ]; then ORT_INCLUDE=$candidate; break; fi
    done
fi
if [ ! -f "$ORT_INCLUDE/onnxruntime_cxx_api.h" ]; then
    echo 'ONNX Runtime headers not found; set ORT_INCLUDE to the installed QNX header directory.' >&2
    exit 1
fi
mkdir -p build
${CXX:-clang++} -std=c++17 -O2 -Wall -Wextra -Wpedantic -I"$ORT_INCLUDE" \
    ai_detector.cpp -o build/ai_detector -lonnxruntime -ljpeg -pthread

${CXX:-clang++} -std=c++17 -O2 -Wall -Wextra -Wpedantic -I"$ORT_INCLUDE" \
    ai_depth.cpp -o build/ai_depth -lonnxruntime -ljpeg -pthread
