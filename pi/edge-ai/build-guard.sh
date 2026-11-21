#!/bin/sh
set -eu
cd "$(dirname "$0")"
mkdir -p build
if [ "$(uname -s)" = QNX ]; then
    clang++ -std=c++17 -O2 -Wall -Wextra -pthread lidar_guard.cpp -lsocket -o build/lidar_guard
else
    c++ -std=c++17 -O2 -Wall -Wextra -pthread lidar_guard.cpp -o build/lidar_guard
fi
