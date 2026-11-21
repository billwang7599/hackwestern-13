#!/bin/bash
# Build the C++ tars_gesture ON the Pi (QNX 8.0.5 QSTI image), following QNX's
# "MediaPipe Camera Sample" codelab:
#   https://qnx.github.io/codelabs/mediapipe-camera-sample/
# MediaPipe for QNX builds self-hosted with Bazel on the Pi, NOT in our Docker
# cross toolchain (the qnx-ports tree carries its own QNX Bazel toolchains).
#
#   cd ~/vision && bash qnx/cpp/build_on_pi.sh          # MP_DIR=~/mediapipe by default
#
# Expect a LONG first build (MediaPipe + TFLite + OpenCV deps from source on a
# Pi 5: plan on hours, not minutes). Start it early, in tmux/screen if you have
# it, and use the Python app (qnx/python) meanwhile. BAZEL_JOBS=2 if it runs
# out of memory.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
VISION="$(cd "$HERE/../.." && pwd)"
MP_DIR="${MP_DIR:-$HOME/mediapipe}"
BAZEL_JOBS="${BAZEL_JOBS:-3}"

echo ">> deps (codelab: bazel6 cmake ninja opencv-dev qnx-sf-base-dev)"
sudo apk add bazel6 cmake ninja opencv-dev qnx-sf-base-dev

if [ ! -d "$MP_DIR/.git" ]; then
    echo ">> cloning qnx-ports/mediapipe (qnx-v0.10.26) into $MP_DIR"
    git clone https://github.com/qnx-ports/mediapipe.git --branch qnx-v0.10.26 "$MP_DIR"
fi

# Codelab: Clang 22 resource dir must exist where the build expects it.
if [ -d /usr/lib/llvm22/lib/clang/22 ] && [ ! -e /usr/lib/clang/22 ]; then
    echo ">> linking /usr/lib/clang/22 (codelab step)"
    sudo mkdir -p /usr/lib/clang
    sudo ln -s /usr/lib/llvm22/lib/clang/22 /usr/lib/clang/22
fi

APP="$MP_DIR/mediapipe/examples/qnx/tars_gesture"
echo ">> installing our app into $APP"
mkdir -p "$APP"
cp "$HERE/BUILD" "$HERE/tars_gesture.cc" "$APP/"
cp "$VISION/common/gesture.hpp" "$VISION/common/learn.hpp" "$APP/"

# QNX's build_qnx_examples.sh creates the QNX Bazel toolchains, then asks y/n
# for every example dir. Answer "y" only for ours (same glob order as the script).
cd "$MP_DIR"
answers=""
for app in mediapipe/examples/qnx/*; do
    [ -d "$app" ] || continue
    name="${app##*/}"
    case "$name" in autoflip|hello_world|media_sequence|object_detection_3d|template_matching) continue ;; esac
    if [ "$name" = tars_gesture ]; then answers="${answers}y\n"; else answers="${answers}n\n"; fi
done
echo ">> building mediapipe/examples/qnx/tars_gesture:tars_gesture_cpu (jobs=$BAZEL_JOBS)"
printf "$answers" | BAZEL_JOBS="$BAZEL_JOBS" ./build_qnx_examples.sh -d "$VISION/qnx/cpp"

BIN="$VISION/qnx/cpp/tars_gesture_cpu"
ls -l "$BIN"
cat <<EOM

>> built $BIN
   run (tars_motor must be running; --dry_run to just print):
   $BIN --model=$VISION/models/hand_landmarker.task --conf=$VISION/gesture.conf --dry_run
EOM
