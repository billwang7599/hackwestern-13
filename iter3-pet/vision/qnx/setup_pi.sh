#!/bin/sh
# Run ON the Pi (QNX 8.0.5 QSTI image with apk), once, from vision/ (deploy.sh puts it at /data/home/qnxuser/vision):
#   sh qnx/setup_pi.sh
# Installs QNX's prebuilt MediaPipe Python package, fetches QNX's camera
# helper from the official codelab, the hand model, and checks imports.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
VISION="$(cd "$HERE/.." && pwd)"

if ! command -v apk >/dev/null 2>&1; then
    echo "!! no 'apk' on this image. The prebuilt MediaPipe package needs the QNX 8.0.5"
    echo "!! QSTI Pi 5 image (QNX Developer Desktop / Everywhere). Either reflash with it,"
    echo "!! or use the laptop fallback (README: 'Fallback')."
    exit 1
fi
echo ">> apk add mediapipe (QNX prebuilt, includes OpenCV + NumPy)"
sudo apk update
sudo apk add mediapipe python3-pip curl

echo ">> QNX camera helper (from qnx/codelabs: mediapipe-camera-python)"
mkdir -p "$HERE/python/vendor"
TMP="$HERE/python/vendor/companion.tar.gz"
curl --fail --location \
  https://raw.githubusercontent.com/qnx/codelabs/main/markdown/mediapipe-camera-python/mediapipe-camera-python-companion.tar.gz \
  --output "$TMP"
tar -xzf "$TMP" -C "$HERE/python/vendor" qnx_camera_helper.py
rm -f "$TMP"

echo ">> model"
sh "$VISION/get_model.sh"

echo ">> import check"
python3 -c 'import cv2, mediapipe, numpy; print("OpenCV", cv2.__version__, "MediaPipe", mediapipe.__version__, "NumPy", numpy.__version__)'
python3 -c 'from mediapipe.tasks.python import vision; print("HandLandmarker OK:", vision.HandLandmarker)'

echo ">> camera units"
ls /dev/sensor/ 2>/dev/null || true
cd "$HERE/python" && python3 tars_gesture.py --list-units || true
echo ">> setup done. Next: python3 $HERE/python/tars_gesture.py --dry-run --save-every 30"
