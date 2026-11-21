#!/bin/sh
# Download the MediaPipe hand landmarker model (Apache 2.0, ~7.8 MB) into
# vision/models/. Works on macOS and on the QNX Pi.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
URL=https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
SHA=fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1
OUT="$HERE/models/hand_landmarker.task"
mkdir -p "$HERE/models"
[ -s "$OUT" ] || curl --fail --location "$URL" --output "$OUT"
if command -v sha256sum >/dev/null 2>&1; then
    printf '%s  %s\n' "$SHA" "$OUT" | sha256sum -c -
elif command -v shasum >/dev/null 2>&1; then
    printf '%s  %s\n' "$SHA" "$OUT" | shasum -a 256 -c -
else
    echo "(no sha256 tool; skipped checksum)"
fi
