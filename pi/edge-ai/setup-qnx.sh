#!/bin/sh
# Install QNX-packaged native inference headers/runtime; no system-wide upgrade.
set -eu
[ "$(uname -s)" = QNX ] || { echo 'Run this setup on QNX' >&2; exit 1; }
sudo -n apk add onnxruntime onnxruntime-dev libjpeg-turbo-dev
