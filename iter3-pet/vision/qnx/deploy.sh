#!/bin/sh
# Run on the Mac: copy vision/ (code, config, model) to the Pi.
#   iter3-pet/vision/qnx/deploy.sh                       # -> qnxuser@tars.local:/data/home/qnxuser/vision
#   PI_HOST=qnxuser@192.168.1.42 iter3-pet/vision/qnx/deploy.sh
# Uses tar over ssh (QNX images don't always have rsync).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
VISION="$(cd "$HERE/.." && pwd)"
PI_HOST="${PI_HOST:-qnxuser@tars.local}"
PI_DIR="${PI_DIR:-/data/home/qnxuser}"
"$VISION/get_model.sh" >/dev/null
echo ">> copying vision/ to $PI_HOST:$PI_DIR/vision"
COPYFILE_DISABLE=1 tar -C "$VISION/.." -cf - \
    --exclude 'vision/.venv' --exclude 'vision/tests/build' --exclude 'vision/tests/assets' \
    --exclude 'vision/learned.json*' --exclude 'vision/qnx/autostart/tars.env' \
    --exclude '__pycache__' --exclude '*.pyc' --exclude '.DS_Store' vision \
  | ssh "$PI_HOST" "mkdir -p '$PI_DIR' && tar -C '$PI_DIR' -xf -"
echo ">> done. On the Pi:"
echo "   cd $PI_DIR/vision && sh qnx/setup_pi.sh          # once"
echo "   python3 qnx/python/tars_gesture.py --dry-run      # then drop --dry-run"
