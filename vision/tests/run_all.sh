#!/bin/sh
# All Mac-side checks for vision/ (no camera, no robot).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
VISION="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$VISION/.." && pwd)"
PY="$VISION/.venv/bin/python"
[ -x "$PY" ] || { echo "make vision/.venv first (see vision/README.md)"; exit 1; }
export MPLCONFIGDIR="${MPLCONFIGDIR:-$HERE/build/mpl}"
echo "== unit";        python3 "$HERE/test_gesture.py"
echo "== learn";       python3 "$HERE/test_learn.py"
echo "== parity";      python3 "$HERE/test_parity.py"
echo "== runtime teach"; "$PY" "$HERE/test_runtime_teach.py"
echo "== bench";       python3 "$HERE/bench_learn.py"
echo "== c++ syntax";  sh "$HERE/check_cpp_syntax.sh"
echo "== server";      make -C "$REPO/pi" test
echo "== end-to-end";  "$PY" "$HERE/e2e_desktop.py"
