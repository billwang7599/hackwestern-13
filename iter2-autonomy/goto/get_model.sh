#!/bin/sh
# Build the detector model on the LAPTOP (macOS/Linux), then deploy it to the Pi.
#   iter2-autonomy/goto/get_model.sh                 YOLO-World, classes from iter2-autonomy/goto/objects.txt, 320 px
#   iter2-autonomy/goto/get_model.sh --imgsz 416     any export_world.py option
#   iter2-autonomy/goto/get_model.sh --coco yolo11n.pt
# Makes iter2-autonomy/goto/models/.venv (ultralytics + CPU torch, ~1 GB) the first time.
# Models are never committed (see .gitignore); everyone runs this instead.
set -e
here=$(cd "$(dirname "$0")" && pwd)
venv="$here/models/.venv"
mkdir -p "$here/models"
if [ ! -x "$venv/bin/python" ]; then
    if command -v uv >/dev/null 2>&1; then
        uv venv -p 3.12 "$venv"
        VIRTUAL_ENV="$venv" uv pip install ultralytics onnx onnxslim "git+https://github.com/ultralytics/CLIP.git"
    else
        python3 -m venv "$venv"
        "$venv/bin/pip" install ultralytics onnx onnxslim "git+https://github.com/ultralytics/CLIP.git"
    fi
fi
"$venv/bin/python" "$here/export_world.py" "$@"
