#!/usr/bin/env python3
"""Export the detector model for detect (QNX). Runs on the LAPTOP, not the robot.

    python3 goto/export_world.py                      # YOLO-World with goto/objects.txt classes
    python3 goto/export_world.py --imgsz 416          # bigger input: farther objects, slower
    python3 goto/export_world.py --coco yolo11n.pt    # plain COCO model instead (faster, 80 classes,
                                                      # no "trash can"; COCO labels map via aliases)

Writes goto/models/yolo_world.onnx + goto/models/labels.txt (one class per line,
same order as the ONNX outputs). `make -C goto deploy` copies both to the Pi.

YOLO-World is open-vocabulary, but running its text encoder on the Pi would be
far too slow. set_classes() bakes the CLIP text embeddings for OUR list into
the network, so the exported ONNX is a normal fixed-class detector, about as
fast as plain YOLOv8s at the same input size. Re-run after changing the
canonical names in objects.txt (aliases don't need a re-export).

Needs: pip install ultralytics onnx onnxslim "git+https://github.com/ultralytics/CLIP.git"
(goto/get_model.sh makes a venv and does all of this.)
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def canonical_names(path: str) -> list[str]:
    """Left of ':' on each non-comment line, like goto_core.c."""
    names = []
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            names.append(" ".join(line.split(":", 1)[0].lower().split()))
    return names


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--objects", default=os.path.join(HERE, "objects.txt"))
    ap.add_argument("--base", default="yolov8s-worldv2.pt", help="YOLO-World weights (v2 exports to ONNX)")
    ap.add_argument("--coco", metavar="WEIGHTS", help="export a plain COCO model (e.g. yolo11n.pt) instead")
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--opset", type=int, default=17)
    ap.add_argument("--out", default=os.path.join(HERE, "models"))
    ap.add_argument("--workdir", default=None, help="where weights are downloaded (default: <out>/work)")
    ap.add_argument("--pt", action="store_true", help="also save the custom-class .pt (laptop testing)")
    a = ap.parse_args()

    from ultralytics import YOLO, YOLOWorld  # heavy import, after argparse so --help is fast

    os.makedirs(a.out, exist_ok=True)
    work = os.path.abspath(a.workdir or os.path.join(a.out, "work"))
    os.makedirs(work, exist_ok=True)
    os.chdir(work)                         # ultralytics downloads weights into the cwd

    if a.coco:
        model = YOLO(a.coco)
        names = [model.names[i] for i in range(len(model.names))]
        print(f"COCO model {a.coco}: {len(names)} classes")
    else:
        names = canonical_names(a.objects)
        if not names:
            sys.exit(f"no classes in {a.objects}")
        model = YOLOWorld(a.base)
        model.set_classes(names)
        print(f"YOLO-World {a.base} with {len(names)} classes: {', '.join(names)}")
        if a.pt:
            pt = os.path.join(a.out, "yolo_world_custom.pt")
            model.save(pt)
            print(f"saved {pt}")

    onnx = model.export(format="onnx", imgsz=a.imgsz, opset=a.opset, simplify=True, dynamic=False)
    dst = os.path.join(a.out, "yolo_world.onnx")
    shutil.copyfile(onnx, dst)
    with open(os.path.join(a.out, "labels.txt"), "w") as f:
        f.write("\n".join(names) + "\n")
    print(f"wrote {dst} ({os.path.getsize(dst) / 1e6:.1f} MB, input {a.imgsz}x{a.imgsz}) and labels.txt")
    print("next: make -C goto deploy   (copies goto, detect, the model and labels to the Pi)")


if __name__ == "__main__":
    main()
