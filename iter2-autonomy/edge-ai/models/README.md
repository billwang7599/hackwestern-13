# QNX RGB detector artifacts

`yolox_nano.onnx` is the 416×416 YOLOX-Nano checkpoint published by
[Megvii's official YOLOX ONNX Runtime demo](https://github.com/Megvii-BaseDetection/YOLOX/blob/main/demo/ONNXRuntime/README.md),
release `0.1.1rc0`. It is about 3.5 MiB, recognizes the 80 COCO classes, and is
separate from the phone's earlier Core ML YOLOv3 Tiny model.

Run `python3 iter2-autonomy/edge-ai/models/fetch-model.py` from the repo root to download or
verify the files. Existing mismatched files are never silently overwritten.

| File | SHA-256 |
|---|---|
| yolox_nano.onnx | `c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d` |
| dog.jpg | `5a9522051c3cec2bbd2f6323fccba32e8fbf3ddcc2b3e2fd46b04c720bc6f866` |

The sample `dog.jpg` is the upstream YOLOX test image; its URL is pinned by
checksum even though the upstream path uses `main`.

The [YOLOX source repository](https://github.com/Megvii-BaseDetection/YOLOX) is
Apache-2.0 licensed. The model and image come from the official project's
published artifacts; this document does not assign a separate inferred license
to pretrained weights or the photograph. See upstream licensing and attribution
before redistributing artifacts for another purpose.

## QNX AI module and model contract

Inference is implemented by **QNX's port of ONNX Runtime**, not by the iPhone,
a Linux process, or a simulated detector. QNX maintains
[its ONNX Runtime port](https://github.com/qnx-ports/onnxruntime) and
[build instructions](https://github.com/qnx-ports/build-files/tree/main/ports/onnxruntime).
ONNX Runtime is MIT licensed upstream. The deployed APK version and target test
results should be recorded alongside the demo; availability of the package alone
does not prove that our model executed on QNX.

The C++ worker uses fixed float input `[1,3,416,416]`, top-left letterboxing with
114 padding, BGR channel order, and 0–255 values without normalization. It
implements YOLOX's raw-head stride decode for output `[1,3549,85]`, with strides
8/16/32, then class-agnostic NMS. Thresholds are 0.35 confidence and 0.45 IoU,
with at most 20 boxes returned. Preprocessing follows the official
[preproc implementation](https://github.com/Megvii-BaseDetection/YOLOX/blob/main/yolox/data/data_augment.py)
and [postprocessor](https://github.com/Megvii-BaseDetection/YOLOX/blob/main/yolox/utils/demo_utils.py).
Our standalone bilinear resize avoids an OpenCV dependency; rounding can differ
slightly from OpenCV's integer interpolation tables.

The RGB detector is advisory. It does not receive LiDAR safety decisions, issue
motor commands, or bypass the independent obstacle/staleness path.

## Independent monocular-depth experiment

`midas-small.onnx` is the official MiDaS v2.1 small ONNX model from the
[Intel ISL release](https://github.com/isl-org/MiDaS/releases/tag/v2_1), SHA-256
`2d8c6cb8f415229daf1eb041024208e2608c9f98e17c81cc7c6ecb449c56fd58`.
A copy of each upstream source license is retained alongside these artifacts.
The upstream [MiDaS repository license](https://github.com/isl-org/MiDaS/blob/v2_1/LICENSE)
is MIT. See Ranftl et al., *Towards Robust Monocular Depth Estimation: Mixing
Datasets for Zero-shot Cross-dataset Transfer*, TPAMI2020.

`ai_depth.cpp` runs this model in a separate ONNX Runtime process. It follows
[the official ONNX example](https://github.com/isl-org/MiDaS/blob/v2_1/tf/run_onnx.py):
RGB0–1, stretched to256×256 with cubic interpolation. Mean/std normalization is
part of the exported graph, so the worker does not apply it again. The result is
a64×64 top-left row-major grid of **relative inverse depth**, not meters.
It is sampled at cell centers from the256×256 output and retains the model's
raw scale and shift. Neither this array nor its later LiDAR-calibrated estimates
may be used by the independent measured-depth obstacle guard.

The phone may fit scale/shift to measured LiDAR only when image IDs, epoch,
intrinsics, pose and depth refer to the same frame. Calibration failure must
remain visible and reject metric estimates; it must not fabricate a range for
missing LiDAR. Model extrapolation beyond measured surfaces remains estimated.
