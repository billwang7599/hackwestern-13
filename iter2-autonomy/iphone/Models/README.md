# Object detector

`YOLOv3TinyFP16.mlmodel` is the 17.7 MB FP16 Core ML model distributed by Apple:

- Model gallery: https://developer.apple.com/machine-learning/models/
- Download: https://ml-assets.apple.com/coreml/models/Image/ObjectDetection/YOLOv3Tiny/YOLOv3TinyFP16.mlmodel
- Upstream model/framework: https://github.com/pjreddie/darknet
- SHA-256: `73406178d0f5793d0d5d1e38274acd146a744c2245c9b63a11998a5015925dda`

The embedded metadata credits Joseph Redmon and Ali Farhadi and directs users
to the upstream Darknet repository for licensing. Those notices remain in the
unmodified model. This detector has 80 fixed object classes; it does not perform
open-vocabulary identification or identify a particular personal belonging.

Run `python3 iphone/Models/fetch-model.py` to verify/recreate this pinned file.
Xcode compiles it into the application. No model download occurs on the robot.
We selected this ready-made artifact for the first hardware integration; its
accuracy and runtime need measurement before selecting a newer detector.
