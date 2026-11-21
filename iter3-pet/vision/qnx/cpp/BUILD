# Copied into <qnx-ports/mediapipe>/mediapipe/examples/qnx/tars_gesture/ by
# build_on_pi.sh, next to QNX's own examples (it reuses their qnx_defs camera
# sink). MediaPipe and its models are Apache 2.0.

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "tars_gesture_cpu",  # *_cpu: the name build_qnx_examples.sh builds
    srcs = [
        "gesture.hpp",
        "learn.hpp",
        "pet.hpp",
        "tars_gesture.cc",
    ],
    # same float results as the Python apps (no fused multiply-add)
    copts = ["-ffp-contract=off"],
    linkopts = [
        "-lcamapi",
        "-lsocket",
    ],
    deps = [
        "//mediapipe/examples/qnx:qnx_defs",
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "//mediapipe/framework/port:opencv_core",
        "//mediapipe/framework/port:opencv_imgproc",
        "//mediapipe/tasks/cc/vision/hand_landmarker",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
    ],
)
