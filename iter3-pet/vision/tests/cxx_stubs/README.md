Minimal stand-in headers so `vision/tests/check_cpp_syntax.sh` can compile
vision/qnx/cpp/tars_gesture.cc on the Mac with -fsyntax-only. They mirror only
the signatures the app uses (checked against qnx-ports/mediapipe qnx-v0.10.26).
The real build happens on the Pi (vision/qnx/cpp/build_on_pi.sh).
