#pragma once
#include <memory>
#include <string>
#include <vector>
#include "mediapipe/framework/formats/image.h"
namespace mediapipe { namespace tasks {
namespace core { struct BaseOptions { std::string model_asset_path; }; }
namespace components { namespace containers {
struct NormalizedLandmark { float x, y, z; };
struct NormalizedLandmarks { std::vector<NormalizedLandmark> landmarks; }; } }
namespace vision {
namespace core { enum class RunningMode { IMAGE, VIDEO, LIVE_STREAM }; }
namespace hand_landmarker {
struct HandLandmarkerOptions { tasks::core::BaseOptions base_options; core::RunningMode running_mode = core::RunningMode::IMAGE;
  int num_hands = 1; float min_hand_detection_confidence = 0.5, min_hand_presence_confidence = 0.5, min_tracking_confidence = 0.5; };
struct HandLandmarkerResult { std::vector<components::containers::NormalizedLandmarks> hand_landmarks; };
class HandLandmarker { public:
  static absl::StatusOr<std::unique_ptr<HandLandmarker>> Create(std::unique_ptr<HandLandmarkerOptions>);
  absl::StatusOr<HandLandmarkerResult> DetectForVideo(mediapipe::Image, int64_t);
  absl::Status Close(); };
} } } }
