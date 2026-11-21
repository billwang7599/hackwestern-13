#pragma once
#include <cstdint>
#include <memory>
namespace mediapipe {
enum class ImageFormat { SRGB };
struct ImageFrame { static const uint32_t kDefaultAlignmentBoundary = 16; ImageFrame(ImageFormat, int, int, uint32_t) {} };
using ImageFrameSharedPtr = std::shared_ptr<ImageFrame>;
}
