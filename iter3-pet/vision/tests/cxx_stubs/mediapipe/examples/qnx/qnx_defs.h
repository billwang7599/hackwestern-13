#pragma once
#include <condition_variable>
#include <mutex>
#include <string>
#include <string_view>
namespace cv {
struct Size { Size(int, int) {} };
enum { INTER_AREA = 3 };
struct Mat { int rows = 0, cols = 0; bool empty() const { return true; } void copyTo(Mat) const {} };
inline void flip(const Mat&, Mat&, int) {}
inline void resize(const Mat&, Mat&, Size, double, double, int) {}
}
namespace absl {
struct Status { bool ok() const { return true; } std::string_view message() const { return ""; } void IgnoreError() const {} };
template <class T> struct StatusOr {
  T val; bool ok() const { return true; } Status status() const { return {}; }
  T& value() { return val; } T* operator->() { return &val; } };
}
typedef int camera_unit_t; typedef int camera_handle_t;
enum { CAMERA_UNIT_INVALID = -1 };
typedef struct mp_camera_info { bool initialized; camera_unit_t unit; camera_handle_t handle; double framerate;
  std::mutex data_m; std::condition_variable data_cv; bool data_ready; cv::Mat data; } mp_camera_info_t;
cv::Mat CameraConsumeData(mp_camera_info_t &ci);
absl::Status InitCameraSink(mp_camera_info_t &ci, const camera_unit_t unit, const bool save_video);
void TeardownCameraSink(mp_camera_info_t &ci);
