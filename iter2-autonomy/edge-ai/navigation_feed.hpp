#pragma once
// Best-effort loopback output. No acknowledgements and no blocking: a stalled
// planner must never delay the LiDAR receiver or the safety control loop.
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

class NavigationFeed {
    int fd_ = -1;
    sockaddr_in target_{};
public:
    explicit NavigationFeed(unsigned port) {
        if (!port || port > 65535) return;
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) return;
        const int flags = fcntl(fd_, F_GETFL, 0);
        if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) { close(fd_); fd_ = -1; return; }
        int size = 262144;
        setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &size, sizeof size);
        target_.sin_family = AF_INET;
        target_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        target_.sin_port = htons(static_cast<uint16_t>(port));
    }
    ~NavigationFeed() { if (fd_ >= 0) close(fd_); }
    NavigationFeed(const NavigationFeed&) = delete;
    NavigationFeed& operator=(const NavigationFeed&) = delete;
    void send(const void* data, size_t size) const {
        if (fd_ >= 0 && size <= 60000)
            (void)sendto(fd_, data, size, 0, reinterpret_cast<const sockaddr*>(&target_), sizeof target_);
    }
};

// YOLOX boxes are already in source-image pixels here; do not transmit the
// normalized x/y/width/height used by the phone overlay as goto's xyxy.
template<class Detections>
std::string navigation_detections(int width, int height, const Detections& detections, const char* const* labels) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::fixed << std::setprecision(4) << "{\"w\":" << width << ",\"h\":" << height << ",\"dets\":[";
    bool first = true;
    for (const auto& d : detections) {
        if (!std::isfinite(d.confidence) || d.confidence < 0 || d.confidence > 1 ||
            !std::isfinite(d.left) || !std::isfinite(d.top) || !std::isfinite(d.right) || !std::isfinite(d.bottom) ||
            d.left < 0 || d.top < 0 || d.right > width || d.bottom > height || d.right <= d.left || d.bottom <= d.top) continue;
        if (!first) out << ',';
        first = false;
        // Labels are the fixed COCO table in ai_detector.cpp, not network input.
        out << "{\"label\":\"" << labels[d.label] << "\",\"conf\":" << d.confidence
            << ",\"box\":[" << d.left << ',' << d.top << ',' << d.right << ',' << d.bottom << "]}";
    }
    out << "]}\n";
    return out.str();
}
