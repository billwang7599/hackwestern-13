// tars_gesture: MediaPipe hand gestures / hand following -> tars_motor, on QNX.
//
// Camera (QNX libcamapi, via the qnx-ports MediaPipe example code qnx_defs.cc)
//   -> flip + downscale -> MediaPipe Tasks HandLandmarker (VIDEO mode, 1 hand)
//   -> tars::classify / tars::Controller (vision/common/gesture.hpp, the same
//      rules as the Python apps, parity-tested) -> TCP 127.0.0.1:5555.
// A 20 ms ticker thread does resends, move ends and the camera-stall STOP.
//
// Build: vision/qnx/cpp/build_on_pi.sh (on the Pi, in the qnx-ports/mediapipe
// tree, per the QNX "MediaPipe Camera Sample" codelab).
//
//   tars_gesture --model=hand_landmarker.task --conf=gesture.conf [--camera_unit=N]
//                [--host=127.0.0.1] [--port=5555] [--dry_run] [--frames=N]
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "mediapipe/examples/qnx/qnx_defs.h"
#include "mediapipe/examples/qnx/tars_gesture/gesture.hpp"
#include "mediapipe/framework/formats/image.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/tasks/cc/vision/hand_landmarker/hand_landmarker.h"

ABSL_FLAG(long, camera_unit, 0, "QNX camera unit (0 = first available).");
ABSL_FLAG(std::string, model, "hand_landmarker.task", "MediaPipe hand landmarker .task file.");
ABSL_FLAG(std::string, conf, "gesture.conf", "vision/gesture.conf (shared with the Python apps).");
ABSL_FLAG(std::string, host, "127.0.0.1", "tars_motor host.");
ABSL_FLAG(int, port, 5555, "tars_motor port.");
ABSL_FLAG(bool, dry_run, false, "Print commands instead of sending them.");
ABSL_FLAG(int, frames, 0, "Stop after N frames (0 = run forever).");

namespace {

using Clock = std::chrono::steady_clock;
double now_ms() {
    return std::chrono::duration<double, std::milli>(Clock::now().time_since_epoch()).count();
}

std::atomic<bool> g_running{true};
void on_signal(int) { g_running = false; }

// ---- link to tars_motor (same behaviour as vision/common/link.py) -------------
class MotorLink {
public:
    MotorLink(std::string host, int port, bool dry) : host_(std::move(host)), port_(port), dry_(dry) {}
    ~MotorLink() { drop("closing"); }
    bool dry() const { return dry_; }
    double rtt_ms = 0;

    std::string send(const std::string &line) {
        if (dry_) {
            if (line[0] != 'D') std::printf("[dry] %s\n", line.c_str());
            return "OK";
        }
        std::lock_guard<std::mutex> lk(m_);
        if (!connect_()) return "";
        double t0 = now_ms();
        std::string out = line + "\n";
        if (::send(fd_, out.data(), out.size(), 0) != (ssize_t)out.size()) { drop("send failed"); return ""; }
        std::string reply;
        char ch;
        while (true) {
            ssize_t n = ::recv(fd_, &ch, 1, 0);
            if (n <= 0) { drop("server closed"); return ""; }
            if (ch == '\n') break;
            reply += ch;
        }
        rtt_ms = now_ms() - t0;
        if (reply.rfind("ERR", 0) == 0 && reply != last_err_) {
            std::fprintf(stderr, "[link] '%s' -> %s\n", line.c_str(), reply.c_str());
            last_err_ = reply;
        }
        return reply;
    }

private:
    bool connect_() {
        if (fd_ >= 0) return true;
        double t = now_ms();
        if (t < next_try_) return false;
        next_try_ = t + 1000;
        addrinfo hints{}, *res = nullptr;
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        if (getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res) != 0 || !res) {
            std::fprintf(stderr, "[link] cannot resolve %s\n", host_.c_str());
            return false;
        }
        int fd = socket(res->ai_family, res->ai_socktype, 0);
        if (fd < 0 || ::connect(fd, res->ai_addr, res->ai_addrlen) != 0) {
            std::fprintf(stderr, "[link] %s:%d unreachable; retrying\n", host_.c_str(), port_);
            if (fd >= 0) close(fd);
            freeaddrinfo(res);
            return false;
        }
        freeaddrinfo(res);
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        timeval tv{1, 0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        fd_ = fd;
        std::fprintf(stderr, "[link] connected to %s:%d\n", host_.c_str(), port_);
        return true;
    }
    void drop(const char *why) {
        if (fd_ >= 0) {
            std::fprintf(stderr, "[link] lost connection: %s\n", why);
            close(fd_);
        }
        fd_ = -1;
    }
    std::string host_;
    int port_;
    bool dry_;
    int fd_ = -1;
    double next_try_ = 0;
    std::string last_err_;
    std::mutex m_;
};

// ---- runtime (same as vision/common/app.py) ------------------------------------
struct Runtime {
    tars::Controller ctl;
    MotorLink &link;
    std::mutex ctl_m;
    double fps = 0, infer_ms = 0, cam_to_cmd_ms = 0, stop_ms = -1;
    double last_frame = -1, next_log = 0;
    std::string last_label = tars::NO_HAND;

    Runtime(tars::Config c, MotorLink &l) : ctl(std::move(c)), link(l) { next_log = now_ms() + 1000; }

    void send(const std::vector<tars::Cmd> &cmds, double t_cap) {
        for (auto &c : cmds) {
            link.send(c.text);
            if (c.text[0] != 'D' || (c.reason != "resend" && c.reason != "follow"))
                std::printf("[cmd] %-14s %-14s mode=%s\n", c.text.c_str(), c.reason.c_str(), ctl.mode.c_str());
            if (t_cap >= 0) {
                double lat = now_ms() - t_cap;
                cam_to_cmd_ms = lat;
                if (c.reason == "palm") {
                    stop_ms = lat;
                    std::printf("[lat] PALM -> STOP: camera frame -> S acked in %.0f ms\n", lat);
                }
            }
        }
        std::fflush(stdout);
    }

    std::string status_line() {
        char b[320], stop[16] = "-", extra[64] = "";
        if (stop_ms >= 0) std::snprintf(stop, sizeof stop, "%.0fms", stop_ms);
        if (ctl.mode == tars::FOLLOW_MODE)
            std::snprintf(extra, sizeof extra, " wheels=%+.2f,%+.2f", ctl.f_cur_l, ctl.f_cur_r);
        else if (ctl.has_move)
            std::snprintf(extra, sizeof extra, " move=%s", ctl.move_intent.c_str());
        std::snprintf(b, sizeof b,
                      "mode=%s fps=%.1f infer=%.0fms cam->cmd=%.0fms palm->stop=%s see=%s%s link_rtt=%.1fms",
                      ctl.mode.c_str(), fps, infer_ms, cam_to_cmd_ms, stop, last_label.c_str(), extra,
                      link.rtt_ms);
        return b;
    }

    void maybe_log() {
        double t = now_ms();
        if (t < next_log) return;
        next_log = t + 1000;
        std::string line;
        {
            std::lock_guard<std::mutex> lk(ctl_m);
            line = status_line();
        }
        std::printf("[status] %s\n", line.c_str());
        std::fflush(stdout);
        if (!link.dry()) {
            char v[128];
            std::snprintf(v, sizeof v, "VIS %s %.1f %.0f %.0f %s", ctl.mode.c_str(), fps, cam_to_cmd_ms,
                          stop_ms, last_label.c_str());
            link.send(v);
        }
    }
};

}  // namespace

int main(int argc, char **argv) {
    absl::ParseCommandLine(argc, argv);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    tars::Config cfg = tars::load_config(absl::GetFlag(FLAGS_conf));
    MotorLink link(absl::GetFlag(FLAGS_host), absl::GetFlag(FLAGS_port), absl::GetFlag(FLAGS_dry_run));
    Runtime rt(cfg, link);

    namespace hl = mediapipe::tasks::vision::hand_landmarker;
    auto opts = std::make_unique<hl::HandLandmarkerOptions>();
    opts->base_options.model_asset_path = absl::GetFlag(FLAGS_model);
    opts->running_mode = mediapipe::tasks::vision::core::RunningMode::VIDEO;
    opts->num_hands = 1;
    opts->min_hand_detection_confidence = (float)cfg["min_detection_confidence"];
    opts->min_hand_presence_confidence = (float)cfg["min_detection_confidence"];
    opts->min_tracking_confidence = (float)cfg["min_tracking_confidence"];
    auto lm_or = hl::HandLandmarker::Create(std::move(opts));
    if (!lm_or.ok()) {
        std::fprintf(stderr, "HandLandmarker: %s\n", std::string(lm_or.status().message()).c_str());
        return 1;
    }
    std::unique_ptr<hl::HandLandmarker> landmarker = std::move(lm_or.value());

    // Camera: qnx_defs picks the first available unit when given CAMERA_UNIT_INVALID.
    auto ci = std::make_unique<mp_camera_info_t>();
    long unit = absl::GetFlag(FLAGS_camera_unit);
    absl::Status cs = InitCameraSink(*ci, unit > 0 ? (camera_unit_t)unit : CAMERA_UNIT_INVALID, false);
    if (!cs.ok()) {
        std::fprintf(stderr, "camera: %s\n", std::string(cs.message()).c_str());
        return 1;
    }
    std::printf("[start] camera unit %d; mode=%s; %s\n", (int)ci->unit, rt.ctl.mode.c_str(),
                link.dry() ? "DRY RUN" : (absl::GetFlag(FLAGS_host) + ":" + std::to_string(absl::GetFlag(FLAGS_port))).c_str());
    std::fflush(stdout);

    // Ticker: resends, move ends, camera-stall STOP (works even if the main
    // thread is stuck waiting for a frame), status line.
    std::thread ticker([&] {
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            std::vector<tars::Cmd> cmds;
            {
                std::lock_guard<std::mutex> lk(rt.ctl_m);
                cmds = rt.ctl.tick(now_ms());
            }
            if (!cmds.empty()) rt.send(cmds, -1);
            rt.maybe_log();
        }
    });

    const bool flip = cfg["flip"] >= 0.5;
    const int infer_w = (int)cfg["infer_width"];
    int64_t last_ts = -1;
    int n = 0, max_frames = absl::GetFlag(FLAGS_frames);
    while (g_running) {
        cv::Mat frame = CameraConsumeData(*ci);  // RGB, blocks until the next frame
        double t_cap = now_ms();
        if (frame.empty()) continue;
        if (flip) cv::flip(frame, frame, 1);
        if (infer_w > 0 && frame.cols > infer_w) {
            int h = (int)std::lround(frame.rows * (double)infer_w / frame.cols);
            cv::resize(frame, frame, cv::Size(infer_w, h), 0, 0, cv::INTER_AREA);
        }
        auto img = std::make_shared<mediapipe::ImageFrame>(mediapipe::ImageFormat::SRGB, frame.cols, frame.rows,
                                                           mediapipe::ImageFrame::kDefaultAlignmentBoundary);
        frame.copyTo(mediapipe::formats::MatView(img.get()));
        double aspect = (double)frame.cols / frame.rows;

        int64_t ts = (int64_t)t_cap;
        if (ts <= last_ts) ts = last_ts + 1;
        last_ts = ts;
        double t0 = now_ms();
        auto res = landmarker->DetectForVideo(mediapipe::Image(img), ts);
        double infer = now_ms() - t0;

        tars::Landmarks lms;
        const tars::Landmarks *lp = nullptr;
        if (res.ok() && !res->hand_landmarks.empty()) {
            for (auto &p : res->hand_landmarks[0].landmarks) lms.push_back({p.x, p.y, p.z});
            lp = &lms;
        } else if (!res.ok()) {
            std::fprintf(stderr, "detect: %s\n", std::string(res.status().message()).c_str());
        }
        std::vector<tars::Cmd> cmds;
        std::string label;
        {
            std::lock_guard<std::mutex> lk(rt.ctl_m);
            label = tars::classify(lp, aspect, rt.ctl.cfg());
            cmds = rt.ctl.on_frame(now_ms(), label, tars::hand_features(lp, aspect));
        }
        rt.send(cmds, t_cap);

        if (rt.last_frame >= 0 && t_cap > rt.last_frame) {
            double f = 1000.0 / (t_cap - rt.last_frame);
            rt.fps = rt.fps == 0 ? f : 0.9 * rt.fps + 0.1 * f;
        }
        rt.last_frame = t_cap;
        rt.infer_ms = rt.infer_ms == 0 ? infer : 0.9 * rt.infer_ms + 0.1 * infer;
        if (label != rt.last_label) {
            std::printf("[see] %s\n", label.c_str());
            std::fflush(stdout);
            rt.last_label = label;
        }
        if (max_frames && ++n >= max_frames) break;
    }

    g_running = false;
    ticker.join();
    link.send("S");
    std::printf("[end] %s\n", rt.status_line().c_str());
    landmarker->Close().IgnoreError();
    TeardownCameraSink(*ci);
    return 0;
}
