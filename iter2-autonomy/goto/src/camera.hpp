// Frame sources for detect.cpp.
//
//   camapi[:N]          QNX Camera library (camera_api.h, -lcamapi), unit N
//                       (default 1 = CAMERA_UNIT_1). Needs the QNX sensor service
//                       running with the USB webcam in its config (README).
//                       QNX only, built with -DHAVE_CAMAPI.
//   raw:WxH[:yuyv|rgb24][:path]
//                       raw frames from a file/pipe (default stdin). Any process
//                       that can produce frames works, e.g. on the Mac:
//                       ffmpeg -f avfoundation -i 0 -s 640x480 -pix_fmt rgb24 -f rawvideo -
//   ppm:path            one binary PPM (P6), served forever (tests / benchmarks)
//   broker[:PORT]       the mounted iPhone's camera, via Andrew's latest-frame broker
//                       (iter2-autonomy/edge-ai/rgb_broker.py, 127.0.0.1:5566 by default):
//                       "GET\n" -> u32 BE length + "RGB1" + u64 BE frame id + JPEG.
//                       Needs -DHAVE_JPEG (libjpeg-turbo, QNX: apk add libjpeg-turbo-dev).
//                       Waits (never fails) while the phone is away: goto's own
//                       "no camera frames" timeout is what stops a GOTO then.
#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "yolo_post.hpp"

#ifdef HAVE_CAMAPI
#include <camera/camera_api.h>
#endif

#ifdef HAVE_JPEG
#include <arpa/inet.h>
#include <netinet/in.h>
#include <setjmp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <jpeglib.h>
#endif

struct FrameSource {
    virtual ~FrameSource() {}
    // Blocks until a frame NEWER than the last one returned is available.
    // false = source finished / failed (msg in err()).
    virtual bool grab(yolo::Image &rgb) = 0;
    std::string error;
};

// ----------------------------------------------------------------- raw / ppm
struct RawSource : FrameSource {
    FILE *f = nullptr;
    int w, h;
    bool yuyv;
    std::vector<uint8_t> buf;
    RawSource(int W, int H, bool YUYV, const std::string &path) : w(W), h(H), yuyv(YUYV)
    {
        f = path.empty() || path == "-" ? stdin : fopen(path.c_str(), "rb");
        if (!f) error = "can't open " + path;
        buf.resize((size_t)W * H * (yuyv ? 2 : 3));
    }
    ~RawSource() { if (f && f != stdin) fclose(f); }
    bool grab(yolo::Image &rgb) override
    {
        if (!f) return false;
        if (fread(buf.data(), 1, buf.size(), f) != buf.size()) { error = "end of input"; return false; }
        if (yuyv) yolo::yuyv_to_rgb(buf.data(), w, h, w * 2, false, rgb);
        else { rgb.resize(w, h); memcpy(rgb.px.data(), buf.data(), buf.size()); }
        return true;
    }
};

static inline bool read_ppm(const std::string &path, yolo::Image &img, std::string &err)
{
    FILE *f = fopen(path.c_str(), "rb");
    if (!f) { err = "can't open " + path; return false; }
    int w, h, mx;
    char magic[3] = {0};
    bool ok = fscanf(f, "%2s %d %d %d", magic, &w, &h, &mx) == 4 && !strcmp(magic, "P6") && mx == 255 &&
              w > 0 && h > 0 && w < 10000 && h < 10000;
    if (ok) {
        fgetc(f);
        img.resize(w, h);
        ok = fread(img.px.data(), 1, img.px.size(), f) == img.px.size();
    }
    fclose(f);
    if (!ok) err = path + ": not a binary 8-bit PPM (P6)";
    return ok;
}

struct PpmSource : FrameSource {
    yolo::Image img;
    bool ok;
    explicit PpmSource(const std::string &path) { ok = read_ppm(path, img, error); }
    bool grab(yolo::Image &rgb) override
    {
        if (!ok) return false;
        rgb = img;
        return true;
    }
};

// ------------------------------------------------------- iPhone via rgb_broker
#ifdef HAVE_JPEG
struct JpegErr { jpeg_error_mgr base; jmp_buf jump; char msg[JMSG_LENGTH_MAX]; };
static inline void jpeg_err_exit(j_common_ptr c)
{
    JpegErr *e = (JpegErr *)c->err;
    (*c->err->format_message)(c, e->msg);
    longjmp(e->jump, 1);
}

// JPEG bytes -> packed RGB24. false + err on a bad or oversized image.
static inline bool decode_jpeg(const uint8_t *data, size_t n, yolo::Image &out, std::string &err)
{
    // Everything libjpeg touches lives in this heap block, so the longjmp can't skip a C++ destructor.
    std::unique_ptr<jpeg_decompress_struct> c(new jpeg_decompress_struct());
    std::unique_ptr<JpegErr> e(new JpegErr());
    c->err = jpeg_std_error(&e->base);
    e->base.error_exit = jpeg_err_exit;
    if (setjmp(e->jump)) {
        err = std::string("JPEG: ") + e->msg;
        jpeg_destroy_decompress(c.get());
        return false;
    }
    jpeg_create_decompress(c.get());
    jpeg_mem_src(c.get(), (unsigned char *)data, (unsigned long)n);
    if (jpeg_read_header(c.get(), TRUE) != JPEG_HEADER_OK) { err = "JPEG: no header"; jpeg_destroy_decompress(c.get()); return false; }
    c->out_color_space = JCS_RGB;
    jpeg_start_decompress(c.get());
    if (c->output_components != 3 || c->output_width == 0 || c->output_width > 4096 || c->output_height > 4096) {
        err = "JPEG: unexpected size/components";
        jpeg_destroy_decompress(c.get());
        return false;
    }
    out.resize((int)c->output_width, (int)c->output_height);
    while (c->output_scanline < c->output_height) {
        JSAMPROW row = &out.px[(size_t)c->output_scanline * out.w * 3];
        jpeg_read_scanlines(c.get(), &row, 1);
    }
    jpeg_finish_decompress(c.get());
    jpeg_destroy_decompress(c.get());
    return true;
}

struct BrokerSource : FrameSource {
    int port;
    uint64_t last_id = 0;
    bool have_last = false;
    double last_warn = 0;
    std::vector<uint8_t> pkt;
    explicit BrokerSource(int p) : port(p) {}

    static double now_s()
    {
        using namespace std::chrono;
        return duration<double>(steady_clock::now().time_since_epoch()).count();
    }
    void warn(const std::string &m)
    {
        double t = now_s();
        if (t - last_warn >= 5.0) { fprintf(stderr, "detect: broker %d: %s\n", port, m.c_str()); last_warn = t; }
    }
    static bool read_n(int fd, uint8_t *p, size_t n)
    {
        while (n) {
            ssize_t r = recv(fd, p, n, 0);
            if (r <= 0) return false;
            p += r;
            n -= (size_t)r;
        }
        return true;
    }
    // One GET. 1 = new frame in pkt, 0 = nothing new yet, -1 = error (in msg).
    int fetch(std::string &msg)
    {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        if (fd < 0) { msg = "socket failed"; return -1; }
        timeval tv{1, 0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons((unsigned short)port);
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        int rc = -1;
        uint8_t hdr[4];
        if (connect(fd, (sockaddr *)&a, sizeof a) != 0) msg = "can't connect (is rgb_broker.py running?)";
        else if (send(fd, "GET\n", 4, 0) != 4 || !read_n(fd, hdr, 4)) msg = "no reply";
        else {
            uint32_t size = ((uint32_t)hdr[0] << 24) | ((uint32_t)hdr[1] << 16) | ((uint32_t)hdr[2] << 8) | hdr[3];
            if (size == 0) { msg = "no fresh phone frame (MotionUSB started? phone on the Pi's USB?)"; rc = 0; }
            else if (size < 16 || size > 2000012) msg = "bad frame length " + std::to_string(size);
            else {
                pkt.resize(size);
                if (!read_n(fd, pkt.data(), size)) msg = "short frame";
                else if (memcmp(pkt.data(), "RGB1", 4) != 0) msg = "not an RGB1 packet";
                else {
                    uint64_t id = 0;
                    for (int i = 0; i < 8; i++) id = (id << 8) | pkt[4 + i];
                    if (have_last && id == last_id) rc = 0;
                    else { last_id = id; have_last = true; rc = 1; }
                }
            }
        }
        close(fd);
        return rc;
    }
    bool grab(yolo::Image &rgb) override
    {
        for (;;) {
            std::string msg;
            int r = fetch(msg);
            if (r == 1) {
                std::string jerr;
                if (decode_jpeg(pkt.data() + 12, pkt.size() - 12, rgb, jerr)) return true;
                warn(jerr);
            } else if (r < 0 || !msg.empty()) {
                warn(msg);
            }
            // the phone sends ~5 fps; a new frame is at most ~0.2 s away
            std::this_thread::sleep_for(std::chrono::milliseconds(r < 0 || !msg.empty() ? 500 : 40));
        }
    }
};
#endif

// ------------------------------------------------------------- QNX camapi
#ifdef HAVE_CAMAPI
// UNTESTED: written against the QNX Camera library docs (camera_api.h), not
// yet compiled with the SDP or run on the Pi. The viewfinder callback runs
// on a camera-library thread; it converts the newest frame to RGB and the
// detector picks it up (older frames are simply overwritten).
struct CamapiSource : FrameSource {
    camera_handle_t handle = CAMERA_HANDLE_INVALID;
    std::mutex mu;
    std::condition_variable cv;
    yolo::Image latest;
    uint64_t seq = 0, taken = 0;
    std::string fmt_seen;

    static void vf_cb(camera_handle_t, camera_buffer_t *buf, void *arg)
    {
        static_cast<CamapiSource *>(arg)->on_frame(buf);
    }
    static void status_cb(camera_handle_t, camera_devstatus_t st, uint16_t extra, void *)
    {
        if (st == CAMERA_STATUS_DISCONNECTED) fprintf(stderr, "[camapi] camera disconnected\n");
        else if (st == CAMERA_STATUS_FRAME_DROPPED) {}
        else fprintf(stderr, "[camapi] status %d extra %u\n", (int)st, (unsigned)extra);
    }

    void on_frame(camera_buffer_t *b)
    {
        yolo::Image img;
        const uint8_t *p = (const uint8_t *)b->framebuf;
        switch (b->frametype) {
        case CAMERA_FRAMETYPE_YCBYCR:
            yolo::yuyv_to_rgb(p, b->framedesc.ycbycr.width, b->framedesc.ycbycr.height, b->framedesc.ycbycr.stride, false, img);
            break;
        case CAMERA_FRAMETYPE_CBYCRY:
            yolo::yuyv_to_rgb(p, b->framedesc.cbycry.width, b->framedesc.cbycry.height, b->framedesc.cbycry.stride, true, img);
            break;
        case CAMERA_FRAMETYPE_NV12:
            yolo::nv12_to_rgb(p, b->framedesc.nv12.stride, p + b->framedesc.nv12.uv_offset, b->framedesc.nv12.uv_stride,
                              b->framedesc.nv12.width, b->framedesc.nv12.height, img);
            break;
        case CAMERA_FRAMETYPE_RGB8888:
            yolo::rgbx_to_rgb(p, b->framedesc.rgb8888.width, b->framedesc.rgb8888.height, b->framedesc.rgb8888.stride, false, img);
            break;
        case CAMERA_FRAMETYPE_BGR8888:
            yolo::rgbx_to_rgb(p, b->framedesc.bgr8888.width, b->framedesc.bgr8888.height, b->framedesc.bgr8888.stride, true, img);
            break;
        default: {
            std::lock_guard<std::mutex> l(mu);
            if (fmt_seen.empty()) {
                fmt_seen = std::to_string((int)b->frametype);
                fprintf(stderr, "[camapi] unsupported frame type %d (add it in camera.hpp)\n", (int)b->frametype);
            }
            return;
        }
        }
        std::lock_guard<std::mutex> l(mu);
        latest.w = img.w;
        latest.h = img.h;
        latest.px.swap(img.px);
        seq++;
        cv.notify_one();
    }

    explicit CamapiSource(int unit)
    {
        camera_error_t e = camera_open((camera_unit_t)unit, CAMERA_MODE_RW | CAMERA_MODE_ROLL, &handle);
        if (e != CAMERA_EOK) {
            /* a USB webcam may refuse RW; viewing only needs RO */
            e = camera_open((camera_unit_t)unit, CAMERA_MODE_RO, &handle);
        }
        if (e != CAMERA_EOK) { error = "camera_open(unit " + std::to_string(unit) + ") failed: " + std::to_string((int)e) +
                                       " (sensor service running? webcam in its config?)"; return; }
        camera_set_vf_mode(handle, CAMERA_VFMODE_DEFAULT);
        e = camera_start_viewfinder(handle, vf_cb, status_cb, this);
        if (e != CAMERA_EOK) { error = "camera_start_viewfinder failed: " + std::to_string((int)e); return; }
    }
    ~CamapiSource()
    {
        if (handle != CAMERA_HANDLE_INVALID) {
            camera_stop_viewfinder(handle);
            camera_close(handle);
        }
    }
    bool grab(yolo::Image &rgb) override
    {
        if (!error.empty()) return false;
        std::unique_lock<std::mutex> l(mu);
        if (!cv.wait_for(l, std::chrono::seconds(3), [&] { return seq > taken; })) {
            error = "no frames from the camera for 3 s";
            return false;
        }
        taken = seq;
        rgb.w = latest.w;
        rgb.h = latest.h;
        rgb.px = latest.px;
        return true;
    }
};
#endif

// "camapi:1", "raw:640x480:yuyv:-", "ppm:test.ppm"
static inline std::unique_ptr<FrameSource> open_source(const std::string &spec, std::string &err)
{
    auto parts = [&](void) {
        std::vector<std::string> v;
        size_t s = 0, p;
        while ((p = spec.find(':', s)) != std::string::npos) { v.push_back(spec.substr(s, p - s)); s = p + 1; }
        v.push_back(spec.substr(s));
        return v;
    }();
    std::unique_ptr<FrameSource> src;
    if (parts[0] == "camapi") {
#ifdef HAVE_CAMAPI
        src.reset(new CamapiSource(parts.size() > 1 ? atoi(parts[1].c_str()) : 1));
#else
        err = "camapi: built without -DHAVE_CAMAPI (QNX only)";
        return nullptr;
#endif
    } else if (parts[0] == "raw" && parts.size() >= 2) {
        int w = 0, h = 0;
        if (sscanf(parts[1].c_str(), "%dx%d", &w, &h) != 2 || w <= 0 || h <= 0) { err = "raw:WxH"; return nullptr; }
        bool yuyv = parts.size() > 2 && parts[2] == "yuyv";
        src.reset(new RawSource(w, h, yuyv, parts.size() > 3 ? parts[3] : "-"));
    } else if (parts[0] == "ppm" && parts.size() >= 2) {
        src.reset(new PpmSource(spec.substr(4)));
    } else if (parts[0] == "broker") {
#ifdef HAVE_JPEG
        int port = parts.size() > 1 ? atoi(parts[1].c_str()) : 5566;
        if (port <= 0 || port > 65535) { err = "broker:PORT"; return nullptr; }
        src.reset(new BrokerSource(port));
#else
        err = "broker: built without -DHAVE_JPEG (libjpeg)";
        return nullptr;
#endif
    } else {
        err = "unknown camera '" + spec + "' (camapi[:N] | broker[:PORT] | raw:WxH[:yuyv|rgb24][:path] | ppm:path)";
        return nullptr;
    }
    if (!src->error.empty()) { err = src->error; return nullptr; }
    return src;
}
