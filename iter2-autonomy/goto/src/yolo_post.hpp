// YOLO pre/post-processing for detect.cpp, header-only and dependency-free so
// it builds with q++ (QNX) and clang++ (Mac tests) and is unit-tested without
// ONNX Runtime. Handles YOLOv8/YOLO11/YOLO-World ONNX exports:
//   input  [1,3,S,S] float32 RGB 0..1, letterboxed (pad 114)
//   output [1,4+nc,N] (ultralytics default) or [1,N,4+nc]; rows cx,cy,w,h,
//          then nc class scores (already sigmoid'ed, no objectness).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace yolo {

struct Image {                 // packed RGB24
    int w = 0, h = 0;
    std::vector<uint8_t> px;   // w*h*3
    void resize(int W, int H) { w = W; h = H; px.assign((size_t)W * H * 3, 0); }
};

static inline uint8_t clamp8(int v) { return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v); }

// BT.601 limited range, integer
static inline void yuv2rgb(int y, int u, int v, uint8_t *o)
{
    int c = y - 16, d = u - 128, e = v - 128;
    o[0] = clamp8((298 * c + 409 * e + 128) >> 8);
    o[1] = clamp8((298 * c - 100 * d - 208 * e + 128) >> 8);
    o[2] = clamp8((298 * c + 516 * d + 128) >> 8);
}

// YUYV (YCbYCr, the usual UVC format). uyvy=true for CbYCrY.
static inline void yuyv_to_rgb(const uint8_t *src, int w, int h, int stride, bool uyvy, Image &out)
{
    out.resize(w, h);
    for (int y = 0; y < h; y++) {
        const uint8_t *s = src + (size_t)y * stride;
        uint8_t *d = &out.px[(size_t)y * w * 3];
        for (int x = 0; x + 1 < w; x += 2, s += 4, d += 6) {
            int y0 = uyvy ? s[1] : s[0], u = uyvy ? s[0] : s[1], y1 = uyvy ? s[3] : s[2], v = uyvy ? s[2] : s[3];
            yuv2rgb(y0, u, v, d);
            yuv2rgb(y1, u, v, d + 3);
        }
    }
}

static inline void nv12_to_rgb(const uint8_t *yp, int ystride, const uint8_t *uvp, int uvstride, int w, int h, Image &out)
{
    out.resize(w, h);
    for (int y = 0; y < h; y++) {
        const uint8_t *yr = yp + (size_t)y * ystride, *uvr = uvp + (size_t)(y / 2) * uvstride;
        uint8_t *d = &out.px[(size_t)y * w * 3];
        for (int x = 0; x < w; x++, d += 3) yuv2rgb(yr[x], uvr[x & ~1], uvr[(x & ~1) + 1], d);
    }
}

// 4 bytes per pixel; bgr=true for B,G,R,X order
static inline void rgbx_to_rgb(const uint8_t *src, int w, int h, int stride, bool bgr, Image &out)
{
    out.resize(w, h);
    for (int y = 0; y < h; y++) {
        const uint8_t *s = src + (size_t)y * stride;
        uint8_t *d = &out.px[(size_t)y * w * 3];
        for (int x = 0; x < w; x++, s += 4, d += 3) {
            d[0] = bgr ? s[2] : s[0];
            d[1] = s[1];
            d[2] = bgr ? s[0] : s[2];
        }
    }
}

// Rotate a frame clockwise by 0/90/180/270 degrees (anything else = copy). The mounted
// iPhone sends sensor-orientation (landscape) frames; detect turns them upright first so
// YOLO sees the room the right way up and box x is the robot's left/right.
static inline void rotate_cw(const Image &in, int deg, Image &out)
{
    deg = ((deg % 360) + 360) % 360;
    if (deg != 90 && deg != 180 && deg != 270) { out = in; return; }
    const int W = in.w, H = in.h;
    if (deg == 180) out.resize(W, H); else out.resize(H, W);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int nx, ny;
            if (deg == 90) { nx = H - 1 - y; ny = x; }
            else if (deg == 270) { nx = y; ny = W - 1 - x; }
            else { nx = W - 1 - x; ny = H - 1 - y; }
            const uint8_t *s = &in.px[((size_t)y * W + x) * 3];
            uint8_t *d = &out.px[((size_t)ny * out.w + nx) * 3];
            d[0] = s[0]; d[1] = s[1]; d[2] = s[2];
        }
}

struct Letterbox { float scale = 1, padx = 0, pady = 0; int size = 320; };

// Bilinear letterbox into a planar CHW float tensor (RGB, 0..1).
static inline Letterbox letterbox(const Image &img, int S, std::vector<float> &chw)
{
    Letterbox lb;
    lb.size = S;
    lb.scale = std::min((float)S / img.w, (float)S / img.h);
    int nw = (int)std::lround(img.w * lb.scale), nh = (int)std::lround(img.h * lb.scale);
    lb.padx = (S - nw) / 2.0f;
    lb.pady = (S - nh) / 2.0f;
    int ox = (int)std::floor(lb.padx), oy = (int)std::floor(lb.pady);
    chw.assign((size_t)3 * S * S, 114.0f / 255.0f);
    float *R = chw.data(), *G = R + (size_t)S * S, *B = G + (size_t)S * S;
    for (int y = 0; y < nh; y++) {
        float sy = (y + 0.5f) / lb.scale - 0.5f;
        int y0 = std::max(0, std::min(img.h - 1, (int)std::floor(sy)));
        int y1 = std::min(img.h - 1, y0 + 1);
        float fy = std::max(0.0f, std::min(1.0f, sy - y0));
        for (int x = 0; x < nw; x++) {
            float sx = (x + 0.5f) / lb.scale - 0.5f;
            int x0 = std::max(0, std::min(img.w - 1, (int)std::floor(sx)));
            int x1 = std::min(img.w - 1, x0 + 1);
            float fx = std::max(0.0f, std::min(1.0f, sx - x0));
            const uint8_t *a = &img.px[((size_t)y0 * img.w + x0) * 3], *b = &img.px[((size_t)y0 * img.w + x1) * 3];
            const uint8_t *c = &img.px[((size_t)y1 * img.w + x0) * 3], *d = &img.px[((size_t)y1 * img.w + x1) * 3];
            size_t o = (size_t)(y + oy) * S + (x + ox);
            float v[3];
            for (int k = 0; k < 3; k++)
                v[k] = ((a[k] * (1 - fx) + b[k] * fx) * (1 - fy) + (c[k] * (1 - fx) + d[k] * fx) * fy) / 255.0f;
            R[o] = v[0]; G[o] = v[1]; B[o] = v[2];
        }
    }
    return lb;
}

struct Det { int cls; float conf; float x1, y1, x2, y2; };

static inline float iou(const Det &a, const Det &b)
{
    float ix = std::max(0.0f, std::min(a.x2, b.x2) - std::max(a.x1, b.x1));
    float iy = std::max(0.0f, std::min(a.y2, b.y2) - std::max(a.y1, b.y1));
    float in = ix * iy, u = (a.x2 - a.x1) * (a.y2 - a.y1) + (b.x2 - b.x1) * (b.y2 - b.y1) - in;
    return u > 0 ? in / u : 0;
}

// Decode the raw output. dims = {1, A, B}: if A < B it's [1,4+nc,N] else [1,N,4+nc].
// Boxes come back in ORIGINAL image pixels (letterbox undone, clipped).
static inline std::vector<Det> decode(const float *out, int64_t A, int64_t B, const Letterbox &lb,
                                      int img_w, int img_h, float conf_th, float iou_th = 0.45f,
                                      size_t max_det = 50)
{
    bool ch_first = A < B;
    int64_t C = ch_first ? A : B, N = ch_first ? B : A, nc = C - 4;
    std::vector<Det> cand;
    if (nc <= 0) return cand;
    auto at = [&](int64_t c, int64_t i) { return ch_first ? out[c * N + i] : out[i * C + c]; };
    for (int64_t i = 0; i < N; i++) {
        int best = -1;
        float bs = conf_th;
        for (int64_t c = 0; c < nc; c++) {
            float s = at(4 + c, i);
            if (s > bs) { bs = s; best = (int)c; }
        }
        if (best < 0) continue;
        float cx = at(0, i), cy = at(1, i), w = at(2, i), h = at(3, i);
        Det d;
        d.cls = best;
        d.conf = bs;
        d.x1 = std::max(0.0f, std::min((float)img_w, (cx - w / 2 - lb.padx) / lb.scale));
        d.y1 = std::max(0.0f, std::min((float)img_h, (cy - h / 2 - lb.pady) / lb.scale));
        d.x2 = std::max(0.0f, std::min((float)img_w, (cx + w / 2 - lb.padx) / lb.scale));
        d.y2 = std::max(0.0f, std::min((float)img_h, (cy + h / 2 - lb.pady) / lb.scale));
        if (d.x2 - d.x1 < 1 || d.y2 - d.y1 < 1) continue;
        cand.push_back(d);
    }
    std::sort(cand.begin(), cand.end(), [](const Det &a, const Det &b) { return a.conf > b.conf; });
    std::vector<Det> keep;
    std::vector<char> dead(cand.size(), 0);
    for (size_t i = 0; i < cand.size() && keep.size() < max_det; i++) {
        if (dead[i]) continue;
        keep.push_back(cand[i]);
        for (size_t j = i + 1; j < cand.size(); j++)
            if (!dead[j] && cand[j].cls == cand[i].cls && iou(cand[i], cand[j]) > iou_th) dead[j] = 1;
    }
    return keep;
}

static inline std::string json_escape(const std::string &s)
{
    std::string o;
    for (char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if ((unsigned char)c < 0x20) o += ' ';
        else o += c;
    }
    return o;
}

// One line of the iter2-autonomy/goto/DETECTIONS.md format (no trailing newline).
static inline std::string to_json(double t, int w, int h, const std::vector<Det> &dets,
                                  const std::vector<std::string> &names)
{
    std::string s;
    char buf[160];
    snprintf(buf, sizeof buf, "{\"t\":%.3f,\"w\":%d,\"h\":%d,\"dets\":[", t, w, h);
    s += buf;
    for (size_t i = 0; i < dets.size(); i++) {
        const Det &d = dets[i];
        std::string name = d.cls >= 0 && (size_t)d.cls < names.size() ? names[d.cls] : "class" + std::to_string(d.cls);
        snprintf(buf, sizeof buf, "%s{\"label\":\"", i ? "," : "");
        s += buf;
        s += json_escape(name);
        snprintf(buf, sizeof buf, "\",\"conf\":%.3f,\"box\":[%.1f,%.1f,%.1f,%.1f]}", d.conf, d.x1, d.y1, d.x2, d.y2);
        s += buf;
    }
    s += "]}";
    return s;
}

}  // namespace yolo
