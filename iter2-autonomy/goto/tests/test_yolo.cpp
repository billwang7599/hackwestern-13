// Host tests for yolo_post.hpp (detect.cpp's pre/post-processing) and the
// detect -> goto JSON contract (to_json output parsed by goto_core).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "../src/goto_core.h"
#include "../src/yolo_post.hpp"

static int passed, failed;
#define CHECK(c, ...) do { if (c) passed++; else { failed++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

using namespace yolo;

static void test_color()
{
    // YUYV: white, black, mid grey pixel pairs
    uint8_t yuyv[] = {235, 128, 235, 128, 16, 128, 16, 128};
    Image o;
    yuyv_to_rgb(yuyv, 4, 1, 8, false, o);
    CHECK(o.px[0] >= 254 && o.px[1] >= 254 && o.px[2] >= 254, "Y=235 -> white (%d %d %d)", o.px[0], o.px[1], o.px[2]);
    CHECK(o.px[6] <= 1 && o.px[7] <= 1 && o.px[8] <= 1, "Y=16 -> black");
    // pure red in BT.601: Y=81 U=90 V=240
    uint8_t red[] = {81, 90, 81, 240};
    yuyv_to_rgb(red, 2, 1, 4, false, o);
    CHECK(o.px[0] > 240 && o.px[1] < 15 && o.px[2] < 15, "red (%d %d %d)", o.px[0], o.px[1], o.px[2]);
    uint8_t redu[] = {90, 81, 240, 81};
    yuyv_to_rgb(redu, 2, 1, 4, true, o);
    CHECK(o.px[0] > 240 && o.px[1] < 15, "UYVY red");
    uint8_t ny[] = {81, 81, 81, 81}, nuv[] = {90, 240};
    nv12_to_rgb(ny, 2, nuv, 2, 2, 2, o);
    CHECK(o.px[9] > 240 && o.px[10] < 15, "NV12 red, last pixel");
    uint8_t bgrx[] = {10, 20, 30, 0};
    rgbx_to_rgb(bgrx, 1, 1, 4, true, o);
    CHECK(o.px[0] == 30 && o.px[1] == 20 && o.px[2] == 10, "BGRX swap");
}

static void test_letterbox()
{
    Image img;
    img.resize(640, 480);
    for (int y = 0; y < 480; y++)
        for (int x = 0; x < 640; x++) {
            uint8_t *p = &img.px[((size_t)y * 640 + x) * 3];
            p[0] = x < 320 ? 255 : 0;       // left half red
            p[1] = 0;
            p[2] = x < 320 ? 0 : 255;       // right half blue
        }
    std::vector<float> t;
    Letterbox lb = letterbox(img, 320, t);
    CHECK(std::fabs(lb.scale - 0.5f) < 1e-6 && lb.padx == 0 && std::fabs(lb.pady - 40) < 1e-6, "scale/pad %.3f %.1f %.1f",
          lb.scale, lb.padx, lb.pady);
    const float *R = t.data(), *B = t.data() + 2 * 320 * 320;
    CHECK(std::fabs(R[10 * 320 + 100] - 114 / 255.0f) < 1e-6, "top padding is 114 grey");
    CHECK(R[160 * 320 + 50] > 0.99f && B[160 * 320 + 50] < 0.01f, "left is red");
    CHECK(R[160 * 320 + 300] < 0.01f && B[160 * 320 + 300] > 0.99f, "right is blue");
    CHECK(std::fabs(R[279 * 320 + 50] - 1.0f) < 1e-3 && std::fabs(R[280 * 320 + 50] - 114 / 255.0f) < 1e-6, "image rows 40..279");
}

// Build a fake [1, 4+nc, N] (or transposed) output with given boxes in letterboxed coords.
struct Fake { float cx, cy, w, h; int cls; float s; };
static std::vector<float> fake_out(const std::vector<Fake> &f, int nc, int N, bool ch_first)
{
    int C = 4 + nc;
    std::vector<float> o((size_t)C * N, 0.0f);
    auto set = [&](int c, int i, float v) { if (ch_first) o[(size_t)c * N + i] = v; else o[(size_t)i * C + c] = v; };
    for (size_t i = 0; i < f.size(); i++) {
        set(0, (int)i, f[i].cx); set(1, (int)i, f[i].cy); set(2, (int)i, f[i].w); set(3, (int)i, f[i].h);
        set(4 + f[i].cls, (int)i, f[i].s);
    }
    return o;
}

static void test_decode()
{
    Letterbox lb;
    lb.scale = 0.5f; lb.padx = 0; lb.pady = 40; lb.size = 320;
    std::vector<Fake> f = {
        {160, 160, 40, 80, 0, 0.9f},     // class 0, centre of image
        {162, 161, 42, 78, 0, 0.6f},     // duplicate -> NMS
        {162, 161, 42, 78, 1, 0.7f},     // same place, other class -> kept
        {50, 100, 20, 20, 2, 0.2f},      // below threshold
        {300, 270, 60, 60, 3, 0.8f},     // partly outside the image -> clipped
    };
    for (int layout = 0; layout < 2; layout++) {
        bool cf = layout == 0;
        int nc = 10, N = 2100;
        auto o = fake_out(f, nc, N, cf);
        auto d = cf ? decode(o.data(), 4 + nc, N, lb, 640, 480, 0.3f) : decode(o.data(), N, 4 + nc, lb, 640, 480, 0.3f);
        CHECK(d.size() == 3, "%s: 3 dets after threshold + NMS (got %zu)", cf ? "[1,C,N]" : "[1,N,C]", d.size());
        if (d.size() < 3) continue;
        CHECK(d[0].cls == 0 && std::fabs(d[0].conf - 0.9f) < 1e-6, "highest first");
        // cx 160 -> x 320, w 40 -> 80 ; cy 160 -> (160-40)/0.5 = 240, h 80 -> 160
        CHECK(std::fabs(d[0].x1 - 280) < 0.01 && std::fabs(d[0].x2 - 360) < 0.01 && std::fabs(d[0].y1 - 160) < 0.01 &&
              std::fabs(d[0].y2 - 320) < 0.01, "letterbox undone: %.1f %.1f %.1f %.1f", d[0].x1, d[0].y1, d[0].x2, d[0].y2);
        const Det *c3 = nullptr;
        for (auto &x : d) if (x.cls == 3) c3 = &x;
        CHECK(c3 && c3->x2 <= 640 && c3->y2 <= 480, "clipped to the image");
    }
    std::vector<float> empty(14 * 100, 0.0f);
    CHECK(decode(empty.data(), 14, 100, lb, 640, 480, 0.3f).empty(), "no dets");
}

static void test_json_contract()
{
    std::vector<Det> d = {{0, 0.81f, 280, 160, 360, 320}, {4, 0.5f, 1, 2, 3, 4}};
    std::vector<std::string> names = {"trash can", "chair", "person", "backpack", "wa\"ter"};
    std::string line = to_json(12.25, 640, 480, d, names);
    goto_frame f;
    CHECK(goto_parse_frame(line.c_str(), &f) == 0, "goto parses detect's JSON: %s", line.c_str());
    CHECK(f.n == 2 && f.w == 640 && f.h == 480 && !strcmp(f.d[0].label, "trash can") && std::fabs(f.d[0].conf - 0.81) < 1e-3 &&
          f.d[0].box[2] == 360, "fields survive");
    CHECK(!strcmp(f.d[1].label, "wa\"ter"), "escaped label survives");
    std::string none = to_json(1, 320, 240, {}, names);
    CHECK(goto_parse_frame(none.c_str(), &f) == 0 && f.n == 0, "empty frame: %s", none.c_str());
    std::vector<Det> unk = {{17, 0.5f, 1, 2, 3, 4}};
    std::string u = to_json(1, 320, 240, unk, names);
    CHECK(u.find("class17") != std::string::npos, "unknown class index -> class<N>");
}

static void test_rotate()
{
    // 3x2 frame, red channel = 10*y + x (the pixel's own coordinates)
    Image in, o;
    in.resize(3, 2);
    for (int y = 0; y < 2; y++)
        for (int x = 0; x < 3; x++) in.px[(y * 3 + x) * 3] = (uint8_t)(10 * y + x);
    auto at = [&](int x, int y) { return (int)o.px[(y * o.w + x) * 3]; };
    rotate_cw(in, 90, o);   // clockwise: the bottom-left pixel becomes the top-left one
    CHECK(o.w == 2 && o.h == 3 && at(0, 0) == 10 && at(1, 0) == 0 && at(0, 2) == 12 && at(1, 2) == 2,
          "rotate 90 cw (%dx%d: %d %d %d %d)", o.w, o.h, at(0, 0), at(1, 0), at(0, 2), at(1, 2));
    rotate_cw(in, 270, o);  // counter-clockwise: the top-right pixel becomes the top-left one
    CHECK(o.w == 2 && o.h == 3 && at(0, 0) == 2 && at(1, 0) == 12 && at(0, 2) == 0 && at(1, 2) == 10,
          "rotate 270 cw (%d %d %d %d)", at(0, 0), at(1, 0), at(0, 2), at(1, 2));
    rotate_cw(in, 180, o);
    CHECK(o.w == 3 && o.h == 2 && at(0, 0) == 12 && at(2, 1) == 0, "rotate 180");
    rotate_cw(in, -90, o);  // = 270
    CHECK(o.w == 2 && at(0, 0) == 2, "rotate -90 = 270");
    rotate_cw(in, 0, o);
    CHECK(o.w == 3 && o.px == in.px, "rotate 0 copies");
}

int main()
{
    test_color();
    test_rotate();
    test_letterbox();
    test_decode();
    test_json_contract();
    printf("%d/%d checks passed\n", passed, passed + failed);
    return failed ? 1 : 0;
}
