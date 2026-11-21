// detect: QNX-native object detector for goto/. Camera -> YOLO (ONNX Runtime,
// CPU) -> one JSON line per frame (goto/DETECTIONS.md) over UDP to goto.
//
//   detect --camera camapi:1 --model models/yolo_world.onnx --labels models/labels.txt
//
// Built two ways:
//   with -DHAVE_ORT    real inference (ONNX Runtime C++ API; QNX port:
//                      github.com/qnx-ports/onnxruntime, branch qnx-v1.23.2)
//   without            --nomodel only: camera + JSON plumbing, empty dets
// -DHAVE_CAMAPI adds the QNX Camera library source (camapi:N).
//
// It runs at normal priority on purpose: heavy ML must never starve
// tars_motor (SCHED_RR 30) or goto (SCHED_RR 20).
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "camera.hpp"
#include "yolo_post.hpp"

#ifdef HAVE_ORT
#include <onnxruntime_cxx_api.h>
#endif

static double mono_s()
{
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

static void usage(const char *a0)
{
    fprintf(stderr,
            "usage: %s [options]\n"
            "  --camera SPEC     camapi[:N] (QNX) | raw:WxH[:yuyv|rgb24][:path] | ppm:path\n"
            "  --model PATH      ONNX model (default models/yolo_world.onnx)\n"
            "  --labels PATH     class names, one per line (default models/labels.txt)\n"
            "  --nomodel         no inference: test the camera + JSON path (empty dets)\n"
            "  --imgsz N         network input size if the model is dynamic (default 320)\n"
            "  --conf X          confidence threshold (default 0.30)\n"
            "  --iou X           NMS IoU (default 0.45)\n"
            "  --fps X           cap the frame rate to save CPU (default 8, 0 = no cap)\n"
            "  --threads N       ONNX Runtime intra-op threads (default 2)\n"
            "  --udp HOST:PORT   where goto listens (default 127.0.0.1:5558)\n"
            "  --no-udp          don't send UDP\n"
            "  --stdout          also print each JSON line (detect --stdout --no-udp | goto -i)\n"
            "  --frames N        stop after N frames (benchmarks / tests)\n"
            "  --quiet           no stats on stderr\n",
            a0);
}

int main(int argc, char **argv)
{
#ifdef HAVE_CAMAPI
    std::string cam = "camapi:1";
#else
    std::string cam;
#endif
    std::string model = "models/yolo_world.onnx", labels_path = "models/labels.txt", udp = "127.0.0.1:5558";
    bool nomodel = false, to_stdout = false, use_udp = true, quiet = false;
    int imgsz = 320, threads = 2;
    long max_frames = 0;
    float conf = 0.30f, iou_th = 0.45f;
    double fps_cap = 8.0;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) { usage(argv[0]); exit(2); }
            return argv[++i];
        };
        if (a == "--camera") cam = next();
        else if (a == "--model") model = next();
        else if (a == "--labels") labels_path = next();
        else if (a == "--nomodel") nomodel = true;
        else if (a == "--imgsz") imgsz = atoi(next().c_str());
        else if (a == "--conf") conf = (float)atof(next().c_str());
        else if (a == "--iou") iou_th = (float)atof(next().c_str());
        else if (a == "--fps") fps_cap = atof(next().c_str());
        else if (a == "--threads") threads = atoi(next().c_str());
        else if (a == "--udp") udp = next();
        else if (a == "--no-udp") use_udp = false;
        else if (a == "--stdout") to_stdout = true;
        else if (a == "--frames") max_frames = atol(next().c_str());
        else if (a == "--quiet") quiet = true;
        else { usage(argv[0]); return a == "-h" || a == "--help" ? 0 : 2; }
    }
    if (cam.empty()) { fprintf(stderr, "detect: --camera is required on this build (no camapi)\n"); return 2; }

    // labels
    std::vector<std::string> names;
    {
        std::ifstream f(labels_path);
        std::string line;
        while (std::getline(f, line)) {
            while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
            if (!line.empty()) names.push_back(line);
        }
        if (names.empty() && !nomodel) fprintf(stderr, "detect: no labels in %s, using class<N>\n", labels_path.c_str());
    }

#ifdef HAVE_ORT
    std::unique_ptr<Ort::Env> env;
    std::unique_ptr<Ort::Session> session;
    std::string in_name, out_name;
    if (!nomodel) {
        try {
            env.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "detect"));
            Ort::SessionOptions so;
            so.SetIntraOpNumThreads(threads);
            so.SetInterOpNumThreads(1);
            so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            session.reset(new Ort::Session(*env, model.c_str(), so));
            Ort::AllocatorWithDefaultOptions al;
            in_name = session->GetInputNameAllocated(0, al).get();
            out_name = session->GetOutputNameAllocated(0, al).get();
            auto shape = session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            if (shape.size() == 4 && shape[2] > 0 && shape[2] == shape[3]) imgsz = (int)shape[2];
            auto oshape = session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            fprintf(stderr, "detect: model %s input %d out [%lld,%lld,%lld] labels=%zu threads=%d\n", model.c_str(), imgsz,
                    (long long)oshape[0], (long long)(oshape.size() > 1 ? oshape[1] : 0),
                    (long long)(oshape.size() > 2 ? oshape[2] : 0), names.size(), threads);
        } catch (const Ort::Exception &e) {
            fprintf(stderr, "detect: can't load %s: %s\n", model.c_str(), e.what());
            return 1;
        }
    }
#else
    (void)imgsz; (void)threads; (void)conf; (void)iou_th;
    if (!nomodel) {
        fprintf(stderr, "detect: built without ONNX Runtime (-DHAVE_ORT); only --nomodel works\n");
        return 2;
    }
#endif

    std::string err;
    auto src = open_source(cam, err);
    if (!src) { fprintf(stderr, "detect: camera: %s\n", err.c_str()); return 1; }

    int sock = -1;
    sockaddr_in dst{};
    if (use_udp) {
        size_t c = udp.rfind(':');
        std::string host = c == std::string::npos ? "127.0.0.1" : udp.substr(0, c);
        dst.sin_family = AF_INET;
        dst.sin_port = htons((unsigned short)atoi(c == std::string::npos ? udp.c_str() : udp.c_str() + c + 1));
        if (inet_pton(AF_INET, host.c_str(), &dst.sin_addr) != 1) { fprintf(stderr, "detect: bad --udp %s\n", udp.c_str()); return 2; }
        sock = socket(AF_INET, SOCK_DGRAM, 0);
    }

    yolo::Image rgb;
    std::vector<float> chw;
    long frames = 0;
    double t_stats = mono_s(), infer_sum = 0, next_frame = 0;
    long stat_frames = 0;
    for (;;) {
        if (max_frames && frames >= max_frames) break;
        if (fps_cap > 0) {
            double now = mono_s();
            if (now < next_frame) std::this_thread::sleep_for(std::chrono::duration<double>(next_frame - now));
            next_frame = std::max(next_frame + 1.0 / fps_cap, mono_s());
        }
        if (!src->grab(rgb)) { fprintf(stderr, "detect: camera: %s\n", src->error.c_str()); break; }
        double t0 = mono_s();
        std::vector<yolo::Det> dets;
#ifdef HAVE_ORT
        if (!nomodel) {
            yolo::Letterbox lb = yolo::letterbox(rgb, imgsz, chw);
            int64_t ishape[4] = {1, 3, imgsz, imgsz};
            auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            Ort::Value in = Ort::Value::CreateTensor<float>(mem, chw.data(), chw.size(), ishape, 4);
            const char *inn[] = {in_name.c_str()}, *outn[] = {out_name.c_str()};
            auto outs = session->Run(Ort::RunOptions{nullptr}, inn, &in, 1, outn, 1);
            auto os = outs[0].GetTensorTypeAndShapeInfo().GetShape();
            int64_t A = os.size() == 3 ? os[1] : os[0], B = os.size() == 3 ? os[2] : os[1];
            dets = yolo::decode(outs[0].GetTensorData<float>(), A, B, lb, rgb.w, rgb.h, conf, iou_th);
        }
#endif
        double t1 = mono_s();
        infer_sum += t1 - t0;
        frames++;
        stat_frames++;
        std::string line = yolo::to_json(t1, rgb.w, rgb.h, dets, names);
        if (sock >= 0) sendto(sock, line.data(), line.size(), 0, (sockaddr *)&dst, sizeof dst);
        if (to_stdout) { fputs(line.c_str(), stdout); fputc('\n', stdout); fflush(stdout); }
        if (!quiet && t1 - t_stats >= 5.0) {
            fprintf(stderr, "detect: %.1f fps, %.0f ms/frame inference, %zu dets\n", stat_frames / (t1 - t_stats),
                    1000.0 * infer_sum / stat_frames, dets.size());
            t_stats = t1;
            infer_sum = 0;
            stat_frames = 0;
        }
    }
    if (sock >= 0) close(sock);
    return 0;
}
