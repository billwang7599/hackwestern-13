// Native QNX ONNX Runtime worker. Non-safety RGB inference only; no motor API.
// YOLOX preprocessing/decoding follows upstream Apache-2.0 documentation;
// see models/README.md for pinned artifacts and provenance.
#include <onnxruntime_cxx_api.h>
#include <cstdio>
#include <jpeglib.h>
#include <setjmp.h>
#include <unistd.h>
#include <signal.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr size_t kMaxJpeg = 2000000;
constexpr int kSide = 416, kClasses = 80, kRows = 3549;
constexpr float kScore = 0.35f, kNms = 0.45f;
using Clock = std::chrono::steady_clock;
const char* labels[] = {
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat","dog",
    "horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella","handbag",
    "tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat","baseball glove",
    "skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse","remote","keyboard",
    "cell phone","microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors",
    "teddy bear","hair drier","toothbrush"};
static_assert(sizeof(labels) / sizeof(labels[0]) == kClasses);
struct Image { int width = 0, height = 0; std::vector<unsigned char> rgb; };
struct JpegError { jpeg_error_mgr base; jmp_buf jump; char message[JMSG_LENGTH_MAX]; };
void jpegFailure(j_common_ptr codec) {
    auto* error = reinterpret_cast<JpegError*>(codec->err);
    (*codec->err->format_message)(codec, error->message);
    longjmp(error->jump, 1);
}
struct JpegState {
    jpeg_decompress_struct codec{};
    JpegError error{};
    Image image;
    bool created = false;
    ~JpegState() { if (created) jpeg_destroy_decompress(&codec); }
};
Image decodeJpeg(const std::vector<unsigned char>& bytes) {
    // All libjpeg-mutated state lives on the heap, so longjmp does not leave
    // modified non-volatile automatic variables or bypass C++ initializers.
    auto state = std::make_unique<JpegState>();
    state->codec.err = jpeg_std_error(&state->error.base);
    state->error.base.error_exit = jpegFailure;
    if (setjmp(state->error.jump)) {
        throw std::runtime_error(std::string("JPEG decode: ") + state->error.message);
    }
    jpeg_create_decompress(&state->codec); state->created = true;
    jpeg_mem_src(&state->codec, bytes.data(), bytes.size());
    jpeg_read_header(&state->codec, TRUE);
    if (!state->codec.image_width || !state->codec.image_height || state->codec.image_width > 2048 || state->codec.image_height > 2048)
        throw std::runtime_error("JPEG dimensions must be 1..2048 pixels");
    state->codec.out_color_space = JCS_RGB;
    jpeg_start_decompress(&state->codec);
    state->image.width = static_cast<int>(state->codec.output_width);
    state->image.height = static_cast<int>(state->codec.output_height);
    if (state->codec.output_components != 3) throw std::runtime_error("JPEG decoder did not return RGB");
    state->image.rgb.resize(size_t(state->image.width) * state->image.height * 3);
    while (state->codec.output_scanline < state->codec.output_height) {
        JSAMPROW row = state->image.rgb.data() + size_t(state->codec.output_scanline) * state->image.width * 3;
        jpeg_read_scanlines(&state->codec, &row, 1);
    }
    jpeg_finish_decompress(&state->codec);
    return std::move(state->image);
}

std::vector<float> preprocess(const Image& image, float& ratio) {
    ratio = std::min(float(kSide) / image.width, float(kSide) / image.height);
    const int width = std::max(1, int(image.width * ratio));
    const int height = std::max(1, int(image.height * ratio));
    std::vector<float> input(3 * kSide * kSide, 114.f);
    // Upstream uses OpenCV BGR, bilinear resize, top-left padding and 0..255
    // float CHW. Do not normalize or swap to RGB for this particular checkpoint.
    for (int y = 0; y < height; ++y) {
        const float sy = std::max(0.f, std::min(float(image.height - 1), (y + .5f) * image.height / height - .5f));
        const int y0 = int(sy), y1 = std::min(y0 + 1, image.height - 1);
        const float dy = sy - y0;
        for (int x = 0; x < width; ++x) {
            const float sx = std::max(0.f, std::min(float(image.width - 1), (x + .5f) * image.width / width - .5f));
            const int x0 = int(sx), x1 = std::min(x0 + 1, image.width - 1);
            const float dx = sx - x0;
            for (int channel = 0; channel < 3; ++channel) {
                const int source = 2 - channel;
                auto pixel = [&](int xx, int yy) { return image.rgb[(size_t(yy) * image.width + xx) * 3 + source]; };
                const float top = pixel(x0, y0) * (1 - dx) + pixel(x1, y0) * dx;
                const float bottom = pixel(x0, y1) * (1 - dx) + pixel(x1, y1) * dx;
                input[size_t(channel) * kSide * kSide + y * kSide + x] = std::round(top * (1 - dy) + bottom * dy);
            }
        }
    }
    return input;
}
struct Detection { float left, top, right, bottom, confidence; int label; };
float overlap(const Detection& a, const Detection& b) {
    const float intersection = std::max(0.f, std::min(a.right, b.right) - std::max(a.left, b.left)) *
                               std::max(0.f, std::min(a.bottom, b.bottom) - std::max(a.top, b.top));
    const float area = (a.right - a.left) * (a.bottom - a.top) + (b.right - b.left) * (b.bottom - b.top) - intersection;
    return area > 0 ? intersection / area : 0;
}
std::vector<Detection> postprocess(const float* output, float ratio, int width, int height) {
    std::vector<Detection> candidates;
    int index = 0;
    for (int stride : {8, 16, 32}) {
        const int side = kSide / stride;
        for (int y = 0; y < side; ++y) for (int x = 0; x < side; ++x, ++index) {
            const float* row = output + index * (kClasses + 5);
            if (!std::isfinite(row[4]) || row[4] < kScore) continue;
            int label = -1; float score = kScore;
            for (int c = 0; c < kClasses; ++c) {
                const float confidence = row[4] * row[c + 5];
                if (std::isfinite(confidence) && confidence > score && confidence <= 1) { label = c; score = confidence; }
            }
            if (label < 0 || !std::isfinite(row[0]) || !std::isfinite(row[1]) || !std::isfinite(row[2]) || !std::isfinite(row[3]) ||
                std::abs(row[2]) > 20 || std::abs(row[3]) > 20) continue;
            const float cx = (row[0] + x) * stride / ratio, cy = (row[1] + y) * stride / ratio;
            const float w = std::exp(row[2]) * stride / ratio, h = std::exp(row[3]) * stride / ratio;
            Detection d{std::clamp(cx - w / 2, 0.f, float(width)), std::clamp(cy - h / 2, 0.f, float(height)),
                        std::clamp(cx + w / 2, 0.f, float(width)), std::clamp(cy + h / 2, 0.f, float(height)), score, label};
            if (d.right > d.left && d.bottom > d.top) candidates.push_back(d);
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) { return a.confidence > b.confidence; });
    std::vector<Detection> keep;
    // Class-agnostic NMS matches upstream default. At most20 output detections.
    for (const auto& d : candidates) {
        if (std::none_of(keep.begin(), keep.end(), [&](const auto& other) { return overlap(d, other) > kNms; })) keep.push_back(d);
        if (keep.size() == 20) break;
    }
    return keep;
}
std::string escaped(const std::string& value) {
    std::string result;
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') { result += '\\'; result += char(c); }
        else if (c < 32 || c >= 127) result += ' ';
        else result += char(c);
    }
    return result;
}
double ms(Clock::time_point start) { return std::chrono::duration<double, std::milli>(Clock::now() - start).count(); }
class Detector {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "qnx-yolox"};
    Ort::SessionOptions options;
    Ort::Session session{nullptr};
    std::string inputName, outputName, modelName;
public:
    Detector(const std::string& path, int threads) {
        modelName = path.find("yolox_tiny") != std::string::npos ? "YOLOX-Tiny" : "YOLOX-Nano";
        options.SetIntraOpNumThreads(threads);
        options.SetInterOpNumThreads(1);
        options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        options.AddConfigEntry("session.intra_op.allow_spinning", "0");
        options.AddConfigEntry("session.inter_op.allow_spinning", "0");
        session = Ort::Session(env, path.c_str(), options);
        if (session.GetInputCount() != 1 || session.GetOutputCount() != 1) throw std::runtime_error("Expected one model input/output");
        auto inputType = session.GetInputTypeInfo(0);
        auto outputType = session.GetOutputTypeInfo(0);
        auto in = inputType.GetTensorTypeAndShapeInfo();
        auto out = outputType.GetTensorTypeAndShapeInfo();
        if (in.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || in.GetShape() != std::vector<int64_t>({1,3,kSide,kSide}) ||
            out.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || out.GetShape() != std::vector<int64_t>({1,kRows,85})) {
            throw std::runtime_error("Expected official YOLOX 416 float input[1,3,416,416], raw output[1,3549,85]");
        }
        Ort::AllocatorWithDefaultOptions allocator;
        inputName = session.GetInputNameAllocated(0, allocator).get();
        outputName = session.GetOutputNameAllocated(0, allocator).get();
        std::cerr << modelName << " ready; ONNX Runtime " << OrtGetApiBase()->GetVersionString() << "; intra threads=" << threads << '\n';
    }
    void run(uint32_t id, const std::vector<unsigned char>& jpeg) {
        const auto start = Clock::now();
        auto image = decodeJpeg(jpeg);
        float ratio;
        auto input = preprocess(image, ratio);
        const std::array<int64_t,4> shape{1,3,kSide,kSide};
        auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        auto tensor = Ort::Value::CreateTensor<float>(memory, input.data(), input.size(), shape.data(), shape.size());
        const char* inputNames[] = {inputName.c_str()};
        const char* outputNames[] = {outputName.c_str()};
        const auto inferenceStart = Clock::now();
        auto result = session.Run(Ort::RunOptions{nullptr}, inputNames, &tensor, 1, outputNames, 1);
        const double inferenceTime = ms(inferenceStart);
        const auto dimensions = result.at(0).GetTensorTypeAndShapeInfo();
        if (dimensions.GetElementCount() != size_t(kRows * 85)) throw std::runtime_error("Unexpected output dimensions");
        auto detections = postprocess(result[0].GetTensorData<float>(), ratio, image.width, image.height);
        std::cout << std::fixed << std::setprecision(6) << "{\"id\":" << id << ",\"width\":" << image.width
                  << ",\"height\":" << image.height << ",\"model\":\"" << modelName << "\",\"backend\":\"ONNX Runtime CPU\",\"runtime\":\""
                  << OrtGetApiBase()->GetVersionString() << "\",\"inference_ms\":" << inferenceTime << ",\"processing_ms\":" << ms(start)
                  << ",\"objects\":[";
        bool first = true;
        for (const auto& d : detections) {
            if (!first) std::cout << ',';
            first = false;
            std::cout << "{\"label\":\"" << labels[d.label] << "\",\"confidence\":" << d.confidence << ",\"bbox\":["
                      << d.left / image.width << ',' << d.top / image.height << ',' << (d.right - d.left) / image.width
                      << ',' << (d.bottom - d.top) / image.height << "]}";
        }
        std::cout << "]}" << std::endl;
        if (!std::cout) throw std::runtime_error("Result channel closed");
    }
};

bool exact(unsigned char* data, size_t count, bool eofAllowed = false) {
    size_t offset = 0;
    while (offset < count) {
        const ssize_t n = ::read(STDIN_FILENO, data + offset, count - offset);
        if (n < 0 && errno == EINTR) continue;
        if (n == 0 && offset == 0 && eofAllowed) return false;
        if (n <= 0) throw std::runtime_error("Truncated worker input");
        offset += size_t(n);
    }
    return true;
}
uint32_t be32(const unsigned char* p) { return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | uint32_t(p[3]); }
std::vector<unsigned char> load(const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    const auto size = file.tellg();
    if (!file || size < 4 || size > std::streamoff(kMaxJpeg)) throw std::runtime_error("Cannot read bounded JPEG file");
    std::vector<unsigned char> bytes(static_cast<size_t>(size));
    file.seekg(0); file.read(reinterpret_cast<char*>(bytes.data()), size);
    if (!file) throw std::runtime_error("Short JPEG file read");
    return bytes;
}
}
int main(int argc, char** argv) {
    signal(SIGPIPE, SIG_IGN);
    std::locale::global(std::locale::classic());
    try {
        std::string model, image;
        int threads = 1, repeat = 1;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (i + 1 >= argc) throw std::runtime_error("Every option requires a value");
            const std::string value = argv[++i];
            if (arg == "--model") model = value;
            else if (arg == "--image") image = value;
            else if (arg == "--threads") threads = std::stoi(value);
            else if (arg == "--repeat") repeat = std::stoi(value);
            else throw std::runtime_error("Unknown option: " + arg);
        }
        if (model.empty() || threads < 1 || threads > 2 || repeat < 1 || repeat > 100) throw std::runtime_error("Usage: ai_detector --model PATH [--threads 1|2] [--image JPEG --repeat 1..100]");
        Detector detector(model, threads);
        if (!image.empty()) {
            const auto jpeg = load(image);
            for (int i = 0; i < repeat; ++i) detector.run(uint32_t(i), jpeg);
            return 0;
        }
        unsigned char header[8];
        while (exact(header, sizeof(header), true)) {
            const auto id = be32(header), length = be32(header + 4);
            if (length < 4 || length > kMaxJpeg) throw std::runtime_error("JPEG length outside 4..2000000 bytes");
            std::vector<unsigned char> jpeg(length);
            exact(jpeg.data(), jpeg.size());
            try { detector.run(id, jpeg); }
            catch (const std::exception& error) {
                std::cout << "{\"id\":" << id << ",\"error\":\"" << escaped(error.what()) << "\",\"objects\":[]}" << std::endl;
                if (!std::cout) return 1;
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "ai_detector: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
