// Native QNX MiDaS worker. Estimated relative inverse depth only; no safety/motor API.
// Preprocessing follows upstream MiDaS MIT-licensed ONNX demo;
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
constexpr int kSide = 256, kOutput = 64;
using Clock = std::chrono::steady_clock;
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

// OpenCV INTER_CUBIC uses a=-0.75. MiDaS's published ONNX example
// stretches RGB0..1 to256×256; its graph already contains mean/std scaling.
float cubic(float x) {
    x = std::abs(x);
    constexpr float a = -.75f;
    if (x <= 1) return (a + 2) * x*x*x - (a + 3) * x*x + 1;
    if (x < 2) return a * x*x*x - 5*a * x*x + 8*a * x - 4*a;
    return 0;
}
std::vector<float> preprocess(const Image& image) {
    std::vector<float> input(3 * kSide * kSide);
    for (int y = 0; y < kSide; ++y) {
        const float sy = (y + .5f) * image.height / kSide - .5f;
        const int baseY = int(std::floor(sy));
        for (int x = 0; x < kSide; ++x) {
            const float sx = (x + .5f) * image.width / kSide - .5f;
            const int baseX = int(std::floor(sx));
            for (int channel = 0; channel < 3; ++channel) {
                float value = 0;
                for (int dy = -1; dy <= 2; ++dy) for (int dx = -1; dx <= 2; ++dx) {
                    const int yy = std::clamp(baseY + dy, 0, image.height - 1);
                    const int xx = std::clamp(baseX + dx, 0, image.width - 1);
                    value += image.rgb[(size_t(yy) * image.width + xx) * 3 + channel] *
                             cubic(sy - (baseY + dy)) * cubic(sx - (baseX + dx));
                }
                input[size_t(channel) * kSide * kSide + y * kSide + x] = value / 255.f;
            }
        }
    }
    return input;
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
class DepthEstimator {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "qnx-midas"};
    Ort::SessionOptions options;
    Ort::Session session{nullptr};
    std::string inputName, outputName;
public:
    DepthEstimator(const std::string& path, int threads) {
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
        const auto outputShape = out.GetShape();
        if (in.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || in.GetShape() != std::vector<int64_t>({1,3,kSide,kSide}) ||
            out.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || out.GetElementCount() != size_t(kSide*kSide) ||
            outputShape.size() < 2 || outputShape[outputShape.size()-2] != kSide || outputShape.back() != kSide) {
            throw std::runtime_error("Expected official MiDaS-small float input[1,3,256,256] and256×256 output");
        }
        Ort::AllocatorWithDefaultOptions allocator;
        inputName = session.GetInputNameAllocated(0, allocator).get();
        outputName = session.GetOutputNameAllocated(0, allocator).get();
        std::cerr << "MiDaS-small ready; ONNX Runtime " << OrtGetApiBase()->GetVersionString() << "; intra threads=" << threads << '\n';
    }
    void run(uint32_t id, const std::vector<unsigned char>& jpeg) {
        const auto start = Clock::now();
        auto image = decodeJpeg(jpeg);
        auto input = preprocess(image);
        const std::array<int64_t,4> shape{1,3,kSide,kSide};
        auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        auto tensor = Ort::Value::CreateTensor<float>(memory, input.data(), input.size(), shape.data(), shape.size());
        const char* inputNames[] = {inputName.c_str()};
        const char* outputNames[] = {outputName.c_str()};
        const auto inferenceStart = Clock::now();
        auto result = session.Run(Ort::RunOptions{nullptr}, inputNames, &tensor, 1, outputNames, 1);
        const double inferenceTime = ms(inferenceStart);
        if (result.at(0).GetTensorTypeAndShapeInfo().GetElementCount() != size_t(kSide*kSide)) throw std::runtime_error("Unexpected depth dimensions");
        const float* raw = result[0].GetTensorData<float>();
        std::array<float,kOutput*kOutput> depth{};
        for (int y = 0; y < kOutput; ++y) for (int x = 0; x < kOutput; ++x) {
            // Pixel-center aligned256→64 sampling. Each output represents the
            // normalized source cell center; no letterboxing or cropping.
            const int xx = 4*x+1, yy = 4*y+1;
            const float value = .25f*(raw[yy*kSide+xx] + raw[yy*kSide+xx+1] + raw[(yy+1)*kSide+xx] + raw[(yy+1)*kSide+xx+1]);
            if (!std::isfinite(value) || std::abs(value) > 1e7f) throw std::runtime_error("Nonfinite or excessive depth output");
            depth[y*kOutput+x] = std::max(0.f, value);
        }
        std::cout << std::setprecision(6) << "{\"id\":" << id << ",\"width\":" << image.width
                  << ",\"height\":" << image.height << ",\"model\":\"MiDaS-small\",\"backend\":\"ONNX Runtime CPU\",\"runtime\":\""
                  << OrtGetApiBase()->GetVersionString() << "\",\"inference_ms\":" << inferenceTime << ",\"processing_ms\":" << ms(start)
                  << ",\"kind\":\"relative_inverse_depth\",\"depth_width\":64,\"depth_height\":64,\"inverse_depth\":[";
        for (size_t i = 0; i < depth.size(); ++i) { if (i) std::cout << ','; std::cout << depth[i]; }
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
        if (model.empty() || threads < 1 || threads > 2 || repeat < 1 || repeat > 100) throw std::runtime_error("Usage: ai_depth --model PATH [--threads 1|2] [--image JPEG --repeat 1..100]");
        DepthEstimator detector(model, threads);
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
                std::cout << "{\"id\":" << id << ",\"error\":\"" << escaped(error.what()) << "\",\"inverse_depth\":[]}" << std::endl;
                if (!std::cout) return 1;
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "ai_depth: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
