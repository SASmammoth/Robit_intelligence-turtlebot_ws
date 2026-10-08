#include "tb_signs/trt_engine.hpp"

#include <cstdio>
#include <fstream>

void TrtEngine::Logger::log(Severity s, const char *msg) noexcept
{
    if (s <= Severity::kWARNING)
        std::fprintf(stderr, "[TensorRT] %s\n", msg);
}

TrtEngine::~TrtEngine()
{
    if (stream_) cudaStreamSynchronize(stream_);
    if (d_in_) cudaFree(d_in_);
    if (d_out_) cudaFree(d_out_);
    if (h_in_) cudaFreeHost(h_in_);
    if (h_out_) cudaFreeHost(h_out_);
    if (stream_) cudaStreamDestroy(stream_);
    ctx_.reset();     // 생성 역순으로 해제
    engine_.reset();
    runtime_.reset();
}

static size_t volume(const nvinfer1::Dims &d)
{
    size_t v = 1;
    for (int i = 0; i < d.nbDims; ++i) v *= static_cast<size_t>(d.d[i]);
    return v;
}

bool TrtEngine::load(const std::string &path, std::string &err)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) { err = "엔진 파일을 열 수 없음: " + path; return false; }
    const std::streamsize size = f.tellg();
    f.seekg(0);
    std::vector<char> blob(static_cast<size_t>(size));
    if (!f.read(blob.data(), size)) { err = "엔진 파일 읽기 실패"; return false; }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) { err = "createInferRuntime 실패"; return false; }
    engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
    if (!engine_) { err = "엔진 역직렬화 실패 (다른 TensorRT 버전/기기에서 만든 엔진이면 이 Jetson에서 다시 빌드)"; return false; }
    ctx_.reset(engine_->createExecutionContext());
    if (!ctx_) { err = "실행 컨텍스트 생성 실패"; return false; }

    // 입출력 찾기
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
        const char *name = engine_->getIOTensorName(i);
        if (engine_->getTensorDataType(name) != nvinfer1::DataType::kFLOAT) {
            err = std::string("텐서 ") + name + " 가 float32가 아님 (trtexec에 --inputIOFormats/--outputIOFormats 를 쓰지 말 것)";
            return false;
        }
        if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) in_name_ = name;
        else out_name_ = name;
    }
    if (in_name_.empty() || out_name_.empty()) { err = "입력/출력 텐서를 찾지 못함"; return false; }

    // 입력 크기 (동적 크기면 1x3x640x640 으로 고정)
    nvinfer1::Dims in = engine_->getTensorShape(in_name_.c_str());
    if (in.nbDims != 4) { err = "입력이 NCHW 4차원이 아님"; return false; }
    bool dynamic = false;
    for (int i = 0; i < 4; ++i) dynamic |= (in.d[i] < 0);
    if (dynamic) {
        in.d[0] = 1; in.d[1] = 3;
        if (in.d[2] < 0) in.d[2] = 640;
        if (in.d[3] < 0) in.d[3] = 640;
        ctx_->setInputShape(in_name_.c_str(), in);
    }
    in_h_ = static_cast<int>(in.d[2]);
    in_w_ = static_cast<int>(in.d[3]);

    const nvinfer1::Dims out = ctx_->getTensorShape(out_name_.c_str());
    out_dims_.assign(out.d, out.d + out.nbDims);
    in_count_ = volume(in);
    out_count_ = volume(out);

    // 메모리
    if (cudaStreamCreate(&stream_) != cudaSuccess ||
        cudaMallocHost(reinterpret_cast<void **>(&h_in_), in_count_ * sizeof(float)) != cudaSuccess ||
        cudaMallocHost(reinterpret_cast<void **>(&h_out_), out_count_ * sizeof(float)) != cudaSuccess ||
        cudaMalloc(&d_in_, in_count_ * sizeof(float)) != cudaSuccess ||
        cudaMalloc(&d_out_, out_count_ * sizeof(float)) != cudaSuccess) {
        err = "CUDA 메모리 할당 실패";
        return false;
    }
    ctx_->setTensorAddress(in_name_.c_str(), d_in_);
    ctx_->setTensorAddress(out_name_.c_str(), d_out_);
    return true;
}

bool TrtEngine::infer()
{
    if (cudaMemcpyAsync(d_in_, h_in_, in_count_ * sizeof(float), cudaMemcpyHostToDevice, stream_) != cudaSuccess)
        return false;
    if (!ctx_->enqueueV3(stream_))
        return false;
    if (cudaMemcpyAsync(h_out_, d_out_, out_count_ * sizeof(float), cudaMemcpyDeviceToHost, stream_) != cudaSuccess)
        return false;
    return cudaStreamSynchronize(stream_) == cudaSuccess;
}
