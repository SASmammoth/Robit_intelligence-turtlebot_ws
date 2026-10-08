#pragma once
// TensorRT 엔진(.engine) 로드 + 추론. 입력 1개, 출력 1개 (YOLO detect)
// TensorRT 8.6(JetPack 6.0) / 10.x(JetPack 6.1+) 둘 다 쓰는 이름 기반 API만 사용
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <memory>
#include <string>
#include <vector>

class TrtEngine
{
public:
    ~TrtEngine();

    // 실패하면 false, err에 이유
    bool load(const std::string &path, std::string &err);

    // host_input()에 전처리 결과를 채운 뒤 호출. 끝나면 host_output()에 결과
    bool infer();

    float *host_input() { return h_in_; }
    const float *host_output() const { return h_out_; }

    int in_w() const { return in_w_; }
    int in_h() const { return in_h_; }
    const std::vector<int64_t> &out_dims() const { return out_dims_; }   // 예: {1, 11, 8400}

private:
    class Logger : public nvinfer1::ILogger
    {
        void log(Severity s, const char *msg) noexcept override;
    } logger_;

    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> ctx_;
    cudaStream_t stream_ = nullptr;

    std::string in_name_, out_name_;
    int in_w_ = 0, in_h_ = 0;
    std::vector<int64_t> out_dims_;
    size_t in_count_ = 0, out_count_ = 0;

    float *h_in_ = nullptr, *h_out_ = nullptr;   // pinned host 메모리
    void *d_in_ = nullptr, *d_out_ = nullptr;    // GPU 메모리
};
