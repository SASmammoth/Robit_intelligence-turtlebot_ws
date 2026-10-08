#pragma once

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

namespace tb_signs
{

struct Detection
{
  cv::Rect2f box;  // 원본 이미지 좌표 (x, y, w, h)
  float score;
  int class_id;
};

// TensorRT 엔진 로드 + 전처리(letterbox) + 추론 + 후처리
// 출력 형태 두 가지 모두 처리:
//   - end-to-end (NMS 없음): [1, N, 6]  x1 y1 x2 y2 score cls
//   - 기존 형태:            [1, 4+nc, A] 또는 [1, A, 4+nc]  cx cy w h + 클래스 점수 → NMS
class TrtDetector
{
public:
  explicit TrtDetector(const std::string &engine_path);
  ~TrtDetector();
  TrtDetector(const TrtDetector &) = delete;
  TrtDetector &operator=(const TrtDetector &) = delete;

  std::vector<Detection> detect(const cv::Mat &bgr, float conf_th, float nms_th);

  int inputWidth() const { return in_w_; }
  int inputHeight() const { return in_h_; }
  float lastInferMs() const { return last_ms_; }
  std::string outputShapeStr() const;

private:
  struct Logger : public nvinfer1::ILogger
  {
    void log(Severity severity, const char *msg) noexcept override;
  };

  std::vector<Detection> decode(float r, int px, int py, int img_w, int img_h,
                                float conf_th, float nms_th) const;

  Logger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> ctx_;

  cudaStream_t stream_{nullptr};
  void *d_in_{nullptr};
  std::vector<void *> d_outs_;  // [0]이 사용하는 출력, 나머지는 주소만 잡아둠

  std::string in_name_;
  int in_w_{0}, in_h_{0};
  std::vector<int64_t> out_dims_;
  size_t out_count_{0};
  std::vector<float> h_out_;
  float last_ms_{0.f};
};

}  // namespace tb_signs
