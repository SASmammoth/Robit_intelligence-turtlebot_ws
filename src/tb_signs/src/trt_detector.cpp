#include "tb_signs/trt_detector.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace tb_signs
{

#define CUDA_CHECK(x)                                                              \
  do {                                                                             \
    cudaError_t e = (x);                                                           \
    if (e != cudaSuccess)                                                          \
      throw std::runtime_error(std::string("CUDA: ") + cudaGetErrorString(e));     \
  } while (0)

void TrtDetector::Logger::log(Severity severity, const char *msg) noexcept
{
  if (severity <= Severity::kWARNING)
    std::cerr << "[TensorRT] " << msg << std::endl;
}

static size_t volume(const nvinfer1::Dims &d)
{
  size_t v = 1;
  for (int i = 0; i < d.nbDims; ++i)
    v *= static_cast<size_t>(d.d[i]);
  return v;
}

TrtDetector::TrtDetector(const std::string &engine_path)
{
  // ---- 엔진 파일 읽기
  std::ifstream f(engine_path, std::ios::binary);
  if (!f)
    throw std::runtime_error("엔진 파일을 열 수 없음: " + engine_path);
  std::vector<char> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  engine_.reset(runtime_->deserializeCudaEngine(buf.data(), buf.size()));
  if (!engine_)
    throw std::runtime_error("엔진 역직렬화 실패 (Nano에서 trtexec로 다시 빌드했는지 확인)");
  ctx_.reset(engine_->createExecutionContext());
  CUDA_CHECK(cudaStreamCreate(&stream_));

  // ---- 입출력 텐서 (TensorRT 10 API)
  bool out_set = false;
  for (int i = 0; i < engine_->getNbIOTensors(); ++i)
  {
    const char *name = engine_->getIOTensorName(i);
    nvinfer1::Dims d = engine_->getTensorShape(name);

    if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT)
    {
      in_name_ = name;
      in_h_ = static_cast<int>(d.d[2]);
      in_w_ = static_cast<int>(d.d[3]);
      CUDA_CHECK(cudaMalloc(&d_in_, volume(d) * sizeof(float)));
      ctx_->setTensorAddress(name, d_in_);
    }
    else
    {
      void *p = nullptr;
      CUDA_CHECK(cudaMalloc(&p, volume(d) * sizeof(float)));
      ctx_->setTensorAddress(name, p);
      if (!out_set)  // 첫 번째 출력만 사용
      {
        out_set = true;
        d_outs_.insert(d_outs_.begin(), p);
        for (int k = 0; k < d.nbDims; ++k)
          out_dims_.push_back(d.d[k]);
        out_count_ = volume(d);
      }
      else
      {
        d_outs_.push_back(p);
      }
    }
  }
  if (in_name_.empty() || !out_set)
    throw std::runtime_error("입출력 텐서를 찾지 못함");
  h_out_.resize(out_count_);
}

TrtDetector::~TrtDetector()
{
  if (d_in_)
    cudaFree(d_in_);
  for (void *p : d_outs_)
    cudaFree(p);
  if (stream_)
    cudaStreamDestroy(stream_);
}

std::string TrtDetector::outputShapeStr() const
{
  std::ostringstream ss;
  ss << "[";
  for (size_t i = 0; i < out_dims_.size(); ++i)
    ss << out_dims_[i] << (i + 1 < out_dims_.size() ? ", " : "");
  ss << "]";
  return ss.str();
}

std::vector<Detection> TrtDetector::detect(const cv::Mat &bgr, float conf_th, float nms_th)
{
  // ---- letterbox
  const float r = std::min(static_cast<float>(in_w_) / bgr.cols,
                           static_cast<float>(in_h_) / bgr.rows);
  const int nw = static_cast<int>(std::round(bgr.cols * r));
  const int nh = static_cast<int>(std::round(bgr.rows * r));
  const int px = (in_w_ - nw) / 2;
  const int py = (in_h_ - nh) / 2;

  cv::Mat canvas(in_h_, in_w_, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::resize(bgr, canvas(cv::Rect(px, py, nw, nh)), cv::Size(nw, nh));

  // BGR→RGB, /255, HWC→CHW
  cv::Mat blob = cv::dnn::blobFromImage(canvas, 1.0 / 255.0, cv::Size(), cv::Scalar(), true, false);

  // ---- 추론
  const auto t0 = std::chrono::steady_clock::now();
  CUDA_CHECK(cudaMemcpyAsync(d_in_, blob.ptr<float>(), blob.total() * sizeof(float),
                             cudaMemcpyHostToDevice, stream_));
  if (!ctx_->enqueueV3(stream_))
    throw std::runtime_error("enqueueV3 실패");
  CUDA_CHECK(cudaMemcpyAsync(h_out_.data(), d_outs_[0], out_count_ * sizeof(float),
                             cudaMemcpyDeviceToHost, stream_));
  CUDA_CHECK(cudaStreamSynchronize(stream_));
  last_ms_ = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();

  return decode(r, px, py, bgr.cols, bgr.rows, conf_th, nms_th);
}

std::vector<Detection> TrtDetector::decode(float r, int px, int py, int img_w, int img_h,
                                           float conf_th, float nms_th) const
{
  std::vector<Detection> res;
  if (out_dims_.size() != 3)
    return res;

  const int64_t a = out_dims_[1], b = out_dims_[2];
  const float *o = h_out_.data();

  auto to_img = [&](float x1, float y1, float x2, float y2)
  {
    x1 = std::clamp((x1 - px) / r, 0.f, static_cast<float>(img_w));
    x2 = std::clamp((x2 - px) / r, 0.f, static_cast<float>(img_w));
    y1 = std::clamp((y1 - py) / r, 0.f, static_cast<float>(img_h));
    y2 = std::clamp((y2 - py) / r, 0.f, static_cast<float>(img_h));
    return cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
  };

  // ---- end-to-end: [1, N, 6]
  if (b == 6)
  {
    for (int64_t i = 0; i < a; ++i)
    {
      const float *d = o + i * 6;
      if (d[4] < conf_th)
        continue;
      res.push_back({to_img(d[0], d[1], d[2], d[3]), d[4], static_cast<int>(d[5])});
    }
    return res;
  }

  // ---- 기존 형태: [1, 4+nc, A] (채널 우선) 또는 [1, A, 4+nc]
  const bool ch_first = a < b;
  const int64_t ch = ch_first ? a : b;
  const int64_t anchors = ch_first ? b : a;
  const int nc = static_cast<int>(ch - 4);
  auto at = [&](int64_t anchor, int64_t c)
  { return ch_first ? o[c * anchors + anchor] : o[anchor * ch + c]; };

  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  std::vector<int> ids;
  for (int64_t i = 0; i < anchors; ++i)
  {
    int best = 0;
    float bs = at(i, 4);
    for (int c = 1; c < nc; ++c)
    {
      const float s = at(i, 4 + c);
      if (s > bs) { bs = s; best = c; }
    }
    if (bs < conf_th)
      continue;
    const float cx = at(i, 0), cy = at(i, 1), w = at(i, 2), h = at(i, 3);
    cv::Rect2f rb = to_img(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2);
    boxes.emplace_back(rb);
    scores.push_back(bs);
    ids.push_back(best);
  }
  // 클래스별 NMS (다른 클래스끼리는 겹쳐도 유지)
  std::vector<int> keep;
  cv::dnn::NMSBoxesBatched(boxes, scores, ids, conf_th, nms_th, keep);
  for (int k : keep)
    res.push_back({cv::Rect2f(boxes[k]), scores[k], ids[k]});
  return res;
}

}  // namespace tb_signs
