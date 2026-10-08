// 표지판/장애물 검출 노드 (YOLO26 + TensorRT)
//   구독: /camera/image_raw/compressed
//   발행: /signs/detections        (tb_interfaces/DetectionArray, path_node·test_gui가 구독)
//         /signs/debug/compressed  (박스 그린 영상, test_gui USB 화면용)
#include "tb_signs/trt_detector.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <tb_interfaces/msg/detection_array.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <map>
#include <memory>

using sensor_msgs::msg::CompressedImage;
using tb_interfaces::msg::Detection;
using tb_interfaces::msg::DetectionArray;

class SignDetectNode : public rclcpp::Node
{
public:
  SignDetectNode() : Node("sign_detect_node")
  {
    const std::string share = ament_index_cpp::get_package_share_directory("tb_signs");
    const auto engine = declare_parameter<std::string>("engine_path", share + "/models/sign.engine");
    const auto names = declare_parameter<std::string>("names_path", share + "/models/data.yaml");
    conf_ = declare_parameter<double>("conf", 0.5);
    nms_ = declare_parameter<double>("nms", 0.5);
    debug_ = declare_parameter<bool>("publish_debug", true);

    loadNames(names);
    det_ = std::make_unique<tb_signs::TrtDetector>(engine);
    RCLCPP_INFO(get_logger(), "엔진 로드: %s (입력 %dx%d, 출력 %s, 클래스 %zu개)",
                engine.c_str(), det_->inputWidth(), det_->inputHeight(),
                det_->outputShapeStr().c_str(), names_.size());

    // 런타임에 conf/nms/publish_debug 변경 가능
    param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &ps)
    {
      for (const auto &p : ps)
      {
        if (p.get_name() == "conf") conf_ = p.as_double();
        else if (p.get_name() == "nms") nms_ = p.as_double();
        else if (p.get_name() == "publish_debug") debug_ = p.as_bool();
      }
      rcl_interfaces::msg::SetParametersResult r;
      r.successful = true;
      return r;
    });

    det_pub_ = create_publisher<DetectionArray>("/signs/detections", 10);
    dbg_pub_ = create_publisher<CompressedImage>("/signs/debug/compressed", rclcpp::SensorDataQoS());
    // depth 1 + best_effort: 추론이 밀리면 오래된 프레임은 버림
    sub_ = create_subscription<CompressedImage>(
        "/camera/image_raw/compressed", rclcpp::SensorDataQoS().keep_last(1),
        [this](CompressedImage::ConstSharedPtr msg) { onImage(*msg); });
  }

private:
  void loadNames(const std::string &path)
  {
    try
    {
      YAML::Node n = YAML::LoadFile(path)["names"];
      if (n.IsMap())
        for (const auto &kv : n)
          names_[kv.first.as<int>()] = kv.second.as<std::string>();
      else if (n.IsSequence())
        for (size_t i = 0; i < n.size(); ++i)
          names_[static_cast<int>(i)] = n[i].as<std::string>();
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(get_logger(), "클래스 이름 파일 읽기 실패(%s): %s → 숫자 ID로 표시",
                  path.c_str(), e.what());
    }
  }

  std::string nameOf(int id) const
  {
    auto it = names_.find(id);
    return it != names_.end() ? it->second : std::to_string(id);
  }

  void onImage(const CompressedImage &msg)
  {
    cv::Mat img = cv::imdecode(msg.data, cv::IMREAD_COLOR);
    if (img.empty())
      return;

    std::vector<tb_signs::Detection> dets;
    try
    {
      dets = det_->detect(img, static_cast<float>(conf_), static_cast<float>(nms_));
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "추론 실패: %s", e.what());
      return;
    }

    // ---- 검출 결과 발행
    DetectionArray out;
    out.header = msg.header;  // 카메라 stamp 그대로 (path_node가 마스크와 시간 비교)
    for (const auto &d : dets)
    {
      Detection det;
      det.label = nameOf(d.class_id);
      det.class_id = d.class_id;
      det.score = d.score;
      det.x = static_cast<int32_t>(std::lround(d.box.x));
      det.y = static_cast<int32_t>(std::lround(d.box.y));
      det.w = static_cast<int32_t>(std::lround(d.box.width));
      det.h = static_cast<int32_t>(std::lround(d.box.height));
      out.detections.push_back(det);
    }
    det_pub_->publish(out);

    // ---- 디버그 영상
    if (!debug_ || dbg_pub_->get_subscription_count() == 0)
      return;
    for (const auto &d : dets)
    {
      const cv::Rect r(d.box);
      cv::rectangle(img, r, {0, 255, 0}, 2);
      char label[64];
      std::snprintf(label, sizeof(label), "%s %.2f", nameOf(d.class_id).c_str(), d.score);
      cv::putText(img, label, {r.x, std::max(r.y - 5, 12)}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 0}, 2);
    }
    char ms[32];
    std::snprintf(ms, sizeof(ms), "%.1f ms", det_->lastInferMs());
    cv::putText(img, ms, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 0, 255}, 2);

    CompressedImage dbg;
    dbg.header = msg.header;
    dbg.format = "jpeg";
    cv::imencode(".jpg", img, dbg.data, {cv::IMWRITE_JPEG_QUALITY, 80});
    dbg_pub_->publish(dbg);
  }

  std::unique_ptr<tb_signs::TrtDetector> det_;
  std::map<int, std::string> names_;
  double conf_, nms_;
  bool debug_;

  rclcpp::Subscription<CompressedImage>::SharedPtr sub_;
  rclcpp::Publisher<DetectionArray>::SharedPtr det_pub_;
  rclcpp::Publisher<CompressedImage>::SharedPtr dbg_pub_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SignDetectNode>());
  rclcpp::shutdown();
  return 0;
}
