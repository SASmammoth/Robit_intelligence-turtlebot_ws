#include "tb_camera/camera_node.hpp"

#include <time.h>

#include <algorithm>

namespace
{
// V4L2 버퍼 타임스탬프와 같은 시계 (UVC 기본값 CLOCK_MONOTONIC) [ms]
double mono_ms()
{
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}
}  // namespace

CameraNode::CameraNode() : Node("camera_node")
{
  // true면 stamp를 "드라이버가 프레임을 받은 시각"으로 당김 (지연 보정이 더 정확해짐). 실행 중 바꿀 수 있음
  declare_parameter<bool>("stamp_from_driver", false);
  stats_period_s_ = declare_parameter<double>("stats_period_s", 5.0);   // 0이면 통계 로그 끔
  drop_ms_ = declare_parameter<int>("drop_ms", 60);
  max_drops_ = std::max(0, static_cast<int>(declare_parameter<int>("max_drops", 3)));
  const int buffers = declare_parameter<int>("buffers", 2);              // V4L2 버퍼 수 (적을수록 덜 쌓임)

  cap_.open(0, cv::CAP_V4L2);
  if (!cap_.isOpened())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
    return;
  }

  cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  if (buffers > 0)
    cap_.set(cv::CAP_PROP_BUFFERSIZE, buffers);

  RCLCPP_INFO(this->get_logger(), "Requested 640x480, actual: %.0fx%.0f, buffers %.0f, drop_ms %d",
              cap_.get(cv::CAP_PROP_FRAME_WIDTH), cap_.get(cv::CAP_PROP_FRAME_HEIGHT),
              cap_.get(cv::CAP_PROP_BUFFERSIZE), drop_ms_);
}

CameraNode::~CameraNode()
{
  running_ = false;
  if (thread_.joinable())
    thread_.join();
}

void CameraNode::init_transport()
{
  it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
  // best effort, 마지막 1장만: 못 받은 프레임은 다시 보내지 않음 (WiFi 너머 GUI가 느려도 카메라가 안 기다림)
  // 구독 쪽도 best effort여야 연결됨 (lineDetect, YOLO, GUI 모두 SensorDataQoS)
  rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
  qos.depth = 1;
  publisher_ = it_->advertise("camera/image_raw", qos);

  if (cap_.isOpened())
  {
    running_ = true;
    thread_ = std::thread(&CameraNode::capture_loop, this);
  }
}

void CameraNode::capture_loop()
{
  while (running_ && rclcpp::ok())
    grab_one();
}

void CameraNode::grab_one()
{
  const double t0 = mono_ms();
  if (!cap_.grab())   // 다음 프레임이 올 때까지 기다림 (디코딩은 아직 안 함)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "camera grab failed");
    return;
  }
  const double tg = mono_ms();

  // 드라이버가 이 프레임을 받은 시각. 오래됐으면 = 앞에서 막혀서 버퍼에 쌓여 있던 옛 프레임
  // → 디코딩하지 않고 버려서 바로 다음(더 새) 프레임으로 따라잡음
  const double buf_ms = cap_.get(cv::CAP_PROP_POS_MSEC);
  const double age_at_grab = (buf_ms > 0.0) ? tg - buf_ms : -1.0;
  if (drop_ms_ > 0 && age_at_grab > drop_ms_ && age_at_grab < 1000.0 && drops_in_row_ < max_drops_)
  {
    ++drops_in_row_;
    ++st_drop_;
    return;
  }
  drops_in_row_ = 0;

  cv::Mat frame;
  if (!cap_.retrieve(frame) || frame.empty())   // MJPG 디코딩
    return;
  const double t1 = mono_ms();
  const rclcpp::Time ros_t1 = this->now();

  // 드라이버가 프레임을 받은 시각 → 디코딩 끝난 시각 (USB 전송 + 큐 대기 + 디코딩)
  const double driver_delay = (buf_ms > 0.0) ? t1 - buf_ms : -1.0;
  const bool delay_ok = driver_delay >= 0.0 && driver_delay < 1000.0;

  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  msg->header.stamp = ros_t1;
  if (get_parameter("stamp_from_driver").as_bool() && delay_ok)   // 실행 중 ros2 param set 가능
    msg->header.stamp = ros_t1 - rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(driver_delay * 1e6));
  if (!rclcpp::ok())
    return;
  publisher_.publish(msg);   // raw + (구독자가 있으면) compressed 인코딩까지 여기서
  const double t2 = mono_ms();

  // ====== 통계 (stats_period_s마다 한 줄)
  if (stats_period_s_ <= 0.0)
    return;
  ++st_n_;
  st_wait_ += tg - t0;
  st_pub_ += t2 - t1;
  st_pub_max_ = std::max(st_pub_max_, t2 - t1);
  if (delay_ok)
  {
    ++st_drv_n_;
    st_drv_ += driver_delay;
    st_drv_max_ = std::max(st_drv_max_, driver_delay);
  }
  if (st_t0_ < 0.0)
    st_t0_ = t2;
  const double span = t2 - st_t0_;
  if (span >= stats_period_s_ * 1000.0)
  {
    RCLCPP_INFO(get_logger(),
                "[latency] fps %.1f | 드라이버→디코딩 끝 avg %.1f max %.1f ms | 프레임 대기 avg %.1f ms | "
                "publish avg %.1f max %.1f ms | 옛 프레임 버림 %d%s",
                st_n_ * 1000.0 / span,
                st_drv_n_ ? st_drv_ / st_drv_n_ : -1.0, st_drv_n_ ? st_drv_max_ : -1.0,
                st_wait_ / st_n_, st_pub_ / st_n_, st_pub_max_, st_drop_,
                st_drv_n_ ? "" : " (버퍼 타임스탬프 없음)");
    st_n_ = st_drv_n_ = st_drop_ = 0;
    st_wait_ = st_pub_ = st_pub_max_ = st_drv_ = st_drv_max_ = 0.0;
    st_t0_ = t2;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraNode>();
  node->init_transport(); // 노드가 shared_ptr로 만들어진 뒤에 호출해야 함 (캡처 스레드 시작)
  rclcpp::spin(node);     // 파라미터 서비스용
  rclcpp::shutdown();
  return 0;
}
