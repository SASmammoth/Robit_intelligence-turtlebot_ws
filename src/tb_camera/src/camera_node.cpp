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
  // true면 stamp를 "드라이버가 프레임을 받은 시각"으로 당김 (지연 보정이 더 정확해짐)
  // false면 기존처럼 디코딩이 끝난 시각. 먼저 로그로 차이를 보고 켤 것
  stamp_from_driver_ = declare_parameter<bool>("stamp_from_driver", false);
  stats_period_s_ = declare_parameter<double>("stats_period_s", 5.0);   // 0이면 통계 로그 끔

  cap_.open(0, cv::CAP_V4L2);
  if (!cap_.isOpened())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
    return;
  }

  cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  RCLCPP_INFO(this->get_logger(), "Requested 640x480, actual: %.0fx%.0f",
              cap_.get(cv::CAP_PROP_FRAME_WIDTH), cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&CameraNode::timer_callback, this));
}

void CameraNode::init_transport()
{
  it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
  publisher_ = it_->advertise("camera/image_raw", 1);
}

void CameraNode::timer_callback()
{
  const double t0 = mono_ms();
  cv::Mat frame;
  cap_ >> frame;   // 다음 프레임이 올 때까지 기다림 + MJPG 디코딩
  if (frame.empty())
    return;
  const double t1 = mono_ms();
  const rclcpp::Time ros_t1 = this->now();

  // 드라이버가 프레임을 받은 시각 → 디코딩 끝난 시각 (USB 전송 + 큐 대기 + 디코딩)
  const double buf_ms = cap_.get(cv::CAP_PROP_POS_MSEC);
  const double driver_delay = (buf_ms > 0.0) ? t1 - buf_ms : -1.0;
  const bool delay_ok = driver_delay >= 0.0 && driver_delay < 1000.0;

  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  msg->header.stamp = ros_t1;
  stamp_from_driver_ = get_parameter("stamp_from_driver").as_bool();   // 실행 중 ros2 param set 가능
  if (stamp_from_driver_ && delay_ok)
    msg->header.stamp = ros_t1 - rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(driver_delay * 1e6));
  publisher_.publish(msg);   // raw + (구독자가 있으면) compressed 인코딩까지 여기서
  const double t2 = mono_ms();

  // ====== 통계 (stats_period_s마다 한 줄)
  if (stats_period_s_ <= 0.0)
    return;
  ++st_n_;
  st_wait_ += t1 - t0;
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
                "publish avg %.1f max %.1f ms%s",
                st_n_ * 1000.0 / span,
                st_drv_n_ ? st_drv_ / st_drv_n_ : -1.0, st_drv_n_ ? st_drv_max_ : -1.0,
                st_wait_ / st_n_, st_pub_ / st_n_, st_pub_max_,
                st_drv_n_ ? "" : " (버퍼 타임스탬프 없음)");
    st_n_ = st_drv_n_ = 0;
    st_wait_ = st_pub_ = st_pub_max_ = st_drv_ = st_drv_max_ = 0.0;
    st_t0_ = t2;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraNode>();
  node->init_transport(); // 노드가 shared_ptr로 만들어진 뒤에 호출해야 함
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
