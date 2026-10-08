#include "tb_camera/camera_node.hpp"

CameraNode::CameraNode() : Node("camera_node")
{
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
  cv::Mat frame;
  cap_ >> frame;
  if (frame.empty())
    return;

  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  msg->header.stamp = this->now();
  publisher_.publish(msg);
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