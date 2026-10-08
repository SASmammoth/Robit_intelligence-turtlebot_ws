#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode();

    // image_transport는 shared_from_this()가 필요해서 생성자 밖에서 초기화
    void init_transport();

private:
    void timer_callback();

    cv::VideoCapture cap_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};