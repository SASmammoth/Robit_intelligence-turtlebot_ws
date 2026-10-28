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

    bool stamp_from_driver_ = false;
    double stats_period_s_ = 5.0;

    // 지연 통계 [ms]
    int st_n_ = 0, st_drv_n_ = 0;
    double st_t0_ = -1.0, st_wait_ = 0.0, st_pub_ = 0.0, st_pub_max_ = 0.0, st_drv_ = 0.0, st_drv_max_ = 0.0;
};