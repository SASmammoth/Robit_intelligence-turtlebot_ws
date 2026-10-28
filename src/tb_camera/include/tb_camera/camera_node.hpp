#pragma once

#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode();
    ~CameraNode() override;

    // image_transport는 shared_from_this()가 필요해서 생성자 밖에서 초기화 (캡처 스레드도 여기서 시작)
    void init_transport();

private:
    // 캡처 전용 스레드: 프레임이 오는 대로 읽음 (타이머로 33ms마다 하나씩 읽으면
    // publish가 한 번 막혀 V4L2 버퍼에 쌓인 옛 프레임을 끝까지 못 따라잡음)
    void capture_loop();
    void grab_one();

    cv::VideoCapture cap_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher publisher_;
    std::thread thread_;
    std::atomic<bool> running_{false};

    double stats_period_s_ = 5.0;
    int drop_ms_ = 60;    // 드라이버가 받은 지 이보다 오래된 프레임은 디코딩 없이 버림 (0이면 안 버림)
    int max_drops_ = 3;   // 연속으로 이만큼 버렸으면 다음 것은 무조건 보냄 (타임스탬프가 이상할 때 대비)
    int drops_in_row_ = 0;

    // 지연 통계 [ms]
    int st_n_ = 0, st_drv_n_ = 0, st_drop_ = 0;
    double st_t0_ = -1.0, st_wait_ = 0.0, st_pub_ = 0.0, st_pub_max_ = 0.0, st_drv_ = 0.0, st_drv_max_ = 0.0;
};
