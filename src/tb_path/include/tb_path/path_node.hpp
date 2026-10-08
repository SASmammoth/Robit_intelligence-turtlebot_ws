#pragma once
// tb_path: /lane/mask + /signs/detections → 스켈레톤 → 주행 경로 /lane/path
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "tb_interfaces/msg/lane_mask.hpp"
#include "tb_interfaces/msg/detection_array.hpp"
#include "tb_path/laneSkeleton.hpp"
#include "tb_path/lanePath.hpp"

class PathNode : public rclcpp::Node
{
public:
    PathNode();

private:
    void mask_callback(const tb_interfaces::msg::LaneMask::ConstSharedPtr msg);
    void reload_params();
    void declare_int(const std::string &name, int def, int max);
    void update_pose(const rclcpp::Time &now);
    void publish_debug(const cv::Mat &img, const std_msgs::msg::Header &header);

    // 스켈레톤
    lane_skel::Params skel_p_;
    cv::Point last_seed_{-1, -1};
    int dash_len_ = 40;

    // 경로
    lane_path::Tracker tracker_;
    lane_path::Geo geo_;
    lane_path::Source last_src_ = lane_path::Source::Lost;
    std::array<double, 9> last_h_{};   // BEV가 바뀌었는지 확인용

    // 미션 (판단 노드 또는 ros2 topic pub 으로 받음)
    uint8_t turn_ = 0;                  // 0 직진, 1 좌, 2 우
    bool parking_ = false;

    // 장애물 (YOLO brick)
    tb_interfaces::msg::DetectionArray::ConstSharedPtr last_det_;
    int brick_depth_mm_ = 60;           // 벽돌 앞뒤 길이
    int brick_margin_mm_ = 30;          // 박스 좌우 여유
    int det_max_age_ms_ = 300;          // 이보다 오래된 탐지는 무시

    // 이동량: /odom 있으면 사용, 없으면 /cmd_vel 적분, 둘 다 없으면 정지로 봄
    enum class MotionSrc { None, Odom, CmdVel };
    lane_path::Pose2D pose_;
    MotionSrc motion_src_ = MotionSrc::None;
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_integ_time_{0, 0, RCL_ROS_TIME};
    geometry_msgs::msg::Twist last_cmd_;

    rclcpp::Subscription<tb_interfaces::msg::LaneMask>::SharedPtr mask_sub_;
    rclcpp::Subscription<tb_interfaces::msg::DetectionArray>::SharedPtr det_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr turn_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr parking_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr src_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;

    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_cb_handle_;
};
