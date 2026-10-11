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
#include "tb_path/construction.hpp"

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
    int last_msg_lane_w_ = -1;          // tb_line이 보낸 차선 폭 (공사 구간에서 덮어쓰므로 따로 기억)

    // 공사 구간 (construction.hpp). /mission/construction 수동 또는 표지판으로 자동 진입
    void update_construction(const tb_interfaces::msg::LaneMask &msg, const std::array<double, 9> &h);
    void draw_construction(cv::Mat &view) const;
    construction::Planner con_;
    bool con_manual_ = false, con_auto_ = false, con_done_ = false;
    int con_sign_seen_ = 0;             // 표지판이 연속으로 보인 프레임 수
    rclcpp::Time con_sign_time_{0, 0, RCL_ROS_TIME};
    builtin_interfaces::msg::Time con_last_det_stamp_;
    int con_lane_w_mm_ = 560, con_sign_frames_ = 3;
    std::vector<cv::Point2d> last_out_path_; // 직전 출력 경로 (진입 때 차선 방향 추정)
    bool con_on() const { return (con_manual_ || con_auto_) && !con_done_; }

    // 미션 (판단 노드 또는 ros2 topic pub 으로 받음)
    uint8_t turn_ = 0;                  // 0 직진, 1 좌, 2 우
    bool parking_ = false;

    // 장애물 (YOLO brick)
    tb_interfaces::msg::DetectionArray::ConstSharedPtr last_det_;
    int brick_depth_mm_ = 60;           // 벽돌 앞뒤 길이
    int brick_margin_mm_ = 30;          // 박스 좌우 여유
    int brick_gap_mm_ = 120;            // 벽돌 끝~차선 틈이 이보다 좁으면 막음
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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr con_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr con_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr src_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;

    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_cb_handle_;
};
