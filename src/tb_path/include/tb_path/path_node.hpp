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
#include "tb_path/obstacles.hpp"

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

    // 공사 구간: 따로 경로를 만들지 않고 일반 주행(스켈레톤) 그대로, 파라미터만 바꿈
    //   - 차선 폭 56cm, 벽돌 벽을 크게(깊이·좌우 여유), 화면 끝에 붙은 벽돌은 화면 끝까지 막음
    //   - tb_drive는 /lane/construction을 보고 속도·lookahead·피봇 파라미터를 바꿈
    // 진입은 /mission/construction true로만 (표지판 판단은 tb_brain, 수동은 GUI C키)
    // 끝: /mission/construction false, /mission/construction_exit, 또는 벽돌 기준 자동 종료
    void update_construction(const tb_interfaces::msg::LaneMask &msg, bool brick_seen);
    bool con_active_ = false;
    enum class ConCmd { None, Start, Stop, Exit };
    ConCmd con_cmd_ = ConCmd::None;      // 토픽으로 받은 명령 (다음 마스크 프레임에서 처리)
    bool con_brick_seen_ = false;       // 공사 구간에서 벽돌을 한 번이라도 봤는지
    lane_path::Pose2D con_brick_pose_;  // 마지막으로 벽돌을 본 위치
    int con_depth_mm_ = 200, con_margin_mm_ = 90, con_edge_mm_ = 60, con_exit_mm_ = 600, con_brick_w_mm_ = 200;
    int con_smooth_mm_ = 80, con_attach_mm_ = 300, con_extend_mm_ = 400;
    static void smooth_path(std::vector<cv::Point2d> &path, double half); // 모서리 둥글게 (공사 구간)
    // 공사 구간: 화면 아랫줄을 막 벗어나는 벽돌을 쪽마다 하나 기억 (로봇 좌표, 이동 거리만큼 뒤로 옮김)
    struct ConHold { bool on = false; obstacles::Quad q; double travel0 = 0.0; };
    ConHold con_hold_[2];                  // [0] 오른쪽, [1] 왼쪽
    double con_travel_ = 0.0;              // 공사 구간에서 지금까지 이동 거리
    lane_path::Pose2D con_last_pose_;
    bool con_have_pose_ = false;
    std::vector<obstacles::Quad> con_near_; // 이번 프레임에 경로를 떼어 놓은 벽돌 (로봇 좌표, 디버그)
    int con_keep_ = 1, con_keep_rear_mm_ = 100;
    int con_hold_mm_ = 40, con_clear_mm_ = 30, con_dead_end_mm_ = 250;
    int con_no_brick_mm_ = 2000; // 진입 후 벽돌을 한 번도 못 보고 이만큼 가면 끝 (잘못 들어온 것)
    int con_cross_override_frames_ = 15; // 공사 구간: 차선 너머 새 경로는 이만큼 연속으로 보여야 믿음
    int override_frames_ = 10, con_override_frames_ = 1; // 일반 / 공사 구간 path.override_frames
    bool con_straight_by_lane_ = true;
    bool con_offscreen_side_ = true;     // path.con_offscreen_side
    lane_path::Pose2D con_start_pose_; // 공사 모드 시작 위치
    std::string con_reason_ = "-";     // 마지막 시작·끝 이유 (표시용)
    void end_construction(const std::string &why);
    std::string construction_info() const;
    int turn_radius_mm_ = 150, turn_min_deg_ = 45; // 급한 모서리를 원호로 (lanePath fillet_corners)
    bool con_on() const { return con_active_; }

    // 미션 (판단 노드 또는 ros2 topic pub 으로 받음)
    uint8_t turn_ = 0;                  // 0 직진, 1 좌, 2 우
    uint8_t last_turn_ = 0;             // 마지막으로 처리한 turn (바뀌었는지 확인용)
    bool turn_drop_memory_ = true;      // path.turn_drop_memory
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
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr con_exit_sub_;  // /mission/construction_exit
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr con_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr con_info_pub_;  // /lane/construction_info (GUI 표시용)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr src_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;

    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_cb_handle_;
};
