#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/string.hpp>

#include "tb_interfaces/msg/drive_cmd.hpp"
#include "tb_drive/pure_pursuit.hpp"

class DriveNode : public rclcpp::Node
{
public:
    DriveNode();

private:
    using DriveCmd = tb_interfaces::msg::DriveCmd;

    void declare_int(const std::string &name, int def, int min, int max);
    void declare_bool(const std::string &name, bool def);
    void reload_params();
    void make_timer();

    void on_timer();
    bool psd_blocked(const rclcpp::Time &t) const;
    void publish_cmd(double v, double w);
    void publish_target(const tb_drive::Target &tg, const rclcpp::Time &t);
    void publish_uart(double v, double w, double t, bool force);

    // ====== 입력
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr src_sub_;
    rclcpp::Subscription<DriveCmd>::SharedPtr cmd_sub_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> psd_subs_;

    // ====== 출력
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr uart_pub_; // TB_Uart_RX

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_cb_handle_;

    // ====== 상태
    tb_drive::Follower follower_;
    tb_drive::Limiter lim_;
    std::string path_src_ = "none";
    DriveCmd::ConstSharedPtr cmd_;
    rclcpp::Time cmd_time_;
    std::map<std::string, std::pair<double, rclcpp::Time>> psd_; // 토픽 → (거리, 받은 시각)
    double last_tick_ = -1.0;
    unsigned tick_ = 0;
    std::string last_state_;

    // ====== 파라미터
    bool enable_ = false;
    bool require_cmd_ = false;
    bool psd_enable_ = false;
    int control_hz_ = 0;
    double cmd_timeout_ = 0.5;
    double weak_scale_ = 0.6;
    double psd_stop_ = 0.15;

    // 모터 출력 (UART)
    bool uart_enable_ = true;
    bool invert_l_ = false, invert_r_ = false;
    double wheel_r_ = 0.0475, wheel_sep_ = 0.1933, rpm_per_unit_ = 0.229;
    int uart_max_raw_ = 1000;
    long uart_l_ = 0, uart_r_ = 0;
    double uart_t_ = -1.0;
    bool uart_send_zero_ = false;
};
