#include "tb_drive/drive_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace {
constexpr double DEG = M_PI / 180.0;
const char *MODE_NAME[] = {"FOLLOW", "STOP", "TWIST"};
} // namespace

DriveNode::DriveNode() : Node("drive_node")
{
    // ====== 파라미터 (GUI에서 바로 바꿀 수 있게 정수 mm, mm/s, deg/s, ms)
    declare_bool("drive.enable", false);       // false면 항상 0 발행. GUI에서 켜야 움직임
    declare_bool("drive.require_cmd", false);  // true: /drive/cmd 없으면 정지 (대회용)
    declare_bool("drive.latency_comp", false); // odom 생기면 켜기
    declare_int("drive.control_hz", 30, 5, 100);

    declare_int("drive.v_max_mm_s", 150, 0, 1000);
    declare_int("drive.v_min_mm_s", 40, 0, 500);
    declare_int("drive.w_max_deg_s", 90, 0, 720);
    declare_int("drive.accel_mm_s2", 300, 10, 5000);
    declare_int("drive.decel_mm_s2", 600, 10, 5000);
    declare_int("drive.w_accel_deg_s2", 360, 10, 5000);

    declare_int("drive.ld_min_mm", 150, 30, 1000);
    declare_int("drive.ld_max_mm", 350, 30, 1500);
    declare_int("drive.ld_time_ms", 1000, 0, 5000);
    declare_int("drive.curv_slow_mm", 100, 0, 1000);
    declare_int("drive.stop_dist_mm", 40, 0, 300);

    declare_int("drive.path_timeout_ms", 400, 50, 3000);
    declare_int("drive.hold_ms", 600, 0, 3000);
    declare_int("drive.cmd_timeout_ms", 500, 50, 3000);
    declare_int("drive.weak_src_pct", 60, 0, 100);

    declare_bool("drive.psd.enable", false);
    declare_int("drive.psd.stop_mm", 150, 0, 1000);
    declare_parameter("drive.psd.topics", std::vector<std::string>{"/psd/front"}); // 시작할 때만 읽음

    // ====== 모터 출력 (tb_uart_node: TB_Uart_RX "velocity L R", 다이나믹셀 원시값)
    // 하드웨어 값이라 GUI 대신 yaml에서 정함
    declare_bool("drive.uart.enable", true);
    declare_parameter("drive.uart.wheel_radius_m", 0.0475);
    declare_parameter("drive.uart.wheel_sep_m", 0.1933);
    declare_parameter("drive.uart.rpm_per_unit", 0.229); // MX-64 Protocol 2.0 Goal Velocity 단위
    declare_int("drive.uart.max_raw", 1000, 0, 2000);    // 넘으면 L/R 비율 유지한 채 축소
    declare_bool("drive.uart.invert_left", false);
    declare_bool("drive.uart.invert_right", false);

    reload_params();
    post_cb_handle_ = add_post_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &) { reload_params(); });

    // ====== 입력
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/lane/path", rclcpp::QoS(1), [this](const nav_msgs::msg::Path::ConstSharedPtr m)
        {
            std::vector<tb_drive::Pt> pts;
            pts.reserve(m->poses.size());
            for (const auto &ps : m->poses)
                pts.push_back({ps.pose.position.x, ps.pose.position.y});
            follower_.set_path(std::move(pts), rclcpp::Time(m->header.stamp, RCL_ROS_TIME).seconds()); });

    src_sub_ = create_subscription<std_msgs::msg::String>(
        "/lane/path_source", 10, [this](const std_msgs::msg::String::ConstSharedPtr m) { path_src_ = m->data; });

    cmd_sub_ = create_subscription<DriveCmd>(
        "/drive/cmd", 10, [this](const DriveCmd::ConstSharedPtr m)
        {
            if (!cmd_ || cmd_->mode != m->mode)
                RCLCPP_INFO(get_logger(), "drive cmd: %s", MODE_NAME[std::min<uint8_t>(m->mode, 2)]);
            cmd_ = m;
            cmd_time_ = now(); });

    for (const auto &topic : get_parameter("drive.psd.topics").as_string_array())
    {
        psd_subs_.push_back(create_subscription<sensor_msgs::msg::Range>(
            topic, rclcpp::SensorDataQoS(), [this, topic](const sensor_msgs::msg::Range::ConstSharedPtr m)
            { psd_[topic] = {m->range, now()}; }));
    }

    // ====== 출력
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/drive/state", 10);
    target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/drive/target", 10);
    uart_pub_ = create_publisher<std_msgs::msg::String>("TB_Uart_RX", 10);

    make_timer();
    RCLCPP_INFO(get_logger(), "drive_node started (enable=%d, require_cmd=%d)", enable_, require_cmd_);
}

void DriveNode::declare_int(const std::string &name, int def, int min, int max)
{
    rcl_interfaces::msg::ParameterDescriptor d;
    rcl_interfaces::msg::IntegerRange r;
    r.from_value = min;
    r.to_value = max;
    r.step = 1;
    d.integer_range.push_back(r);
    declare_parameter(name, def, d);
}

void DriveNode::declare_bool(const std::string &name, bool def)
{
    declare_parameter(name, def);
}

void DriveNode::reload_params()
{
    auto mm = [this](const char *n) { return get_parameter(n).as_int() / 1000.0; };
    auto deg = [this](const char *n) { return get_parameter(n).as_int() * DEG; };

    const bool enable = get_parameter("drive.enable").as_bool();
    if (enable != enable_)
    {
        lim_.reset();
        follower_.reset(); // 꺼져 있던 동안의 경로·이동 기록은 버림
        if (!enable)
            uart_send_zero_ = true; // 꺼질 때 모터에 0 한 번 보내고 그 뒤로는 조용히 (WASD와 안 섞이게)
        RCLCPP_INFO(get_logger(), "drive %s", enable ? "ENABLED" : "disabled");
    }
    enable_ = enable;
    require_cmd_ = get_parameter("drive.require_cmd").as_bool();
    follower_.latency_comp = get_parameter("drive.latency_comp").as_bool();

    auto &p = follower_.p;
    p.v_max = mm("drive.v_max_mm_s");
    p.v_min = mm("drive.v_min_mm_s");
    p.w_max = deg("drive.w_max_deg_s");
    p.accel = mm("drive.accel_mm_s2");
    p.decel = mm("drive.decel_mm_s2");
    p.w_accel = deg("drive.w_accel_deg_s2");
    p.ld_min = mm("drive.ld_min_mm");
    p.ld_max = std::max(p.ld_min, mm("drive.ld_max_mm"));
    p.ld_time = mm("drive.ld_time_ms"); // ms → s 도 /1000
    p.curv_slow = mm("drive.curv_slow_mm");
    p.stop_dist = mm("drive.stop_dist_mm");

    follower_.path_timeout = mm("drive.path_timeout_ms");
    follower_.hold_time = mm("drive.hold_ms");
    cmd_timeout_ = mm("drive.cmd_timeout_ms");
    weak_scale_ = get_parameter("drive.weak_src_pct").as_int() / 100.0;

    psd_enable_ = get_parameter("drive.psd.enable").as_bool();
    psd_stop_ = mm("drive.psd.stop_mm");

    uart_enable_ = get_parameter("drive.uart.enable").as_bool();
    wheel_r_ = std::max(1e-3, get_parameter("drive.uart.wheel_radius_m").as_double());
    wheel_sep_ = get_parameter("drive.uart.wheel_sep_m").as_double();
    rpm_per_unit_ = std::max(1e-6, get_parameter("drive.uart.rpm_per_unit").as_double());
    uart_max_raw_ = get_parameter("drive.uart.max_raw").as_int();
    invert_l_ = get_parameter("drive.uart.invert_left").as_bool();
    invert_r_ = get_parameter("drive.uart.invert_right").as_bool();

    const int hz = get_parameter("drive.control_hz").as_int();
    if (hz != control_hz_)
    {
        control_hz_ = hz;
        if (timer_)
            make_timer();
    }
}

void DriveNode::make_timer()
{
    timer_ = create_wall_timer(std::chrono::microseconds(1000000 / control_hz_), [this]() { on_timer(); });
}

bool DriveNode::psd_blocked(const rclcpp::Time &t) const
{
    if (!psd_enable_)
        return false;
    for (const auto &[topic, e] : psd_)
        if ((t - e.second).seconds() < 0.5 && std::isfinite(e.first) && e.first < psd_stop_)
            return true;
    return false;
}

void DriveNode::on_timer()
{
    const rclcpp::Time now_t = now();
    const double t = now_t.seconds();
    const double dt = last_tick_ > 0.0 ? std::clamp(t - last_tick_, 0.0, 0.1) : 1.0 / control_hz_;
    last_tick_ = t;
    ++tick_;

    std::string mode_txt = "DISABLED";
    const char *why = "";
    tb_drive::Follower::Out o;

    if (!enable_)
    {
        lim_.reset();
    }
    else
    {
        // ====== 모드 결정
        const bool cmd_fresh = cmd_ && (now_t - cmd_time_).seconds() < cmd_timeout_;
        uint8_t mode = cmd_fresh ? cmd_->mode : (require_cmd_ ? DriveCmd::STOP : DriveCmd::FOLLOW);
        if (mode > DriveCmd::TWIST)
            mode = DriveCmd::STOP;
        mode_txt = MODE_NAME[mode];
        if (!cmd_fresh && require_cmd_)
            why = "no_cmd";

        double vd = 0.0, wd = 0.0;
        if (mode == DriveCmd::TWIST)
        {
            vd = cmd_->v;
            wd = cmd_->w;
        }
        else if (mode == DriveCmd::FOLLOW)
        {
            double limit = follower_.p.v_max;
            if (cmd_fresh && cmd_->speed_limit > 0.0f)
                limit = std::min<double>(limit, cmd_->speed_limit);
            if (path_src_ == "memory" || path_src_ == "color_rule")
                limit *= weak_scale_;

            o = follower_.follow(t, lim_.v, limit);
            vd = o.v;
            wd = o.w;
            why = o.why;
        }

        // ====== 가감속 제한 → PSD (전진만 막고 즉시 정지)
        lim_.step(vd, wd, dt, follower_.p);
        if (lim_.v > 0.0 && psd_blocked(now_t))
        {
            lim_.v = 0.0;
            why = "psd";
        }
    }

    publish_cmd(lim_.v, lim_.w);
    follower_.record(t, lim_.v, lim_.w);
    if (enable_)
        publish_uart(lim_.v, lim_.w, t, false);
    else if (uart_send_zero_)
    {
        publish_uart(0.0, 0.0, t, true);
        uart_send_zero_ = false;
    }
    if (o.tg.ok)
        publish_target(o.tg, now_t);

    // ====== 상태 (10Hz)
    if (tick_ % std::max(1, control_hz_ / 10) == 0)
    {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "%s src=%s v=%.3f w=%.2f ld=%.2f end=%.2f%s%s%s",
                      mode_txt.c_str(), path_src_.c_str(), lim_.v, lim_.w, o.tg.ld, o.tg.end_dist,
                      o.held ? " HOLD" : "", *why ? " stop:" : "", why);
        std_msgs::msg::String s;
        s.data = buf;
        state_pub_->publish(s);

        const std::string key = mode_txt + why;
        if (key != last_state_)
        {
            RCLCPP_INFO(get_logger(), "%s%s%s", mode_txt.c_str(), *why ? " / " : "", why);
            last_state_ = key;
        }
    }
}

void DriveNode::publish_cmd(double v, double w)
{
    // 정지 중에도 0을 계속 발행 (tb_path가 /cmd_vel을 적분해서 위치를 추정함)
    geometry_msgs::msg::Twist tw;
    tw.linear.x = v;
    tw.angular.z = w;
    cmd_pub_->publish(tw);
}

// (v, w) → 바퀴 선속도 → rpm → 다이나믹셀 원시값 → "velocity L R"
// GUI WASD와 같은 규칙: 전진이면 L, R 둘 다 양수 (오른쪽 모터 반전은 tb_uart/STM32 쪽에서 처리)
void DriveNode::publish_uart(double v, double w, double t, bool force)
{
    if (!uart_enable_)
        return;

    const double to_raw = 60.0 / (2.0 * M_PI * wheel_r_) / rpm_per_unit_; // [m/s] → 원시값
    double l = (v - w * wheel_sep_ / 2.0) * to_raw;
    double r = (v + w * wheel_sep_ / 2.0) * to_raw;
    const double peak = std::max(std::abs(l), std::abs(r));
    if (uart_max_raw_ > 0 && peak > uart_max_raw_)
    {
        l *= uart_max_raw_ / peak; // 곡률 유지
        r *= uart_max_raw_ / peak;
    }
    const long L = std::lround(l) * (invert_l_ ? -1 : 1);
    const long R = std::lround(r) * (invert_r_ ? -1 : 1);

    // 값이 바뀌었을 때 + 같아도 0.2초마다 (UART 부하 줄이기)
    if (!force && L == uart_l_ && R == uart_r_ && t - uart_t_ < 0.2)
        return;
    uart_l_ = L;
    uart_r_ = R;
    uart_t_ = t;

    std_msgs::msg::String m;
    m.data = "velocity " + std::to_string(L) + " " + std::to_string(R);
    uart_pub_->publish(m);
}

void DriveNode::publish_target(const tb_drive::Target &tg, const rclcpp::Time &t)
{
    if (target_pub_->get_subscription_count() == 0)
        return;
    geometry_msgs::msg::PointStamped ps;
    ps.header.stamp = t;
    ps.header.frame_id = "base_link";
    ps.point.x = tg.p.x;
    ps.point.y = tg.p.y;
    target_pub_->publish(ps);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveNode>());
    rclcpp::shutdown();
    return 0;
}
