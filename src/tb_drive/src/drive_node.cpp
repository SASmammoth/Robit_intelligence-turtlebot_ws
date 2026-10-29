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
    // 코너 속도: 앞 경로(lookahead + 제동거리)가 curve_start~curve_full만큼 틀어지면 상한을 v_max → v_curve로
    declare_int("drive.v_curve_mm_s", 0, 0, 1000);      // 코너 속도 상한 (0이면 끔 = 예전처럼 v_max에 curv_slow 배율만)
    declare_int("drive.curve_start_deg", 20, 0, 180);   // 앞 경로가 이보다 덜 틀어지면 최고 속도
    declare_int("drive.curve_full_deg", 60, 1, 180);    // 이만큼 틀어지면 코너 속도
    declare_int("drive.w_max_deg_s", 90, 0, 720);
    declare_int("drive.accel_mm_s2", 300, 10, 5000);
    declare_int("drive.decel_mm_s2", 600, 10, 5000);
    declare_int("drive.w_accel_deg_s2", 360, 10, 5000);

    declare_int("drive.ld_min_mm", 150, 30, 1000);
    declare_int("drive.ld_max_mm", 350, 30, 1500);
    declare_int("drive.ld_time_ms", 1000, 0, 5000);
    declare_int("drive.ld_sharp_mm", 120, 0, 1000);       // 급한 곳에서 줄일 lookahead (0이면 끔)
    declare_int("drive.sharp_turn_deg", 60, 1, 180);      // 앞 경로가 이만큼 틀어지면 ld_sharp를 다 씀
    declare_int("drive.sharp_hold_ms", 1000, 0, 5000);    // 급한 곳을 본 뒤 이 시간 동안 짧게 유지
    declare_int("drive.sharp_hold_mm", 0, 0, 2000);       // >0이면 시간 대신 이 거리를 가는 동안 유지 (속도와 무관)
    // 교차로 명령(/mission/turn)이 없을 때: 일반 곡선(최대 ~75°)에서는 줄이지 않고, 헤어핀·명령 없는 급코너만
    declare_int("drive.sharp_curve_start_deg", 85, 0, 180);  // 이보다 덜 틀어지면 lookahead 안 줄임
    declare_int("drive.sharp_curve_full_deg", 105, 1, 180);  // 이만큼 틀어지면 ld_sharp를 다 씀
    // 교차로 명령·공사 구간이 아닐 때 경로 x에 더할 거리. bev_bottom 실측(205) - 설정값(130) = 75
    declare_int("drive.curve_x_offset_mm", 0, -200, 200);
    declare_int("drive.curv_slow_mm", 100, 0, 1000);
    declare_int("drive.stop_dist_mm", 40, 0, 300);

    declare_int("drive.path_timeout_ms", 400, 50, 3000);
    declare_int("drive.hold_ms", 600, 0, 3000);
    declare_int("drive.cmd_timeout_ms", 500, 50, 3000);
    declare_int("drive.weak_src_pct", 60, 0, 100);
    declare_int("drive.weak_delay_ms", 0, 0, 2000);
    // 한쪽 선만 보이는 상태(/lane/one_line)가 one_line_delay_ms 넘게 이어지면 속도 상한 (0 = 끔). 공사 구간은 제외
    declare_int("drive.one_line_v_mm_s", 0, 0, 1000);
    declare_int("drive.one_line_delay_ms", 800, 0, 5000);    // 약한 경로가 이만큼 이어져야 weak_src_pct로 감속 (0 = 바로, 예전과 같음)
    // 일반 주행 회전 세기 % (pure pursuit 곡률 배율. 120 = 같은 lookahead에서 반경 0.83배)
    declare_int("drive.turn_gain_pct", 100, 50, 300);
    // 교차로 명령(/mission/turn ≠ 0) 중: 최고 속도·회전 세기·최소 lookahead (공사 구간이면 공사 값이 우선)
    declare_int("drive.cross_v_mm_s", 0, 0, 1000);         // 0이면 끔 (v_max 그대로)
    declare_int("drive.cross_turn_gain_pct", 100, 50, 300);
    declare_int("drive.cross_ld_min_mm", 0, 0, 1000);      // 0이면 ld_min_mm 그대로
    declare_int("drive.cross_ld_max_mm", 0, 0, 1500);      // 0이면 ld_max_mm 그대로
    declare_int("drive.cross_v_min_mm_s", 0, 0, 500);      // 0이면 v_min_mm_s 그대로
    declare_int("drive.cross_v_curve_mm_s", 0, 0, 1000);   // 코너 속도 상한. 0이면 v_curve_mm_s 그대로
    declare_int("drive.con_v_mm_s", 100, 0, 1000);        // 공사 구간 최고 속도
    declare_int("drive.con_ld_min_mm", 250, 0, 1000);     // 공사 구간 최소 lookahead (길게: 멀리 보고 미리 완만하게 꺾음)
    declare_int("drive.con_ld_max_mm", 0, 0, 1500);       // 공사 구간 최대 lookahead (0이면 ld_max_mm 그대로)
    declare_int("drive.con_v_min_mm_s", 0, 0, 500);       // 공사 구간 최저 속도 (0이면 v_min_mm_s 그대로)
    declare_int("drive.con_v_curve_mm_s", 0, 0, 1000);    // 공사 구간 코너 속도 상한 (0이면 v_curve_mm_s 그대로)
    declare_int("drive.con_ld_sharp_mm", 0, 0, 1000);
    declare_int("drive.con_turn_gain_pct", 130, 50, 300); // 공사 구간 회전 세기 % (130 = 같은 lookahead에서 반경 0.77배)     // 공사 구간 급한 곳 lookahead (0이면 안 줄임. 줄이면 ㄱ자에서 늦게 꺾음)
    declare_int("drive.con_pivot_enter_deg", 180, 0, 180); // 공사 구간 피봇 시작 각도 (180 = 코너 피봇 안 함. 벽돌 옆 피봇은 뒤가 쓸림)
    declare_int("drive.con_pivot_keep_deg", 15, 0, 180);  // 공사 구간에서 pure pursuit에 맡기는 각도

    declare_int("drive.pivot_enter_deg", 100, 0, 180);   // 코너가 이보다 크게 꺾이면 피봇
    declare_int("drive.pivot_keep_deg", 20, 0, 180);    // 이 각도까지는 pure pursuit에 맡기고 초과분만 피봇
    declare_int("drive.pivot_confirm", 3, 1, 30);       // 코너가 이만큼 연속으로 보여야 피봇 (제어 주기 수)
    declare_int("drive.pivot_exit_deg", 15, 0, 90);
    declare_int("drive.pivot_w_deg_s", 90, 0, 360);
    declare_int("drive.pivot_fallback_deg", 75, 0, 180); // 코너를 못 찾았는데 목표점이 이보다 옆이면 바로 피봇
    declare_int("drive.corner_min_x_mm", 200, 0, 1000);  // 이보다 가까운 경로에서는 코너를 찾지 않음 (BEV 아랫줄 거리)

    declare_bool("drive.psd.enable", false);
    declare_int("drive.psd.stop_mm", 150, 0, 1000);
    // /mission/pause (차단바·주차) 정지 방식: 0 감속 제한으로 부드럽게 (예전), 1 급정거 (받는 즉시 모터 0)
    declare_int("drive.pause_hard_stop", 0, 0, 1);
    declare_parameter("drive.psd.topics", std::vector<std::string>{"/psd/front"}); // 시작할 때만 읽음

    // ====== 모터 출력 (tb_uart_node: TB_Uart_RX "velocity L R", 다이나믹셀 원시값)
    // 하드웨어 값이라 GUI 대신 yaml에서 정함
    declare_bool("drive.uart.enable", true);
    declare_parameter("drive.uart.wheel_radius_m", 0.0475);
    declare_parameter("drive.uart.wheel_sep_m", 0.1933); // 전진하며 돌 때(두 바퀴 같은 방향) 실측 간격
    declare_parameter("drive.uart.pivot_sep_m", 0.1933); // 제자리 회전 때 실측 간격 (바퀴가 미끄러져 더 작게 나옴)
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
        "/lane/path_source", 10, [this](const std_msgs::msg::String::ConstSharedPtr m)
        {
            const bool weak = m->data == "memory" || m->data == "color_rule";
            if (weak && !weak_src_)
                weak_since_ = now(); // 약한 경로가 시작된 시각 (drive.weak_delay_ms)
            weak_src_ = weak;
            path_src_ = m->data; });
    one_line_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/lane/one_line", 10, [this](const std_msgs::msg::Bool::ConstSharedPtr m)
        {
            if (m->data && !one_line_)
                one_line_since_ = now();
            one_line_ = m->data; });
    con_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/lane/construction", 10, [this](const std_msgs::msg::Bool::ConstSharedPtr m)
        {
            if (m->data == construction_)
                return;
            construction_ = m->data;
            apply_profile();
            RCLCPP_INFO(get_logger(), "construction profile: %s", construction_ ? "ON" : "off"); });

    turn_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "/mission/turn", 10, [this](const std_msgs::msg::UInt8::ConstSharedPtr m)
        {
            const bool t = m->data != 0;
            if (t == turning_)
                return;
            turning_ = t;
            apply_profile();
            RCLCPP_INFO(get_logger(), "sharp profile: %s", turning_ ? "intersection" : "curve"); });

    pause_sub_ = create_subscription<std_msgs::msg::Bool>(          //pause 토픽 추가
        "/mission/pause", 10, [this](const std_msgs::msg::Bool::ConstSharedPtr m)
        {
            if (m->data == paused_)
                return;
            paused_ = m->data;
            RCLCPP_INFO(get_logger(), "mission pause: %s%s", paused_ ? "ON" : "off",
                        paused_ && pause_hard_ ? " (급정거)" : "");
            if (paused_ && pause_hard_ && enable_)
            {
                // 다음 제어 주기(최대 33ms)를 안 기다리고 바로 0
                lim_.reset();
                pivot_.reset();
                publish_uart(0.0, 0.0, now().seconds(), true);
            } });

    cmd_sub_ = create_subscription<DriveCmd>(
        "/drive/cmd", 10, [this](const DriveCmd::ConstSharedPtr m)
        {
            if (!cmd_ || cmd_->mode != m->mode)
                RCLCPP_INFO(get_logger(), "drive cmd: %s", MODE_NAME[std::min<uint8_t>(m->mode, 2)]);
            cmd_ = m;
            cmd_time_ = now(); });

    // 전체 초기화 (GUI 초기화 버튼): 추종·피봇·가감속 상태, 받은 명령·미션 값을 버림 (enable·파라미터는 그대로)
    reset_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/mission/reset", 10, [this](const std_msgs::msg::Empty::ConstSharedPtr)
        {
            lim_.reset();
            follower_.reset();
            pivot_.reset();
            cmd_.reset();
            paused_ = false;
            turning_ = false;
            construction_ = false;
            apply_profile();
            RCLCPP_INFO(get_logger(), "reset: 추종 상태·명령·미션 값 초기화"); });

    // as_string_array()는 임시 Parameter 안을 참조로 돌려주므로 먼저 복사해야 함 (안 하면 dangling)
    const std::vector<std::string> psd_topics = get_parameter("drive.psd.topics").as_string_array();
    for (const auto &topic : psd_topics)
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

void DriveNode::apply_profile()
{
    // 우선순위: 공사 구간 > 교차로 명령 > 일반
    const bool cross = turning_ && !construction_;
    // 공사·교차로 값이 0이면 일반 값 (ld_min·ld_sharp·turn_gain은 예전 규칙 그대로)
    auto pick = [&](double normal, double cr, double co)
    { return construction_ ? (co > 0.0 ? co : normal) : (cross && cr > 0.0) ? cr : normal; };
    follower_.p.ld_min = construction_ ? ld_min_con_ : (cross && ld_min_cross_ > 0.0) ? ld_min_cross_ : ld_min_normal_;
    follower_.p.ld_sharp = construction_ ? ld_sharp_con_ : ld_sharp_normal_;
    follower_.p.turn_gain = construction_ ? turn_gain_con_ : cross ? turn_gain_cross_ : turn_gain_normal_;
    // 매번 일반 값에서 다시 계산 (예전에는 ld_max가 한 번 커지면 일반 주행으로 돌아와도 안 줄었음)
    follower_.p.ld_max = std::max(pick(ld_max_normal_, ld_max_cross_, ld_max_con_), follower_.p.ld_min);
    follower_.p.v_min = pick(v_min_normal_, v_min_cross_, v_min_con_);
    follower_.p.v_curve = pick(v_curve_normal_, v_curve_cross_, v_curve_con_);
    pivot_.p = construction_ ? pv_con_ : pv_normal_;
    // 교차로 명령 중에는 예전처럼 0°부터 비례, 아니면 일반 곡선에서는 lookahead를 줄이지 않음
    follower_.p.sharp_start = turning_ ? 0.0 : sharp_curve_start_;
    follower_.p.sharp_turn = turning_ ? sharp_turn_cross_ : std::max(sharp_curve_full_, sharp_curve_start_ + 1e-3);
    // 거리 보정은 일반 주행에서만 (교차로·공사 구간은 지금 맞춰진 값 그대로)
    follower_.p.x_offset = (turning_ || construction_) ? 0.0 : curve_x_offset_;
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
    v_min_normal_ = mm("drive.v_min_mm_s");
    v_curve_normal_ = mm("drive.v_curve_mm_s");
    v_min_cross_ = mm("drive.cross_v_min_mm_s");
    v_curve_cross_ = mm("drive.cross_v_curve_mm_s");
    v_min_con_ = mm("drive.con_v_min_mm_s");
    v_curve_con_ = mm("drive.con_v_curve_mm_s");
    p.curve_start = deg("drive.curve_start_deg");
    p.curve_full = std::max(deg("drive.curve_full_deg"), p.curve_start + 1e-3);
    p.w_max = deg("drive.w_max_deg_s");
    p.accel = mm("drive.accel_mm_s2");
    p.decel = mm("drive.decel_mm_s2");
    p.w_accel = deg("drive.w_accel_deg_s2");
    ld_min_normal_ = mm("drive.ld_min_mm");
    ld_min_con_ = mm("drive.con_ld_min_mm");
    ld_max_normal_ = std::max(ld_min_normal_, mm("drive.ld_max_mm"));
    ld_max_cross_ = mm("drive.cross_ld_max_mm");
    ld_max_con_ = mm("drive.con_ld_max_mm");
    p.ld_time = mm("drive.ld_time_ms"); // ms → s 도 /1000
    ld_sharp_normal_ = mm("drive.ld_sharp_mm");
    ld_sharp_con_ = mm("drive.con_ld_sharp_mm");
    turn_gain_con_ = get_parameter("drive.con_turn_gain_pct").as_int() / 100.0;
    turn_gain_normal_ = get_parameter("drive.turn_gain_pct").as_int() / 100.0;
    turn_gain_cross_ = get_parameter("drive.cross_turn_gain_pct").as_int() / 100.0;
    ld_min_cross_ = mm("drive.cross_ld_min_mm");
    cross_v_ = mm("drive.cross_v_mm_s");
    sharp_turn_cross_ = deg("drive.sharp_turn_deg");
    sharp_curve_start_ = deg("drive.sharp_curve_start_deg");
    sharp_curve_full_ = deg("drive.sharp_curve_full_deg");
    curve_x_offset_ = mm("drive.curve_x_offset_mm");
    p.sharp_hold = mm("drive.sharp_hold_ms");
    p.sharp_hold_dist = mm("drive.sharp_hold_mm");
    p.curv_slow = mm("drive.curv_slow_mm");
    p.stop_dist = mm("drive.stop_dist_mm");
    p.corner_min_x = mm("drive.corner_min_x_mm");

    follower_.path_timeout = mm("drive.path_timeout_ms");
    follower_.hold_time = mm("drive.hold_ms");
    cmd_timeout_ = mm("drive.cmd_timeout_ms");
    weak_scale_ = get_parameter("drive.weak_src_pct").as_int() / 100.0;
    weak_delay_ = mm("drive.weak_delay_ms"); // ms → s
    one_line_v_ = mm("drive.one_line_v_mm_s");
    one_line_delay_ = mm("drive.one_line_delay_ms");
    con_v_ = mm("drive.con_v_mm_s");

    auto &pv = pv_normal_;
    pv.enter = deg("drive.pivot_enter_deg");
    pv.keep = std::min(deg("drive.pivot_keep_deg"), pv.enter);
    pv.confirm = get_parameter("drive.pivot_confirm").as_int();
    pv.exit = std::min(deg("drive.pivot_exit_deg"), pv.keep);
    pv.w = deg("drive.pivot_w_deg_s");
    pv.fallback = deg("drive.pivot_fallback_deg");
    pv_con_ = pv; // 나머지는 같고 시작 각도·맡기는 각도만
    pv_con_.enter = deg("drive.con_pivot_enter_deg");
    pv_con_.keep = std::min(deg("drive.con_pivot_keep_deg"), pv_con_.enter);
    pv_con_.exit = std::min(pv_con_.exit, pv_con_.keep);
    apply_profile();

    psd_enable_ = get_parameter("drive.psd.enable").as_bool();
    psd_stop_ = mm("drive.psd.stop_mm");
    pause_hard_ = get_parameter("drive.pause_hard_stop").as_int() != 0;

    uart_enable_ = get_parameter("drive.uart.enable").as_bool();
    wheel_r_ = std::max(1e-3, get_parameter("drive.uart.wheel_radius_m").as_double());
    wheel_sep_ = get_parameter("drive.uart.wheel_sep_m").as_double();
    pivot_sep_ = get_parameter("drive.uart.pivot_sep_m").as_double();
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
        pivot_.reset(); // 꺼졌다 켜질 때 회전 상태가 남지 않게
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
            // 약한 경로(memory 등)가 weak_delay_ms 넘게 이어질 때만 감속 (live↔memory가 한두 프레임씩
            // 번갈아 나오면 감속·가속이 반복돼 출렁임. 기억 경로는 odom 배율을 맞춘 뒤로 짧게는 정확함)
            if (weak_src_ && (weak_delay_ <= 0.0 || (now_t - weak_since_).seconds() >= weak_delay_))
                limit *= weak_scale_;
            // 한쪽 선만 오래 보임: 다음 꺾임을 BEV에서 늦게 보므로 감속 (녹화 1029 07:08 S자: 3초 동안 노란선만)
            if (one_line_v_ > 0.0 && one_line_ && !construction_ && (now_t - one_line_since_).seconds() >= one_line_delay_)
                limit = std::min(limit, one_line_v_);
            if (construction_ && con_v_ > 0.0)
                limit = std::min(limit, con_v_);
            else if (turning_ && cross_v_ > 0.0) // 교차로 명령 중 (공사 구간이면 공사 속도)
                limit = std::min(limit, cross_v_);

            o = follower_.follow(t, lim_.v, limit);
            vd = o.v;
            wd = o.w;


            // 날카로운 꺾임에서만 제자리 회전 (자세한 규칙은 Pivoter)
            pivot_.step(o, lim_.v, lim_.w, dt, std::max(follower_.p.v_min, 0.5 * follower_.p.v_max), vd, wd, why);
        }
        if (mode != DriveCmd::FOLLOW)
            pivot_.reset();

            
        // ====== 일시정지 (/mission/pause): 목표 속도를 0으로. 이후 lim_.step의 감속 제한으로 부드럽게 멈춤
        if (paused_)
        {
            vd = 0.0;
            wd = 0.0;
            pivot_.reset(); // 정지 중에 제자리 회전이 시작되지 않게
            if (!*why)
                why = "pause";
        }

        // ====== 가감속 제한 → PSD (전진만 막고 즉시 정지)
        lim_.step(vd, wd, dt, follower_.p);
        
        // FOLLOW에서는 가감속 제한 뒤에도 곡률을 유지.
        // 감속이 덜 된 상태에서 w만 작게 나가면 원하는 것보다 크게 돌아 바깥으로 밀림
        if (mode == DriveCmd::FOLLOW && pivot_.phase() == tb_drive::Pivoter::Phase::Follow && o.tg.ok && o.v > 1e-3)
        {
            const double k = o.w / o.v; // 원하는 곡률 (w_max로 깎였으면 그 값)
            double w = lim_.v * k;
            if (std::abs(w) > follower_.p.w_max)
            {
                w = std::copysign(follower_.p.w_max, w);
                lim_.v = follower_.p.w_max / std::abs(k); // 각속도가 모자라면 속도를 바로 낮춤
            }
            lim_.w = w;
        }

        // 급정거: pause 동안은 감속 제한 없이 0
        if (paused_ && pause_hard_)
            lim_.reset();

        if (lim_.v > 0.0 && psd_blocked(now_t))
        {
            lim_.v = 0.0;
            why = "psd";
        }
    }

    publish_cmd(lim_.v, lim_.w);
    follower_.record(t, lim_.v, lim_.w);
    // /mission/pause로 완전히 멈춘 뒤에는 0을 한 번 보내고 조용히 (그동안 tb_brain 주차가 모터를 직접 돌림.
    // 계속 0을 보내면 brain의 "velocity L R"을 0.2초마다 덮어써서 주차 동작이 안 됨)
    const bool stopped = std::abs(lim_.v) < 1e-4 && std::abs(lim_.w) < 1e-4;
    if (enable_ && paused_ && stopped)
    {
        if (!pause_quiet_)
        {
            publish_uart(0.0, 0.0, t, true);
            pause_quiet_ = true;
            RCLCPP_INFO(get_logger(), "pause: 정지 완료 → UART 보내지 않음 (brain이 모터 사용 가능)");
        }
    }
    else if (enable_)
    {
        pause_quiet_ = false;
        publish_uart(lim_.v, lim_.w, t, false);
    }
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
        // age: 지금 모터 명령을 만든 경로가 카메라에서 찍힌 지 몇 ms 됐는지 (영상 → 모터 명령 지연)
        const double ps = follower_.path_stamp();
        const int age_ms = ps >= 0.0 ? static_cast<int>(std::lround((t - ps) * 1000.0)) : -1;
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s src=%s v=%.3f w=%.2f ld=%.2f end=%.2f cap=%.3f dev=%.0f age=%d prof=%s%s%s%s",
                      mode_txt.c_str(), path_src_.c_str(), lim_.v, lim_.w, o.tg.ld, o.tg.end_dist,
                      o.v_cap, o.dev * 180.0 / M_PI, age_ms,
                      construction_ ? "con" : turning_ ? "cross" : "curve",
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
    // 바퀴 간격: 두 바퀴가 같은 방향으로 구르면(원호) wheel_sep, 안쪽 바퀴가 거꾸로 돌면
    // 제자리 회전에 가까워질수록 바닥을 비비며 미끄러져 덜 돌므로 pivot_sep 쪽으로 섞음
    double sep = wheel_sep_;
    const double half_turn = std::abs(w) * wheel_sep_ / 2.0;
    if (half_turn > std::abs(v))
        sep = pivot_sep_ + (wheel_sep_ - pivot_sep_) * (std::abs(v) / half_turn); // 0 = 제자리, 1 = 안쪽 바퀴 정지
    double l = (v - w * sep / 2.0) * to_raw;
    double r = (v + w * sep / 2.0) * to_raw;
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
