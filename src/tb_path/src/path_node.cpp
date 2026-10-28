#include "tb_path/path_node.hpp"
#include "tb_path/obstacles.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
// tb_line 라벨 값
constexpr uint8_t LABEL_WHITE = 1;
constexpr uint8_t LABEL_YELLOW = 2;
constexpr uint8_t LABEL_RED = 250;
constexpr uint8_t LABEL_BLIND = 255;
} // namespace

PathNode::PathNode() : Node("path_node")
{
    // ====== 입력
    mask_sub_ = create_subscription<tb_interfaces::msg::LaneMask>(
        "/lane/mask", rclcpp::QoS(1),
        std::bind(&PathNode::mask_callback, this, std::placeholders::_1));

    det_sub_ = create_subscription<tb_interfaces::msg::DetectionArray>(
        "/signs/detections", rclcpp::QoS(1),
        [this](const tb_interfaces::msg::DetectionArray::ConstSharedPtr m) { last_det_ = m; });

    turn_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "/mission/turn", 10, [this](const std_msgs::msg::UInt8::ConstSharedPtr m)
        {
            if (m->data != turn_)
                RCLCPP_INFO(get_logger(), "turn: %u", m->data);
            turn_ = m->data; });

    parking_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/mission/parking", 10, [this](const std_msgs::msg::Bool::ConstSharedPtr m)
        {
            if (m->data != parking_)
                RCLCPP_INFO(get_logger(), "parking: %d", m->data);
            parking_ = m->data; });

    white_dash_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/mission/white_dash", 10, [this](const std_msgs::msg::Bool::ConstSharedPtr m)
        {
        if (m->data != white_dash_)
            RCLCPP_INFO(get_logger(), "white_dash: %d", m->data);
        white_dash_ = m->data; });

    // 정지선 모드 (tb_brain이 빨간불 등에서 켬). 켜져 있는 동안 빨간 가로선 앞에서 경로를 자름
    // true → false가 오면 끄고 기억한 정지선도 지움. 켤 때도 새로 찾음
    stop_line_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/mission/stop_line", 10, [this](const std_msgs::msg::Bool::ConstSharedPtr m)
        {
            if (m->data == stop_line_)
                return;
            RCLCPP_INFO(get_logger(), "stop_line: %s", m->data ? "켬" : "끔");
            stop_line_ = m->data;
            sl_have_ = false; });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](const nav_msgs::msg::Odometry::ConstSharedPtr m)
        {
            const auto &q = m->pose.pose.orientation;
            pose_.x = m->pose.pose.position.x;
            pose_.y = m->pose.pose.position.y;
            pose_.th = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            last_odom_time_ = now(); });

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::ConstSharedPtr m)
        {
            last_cmd_ = *m;
            last_cmd_time_ = now(); });

    // 공사 구간 진입/끝 명령. true = 진입 (tb_brain이 표지판을 보고, 또는 GUI C키), false = 끝
    // 표지판은 path_node가 직접 보지 않음 (판단은 tb_brain 한 곳에서)
    con_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/mission/construction", 10, [this](const std_msgs::msg::Bool::ConstSharedPtr m)
        {
            RCLCPP_INFO(get_logger(), "construction 명령: %s", m->data ? "진입" : "끝");
            con_cmd_ = m->data ? ConCmd::Start : ConCmd::Stop; });

    // 지금 공사 모드를 바로 끝냄 (GUI X키). true 한 번이면 됨. /mission/construction false와 같음
    con_exit_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/mission/construction_exit", 10, [this](const std_msgs::msg::Bool::ConstSharedPtr m)
        {
            if (!m->data)
                return;
            con_cmd_ = ConCmd::Exit;
            RCLCPP_INFO(get_logger(), "construction: 끝내기 요청"); });

    // 전체 초기화 (GUI 초기화 버튼, tb_brain·tb_drive도 같은 토픽을 받음)
    reset_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/mission/reset", 10, [this](const std_msgs::msg::Empty::ConstSharedPtr) { reset_all(); });

    // ====== 출력
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/lane/path", 10);
    src_pub_ = create_publisher<std_msgs::msg::String>("/lane/path_source", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/lane/debug/skel/compressed", rclcpp::SensorDataQoS().keep_last(1));
    con_pub_ = create_publisher<std_msgs::msg::Bool>("/lane/construction", 10);
    world_pub_ = create_publisher<tb_interfaces::msg::PathWorld>("/lane/world", rclcpp::SensorDataQoS().keep_last(1));
    con_info_pub_ = create_publisher<std_msgs::msg::String>("/lane/construction_info", 10);

    // ====== 파라미터
    declare_int("skel.seal_len", 30, 100);
    declare_int("skel.prune_len", 20, 100);
    declare_int("skel.dash_len", 40, 160);
    declare_int("skel.wdash_max_mm", 150, 500);   // 흰 점선 잇기: 이보다 짧은 흰 조각을 점선으로 봄 (0이면 끔)
    declare_int("skel.wdash_gap_mm", 150, 500);   // 조각 끝 사이가 이 안이면 이음
    declare_int("skel.wdash_angle_deg", 30, 90);  // 두 조각·잇는 선의 방향 차이 허용
    declare_int("skel.wdash_offset_mm", 40, 200); // 한 조각 중심이 다른 조각 축에서 옆으로 벗어나도 되는 거리
    declare_int("path.brick_depth_mm", 60, 500);
    declare_int("path.brick_margin_mm", 30, 300);
    declare_int("path.brick_always", 0, 1);       // 1: 공사 모드가 아니어도 벽돌을 벽으로 (예전 동작), 0: 공사 모드에서만
    declare_int("path.brick_gap_mm", 120, 300);   // 벽돌 끝과 차선 사이 이보다 좁은 틈은 막음 (0이면 안 막음)
    declare_int("path.det_max_age_ms", 300, 2000);
    declare_int("path.mem_travel_mm", 600, 3000);
    declare_int("skel.max_lane_thick", 20, 200);
    declare_int("path.max_dev_mm", 50, 500);
    declare_int("path.override_frames", 10, 100);
    declare_int("path.turn_drop_memory", 1, 1);       // 1: 좌/우 명령이 새로 들어오면 직진 때 기억한 경로를 버림
    declare_int("path.min_len_mm", 100, 1000);
    declare_int("path.branch_look_mm", 40, 300);
    declare_int("path.blind_max_mm", 250, 1000);
    declare_int("path.fill_blind", 1, 2);          // 사각지대를 지나온 경로로 채움: 0 끔, 1 직진 명령일 때만, 2 항상
    declare_int("path.fill_join_mm", 40, 200);     // 기억과 실시간 경로가 이보다 떨어져 있으면 안 채움
    declare_int("path.turn_radius_mm", 150, 500);  // 급한 모서리(교차로 ㄱ자)를 이 반지름 원호로 (0이면 끔). 피봇 대신
    declare_int("path.turn_min_deg", 45, 180);     // 앞뒤 6cm 사이에서 이보다 크게 꺾이는 곳만 둥글게
    // 공사 구간 (실측: 차폭 56cm, 벽돌 20x8cm 선에 붙어 좌우 번갈아, 빈 공간 40cm)
    declare_int("path.con_brick_depth_mm", 200, 500);  // 공사 구간 벽돌 벽 앞뒤 길이 (일반: brick_depth_mm)
    declare_int("path.con_brick_margin_mm", 90, 300);  // 공사 구간 벽돌 벽 좌우 여유 (일반: brick_margin_mm)
    declare_int("path.con_brick_w_mm", 200, 600);      // 벽돌 실제 가로. 선에 닿은 벽돌은 선에서 이만큼(+여유)만 벽
    declare_int("path.con_attach_mm", 300, 600);       // 선에 붙은 벽돌은 붙은 쪽을 이 거리 안의 선까지 막음
    declare_int("path.con_extend_mm", 400, 1000);      // 벽돌에 가려 끊긴 선을 이만큼 이음 (0이면 끔)
    declare_int("path.con_edge_mm", 60, 300);
    declare_int("path.con_dead_end_mm", 250, 1000);    // 끝이 벽에 막힌 가지가 이보다 짧으면 삭제 (0이면 끔)
    declare_int("path.con_bridge_mm", 600, 1500);     // 같은 쪽 벽돌(앞 벽돌 뒷면~다음 앞면)이 이 안이면 사이를 막음 (0 끔)
    declare_int("path.con_bridge_margin_mm", 50, 300); // 이은 벽(보라)을 벽돌 안쪽 끝에서 길 쪽으로 더 막는 폭
    declare_int("path.con_bridge_dy_mm", 0, 500);      // 두 벽돌 안쪽 끝 y 차이가 이보다 크면 안 이음 (0 = 안 봄. 쪽은 붙은 선으로 정하므로 보통 0)
    declare_int("path.con_bridge_from_back", 0, 1);    // 1: 앞 벽돌 뒷면부터 막음 (0: 앞면부터 = 벽돌 옆도 겹쳐 막음)
    declare_int("path.con_keep", 0, 2);                // 지난 벽돌 피하기: 0 끔, 1 그쪽으로 꺾지만 못하게(약), 2 벽돌에서 떼어 놓기(강)
    declare_int("path.con_hold_mm", 40, 500);
    declare_int("path.con_keep_rear_mm", 100, 300);    // 지난 벽돌: 바퀴 축이 뒷면을 지나고 이만큼 더 가야 그쪽으로 꺾음 (로봇 뒤 끝 14cm)          // 기억한 벽돌 뒷면을 로봇 뒤 끝이 지나고 이만큼 더 가야 그쪽으로 붙음
    declare_int("path.con_clear_mm", 30, 200);         // 기억한 벽돌과 로봇 옆면 사이 여유          // 선에 안 닿은 벽돌 끝이 화면 끝에서 이 안이면 화면 끝까지 막음
    declare_int("path.con_exit_mm", 600, 3000);
    declare_int("path.con_smooth_mm", 80, 300);        // 공사 구간 경로를 이 폭으로 평균내 모서리를 둥글게 (0이면 끔)        // 벽돌이 이만큼 가는 동안 안 보이면 공사 구간 끝
    declare_int("path.con_no_brick_mm", 2000, 10000); // 진입 후 벽돌을 한 번도 못 보고 이만큼 가면 끝 (0이면 끔)
    declare_int("path.con_override_frames", 1, 100);  // 공사 구간: 새 경로가 기억과 달라도 이만큼만 의심 (벽돌이 갑자기 보이면 경로가 크게 바뀌는 게 정상)
    declare_int("path.con_offscreen_side", 0, 1);     // 공사 구간: 화면 옆 끝까지 간 벽돌은 화면 밖 선에 붙은 것 (반대쪽 선 틈은 막지 않음)
    declare_int("path.con_cross_override_frames", 15, 100); // 공사 구간: 새 경로가 기억과 차선 너머에 있으면 이만큼 연속으로 보여야 믿음 (0 끔)
    declare_int("path.con_straight_by_lane", 1, 1);   // 공사 구간: 직진 갈림길에서 차선 방향 기준으로 가장 곧은 가지 (화면 밖 선에 붙은 벽돌 앞으로 새지 않게)
    // 정지선 (/mission/stop_line true 동안만)
    declare_int("path.sl_margin_mm", 60, 500);  // 경로 끝 = 정지선 - 이만큼 (바퀴 축 기준. 로봇 앞 9cm + tb_drive stop_dist도 생각)
    declare_int("path.sl_min_w_px", 60, 320);   // BEV 한 줄에 빨강이 이만큼 있어야 정지선 (잡티·옆 빨간 것 거르기)

    reload_params();
    skel_p_.max_lane_thick = get_parameter("skel.max_lane_thick").as_int();
    auto &tp = tracker_.p;
    tp.max_dev_m = get_parameter("path.max_dev_mm").as_int() / 1000.0;
    tp.override_frames = override_frames_ = get_parameter("path.override_frames").as_int();
    tp.min_len_m = get_parameter("path.min_len_mm").as_int() / 1000.0;
    tp.lookahead_m = get_parameter("path.branch_look_mm").as_int() / 1000.0;
    tp.blind_max_m = get_parameter("path.blind_max_mm").as_int() / 1000.0;
    tp.fill_blind = get_parameter("path.fill_blind").as_int();
    tp.fill_join_m = get_parameter("path.fill_join_mm").as_int() / 1000.0;
    turn_radius_mm_ = get_parameter("path.turn_radius_mm").as_int();
    turn_min_deg_ = get_parameter("path.turn_min_deg").as_int();
    post_cb_handle_ = add_post_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &) { reload_params(); });

    RCLCPP_INFO(get_logger(), "path_node started");
}

// 탐색 상태를 처음처럼: 기억 경로·시드, 위치, 미션 입력(turn·parking·stop_line), 공사 모드
// (파라미터는 그대로. 미션 값은 brain·GUI가 다시 보내면 그 값으로)
void PathNode::reset_all()
{
    tracker_.reset();
    last_seed_ = {-1, -1};
    pose_ = lane_path::Pose2D{};
    turn_ = last_turn_ = 0;
    parking_ = false;
    white_dash_ = false;
    stop_line_ = false;
    sl_have_ = false;
    last_det_.reset();
    con_cmd_ = ConCmd::None;
    if (con_active_)
        end_construction("초기화");
    con_brick_seen_ = false;
    con_hold_[0].on = con_hold_[1].on = false;
    con_near_.clear();
    con_have_pose_ = false;
    con_reason_ = "초기화";
    RCLCPP_INFO(get_logger(), "reset: 경로 기억·미션 상태 초기화");
}

void PathNode::declare_int(const std::string &name, int def, int max)
{
    rcl_interfaces::msg::ParameterDescriptor d;
    rcl_interfaces::msg::IntegerRange r;
    r.from_value = 0;
    r.to_value = max;
    r.step = 1;
    d.integer_range.push_back(r);
    declare_parameter(name, def, d);
}

void PathNode::reload_params()
{
    skel_p_.seal_len = get_parameter("skel.seal_len").as_int();
    skel_p_.prune_len = get_parameter("skel.prune_len").as_int();
    dash_len_ = get_parameter("skel.dash_len").as_int();
    wdash_max_mm_ = get_parameter("skel.wdash_max_mm").as_int();
    wdash_gap_mm_ = get_parameter("skel.wdash_gap_mm").as_int();
    wdash_angle_deg_ = get_parameter("skel.wdash_angle_deg").as_int();
    wdash_offset_mm_ = get_parameter("skel.wdash_offset_mm").as_int();
    brick_depth_mm_ = get_parameter("path.brick_depth_mm").as_int();
    brick_margin_mm_ = get_parameter("path.brick_margin_mm").as_int();
    brick_gap_mm_ = get_parameter("path.brick_gap_mm").as_int();
    brick_always_ = get_parameter("path.brick_always").as_int() != 0;
    det_max_age_ms_ = get_parameter("path.det_max_age_ms").as_int();
    tracker_.p.mem_travel_m = get_parameter("path.mem_travel_mm").as_int() / 1000.0;

    // 선언만 되어 있고 읽지 않던 값들 (yaml·GUI에서 바꿔도 반영이 안 됐음)
    skel_p_.max_lane_thick = get_parameter("skel.max_lane_thick").as_int();
    auto &tp = tracker_.p;
    tp.max_dev_m = get_parameter("path.max_dev_mm").as_int() / 1000.0;
    tp.override_frames = override_frames_ = get_parameter("path.override_frames").as_int();
    turn_drop_memory_ = get_parameter("path.turn_drop_memory").as_int() != 0;
    tp.min_len_m = get_parameter("path.min_len_mm").as_int() / 1000.0;
    tp.lookahead_m = get_parameter("path.branch_look_mm").as_int() / 1000.0;
    tp.blind_max_m = get_parameter("path.blind_max_mm").as_int() / 1000.0;
    tp.fill_blind = get_parameter("path.fill_blind").as_int();
    tp.fill_join_m = get_parameter("path.fill_join_mm").as_int() / 1000.0;
    turn_radius_mm_ = get_parameter("path.turn_radius_mm").as_int();
    turn_min_deg_ = get_parameter("path.turn_min_deg").as_int();
    con_depth_mm_ = get_parameter("path.con_brick_depth_mm").as_int();
    con_margin_mm_ = get_parameter("path.con_brick_margin_mm").as_int();
    con_edge_mm_ = get_parameter("path.con_edge_mm").as_int();
    con_attach_mm_ = get_parameter("path.con_attach_mm").as_int();
    con_extend_mm_ = get_parameter("path.con_extend_mm").as_int();
    con_hold_mm_ = get_parameter("path.con_hold_mm").as_int();
    con_keep_ = get_parameter("path.con_keep").as_int();
    con_bridge_mm_ = int(get_parameter("path.con_bridge_mm").as_int());
    con_bridge_margin_mm_ = int(get_parameter("path.con_bridge_margin_mm").as_int());
    con_bridge_dy_mm_ = int(get_parameter("path.con_bridge_dy_mm").as_int());
    con_bridge_from_back_ = get_parameter("path.con_bridge_from_back").as_int() != 0;
    con_keep_rear_mm_ = get_parameter("path.con_keep_rear_mm").as_int();
    con_dead_end_mm_ = get_parameter("path.con_dead_end_mm").as_int();
    con_clear_mm_ = get_parameter("path.con_clear_mm").as_int();
    con_brick_w_mm_ = get_parameter("path.con_brick_w_mm").as_int();
    con_exit_mm_ = get_parameter("path.con_exit_mm").as_int();
    con_smooth_mm_ = get_parameter("path.con_smooth_mm").as_int();
    con_no_brick_mm_ = int(get_parameter("path.con_no_brick_mm").as_int());
    con_override_frames_ = int(get_parameter("path.con_override_frames").as_int());
    con_cross_override_frames_ = int(get_parameter("path.con_cross_override_frames").as_int());
    con_straight_by_lane_ = get_parameter("path.con_straight_by_lane").as_int() != 0;
    con_offscreen_side_ = get_parameter("path.con_offscreen_side").as_int() != 0;
    sl_margin_mm_ = int(get_parameter("path.sl_margin_mm").as_int());
    sl_min_w_px_ = int(get_parameter("path.sl_min_w_px").as_int());
}

// BEV에서 로봇에 가장 가까운 빨간 가로선 → 로봇 기준 거리 [m]. 없으면 false
// 아래 행부터 올라가며 빨강이 min_w 이상인 첫 행 (그 행 빨강 가운데 열 기준)
static bool find_stop_line(const cv::Mat &red, int min_w, const lane_path::Geo &g, double &x_m)
{
    for (int y = red.rows - 1; y >= 0; --y)
    {
        const uchar *r = red.ptr<uchar>(y);
        int n = 0;
        long sx = 0;
        for (int x = 0; x < red.cols; ++x)
            if (r[x])
                ++n, sx += x;
        if (n >= std::max(1, min_w))
        {
            x_m = lane_path::bev_to_robot({double(sx) / n, double(y)}, g).x;
            return true;
        }
    }
    return false;
}

// 로봇 앞 통로(차선 폭) 안의 각 열에서 처음 만나는 차선 행 → 대부분 열이 막혀 있고 높이가 비슷하면 벽
// 반환: 벽까지 거리 [m], 없으면 -1. angle: 벽 기울기 (피봇 각도 보정용)
double front_wall(const cv::Mat &wall, const cv::Mat &valid, const lane_path::Geo &g, double &angle)
{
    const int cx = g.bev_size / 2, half = g.lane_w_px / 2 - 6;
    std::vector<cv::Point2f> hits;
    int cols = 0;
    for (int x = cx - half; x <= cx + half; x += 2)
    {
        ++cols;
        for (int y = wall.rows - 1; y >= 0; --y)
            if (valid.at<uchar>(y, x) && wall.at<uchar>(y, x))
            {
                hits.emplace_back(x, y);
                break;
            }
    }
    if (hits.size() < 0.7 * cols)
        return -1; // 앞이 뚫려 있음
    cv::Vec4f L;
    cv::fitLine(hits, L, cv::DIST_HUBER, 0, 0.01, 0.01);
    angle = std::atan2(-L[1], L[0]); // 수평 기준 기울기
    if (std::abs(angle) > 0.5)
        return -1; // 30° 넘게 기울면 벽 아님 (커브)
    const double ymid = L[3] + (cx - L[2]) * L[1] / L[0];
    return lane_path::bev_to_robot({double(cx), ymid}, g).x;
}

// /odom 이 0.5초 안에 왔으면 그대로, 아니면 /cmd_vel 적분
void PathNode::update_pose(const rclcpp::Time &t)
{
    MotionSrc src = MotionSrc::None;
    if (last_odom_time_.nanoseconds() > 0 && (t - last_odom_time_).seconds() < 0.5)
        src = MotionSrc::Odom;
    else if (last_cmd_time_.nanoseconds() > 0 && (t - last_cmd_time_).seconds() < 0.5)
        src = MotionSrc::CmdVel;

    if (src != motion_src_)
    {
        tracker_.reset(); // 기준 좌표계가 바뀌면 기억한 경로는 무효
        static const char *names[] = {"none", "odom", "cmd_vel"};
        RCLCPP_INFO(get_logger(), "motion source: %s", names[static_cast<int>(src)]);
        motion_src_ = src;
    }

    if (src == MotionSrc::CmdVel && last_integ_time_.nanoseconds() > 0)
    {
        const double dt = std::min((t - last_integ_time_).seconds(), 0.2);
        const double v = last_cmd_.linear.x, w = last_cmd_.angular.z;
        pose_.x += v * dt * std::cos(pose_.th + 0.5 * w * dt);
        pose_.y += v * dt * std::sin(pose_.th + 0.5 * w * dt);
        pose_.th += w * dt;
    }
    last_integ_time_ = t;
}

// 벽돌 박스 → 로봇 좌표 앞면 x, 좌우 끝 y, 어느 쪽인지 (공사 구간 기억·잇기용)
// 박스 아랫변이 화면 위로 벗어났거나 영상 아래에서 잘렸으면(바닥 위치가 틀림), 로봇 정면에 걸쳐 쪽을 모르면 false
bool PathNode::brick_robot(const obstacles::Box &b, const cv::Mat &H, const cv::Mat &lines, int rows, int side_hint, BrickR &out) const
{
    std::vector<cv::Point2f> src{{float(b.x), float(b.y + b.h)}, {float(b.x + b.w), float(b.y + b.h)}}, dst;
    cv::perspectiveTransform(src, dst, H);
    if (dst[0].y < 0 || dst[1].y < 0 || dst[0].y >= rows - 2 || dst[1].y >= rows - 2)
        return false;
    const cv::Point2d A = lane_path::bev_to_robot({dst[0].x, dst[0].y}, geo_);
    const cv::Point2d C = lane_path::bev_to_robot({dst[1].x, dst[1].y}, geo_);
    const double x0 = std::min(A.x, C.x);
    double yl = std::max(A.y, C.y), yr = std::min(A.y, C.y);
    // 쪽: 벽돌이 붙은 선(side_hint)을 먼저 씀. 곡선 진입에서는 오른쪽 선 벽돌이 로봇 왼쪽에 보이기도 해서
    // 로봇 정면 기준 좌우로 정하면 틀림. 붙은 선을 모를 때만 로봇 기준 좌우 (정면에 걸치면 모름)
    if (side_hint == 0 && yl >= 0.0 && yr <= 0.0)
        return false;
    const bool right = side_hint != 0 ? side_hint > 0 : yl < 0.0;
    // 박스는 옆면까지 잡혀 안쪽 끝이 실제보다 로봇 쪽으로 나옴 → 벽돌이 붙은 선이 보이면 "선 + 벽돌 가로"로 고침
    // (벽돌 앞면 줄부터 화면 아래까지, 붙은 쪽으로 con_attach_mm 안의 선)
    if (con_brick_w_mm_ > 0)
    {
        const int cols = lines.cols;
        const int row0 = std::clamp(int(std::max(dst[0].y, dst[1].y)), 0, rows - 1);
        const int cx = std::clamp(int((dst[0].x + dst[1].x) / 2), 0, cols - 1);
        const int reach = int(std::lround(con_attach_mm_ / 1000.0 / geo_.m_per_px_x));
        const int xe = right ? std::min(cols - 1, int(std::max(dst[0].x, dst[1].x)) + reach)
                             : std::max(0, int(std::min(dst[0].x, dst[1].x)) - reach);
        std::vector<double> ly;
        for (int y = row0; y < rows; y += 2)
        {
            const uchar *lr = lines.ptr<uchar>(y);
            if (right)
            {
                for (int x = cx; x <= xe; ++x)
                    if (lr[x]) { ly.push_back(lane_path::bev_to_robot({double(x), double(y)}, geo_).y); break; }
            }
            else
            {
                for (int x = cx; x >= xe; --x)
                    if (lr[x]) { ly.push_back(lane_path::bev_to_robot({double(x), double(y)}, geo_).y); break; }
            }
        }
        if (ly.size() >= 3)
        {
            std::nth_element(ly.begin(), ly.begin() + ly.size() / 2, ly.end());
            const double line_y = ly[ly.size() / 2], bw = con_brick_w_mm_ / 1000.0;
            if (right)
                yl = std::min(yl, line_y + bw);
            else
                yr = std::max(yr, line_y - bw);
        }
    }
    out = {x0, yl, yr, right};
    return true;
}

cv::Point2d PathNode::robot_to_world(const cv::Point2d &r, const lane_path::Pose2D &p)
{
    const double c = std::cos(p.th), s = std::sin(p.th);
    return {p.x + c * r.x - s * r.y, p.y + s * r.x + c * r.y};
}

// 기억한 벽돌 k(0 오른쪽 선, 1 왼쪽 선)를 지금 로봇 좌표로 (월드 좌표로 기억함)
obstacles::Quad PathNode::held_quads_one(int k) const
{
    obstacles::Quad r;
    const double c = std::cos(pose_.th), s = std::sin(pose_.th);
    for (int j = 0; j < 4; ++j)
    {
        const double dx = con_hold_[k].q[j].x - pose_.x, dy = con_hold_[k].q[j].y - pose_.y;
        r[j] = {c * dx + s * dy, -s * dx + c * dy};
    }
    return r;
}

void PathNode::mask_callback(const tb_interfaces::msg::LaneMask::ConstSharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg->label, msg, "mono8");
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
        return;
    }
    const cv::Mat &label = cv_ptr->image;
    if (label.empty())
        return;

    update_pose(now());

    // ====== BEV가 바뀌었으면 (tb_line 파라미터 변경) 기억 초기화
    std::array<double, 9> h;
    std::copy(msg->homography.begin(), msg->homography.end(), h.begin());
    if (h != last_h_ || msg->lane_w_px != last_msg_lane_w_)
    {
        tracker_.reset();
        last_seed_ = {-1, -1};
        last_h_ = h;
        last_msg_lane_w_ = msg->lane_w_px;
    }

    geo_.bev_size = label.cols;
    geo_.lane_w_px = std::max(8, msg->lane_w_px);
    geo_.m_per_px_x = msg->m_per_px_x;
    geo_.m_per_px_y = msg->m_per_px_y;
    geo_.bottom_m = msg->bottom_offset_m;
    skel_p_.lane_w = geo_.lane_w_px;

    // ====== 라벨 → 마스크
    cv::Mat white = (label == LABEL_WHITE);
    // 흰 점선 → 실선 (조각 10cm·빈칸 10cm 구간). 벽·색 규칙 모두 이어진 선으로 봄
    if (white_dash_ && wdash_max_mm_ > 0)
        lane_skel::bridge_dashes(white, geo_.m_per_px_x, geo_.m_per_px_y, wdash_max_mm_ / 1000.0,
                                 wdash_gap_mm_ / 1000.0, wdash_angle_deg_, wdash_offset_mm_ / 1000.0,
                                 std::max(3, int(std::lround(0.018 / std::max(1e-4, geo_.m_per_px_x)))));
    const cv::Mat yellow = (label == LABEL_YELLOW);
    const cv::Mat valid = (label != LABEL_BLIND);

    // ====== 정지선: 보이면 월드 좌표로 기억 (BEV 아래 사각지대로 들어가 안 보여도 계속 자르게)
    cv::Mat red;
    if (stop_line_)
    {
        red = (label == LABEL_RED);
        double xr;
        if (find_stop_line(red, sl_min_w_px_, geo_, xr))
        {
            if (!sl_have_)
                RCLCPP_INFO(get_logger(), "stop_line: 정지선 찾음 %.2fm", xr);
            sl_pt_ = {pose_.x + xr * std::cos(pose_.th), pose_.y + xr * std::sin(pose_.th)};
            sl_have_ = true;
        }
    }

    // 주차 모드: 노란 점선을 벽에서 빼서 칸 안까지 가지가 뻗게 함
    cv::Mat yellow_wall = yellow;
    if (parking_)
    {
        yellow_wall = yellow.clone();
        lane_skel::remove_dashes(yellow_wall, dash_len_);
    }
    cv::Mat wall = white | yellow_wall;

    // ====== 벽돌 → 벽
    std::vector<obstacles::Box> bricks;
    if (last_det_)
    {
        const double age_ms = std::abs((rclcpp::Time(msg->header.stamp) -
                                        rclcpp::Time(last_det_->header.stamp)).seconds()) * 1000.0;
        if (age_ms <= det_max_age_ms_)
            for (const auto &d : last_det_->detections)
                if (d.label == "brick")
                    bricks.push_back({d.x, d.y, d.w, d.h});
    }
    // ====== 공사 구간: 진입/이탈, 벽돌 벽 크기 바꿈 (차선 폭은 일반 값 그대로: 한쪽 선만 보여도 26cm 차로로 봄)
    update_construction(*msg, !bricks.empty());
    const bool con = con_on();
    // 공사 구간 이동 거리 (지난 벽돌 기억을 이만큼 뒤로 옮김). 벽돌 잇기·지난 벽돌 피하기에서 같이 씀
    if (!con)
    {
        con_hold_[0].on = con_hold_[1].on = false;
        con_have_pose_ = false;
    }
    else
    {
        if (con_have_pose_)
            con_travel_ += std::hypot(pose_.x - con_last_pose_.x, pose_.y - con_last_pose_.y);
        con_last_pose_ = pose_;
        con_have_pose_ = true;
    }

    const cv::Mat H(3, 3, CV_64F, h.data());
    const cv::Mat lines = wall.clone(); // 벽돌 그리기 전 차선만
    cv::Mat con_ext;                    // 공사 구간: 벽돌에 가려 끊긴 선을 이은 부분 (디버그 표시용)
    std::vector<int> brick_sides;       // 벽돌마다 붙은 선 (+1 오른쪽, -1 왼쪽, 0 모름). draw_boxes가 선을 보고 정함
    // 벽돌은 공사 모드에서만 경로에 반영 (다른 구간의 오검출·트랙 밖 벽돌로 경로가 꺾이지 않게).
    // 공사 모드 진입·종료 판단(update_construction)에는 위에서 그대로 씀
    if (!bricks.empty() && (con || brick_always_))
    {
        const double depth = (con ? con_depth_mm_ : brick_depth_mm_) / 1000.0;
        const double margin = (con ? con_margin_mm_ : brick_margin_mm_) / 1000.0;
        cv::Mat brick_m = con ? cv::Mat::zeros(wall.size(), CV_8U) : wall;
        obstacles::draw_boxes(brick_m, H, bricks, depth / geo_.m_per_px_y, margin / geo_.m_per_px_x,
                              &lines, brick_gap_mm_ / 1000.0 / geo_.m_per_px_x,
                              con ? con_edge_mm_ / 1000.0 / geo_.m_per_px_x : 0.0,
                              con ? con_brick_w_mm_ / 1000.0 / geo_.m_per_px_x : 0.0,
                              con ? con_attach_mm_ / 1000.0 / geo_.m_per_px_x : 0.0,
                              !con, // 공사 구간: BEV보다 먼 벽돌은 붙은 쪽을 몰라 잘못 피하므로 안 그림
                              con && con_offscreen_side_, // 화면 옆 끝까지 간 벽돌은 그쪽(화면 밖) 선에 붙은 것
                              &brick_sides);
        if (con)
        {
            // 벽돌에 가려 끊긴 선을 벽돌 너머로 이음 (벽돌과 선 사이로 경로가 새지 않게)
            if (con_extend_mm_ > 0)
            {
                con_ext = cv::Mat::zeros(wall.size(), CV_8U);
                obstacles::extend_occluded(lines, brick_m, con_ext, 0.08 / geo_.m_per_px_y,
                                           con_extend_mm_ / 1000.0 / geo_.m_per_px_y,
                                           std::max(3, int(std::lround(0.012 / geo_.m_per_px_x))));
                wall |= con_ext;
            }
            wall |= brick_m;
        }
    }

    // ====== 공사 구간: 같은 쪽에 붙은 벽돌끼리 사이를 막음 (r-r-l처럼 같은 쪽이 연달아 올 때)
    //   앞 벽돌이 화면 아래로 빠지면 두 벽돌 사이 선 옆 빈 곳이 길로 보여 그리로 들어감 (녹화 1029 rrl)
    //   기억 중인 지난 벽돌(con_hold_) + 지금 보이는 벽돌을 쪽마다 x 순서로 보고, 이웃한 둘 사이를 선 쪽으로 막음
    con_bridges_.clear();
    cv::Mat bridge_m; // 디버그 표시용
    if (con && con_bridge_mm_ > 0)
    {
        std::vector<obstacles::SideBrick> sb;
        for (int k = 0; k < 2; ++k)
        {
            if (!con_hold_[k].on)
                continue;
            const auto q = held_quads_one(k);
            double x0 = 1e9, x1 = -1e9, yl = -1e9, yr = 1e9;
            for (const auto &pt : q)
                x0 = std::min(x0, pt.x), x1 = std::max(x1, pt.x), yl = std::max(yl, pt.y), yr = std::min(yr, pt.y);
            const bool right = k == 0;
            sb.push_back({x0, x1, right ? yl : yr, right});
        }
        for (size_t i = 0; i < bricks.size(); ++i)
        {
            BrickR r;
            if (!brick_robot(bricks[i], H, lines, label.rows, i < brick_sides.size() ? brick_sides[i] : 0, r))
                continue;
            sb.push_back({r.x0, r.x0 + 0.08, r.right ? r.yl : r.yr, r.right}); // 벽돌 두께 8cm
        }
        con_bridges_ = obstacles::bridge_same_side(sb, con_bridge_mm_ / 1000.0, 0.05, con_bridge_dy_mm_ / 1000.0,
                                                   con_bridge_margin_mm_ / 1000.0, 0.6, con_bridge_from_back_);
        if (!con_bridges_.empty())
        {
            bridge_m = cv::Mat::zeros(wall.size(), CV_8U);
            for (const auto &q : con_bridges_)
            {
                std::vector<cv::Point> px;
                for (const auto &pt : q)
                {
                    const cv::Point2d b = lane_path::robot_to_bev(pt, geo_);
                    px.emplace_back(cvRound(b.x), cvRound(b.y));
                }
                cv::fillConvexPoly(bridge_m, px, cv::Scalar(255));
            }
            wall |= bridge_m;
        }
    }

    // ====== 좌/우 명령이 새로 들어오면 직진 때 기억한 경로를 버림
    // (표지판이 교차로 바로 앞에서 보이면, 직진으로 기억한 다른 가지를 따라가거나
    //  turn으로 고른 새 가지를 "기억과 다르다"고 10프레임 동안 거부해서 T자에서 엉뚱하게 감)
    if (turn_ != last_turn_)
    {
        if (turn_ != 0 && turn_drop_memory_)
        {
            tracker_.drop_memory();
            last_seed_ = {-1, -1};
            RCLCPP_INFO(get_logger(), "turn %u: 기억 경로 버림", turn_);
        }
        last_turn_ = turn_;
    }

    // ====== 스켈레톤 → 경로
    // 시드 힌트: 기억 경로의 연장선(움직임 반영) > 직전 시드 > 하단 중앙
    // 기억 경로로 예측한 점이 있으면 시드·시작점 모두 그 근처에서 고름
    cv::Point seed_hint = last_seed_, predicted;
    const bool has_pred = tracker_.predict_seed(pose_, geo_, predicted);
    if (has_pred)
        seed_hint = predicted;
    lane_skel::Params sp = skel_p_;
    if (con && con_dead_end_mm_ > 0) // 공사 구간: 벽돌과 선 사이 구석으로 뻗은 막다른 가지 삭제
        sp.dead_end_len = int(std::lround(con_dead_end_mm_ / 1000.0 / geo_.m_per_px_y));
    const lane_skel::Result skel = lane_skel::build(wall, valid, sp, &seed_hint,
                                                    has_pred ? &predicted : nullptr);
    last_seed_ = skel.ok ? skel.seed : cv::Point(-1, -1);

    const auto turn = static_cast<lane_path::Turn>(std::min<uint8_t>(turn_, 2));
    // 공사 구간 전용: 기억 의심 프레임, 직진 갈림길 기준
    tracker_.p.override_frames = con ? con_override_frames_ : override_frames_;
    tracker_.p.cross_override_frames = con ? con_cross_override_frames_ : 0;
    tracker_.p.straight_by_lane = con && con_straight_by_lane_;
    lane_path::Output out = tracker_.update(skel, skel_p_.scale, white, yellow, valid, geo_, turn, pose_);

    // 공사 구간: 스켈레톤 경로는 벽돌 앞에서 ㄱ자로 꺾이므로 모서리를 둥글게 (기억에는 원래 경로가 남음)
    if (con && con_smooth_mm_ > 0)
        smooth_path(out.path, con_smooth_mm_ / 1000.0);

    // 교차로 명령 중: 사각지대 때문에 경로가 모서리(BEV 아랫줄)에서 시작하면
    // 로봇 → 첫 점 직선을 붙여 fillet이 모서리를 원호로 깎을 수 있게 함
    if (!con && turn != lane_path::Turn::Straight && !out.path.empty())
    {
        const cv::Point2d p0 = out.path.front();
        const double d = std::hypot(p0.x, p0.y);
        if (p0.x > 0.03)
        {
            const int n = int(std::ceil(d / 0.02)); // 2cm 간격
            std::vector<cv::Point2d> lead;
            for (int i = 0; i < n; ++i)
                lead.push_back(p0 * (double(i) / n));
            out.path.insert(out.path.begin(), lead.begin(), lead.end());
        }
    }
    // 일반 주행: 급한 모서리(교차로 ㄱ자)를 원호로 → 피봇 없이 전진하며 돎 (코너까지 거리를 재지 않아 지나치지 않음)
    if (!con && turn_radius_mm_ > 0)
        lane_path::fillet_corners(out.path, turn_radius_mm_ / 1000.0, turn_min_deg_ * M_PI / 180.0);

    // 공사 구간: 화면 아랫줄을 막 벗어나려는 벽돌을 쪽(좌/우)마다 하나씩 기억 → 몸통이 옆을 지나는 동안 경로를 떼어 놓음
    //   위치는 회전을 무시하고 "이동 거리만큼 뒤로" 옮김 (오도메트리 방향 오차가 쌓이지 않게, 수십 cm만 씀)
    if (!con)
        con_near_.clear();
    else
    {
        for (size_t i = 0; i < bricks.size(); ++i)
        {
            BrickR r;
            if (!brick_robot(bricks[i], H, lines, label.rows, i < brick_sides.size() ? brick_sides[i] : 0, r))
                continue;
            if (r.x0 > geo_.bottom_m + 0.08)
                continue; // 아직 멀리 있음 (스켈레톤이 피함)
            const double x0 = r.x0, yl = r.yl, yr = r.yr;
            const bool right = r.right;
            ConHold &hd = con_hold_[right ? 0 : 1]; // [0] 오른쪽, [1] 왼쪽
            hd.on = true;
            // 월드(odom) 좌표로 기억 → 로봇이 돌아도 제자리 (예전: 회전 무시하고 이동 거리만큼 뒤로만 옮겨서 곡선에서 어긋남)
            const obstacles::Quad qr = {cv::Point2d(x0, yl), cv::Point2d(x0, yr), cv::Point2d(x0 + 0.10, yr), cv::Point2d(x0 + 0.10, yl)}; // 두께 8cm + 여유
            for (int j = 0; j < 4; ++j)
                hd.q[j] = robot_to_world(qr[j], pose_);
            hd.travel0 = con_travel_;
        }

        std::vector<obstacles::Quad> near;
        for (int k = 0; k < 2; ++k)
        {
            ConHold &hd = con_hold_[k];
            if (!hd.on)
                continue;
            // 0.8m 넘게 가면 기억을 버림 (회전을 무시하고 옮기므로 오래 쓰면 위치가 틀림)
            if (con_travel_ - hd.travel0 > 0.8)
            {
                hd.on = false;
                continue;
            }
            const obstacles::Quad r = held_quads_one(k);
            double x0 = 1e9, x1 = -1e9;
            for (const auto &pt : r)
                x0 = std::min(x0, pt.x), x1 = std::max(x1, pt.x);
            // 로봇 뒤 끝(14cm)이 벽돌 뒷면을 지나고 여유만큼 더 갔으면 피하기는 끝 (기억은 벽돌 잇기에 계속 씀)
            if (x1 + (con_keep_rear_mm_ + con_clear_mm_ + con_hold_mm_) / 1000.0 < 0.0)
                continue;
            // 아직 화면에 보이는 동안은 빨간 벽으로 피하므로 적용 안 함 (사각지대에 들어간 뒤에만)
            // (녹화: 보이는 동안부터 걸어 출구 좌회전 앞 0.16~0.34m 경로가 1.2초 동안 정면으로 눌려 선을 넘음)
            if (x0 > geo_.bottom_m)
                continue;
            near.push_back(r);
        }
        // 로봇: 바퀴 축 기준 앞 9cm, 뒤 14cm, 반폭 10.5cm
        if (con_keep_ > 0)
            obstacles::keep_clear(out.path, near, 0.09, con_keep_rear_mm_ / 1000.0, 0.105, con_clear_mm_ / 1000.0, con_hold_mm_ / 1000.0,
                                  con_keep_ == 1);
        else
            near.clear();
        con_near_ = near;
    }

    // ====== 정지선: 경로를 정지선 - 여유 앞에서 자름 (fillet·공사 처리 뒤, 점이 새로 생길 수 있어서 마지막에)
    //   필터가 아니라 처음 넘는 점에서 끊음 (커브로 다시 앞으로 오는 점이 남지 않게)
    //   다 잘리면 빈 경로 대신 로봇 앞 짧은 경로 → tb_drive가 경로 끝(stop_dist)에서 멈춤
    //   (빈 경로를 보내면 tb_drive가 hold_ms 동안 직전 경로로 계속 감)
    sl_cut_x_ = -1.0;
    if (stop_line_ && sl_have_)
    {
        const double x_line = (sl_pt_.x - pose_.x) * std::cos(pose_.th) + (sl_pt_.y - pose_.y) * std::sin(pose_.th);
        sl_cut_x_ = x_line - sl_margin_mm_ / 1000.0;
        size_t keep = 0;
        while (keep < out.path.size() && out.path[keep].x <= sl_cut_x_)
            ++keep;
        if (keep < out.path.size())
            out.path.resize(keep);
        if (out.path.size() < 2)
            out.path = {cv::Point2d(0.0, 0.0), cv::Point2d(0.01, 0.0)};
    }


    nav_msgs::msg::Path path;
    path.header.stamp = msg->header.stamp;
    path.header.frame_id = "base_link"; // x 앞, y 왼쪽 [m]
    for (const auto &q : out.path)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = q.x;
        ps.pose.position.y = q.y;
        ps.pose.orientation.w = 1.0;
        path.poses.push_back(ps);
    }
    path_pub_->publish(path);

    std_msgs::msg::String src;
    src.data = lane_path::source_name(out.src);
    src_pub_->publish(src);
    if (out.src != last_src_)
    {
        RCLCPP_INFO(get_logger(), "path source: %s", src.data.c_str());
        last_src_ = out.src;
    }
    std_msgs::msg::Bool con_state;
    con_state.data = con_on();
    con_pub_->publish(con_state);
    std_msgs::msg::String con_info;
    con_info.data = construction_info();
    con_info_pub_->publish(con_info);

    // ====== GUI 주행+ 탭: 선·벽돌·기억을 로봇 좌표로 (구독자가 있을 때만)
    if (world_pub_->get_subscription_count() > 0)
    {
        tb_interfaces::msg::PathWorld w;
        w.header.stamp = msg->header.stamp;
        w.header.frame_id = "base_link";
        auto P = [](const cv::Point2d &q)
        {
            geometry_msgs::msg::Point o;
            o.x = q.x;
            o.y = q.y;
            return o;
        };
        auto poly = [](const obstacles::Quad &q)
        {
            geometry_msgs::msg::Polygon o;
            for (const auto &pt : q)
            {
                geometry_msgs::msg::Point32 p;
                p.x = float(pt.x);
                p.y = float(pt.y);
                o.points.push_back(p);
            }
            return o;
        };
        // 마스크 → 줄마다 선 조각 가운데 (3줄마다)
        auto runs = [this, &P](const cv::Mat &m, std::vector<geometry_msgs::msg::Point> &out)
        {
            if (m.empty())
                return;
            for (int y = 0; y < m.rows; y += 3)
            {
                const uchar *r = m.ptr<uchar>(y);
                for (int x = 0; x < m.cols; ++x)
                {
                    if (!r[x])
                        continue;
                    int e = x;
                    while (e + 1 < m.cols && r[e + 1])
                        ++e;
                    out.push_back(P(lane_path::bev_to_robot({(x + e) / 2.0, double(y)}, geo_)));
                    x = e;
                }
            }
        };
        runs(white & valid, w.white);
        runs(yellow & valid, w.yellow);
        if (!red.empty())
            runs(red & valid, w.red);
        runs(con_ext, w.ext);
        // 벽돌 벽 = 벽 - 선 - 이은 선 - 잇기
        cv::Mat bm = (wall != 0) & (white == 0) & (yellow_wall == 0);
        if (!con_ext.empty())
            bm &= (con_ext == 0);
        if (!bridge_m.empty())
            bm &= (bridge_m == 0);
        std::vector<std::vector<cv::Point>> cs;
        cv::findContours(bm, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (auto &c : cs)
        {
            if (cv::contourArea(c) < 20)
                continue;
            std::vector<cv::Point> a;
            cv::approxPolyDP(c, a, 2.0, true);
            geometry_msgs::msg::Polygon o;
            for (const auto &q : a)
            {
                const cv::Point2d r = lane_path::bev_to_robot({double(q.x), double(q.y)}, geo_);
                geometry_msgs::msg::Point32 p;
                p.x = float(r.x);
                p.y = float(r.y);
                o.points.push_back(p);
            }
            w.bricks.push_back(o);
        }
        for (int k = 0; k < 2; ++k)
            if (con_hold_[k].on)
                w.held.push_back(poly(held_quads_one(k)));
        for (const auto &q : con_bridges_)
            w.bridges.push_back(poly(q));
        for (const auto &q : con_near_)
            w.keep.push_back(poly(q));
        for (const auto &q : tracker_.memory_robot(pose_))
            w.memory.push_back(P(q));
        for (const auto &q : tracker_.trail_robot(pose_))
            w.trail.push_back(P(q));
        w.stop_x = -1.0f;
        if (stop_line_ && sl_have_)
            w.stop_x = float((sl_pt_.x - pose_.x) * std::cos(pose_.th) + (sl_pt_.y - pose_.y) * std::sin(pose_.th));
        w.stop_cut_x = float(sl_cut_x_);
        w.construction = con_on();
        w.turn = turn_;
        world_pub_->publish(w);
    }

    // ====== 디버그: 라벨을 색으로 그리고 스켈레톤·경로·벽돌 표시
    if (debug_pub_->get_subscription_count() > 0)
    {
        cv::Mat view(label.size(), CV_8UC3, cv::Scalar(70, 70, 70));
        lane_skel::draw_debug(view, valid, skel, skel_p_.scale);
        view.setTo(cv::Scalar(0, 0, 200), (wall != 0) & (white == 0) & (yellow_wall == 0)); // 벽돌: 빨강
        if (!con_ext.empty())
            view.setTo(cv::Scalar(170, 170, 170), con_ext); // 이은 선: 회색
        if (!bridge_m.empty())
            view.setTo(cv::Scalar(180, 60, 200), bridge_m & (white == 0) & (yellow_wall == 0)); // 같은 쪽 벽돌 잇기: 보라
        view.setTo(cv::Scalar(235, 235, 235), white);
        view.setTo(cv::Scalar(0, 210, 255), yellow);
        if (skel.ok)
        {
            std::vector<cv::Point> pts;
            cv::findNonZero(skel.skel, pts);
            for (const auto &p : pts)
                cv::circle(view, p * skel_p_.scale, 1, cv::Scalar(200, 255, 100), -1);
        }
        if (!red.empty())
            view.setTo(cv::Scalar(60, 60, 255), red & valid); // 빨간선 (정지선 모드일 때만)
        lane_path::draw_path(view, out, geo_);
        if (sl_cut_x_ > geo_.bottom_m) // 경로를 자른 줄 (보이는 곳에 있을 때)
        {
            const cv::Point2d a = lane_path::robot_to_bev({sl_cut_x_, 0.15}, geo_);
            const cv::Point2d b = lane_path::robot_to_bev({sl_cut_x_, -0.15}, geo_);
            cv::line(view, {cvRound(a.x), cvRound(a.y)}, {cvRound(b.x), cvRound(b.y)}, cv::Scalar(0, 0, 255), 2);
        }
        for (const auto &q : con_near_) // 기억한 벽돌 (주황)
        {
            std::vector<cv::Point> px;
            for (const auto &r : q)
            {
                const cv::Point2d b = lane_path::robot_to_bev(r, geo_);
                px.emplace_back(cvRound(b.x), cvRound(b.y));
            }
            cv::polylines(view, px, true, cv::Scalar(0, 140, 255), 1);
        }
        static const char *turn_txt[] = {"straight", "left", "right"};
        cv::putText(view, std::string(turn_txt[static_cast<int>(turn)]) + (parking_ ? " +park" : "") + (white_dash_ ? " +wdash" : "") + (con_on() ? " +con" : "") + (stop_line_ ? (sl_have_ ? " +stop" : " +stop?") : ""),
                    {6, label.rows - 8}, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);
        publish_debug(view, msg->header);
    }
}

// 공사 구간 진입/이탈. 진입은 /mission/construction true로만 (표지판 판단은 tb_brain)
// brick_seen: 이번 프레임에 벽돌 탐지가 있는지
void PathNode::update_construction(const tb_interfaces::msg::LaneMask &, bool brick_seen)
{
    // 토픽으로 받은 명령 처리
    const ConCmd cmd = con_cmd_;
    con_cmd_ = ConCmd::None;
    if (cmd == ConCmd::Start && !con_active_)
    {
        con_active_ = true;
        con_brick_seen_ = false;
        con_start_pose_ = pose_;
        con_reason_ = "명령";
        RCLCPP_INFO(get_logger(), "construction: 시작");
    }
    else if ((cmd == ConCmd::Stop || cmd == ConCmd::Exit) && con_active_)
    {
        end_construction(cmd == ConCmd::Stop ? "명령" : "끝내기 요청");
    }
    if (!con_active_)
        return;

    // 벽돌을 한 번도 못 보고 con_no_brick_mm 넘게 가면 잘못 들어온 것으로 보고 끝 (표지판 오검출 등)
    if (!con_brick_seen_ && !brick_seen && con_no_brick_mm_ > 0 &&
        std::hypot(pose_.x - con_start_pose_.x, pose_.y - con_start_pose_.y) * 1000.0 > con_no_brick_mm_)
    {
        end_construction("벽돌 없이 " + std::to_string(con_no_brick_mm_) + "mm");
        return;
    }

    // 벽돌을 본 뒤 con_exit_mm 가는 동안 벽돌이 안 보이면 끝 (0이면 명령으로 끌 때까지 유지)
    if (brick_seen)
    {
        con_brick_seen_ = true;
        con_brick_pose_ = pose_;
    }
    else if (con_brick_seen_ && con_exit_mm_ > 0 &&
             std::hypot(pose_.x - con_brick_pose_.x, pose_.y - con_brick_pose_.y) * 1000.0 > con_exit_mm_)
    {
        end_construction("벽돌이 " + std::to_string(con_exit_mm_) + "mm 동안 안 보임");
    }
}

void PathNode::end_construction(const std::string &why)
{
    con_active_ = false;
    con_reason_ = "끝: " + why;
    RCLCPP_INFO(get_logger(), "construction: %s, 일반 주행으로", why.c_str());
}

// GUI 표시용 한 줄: "ON 명령 brick=1 dist=0.42" / "off 끝: 끝내기 요청"
std::string PathNode::construction_info() const
{
    char buf[160];
    if (con_on())
        std::snprintf(buf, sizeof(buf), "ON %s brick=%d dist=%.2f", con_reason_.c_str(), con_brick_seen_ ? 1 : 0,
                      std::hypot(pose_.x - con_start_pose_.x, pose_.y - con_start_pose_.y));
    else
        std::snprintf(buf, sizeof(buf), "off %s", con_reason_.c_str());
    return buf;
}

// 경로 점을 앞뒤 half 거리 안의 점들 평균으로 2번 (끝점은 그대로, 끝 가까이는 창을 줄임)
void PathNode::smooth_path(std::vector<cv::Point2d> &path, double half)
{
    const size_t n = path.size();
    if (n < 5)
        return;
    std::vector<double> s(n, 0.0); // 누적 길이
    for (size_t i = 1; i < n; ++i)
        s[i] = s[i - 1] + cv::norm(path[i] - path[i - 1]);
    for (int pass = 0; pass < 2; ++pass)
    {
        std::vector<cv::Point2d> out(path);
        for (size_t i = 1; i + 1 < n; ++i)
        {
            const double h = std::min({half, s[i] - s[0], s[n - 1] - s[i]}); // 양쪽 같은 폭
            cv::Point2d sum(0, 0);
            int cnt = 0;
            for (size_t j = i; j-- > 0 && s[i] - s[j] <= h;)
                sum += path[j], ++cnt;
            for (size_t j = i; j < n && s[j] - s[i] <= h; ++j)
                sum += path[j], ++cnt;
            if (cnt > 0)
                out[i] = sum * (1.0 / cnt);
        }
        path.swap(out);
    }
}

void PathNode::publish_debug(const cv::Mat &img, const std_msgs::msg::Header &header)
{
    sensor_msgs::msg::CompressedImage out;
    out.header = header;
    out.format = "bgr8; jpeg compressed bgr8";
    cv::imencode(".jpg", img, out.data, {cv::IMWRITE_JPEG_QUALITY, 80});
    debug_pub_->publish(out);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathNode>());
    rclcpp::shutdown();
    return 0;
}
