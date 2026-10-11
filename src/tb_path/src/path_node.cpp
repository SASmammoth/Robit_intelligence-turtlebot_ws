#include "tb_path/path_node.hpp"
#include "tb_path/obstacles.hpp"

#include <algorithm>
#include <cmath>

namespace {
// tb_line 라벨 값
constexpr uint8_t LABEL_WHITE = 1;
constexpr uint8_t LABEL_YELLOW = 2;
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

    con_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/mission/construction", 10, [this](const std_msgs::msg::Bool::ConstSharedPtr m)
        {
            if (m->data != con_manual_)
                RCLCPP_INFO(get_logger(), "construction (manual): %d", m->data);
            con_manual_ = m->data;
            if (!m->data)
                con_done_ = false; });

    // ====== 출력
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/lane/path", 10);
    src_pub_ = create_publisher<std_msgs::msg::String>("/lane/path_source", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/lane/debug/skel/compressed", 10);
    con_pub_ = create_publisher<std_msgs::msg::Bool>("/lane/construction", 10);

    // ====== 파라미터
    declare_int("skel.seal_len", 30, 100);
    declare_int("skel.prune_len", 20, 100);
    declare_int("skel.dash_len", 40, 160);
    declare_int("path.brick_depth_mm", 60, 500);
    declare_int("path.brick_margin_mm", 30, 300);
    declare_int("path.brick_gap_mm", 120, 300);   // 벽돌 끝과 차선 사이 이보다 좁은 틈은 막음 (0이면 안 막음)
    declare_int("path.det_max_age_ms", 300, 2000);
    declare_int("path.mem_travel_mm", 600, 3000);
    declare_int("skel.max_lane_thick", 20, 200);
    declare_int("path.max_dev_mm", 50, 500);
    declare_int("path.override_frames", 10, 100);
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
    declare_int("path.con_keep", 1, 2);                // 지난 벽돌 피하기: 0 끔, 1 그쪽으로 꺾지만 못하게(약), 2 벽돌에서 떼어 놓기(강)
    declare_int("path.con_hold_mm", 40, 500);
    declare_int("path.con_keep_rear_mm", 100, 300);    // 지난 벽돌: 바퀴 축이 뒷면을 지나고 이만큼 더 가야 그쪽으로 꺾음 (로봇 뒤 끝 14cm)          // 기억한 벽돌 뒷면을 로봇 뒤 끝이 지나고 이만큼 더 가야 그쪽으로 붙음
    declare_int("path.con_clear_mm", 30, 200);         // 기억한 벽돌과 로봇 옆면 사이 여유          // 선에 안 닿은 벽돌 끝이 화면 끝에서 이 안이면 화면 끝까지 막음
    declare_int("path.con_exit_mm", 600, 3000);
    declare_int("path.con_smooth_mm", 80, 300);        // 공사 구간 경로를 이 폭으로 평균내 모서리를 둥글게 (0이면 끔)        // 벽돌이 이만큼 가는 동안 안 보이면 공사 구간 끝
    declare_int("path.con_sign_frames", 3, 30);     // 공사 표지판이 이만큼 연속으로 보이면 자동 진입

    reload_params();
    skel_p_.max_lane_thick = get_parameter("skel.max_lane_thick").as_int();
    auto &tp = tracker_.p;
    tp.max_dev_m = get_parameter("path.max_dev_mm").as_int() / 1000.0;
    tp.override_frames = get_parameter("path.override_frames").as_int();
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
    brick_depth_mm_ = get_parameter("path.brick_depth_mm").as_int();
    brick_margin_mm_ = get_parameter("path.brick_margin_mm").as_int();
    brick_gap_mm_ = get_parameter("path.brick_gap_mm").as_int();
    det_max_age_ms_ = get_parameter("path.det_max_age_ms").as_int();
    tracker_.p.mem_travel_m = get_parameter("path.mem_travel_mm").as_int() / 1000.0;

    // 선언만 되어 있고 읽지 않던 값들 (yaml·GUI에서 바꿔도 반영이 안 됐음)
    skel_p_.max_lane_thick = get_parameter("skel.max_lane_thick").as_int();
    auto &tp = tracker_.p;
    tp.max_dev_m = get_parameter("path.max_dev_mm").as_int() / 1000.0;
    tp.override_frames = get_parameter("path.override_frames").as_int();
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
    con_keep_rear_mm_ = get_parameter("path.con_keep_rear_mm").as_int();
    con_dead_end_mm_ = get_parameter("path.con_dead_end_mm").as_int();
    con_clear_mm_ = get_parameter("path.con_clear_mm").as_int();
    con_brick_w_mm_ = get_parameter("path.con_brick_w_mm").as_int();
    con_exit_mm_ = get_parameter("path.con_exit_mm").as_int();
    con_smooth_mm_ = get_parameter("path.con_smooth_mm").as_int();
    con_sign_frames_ = std::max(1, int(get_parameter("path.con_sign_frames").as_int()));
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
    const cv::Mat white = (label == LABEL_WHITE);
    const cv::Mat yellow = (label == LABEL_YELLOW);
    const cv::Mat valid = (label != LABEL_BLIND);

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

    const cv::Mat H(3, 3, CV_64F, h.data());
    const cv::Mat lines = wall.clone(); // 벽돌 그리기 전 차선만
    cv::Mat con_ext;                    // 공사 구간: 벽돌에 가려 끊긴 선을 이은 부분 (디버그 표시용)
    if (!bricks.empty())
    {
        const double depth = (con ? con_depth_mm_ : brick_depth_mm_) / 1000.0;
        const double margin = (con ? con_margin_mm_ : brick_margin_mm_) / 1000.0;
        cv::Mat brick_m = con ? cv::Mat::zeros(wall.size(), CV_8U) : wall;
        obstacles::draw_boxes(brick_m, H, bricks, depth / geo_.m_per_px_y, margin / geo_.m_per_px_x,
                              &lines, brick_gap_mm_ / 1000.0 / geo_.m_per_px_x,
                              con ? con_edge_mm_ / 1000.0 / geo_.m_per_px_x : 0.0,
                              con ? con_brick_w_mm_ / 1000.0 / geo_.m_per_px_x : 0.0,
                              con ? con_attach_mm_ / 1000.0 / geo_.m_per_px_x : 0.0,
                              !con); // 공사 구간: BEV보다 먼 벽돌은 붙은 쪽을 몰라 잘못 피하므로 안 그림
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
    lane_path::Output out = tracker_.update(skel, skel_p_.scale, white, yellow, valid, geo_, turn, pose_);

    // 공사 구간: 스켈레톤 경로는 벽돌 앞에서 ㄱ자로 꺾이므로 모서리를 둥글게 (기억에는 원래 경로가 남음)
    if (con && con_smooth_mm_ > 0)
        smooth_path(out.path, con_smooth_mm_ / 1000.0);
    // 일반 주행: 급한 모서리(교차로 ㄱ자)를 원호로 → 피봇 없이 전진하며 돎 (코너까지 거리를 재지 않아 지나치지 않음)
    if (!con && turn_radius_mm_ > 0)
        lane_path::fillet_corners(out.path, turn_radius_mm_ / 1000.0, turn_min_deg_ * M_PI / 180.0);

    // 공사 구간: 화면 아랫줄을 막 벗어나려는 벽돌을 쪽(좌/우)마다 하나씩 기억 → 몸통이 옆을 지나는 동안 경로를 떼어 놓음
    //   위치는 회전을 무시하고 "이동 거리만큼 뒤로" 옮김 (오도메트리 방향 오차가 쌓이지 않게, 수십 cm만 씀)
    if (!con)
    {
        con_hold_[0].on = con_hold_[1].on = false;
        con_near_.clear();
        con_have_pose_ = false;
    }
    else
    {
        if (con_have_pose_)
            con_travel_ += std::hypot(pose_.x - con_last_pose_.x, pose_.y - con_last_pose_.y);
        con_last_pose_ = pose_;
        con_have_pose_ = true;

        for (const auto &b : bricks)
        {
            std::vector<cv::Point2f> src{{float(b.x), float(b.y + b.h)}, {float(b.x + b.w), float(b.y + b.h)}}, dst;
            cv::perspectiveTransform(src, dst, H);
            if (dst[0].y < 0 || dst[1].y < 0 || dst[0].y >= label.rows - 2 || dst[1].y >= label.rows - 2)
                continue; // 화면 위로 벗어났거나 박스 아랫변이 영상 아래에서 잘림 (바닥 위치가 틀림)
            const cv::Point2d A = lane_path::bev_to_robot({dst[0].x, dst[0].y}, geo_);
            const cv::Point2d C = lane_path::bev_to_robot({dst[1].x, dst[1].y}, geo_);
            const double x0 = std::min(A.x, C.x);
            double yl = std::max(A.y, C.y), yr = std::min(A.y, C.y);
            if (x0 > geo_.bottom_m + 0.08)
                continue; // 아직 멀리 있음 (스켈레톤이 피함)
            if (yl >= 0.0 && yr <= 0.0)
                continue; // 로봇 정면에 걸침 → 어느 쪽인지 모름
            const bool right = yl < 0.0;
            // 박스는 옆면까지 잡혀 안쪽 끝이 실제보다 로봇 쪽으로 나옴 → 벽돌이 붙은 선이 보이면 "선 + 벽돌 가로"로 고침
            // (벽돌 앞면 줄부터 화면 아래까지, 붙은 쪽으로 con_attach_mm 안의 선)
            if (con_brick_w_mm_ > 0)
            {
                const int row0 = std::clamp(int(std::max(dst[0].y, dst[1].y)), 0, label.rows - 1);
                const int cx = std::clamp(int((dst[0].x + dst[1].x) / 2), 0, label.cols - 1);
                const int reach = int(std::lround(con_attach_mm_ / 1000.0 / geo_.m_per_px_x));
                const int xe = right ? std::min(label.cols - 1, int(std::max(dst[0].x, dst[1].x)) + reach)
                                     : std::max(0, int(std::min(dst[0].x, dst[1].x)) - reach);
                std::vector<double> ly;
                for (int y = row0; y < label.rows; y += 2)
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
            ConHold &hd = con_hold_[right ? 0 : 1]; // [0] 오른쪽, [1] 왼쪽
            hd.on = true;
            hd.q = {cv::Point2d(x0, yl), cv::Point2d(x0, yr), cv::Point2d(x0 + 0.10, yr), cv::Point2d(x0 + 0.10, yl)}; // 두께 8cm + 여유
            hd.travel0 = con_travel_;
        }

        std::vector<obstacles::Quad> near;
        for (auto &hd : con_hold_)
        {
            if (!hd.on)
                continue;
            const double moved = con_travel_ - hd.travel0;
            obstacles::Quad r = hd.q;
            double x0 = 1e9, x1 = -1e9;
            for (auto &pt : r)
                pt.x -= moved, x0 = std::min(x0, pt.x), x1 = std::max(x1, pt.x);
            // 로봇 뒤 끝(14cm)이 벽돌 뒷면을 지나고 여유만큼 더 갔으면 끝
            if (x1 + (con_keep_rear_mm_ + con_clear_mm_ + con_hold_mm_) / 1000.0 < 0.0 || moved > 0.8)
            {
                hd.on = false;
                continue;
            }
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

    // ====== 디버그: 라벨을 색으로 그리고 스켈레톤·경로·벽돌 표시
    if (debug_pub_->get_subscription_count() > 0)
    {
        cv::Mat view(label.size(), CV_8UC3, cv::Scalar(70, 70, 70));
        lane_skel::draw_debug(view, valid, skel, skel_p_.scale);
        view.setTo(cv::Scalar(0, 0, 200), (wall != 0) & (white == 0) & (yellow_wall == 0)); // 벽돌: 빨강
        if (!con_ext.empty())
            view.setTo(cv::Scalar(170, 170, 170), con_ext); // 이은 선: 회색
        view.setTo(cv::Scalar(235, 235, 235), white);
        view.setTo(cv::Scalar(0, 210, 255), yellow);
        if (skel.ok)
        {
            std::vector<cv::Point> pts;
            cv::findNonZero(skel.skel, pts);
            for (const auto &p : pts)
                cv::circle(view, p * skel_p_.scale, 1, cv::Scalar(200, 255, 100), -1);
        }
        lane_path::draw_path(view, out, geo_);
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
        cv::putText(view, std::string(turn_txt[static_cast<int>(turn)]) + (parking_ ? " +park" : "") +
                              (con_on() ? " +con" : ""),
                    {6, label.rows - 8}, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);
        publish_debug(view, msg->header);
    }
}

// 공사 구간 진입/이탈 (표지판 또는 /mission/construction). brick_seen: 이번 프레임에 벽돌 탐지가 있는지
void PathNode::update_construction(const tb_interfaces::msg::LaneMask &msg, bool brick_seen)
{
    const rclcpp::Time stamp(msg.header.stamp);
    const bool det_fresh = last_det_ &&
        std::abs((stamp - rclcpp::Time(last_det_->header.stamp)).seconds()) * 1000.0 <= det_max_age_ms_;

    // 표지판으로 자동 진입 (연속 con_sign_frames 프레임)
    bool sign = false;
    if (det_fresh)
        for (const auto &d : last_det_->detections)
            if (d.label == "construction")
                sign = true;
    con_sign_seen_ = sign ? con_sign_seen_ + 1 : 0;
    if (sign)
        con_sign_time_ = stamp;
    if (con_done_ && !con_manual_ && (stamp - con_sign_time_).seconds() > 3.0)
        con_done_ = false; // 끝난 뒤 표지판이 3초 넘게 안 보이면 다음 진입 허용
    if (!con_auto_ && !con_done_ && con_sign_seen_ >= con_sign_frames_)
    {
        con_auto_ = true;
        RCLCPP_INFO(get_logger(), "construction: 표지판으로 진입");
    }

    if (con_on() != con_active_)
    {
        con_active_ = con_on();
        con_brick_seen_ = false;
        RCLCPP_INFO(get_logger(), "construction: %s", con_active_ ? "시작" : "끝");
    }
    if (!con_active_)
        return;

    // 벽돌을 본 뒤 con_exit_mm 가는 동안 벽돌이 안 보이면 끝 (자동 진입일 때만. 수동은 끌 때까지)
    if (brick_seen)
    {
        con_brick_seen_ = true;
        con_brick_pose_ = pose_;
    }
    else if (con_brick_seen_ && con_auto_ && !con_manual_ && con_exit_mm_ > 0 &&
             std::hypot(pose_.x - con_brick_pose_.x, pose_.y - con_brick_pose_.y) * 1000.0 > con_exit_mm_)
    {
        con_auto_ = false;
        con_done_ = true;
        con_active_ = false;
        RCLCPP_INFO(get_logger(), "construction: 벽돌이 %dmm 동안 안 보임, 일반 주행으로", con_exit_mm_);
    }
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
