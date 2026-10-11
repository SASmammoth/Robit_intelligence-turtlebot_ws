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
    // 공사 구간 (실측: 차폭 56cm, 벽돌 20x8cm 선에 붙어 좌우 번갈아, 빈 공간 40cm)
    declare_int("path.con_lane_w_mm", 560, 1500);   // 공사 구간 안쪽 차폭
    declare_int("path.con_brick_w_mm", 200, 600);   // 벽돌이 선에서 안쪽으로 나온 길이
    declare_int("path.con_brick_d_mm", 80, 300);    // 벽돌 앞뒤 두께
    declare_int("path.con_pass_mm", 100, 300);      // 벽돌 옆을 지날 때 차선 중심에서 반대쪽으로
    declare_int("path.con_trans_mm", 400, 1000);    // 반대쪽 틈으로 넘어가는 길이
    declare_int("path.con_spacing_mm", 480, 1500);  // 벽돌 중심 사이 기본 간격
    declare_int("path.con_exit_mm", 500, 2000);     // 마지막 벽돌을 지나 이만큼 가면 공사 구간 끝
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
    auto &cp = con_.p;
    con_lane_w_mm_ = get_parameter("path.con_lane_w_mm").as_int();
    cp.lane_w = con_lane_w_mm_ / 1000.0;
    cp.brick_w = get_parameter("path.con_brick_w_mm").as_int() / 1000.0;
    cp.brick_d = get_parameter("path.con_brick_d_mm").as_int() / 1000.0;
    cp.pass_off = get_parameter("path.con_pass_mm").as_int() / 1000.0;
    cp.trans_len = get_parameter("path.con_trans_mm").as_int() / 1000.0;
    cp.spacing = get_parameter("path.con_spacing_mm").as_int() / 1000.0;
    cp.exit_after = get_parameter("path.con_exit_mm").as_int() / 1000.0;
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

    // ====== 공사 구간: 진입/이탈, 벽돌 관측, 차선 폭 덮어쓰기
    update_construction(*msg, h);
    if (con_on())
    {
        geo_.lane_w_px = std::max(8, int(std::lround(con_lane_w_mm_ / 1000.0 / geo_.m_per_px_x)));
        skel_p_.lane_w = geo_.lane_w_px;
    }

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
    if (!bricks.empty() && !con_on())
    {
        const cv::Mat H(3, 3, CV_64F, h.data());
        obstacles::draw_boxes(wall, H, bricks,
                              brick_depth_mm_ / 1000.0 / geo_.m_per_px_y,
                              brick_margin_mm_ / 1000.0 / geo_.m_per_px_x);
    }

    // ====== 스켈레톤 → 경로
    // 시드 힌트: 기억 경로의 연장선(움직임 반영) > 직전 시드 > 하단 중앙
    // 기억 경로로 예측한 점이 있으면 시드·시작점 모두 그 근처에서 고름
    cv::Point seed_hint = last_seed_, predicted;
    const bool has_pred = tracker_.predict_seed(pose_, geo_, predicted);
    if (has_pred)
        seed_hint = predicted;
    const lane_skel::Result skel = lane_skel::build(wall, valid, skel_p_, &seed_hint,
                                                    has_pred ? &predicted : nullptr);
    last_seed_ = skel.ok ? skel.seed : cv::Point(-1, -1);

    const auto turn = static_cast<lane_path::Turn>(std::min<uint8_t>(turn_, 2));
    lane_path::Output out = tracker_.update(skel, skel_p_.scale, white, yellow, valid, geo_, turn, pose_);

    // 공사 구간: 벽돌을 하나라도 봤으면 지도로 만든 지그재그 경로, 아직이면 스켈레톤으로 차선 기준만 맞춤
    bool con_path = false;
    if (con_.active())
    {
        std::vector<cv::Point2d> cp;
        if (con_.path(pose_, cp))
        {
            out.path = cp;
            con_path = true;
        }
        else if (out.src == lane_path::Source::Live)
            con_.lane_hint(out.path, pose_);
    }
    last_out_path_ = out.path;

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
    src.data = con_path ? "construction" : lane_path::source_name(out.src);
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
        view.setTo(cv::Scalar(235, 235, 235), white);
        view.setTo(cv::Scalar(0, 210, 255), yellow);
        if (skel.ok)
        {
            std::vector<cv::Point> pts;
            cv::findNonZero(skel.skel, pts);
            for (const auto &p : pts)
                cv::circle(view, p * skel_p_.scale, 1, cv::Scalar(200, 255, 100), -1);
        }
        if (con_path)
            draw_construction(view);
        else
            lane_path::draw_path(view, out, geo_);
        static const char *turn_txt[] = {"straight", "left", "right"};
        cv::putText(view, std::string(turn_txt[static_cast<int>(turn)]) + (parking_ ? " +park" : "") +
                              (con_on() ? " +con" : ""),
                    {6, label.rows - 8}, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);
        publish_debug(view, msg->header);
    }
}

// 공사 구간 진입/이탈과 벽돌 관측
void PathNode::update_construction(const tb_interfaces::msg::LaneMask &msg, const std::array<double, 9> &h)
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

    // 켜짐/꺼짐에 맞춰 계획기 시작·정리
    if (con_on() && !con_.active())
    {
        // 차선 방향: 직전 경로의 앞쪽 현, 없으면 로봇 방향
        double heading = pose_.th;
        cv::Point2d point(pose_.x, pose_.y);
        std::vector<cv::Point2d> seg;
        for (const auto &q : last_out_path_)
            if (q.x > 0.15 && q.x < 0.6)
                seg.push_back(q);
        if (seg.size() >= 5 && cv::norm(seg.back() - seg.front()) > 0.15)
        {
            const cv::Point2d d = seg.back() - seg.front(), m = (seg.front() + seg.back()) * 0.5;
            const double c = std::cos(pose_.th), sn = std::sin(pose_.th);
            heading = pose_.th + std::atan2(d.y, d.x);
            point = {pose_.x + c * m.x - sn * m.y, pose_.y + sn * m.x + c * m.y};
        }
        con_.start(heading, point);
        tracker_.reset(); // 차선 폭이 바뀌므로 기억 초기화
        RCLCPP_INFO(get_logger(), "construction: 시작 (차선 방향 %.0f°)", heading * 180 / M_PI);
    }
    else if (!con_on() && con_.active())
    {
        con_.reset();
        tracker_.reset();
        RCLCPP_INFO(get_logger(), "construction: 끝");
    }
    if (!con_.active())
        return;

    // 벽돌 관측: 박스 아랫변 가운데 = 로봇 쪽 앞면이 바닥에 닿은 곳 (같은 탐지 메시지는 한 번만)
    if (det_fresh && last_det_->header.stamp != con_last_det_stamp_)
    {
        con_last_det_stamp_ = last_det_->header.stamp;
        std::array<double, 9> hh = h;
        const cv::Mat H(3, 3, CV_64F, hh.data());
        for (const auto &d : last_det_->detections)
        {
            if (d.label != "brick")
                continue;
            std::vector<cv::Point2f> src{{float(d.x + d.w / 2.0), float(d.y + d.h)}}, dst;
            cv::perspectiveTransform(src, dst, H);
            con_.observe(lane_path::bev_to_robot(cv::Point2d(dst[0].x, dst[0].y), geo_), pose_);
        }
    }

    // 마지막 벽돌을 지나 con_exit만큼 가면 끝 (수동이면 다시 끌 때까지 일반 주행)
    if (con_.finished(pose_))
    {
        con_auto_ = false;
        con_done_ = true;
        con_.reset();
        tracker_.reset();
        RCLCPP_INFO(get_logger(), "construction: 마지막 벽돌 통과, 일반 주행으로");
    }
}

// 디버그: 벽돌 지도(청록)와 계획 경로(분홍)
void PathNode::draw_construction(cv::Mat &view) const
{
    const double c = std::cos(pose_.th), sn = std::sin(pose_.th);
    auto to_px = [&](const cv::Point2d &w)
    {
        const double dx = w.x - pose_.x, dy = w.y - pose_.y;
        const cv::Point2d b = lane_path::robot_to_bev({c * dx + sn * dy, -sn * dx + c * dy}, geo_);
        return cv::Point(cvRound(b.x), cvRound(b.y));
    };
    const double h = con_.heading(), bw = con_.p.brick_w / 2, bd = con_.p.brick_d / 2;
    const cv::Point2d t(std::cos(h), std::sin(h)), n(-std::sin(h), std::cos(h));
    for (const auto &b : con_.bricks())
    {
        const std::vector<cv::Point> poly = {to_px(b.c + t * bd + n * bw), to_px(b.c + t * bd - n * bw),
                                             to_px(b.c - t * bd - n * bw), to_px(b.c - t * bd + n * bw)};
        cv::fillConvexPoly(view, poly, cv::Scalar(200, 220, 0));
    }
    std::vector<cv::Point2d> path;
    if (con_.path(pose_, path) && path.size() >= 2)
    {
        std::vector<cv::Point> px;
        for (const auto &q : path)
        {
            const cv::Point2d b = lane_path::robot_to_bev(q, geo_);
            px.emplace_back(cvRound(b.x), cvRound(b.y));
        }
        cv::polylines(view, px, false, cv::Scalar(180, 80, 255), 2, cv::LINE_AA);
    }
    cv::putText(view, "construction", {6, 18}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 80, 255), 1, cv::LINE_AA);
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
