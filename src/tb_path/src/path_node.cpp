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

    // ====== 출력
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/lane/path", 10);
    src_pub_ = create_publisher<std_msgs::msg::String>("/lane/path_source", 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/lane/debug/skel/compressed", 10);

    // ====== 파라미터
    declare_int("skel.seal_len", 30, 100);
    declare_int("skel.prune_len", 20, 100);
    declare_int("skel.dash_len", 40, 160);
    declare_int("path.brick_depth_mm", 60, 500);
    declare_int("path.brick_margin_mm", 30, 300);
    declare_int("path.det_max_age_ms", 300, 2000);
    declare_int("path.mem_travel_mm", 600, 3000);

    reload_params();
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
    if (h != last_h_ || msg->lane_w_px != geo_.lane_w_px)
    {
        tracker_.reset();
        last_seed_ = {-1, -1};
        last_h_ = h;
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
    if (!bricks.empty())
    {
        const cv::Mat H(3, 3, CV_64F, h.data());
        obstacles::draw_boxes(wall, H, bricks,
                              brick_depth_mm_ / 1000.0 / geo_.m_per_px_y,
                              brick_margin_mm_ / 1000.0 / geo_.m_per_px_x);
    }

    // ====== 스켈레톤 → 경로
    const lane_skel::Result skel = lane_skel::build(wall, valid, skel_p_, &last_seed_);
    last_seed_ = skel.ok ? skel.seed : cv::Point(-1, -1);

    const auto turn = static_cast<lane_path::Turn>(std::min<uint8_t>(turn_, 2));
    const lane_path::Output out = tracker_.update(skel, skel_p_.scale, white, yellow, valid, geo_, turn, pose_);

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
        lane_path::draw_path(view, out, geo_);
        static const char *turn_txt[] = {"straight", "left", "right"};
        cv::putText(view, std::string(turn_txt[static_cast<int>(turn)]) + (parking_ ? " +park" : ""),
                    {6, label.rows - 8}, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);
        publish_debug(view, msg->header);
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
