#include "tb_line/lineDetect_node.hpp"

#include <algorithm>

// 작은 덩어리 제거 + 내부 구멍 메움
static void clean_mask(cv::Mat &mask, int min_area)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    mask.setTo(0);
    for (size_t i = 0; i < contours.size(); ++i)
    {
        if (cv::contourArea(contours[i]) < min_area)
            continue;
        cv::drawContours(mask, contours, static_cast<int>(i), 255, cv::FILLED);
    }
}

LineDetectNode::LineDetectNode() : Node("lineDetect_node")
{
    // ====== 구독 / 발행
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&LineDetectNode::image_callback, this, std::placeholders::_1));

    mask_pub_ = create_publisher<tb_interfaces::msg::LaneMask>("/lane/mask", 10);

    debug_mask_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/lane/debug/mask/compressed", 10);
    debug_bev_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/lane/debug/bev/compressed", 10);
    debug_roi_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/lane/debug/roi/compressed", 10);

    // ====== HSV
    hsv_ranges_[idx(Target::WhiteLine)] = {cv::Scalar(75, 0, 200), cv::Scalar(179, 160, 255)};
    hsv_ranges_[idx(Target::YellowLine)] = {cv::Scalar(35, 60, 60), cv::Scalar(60, 255, 255)};

    for (size_t i = 0; i < idx(Target::COUNT); ++i)
    {
        const std::string prefix = target_name(static_cast<Target>(i));
        const HsvRange &d = hsv_ranges_[i];
        declare_int(prefix + ".h_min", static_cast<int>(d.lower[0]), 179);
        declare_int(prefix + ".h_max", static_cast<int>(d.upper[0]), 179);
        declare_int(prefix + ".s_min", static_cast<int>(d.lower[1]), 255);
        declare_int(prefix + ".s_max", static_cast<int>(d.upper[1]), 255);
        declare_int(prefix + ".v_min", static_cast<int>(d.lower[2]), 255);
        declare_int(prefix + ".v_max", static_cast<int>(d.upper[2]), 255);
    }

    // ====== 노이즈 필터 (커널은 홀수만)
    auto declare_odd = [this](const std::string &name, int def, int max)
    {
        rcl_interfaces::msg::ParameterDescriptor d;
        rcl_interfaces::msg::IntegerRange r;
        r.from_value = 1;
        r.to_value = max;
        r.step = 2;
        d.integer_range.push_back(r);
        declare_parameter(name, def, d);
    };
    declare_odd("morph.open", 3, 15);
    declare_odd("morph.close", 9, 31);
    declare_int("morph.min_area", 150, 5000);

    // ====== BEV
    declare_int("bev.top_y", 40, 100);
    declare_int("bev.bot_y", 100, 100);
    declare_int("bev.top_w", 32, 100);
    declare_int("bev.bot_w", 100, 100);
    declare_int("bev.lane_w_px", 160, 320); // BEV에서 차선 안쪽 간격 (직선 구간에서 실측)

    // ====== 거리 환산 (실측)
    declare_int("geo.lane_w_mm", 260, 1000);
    declare_int("geo.bev_depth_mm", 520, 3000);
    declare_int("geo.bev_bottom_mm", 150, 2000);

    // ====== 선언이 전부 끝난 뒤에 읽기
    reload_params();
    post_cb_handle_ = add_post_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &)
        { reload_params(); });

    RCLCPP_INFO(get_logger(), "lineDetect_node started");
}

void LineDetectNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
        return;
    }

    const cv::Mat &frame = cv_ptr->image;
    if (frame.empty())
        return;

    cv::Mat blur;
    cv::GaussianBlur(frame, blur, cv::Size(5, 5), 0);

    if (bev_dirty_ || frame.size() != bev_src_size_)
        update_bev_matrix(frame.size());

    cv::Mat bev;
    cv::warpPerspective(blur, bev, bev_M_, cv::Size(bev_out_, bev_out_), cv::INTER_LINEAR);

    cv::Mat hsv;
    cv::cvtColor(bev, hsv, cv::COLOR_BGR2HSV);

    cv::Mat white_bin = make_mask(hsv, Target::WhiteLine);
    cv::Mat yellow_bin = make_mask(hsv, Target::YellowLine);

    // 사각지대 경계의 색 번짐 제거
    cv::bitwise_and(white_bin, bev_valid_, white_bin);
    cv::bitwise_and(yellow_bin, bev_valid_, yellow_bin);

    // ====== 라벨 영상: 0 없음, 1 흰, 2 노랑, 255 사각지대
    cv::Mat label(bev.size(), CV_8U, cv::Scalar(LABEL_NONE));
    label.setTo(LABEL_WHITE, white_bin);
    label.setTo(LABEL_YELLOW, yellow_bin); // 겹치면 노란색 우선
    label.setTo(LABEL_BLIND, bev_valid_ == 0);

    tb_interfaces::msg::LaneMask out;
    out.header = msg->header;
    out.label = *cv_bridge::CvImage(msg->header, "mono8", label).toImageMsg();
    out.m_per_px_x = static_cast<float>(lane_w_mm_ / 1000.0 / std::max(1, lane_w_px_));
    out.m_per_px_y = static_cast<float>(bev_depth_mm_ / 1000.0 / bev_out_);
    out.bottom_offset_m = static_cast<float>(bev_bottom_mm_ / 1000.0);
    out.lane_w_px = lane_w_px_;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out.homography[r * 3 + c] = bev_M_.at<double>(r, c);
    mask_pub_->publish(out);

    // ====== 디버그
    if (debug_roi_pub_->get_subscription_count() > 0)
    {
        cv::Mat roi_view = frame.clone();
        const float w = frame.cols, h = frame.rows, cx = w / 2.0f;
        std::vector<cv::Point> quad = {
            {int(cx - w * bev_top_w_ / 200.0f), int(h * bev_top_y_ / 100.0f)},
            {int(cx + w * bev_top_w_ / 200.0f), int(h * bev_top_y_ / 100.0f)},
            {int(cx + w * bev_bot_w_ / 200.0f), int(h * bev_bot_y_ / 100.0f) - 1},
            {int(cx - w * bev_bot_w_ / 200.0f), int(h * bev_bot_y_ / 100.0f) - 1}};
        cv::polylines(roi_view, quad, true, cv::Scalar(0, 0, 255), 2);
        publish_debug(debug_roi_pub_, roi_view, msg->header);
    }
    if (debug_bev_pub_->get_subscription_count() > 0)
    {
        // 차선 간격 확인용 세로선 (가운데 기준 ±lane_w_px/2)
        cv::Mat view = bev.clone();
        const int c = bev_out_ / 2, hw = lane_w_px_ / 2;
        cv::line(view, {c - hw, 0}, {c - hw, bev_out_ - 1}, cv::Scalar(255, 0, 255), 1);
        cv::line(view, {c + hw, 0}, {c + hw, bev_out_ - 1}, cv::Scalar(255, 0, 255), 1);
        publish_debug(debug_bev_pub_, view, msg->header);
    }
    if (debug_mask_pub_->get_subscription_count() > 0)
    {
        // 흰선은 흰색, 노란선은 노란색, 사각지대는 진회색으로 한 장에
        cv::Mat view(bev.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        view.setTo(cv::Scalar(60, 60, 60), bev_valid_ == 0);
        view.setTo(cv::Scalar(255, 255, 255), white_bin);
        view.setTo(cv::Scalar(0, 220, 255), yellow_bin);
        publish_debug(debug_mask_pub_, view, msg->header);
    }
}

void LineDetectNode::declare_int(const std::string &name, int def, int max)
{
    rcl_interfaces::msg::ParameterDescriptor d;
    rcl_interfaces::msg::IntegerRange r;
    r.from_value = 0;
    r.to_value = max;
    r.step = 1;
    d.integer_range.push_back(r);
    declare_parameter(name, def, d);
}

cv::Mat LineDetectNode::make_mask(const cv::Mat &hsv, Target t) const
{
    const HsvRange &r = hsv_ranges_[idx(t)];

    cv::Mat mask;
    cv::inRange(hsv, r.lower, r.upper, mask);

    const cv::Mat k_open = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_open_, morph_open_));
    const cv::Mat k_close = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(morph_close_, morph_close_));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k_open);   // 1~2픽셀 잡티
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, k_close); // 구멍 메우기

    clean_mask(mask, min_blob_area_);
    return mask;
}

void LineDetectNode::publish_debug(
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr &pub,
    const cv::Mat &img, const std_msgs::msg::Header &header)
{
    if (pub->get_subscription_count() == 0)
        return;

    sensor_msgs::msg::CompressedImage out;
    out.header = header;
    if (img.channels() == 1)
    {
        out.format = "mono8; png compressed mono8";
        cv::imencode(".png", img, out.data, {cv::IMWRITE_PNG_COMPRESSION, 1});
    }
    else
    {
        out.format = "bgr8; jpeg compressed bgr8";
        cv::imencode(".jpg", img, out.data, {cv::IMWRITE_JPEG_QUALITY, 70});
    }
    pub->publish(out);
}

void LineDetectNode::update_bev_matrix(const cv::Size &sz)
{
    const float w = sz.width, h = sz.height, cx = w / 2.0f;
    const float ty = h * bev_top_y_ / 100.0f;
    const float by = h * bev_bot_y_ / 100.0f - 1;
    const float tw = w * bev_top_w_ / 200.0f;
    const float bw = w * bev_bot_w_ / 200.0f;

    const std::vector<cv::Point2f> src = {{cx - tw, ty}, {cx + tw, ty}, {cx + bw, by}, {cx - bw, by}};

    // 좌우에 여유를 둬서 곡선이 바깥으로 나가도 보이게
    const float S = bev_out_, m = S * 0.25f;
    const std::vector<cv::Point2f> dst = {{m, 0}, {S - m, 0}, {S - m, S}, {m, S}};

    bev_M_ = cv::getPerspectiveTransform(src, dst);
    bev_src_size_ = sz;
    bev_dirty_ = false;

    // 원본 전체를 펴서 보이는 영역 마스크 만들기 (경계 번짐은 조금 깎음)
    const cv::Mat ones(sz, CV_8U, cv::Scalar(255));
    cv::warpPerspective(ones, bev_valid_, bev_M_, cv::Size(bev_out_, bev_out_),
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    cv::erode(bev_valid_, bev_valid_, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
}

void LineDetectNode::reload_params()
{
    for (size_t i = 0; i < idx(Target::COUNT); ++i)
    {
        const std::string prefix = target_name(static_cast<Target>(i));
        const int h_min = get_parameter(prefix + ".h_min").as_int();
        const int h_max = get_parameter(prefix + ".h_max").as_int();
        const int s_min = get_parameter(prefix + ".s_min").as_int();
        const int s_max = get_parameter(prefix + ".s_max").as_int();
        const int v_min = get_parameter(prefix + ".v_min").as_int();
        const int v_max = get_parameter(prefix + ".v_max").as_int();
        hsv_ranges_[i].lower = cv::Scalar(h_min, s_min, v_min);
        hsv_ranges_[i].upper = cv::Scalar(h_max, s_max, v_max);
    }

    morph_open_ = get_parameter("morph.open").as_int();
    morph_close_ = get_parameter("morph.close").as_int();
    min_blob_area_ = get_parameter("morph.min_area").as_int();

    const int top_y = get_parameter("bev.top_y").as_int();
    const int bot_y = get_parameter("bev.bot_y").as_int();
    const int top_w = get_parameter("bev.top_w").as_int();
    const int bot_w = get_parameter("bev.bot_w").as_int();
    if (top_y != bev_top_y_ || bot_y != bev_bot_y_ || top_w != bev_top_w_ || bot_w != bev_bot_w_)
    {
        bev_top_y_ = top_y;
        bev_bot_y_ = bot_y;
        bev_top_w_ = top_w;
        bev_bot_w_ = bot_w;
        bev_dirty_ = true; // BEV 값이 바뀐 경우에만 행렬 다시 계산
    }

    lane_w_px_ = std::max(8, static_cast<int>(get_parameter("bev.lane_w_px").as_int()));
    lane_w_mm_ = std::max(1, static_cast<int>(get_parameter("geo.lane_w_mm").as_int()));
    bev_depth_mm_ = std::max(1, static_cast<int>(get_parameter("geo.bev_depth_mm").as_int()));
    bev_bottom_mm_ = static_cast<int>(get_parameter("geo.bev_bottom_mm").as_int());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineDetectNode>());
    rclcpp::shutdown();
    return 0;
}