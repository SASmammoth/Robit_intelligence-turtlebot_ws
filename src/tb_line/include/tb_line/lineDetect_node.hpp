#pragma once
// tb_line: 카메라 영상 → BEV → 흰색/노란색 차선 마스크 → /lane/mask
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "tb_interfaces/msg/lane_mask.hpp"

class LineDetectNode : public rclcpp::Node
{
public:
    LineDetectNode();

    enum class Target
    {
        WhiteLine = 0,
        YellowLine,
        COUNT
    };

    static constexpr size_t idx(Target t) { return static_cast<size_t>(t); }

    static constexpr const char *target_name(Target t)
    {
        switch (t)
        {
        case Target::WhiteLine:
            return "white";
        case Target::YellowLine:
            return "yellow";
        default:
            return "";
        }
    }

    // 라벨 값 (tb_interfaces/LaneMask.label)
    static constexpr uint8_t LABEL_NONE = 0;
    static constexpr uint8_t LABEL_WHITE = 1;
    static constexpr uint8_t LABEL_YELLOW = 2;
    static constexpr uint8_t LABEL_BLIND = 255;

private:
    struct HsvRange
    {
        cv::Scalar lower;
        cv::Scalar upper;
    };

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void declare_int(const std::string &name, int def, int max); // 0~max 정수 파라미터
    void reload_params();
    void update_bev_matrix(const cv::Size &img_size);
    cv::Mat make_mask(const cv::Mat &hsv, Target t) const;
    void publish_debug(const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr &pub,
                       const cv::Mat &img, const std_msgs::msg::Header &header);

    // HSV
    std::array<HsvRange, static_cast<size_t>(Target::COUNT)> hsv_ranges_;

    // 모폴로지
    int morph_open_ = 3;
    int morph_close_ = 9;
    int min_blob_area_ = 150;

    // BEV
    int bev_top_y_ = -1, bev_bot_y_ = -1, bev_top_w_ = -1, bev_bot_w_ = -1; // -1: 아직 안 읽음
    int bev_out_ = 320;                                                     // BEV 출력 크기 (정사각형)
    cv::Mat bev_M_;                                                         // 원본 → BEV 변환 행렬 (CV_64F 3x3)
    cv::Mat bev_valid_;                                                     // BEV에서 보이는 영역 255, 사각지대 0
    cv::Size bev_src_size_;                                                 // 행렬을 만들 때의 원본 크기
    bool bev_dirty_ = true;                                                 // BEV 값이 바뀌면 행렬 재계산

    // 거리 환산 (BEV 보정값과 한 세트)
    int lane_w_px_ = 160;     // BEV에서 두 차선 안쪽 간격 [px]
    int lane_w_mm_ = 260;     // 실제 두 차선 안쪽 간격
    int bev_depth_mm_ = 520;  // BEV 맨 아래 ~ 맨 위 줄 실제 거리
    int bev_bottom_mm_ = 150; // 바퀴 축 ~ BEV 맨 아래 줄 거리

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<tb_interfaces::msg::LaneMask>::SharedPtr mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_mask_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_bev_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_roi_pub_;

    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_cb_handle_;
};