// tb_signs: 카메라 영상 → YOLO(TensorRT) → /signs/detections
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "tb_interfaces/msg/detection_array.hpp"
#include "tb_signs/trt_engine.hpp"
#include "tb_signs/yolo.hpp"

class SignsNode : public rclcpp::Node
{
public:
    SignsNode() : Node("signs_node")
    {
        // 클래스 순서는 학습 때 data.yaml 의 names 순서와 같아야 함
        declare_parameter("engine_path", std::string(""));
        declare_parameter("class_names", std::vector<std::string>{
            "right", "left", "parking", "construction", "barrier_open", "barrier_closed", "brick"});
        declare_parameter("conf_thres", 0.5);
        declare_parameter("iou_thres", 0.45);
        declare_parameter("image_topic", std::string("/camera/image_raw"));

        names_ = get_parameter("class_names").as_string_array();
        const std::string path = get_parameter("engine_path").as_string();

        std::string err;
        if (!engine_.load(path, err))
            throw std::runtime_error("TensorRT 엔진 로드 실패: " + err);

        const auto &od = engine_.out_dims();
        if (od.size() != 3)
            throw std::runtime_error("출력이 [1, A, B] 3차원이 아님");
        out_d1_ = static_cast<int>(od[1]);
        out_d2_ = static_cast<int>(od[2]);
        const int nc = static_cast<int>(names_.size());
        if (out_d1_ != 4 + nc && out_d2_ != 4 + nc)
            throw std::runtime_error("출력 크기(" + std::to_string(out_d1_) + "x" + std::to_string(out_d2_) +
                                     ")가 클래스 수 " + std::to_string(nc) + "와 안 맞음 → class_names 확인");

        RCLCPP_INFO(get_logger(), "engine: %s  input %dx%d  output [%d, %d]  classes %d",
                    path.c_str(), engine_.in_w(), engine_.in_h(), out_d1_, out_d2_, nc);

        det_pub_ = create_publisher<tb_interfaces::msg::DetectionArray>("/signs/detections", 10);
        debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/signs/debug/compressed", 10);

        // 항상 최신 프레임만 처리 (추론이 카메라보다 느려도 밀리지 않게)
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            get_parameter("image_topic").as_string(), rclcpp::QoS(1).best_effort(),
            std::bind(&SignsNode::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        }
        catch (const cv_bridge::Exception &e)
        {
            t std::string share = ament_index_cpp::get_package_share_directory("tb_signs");
            const auto engine = declare_parameter<std::string>("engine_path", share + "/models/sign.engine");
            const auto names = declare_parameter<std::string>("names_path", share + "/models/data.yaml");
            conf_ = declare_parameter<double>("conf", 0.5);
            nms_ = declare_parameter<double>("nms", 0.5);
            debug_ = declare_parameter<bool>("publish_debug", true);

            RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
            return;
        }
        const cv::Mat &frame = cv_ptr->image;
        if (frame.empty())
            return;

        const auto t0 = std::chrono::steady_clock::now();
        const yolo::Letterbox lb = yolo::preprocess(frame, engine_.in_w(), engine_.in_h(), engine_.host_input());
        if (!engine_.infer())
        {
            RCLCPP_ERROR(get_logger(), "추론 실패");
            return;
        }
        const float conf = static_cast<float>(get_parameter("conf_thres").as_double());
        const float iou = static_cast<float>(get_parameter("iou_thres").as_double());
        auto dets = yolo::decode(engine_.host_output(), out_d1_, out_d2_, static_cast<int>(names_.size()),
                                 conf, lb, frame.size());
        dets = yolo::nms(std::move(dets), iou);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        avg_ms_ = avg_ms_ * 0.9 + ms * 0.1;

        tb_interfaces::msg::DetectionArray out;
        out.header = msg->header; // 원본 영상 stamp 그대로 (tb_path가 시간 맞출 때 사용)
        for (const auto &d : dets)
        {
            tb_interfaces::msg::Detection m;
            m.class_id = d.cls;
            m.label = names_[d.cls];
            m.score = d.score;
            m.x = static_cast<int>(d.box.x);
            m.y = static_cast<int>(d.box.y);
            m.w = static_cast<int>(d.box.width);
            m.h = static_cast<int>(d.box.height);
            out.detections.push_back(m);
        }
        det_pub_->publish(out);

        if (debug_pub_->get_subscription_count() > 0)
        {
            cv::Mat view = frame.clone();
            for (const auto &d : dets)
            {
                const cv::Scalar col = color(d.cls);
                cv::rectangle(view, d.box, col, 2);
                char txt[64];
                std::snprintf(txt, sizeof txt, "%s %.2f", names_[d.cls].c_str(), d.score);
                cv::putText(view, txt, cv::Point(int(d.box.x), std::max(12, int(d.box.y) - 4)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv::LINE_AA);
            }
            char fps[32];
            std::snprintf(fps, sizeof fps, "%.1f ms", avg_ms_);
            cv::putText(view, fps, {6, 18}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

            sensor_msgs::msg::CompressedImage img;
            img.header = msg->header;
            img.format = "bgr8; jpeg compressed bgr8";
            cv::imencode(".jpg", view, img.data, {cv::IMWRITE_JPEG_QUALITY, 70});
            debug_pub_->publish(img);
        }
    }

    // 라벨 스튜디오 색과 같은 순서 (BGR)
    static cv::Scalar color(int c)
    {
        static const cv::Scalar t[] = {{180, 119, 31}, {14, 127, 255}, {44, 160, 44}, {40, 39, 214},
                                       {189, 103, 148}, {75, 86, 140}, {194, 119, 227}};
        return t[c % 7];
    }

    TrtEngine engine_;
    std::vector<std::string> names_;
    int out_d1_ = 0, out_d2_ = 0;
    double avg_ms_ = 0;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<tb_interfaces::msg::DetectionArray>::SharedPtr det_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<SignsNode>());
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "[signs_node] %s\n", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
