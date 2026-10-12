// tb_brain: /signs/detections (YOLO 표지판, class_id) → 미션 토픽 발행
//
//   class_id            동작
//   ------------------  ------------------------------------------------------
//   1 barrier_closed    pause = 1
//   2 barrier_open      pause = 0
//   3 brick             (구현 안 함)
//   4 construction      /mission/construction = 1
//   6 parking           /mission/construction = 0, 이어서 pause = 1
//   5 left              /mission/turn = 1 을 turn_duration_sec 동안 발행 후 0
//   7 right             /mission/turn = 2 를 turn_duration_sec 동안 발행 후 0
//
// 같은 표지판이 연속 프레임에서 계속 검출되므로 "처음 보였을 때" 한 번만 동작한다.
// (rearm_sec 이상 안 보이다가 다시 보이면 새 표지판으로 취급)
#include <chrono>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "tb_interfaces/msg/detection_array.hpp"

using tb_interfaces::msg::DetectionArray;

// ====== 토픽 이름
static constexpr const char *TOPIC_DETECTIONS = "/signs/detections";     // 구독 (DetectionArray)
static constexpr const char *TOPIC_TURN = "/mission/turn";               // 발행 (UInt8: 0 직진, 1 좌, 2 우)
static constexpr const char *TOPIC_CONSTRUCTION = "/mission/construction"; // 발행 (Bool)
static constexpr const char *TOPIC_PAUSE = "/mission/pause";             // 발행 (Bool)

// ====== 표지판 class_id
static constexpr int ID_BARRIER_CLOSED = 1;
static constexpr int ID_BARRIER_OPEN = 2;
static constexpr int ID_BRICK = 3;
static constexpr int ID_CONSTRUCTION = 4;
static constexpr int ID_LEFT = 5;
static constexpr int ID_PARKING = 6;
static constexpr int ID_RIGHT = 7;

// ====== /mission/turn 값
static constexpr uint8_t TURN_NONE = 0;
static constexpr uint8_t TURN_LEFT = 1;
static constexpr uint8_t TURN_RIGHT = 2;

class BrainNode : public rclcpp::Node
{
public:
    BrainNode() : Node("brain_node")
    {
        // 기본값은 코드에 있음. 바꾸려면 -p turn_duration_sec:=15.0 또는 ros2 param set
        turn_duration_sec_ = declare_parameter<double>("turn_duration_sec", 10.0);
        rearm_sec_ = declare_parameter<double>("rearm_sec", 2.0);

        turn_pub_ = create_publisher<std_msgs::msg::UInt8>(TOPIC_TURN, 10);
        construction_pub_ = create_publisher<std_msgs::msg::Bool>(TOPIC_CONSTRUCTION, 10);
        pause_pub_ = create_publisher<std_msgs::msg::Bool>(TOPIC_PAUSE, 10);

        det_sub_ = create_subscription<DetectionArray>(
            TOPIC_DETECTIONS, 10,
            [this](const DetectionArray::ConstSharedPtr m) { on_detections(*m); });

        // turn 유지/종료 (10Hz)
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { on_timer(); });

        RCLCPP_INFO(get_logger(), "brain_node started (turn %.1fs)", turn_duration_sec_);
    }

private:
    void on_detections(const DetectionArray &msg)
    {
        const rclcpp::Time now = this->now();
        rearm_sec_ = get_parameter("rearm_sec").as_double();

        for (const auto &d : msg.detections)
        {
            auto it = last_seen_.find(d.class_id);
            const bool fresh = (it == last_seen_.end()) || (now - it->second).seconds() > rearm_sec_;
            last_seen_[d.class_id] = now;
            if (fresh)
                handle(d.class_id, now);
        }
    }

    void handle(int id, const rclcpp::Time &now)
    {
        switch (id)
        {
        case ID_BARRIER_CLOSED:
            RCLCPP_INFO(get_logger(), "barrier_closed → pause 1");
            publish_bool(pause_pub_, true);
            break;
        case ID_BARRIER_OPEN:
            RCLCPP_INFO(get_logger(), "barrier_open → pause 0");
            publish_bool(pause_pub_, false);
            break;
        case ID_BRICK:
            break; // 구현 안 함
        case ID_CONSTRUCTION:
            RCLCPP_INFO(get_logger(), "construction → construction 1");
            publish_bool(construction_pub_, true);
            break;
        case ID_PARKING:
            RCLCPP_INFO(get_logger(), "parking → construction 0, pause 1");
            publish_bool(construction_pub_, false);
            publish_bool(pause_pub_, true);
            break;
        case ID_LEFT:
            start_turn(TURN_LEFT, now, "left");
            break;
        case ID_RIGHT:
            start_turn(TURN_RIGHT, now, "right");
            break;
        default:
            break;
        }
    }

    void start_turn(uint8_t value, const rclcpp::Time &now, const char *name)
    {
        if (turn_value_ != TURN_NONE) // 이미 회전 신호 중이면 무시
            return;
        turn_duration_sec_ = get_parameter("turn_duration_sec").as_double();
        turn_value_ = value;
        turn_end_ = now + rclcpp::Duration::from_seconds(turn_duration_sec_);
        RCLCPP_INFO(get_logger(), "%s → turn %u for %.1fs", name, value, turn_duration_sec_);
        publish_turn(value);
    }

    void on_timer()
    {
        if (turn_value_ == TURN_NONE)
            return;
        if (now() >= turn_end_)
        {
            RCLCPP_INFO(get_logger(), "turn end → 0");
            turn_value_ = TURN_NONE;
            publish_turn(TURN_NONE);
        }
        else
        {
            publish_turn(turn_value_);
        }
    }

    void publish_turn(uint8_t v)
    {
        std_msgs::msg::UInt8 m;
        m.data = v;
        turn_pub_->publish(m);
    }

    static void publish_bool(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &pub, bool v)
    {
        std_msgs::msg::Bool m;
        m.data = v;
        pub->publish(m);
    }

    double turn_duration_sec_ = 10.0; // left/right 발행 시간 [s]
    double rearm_sec_ = 2.0;

    uint8_t turn_value_ = TURN_NONE;
    rclcpp::Time turn_end_{0, 0, RCL_ROS_TIME};
    std::map<int, rclcpp::Time> last_seen_; // class_id → 마지막으로 본 시각

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr turn_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr construction_pub_, pause_pub_;
    rclcpp::Subscription<DetectionArray>::SharedPtr det_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrainNode>());
    rclcpp::shutdown();
    return 0;
}