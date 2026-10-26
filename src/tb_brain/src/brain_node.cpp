// tb_brain: /signs/detections (YOLO 표지판, class_id) → 미션 토픽 발행
//
//   class_id            동작
//   ------------------  ------------------------------------------------------
//   0 barrier_closed    pause = 1
//   1 barrier_open      pause = 0
//   2 brick             (구현 안 함)
//   3 construction      /mission/construction = 1
//   5 parking           /mission/construction = 0, 이어서 pause = 1
//   4 left              /mission/turn = 1 을 turn_duration_sec 동안 발행 후 0
//   6 right             /mission/turn = 2 를 turn_duration_sec 동안 발행 후 0
//   7 tl_green          /drive/cmd = FOLLOW 를 계속 발행 (stop 받을 때까지)
//   8 tl_off
//   9 tl_red
//   10 tl_yellow
//
// /brain/run 으로 "start"를 받아야 동작(run_state 1), "stop"을 받으면 멈춤(run_state 0).
// run_state 0 동안에는 검출을 무시하고 어떤 토픽도 발행하지 않는다.
//
// 같은 표지판이 연속 프레임에서 계속 검출되므로 "처음 보였을 때" 한 번만 동작한다.
// (rearm_sec 이상 안 보이다가 다시 보이면 새 표지판으로 취급)
#include <chrono>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "tb_interfaces/msg/detection_array.hpp"
#include "tb_interfaces/msg/drive_cmd.hpp"

using tb_interfaces::msg::DetectionArray;
using tb_interfaces::msg::DriveCmd;

// ====== 토픽 이름
static constexpr const char *TOPIC_DETECTIONS = "/signs/detections";     // 구독 (DetectionArray)
static constexpr const char *TOPIC_TURN = "/mission/turn";               // 발행 (UInt8: 0 직진, 1 좌, 2 우)
static constexpr const char *TOPIC_CONSTRUCTION = "/mission/construction"; // 발행 (Bool)
static constexpr const char *TOPIC_PAUSE = "/mission/pause";             // 발행 (Bool)
static constexpr const char *TOPIC_DRIVE_CMD = "/drive/cmd";             // 발행 (DriveCmd)
static constexpr const char *TOPIC_RUN = "/brain/run";                   // 구독 (String: "start" / "stop")

// ====== /brain/run 명령
static constexpr const char *RUN_START = "start";
static constexpr const char *RUN_STOP = "stop";

// ====== 표지판 class_id
static constexpr int ID_BARRIER_CLOSED = 0;
static constexpr int ID_BARRIER_OPEN = 1;
static constexpr int ID_BRICK = 2;
static constexpr int ID_CONSTRUCTION = 3;
static constexpr int ID_LEFT = 4;
static constexpr int ID_PARKING = 5;
static constexpr int ID_RIGHT = 6;
static constexpr int ID_TL_GREEN = 7;
static constexpr int ID_TL_OFF = 8;
static constexpr int ID_TL_RED = 9;
static constexpr int ID_TL_YELLOW = 10;

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
        drive_cmd_pub_ = create_publisher<DriveCmd>(TOPIC_DRIVE_CMD, 10);

        run_sub_ = create_subscription<std_msgs::msg::String>(
            TOPIC_RUN, 10,
            [this](const std_msgs::msg::String::ConstSharedPtr m) { on_run(m->data); });

        det_sub_ = create_subscription<DetectionArray>(
            TOPIC_DETECTIONS, 10,
            [this](const DetectionArray::ConstSharedPtr m) { on_detections(*m); });

        // turn 유지/종료, drive cmd 유지 (10Hz)
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { on_timer(); });

        RCLCPP_INFO(get_logger(), "brain_node started (turn %.1fs, run_state 0: '%s' on %s to run)",
                    turn_duration_sec_, RUN_START, TOPIC_RUN);
    }

private:
    void on_run(const std::string &cmd)
    {
        if (cmd == RUN_START)
        {
            if (run_state_ == 1)
                return;
            run_state_ = 1;
            RCLCPP_INFO(get_logger(), "run: start → run_state 1");
        }
        else if (cmd == RUN_STOP)
        {
            if (run_state_ == 0)
                return;
            run_state_ = 0;
            // 다시 start 했을 때 이전 상태가 이어지지 않게
            turn_value_ = TURN_NONE;
            follow_ = false;
            RCLCPP_INFO(get_logger(), "run: stop → run_state 0");
        }
        else
        {
            RCLCPP_WARN(get_logger(), "run: unknown command '%s' (use '%s' / '%s')", cmd.c_str(), RUN_START, RUN_STOP);
        }
    }

    void on_detections(const DetectionArray &msg)
    {
        if (run_state_ != 1)
            return;
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
        case ID_TL_GREEN:
            if (follow_)
                break;
            RCLCPP_INFO(get_logger(), "tl_green → drive FOLLOW");
            follow_ = true;
            publish_follow();
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
        if (run_state_ != 1)
            return;
        // tb_drive는 cmd_timeout_ms 동안 명령이 없으면 버리므로 계속 발행
        if (follow_)
            publish_follow();

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
        if (run_state_ != 1)
            return;
        std_msgs::msg::UInt8 m;
        m.data = v;
        turn_pub_->publish(m);
    }

    void publish_follow()
    {
        if (run_state_ != 1)
            return;
        DriveCmd m;
        m.header.stamp = now();
        m.mode = DriveCmd::FOLLOW;
        m.speed_limit = 0.0f; // drive.v_max 사용
        drive_cmd_pub_->publish(m);
    }

    void publish_bool(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &pub, bool v)
    {
        if (run_state_ != 1)
            return;
        std_msgs::msg::Bool m;
        m.data = v;
        pub->publish(m);
    }

    double turn_duration_sec_ = 10.0; // left/right 발행 시간 [s]
    double rearm_sec_ = 2.0;

    int run_state_ = 0;   // 0: 정지(발행 안 함), 1: 동작. /brain/run 으로 변경
    bool follow_ = false; // tl_green 본 뒤 FOLLOW 발행 중

    uint8_t turn_value_ = TURN_NONE;
    rclcpp::Time turn_end_{0, 0, RCL_ROS_TIME};
    std::map<int, rclcpp::Time> last_seen_; // class_id → 마지막으로 본 시각

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr turn_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr construction_pub_, pause_pub_;
    rclcpp::Publisher<DriveCmd>::SharedPtr drive_cmd_pub_;
    rclcpp::Subscription<DetectionArray>::SharedPtr det_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr run_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrainNode>());
    rclcpp::shutdown();
    return 0;
}