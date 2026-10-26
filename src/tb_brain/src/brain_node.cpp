// tb_brain: /signs/detections (YOLO 표지판, class_id) → 미션 토픽 발행
//
//   class_id            동작
//   ------------------  ------------------------------------------------------
//   0 barrier_closed    pause = 1  (mission.barrier_release_ms 동안 안 보이면 pause = 0)
//   1 barrier_open      pause = 0
//   2 brick             (구현 안 함)
//   3 construction      /mission/construction = 1  (끝나는 방법은 mission.con_end_mode)
//   5 parking           /mission/construction = 0  (mission.parking_pause = 1이면 pause = 1도)
//   4 left              /mission/turn = 1 을 mission.turn_ms 동안 발행 후 0
//   6 right             /mission/turn = 2 를 mission.turn_ms 동안 발행 후 0
//   7 tl_green          /drive/cmd = FOLLOW 를 계속 발행 (stop 받을 때까지)
//   8 tl_off
//   9 tl_red
//   10 tl_yellow
//
// /brain/run 으로 "start"를 받아야 동작(run_state 1), "stop"을 받으면 멈춤(run_state 0).
// run_state 0 동안에는 검출을 무시하고 어떤 토픽도 발행하지 않는다.
//
// 같은 표지판이 연속 프레임에서 계속 검출되므로 "처음 보였을 때" 한 번만 동작한다.
// (mission.rearm_ms 이상 안 보이다가 다시 보이면 새 표지판으로 취급)
//
// 검출 필터 (GUI Params 탭에서 조정, 0이면 끔):
//   sign.<label>.min_h_px / min_w_px  이보다 작은 박스(멀리 있는 표지판)는 무시
//   sign.<label>.min_area_px          박스 면적(w×h)이 이보다 작으면 무시 (차단바처럼 가로로 긴 것)
//   sign.min_score_pct               이보다 낮은 점수는 무시
//   sign.confirm_frames              필터를 통과한 검출이 이만큼 연속 프레임이어야 동작 (한 프레임 오검출 거르기)
//
// 공사 구간 (판단은 여기 한 곳에서만, path_node는 /mission/construction만 받음)
//   mission.con_end_mode  0: path_node가 벽돌 기준으로 끝냄 (path.con_exit_mm, con_no_brick_mm)
//                         1: 진입 후 mission.con_hold_ms 지나면 끝
//                         2: 주차 표지판을 볼 때까지 유지
//     1, 2를 쓸 때는 path.con_exit_mm, path.con_no_brick_mm를 0으로 (path_node가 먼저 끄지 않게)
//   mission.con_once = 1  한 번 들어간 뒤에는 주차 표지판을 볼 때까지 공사 표지판을 무시
//                         (공사 구간 출구 쪽에 보이는 표지판으로 다시 들어가지 않게)
//   주차 표지판은 모드와 상관없이 공사 구간을 끝냄
//
// 가짜 표지판 (/brain/inject, String: 표지판 이름): 필터·연속 프레임·rearm 없이 바로 그 표지판 동작
//   GUI "가짜 표지판" 버튼. 중간에서 brain을 시작할 때 tl_green으로 출발, 놓친 표지판 대신 등
//   run_state 1일 때만 받음
// mission.start_follow = 1: /brain/run start를 받자마자 FOLLOW (신호등 구간을 건너뛰고 중간에서 시작)
//
// 파라미터는 모두 정수 (GUI 스핀박스, Save 파일의 /brain_node 항목)
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <set>
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
static constexpr const char *TOPIC_INJECT = "/brain/inject";             // 구독 (String: 표지판 이름, 가짜 표지판)

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
        // 미션 동작 (정수: GUI 스핀박스로 바꿈. ros2 param set /brain_node mission.turn_ms 4000 도 가능)
        declare_int("mission.turn_ms", 4000, 0, 30000);         // 좌/우 표지판 → turn 유지 시간
        declare_int("mission.rearm_ms", 2000, 0, 30000);        // 같은 표지판을 이만큼 안 보다가 다시 보면 새 표지판
        declare_int("mission.parking_pause", 0, 0, 1);          // 1: 주차 표지판에서 pause도 (0: 공사 구간만 끔)
        declare_int("mission.barrier_release_ms", 0, 0, 30000); // 차단바(닫힘)가 이만큼 안 보이면 pause 해제 (0: barrier_open만)
        declare_int("mission.con_end_mode", 0, 0, 2);           // 0 path_node 벽돌 기준, 1 시간, 2 주차 표지판까지
        declare_int("mission.con_hold_ms", 10000, 0, 120000);   // con_end_mode 1: 진입 후 이만큼 지나면 끝
        declare_int("mission.con_once", 1, 0, 1);               // 1: 진입 후 주차 표지판 전까지 공사 표지판 무시
        declare_int("mission.start_follow", 0, 0, 1);           // 1: start 받자마자 FOLLOW (초록불 안 기다림)

        // 검출 필터
        declare_int("sign.confirm_frames", 2, 1, 30);
        declare_int("sign.min_score_pct", 0, 0, 100);
        for (const char *l : FILTER_LABELS)
        {
            declare_int(std::string("sign.") + l + ".min_h_px", 0, 0, 480);
            declare_int(std::string("sign.") + l + ".min_w_px", 0, 0, 640);
            declare_int(std::string("sign.") + l + ".min_area_px", 0, 0, 640 * 480);
        }
        reload_filter();
        post_cb_handle_ = add_post_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &) { reload_filter(); });

        turn_pub_ = create_publisher<std_msgs::msg::UInt8>(TOPIC_TURN, 10);
        construction_pub_ = create_publisher<std_msgs::msg::Bool>(TOPIC_CONSTRUCTION, 10);
        pause_pub_ = create_publisher<std_msgs::msg::Bool>(TOPIC_PAUSE, 10);
        drive_cmd_pub_ = create_publisher<DriveCmd>(TOPIC_DRIVE_CMD, 10);

        run_sub_ = create_subscription<std_msgs::msg::String>(
            TOPIC_RUN, 10,
            [this](const std_msgs::msg::String::ConstSharedPtr m) { on_run(m->data); });
        inject_sub_ = create_subscription<std_msgs::msg::String>(
            TOPIC_INJECT, 10,
            [this](const std_msgs::msg::String::ConstSharedPtr m) { on_inject(m->data); });

        det_sub_ = create_subscription<DetectionArray>(
            TOPIC_DETECTIONS, 10,
            [this](const DetectionArray::ConstSharedPtr m) { on_detections(*m); });

        // turn 유지/종료, drive cmd 유지 (10Hz)
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { on_timer(); });

        RCLCPP_INFO(get_logger(), "brain_node started (turn %dms, run_state 0: '%s' on %s to run)",
                    turn_ms_, RUN_START, TOPIC_RUN);
    }

private:
    void on_run(const std::string &cmd)
    {
        if (cmd == RUN_START)
        {
            if (run_state_ == 1)
                return;
            run_state_ = 1;
            RCLCPP_INFO(get_logger(), "run: start → run_state 1%s", start_follow_ ? " (start_follow: 바로 FOLLOW)" : "");
            if (start_follow_)
            {
                follow_ = true;
                publish_follow();
            }
        }
        else if (cmd == RUN_STOP)
        {
            if (run_state_ == 0)
                return;
            run_state_ = 0;
            // 다시 start 했을 때 이전 상태가 이어지지 않게
            turn_value_ = TURN_NONE;
            follow_ = false;
            streak_.clear();
            last_seen_.clear(); // 다시 켰을 때 앞에 보이는 표지판을 새로 봄
            con_on_ = false;
            con_latched_ = false;
            barrier_pause_ = false;
            RCLCPP_INFO(get_logger(), "run: stop → run_state 0");
        }
        else
        {
            RCLCPP_WARN(get_logger(), "run: unknown command '%s' (use '%s' / '%s')", cmd.c_str(), RUN_START, RUN_STOP);
        }
    }

    // 가짜 표지판: 이름 → class_id, 필터·rearm 없이 바로 동작
    void on_inject(const std::string &label)
    {
        static const std::map<std::string, int> ids = {
            {"barrier_closed", ID_BARRIER_CLOSED}, {"barrier_open", ID_BARRIER_OPEN}, {"construction", ID_CONSTRUCTION},
            {"left", ID_LEFT}, {"parking", ID_PARKING}, {"right", ID_RIGHT}, {"tl_green", ID_TL_GREEN}};
        if (run_state_ != 1)
        {
            RCLCPP_WARN(get_logger(), "inject '%s' 무시: brain 꺼져 있음 (/brain/run start 먼저)", label.c_str());
            return;
        }
        auto it = ids.find(label);
        if (it == ids.end())
        {
            RCLCPP_WARN(get_logger(), "inject: 모르는 표지판 '%s'", label.c_str());
            return;
        }
        const rclcpp::Time t = now();
        RCLCPP_INFO(get_logger(), "inject: %s", label.c_str());
        if (it->second == ID_BARRIER_CLOSED)
            barrier_last_ = t; // barrier_release_ms 동안은 유지
        if (it->second == ID_LEFT || it->second == ID_RIGHT)
            turn_value_ = TURN_NONE; // 회전 중이어도 새 방향으로
        last_seen_[it->second] = t; // 진짜 표지판이 바로 보여도 rearm 동안 중복 동작 안 함
        handle(it->second, t);
    }

    // 동작에 쓰는 표지판만 필터 파라미터를 둠 (brick, tl_off/red/yellow는 동작 없음)
    static constexpr const char *FILTER_LABELS[] = {"barrier_closed", "barrier_open", "construction",
                                                    "left", "right", "parking", "tl_green"};

    void declare_int(const std::string &name, int def, int min, int max)
    {
        rcl_interfaces::msg::ParameterDescriptor d;
        rcl_interfaces::msg::IntegerRange r;
        r.from_value = min;
        r.to_value = max;
        r.step = 1;
        d.integer_range.push_back(r);
        declare_parameter(name, def, d);
    }

    void reload_filter()
    {
        auto ms = [this](const char *n) { return int(get_parameter(n).as_int()); };
        turn_ms_ = ms("mission.turn_ms");
        rearm_ms_ = ms("mission.rearm_ms");
        parking_pause_ = ms("mission.parking_pause") != 0;
        barrier_release_ms_ = ms("mission.barrier_release_ms");
        con_end_mode_ = ms("mission.con_end_mode");
        con_hold_ms_ = ms("mission.con_hold_ms");
        con_once_ = ms("mission.con_once") != 0;
        start_follow_ = ms("mission.start_follow") != 0;

        confirm_frames_ = std::max(1, int(get_parameter("sign.confirm_frames").as_int()));
        min_score_ = get_parameter("sign.min_score_pct").as_int() / 100.0;
        for (const char *l : FILTER_LABELS)
        {
            const std::string p = std::string("sign.") + l;
            min_size_[l] = {int(get_parameter(p + ".min_h_px").as_int()), int(get_parameter(p + ".min_w_px").as_int()),
                            int(get_parameter(p + ".min_area_px").as_int())};
        }
    }

    bool pass_filter(const tb_interfaces::msg::Detection &d) const
    {
        if (d.score < min_score_)
            return false;
        auto it = min_size_.find(d.label);
        if (it == min_size_.end())
            return true;
        const MinSize &m = it->second;
        return d.h >= m.h && d.w >= m.w && int64_t(d.w) * d.h >= m.area;
    }

    void on_detections(const DetectionArray &msg)
    {
        if (run_state_ != 1)
            return;
        const rclcpp::Time now = this->now();

        // 이번 프레임에 필터를 통과한 class_id → 연속 프레임 수 갱신 (안 보이면 0부터 다시)
        std::set<int> passed;
        for (const auto &d : msg.detections)
            if (pass_filter(d))
                passed.insert(d.class_id);
        for (auto it = streak_.begin(); it != streak_.end();)
            it = passed.count(it->first) ? std::next(it) : streak_.erase(it);

        for (int id : passed)
        {
            if (++streak_[id] < confirm_frames_)
                continue; // 아직 확인 중
            if (id == ID_BARRIER_CLOSED)
                barrier_last_ = now; // 보이는 동안 계속 갱신 (barrier_release_ms 기준)
            auto it = last_seen_.find(id);
            const bool fresh = (it == last_seen_.end()) || (now - it->second).seconds() * 1000.0 > rearm_ms_;
            last_seen_[id] = now;
            // 차단바 닫힘은 pause가 풀려 있으면 rearm과 상관없이 다시 걸음 (잠깐 놓쳐서 풀린 뒤 다시 보인 경우)
            if (fresh || (id == ID_BARRIER_CLOSED && !barrier_pause_))
                handle(id, now);
        }
    }

    void handle(int id, const rclcpp::Time &now)
    {
        switch (id)
        {
        case ID_BARRIER_CLOSED:
            RCLCPP_INFO(get_logger(), "barrier_closed → pause 1");
            barrier_pause_ = true;
            publish_bool(pause_pub_, true);
            break;
        case ID_BARRIER_OPEN:
            RCLCPP_INFO(get_logger(), "barrier_open → pause 0");
            barrier_pause_ = false;
            publish_bool(pause_pub_, false);
            break;
        case ID_BRICK:
            break; // 구현 안 함
        case ID_CONSTRUCTION:
            // con_on_은 1·2 모드에서만 의미 있음 (0은 path_node가 벽돌로 끄는 것을 brain이 모름)
            if ((con_on_ && con_end_mode_ != 0) || (con_once_ && con_latched_))
            {
                RCLCPP_INFO(get_logger(), "construction → 무시 (%s)",
                            (con_on_ && con_end_mode_ != 0) ? "이미 공사 구간" : "주차 표지판 전까지 한 번만");
                break;
            }
            RCLCPP_INFO(get_logger(), "construction → construction 1 (end_mode %d)", con_end_mode_);
            con_on_ = true;
            con_latched_ = true;
            con_end_ = now + rclcpp::Duration::from_seconds(con_hold_ms_ / 1000.0);
            publish_bool(construction_pub_, true);
            break;
        case ID_PARKING:
            RCLCPP_INFO(get_logger(), "parking → construction 0%s", parking_pause_ ? ", pause 1" : "");
            con_on_ = false;
            con_latched_ = false; // 다음 바퀴 공사 표지판은 다시 받음
            publish_bool(construction_pub_, false);
            if (parking_pause_)
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
        turn_value_ = value;
        turn_end_ = now + rclcpp::Duration::from_seconds(turn_ms_ / 1000.0);
        RCLCPP_INFO(get_logger(), "%s → turn %u for %dms", name, value, turn_ms_);
        publish_turn(value);
    }

    void on_timer()
    {
        if (run_state_ != 1)
            return;
        // tb_drive는 cmd_timeout_ms 동안 명령이 없으면 버리므로 계속 발행
        if (follow_)
            publish_follow();

        const rclcpp::Time t = now();
        // 공사 구간: 시간으로 끝내기 (con_end_mode 1)
        if (con_on_ && con_end_mode_ == 1 && t >= con_end_)
        {
            RCLCPP_INFO(get_logger(), "construction %dms 지남 → construction 0", con_hold_ms_);
            con_on_ = false;
            publish_bool(construction_pub_, false);
        }
        // 차단바: 닫힘이 barrier_release_ms 동안 안 보이면 pause 해제 (올라가서 화면 밖으로 나간 경우)
        if (barrier_pause_ && barrier_release_ms_ > 0 && (t - barrier_last_).seconds() * 1000.0 > barrier_release_ms_)
        {
            RCLCPP_INFO(get_logger(), "barrier_closed %dms 동안 안 보임 → pause 0", barrier_release_ms_);
            barrier_pause_ = false;
            publish_bool(pause_pub_, false);
        }

        if (turn_value_ == TURN_NONE)
            return;
        if (t >= turn_end_)
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

    int turn_ms_ = 4000; // left/right 발행 시간
    int rearm_ms_ = 2000;
    bool parking_pause_ = false;
    int barrier_release_ms_ = 0;
    int con_end_mode_ = 0, con_hold_ms_ = 10000;
    bool con_once_ = true;

    bool con_on_ = false;      // brain이 공사 구간을 켜 둔 상태 (path_node가 벽돌로 먼저 끈 것은 모름)
    bool con_latched_ = false; // 이번 바퀴에 이미 공사 구간에 들어갔음 (주차 표지판에서 풂)
    rclcpp::Time con_end_{0, 0, RCL_ROS_TIME};
    bool barrier_pause_ = false; // 차단바 때문에 pause 중
    rclcpp::Time barrier_last_{0, 0, RCL_ROS_TIME};

    int confirm_frames_ = 2;
    double min_score_ = 0.0;
    struct MinSize { int h = 0, w = 0, area = 0; };   // [px], area = w×h [px²]
    std::map<std::string, MinSize> min_size_;          // label → 최소 박스 크기
    std::map<int, int> streak_;                           // class_id → 필터 통과 연속 프레임 수
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_cb_handle_;

    int run_state_ = 0;   // 0: 정지(발행 안 함), 1: 동작. /brain/run 으로 변경
    bool follow_ = false; // tl_green 본 뒤 FOLLOW 발행 중
    bool start_follow_ = false;

    uint8_t turn_value_ = TURN_NONE;
    rclcpp::Time turn_end_{0, 0, RCL_ROS_TIME};
    std::map<int, rclcpp::Time> last_seen_; // class_id → 마지막으로 본 시각

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr turn_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr construction_pub_, pause_pub_;
    rclcpp::Publisher<DriveCmd>::SharedPtr drive_cmd_pub_;
    rclcpp::Subscription<DetectionArray>::SharedPtr det_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr run_sub_, inject_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrainNode>());
    rclcpp::shutdown();
    return 0;
}