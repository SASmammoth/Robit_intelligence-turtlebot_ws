// tb_brain: /signs/detections (YOLO 표지판, class_id) → 미션 토픽 발행
//
//   class_id            동작
//   ------------------  ------------------------------------------------------
//   0 barrier_closed    pause = 1  (mission.barrier_release_ms 동안 안 보이면 pause = 0)
//   1 barrier_open      pause = 0
//   차단바 끝 (pause 해제)  /mission/stop_line = 1  (mission.stop_line = 1일 때, 한 번)
//   2 brick             (구현 안 함)
//   3 construction      /mission/construction = 1  (끝나는 방법은 mission.con_end_mode)
//   5 parking           /mission/construction = 0, 주차 동작 시작 (아래 "주차")
//   4 left              /mission/turn = 1 을 mission.turn_ms 동안 발행 후 0
//   6 right             /mission/turn = 2 를 mission.turn_ms 동안 발행 후 0
//   7 tl_green          /drive/cmd = FOLLOW 를 계속 발행 (stop 받을 때까지)
//   8 tl_off
//   9 tl_red
//   10 tl_yellow
//
// 정지선 (/mission/stop_line): 차단바 다음 단계. 차단바가 끝나면(열림 검출 또는 닫힘이
//   barrier_release_ms 동안 안 보임) 켜고, 켜져 있는 동안 path_node가 빨간 가로선 앞에서
//   경로를 잘라 tb_drive가 선 앞에서 멈춤. 한 번 켜면 /brain/run stop 또는 /mission/reset까지 유지.
//   10Hz로 현재 값을 계속 보냄 (path_node 재시작 대비). mission.stop_line = 0이면 아예 발행하지 않음
//   가짜 표지판 "stop_line"으로 바로 켤 수도 있음
//
// /mission/reset (Empty): 미션 상태를 처음처럼 (run_state는 그대로). turn 0, construction 0,
//   pause 0, stop_line 0을 한 번 보냄. path_node·drive_node도 같은 토픽으로 초기화
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
//   sign.<label>.confirm_frames      그 표지판만 따로 (0이면 sign.confirm_frames)
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
// 주차 (parking 표지판, mission.parking.*):
//   1. /mission/turn = 1(left)을 time_enter초 동안 → pause 1
//   2. 다음에 받은 brick 박스 중심이 x 320보다 왼쪽이면 brick_pos left(crit psd_L), 아니면 right(crit psd_R)
//   3. TB_Uart_RX "psd period 100", "velocity move_vel move_vel"
//      brick_pos 쪽 PSD(left: psd1, right: psd2)가 crit 초과 → 0.5초 더 간 뒤 velocity 0 0
//   4. 0.5초 뒤 제자리 회전 (left: -turn_vel turn_vel, right: turn_vel -turn_vel), turn_deg만큼 도는 시간 뒤 → 0 0
//      회전 시간 = turn_deg / (2v / PIVOT_SEP_M), v = turn_vel 원시값 → m/s (psd_F는 안 씀, GUI용으로 선언만 유지)
//   5. dist_mm 후진 → 0 0 → parking_pause초 대기 → dist_mm 직진 → 0 0
//   6. 같은 방향 제자리 회전, turn_deg만큼 도는 시간 뒤 → 0 0
//   7. pause 0, /mission/turn = 1을 time_exit초 동안
//   PSD는 TB_Uart_TX "psd <0> <1> <2>" (tb_uart: F L R 순서). 주차 중에는 다른 표지판 무시
//   거리는 TB_Uart_TX "velocity L R" 피드백(33ms)을 적분: 원시값 → m/s는 tb_drive publish_uart의 역
//   (피드백에는 direction 부호가 안 곱해져 있으므로 |L|, |R| 평균. 방향은 보낸 명령으로 앎)
//   피드백이 0.3초 넘게 안 오면 보낸 명령 값 × 시간으로 계산
//   tb_drive는 pause로 멈춘 뒤 UART를 보내지 않음 (안 그러면 brain 명령을 0으로 덮어씀)
//   parking_pause는 GUI가 읽는 mission.parking_pause [s]
//
// 파라미터는 모두 정수 (GUI 스핀박스, Save 파일의 /brain_node 항목)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
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
static constexpr const char *TOPIC_STOP_LINE = "/mission/stop_line";     // 발행 (Bool: 정지선 앞에서 경로 자르기)
static constexpr const char *TOPIC_DRIVE_CMD = "/drive/cmd";             // 발행 (DriveCmd)
static constexpr const char *TOPIC_RUN = "/brain/run";                   // 구독 (String: "start" / "stop")
static constexpr const char *TOPIC_INJECT = "/brain/inject";             // 구독 (String: 표지판 이름, 가짜 표지판)
static constexpr const char *TOPIC_RESET = "/mission/reset";             // 구독 (Empty: 전체 초기화)
static constexpr const char *TOPIC_UART_RX = "TB_Uart_RX";              // 발행 (String: tb_uart 명령)
static constexpr const char *TOPIC_UART_TX = "TB_Uart_TX";              // 구독 (String: "psd F L R" 등)

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

// ====== 주차
static constexpr int IMG_CX = 320;                // 화면 가운데 x (640x480)
static constexpr double PARK_APPROACH_EXTRA_S = 0.5; // 접근 PSD 기준을 넘은 뒤 이만큼 더 가고 정지
static constexpr double PARK_STOP_S = 0.5;        // PSD로 멈춘 뒤 회전 전 정지 시간
static constexpr double WHEEL_RADIUS_M = 0.0475;  // tb_drive drive.uart.wheel_radius_m
static constexpr double RPM_PER_UNIT = 0.43;      // tb_drive drive.uart.rpm_per_unit (실측 보정값. 0.229면 거리를 약 1.9배 짧게 잼)
static constexpr double PIVOT_SEP_M = 0.165;      // tb_drive drive.uart.pivot_sep_m (제자리 회전 실측 간격, 바퀴 사이 실제 193.4mm)
static constexpr double PARK_SETTLE_S = 1.0;      // pause 뒤 tb_drive가 감속을 끝내고 UART를 놓을 때까지 기다림
static constexpr double VEL_FB_TIMEOUT_S = 0.3;   // velocity 피드백이 이만큼 없으면 보낸 명령 × 시간으로 거리 계산

class BrainNode : public rclcpp::Node
{
public:
    BrainNode() : Node("brain_node")
    {
        // 미션 동작 (정수: GUI 스핀박스로 바꿈. ros2 param set /brain_node mission.turn_ms 4000 도 가능)
        declare_int("mission.turn_ms", 4000, 0, 30000);         // 좌/우 표지판 → turn 유지 시간
        declare_int("mission.rearm_ms", 2000, 0, 30000);        // 같은 표지판을 이만큼 안 보다가 다시 보면 새 표지판
        declare_int("mission.barrier_release_ms", 0, 0, 30000); // 차단바(닫힘)가 이만큼 안 보이면 pause 해제 (0: barrier_open만)
        declare_int("mission.con_end_mode", 0, 0, 2);           // 0 path_node 벽돌 기준, 1 시간, 2 주차 표지판까지
        declare_int("mission.con_hold_ms", 10000, 0, 120000);   // con_end_mode 1: 진입 후 이만큼 지나면 끝
        declare_int("mission.con_once", 1, 0, 1);               // 1: 진입 후 주차 표지판 전까지 공사 표지판 무시
        declare_int("mission.start_follow", 0, 0, 1);           // 1: start 받자마자 FOLLOW (초록불 안 기다림)
        declare_int("mission.stop_line", 1, 0, 1);              // 1: 차단바가 끝나면 정지선 켬 (빨간 가로선 앞에서 멈춤)

        // 주차
        declare_int("mission.parking.time_enter", 4, 0, 60);     // [s] 주차 표지판 → left 유지 후 pause
        declare_int("mission.parking.time_exit", 6, 0, 60);      // [s] 주차 끝 → left 유지
        declare_int("mission.parking.psd_L", 2000, 0, 65535);    // brick left: 접근 정지 기준 / right: 나올 때 회전 정지 기준
        declare_int("mission.parking.psd_R", 2500, 0, 65535);    // brick right: 접근 정지 기준 / left: 나올 때 회전 정지 기준
        declare_int("mission.parking.psd_F", 3000, 0, 65535);    // 들어갈 때 회전 정지 기준 (psd0)
        declare_int("mission.parking.turn_vel", 70, 0, 1000);    // 제자리 회전 속도 (원시값)
        declare_int("mission.parking.turn_deg", 90, 0, 360);     // 제자리 회전 각도 (들어갈 때·나올 때)
        declare_int("mission.parking.move_vel", 70, 1, 1000);    // 전진·후진 속도 (원시값)
        declare_int("mission.parking.dist_mm", 180, 0, 2000);    // 후진·직진 거리
        declare_int("mission.parking_pause", 1, 0, 60);          // [s] 주차: 후진 뒤 정지 시간

        // 검출 필터
        declare_int("sign.confirm_frames", 2, 1, 30);
        declare_int("sign.min_score_pct", 0, 0, 100);
        for (const char *l : FILTER_LABELS)
        {
            declare_int(std::string("sign.") + l + ".min_h_px", 0, 0, 480);
            declare_int(std::string("sign.") + l + ".min_w_px", 0, 0, 640);
            declare_int(std::string("sign.") + l + ".min_area_px", 0, 0, 640 * 480);
            declare_int(std::string("sign.") + l + ".confirm_frames", 0, 0, 30); // 0이면 sign.confirm_frames
        }
        reload_filter();
        post_cb_handle_ = add_post_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &) { reload_filter(); });

        turn_pub_ = create_publisher<std_msgs::msg::UInt8>(TOPIC_TURN, 10);
        construction_pub_ = create_publisher<std_msgs::msg::Bool>(TOPIC_CONSTRUCTION, 10);
        pause_pub_ = create_publisher<std_msgs::msg::Bool>(TOPIC_PAUSE, 10);
        stop_line_pub_ = create_publisher<std_msgs::msg::Bool>(TOPIC_STOP_LINE, 10);
        drive_cmd_pub_ = create_publisher<DriveCmd>(TOPIC_DRIVE_CMD, 10);
        uart_pub_ = create_publisher<std_msgs::msg::String>(TOPIC_UART_RX, 10);

        run_sub_ = create_subscription<std_msgs::msg::String>(
            TOPIC_RUN, 10,
            [this](const std_msgs::msg::String::ConstSharedPtr m) { on_run(m->data); });
        reset_sub_ = create_subscription<std_msgs::msg::Empty>(
            TOPIC_RESET, 10, [this](const std_msgs::msg::Empty::ConstSharedPtr) { on_reset(); });
        inject_sub_ = create_subscription<std_msgs::msg::String>(
            TOPIC_INJECT, 10,
            [this](const std_msgs::msg::String::ConstSharedPtr m) { on_inject(m->data); });

        det_sub_ = create_subscription<DetectionArray>(
            TOPIC_DETECTIONS, 10,
            [this](const DetectionArray::ConstSharedPtr m) { on_detections(*m); });
        uart_sub_ = create_subscription<std_msgs::msg::String>(
            TOPIC_UART_TX, 10,
            [this](const std_msgs::msg::String::ConstSharedPtr m) { on_uart(m->data); });

        // turn 유지/종료, drive cmd 유지 (10Hz)
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { on_timer(); });
        // 주차 단계의 시간 조건 (정지 0.5초, parking_pause)
        park_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { parking_step(now()); });

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
            if (park_ != Park::Idle)
            {
                // 주차 중이면 brain이 직접 돌리던 모터를 세우고 drive에 넘겨줌
                RCLCPP_WARN(get_logger(), "run: stop → 주차 중단");
                abort_parking();
            }
            run_state_ = 0;
            // 다시 start 했을 때 이전 상태가 이어지지 않게
            if (stop_line_on_) // 멈추면 더 발행하지 않으므로 마지막으로 풀어 둠
                publish_now(stop_line_pub_, false);
            clear_state();
            RCLCPP_INFO(get_logger(), "run: stop → run_state 0");
        }
        else
        {
            RCLCPP_WARN(get_logger(), "run: unknown command '%s' (use '%s' / '%s')", cmd.c_str(), RUN_START, RUN_STOP);
        }
    }

    // 미션 상태만 처음으로 (run_state·파라미터는 그대로)
    void clear_state()
    {
        turn_value_ = TURN_NONE;
        follow_ = false;
        streak_.clear();
        last_seen_.clear(); // 앞에 보이는 표지판을 새로 봄
        con_on_ = false;
        con_latched_ = false;
        barrier_pause_ = false;
        stop_line_on_ = false;
    }

    // 주차 중단: 모터 0, pause 풀기 (run_state와 상관없이)
    void abort_parking()
    {
        send_velocity(0, 0);
        publish_now(pause_pub_, false);
        park_ = Park::Idle;
    }

    // /mission/reset: 상태를 지우고 중립 값을 한 번 보냄 (run_state 0이어도 보냄: 남아 있는 값 풀기)
    void on_reset()
    {
        if (park_ != Park::Idle)
        {
            RCLCPP_WARN(get_logger(), "reset: 주차 중단");
            abort_parking();
        }
        clear_state();
        publish_now(turn_pub_, TURN_NONE);
        publish_now(construction_pub_, false);
        publish_now(pause_pub_, false);
        publish_now(stop_line_pub_, false);
        RCLCPP_INFO(get_logger(), "reset: 미션 상태 초기화 (run_state %d 유지)", run_state_);
        if (run_state_ == 1 && start_follow_)
        {
            follow_ = true;
            publish_follow();
        }
    }

    // run_state와 상관없이 바로 발행 (reset·stop 때 남은 값 풀기)
    static void publish_now(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr &pub, bool v)
    {
        std_msgs::msg::Bool m;
        m.data = v;
        pub->publish(m);
    }
    static void publish_now(const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr &pub, uint8_t v)
    {
        std_msgs::msg::UInt8 m;
        m.data = v;
        pub->publish(m);
    }

    // 차단바 단계 끝 → 정지선 켬 (한 번)
    void barrier_done(const char *why)
    {
        if (!stop_line_mode_ || stop_line_on_)
            return;
        RCLCPP_INFO(get_logger(), "차단바 끝 (%s) → stop_line 1 (빨간 정지선 앞에서 멈춤)", why);
        stop_line_on_ = true;
        publish_bool(stop_line_pub_, true);
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
        if (label == "stop_line") // 표지판이 아닌 단계: 바로 정지선 켬
        {
            RCLCPP_INFO(get_logger(), "inject: stop_line");
            barrier_done("가짜 표지판");
            return;
        }
        auto it = ids.find(label);
        if (it == ids.end())
        {
            RCLCPP_WARN(get_logger(), "inject: 모르는 표지판 '%s'", label.c_str());
            return;
        }
        if (park_ != Park::Idle)
        {
            RCLCPP_WARN(get_logger(), "inject '%s' 무시: 주차 중", label.c_str());
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
        barrier_release_ms_ = ms("mission.barrier_release_ms");
        con_end_mode_ = ms("mission.con_end_mode");
        con_hold_ms_ = ms("mission.con_hold_ms");
        con_once_ = ms("mission.con_once") != 0;
        start_follow_ = ms("mission.start_follow") != 0;
        stop_line_mode_ = ms("mission.stop_line") != 0;
        park_time_enter_ = ms("mission.parking.time_enter");
        park_time_exit_ = ms("mission.parking.time_exit");
        park_psd_l_ = ms("mission.parking.psd_L");
        park_psd_r_ = ms("mission.parking.psd_R");
        park_psd_f_ = ms("mission.parking.psd_F");
        park_turn_vel_ = ms("mission.parking.turn_vel");
        park_turn_deg_ = ms("mission.parking.turn_deg");
        park_move_vel_ = ms("mission.parking.move_vel");
        park_dist_mm_ = ms("mission.parking.dist_mm");
        park_pause_s_ = ms("mission.parking_pause");

        confirm_frames_ = std::max(1, int(get_parameter("sign.confirm_frames").as_int()));
        min_score_ = get_parameter("sign.min_score_pct").as_int() / 100.0;
        for (const char *l : FILTER_LABELS)
        {
            const std::string p = std::string("sign.") + l;
            min_size_[l] = {int(get_parameter(p + ".min_h_px").as_int()), int(get_parameter(p + ".min_w_px").as_int()),
                            int(get_parameter(p + ".min_area_px").as_int()),
                            int(get_parameter(p + ".confirm_frames").as_int())};
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
        if (park_ == Park::WaitBrick)
            parking_brick(msg, now);

        // 이번 프레임에 필터를 통과한 class_id → 연속 프레임 수 갱신 (안 보이면 0부터 다시)
        std::set<int> passed;
        std::map<int, int> need; // class_id → 필요한 연속 프레임 (표지판별 값, 0이면 공통 값)
        for (const auto &d : msg.detections)
            if (pass_filter(d))
            {
                passed.insert(d.class_id);
                auto m = min_size_.find(d.label);
                need[d.class_id] = (m != min_size_.end() && m->second.confirm > 0) ? m->second.confirm : confirm_frames_;
            }
        for (auto it = streak_.begin(); it != streak_.end();)
            it = passed.count(it->first) ? std::next(it) : streak_.erase(it);

        for (int id : passed)
        {
            if (++streak_[id] < need[id])
                continue; // 아직 확인 중
            if (id == ID_BARRIER_CLOSED)
                barrier_last_ = now; // 보이는 동안 계속 갱신 (barrier_release_ms 기준)
            auto it = last_seen_.find(id);
            const bool fresh = (it == last_seen_.end()) || (now - it->second).seconds() * 1000.0 > rearm_ms_;
            last_seen_[id] = now;
            // 차단바 닫힘은 pause가 풀려 있으면 rearm과 상관없이 다시 걸음 (잠깐 놓쳐서 풀린 뒤 다시 보인 경우)
            if (park_ != Park::Idle)
                continue; // 주차 중에는 다른 표지판 무시
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
            barrier_done("열림");
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
            RCLCPP_INFO(get_logger(), "parking → construction 0, 주차 시작 (left %ds)", park_time_enter_);
            con_on_ = false;
            con_latched_ = false; // 다음 바퀴 공사 표지판은 다시 받음
            publish_bool(construction_pub_, false);
            park_ = Park::Enter;
            park_t_ = now;
            set_turn(TURN_LEFT, now, park_time_enter_ * 1000);
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
        RCLCPP_INFO(get_logger(), "%s → turn %u for %dms", name, value, turn_ms_);
        set_turn(value, now, turn_ms_);
    }

    // 회전 신호 중이어도 덮어씀 (끝내기는 on_timer)
    void set_turn(uint8_t value, const rclcpp::Time &now, int ms)
    {
        turn_value_ = value;
        turn_end_ = now + rclcpp::Duration::from_seconds(ms / 1000.0);
        publish_turn(value);
    }

    // ====== 주차
    // TB_Uart_TX "psd <0> <1> <2>" (tb_uart가 F L R 순서로 보냄), "velocity <L> <R>" (33ms 피드백)
    void on_uart(const std::string &s)
    {
        std::istringstream iss(s);
        std::string head;
        int p[3];
        if (!(iss >> head))
            return;
        const rclcpp::Time t = now();
        if (head == "psd" && (iss >> p[0] >> p[1] >> p[2]))
        {
            for (int i = 0; i < 3; ++i)
                psd_[i] = p[i];
            psd_t_ = t;
        }
        else if (head == "velocity" && (iss >> p[0] >> p[1]))
        {
            // 직진·후진 중 이동 거리 적분 (단계 시작 또는 지난 피드백부터)
            if (park_ == Park::Back || park_ == Park::Forward)
            {
                const double dt = std::clamp((t - vel_t_).seconds(), 0.0, 0.1);
                const double raw = (std::abs(p[0]) + std::abs(p[1])) / 2.0;
                park_dist_ += raw * RPM_PER_UNIT * 2.0 * M_PI * WHEEL_RADIUS_M / 60.0 * dt;
                vel_fb_ = true;
            }
            vel_t_ = t;
        }
        else
            return; // "psd F 123", "velocity L 5" 같은 단일 값은 안 씀
        if (park_ != Park::Idle)
            parking_step(t);
    }

    // 다음에 받은 brick 박스(여러 개면 가장 큰 것)의 중심으로 좌/우 결정
    void parking_brick(const DetectionArray &msg, const rclcpp::Time &now)
    {
        const tb_interfaces::msg::Detection *best = nullptr;
        for (const auto &d : msg.detections)
            if (d.class_id == ID_BRICK && d.score >= min_score_ &&
                (!best || int64_t(d.w) * d.h > int64_t(best->w) * best->h))
                best = &d;
        if (!best)
            return;
        // pause 직후에는 tb_drive가 아직 감속 중이라 brain 명령을 덮어씀 → 멈춘 뒤에 시작
        if ((now - park_t_).seconds() < PARK_SETTLE_S)
            return;
        const double cx = best->x + best->w / 2.0;
        brick_left_ = cx < IMG_CX;
        park_crit_ = brick_left_ ? park_psd_l_ : park_psd_r_;
        RCLCPP_INFO(get_logger(), "parking: brick x %.0f → brick_pos %s (crit %d)", cx,
                    brick_left_ ? "left" : "right", park_crit_);
        send_uart("psd period 100");
        send_uart("velocity period 33"); // 직진·후진 거리용 피드백 (안 오면 시간으로 계산)
        send_velocity(park_move_vel_, park_move_vel_);
        park_next(Park::Approach, now);
    }

    void parking_step(const rclcpp::Time &t)
    {
        if (park_ == Park::Idle || run_state_ != 1)
            return;
        const double el = (t - park_t_).seconds();
        // 직진·후진 중 velocity 피드백이 끊기면 보낸 명령 × 시간으로 거리 (피드백이 안 오면 영원히 안 끝나는 것 방지)
        if ((park_ == Park::Back || park_ == Park::Forward) && (t - vel_t_).seconds() > VEL_FB_TIMEOUT_S)
        {
            const double dt = std::clamp((t - vel_t_).seconds(), 0.0, 0.5); // 끊긴 동안 전체
            if (!vel_fb_warned_)
            {
                RCLCPP_WARN(get_logger(), "parking: velocity 피드백 없음 → 명령 값 × 시간으로 거리 계산");
                vel_fb_warned_ = true;
            }
            park_dist_ += park_move_vel_ * RPM_PER_UNIT * 2.0 * M_PI * WHEEL_RADIUS_M / 60.0 * dt;
            vel_t_ = t;
        }
        // 이 단계에 들어온 뒤 받은 PSD만 씀 (예전 값으로 바로 넘어가지 않게)
        const bool psd_new = psd_t_ > park_t_;
        const int tv = brick_left_ ? -park_turn_vel_ : park_turn_vel_; // left: 왼쪽 바퀴 뒤로

        switch (park_)
        {
        case Park::Enter:
            if (el < park_time_enter_)
                return;
            RCLCPP_INFO(get_logger(), "parking: pause 1, brick 기다림");
            publish_bool(pause_pub_, true);
            park_next(Park::WaitBrick, t);
            break;
        case Park::WaitBrick:
            break; // parking_brick
        case Park::Approach:
            if (!psd_new || psd_[brick_left_ ? 1 : 2] <= park_crit_)
                return;
            park_next(Park::ApproachExtra, t); // 그대로 진행, 0.5초 뒤 정지
            break;
        case Park::ApproachExtra:
            if (el < PARK_APPROACH_EXTRA_S)
                return;
            send_velocity(0, 0);
            park_next(Park::Stop, t);
            break;
        case Park::Stop:
            if (el < PARK_STOP_S)
                return;
            send_velocity(tv, -tv);
            park_next(Park::TurnIn, t);
            break;
        case Park::TurnIn:
            if (el < pivot_time_s())
                return;
            send_velocity(0, 0);
            send_velocity(-park_move_vel_, -park_move_vel_);
            park_next(Park::Back, t);
            break;
        case Park::Back:
            if (park_dist_ * 1000.0 < park_dist_mm_)
                return;
            send_velocity(0, 0);
            park_next(Park::Hold, t);
            break;
        case Park::Hold:
            if (el < park_pause_s_)
                return;
            send_velocity(park_move_vel_, park_move_vel_);
            park_next(Park::Forward, t);
            break;
        case Park::Forward:
            if (park_dist_ * 1000.0 < park_dist_mm_)
                return;
            send_velocity(0, 0);
            send_velocity(tv, -tv);
            park_next(Park::TurnOut, t);
            break;
        case Park::TurnOut:
            if (el < pivot_time_s())
                return;
            send_velocity(0, 0);
            publish_bool(pause_pub_, false);
            set_turn(TURN_LEFT, t, park_time_exit_ * 1000);
            last_seen_[ID_PARKING] = t; // 나오면서 다시 보여도 rearm 동안은 무시
            RCLCPP_INFO(get_logger(), "parking: 끝 → pause 0, left %ds", park_time_exit_);
            park_next(Park::Idle, t);
            break;
        case Park::Idle:
            break;
        }
    }

    enum class Park { Idle, Enter, WaitBrick, Approach, ApproachExtra, Stop, TurnIn, Back, Hold, Forward, TurnOut };
    static const char *park_name(Park p)
    {
        static const char *const n[] = {"idle", "enter", "wait_brick", "approach", "approach_extra", "stop",
                                        "turn_in", "back", "hold", "forward", "turn_out"};
        return n[int(p)];
    }

    // 제자리 회전 시간 [s]: turn_vel(원시값) → 바퀴 선속도 v [m/s], 각속도 = 2v / 바퀴 간격
    double pivot_time_s() const
    {
        const double v = park_turn_vel_ * RPM_PER_UNIT * 2.0 * M_PI * WHEEL_RADIUS_M / 60.0;
        return v > 0.0 ? park_turn_deg_ * M_PI / 180.0 * PIVOT_SEP_M / (2.0 * v) : 0.0;
    }

    void park_next(Park p, const rclcpp::Time &t)
    {
        RCLCPP_INFO(get_logger(), "parking: %s → %s", park_name(park_), park_name(p));
        park_ = p;
        park_t_ = t;
        park_dist_ = 0.0;
        vel_t_ = t;
        vel_fb_ = false;
    }

    void send_velocity(int l, int r)
    {
        send_uart("velocity " + std::to_string(l) + " " + std::to_string(r));
    }

    void send_uart(const std::string &s)
    {
        std_msgs::msg::String m;
        m.data = s;
        uart_pub_->publish(m);
    }

    void on_timer()
    {
        if (run_state_ != 1)
            return;
        // tb_drive는 cmd_timeout_ms 동안 명령이 없으면 버리므로 계속 발행
        if (follow_)
            publish_follow();
        if (stop_line_mode_)
            publish_bool(stop_line_pub_, stop_line_on_); // path_node가 재시작돼도 맞게

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
            barrier_done("닫힘이 안 보임");
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
    int barrier_release_ms_ = 0;
    int con_end_mode_ = 0, con_hold_ms_ = 10000;
    bool con_once_ = true;

    bool con_on_ = false;      // brain이 공사 구간을 켜 둔 상태 (path_node가 벽돌로 먼저 끈 것은 모름)
    bool con_latched_ = false; // 이번 바퀴에 이미 공사 구간에 들어갔음 (주차 표지판에서 풂)
    rclcpp::Time con_end_{0, 0, RCL_ROS_TIME};
    bool barrier_pause_ = false; // 차단바 때문에 pause 중
    rclcpp::Time barrier_last_{0, 0, RCL_ROS_TIME};

    // 주차
    int park_time_enter_ = 4, park_time_exit_ = 6, park_pause_s_ = 1; // [s]
    int park_psd_l_ = 2000, park_psd_r_ = 2500, park_psd_f_ = 3000;
    int park_turn_vel_ = 70, park_move_vel_ = 70, park_dist_mm_ = 180;
    int park_turn_deg_ = 90;
    Park park_ = Park::Idle;
    rclcpp::Time park_t_{0, 0, RCL_ROS_TIME}; // 지금 단계 시작 시각
    bool brick_left_ = true;                  // brick_pos (true: left)
    int park_crit_ = 0;
    int psd_[3] = {0, 0, 0};                  // psd0 F, psd1 L, psd2 R
    rclcpp::Time psd_t_{0, 0, RCL_ROS_TIME};
    double park_dist_ = 0.0;                  // [m] 이번 단계에서 velocity 피드백으로 잰 이동 거리
    rclcpp::Time vel_t_{0, 0, RCL_ROS_TIME};  // 마지막 velocity 피드백 (적분 구간 시작)
    bool vel_fb_ = false;                     // 이번 단계에서 velocity 피드백을 받았는지
    bool vel_fb_warned_ = false;

    int confirm_frames_ = 2;
    double min_score_ = 0.0;
    struct MinSize { int h = 0, w = 0, area = 0, confirm = 0; }; // [px], area = w×h [px²], confirm: 연속 프레임 (0 = 공통)
    std::map<std::string, MinSize> min_size_;          // label → 최소 박스 크기
    std::map<int, int> streak_;                           // class_id → 필터 통과 연속 프레임 수
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_cb_handle_;

    int run_state_ = 0;   // 0: 정지(발행 안 함), 1: 동작. /brain/run 으로 변경
    bool follow_ = false; // tl_green 본 뒤 FOLLOW 발행 중
    bool start_follow_ = false;
    bool stop_line_mode_ = true; // mission.stop_line
    bool stop_line_on_ = false;  // 정지선 켜 둔 상태 (차단바 끝 → 1, run stop·reset → 0)

    uint8_t turn_value_ = TURN_NONE;
    rclcpp::Time turn_end_{0, 0, RCL_ROS_TIME};
    std::map<int, rclcpp::Time> last_seen_; // class_id → 마지막으로 본 시각

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr turn_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr construction_pub_, pause_pub_, stop_line_pub_;
    rclcpp::Publisher<DriveCmd>::SharedPtr drive_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr uart_pub_;
    rclcpp::Subscription<DetectionArray>::SharedPtr det_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr run_sub_, inject_sub_, uart_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
    rclcpp::TimerBase::SharedPtr timer_, park_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrainNode>());
    rclcpp::shutdown();
    return 0;
}