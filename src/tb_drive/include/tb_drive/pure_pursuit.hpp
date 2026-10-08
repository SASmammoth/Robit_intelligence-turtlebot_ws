#pragma once
// ROS 의존성 없는 경로 추종 계산부 (단위: m, s, rad)
// 좌표계는 base_link: x 앞, y 왼쪽

#include <deque>
#include <vector>

namespace tb_drive {

struct Pt
{
    double x = 0.0, y = 0.0;
};

struct Params
{
    double v_max = 0.15;    // 최고 속도 [m/s]
    double v_min = 0.04;    // 추종 중 최저 속도 [m/s] (모터 데드밴드 넘기기)
    double w_max = 1.57;    // 최대 각속도 [rad/s]
    double accel = 0.3;     // 가속 제한 [m/s^2]
    double decel = 0.6;     // 감속 제한 [m/s^2]
    double w_accel = 6.28;  // 각가속 제한 [rad/s^2]
    double ld_min = 0.15;   // lookahead 최소 [m]
    double ld_max = 0.35;   // lookahead 최대 [m]
    double ld_time = 1.0;   // lookahead = 속도 × ld_time [s]
    double curv_slow = 0.1; // v = limit / (1 + curv_slow·|곡률|) [m]
    double stop_dist = 0.04; // 경로 끝까지 이만큼 남으면 정지 [m]
};

struct Target
{
    bool ok = false;
    bool at_end = false;   // lookahead 원 안에서 경로가 끝남 → 마지막 점이 목표
    Pt p;                  // 목표점
    double ld = 0.0;       // 로봇에서 목표점까지 거리
    double kappa = 0.0;    // 곡률 (양수 = 좌회전)
    double end_dist = 0.0; // 로봇 → 첫 점 → ... → 마지막 점 길이
};

// 경로에서 거리 ld인 목표점을 찾는다.
// x <= 0인 앞부분 점(지연 보정 후 로봇 뒤로 밀려난 점)은 건너뛴다.
// 첫 점이 이미 ld보다 멀면(사각지대) 첫 점을 목표로 한다.
Target find_target(const std::vector<Pt> &path, double ld);

// 목표점 → (v, w). limit은 이번 주기 속도 상한 [m/s].
// 경로 끝이 가까우면 decel로 멈출 수 있는 속도까지 낮추고, stop_dist 안이면 0.
void command(const Target &t, const Params &p, double limit, double &v, double &w);

// 가감속 제한
struct Limiter
{
    double v = 0.0, w = 0.0;
    void reset() { v = w = 0.0; }
    void step(double v_des, double w_des, double dt, const Params &p);
};

// 발행한 (v, w) 기록. 영상 지연 동안 로봇이 움직인 양을 구하는 데 쓴다.
class MotionHistory
{
public:
    void push(double t, double v, double w);
    void clear() { h_.clear(); }
    // t0 → t1 동안의 이동 (t0 시점 base_link 기준)
    void delta(double t0, double t1, double &dx, double &dy, double &dth) const;

private:
    struct E
    {
        double t, v, w; // t부터 (v, w)로 움직임
    };
    std::deque<E> h_;
};

// 옛 base_link 기준 경로를, 로봇이 (dx, dy, dth)만큼 움직인 뒤의 base_link 기준으로 바꿈
std::vector<Pt> shift_path(const std::vector<Pt> &in, double dx, double dy, double dth);

// 경로 관리 + 목표점 계산 (노드와 시뮬레이션이 같은 코드를 씀)
class Follower
{
public:
    Params p;
    bool latency_comp = false; // 영상 지연 동안 움직인 양만큼 경로를 당김
    double path_timeout = 0.4; // 이보다 오래된 경로는 버림 [s]
    double hold_time = 0.6;    // 경로가 비거나 끊긴 뒤 마지막 경로를 추측항법으로 따라가는 시간 [s]

    struct Out
    {
        double v = 0.0, w = 0.0; // 원하는 속도 (가감속 제한 전)
        Target tg;
        bool held = false;       // 마지막 경로를 붙잡고 가는 중
        const char *why = "";    // 정지 이유 (움직이면 "")
    };

    void reset();
    void set_path(std::vector<Pt> pts, double stamp); // 빈 경로도 그대로 넣는다
    void record(double t, double v, double w) { hist_.push(t, v, w); }
    // v_now: 지금 속도 (lookahead 길이 계산용), limit: 속도 상한
    Out follow(double t, double v_now, double limit) const;

private:
    MotionHistory hist_;
    std::vector<Pt> cur_, good_;
    double cur_stamp_ = -1.0, good_stamp_ = -1.0;
};

} // namespace tb_drive
