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
    double turn_gain = 1.0; // pure pursuit 곡률 배율. 1보다 크면 같은 lookahead에서 더 작게 돎 (공사 구간)
    double stop_dist = 0.04; // 경로 끝까지 이만큼 남으면 정지 [m]
    double corner_min_x = 0.20; // 이보다 가까운 경로에서는 코너를 찾지 않음 [m] (= BEV 아랫줄 거리)
    // 급한 곳에서 lookahead 줄이기: 앞 sharp_look 안에서 경로가 로봇 방향과 sharp_turn만큼 틀어지면 ld_sharp까지
    double ld_sharp = 0.0;      // 0이면 끔 [m]
    double sharp_start = 0.0;   // 이보다 덜 틀어지면 lookahead를 줄이지 않음 [rad]
    double sharp_turn = 1.05;   // 이만큼 틀어지면 ld_sharp를 다 씀 (sharp_start~sharp_turn 사이는 비례) [rad]
    double sharp_look = 0.35;   // 앞 이 길이 안의 경로를 봄 [m]
    double sharp_hold = 1.0;    // 급한 곳을 본 뒤 이 시간 동안은 ld_sharp 유지 (피봇·코너 빠져나올 때까지) [s]
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

// 경로에서 처음 크게 꺾이는 점과 꺾인 각도 (왼쪽 +)
// min_x보다 가까운 점(사각지대를 지나온 경로로 채운 구간)은 보지 않음.
// 그 구간은 이미 돌고 있거나 다 돈 곳이라, 거기서 코너를 찾으면 커브 직후에 피봇을 또 함
bool find_corner(const std::vector<Pt> &path, double min_turn, Pt &corner, double &turn, double min_x = 0.0);

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
        bool held = false;        // 마지막 경로를 붙잡고 가는 중
        const char *why = "";     // 정지 이유 (움직이면 "")
        bool has_corner = false;  // 경로에 큰 꺾임이 있음
        Pt corner;                // 꺾이는 지점 (base_link)
        double corner_turn = 0.0; // 꺾인 각도 [rad], 왼쪽 +
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
    mutable double sharp_t_ = -1e9; // 마지막으로 급한 곳을 본 시각 (ld_sharp 유지용)
};

// 날카로운 꺾임(교차로, 벽돌 회피)에서만 쓰는 제자리 회전.
//  - 코너가 confirm 주기 연속으로 보여야 시작 (스켈레톤 흔들림으로 생기는 가짜 코너 거르기)
//  - pure pursuit가 감당할 수 있는 keep 각도는 남기고 초과분만 제자리에서 돎
//  - 안전장치: 코너를 못 찾았는데 목표점이 fallback보다 옆이면, keep 안으로 들어올 때까지만 돎
struct PivotParams
{
    double enter = 1.75;    // 코너가 이보다 크게 꺾이면 피봇 [rad]
    double keep = 0.35;     // 이 각도까지는 pure pursuit에 맡김 (피봇은 초과분만) [rad]
    double exit = 0.26;     // 피봇 중 목표점이 이 안으로 들어오면 종료 [rad]
    double w = 1.57;        // 피봇 각속도 [rad/s]
    double fallback = 1.31; // 안전장치: 목표점 방향이 이보다 옆이면 바로 피봇 [rad]
    int confirm = 3;        // 코너가 이만큼 연속으로 보여야 시작 (제어 주기 수)
    double reach = 0.35;    // 이보다 가까운 코너만 다룸 [m]
};

class Pivoter
{
public:
    enum class Phase { Follow, Approach, Pivot };
    PivotParams p;

    void reset();
    // o: 이번 주기 Follower 결과, v_now·w_now: 지난 주기 실제 출력, dt: 주기
    // 피봇이 끼어들면 true를 돌려주고 v, w를 덮어씀 (why: "approach" / "pivot")
    // corner_ok=false면 코너 피봇은 하지 않고 안전장치만 (공사 구간처럼 경로를 미리 계획한 곳)
    bool step(const Follower::Out &o, double v_now, double w_now, double dt,
              double v_approach, double &v, double &w, const char *&why, bool corner_ok = true);
    Phase phase() const { return phase_; }

private:
    Phase phase_ = Phase::Follow;
    Pt corner_;           // 코너 위치 (로봇 기준, 매 주기 이동만큼 갱신)
    double turn_ = 0.0;   // 코너에서 꺾이는 각도 (왼쪽 +)
    double target_ = 0.0; // 이번 피봇에서 돌 각도 (>0)
    double done_ = 0.0;   // 지금까지 돈 각도
    bool from_fallback_ = false;
    int seen_ = 0;        // 같은 방향 코너가 연속으로 보인 횟수
    int seen_sign_ = 0;
};

} // namespace tb_drive
