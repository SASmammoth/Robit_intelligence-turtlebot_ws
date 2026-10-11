#pragma once
// 공사 구간: 벽돌 지도 + 지그재그 경로 (ROS 의존성 없음)
//
// 공사 구간 배치 (실측):
//   안쪽 차폭 56cm, 벽돌 20cm(가로) × 8cm(앞뒤), 차선에 딱 붙어 좌우 번갈아, 벽돌 사이 빈 공간 약 40cm
//   → 벽돌 옆 틈은 36cm, 로봇 폭 21cm. 틈 가운데는 차선 중심에서 벽돌 반대쪽으로 10cm
//
// 방식:
//   - 차선 좌표계(s: 차선 방향 거리, l: 차선 중심에서 왼쪽 +)를 월드 좌표에 둠
//   - 벽돌은 차선에 붙어 있으므로 좌우 위치는 "어느 쪽인가"만 쓰고, 앞뒤 위치만 관측으로 평균
//   - 벽돌 중심 = 차선 중심 ± (차폭/2 − 벽돌 가로/2) 이므로 벽돌 위치로 차선 중심선을 거꾸로 맞춤
//     (공사 구간에서는 두 선이 한 화면에 안 보이고, 벽돌이 선을 가려서 선으로는 맞추기 어려움)
//   - 경로: 벽돌 옆에서는 반대쪽 틈 가운데(±pass_off), 벽돌 사이에서 cos 모양으로 넘어감
//     (test/construction_sim.cpp: 로봇 21×23cm, 바퀴 축 앞 9·뒤 14cm에서 최소 여유 약 7cm, 최대 38°)
#include <opencv2/core.hpp>
#include <vector>
#include "tb_path/lanePath.hpp"

namespace construction {

using lane_path::Pose2D;

struct Params {
    double lane_w        = 0.56;  // 공사 구간 안쪽 차폭 [m]
    double brick_w       = 0.20;  // 벽돌이 차선에서 안쪽으로 나온 길이 (가로) [m]
    double brick_d       = 0.08;  // 벽돌 앞뒤 두께 [m]
    double pass_off      = 0.10;  // 벽돌 옆을 지날 때 차선 중심에서 반대쪽으로 [m]
    double trans_len     = 0.40;  // 한쪽 틈에서 반대쪽 틈으로 넘어가는 길이 [m]
    double spacing       = 0.48;  // 벽돌 중심 사이 기본 간격 (첫 벽돌 앞·마지막 벽돌 뒤 넘어가기에 씀) [m]
    double merge_dist    = 0.12;  // 같은 쪽에서 앞뒤로 이보다 가까운 관측은 같은 벽돌 [m]
    double keep_behind   = 0.60;  // 로봇 뒤로 이만큼 지난 벽돌은 지움 [m]
    double exit_after    = 0.50;  // 마지막 벽돌을 지나 이만큼 가면 공사 구간 끝 [m]
    double max_turn      = 0.35;  // 벽돌로 맞춘 차선 방향이 처음 방향에서 이보다 틀어지면 무시 [rad]
    double step          = 0.02;  // 경로 점 간격 [m]
    double behind        = 0.20;  // 경로를 로봇 뒤 이만큼부터 [m] (사각지대 채움과 같은 역할)
    double ahead         = 0.80;  // 경로를 로봇 앞 이만큼까지 [m]
};

struct Brick {
    cv::Point2d c;   // 중심 (월드)
    int side = 0;    // +1 차선 왼쪽, -1 오른쪽
    int n = 0;       // 관측 횟수
};

class Planner {
public:
    Params p;

    // 공사 구간 시작. lane_heading: 차선 방향(월드), lane_point: 차선 중심 위의 한 점(월드)
    void start(double lane_heading, const cv::Point2d &lane_point);
    void reset();
    bool active() const { return active_; }

    // 벽돌이 아직 없을 때 차선 기준을 고쳐 줌 (스켈레톤 중심선, 로봇 좌표)
    void lane_hint(const std::vector<cv::Point2d> &centerline_robot, const Pose2D &pose);

    // 벽돌 관측: 로봇 쪽을 보는 앞면의 가운데 (로봇 좌표, m)
    void observe(const cv::Point2d &front_face_robot, const Pose2D &pose);

    // 계획한 경로 (로봇 좌표, 가까운 점부터). 벽돌이 하나도 없으면 false
    bool path(const Pose2D &pose, std::vector<cv::Point2d> &out) const;

    // 마지막 벽돌을 지나 exit_after만큼 갔는지
    bool finished(const Pose2D &pose) const;

    const std::vector<Brick> &bricks() const { return bricks_; }
    double s_of(const cv::Point2d &w) const;  // 차선 방향 거리
    double l_of(const cv::Point2d &w) const;  // 차선 중심에서 왼쪽 거리
    double heading() const { return h_; }

private:
    void refit();
    double offset(double s) const;  // 차선 중심에서 목표 옆 위치
    cv::Point2d lane_to_world(double s, double l) const;

    bool active_ = false;
    double h0_ = 0.0, h_ = 0.0;  // 처음 차선 방향, 지금 차선 방향
    cv::Point2d o_;              // 차선 중심 위의 기준점 (월드)
    std::vector<Brick> bricks_;
    bool has_last_ = false;
    cv::Point2d last_c_;         // 지금까지 본 벽돌 중 가장 앞 (월드, 지워져도 남김)
};

} // namespace construction
