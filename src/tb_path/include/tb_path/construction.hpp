#pragma once
// 공사 구간: 가까운 벽돌 기준 지그재그 경로 (ROS 의존성 없음)
//
// 공사 구간 배치 (실측):
//   좌회전 → 직선(안쪽 차폭 56cm, 벽돌 3개) → 좌회전. 첫 벽돌은 꺾임이 끝나자마자, 좌우는 랜덤
//   벽돌 20cm(가로) × 8cm(앞뒤), 차선에 딱 붙어 좌우 번갈아, 벽돌 사이 빈 공간 약 40cm
//   → 벽돌 옆 틈 36cm, 로봇 폭 21cm. 틈 가운데 = 벽돌이 붙은 선에서 20 + 18 = 38cm
//
// 방식 (긴 지도·직선 차선 가정 없이, 매 프레임 보이는 것 기준):
//   - 차선 방향: 벽돌 근처에 보이는 선의 방향 (곡선에서 들어와도 그때그때 맞음)
//   - 벽돌이 붙은 쪽: 벽돌 끝에 선이 닿아 있는 쪽 / 반대쪽에 넓은 틈 너머 선이 보이면 그 반대 / 박스가 화면 끝에서 잘린 쪽
//   - 지날 점: 벽돌이 붙은 선(또는 박스 끝)에서 38cm, 반대쪽 선이 보이면 그 선에서 18cm
//     (박스 폭은 옆면까지 잡혀 넓어지므로 안쪽 끝은 쓰지 않음)
//   - 벽돌은 지나갈 때까지만 짧게 기억 (사각지대에 들어가도 경로 유지). 이동량 오차가 쌓일 시간이 없음
//   - 경로: 벽돌 옆에서는 지날 점의 옆 위치, 벽돌 사이 빈 공간에서 cos 모양으로 넘어감
#include <opencv2/core.hpp>
#include <vector>
#include "tb_path/lanePath.hpp"

namespace construction {

using lane_path::Pose2D;

struct Params {
    double lane_w      = 0.56;  // 공사 구간 안쪽 차폭 [m]
    double brick_w     = 0.20;  // 벽돌이 선에서 안쪽으로 나온 길이 [m]
    double brick_d     = 0.08;  // 벽돌 앞뒤 두께 [m]
    double contact     = 0.12;  // 벽돌 끝에서 이 안에 선이 있으면 붙어 있다고 봄 [m]
    double trans_len   = 0.40;  // 첫 벽돌 앞에서 옆으로 옮겨 가는 길이 [m]
    double first_lead  = 0.06;  // 첫 벽돌은 앞면보다 이만큼 먼저 옆 위치에 도착 (꺾임에서 막 나와 늦게 붙음) [m]
    double hold_after  = 0.04;  // 벽돌 뒷면을 지나 이만큼 더 옆 위치 유지 후 넘어감 (로봇 뒤가 앞보다 길어서) [m]
    double rear_clear  = 0.18;  // 벽돌 뒷면이 바퀴 축 뒤로 이만큼 지나야 지난 것으로 셈 (로봇 뒤 14cm + 여유) [m]
    double merge_dist  = 0.24;  // 앞뒤로 이보다 가까운 관측은 같은 벽돌 [m] (벽돌 간격 48cm의 절반)
    double keep_behind = 0.35;  // 벽돌 앞면이 로봇 뒤로 이만큼 지나면 지움 [m]
    double exit_after  = 0.30;  // 마지막 벽돌 뒷면을 지나 이만큼 가고 앞에 벽돌이 없으면 끝 [m]
    int    count       = 3;     // 벽돌 수. 이만큼 지나면 바로 끝 (0이면 exit_after로만)
    double max_turn    = 1.1;   // 선 방향이 로봇 방향에서 이보다 틀어지면 무시 [rad]
    double step        = 0.02;  // 경로 점 간격 [m]
    double behind      = 0.20;  // 경로를 로봇 뒤 이만큼부터 [m]
    double ahead       = 0.80;  // 경로를 로봇 앞 이만큼까지 [m]
};

// 벽돌 박스 아랫변 (YOLO 박스는 영상에서 항상 수평). 로봇 좌표 [m]
struct BrickObs {
    cv::Point2d l, r;          // 왼쪽(+y) 끝, 오른쪽 끝
    bool l_clip = false;       // 박스가 영상 왼쪽 끝에서 잘림 (벽돌이 화면 밖까지 이어짐)
    bool r_clip = false;
};

struct Target {
    cv::Point2d face;          // 벽돌 앞면 위의 점 (월드). 앞뒤 위치로만 씀
    cv::Point2d pass[2];       // 지날 점 (월드) [0]: 벽돌이 왼쪽, [1]: 오른쪽일 때
    int n[2] = {0, 0};         // 쪽별 관측 수
    double vote[2] = {0, 0};   // 쪽별 표 (근거가 좋을수록 크게, 오래된 표는 줄어듦)
    int quality[2] = {9, 9};   // 지날 점 근거 (0 선에 닿음, 1 반대쪽 선, 2 박스 끝). 작을수록 좋음
    int miss = 0;              // 보여야 하는데 안 보인 횟수
    bool passed = false;       // 뒷면이 로봇 뒤로 지나감
    bool face_rough = false;   // 앞뒤 위치를 박스 가운데로 대충 잡음 (가까운 끝이 잘렸을 때)
    int side() const { return vote[0] >= vote[1] ? 1 : -1; } // +1 벽돌이 왼쪽
    const cv::Point2d &pass_pt() const { return pass[side() > 0 ? 0 : 1]; }
};

class Planner {
public:
    Params p;

    void start(const Pose2D &pose);
    void reset();
    bool active() const { return active_; }

    // 매 프레임. lines: 보이는 차선 픽셀 (로봇 좌표, 성기게)
    // fresh: 이번에 새 탐지 결과가 왔는지 (bricks는 fresh일 때만 씀)
    // view_x0/x1: 벽돌을 탐지할 수 있는 앞쪽 거리 범위 (놓친 횟수 셈에 씀)
    void update(const std::vector<cv::Point2d> &lines, const std::vector<BrickObs> &bricks, bool fresh,
                const Pose2D &pose, double view_x0 = 0.25, double view_x1 = 0.55);

    // 계획 경로 (로봇 좌표, 가까운 점부터). 아는 벽돌이 없으면 false
    bool path(const Pose2D &pose, std::vector<cv::Point2d> &out) const;

    bool finished(const Pose2D &pose) const;

    const std::vector<Target> &targets() const { return targets_; }
    double heading() const { return h_; }
    int passed() const { return passed_; }

private:
    void lane_dir(const std::vector<cv::Point2d> &lines, const Pose2D &pose);
    void observe(const BrickObs &b, const std::vector<cv::Point2d> &lines, const Pose2D &pose);
    double offset(double s, const std::vector<std::pair<double, double>> &nodes, double l_end) const;

    bool active_ = false;
    double h_ = 0.0;            // 차선 방향 (월드)
    std::vector<Target> targets_;
    int passed_ = 0;            // 지나간 벽돌 수
    bool has_last_ = false;
    cv::Point2d last_back_;     // 마지막으로 지나간 벽돌 뒷면 (월드)
};

} // namespace construction
