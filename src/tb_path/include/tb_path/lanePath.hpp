#pragma once
// 스켈레톤 → 주행 경로 (로봇 좌표, m)
//  1) 분기점에서 방향 선택 (기억한 경로가 있으면 그것과 일치하는 가지, 없으면 turn 명령)
//  2) 경로 기억: 화면이 애매할 때(갈림길 입구 등) 직전 경로를 이동량만큼 옮겨서 계속 사용
//  3) 색 규칙: 기억도 없을 때 섬 모서리 색으로 목표점 결정 (좌: 흰선 왼끝, 우: 노란선 오른끝)
// ROS 의존성 없음
#include <opencv2/core.hpp>
#include <vector>
#include "tb_path/laneSkeleton.hpp"

namespace lane_path {

// BEV 픽셀 ↔ 실제 거리 변환에 필요한 값
struct Geo {
    int    bev_size      = 320;    // BEV 한 변 (px)
    int    lane_w_px     = 160;    // BEV에서 두 차선 안쪽 간격 (스켈레톤 lane_w와 같은 값)
    double m_per_px_x    = 0.26 / 160;  // 가로 1px당 거리 [m]
    double m_per_px_y    = 0.52 / 320;  // 세로 1px당 거리 [m]
    double bottom_m      = 0.15;   // 로봇 회전 중심(바퀴 축) ~ BEV 맨 아래 줄 거리 [m]

    double mx() const { return m_per_px_x; }
    double my() const { return m_per_px_y; }
};

// 로봇 좌표: x = 앞, y = 왼쪽 (m). REP-103 base_link와 같음
cv::Point2d bev_to_robot(const cv::Point2d &px, const Geo &g);
cv::Point2d robot_to_bev(const cv::Point2d &p, const Geo &g);

struct Pose2D { double x = 0, y = 0, th = 0; };   // 월드(시작 위치 기준) 자세

enum class Turn { Straight = 0, Left = 1, Right = 2 };
enum class Source { Live, Memory, ColorRule, Lost };
const char *source_name(Source s);

struct TrackerParams {
    double max_dev_m       = 0.05;  // 새 경로가 기억과 평균 이만큼 넘게 다르면 의심
    int    override_frames = 10;    // 의심이 이만큼 연속되면 새 경로를 믿음
    double mem_travel_m    = 0.60;  // 기억만으로 갈 수 있는 최대 이동 거리
    double min_len_m       = 0.10;  // 이보다 짧은 경로는 무효
    double step_m          = 0.02;  // 출력 경로 점 간격
    double lookahead_m     = 0.04;  // 분기 판단 때 각 가지를 미리 보는 길이
    double blind_max_m     = 0.25;  // 경로가 사각지대 안을 연속으로 지날 수 있는 최대 길이
};

struct Output {
    Source src = Source::Lost;
    std::vector<cv::Point2d> path;  // 로봇 좌표 (m), 가까운 점부터
};

class Tracker {
public:
    TrackerParams p;

    // sk: lane_skel::build 결과, skel_scale: lane_skel::Params::scale
    // white/yellow/valid: BEV 마스크, pose: 현재 추정 자세
    Output update(const lane_skel::Result &sk, int skel_scale,
                  const cv::Mat &white, const cv::Mat &yellow, const cv::Mat &valid,
                  const Geo &g, Turn turn, const Pose2D &pose);

    void reset() { mem_.clear(); suspicious_ = 0; }

private:
    std::vector<cv::Point2d> mem_;  // 기억한 경로 (월드 좌표)
    Pose2D mem_pose_;               // 기억을 마지막으로 갱신한 자세
    int suspicious_ = 0;
};

// BEV 디버그 영상에 경로 그리기 (Live 초록, Memory 보라, ColorRule 주황)
void draw_path(cv::Mat &bev_bgr, const Output &o, const Geo &g);

} // namespace lane_path
