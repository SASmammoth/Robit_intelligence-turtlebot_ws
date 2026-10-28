#pragma once
// BEV 차선 마스크 → 도로 영역 → 중심선 스켈레톤
// ROS 의존성 없음 (OpenCV만 사용)
#include <opencv2/core.hpp>
#include <vector>

namespace lane_skel {

struct Params {
    int lane_w         = 160;  // 도로 폭: 두 차선 안쪽 간격 (BEV px)
    int seal_len       = 30;   // 보이는 곳에서 끊긴 차선 끝 연장 길이 (BEV px), lane_w/3 이하
    int prune_len      = 20;   // 이보다 짧은 잔가지는 삭제 (BEV px)
    int max_lane_thick = 20;   // 이보다 두꺼운 덩어리의 끝은 연장하지 않음 (BEV px)
    int scale          = 2;    // 내부 처리 축소 배율 (2 → 320x320을 160x160에서 처리)
    int dead_end_len   = 0;    // 끝이 벽에 막힌(화면 끝·사각지대로 안 열린) 가지가 이보다 짧으면 삭제 (BEV px, 0이면 끔)
};

struct Result {
    bool ok = false;
    cv::Mat region;                    // 도로 영역 (BEV 해상도, 0/255)
    cv::Mat skel;                      // 중심선 (1/scale 해상도, 0/255)
    std::vector<cv::Point> ends;       // 스켈레톤 끝점 (BEV 좌표)
    std::vector<cv::Point> junctions;  // 스켈레톤 분기점 (BEV 좌표)
    cv::Point seed{-1, -1};            // 로봇에 가장 가까운 중심선 점 (BEV 좌표), 다음 프레임 시드용
};

// BEV에서 실제로 보이는 영역(255)과 사각지대(0)
cv::Mat make_valid_mask(const cv::Mat &M, const cv::Size &src, const cv::Size &dst, int erode_px = 5);

// 길이가 max_len 미만인 조각(점선 한 칸)을 마스크에서 제거
void remove_dashes(cv::Mat &mask, int max_len);

// 점선 잇기: 길이가 max_piece_m 이하인 조각끼리, 같은 줄 위에 있고 끝점 사이가 max_gap_m 이하면
// 끝점을 thick_px 굵기 선으로 이어 실선으로 만듦. mpx, mpy = BEV 1px당 가로·세로 거리 [m]
//   같은 줄: 두 조각 방향 차이와 잇는 선의 방향 차이가 max_angle_deg 이하,
//           한 조각 중심이 다른 조각 축에서 옆으로 max_offset_m 이하
// 반환: 이은 선 개수
int bridge_dashes(cv::Mat &mask, double mpx, double mpy, double max_piece_m, double max_gap_m,
                  double max_angle_deg, double max_offset_m, int thick_px);

// lane: BEV 차선 마스크(흰색|노란색), valid: make_valid_mask 결과
// seed_hint: 직전 프레임 Result::seed (없으면 nullptr → 화면 하단 중앙에서 찾음)
// anchor_hint: 경로 시작점을 고를 기준 (BEV 좌표, 없으면 화면 하단 중앙 = 로봇 정면).
//   커브에서는 하단 중앙이 차선 밖에 놓여 선 너머 중심선이 더 가까워지므로,
//   기억 경로로 예측한 점을 주면 그 근처 중심선에서 시작함
Result build(const cv::Mat &lane, const cv::Mat &valid, const Params &p,
             const cv::Point *seed_hint = nullptr, const cv::Point *anchor_hint = nullptr);

// 디버그 그림: 사각지대 회색, 도로 영역 푸른 틴트, 중심선 민트, 분기점 주황, 끝점 빨강
void draw_debug(cv::Mat &bev_bgr, const cv::Mat &valid, const Result &r, int scale);

} // namespace lane_skel
