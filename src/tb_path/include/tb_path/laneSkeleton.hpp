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

// lane: BEV 차선 마스크(흰색|노란색), valid: make_valid_mask 결과
// seed_hint: 직전 프레임 Result::seed (없으면 nullptr → 화면 하단 중앙에서 찾음)
Result build(const cv::Mat &lane, const cv::Mat &valid, const Params &p,
             const cv::Point *seed_hint = nullptr);

// 디버그 그림: 사각지대 회색, 도로 영역 푸른 틴트, 중심선 민트, 분기점 주황, 끝점 빨강
void draw_debug(cv::Mat &bev_bgr, const cv::Mat &valid, const Result &r, int scale);

} // namespace lane_skel
