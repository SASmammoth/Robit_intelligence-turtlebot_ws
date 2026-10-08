#pragma once
// YOLO 박스(원본 영상 좌표) → BEV 벽 마스크
// 박스 아랫변 = 물체가 바닥에 닿은 곳이라 BEV 변환이 맞는 유일한 부분.
// 그 아랫변을 BEV로 옮기고, 로봇에서 멀어지는 방향으로 depth_px 만큼 채운 사각형을 벽으로 씀
#include <opencv2/core.hpp>
#include <vector>

namespace obstacles {

struct Box { int x, y, w, h; };

// H: 원본 → BEV (CV_64F 3x3), wall: BEV 크기 CV_8U (여기에 255로 그림)
// margin_px: 박스 좌우로 더 넓히는 폭 (탐지 박스 오차 + 로봇 폭 여유)
// 반환: 실제로 그린 개수 (BEV 밖이면 안 그림)
int draw_boxes(cv::Mat &wall, const cv::Mat &H, const std::vector<Box> &boxes,
               double depth_px, double margin_px);

} // namespace obstacles
