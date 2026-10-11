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
// lines: 차선(흰·노랑) 마스크. 주면 벽돌 양 끝과 차선 사이 gap_px 이하 틈을 막음
//   (YOLO 박스는 항상 수평이라 BEV 벽도 수평. 곡선에서 비스듬히 선에 붙은 실제 벽돌과
//    선 사이에 삼각형 틈이 생겨 스켈레톤이 그리로 빠짐. 로봇(21cm)이 못 지나는 틈은 벽으로 봐도 됨)
// edge_px: 벽돌 한쪽 끝에 선이 없고 화면 끝에서 이 안이면 화면 끝까지 막음 (0이면 안 함)
//   (공사 구간은 차폭 56cm > 화면 48cm라 벽돌이 붙은 선이 화면 밖에 있을 때가 많음.
//    안 막으면 스켈레톤이 벽돌과 화면 끝 사이로 빠짐)
// width_px: 벽돌 실제 가로. 벽돌이 한쪽 선에 닿았으면 반대쪽 끝을 "선에서 width + margin"까지로 줄임
//   (박스는 옆면·곡선 때문에 실제보다 넓어 지나갈 쪽 길을 막음). 0이면 박스 그대로
//   edge_px는 양쪽 다 선에 안 닿았을 때만 씀
// keep_far: false면 BEV 위로 벗어난 먼 벽돌은 그리지 않음 (공사 구간: 선이 안 보여 붙은 쪽을 잘못 판단함)
// attach_px: 한쪽 선에 닿은 벽돌은 그쪽을 벽돌의 모든 줄에서 이 거리 안의 선까지 막음
//   (곡선에서 수평 박스 아래쪽 줄과 선 사이가 gap보다 벌어져 생기는 주머니 막기)
// 반환: 실제로 그린 개수 (BEV 밖이면 안 그림)
int draw_boxes(cv::Mat &wall, const cv::Mat &H, const std::vector<Box> &boxes,
               double depth_px, double margin_px,
               const cv::Mat *lines = nullptr, double gap_px = 0.0, double edge_px = 0.0,
               double width_px = 0.0, double attach_px = 0.0, bool keep_far = true);

// 벽돌에 가려 끊긴 선을 끝 방향 그대로 이어 그림 (공사 구간)
// lines: 차선 마스크, bricks: 벽돌 벽만 그린 마스크, out: 이은 부분을 그릴 곳
// 선의 먼 쪽(위쪽) 끝에서 probe_px 안에 벽돌이 있을 때만 len_px만큼 이음 (곡선 끝을 잘못 잇지 않게)
// 반환: 이은 선 수
int extend_occluded(const cv::Mat &lines, const cv::Mat &bricks, cv::Mat &out,
                    double probe_px, double len_px, int thick);

} // namespace obstacles
