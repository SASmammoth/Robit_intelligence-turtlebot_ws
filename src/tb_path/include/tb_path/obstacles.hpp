#pragma once
// YOLO 박스(원본 영상 좌표) → BEV 벽 마스크
// 박스 아랫변 = 물체가 바닥에 닿은 곳이라 BEV 변환이 맞는 유일한 부분.
// 그 아랫변을 BEV로 옮기고, 로봇에서 멀어지는 방향으로 depth_px 만큼 채운 사각형을 벽으로 씀
#include <opencv2/core.hpp>
#include "tb_path/lanePath.hpp"
#include <array>
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
// off_screen: 박스 아랫변이 화면 한쪽 옆으로 나가면 그쪽 (화면 밖) 선에 붙은 벽돌로 봄.
//   반대쪽 선에 닿아 보여도(곡선에서 먼 줄만 닿음) 그쪽 틈은 막지 않음 (공사 구간)
// sides: 주면 박스마다 붙은 쪽 (+1 오른쪽 선, -1 왼쪽 선, 0 모름·안 그림). 로봇이 틀어져 있어도
//   "어느 선에 붙었나"로 정하므로, 로봇 정면 기준 좌우보다 정확함 (곡선 진입에서 오른쪽 선 벽돌이 화면 왼쪽에 보임)
// 반환: 실제로 그린 개수 (BEV 밖이면 안 그림)
int draw_boxes(cv::Mat &wall, const cv::Mat &H, const std::vector<Box> &boxes,
               double depth_px, double margin_px,
               const cv::Mat *lines = nullptr, double gap_px = 0.0, double edge_px = 0.0,
               double width_px = 0.0, double attach_px = 0.0, bool keep_far = true, bool off_screen = false,
               std::vector<int> *sides = nullptr);

// 벽돌에 가려 끊긴 선을 끝 방향 그대로 이어 그림 (공사 구간)
// lines: 차선 마스크, bricks: 벽돌 벽만 그린 마스크, out: 이은 부분을 그릴 곳
// 선의 먼 쪽(위쪽) 끝에서 probe_px 안에 벽돌이 있을 때만 len_px만큼 이음 (곡선 끝을 잘못 잇지 않게)
// 반환: 이은 선 수
int extend_occluded(const cv::Mat &lines, const cv::Mat &bricks, cv::Mat &out,
                    double probe_px, double len_px, int thick);

// 로봇 옆을 지나는 중인 벽돌(로봇 좌표 사각형)에서 경로를 떼어 놓음 (공사 구간)
// 카메라는 로봇 앞 일정 거리부터 보므로 몸통이 벽돌 옆을 지나는 동안 벽돌이 화면에 없고,
// 그 사이 경로가 벽돌 쪽으로 바뀌면 몸통 옆이 벽돌을 침.
// 벽돌 앞면 - front ~ 뒷면 + rear + hold 구간의 경로 점은 벽돌 안쪽 끝에서 half + clear 이상 떨어지게 옆으로 밂.
// soft = true면 로봇 정면 직선(y=0)보다 반대쪽으로는 밀지 않음 = 벽돌 쪽으로 꺾지만 못하게
//   (박스가 옆면까지 잡혀 안쪽 끝이 실제보다 안쪽이면 경로를 너무 밀어냄. 지금 자리는 이미 안전하다고 봄)
// 벽돌이 로봇 가운데(y=0)에 걸쳐 있으면 어느 쪽인지 모르므로 건드리지 않음
using Quad = std::array<cv::Point2d, 4>;
void keep_clear(std::vector<cv::Point2d> &path, const std::vector<Quad> &bricks,
                double front, double rear, double half, double clear, double hold, bool soft = false);

// 같은 쪽(같은 선)에 붙은 벽돌끼리 사이를 막음 (공사 구간, r-r-l처럼 같은 쪽이 연달아 올 때)
// 앞 벽돌이 화면 아래로 빠지면 두 벽돌 사이 선 옆 빈 곳이 "길"로 보여 스켈레톤이 그리로 들어감.
// 기억 중인 지난 벽돌 + 지금 보이는 벽돌을 넣으면, 쪽마다 x 순서로 이웃한 두 벽돌 사이를
// 붙은 선 쪽으로 막는 사각형(로봇 좌표)을 돌려줌.
//   x0, x1: 벽돌 앞면·뒷면 x [m], yin: 지나갈 길 쪽 끝 y [m] (오른쪽 벽돌은 왼쪽 끝, 왼쪽 벽돌은 오른쪽 끝)
//   min_gap 이하로 겹치면 같은 벽돌(기억과 화면이 같은 것), max_gap 넘게 떨어지면 따로 피함
//   max_dy: 두 벽돌 안쪽 끝 y 차이가 이보다 크면 안 이음 (0이면 안 봄). 곡선에서는 같은 선이어도 차이가 커짐
//   막는 면은 두 벽돌 안쪽 끝을 잇는 비스듬한 사각형 (곡선에서 길 쪽으로 튀어나오지 않게)
//   margin: 안쪽 끝에서 길 쪽으로 더 막는 폭, far: 선 쪽으로 막는 폭 (화면 밖까지)
//   from_back: true면 앞 벽돌 뒷면부터 막음 (false: 앞 벽돌 앞면부터 = 벽돌 옆까지 겹쳐 막음)
struct SideBrick { double x0, x1, yin; bool right; };
std::vector<Quad> bridge_same_side(std::vector<SideBrick> bricks, double max_gap, double min_gap = 0.05,
                                   double max_dy = 0.15, double margin = 0.0, double far = 0.6, bool from_back = false);

// 공사 구간: 벽돌 옆을 지날 때 경로를 벽돌에서 떼어 반대쪽 차선 쪽으로 붙임 (path.con_line_hug)
// 스켈레톤 경로는 벽돌과 선 사이 빈 곳의 한가운데라, 벽돌 박스가 실제보다 크거나 늦게 그려지면 벽돌에 가까워짐.
// 경로 점마다 좌우 수직 방향으로 BEV를 훑어, 한쪽 zone 안에 벽돌이 있고 반대쪽에 선이 보이면
// 그 선에서 keep만큼 떨어진 곳까지(최대 max_shift) 선 쪽으로 옮김. 선이 안 보이면 안 옮김.
// 옮긴 양은 앞뒤로 부드럽게 하고, 경로 시작 taper 구간은 0에서부터 늘림 (로봇 바로 앞이 꺾이지 않게)
//   lines: 차선 마스크(BEV), bricks: 벽돌 벽만 그린 마스크(BEV). 반환: 옮긴 점 수
struct HugParams
{
    double keep = 0.14;      // 로봇 중심 ~ 선 거리 [m] (반폭 10.5cm + 여유)
    double zone = 0.25;      // 이 안에 벽돌이 있을 때만 [m]
    double max_shift = 0.12; // 한 번에 옮기는 최대 [m]
    double reach = 0.45;     // 좌우로 훑는 최대 거리 [m]
    double taper = 0.10;     // 경로 시작부터 이 길이 동안 옮김을 0 → 1로 [m]
};
int hug_line(std::vector<cv::Point2d> &path, const cv::Mat &lines, const cv::Mat &bricks,
             const lane_path::Geo &g, const HugParams &p);

} // namespace obstacles
