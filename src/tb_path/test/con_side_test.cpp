// 공사 구간: 벽돌이 화면 밖 선(왼쪽)에 붙어 있고 흰 선만 오른쪽에 보일 때 (claude10·11 영상)
//   실제 BEV 설정(top_y 45, top_w 32, bot_w 100, lane_w_px 172, depth 380, bottom 190)으로 재현
//   빌드: cd tb_path && g++ -O2 -std=c++17 -Iinclude test/con_side_test.cpp src/laneSkeleton.cpp src/lanePath.cpp \
//         src/obstacles.cpp $(pkg-config --cflags --libs opencv4) -o con_side_test
//   실행: BYLANE=1 ./con_side_test 0 [벽돌 앞면 거리 m] [흰선 y m] [벽돌 안쪽 끝 y m]
//         BYLANE=1 → Tracker::p.straight_by_lane (path.con_straight_by_lane). 없으면 예전 방식
//   (흰선 -0.20, 벽돌 안쪽 +0.10, 앞면 0.36m: 예전 방식은 벽돌 앞 빈 곳 → 왼쪽으로 감)
#include "tb_path/laneSkeleton.hpp"
#include "tb_path/lanePath.hpp"
#include "tb_path/obstacles.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char **argv)
{
    const bool seal = false; (void)argc; // 첫 인자는 예전 실험용 (무시)
    const double xf = argc > 2 ? atof(argv[2]) : 0.42;   // 벽돌 앞면 (로봇 기준 앞)
    const double wy = argc > 3 ? atof(argv[3]) : -0.20;  // 흰 선 (오른쪽)
    const double yin = argc > 4 ? atof(argv[4]) : 0.10;  // 벽돌 안쪽 끝 (왼쪽 선에 붙음)
    const double depth = 0.13, margin = 0.05;

    const float w = 640, h = 480, cx = w / 2, ty = h * 0.45f, by = h - 1, tw = w * 0.16f, bw = w * 0.5f;
    const std::vector<cv::Point2f> src = {{cx - tw, ty}, {cx + tw, ty}, {cx + bw, by}, {cx - bw, by}};
    const float S = 320, m = S * 0.25f;
    const std::vector<cv::Point2f> dst = {{m, 0}, {S - m, 0}, {S - m, S}, {m, S}};
    const cv::Mat M = cv::getPerspectiveTransform(src, dst);
    const cv::Mat valid = lane_skel::make_valid_mask(M, cv::Size(640, 480), cv::Size(320, 320));

    lane_path::Geo g;
    g.bev_size = 320;
    g.lane_w_px = 172;
    g.m_per_px_x = 0.26 / 172;
    g.m_per_px_y = 0.38 / 320;
    g.bottom_m = 0.19;

    cv::Mat white = cv::Mat::zeros(320, 320, CV_8U), yellow = white.clone();
    const cv::Point2d a = lane_path::robot_to_bev({0.0, wy}, g), b = lane_path::robot_to_bev({1.0, wy}, g);
    cv::line(white, cv::Point(a), cv::Point(b), 255, int(0.02 / g.mx()));
    white &= valid;
    const cv::Mat lines = white | yellow;

    // 벽돌 벽 (path_node 공사 구간처럼 앞뒤 depth, 안쪽 margin, 선 쪽은 화면 밖까지)
    cv::Mat brick = cv::Mat::zeros(320, 320, CV_8U);
    std::vector<cv::Point> poly;
    for (auto q : {cv::Point2d(xf, yin - margin), cv::Point2d(xf + depth, yin - margin), cv::Point2d(xf + depth, 1.0), cv::Point2d(xf, 1.0)})
        poly.push_back(cv::Point(lane_path::robot_to_bev(q, g)));
    cv::fillPoly(brick, std::vector<std::vector<cv::Point>>{poly}, 255);

    const cv::Mat wall = lines | brick;
    lane_skel::Params sp;
    sp.lane_w = g.lane_w_px;
    sp.dead_end_len = int(0.25 / g.my());
    const auto sk = lane_skel::build(wall, valid, sp);
    lane_path::Tracker tr;
    tr.p.straight_by_lane = getenv("BYLANE") != nullptr;
    const auto o = tr.update(sk, sp.scale, white, yellow, valid, g, lane_path::Turn::Straight, {});

    // 벽돌 앞면 바로 앞 줄에서 경로 y, 경로가 벽돌 쪽(왼쪽)으로 가장 많이 간 y
    double y_face = 99, best = 1e9, maxy = -1;
    for (const auto &q : o.path)
    {
        if (std::abs(q.x - (xf - 0.03)) < best) best = std::abs(q.x - (xf - 0.03)), y_face = q.y;
        maxy = std::max(maxy, q.y);
    }
    const bool ok = maxy < yin - margin && o.path.size() >= 2;
    std::printf("seal=%d face=%.2fm white=%+.2f brick_in=%+.2f  path n=%zu y@face=%+.3f max_y=%+.3f  %s\n", seal, xf, wy, yin,
                o.path.size(), y_face, maxy, ok ? "OK (벽돌 오른쪽)" : "FAIL (벽돌 쪽/왼쪽으로 감)");
    cv::Mat view(320, 320, CV_8UC3, cv::Scalar(70, 70, 70));
    lane_skel::draw_debug(view, valid, sk, sp.scale);
    view.setTo(cv::Scalar(0, 0, 200), brick);
    view.setTo(cv::Scalar(235, 235, 235), white);
    lane_path::draw_path(view, o, g);
    cv::imwrite(std::string("side_") + (seal ? "seal_" : "base_") + std::to_string(int(xf * 100)) + ".png", view);
    return ok ? 0 : 1;
}
