// 공사 구간 "벽돌 옆에서 차선 쪽으로 붙기" (path.con_line_hug) 시험
//   빌드: cd tb_path && g++ -O2 -std=c++17 -Iinclude test/hug_test.cpp src/laneSkeleton.cpp src/lanePath.cpp
//         src/obstacles.cpp $(pkg-config --cflags --libs opencv4) -o hug_test && ./hug_test
#include "tb_path/lanePath.hpp"
#include "tb_path/obstacles.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <cstdio>

namespace {
int fails = 0;
void check(bool ok, const char *what)
{
    std::printf("  %-54s %s\n", what, ok ? "OK" : "FAIL");
    if (!ok)
        ++fails;
}
lane_path::Geo geo()
{
    lane_path::Geo g;
    g.bev_size = 320;
    g.lane_w_px = 172;
    g.m_per_px_x = 0.26 / 172;
    g.m_per_px_y = 0.38 / 320;
    g.bottom_m = 0.19;
    return g;
}
void fill(cv::Mat &m, double x0, double x1, double y0, double y1, const lane_path::Geo &g)
{
    std::vector<cv::Point> px;
    for (auto q : {cv::Point2d(x0, y0), cv::Point2d(x1, y0), cv::Point2d(x1, y1), cv::Point2d(x0, y1)})
        px.push_back(cv::Point(lane_path::robot_to_bev(q, g)));
    cv::fillConvexPoly(m, px, 255);
}
std::vector<cv::Point2d> straight(double y)
{
    std::vector<cv::Point2d> p;
    for (double x = 0.0; x <= 0.60; x += 0.02)
        p.push_back({x, y});
    return p;
}
double y_at(const std::vector<cv::Point2d> &p, double x)
{
    double best = 1e9, y = 0;
    for (const auto &q : p)
        if (std::abs(q.x - x) < best)
            best = std::abs(q.x - x), y = q.y;
    return y;
}
} // namespace

int main()
{
    const lane_path::Geo g = geo();
    obstacles::HugParams hp; // keep 0.14, zone 0.25, max 0.12
    // 빈 곳 36cm (실측 차폭 56 - 벽돌 20): 오른쪽 흰선 안쪽 끝 y=-0.22, 왼쪽 벽돌 안쪽 끝 y=+0.13 → 가운데 ≈ -0.05
    cv::Mat lines = cv::Mat::zeros(320, 320, CV_8U), bricks = lines.clone();
    cv::line(lines, cv::Point(lane_path::robot_to_bev({0.0, -0.23}, g)), cv::Point(lane_path::robot_to_bev({1.0, -0.23}, g)), 255,
             int(0.02 / g.mx()));
    fill(bricks, 0.30, 0.45, 0.13, 0.60, g);

    std::printf("[왼쪽 벽돌, 오른쪽 선]\n");
    auto p = straight(-0.05);
    const int n = obstacles::hug_line(p, lines, bricks, g, hp);
    const double y_side = y_at(p, 0.38), y_far = y_at(p, 0.60), y0 = y_at(p, 0.0);
    std::printf("  옮긴 점 %d, 벽돌 옆 y=%+.3f (목표 %+.3f), 끝 y=%+.3f, 시작 y=%+.3f\n", n, y_side, -0.22 + hp.keep, y_far, y0);
    check(y_side < -0.05 - 0.02, "벽돌 옆에서 선 쪽(오른쪽)으로 옮김");
    check(y_side > -0.22 + hp.keep - 0.01, "선에 keep보다 가까이 가지 않음");
    check(std::abs(y0 + 0.05) < 1e-6, "로봇 바로 앞(시작점)은 그대로");

    std::printf("[벽돌 없음]\n");
    p = straight(-0.05);
    check(obstacles::hug_line(p, lines, cv::Mat::zeros(320, 320, CV_8U), g, hp) == 0, "벽돌이 없으면 안 옮김");

    std::printf("[벽돌이 zone 밖]\n");
    p = straight(-0.05);
    cv::Mat far_b = cv::Mat::zeros(320, 320, CV_8U);
    fill(far_b, 0.30, 0.45, 0.22, 0.60, g); // 27cm
    check(obstacles::hug_line(p, lines, far_b, g, hp) == 0, "벽돌이 25cm보다 멀면 안 옮김");

    std::printf("[오른쪽 벽돌, 왼쪽 선 (대칭)]\n");
    cv::Mat l2 = cv::Mat::zeros(320, 320, CV_8U), b2 = l2.clone();
    cv::line(l2, cv::Point(lane_path::robot_to_bev({0.0, 0.23}, g)), cv::Point(lane_path::robot_to_bev({1.0, 0.23}, g)), 255,
             int(0.02 / g.mx()));
    fill(b2, 0.30, 0.45, -0.60, -0.13, g);
    p = straight(0.05);
    obstacles::hug_line(p, l2, b2, g, hp);
    check(y_at(p, 0.38) > 0.05 + 0.02, "왼쪽으로 옮김");

    std::printf("[선이 화면 밖]\n");
    p = straight(-0.05);
    check(obstacles::hug_line(p, cv::Mat::zeros(320, 320, CV_8U), bricks, g, hp) == 0, "붙을 선이 안 보이면 안 옮김");

    std::printf("%s\n", fails ? "FAIL" : "ALL OK");
    return fails ? 1 : 0;
}
