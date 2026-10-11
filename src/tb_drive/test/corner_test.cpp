// ROS 없이: g++ -std=c++17 -Iinclude test/corner_test.cpp src/pure_pursuit.cpp -o corner_test && ./corner_test
#include "tb_drive/pure_pursuit.hpp"
#include <cmath>
#include <cstdio>
using namespace tb_drive;
// 앞 x0까지 heading h0로 오다가(곡선 꼬리) 이후 직진
static std::vector<Pt> path(double turn_deg, double at_x) {
    std::vector<Pt> p; double x = 0.02, y = 0, h = -turn_deg * M_PI / 180; // 꼬리: 비스듬히 시작
    while (x < at_x) { p.push_back({x, y}); h += (turn_deg * M_PI / 180) * 0.02 / at_x; x += 0.02 * std::cos(h); y += 0.02 * std::sin(h); }
    for (int i = 0; i < 25; ++i) { p.push_back({x, y}); x += 0.02; }
    return p;
}
static std::vector<Pt> sharp(double ang_deg, double at_x) {   // 직진하다 at_x에서 급히 꺾임
    std::vector<Pt> p; double x = 0.02, y = 0;
    for (; x < at_x; x += 0.02) p.push_back({x, y});
    const double a = ang_deg * M_PI / 180;
    for (int i = 1; i < 20; ++i) p.push_back({at_x + 0.02 * i * std::cos(a), 0.02 * i * std::sin(a)});
    return p;
}
int main() {
    Pt c; double t;
    for (double mx : {0.0, 0.20}) {
        std::printf("min_x=%.2f\n", mx);
        bool a = find_corner(path(80, 0.18), 0.6, c, t, mx);
        std::printf("  사각지대 안의 커브 꼬리 → 직진 : %s\n", a ? "코너 검출 (잘못된 피봇)" : "검출 안 됨 (정상)");
        bool b = find_corner(sharp(70, 0.32), 0.6, c, t, mx);
        std::printf("  앞 0.32m에서 70° 급히 꺾임   : %s", b ? "코너 검출" : "검출 안 됨");
        if (b) std::printf(" (x=%.2f, %.0f°)", c.x, t * 180 / M_PI);
        std::printf("\n");
    }
}
