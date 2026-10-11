// 빌드: g++ -O2 -std=c++17 -Itb_path/include -Itb_drive/include tb_path/test/keep_clear_test.cpp tb_path/src/obstacles.cpp tb_drive/src/pure_pursuit.cpp $(pkg-config --cflags --libs opencv4) -o keep_clear_test  (src에서)
// 실행: ./keep_clear_test 1 -0.10   (1=기억 벽돌로 경로 떼어 놓기, -0.10=시작 때 바퀴 축이 벽돌 앞면보다 10cm 뒤)
#include "tb_path/obstacles.hpp"
#include "tb_drive/pure_pursuit.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
// 오른쪽 벽돌(x 0~0.08, y -0.30~-0.13) 옆을 지나는 중, 경로는 y=-0.12 쪽으로 붙으려 함
int main(int argc, char **argv)
{
    const bool clamp = atoi(argv[1]);
    const double x_start = argc > 2 ? atof(argv[2]) : -0.05; // 시작 때 바퀴 축 x (벽돌 앞면 = 0)
    const double bx0 = 0.0, bx1 = 0.08, by0 = -0.30, by1 = -0.13;
    tb_drive::Follower f;
    f.p.v_max = 0.10; f.p.v_min = 0.04; f.p.w_max = 1.57; f.p.accel = 0.3; f.p.decel = 0.6; f.p.w_accel = 6.28;
    f.p.ld_min = 0.25; f.p.ld_max = 0.35; f.p.ld_time = 1.0; f.p.curv_slow = 0.10; f.p.stop_dist = 0.04;
    f.p.turn_gain = 1.3; f.latency_comp = true;
    tb_drive::Limiter lim;
    double X = x_start, Y = 0.0, TH = 0.0, minc = 1e9;
    const double dt = 1.0 / 30;
    for (int k = 0; k < 30 * 8; ++k) {
        const double t = k * dt, c = std::cos(TH), s = std::sin(TH);
        auto to_r = [&](double wx, double wy) { const double dx = wx - X, dy = wy - Y; return cv::Point2d(c * dx + s * dy, -s * dx + c * dy); };
        std::vector<cv::Point2d> path;
        for (double wx = X - 0.1; wx < X + 0.8; wx += 0.02) { // 선에 붙는 경로: 차선 중심 y=-0.12 (벽돌 안쪽 끝 바로 옆)
            const double wy = wx < X + 0.13 ? Y : -0.12;  // 화면 아랫줄(0.13) 너머부터 오른쪽
            path.push_back(to_r(wx, wy));
        }
        if (clamp) {
            obstacles::Quad q = {to_r(bx0, by1), to_r(bx1, by1), to_r(bx1, by0), to_r(bx0, by0)};
            obstacles::keep_clear(path, {q}, 0.09, 0.14, 0.105, 0.03, 0.08);
        }
        std::vector<tb_drive::Pt> pts; for (auto &q : path) pts.push_back({q.x, q.y});
        f.set_path(pts, t - 0.04);
        auto o = f.follow(t, lim.v, 0.10);
        lim.step(o.v, o.w, dt, f.p);
        if (o.tg.ok && o.v > 1e-3) { const double kk = o.w / o.v; double w = lim.v * kk; if (std::abs(w) > f.p.w_max) { w = std::copysign(f.p.w_max, w); lim.v = f.p.w_max / std::abs(kk); } lim.w = w; }
        f.record(t, lim.v, lim.w);
        X += lim.v * dt * std::cos(TH + 0.5 * lim.w * dt); Y += lim.v * dt * std::sin(TH + 0.5 * lim.w * dt); TH += lim.w * dt;
        const double c2 = std::cos(TH), s2 = std::sin(TH);
        for (double a = -0.14; a <= 0.09; a += 0.005) for (double b : {-0.105, 0.105}) {
            const double qx = X + c2 * a - s2 * b, qy = Y + s2 * a + c2 * b;
            const double dx = std::max({bx0 - qx, 0.0, qx - bx1}), dy = std::max({by0 - qy, 0.0, qy - by1});
            minc = std::min(minc, (dx == 0 && dy == 0) ? -std::min({qx - bx0, bx1 - qx, qy - by0, by1 - qy}) : std::hypot(dx, dy));
        }
        for (double b = -0.105; b <= 0.105; b += 0.005) for (double a : {-0.14, 0.09}) {
            const double qx = X + c2 * a - s2 * b, qy = Y + s2 * a + c2 * b;
            const double dx = std::max({bx0 - qx, 0.0, qx - bx1}), dy = std::max({by0 - qy, 0.0, qy - by1});
            minc = std::min(minc, (dx == 0 && dy == 0) ? -std::min({qx - bx0, bx1 - qx, qy - by0, by1 - qy}) : std::hypot(dx, dy));
        }
    }
    std::printf("clamp=%d 시작 x=%+.2f  벽돌과 최소 여유 %+.1fcm %s  (끝 y=%+.3f)\n", clamp, x_start, minc * 100, minc > 0 ? "" : "★충돌", Y);
}
