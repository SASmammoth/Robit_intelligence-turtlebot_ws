// 빌드(src에서): g++ -O2 -std=c++17 -Itb_path/include -Itb_drive/include tb_path/test/fillet_test.cpp tb_path/src/lanePath.cpp tb_path/src/laneSkeleton.cpp tb_drive/src/pure_pursuit.cpp $(pkg-config --cflags --libs opencv4) -o fillet_test
// 실행: ./fillet_test 0.15 0.2 0.17   (반지름 m, 속도 m/s, ld_min m)
// 90° 좌회전(차폭 26cm)에서 경로 모서리 둥글게(fillet) 반지름별 차선 이탈
#include "tb_path/lanePath.hpp"
#include "tb_drive/pure_pursuit.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
int main(int argc, char **argv)
{
    const double R = atof(argv[1]), v = argc > 2 ? atof(argv[2]) : 0.20, ldmin = argc > 3 ? atof(argv[3]) : 0.17;
    const double HW = 0.13; // 차선 반폭
    // 중심선: (-0.6,0)→(0.6,0)→(0.6,0.8)
    std::vector<cv::Point2d> cl;
    for (double x = -0.6; x < 0.6; x += 0.005) cl.push_back({x, 0});
    for (double y = 0; y <= 0.8; y += 0.005) cl.push_back({0.6, y});
    tb_drive::Follower f;
    f.p.v_max = v; f.p.v_min = 0.04; f.p.w_max = 1.57; f.p.accel = 0.3; f.p.decel = 0.6; f.p.w_accel = 6.28;
    f.p.ld_min = ldmin; f.p.ld_max = 0.35; f.p.ld_time = 1.0; f.p.curv_slow = 0.10; f.p.stop_dist = 0.04; f.latency_comp = true;
    tb_drive::Limiter lim;
    double X = -0.5, Y = 0, TH = 0, worst = 0;
    size_t ci = 0;
    const double dt = 1.0 / 30;
    for (int k = 0; k < 30 * 12 && !(Y > 0.6); ++k) {
        const double t = k * dt, c = std::cos(TH), s = std::sin(TH);
        while (ci + 1 < cl.size() && std::hypot(cl[ci + 1].x - X, cl[ci + 1].y - Y) < std::hypot(cl[ci].x - X, cl[ci].y - Y)) ++ci;
        std::vector<cv::Point2d> path;
        for (size_t i = ci; i < cl.size() && i < ci + 150; i += 4) { // 앞 0.75m (사각지대는 지난 경로로 채운 것으로 봄)
            const double dx = cl[i].x - X, dy = cl[i].y - Y; path.push_back({c * dx + s * dy, -s * dx + c * dy});
        }
        if (R > 0) lane_path::fillet_corners(path, R, 45 * M_PI / 180);
        std::vector<tb_drive::Pt> pts; for (auto &q : path) pts.push_back({q.x, q.y});
        f.set_path(pts, t - 0.04);
        auto o = f.follow(t, lim.v, v);
        lim.step(o.v, o.w, dt, f.p);
        if (o.tg.ok && o.v > 1e-3) { const double kk = o.w / o.v; double w = lim.v * kk; if (std::abs(w) > f.p.w_max) { w = std::copysign(f.p.w_max, w); lim.v = f.p.w_max / std::abs(kk); } lim.w = w; }
        f.record(t, lim.v, lim.w);
        X += lim.v * dt * std::cos(TH + 0.5 * lim.w * dt); Y += lim.v * dt * std::sin(TH + 0.5 * lim.w * dt); TH += lim.w * dt;
        // 로봇 둘레 점의 차선 중심선까지 거리 (차선 밖이면 > 0.13)
        const double c2 = std::cos(TH), s2 = std::sin(TH);
        for (double a : {-0.14, 0.09}) for (double b : {-0.105, 0.105}) {
            const double qx = X + c2 * a - s2 * b, qy = Y + s2 * a + c2 * b;
            double dmin = 1e9; for (auto &p : cl) dmin = std::min(dmin, std::hypot(p.x - qx, p.y - qy));
            worst = std::max(worst, dmin);
        }
    }
    std::printf("R=%.2f v=%.2f ld_min=%.2f  몸통 최대 중심선 거리 %.1fcm  (차선 반폭 13cm, 여유 %+.1fcm)%s\n", R, v, ldmin, worst * 100,
                (HW - worst) * 100, worst > HW ? " ★이탈" : "");
}
