// ROS 없이 커브·코너 주행 시뮬레이션 (tb_drive의 Follower + Pivoter 그대로 사용)
//   g++ -O2 -std=c++17 -Iinclude test/turn_sim.cpp src/pure_pursuit.cpp -o turn_sim
//   ./turn_sim v .30 ldmin .17 bottom .02 R .03 ang 90 seed 3   # 반경 3cm 90° 꺾임 (seed: 흔들림 난수)
//   xb .075 = 경로 x를 7.5cm 가깝게 줌 (bev_bottom 실측 205를 130으로 둔 경우)
//   ldsharp 0 = 가변 lookahead 끔, pivot 0 = 피봇 끔, enter/keep = 피봇 각도
//   ./turn_sim v .30 ldmin .17 bottom .02 R .26 ang 180 pivot 0 # 피봇 끄고 반경 26cm 헤어핀
//   vcurve .20 = 코너 속도 (cstart/cfull = 시작·도달 각도 °), holdmm .15 = 급한 곳 유지를 거리로
//   tau .15 = 모터 응답 지연 (실제 v,w가 명령을 이 시정수로 따라감. 0 = 즉시)
// 90° 커브(중심선 반경 R) 주행 시뮬레이션. tb_drive의 실제 Follower/Limiter + drive_node의 곡률 유지 로직 사용
// 카메라: 로봇 기준 x ∈ [bev_bottom, bev_bottom+depth], |y| ≤ half_w 안에 보이는 중심선만 경로로 받음
#include "tb_drive/pure_pursuit.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <random>
using namespace tb_drive;

struct Cfg { int seed=1; double hold=1.0; double ldsharp=0.12, sharpturn=60; double noise=0.005; int confirm=3; double enter=100, keep=20; bool pivot=true; double ang=90; double v_max=.15, w_max_deg=150, w_acc_deg=600, ld_min=.21, ld_max=.35, ld_time=1.0, curv=.35;
             double R=.26, bottom=.205, depth=.54, half_w=.24, latency=.04; double xb=0; bool extend=false, lat_comp=false;
             double vcurve=0, cstart=20, cfull=60, holdmm=0, tau=0; };

// 트랙 중심선: (0,0)에서 +x로 1m 직진 → 반경 R 좌회전 90° → +y로 1m
static std::vector<Pt> track(double R, double ang) {
    std::vector<Pt> c;
    for (double s = 0; s < 1.0; s += .005) c.push_back({s, 0});
    const double A = ang * M_PI / 180;
    for (double a = 0; a < A; a += .005 / R) c.push_back({1.0 + R * std::sin(a), R - R * std::cos(a)});
    const Pt e{1.0 + R * std::sin(A), R - R * std::cos(A)};
    for (double s = 0; s <= 1.0; s += .005) c.push_back({e.x + s * std::cos(A), e.y + s * std::sin(A)});
    return c;
}
// 중심선에서의 부호 있는 거리 (+ = 커브 바깥쪽/오른쪽)
static double dev(const std::vector<Pt> &c, double x, double y, size_t &near) {
    double bd = 1e9; size_t bi = 0;
    for (size_t i = 0; i < c.size(); ++i) { double d = std::hypot(c[i].x - x, c[i].y - y); if (d < bd) { bd = d; bi = i; } }
    near = bi;
    const Pt a = c[bi], b = c[std::min(bi + 1, c.size() - 1)];
    const double cross = (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x);
    return cross > 0 ? -bd : bd;   // 진행방향 왼쪽(커브 안쪽)이면 음수
}
static void extend_tail(std::vector<Pt> &p, double len, double min_turn) {
    if (p.size() < 5) return;
    const Pt a = p[p.size() - 4], b = p.back();
    const double hx = b.x - a.x, hy = b.y - a.y, L = std::hypot(hx, hy);
    if (L < 1e-3) return;
    const double he = std::atan2(hy, hx), hs = std::atan2(p[1].y - p[0].y, p[1].x - p[0].x);
    if (std::abs(std::remainder(he - hs, 2 * M_PI)) < min_turn) return;
    for (int i = 1; i <= int(len / .02); ++i) p.push_back({b.x + hx / L * .02 * i, b.y + hy / L * .02 * i});
}

int main(int argc, char **argv) {
    Cfg c;
    for (int i = 1; i + 1 < argc; i += 2) {
        std::string k = argv[i]; double v = std::atof(argv[i + 1]);
        if (k == "v") c.v_max = v; else if (k == "ldmin") c.ld_min = v; else if (k == "ldmax") c.ld_max = v;
        else if (k == "curv") c.curv = v; else if (k == "wmax") c.w_max_deg = v; else if (k == "wacc") c.w_acc_deg = v;
        else if (k == "ext") c.extend = v > 0; else if (k == "lat") c.latency = v; else if (k == "comp") c.lat_comp = v > 0;
        else if (k == "R") c.R = v; else if (k == "bottom") c.bottom = v; else if (k == "ang") c.ang = v; else if (k == "noise") c.noise = v; else if (k == "confirm") c.confirm = int(v); else if (k == "enter") c.enter = v; else if (k == "keep") c.keep = v; else if (k == "pivot") c.pivot = v > 0; else if (k == "ldsharp") c.ldsharp = v; else if (k == "sharpturn") c.sharpturn = v; else if (k == "hold") c.hold = v; else if (k == "seed") c.seed = int(v); else if (k == "halfw") c.half_w = v; else if (k == "xb") c.xb = v;
        else if (k == "vcurve") c.vcurve = v; else if (k == "cstart") c.cstart = v; else if (k == "cfull") c.cfull = v;
        else if (k == "holdmm") c.holdmm = v; else if (k == "tau") c.tau = v;
    }
    const auto C = track(c.R, c.ang);
    Follower f; f.p.v_max = c.v_max; f.p.v_min = .04; f.p.w_max = c.w_max_deg * M_PI / 180; f.p.accel = .3; f.p.decel = .6;
    f.p.w_accel = c.w_acc_deg * M_PI / 180; f.p.ld_min = c.ld_min; f.p.ld_max = c.ld_max; f.p.ld_time = c.ld_time;
    f.p.curv_slow = c.curv; f.p.ld_sharp = c.ldsharp; f.p.sharp_turn = c.sharpturn * M_PI / 180; f.p.sharp_hold = c.hold; f.p.stop_dist = .04; f.latency_comp = c.lat_comp;
    f.p.v_curve = c.vcurve; f.p.curve_start = c.cstart * M_PI / 180; f.p.curve_full = c.cfull * M_PI / 180; f.p.sharp_hold_dist = c.holdmm;
    Limiter lim;
    Pivoter pv; pv.p.enter = c.enter * M_PI / 180; pv.p.keep = c.keep * M_PI / 180; pv.p.confirm = c.confirm;
    pv.p.exit = 15 * M_PI / 180; pv.p.w = 90 * M_PI / 180; pv.p.fallback = 75 * M_PI / 180;
    std::mt19937 rng(c.seed); std::normal_distribution<double> nd(0, c.noise);
    int n_pivot = 0; double pivot_total = 0; bool was_pivot = false;
    double x = 0, y = 0, th = 0, t = 0, next_cam = 0, max_out = 0, max_in = 0;
    double vr = 0, wr = 0, v_entry = 0, t_end = 0; // 실제 속도(모터 지연), 커브 진입 속도, 도착 시각
    const double dt = 1.0 / 30;
    struct Snap { double t, x, y, th; }; std::vector<Snap> hist;
    size_t near = 0;
    for (int k = 0; k < 30 * 30 && near + 5 < C.size(); ++k, t += dt) {
        hist.push_back({t, x, y, th});
        if (t >= next_cam) {           // 카메라 프레임: latency 전 자세에서 본 경로를 지금 도착시킴
            next_cam += dt;
            Snap s = hist.front();
            for (auto &h : hist) if (h.t <= t - c.latency) s = h;
            std::vector<Pt> path; bool started = false;
            size_t n0; dev(C, s.x, s.y, n0);
            for (size_t i = n0; i < C.size(); ++i) {
                const double dx = C[i].x - s.x, dy = C[i].y - s.y;
                const double rx = std::cos(s.th) * dx + std::sin(s.th) * dy, ry = -std::sin(s.th) * dx + std::cos(s.th) * dy;
                const bool vis = rx >= c.bottom && rx <= c.bottom + c.depth && std::abs(ry) <= c.half_w;
                if (vis) { if (path.empty() || std::hypot(rx - path.back().x, ry - path.back().y) >= .02) path.push_back({rx, ry}); started = true; }
                else if (started) break;
            }
            for (auto &q : path) { q.y += nd(rng); q.x -= c.xb; }
            if (c.extend) extend_tail(path, c.ld_max, .35);
            f.set_path(path, s.t);
        }
        auto o = f.follow(t, lim.v, c.v_max);
        double vd = o.v, wd = o.w; const char *why = "";
        const auto ph0 = pv.phase();
        bool piv = c.pivot && pv.step(o, lim.v, lim.w, dt, std::max(.04, .5 * c.v_max), vd, wd, why);
        if (piv && std::string(why) == "pivot") { if (!was_pivot) ++n_pivot; pivot_total += std::abs(lim.w) * dt; was_pivot = true; } else was_pivot = false;
        if (getenv("DBG") && ph0 == Pivoter::Phase::Follow && pv.phase() != Pivoter::Phase::Follow)
            std::printf("    t=%.2f 시작: %s  corner=%d turn=%.0f° x=%.2f  tg=%.0f°\n", t, pv.phase()==Pivoter::Phase::Approach?"코너":"안전장치",
                        o.has_corner, o.corner_turn*180/M_PI, o.corner.x, o.tg.ok?std::atan2(o.tg.p.y,o.tg.p.x)*180/M_PI:0.0);
        lim.step(vd, wd, dt, f.p);
        if (!piv && o.tg.ok && o.v > 1e-3) {   // drive_node: 곡률 유지
            const double kk = o.w / o.v; double w = lim.v * kk;
            if (std::abs(w) > f.p.w_max) { w = std::copysign(f.p.w_max, w); lim.v = f.p.w_max / std::abs(kk); }
            lim.w = w;
        }
        f.record(t, lim.v, lim.w);
        if (c.tau > 0) { vr += (lim.v - vr) * std::min(1.0, dt / c.tau); wr += (lim.w - wr) * std::min(1.0, dt / c.tau); }
        else { vr = lim.v; wr = lim.w; }
        x += vr * dt * std::cos(th + .5 * wr * dt); y += vr * dt * std::sin(th + .5 * wr * dt); th += wr * dt;
        const double d = dev(C, x, y, near);
        if (near < 200) v_entry = vr;                       // 커브 시작(직진 1m = 200점) 직전 속도
        if (t_end == 0 && near + 40 >= C.size()) t_end = t; // 커브 뒤 직선을 거의 다 간 시각
        if (d > max_out) max_out = d; if (d < max_in) max_in = d;
    }
    std::printf("최대 오차 %4.1fcm (안쪽 %4.1f 바깥 %4.1f)  피봇 %d회 (총 %3.0f°)  진입 %.2fm/s  %.1fs\n", std::max(max_out, -max_in) * 100, -max_in * 100, max_out * 100, n_pivot, pivot_total * 180 / M_PI, v_entry, t_end);
}
