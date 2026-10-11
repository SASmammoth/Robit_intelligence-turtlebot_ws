// 공사 구간 주행 시뮬레이션 (ROS 없이): construction::Planner + tb_drive::Follower 그대로 사용
//   cd tb_path
//   g++ -O2 -std=c++17 -Iinclude -I../tb_drive/include src/construction.cpp src/lanePath.cpp src/laneSkeleton.cpp
//       ../tb_drive/src/pure_pursuit.cpp test/construction_sim.cpp $(pkg-config --cflags --libs opencv4) -o construction_sim
//   ./construction_sim                          # 기본 조건 여러 개
//   ./construction_sim R .5 side -1 img 1       # 한 조건, 궤적 그림(construction_sim.png) 저장
//
// 세계: 좌회전(중심 반지름 R, 90°) → 직선(안쪽 폭 56cm, 벽돌 3개) → 좌회전
//   벽돌 20×8cm가 선에 붙어 좌우 번갈아, 빈 공간 40cm. 첫 벽돌은 꺾임이 끝나고 d0 뒤, 좌우는 side
// 로봇: 21×23cm, 바퀴 축 기준 앞 9cm·뒤 14cm
// 관측 (실제 노드와 같은 형태):
//   - 선: BEV(로봇 앞 0.13~0.67m, 좌우 ±0.24m)에 보이는 선 점. 벽돌이 가린 곳은 안 보임
//   - 벽돌: YOLO 박스 아랫변 = 가장 가까운 모서리 줄에서, 네 모서리를 카메라 방향으로 그 줄에 투영한 좌우 끝
//     (옆면까지 잡혀서 실제보다 넓어짐). 영상 밖이면 잘림 표시, 아랫변이 영상 아래로 나가면 탐지 안 함
//   - 탐지 15Hz, 80%만, 잡음. 이동량 추정에 오차
#include "tb_path/construction.hpp"
#include "tb_drive/pure_pursuit.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

namespace {
constexpr double FRONT = 0.09, REAR = 0.14, HALF = 0.105; // 로봇 (바퀴 축 기준)
constexpr double LANE = 0.28, BW = 0.20, BD = 0.08, GAP_DEF = 0.40;
constexpr double BEV_X0 = 0.13, BEV_X1 = 0.67, BEV_Y = 0.24, CAM_X = 0.03;

struct Cfg {
    double v = 0.10;           // 공사 구간 속도 [m/s]
    double R = 0.6;            // 꺾임 중심선 반지름 [m]
    int side = 1;              // 첫 벽돌 쪽 (+1 왼쪽 = 꺾임 안쪽)
    double d0 = 0.03;          // 꺾임 끝 ~ 첫 벽돌 앞면 [m]
    double gap = GAP_DEF;      // 벽돌 사이 빈 공간
    double y0 = 0.0;           // 시작 옆 위치 [m]
    double es = 0.0, ew = 0.0; // 이동량 추정 오차 (속도, 각속도 배율)
    double noise = 1.0;        // 관측 잡음 배율
    double det = 0.8;          // 탐지율
    int seed = 1, img = 0;
    double ld = 0.17, ld_sharp = 0.12;
    double lead = -1, trans = -1, hold = -1; // <0: Planner 기본값
};

struct Rect { double x0, x1, y0, y1; };

double rect_dist(const Rect &r, double x, double y)
{
    const double dx = std::max({r.x0 - x, 0.0, x - r.x1}), dy = std::max({r.y0 - y, 0.0, y - r.y1});
    if (dx == 0 && dy == 0)
        return -std::min({x - r.x0, r.x1 - x, y - r.y0, r.y1 - y});
    return std::hypot(dx, dy);
}

// 선분 a→b가 사각형을 지나는지 (Liang–Barsky)
bool seg_hits(const Rect &r, cv::Point2d a, cv::Point2d b)
{
    double t0 = 0, t1 = 1;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double pp[4] = {-dx, dx, -dy, dy}, qq[4] = {a.x - r.x0, r.x1 - a.x, a.y - r.y0, r.y1 - a.y};
    for (int i = 0; i < 4; ++i) {
        if (std::abs(pp[i]) < 1e-12) { if (qq[i] < 0) return false; continue; }
        const double u = qq[i] / pp[i];
        if (pp[i] < 0) t0 = std::max(t0, u); else t1 = std::min(t1, u);
        if (t0 > t1) return false;
    }
    return true;
}

struct Track {
    double R, L;
    // 차선 중심에서 왼쪽 거리 (+), 그리고 진행 거리
    double lat(const cv::Point2d &p) const {
        if (p.x < 0) return R - std::hypot(p.x, p.y - R);
        if (p.x > L) return R - std::hypot(p.x - L, p.y - R);
        return p.y;
    }
    // 진행 방향으로 u만큼 간 곳의 lat 위치 (u: 꺾임 시작 = 0)
    cv::Point2d at(double u, double l) const {
        const double a1 = R * M_PI / 2;
        if (u < a1) { const double th = -M_PI / 2 + u / R; const double r = R - l; return {r * std::sin(th), R - r * std::cos(th)}; }
        if (u < a1 + L) return {u - a1, l};
        const double th = (u - a1 - L) / R; const double r = R - l;
        return {L + r * std::sin(th), R - r * std::cos(th)};
    }
    double total() const { return R * M_PI + L; }
};

struct Result { double clear = 1e9, lane = 1e9; bool finished = false, done_ok = false; int planned = 0; };

Result run(const Cfg &c, bool draw = false)
{
    std::mt19937 rng(c.seed);
    std::normal_distribution<double> nd(0, 1);
    std::uniform_real_distribution<double> ud(0, 1);

    std::vector<Rect> bricks;
    double x = c.d0;
    for (int k = 0; k < 3; ++k) {
        const int sd = (k % 2 == 0) ? c.side : -c.side;
        bricks.push_back({x, x + BD, sd > 0 ? LANE - BW : -LANE, sd > 0 ? LANE : -LANE + BW});
        x += BD + c.gap;
    }
    const Track tk{c.R, x - c.gap + 0.03};

    // 선 점 (월드): 안쪽 가장자리 ±0.28 ~ 바깥 0.30
    std::vector<cv::Point2d> line_w;
    for (double u = 0; u < tk.total(); u += 0.01)
        for (double l : {LANE, LANE + 0.01, LANE + 0.02, -LANE, -LANE - 0.01, -LANE - 0.02})
            line_w.push_back(tk.at(u, l));
    // 중심선 (벽돌 경로가 없을 때 스켈레톤 흉내)
    std::vector<cv::Point2d> center;
    for (double u = 0; u < tk.total() + 0.5; u += 0.01)
        center.push_back(tk.at(std::min(u, tk.total()), 0.0) + (u > tk.total() ? cv::Point2d(0, u - tk.total()) : cv::Point2d()));

    const cv::Point2d s0 = tk.at(0.0, c.y0);
    lane_path::Pose2D tru{s0.x, s0.y, -M_PI / 2}, est = tru;
    construction::Planner pl;
    if (c.lead >= 0) pl.p.first_lead = c.lead;
    if (c.trans >= 0) pl.p.trans_len = c.trans;
    if (c.hold >= 0) pl.p.hold_after = c.hold;
    pl.start(est);

    tb_drive::Follower f;
    f.p.v_max = c.v; f.p.v_min = 0.04; f.p.w_max = 150 * M_PI / 180; f.p.accel = 0.3; f.p.decel = 0.6;
    f.p.w_accel = 360 * M_PI / 180; f.p.ld_min = c.ld; f.p.ld_max = 0.35; f.p.ld_time = 1.0;
    f.p.curv_slow = 0.10; f.p.stop_dist = 0.04; f.p.ld_sharp = c.ld_sharp; f.p.sharp_turn = 60 * M_PI / 180;
    f.p.sharp_hold = 1.0; f.latency_comp = true;
    tb_drive::Limiter lim;

    Result res;
    std::vector<cv::Point2d> trail;
    const double dt = 1.0 / 30;
    bool con = true;
    size_t ci = 0;
    for (int k = 0; k < 30 * 60; ++k) {
        const double t = k * dt;
        const double ct = std::cos(tru.th), st = std::sin(tru.th);
        auto to_robot = [&](const cv::Point2d &w) {
            const double dx = w.x - tru.x, dy = w.y - tru.y;
            return cv::Point2d(ct * dx + st * dy, -st * dx + ct * dy);
        };
        auto to_world = [&](const cv::Point2d &r) { return cv::Point2d(tru.x + ct * r.x - st * r.y, tru.y + st * r.x + ct * r.y); };
        const cv::Point2d cam = to_world({CAM_X, 0});

        // ── 선 관측 (BEV 안, 벽돌에 안 가린 것)
        std::vector<cv::Point2d> lines;
        for (const auto &w : line_w) {
            const cv::Point2d q = to_robot(w);
            if (q.x < BEV_X0 || q.x > BEV_X1 || std::abs(q.y) > BEV_Y)
                continue;
            bool hid = false;
            for (const auto &r : bricks)
                if (seg_hits(r, cam, w)) { hid = true; break; }
            if (!hid)
                lines.push_back({q.x + 0.004 * c.noise * nd(rng), q.y + 0.004 * c.noise * nd(rng)});
        }

        // ── 벽돌 관측 (15Hz)
        std::vector<construction::BrickObs> obs;
        const bool fresh = k % 2 == 0;
        if (fresh)
            for (const auto &r : bricks) {
                const cv::Point2d cw[4] = {{r.x0, r.y0}, {r.x0, r.y1}, {r.x1, r.y0}, {r.x1, r.y1}};
                cv::Point2d cr[4];
                double x0 = 1e9;
                for (int i = 0; i < 4; ++i) { cr[i] = to_robot(cw[i]); x0 = std::min(x0, cr[i].x); }
                if (x0 < BEV_X0 || x0 > 1.0 || ud(rng) > c.det)
                    continue;
                double yl = -1e9, yr = 1e9;
                for (const auto &q : cr) {
                    const double y = q.y * (x0 - CAM_X) / (q.x - CAM_X);
                    yl = std::max(yl, y); yr = std::min(yr, y);
                }
                const double hw = BEV_Y + 0.4 * (x0 - BEV_X0);
                if (yr > hw || yl < -hw)
                    continue;
                construction::BrickObs o;
                o.l_clip = yl > hw; o.r_clip = yr < -hw;
                yl = std::min(yl, hw); yr = std::max(yr, -hw);
                if (yl - yr < 0.03)
                    continue;
                const double xn = x0 + 0.015 * c.noise * nd(rng);
                o.l = {xn, yl + 0.01 * c.noise * nd(rng)};
                o.r = {xn, yr + 0.01 * c.noise * nd(rng)};
                obs.push_back(o);
            }

        // ── 경로
        std::vector<cv::Point2d> path;
        if (con) {
            const size_t nt0 = pl.targets().size(); const int np0 = pl.passed();
            pl.update(lines, obs, fresh, est);
            if (draw && (pl.targets().size() != nt0 || pl.passed() != np0)) {
                std::printf("t=%.1f 로봇(%.2f,%.2f,%.0f°) 차선 %.0f° 지남=%d 벽돌:", t, tru.x, tru.y, tru.th * 57.3, pl.heading() * 57.3, pl.passed());
                for (const auto &tg : pl.targets()) std::printf(" [face %.2f,%.2f side %d n %d/%d q %d/%d pass %.2f,%.2f]", tg.face.x, tg.face.y, tg.side(), tg.n[0], tg.n[1], tg.quality[0], tg.quality[1], tg.pass_pt().x, tg.pass_pt().y);
                std::printf("\n");
            }
            if (pl.finished(est)) {
                con = false;
                res.finished = true;
                // 마지막 벽돌을 다 지났는지 (로봇 뒤 끝이 마지막 벽돌 뒷면을 지남)
                res.done_ok = tk.lat(cv::Point2d(tru.x, tru.y)) > -1 && tru.x - REAR > bricks.back().x1 - 0.02;
            }
        }
        if (con && pl.path(est, path)) {
            ++res.planned;
        } else {
            path.clear();
            while (ci + 1 < center.size() && cv::norm(center[ci + 1] - cv::Point2d(tru.x, tru.y)) <
                                                  cv::norm(center[ci] - cv::Point2d(tru.x, tru.y)))
                ++ci;
            for (size_t i = ci; i < center.size() && i < ci + 80; ++i) {
                const cv::Point2d q = to_robot(center[i]);
                if (q.x > BEV_X0)
                    path.push_back(q);
            }
        }
        std::vector<tb_drive::Pt> pts;
        for (const auto &q : path)
            pts.push_back({q.x, q.y});
        f.set_path(pts, t - 0.04);

        auto o = f.follow(t, lim.v, c.v);
        lim.step(o.v, o.w, dt, f.p);
        if (o.tg.ok && o.v > 1e-3) { // drive_node: 곡률 유지
            const double kk = o.w / o.v;
            double w = lim.v * kk;
            if (std::abs(w) > f.p.w_max) { w = std::copysign(f.p.w_max, w); lim.v = f.p.w_max / std::abs(kk); }
            lim.w = w;
        }
        f.record(t, lim.v, lim.w);

        auto step = [&](lane_path::Pose2D &p, double sv, double sw) {
            p.x += lim.v * sv * dt * std::cos(p.th + 0.5 * lim.w * sw * dt);
            p.y += lim.v * sv * dt * std::sin(p.th + 0.5 * lim.w * sw * dt);
            p.th += lim.w * sw * dt;
        };
        step(tru, 1, 1);
        step(est, 1 + c.es, 1 + c.ew);
        trail.push_back({tru.x, tru.y});

        // ── 충돌·차선 검사 (로봇 둘레 5mm 간격)
        const double c2 = std::cos(tru.th), s2 = std::sin(tru.th);
        auto corner = [&](double a, double b) { return cv::Point2d(tru.x + c2 * a - s2 * b, tru.y + s2 * a + c2 * b); };
        const cv::Point2d C[4] = {corner(FRONT, HALF), corner(FRONT, -HALF), corner(-REAR, -HALF), corner(-REAR, HALF)};
        for (int e = 0; e < 4; ++e) {
            const cv::Point2d a = C[e], b = C[(e + 1) % 4];
            const int ns = int(cv::norm(b - a) / 0.005) + 1;
            for (int i = 0; i <= ns; ++i) {
                const cv::Point2d q = a + (b - a) * (double(i) / ns);
                for (const auto &r : bricks)
                    res.clear = std::min(res.clear, rect_dist(r, q.x, q.y));
                res.lane = std::min(res.lane, LANE - std::abs(tk.lat(q)));
            }
        }
        // 나가는 꺾임을 60° 돌면 끝
        if (tru.x > tk.L && std::atan2(tru.x - tk.L, tk.R - tru.y) > M_PI / 3)
            break;
    }
    if (draw) {
        const double S = 300, ox = c.R + 0.5, oy = c.R + 0.5;
        cv::Mat im(int((c.R + 1.0) * S), int((tk.L + 2 * c.R + 1.0) * S), CV_8UC3, cv::Scalar(60, 60, 60));
        auto P = [&](cv::Point2d p) { return cv::Point(int((p.x + ox) * S), int((oy - p.y) * S)); };
        for (const auto &w : line_w)
            cv::circle(im, P(w), 1, cv::Scalar(235, 235, 235), -1);
        for (const auto &r : bricks)
            cv::rectangle(im, P({r.x0, r.y1}), P({r.x1, r.y0}), cv::Scalar(200, 220, 0), cv::FILLED);
        for (size_t i = 1; i < trail.size(); ++i)
            cv::line(im, P(trail[i - 1]), P(trail[i]), cv::Scalar(0, 220, 0), 2);
        cv::imwrite("construction_sim.png", im);
    }
    return res;
}

void print(const char *name, const Cfg &c)
{
    const Result r = run(c, c.img != 0);
    std::printf("%-30s 벽돌 여유 %5.1fcm  선 여유 %5.1fcm  끝 판단 %s%s  계획 %d프레임\n", name, r.clear * 100,
                r.lane * 100, r.finished ? "O" : "X", r.finished && !r.done_ok ? "(너무 일찍)" : "", r.planned);
}
} // namespace

int main(int argc, char **argv)
{
    Cfg c;
    bool single = false;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string k = argv[i];
        const double v = std::atof(argv[i + 1]);
        single = true;
        if (k == "v") c.v = v; else if (k == "R") c.R = v; else if (k == "side") c.side = int(v);
        else if (k == "d0") c.d0 = v; else if (k == "gap") c.gap = v; else if (k == "y0") c.y0 = v;
        else if (k == "es") c.es = v; else if (k == "ew") c.ew = v; else if (k == "noise") c.noise = v;
        else if (k == "det") c.det = v; else if (k == "seed") c.seed = int(v); else if (k == "img") c.img = int(v);
        else if (k == "ld") c.ld = v; else if (k == "ldsharp") c.ld_sharp = v;
        else if (k == "lead") c.lead = v; else if (k == "trans") c.trans = v; else if (k == "hold") c.hold = v;
        else if (k == "all") single = false;
    }
    if (single) { print("지정 조건", c); return 0; }

    struct Case { std::string name; Cfg c; };
    std::vector<Case> cases;
    for (int sd : {1, -1}) {
        const std::string s = sd > 0 ? "첫 벽돌 왼쪽" : "첫 벽돌 오른쪽";
        auto add = [&](const std::string &n, auto fn) { Cfg x = c; x.side = sd; fn(x); cases.push_back({s + " " + n, x}); };
        add("기본 R0.6", [](Cfg &) {});
        add("R0.45", [](Cfg &x) { x.R = 0.45; });
        add("R0.9", [](Cfg &x) { x.R = 0.9; });
        add("꺾임 바로 뒤(0)", [](Cfg &x) { x.d0 = 0.0; });
        add("꺾임 뒤 12cm", [](Cfg &x) { x.d0 = 0.12; });
        add("0.15m/s", [](Cfg &x) { x.v = 0.15; });
        add("빈 공간 35cm", [](Cfg &x) { x.gap = 0.35; });
        add("이동량 오차 5%", [](Cfg &x) { x.es = 0.05; x.ew = 0.05; });
        add("잡음 2배 + 탐지 60%", [](Cfg &x) { x.noise = 2.0; x.det = 0.6; });
        add("시작 옆 5cm", [](Cfg &x) { x.y0 = 0.05 * x.side; });
    }
    int bad = 0;
    for (auto &cs : cases) {
        double clear = 1e9, lane = 1e9;
        int fin = 0, early = 0;
        for (int s = 1; s <= 8; ++s) {
            cs.c.seed = s;
            const Result r = run(cs.c);
            clear = std::min(clear, r.clear); lane = std::min(lane, r.lane);
            fin += r.finished; early += r.finished && !r.done_ok;
        }
        const bool ok = clear > 0.0 && lane > 0.0 && fin == 8 && early == 0;
        bad += !ok;
        std::printf("%-34s 벽돌 여유(최소) %5.1fcm  선 여유 %5.1fcm  끝 판단 %d/8 %s %s\n", cs.name.c_str(), clear * 100,
                    lane * 100, fin, early ? "(일찍)" : "", ok ? "" : "★");
    }
    std::printf("실패 %d건\n", bad);
    return bad;
}
