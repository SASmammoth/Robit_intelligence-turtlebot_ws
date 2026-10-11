// 공사 구간 주행 시뮬레이션 (ROS 없이): construction::Planner + tb_drive::Follower 그대로 사용
//   cd tb_path
//   g++ -O2 -std=c++17 -Iinclude -I../tb_drive/include src/construction.cpp src/lanePath.cpp src/laneSkeleton.cpp
//       ../tb_drive/src/pure_pursuit.cpp test/construction_sim.cpp $(pkg-config --cflags --libs opencv4) -o construction_sim
//   ./construction_sim                      # 기본 조건 여러 개
//   ./construction_sim v .15 seed 3 img 1   # 한 조건, 궤적 그림(construction_sim.png) 저장
//
// 세계: 차선이 +x 방향, 안쪽 폭 56cm(y = ±0.28). 벽돌 20×8cm가 선에 붙어 좌우 번갈아, 빈 공간 40cm
// 로봇: 21×23cm, 바퀴 축 기준 앞 9cm·뒤 14cm. 카메라는 로봇 앞 0.13~1.0m, 좌우는 멀수록 넓게 봄
// 관측: 벽돌 앞면 가운데 (앞뒤 ±2cm, 좌우 ±1cm 잡음, 프레임마다 80%만 탐지). 이동량 추정에 오차
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
constexpr double LANE = 0.28, BW = 0.20, BD = 0.08;

struct Cfg {
    double v = 0.10;          // 공사 구간 속도 [m/s]
    double gap = 0.40;        // 벽돌 사이 빈 공간
    int n = 4;                // 벽돌 수
    double y0 = 0.0, th0 = 0; // 시작 옆 위치 [m], 방향 [deg] (좌회전을 막 끝낸 상태 흉내)
    double hint_err = 0;      // 처음 차선 방향 추정 오차 [deg]
    double es = 0.0, ew = 0.0;// 이동량 추정 오차 (속도, 각속도 배율)
    double noise = 1.0;       // 관측 잡음 배율
    int seed = 1, img = 0;
    double ld = 0.17, ld_sharp = 0.12;
};

struct Rect { double x0, x1, y0, y1; };

// 점과 축 정렬 사각형의 거리 (안이면 음수)
double rect_dist(const Rect &r, double x, double y)
{
    const double dx = std::max({r.x0 - x, 0.0, x - r.x1}), dy = std::max({r.y0 - y, 0.0, y - r.y1});
    if (dx == 0 && dy == 0)
        return -std::min({x - r.x0, r.x1 - x, y - r.y0, r.y1 - y});
    return std::hypot(dx, dy);
}

struct Result { double clear = 1e9, lane = 1e9, max_head = 0; bool finished = false; int frames_planned = 0; };

Result run(const Cfg &c, std::vector<cv::Point2d> *trail = nullptr)
{
    std::mt19937 rng(c.seed);
    std::normal_distribution<double> nd(0, 1);
    std::uniform_real_distribution<double> ud(0, 1);

    std::vector<Rect> bricks;
    std::vector<int> side;
    for (int k = 0; k < c.n; ++k) {
        const int sd = k % 2 == 0 ? 1 : -1;
        const double xc = 0.40 + k * (c.gap + BD);
        bricks.push_back({xc - BD / 2, xc + BD / 2, sd > 0 ? LANE - BW : -LANE, sd > 0 ? LANE : -LANE + BW});
        side.push_back(sd);
    }

    lane_path::Pose2D tru{-0.9, c.y0, c.th0 * M_PI / 180}, est = tru;
    construction::Planner pl;
    pl.start((c.hint_err) * M_PI / 180, {est.x, est.y - c.y0}); // 차선 중심 위의 점 (시작 때 옆 위치는 대충 앎)

    tb_drive::Follower f;
    f.p.v_max = c.v; f.p.v_min = 0.04; f.p.w_max = 150 * M_PI / 180; f.p.accel = 0.3; f.p.decel = 0.6;
    f.p.w_accel = 360 * M_PI / 180; f.p.ld_min = c.ld; f.p.ld_max = 0.35; f.p.ld_time = 1.0;
    f.p.curv_slow = 0.10; f.p.stop_dist = 0.04; f.p.ld_sharp = c.ld_sharp; f.p.sharp_turn = 60 * M_PI / 180;
    f.p.sharp_hold = 1.0; f.latency_comp = true;
    tb_drive::Limiter lim;

    Result res;
    const double dt = 1.0 / 30;
    const double x_end = bricks.back().x1 + 0.8;
    for (int k = 0; k < 30 * 90 && tru.x < x_end; ++k) {
        const double t = k * dt;
        const double ct = std::cos(tru.th), st = std::sin(tru.th);
        auto to_robot = [&](double wx, double wy) {
            const double dx = wx - tru.x, dy = wy - tru.y;
            return cv::Point2d(ct * dx + st * dy, -st * dx + ct * dy);
        };

        // ── 관측: 벽돌 앞면 가운데
        for (size_t b = 0; b < bricks.size(); ++b) {
            const cv::Point2d q = to_robot(bricks[b].x0, side[b] * (LANE - BW / 2));
            const bool in_view = q.x > 0.13 && q.x < 1.0 && std::abs(q.y) < 0.20 + 0.35 * (q.x - 0.13);
            if (in_view && ud(rng) < 0.8)
                pl.observe({q.x + 0.02 * c.noise * nd(rng), q.y + 0.01 * c.noise * nd(rng)}, est);
        }

        // ── 경로: 벽돌이 있으면 계획 경로, 없으면 차선 중심선(스켈레톤 흉내)
        std::vector<cv::Point2d> path;
        if (pl.path(est, path)) {
            ++res.frames_planned;
        } else {
            for (double x = 0.15; x < 0.8; x += 0.02)
                path.push_back(to_robot(tru.x + x, 0.0));
            pl.lane_hint(path, est);
        }
        std::vector<tb_drive::Pt> pts;
        for (const auto &q : path)
            pts.push_back({q.x, q.y});
        f.set_path(pts, t - 0.04); // 영상 지연

        auto o = f.follow(t, lim.v, c.v);
        lim.step(o.v, o.w, dt, f.p);
        if (o.tg.ok && o.v > 1e-3) { // drive_node: 곡률 유지
            const double kk = o.w / o.v;
            double w = lim.v * kk;
            if (std::abs(w) > f.p.w_max) { w = std::copysign(f.p.w_max, w); lim.v = f.p.w_max / std::abs(kk); }
            lim.w = w;
        }
        f.record(t, lim.v, lim.w);

        // ── 이동 (참) / 추정 (오차)
        auto step = [&](lane_path::Pose2D &p, double sv, double sw) {
            p.x += lim.v * sv * dt * std::cos(p.th + 0.5 * lim.w * sw * dt);
            p.y += lim.v * sv * dt * std::sin(p.th + 0.5 * lim.w * sw * dt);
            p.th += lim.w * sw * dt;
        };
        step(tru, 1, 1);
        step(est, 1 + c.es, 1 + c.ew);
        if (trail) trail->push_back({tru.x, tru.y});

        // ── 충돌·차선 검사 (로봇 둘레를 5mm 간격으로)
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
                res.lane = std::min(res.lane, LANE - std::abs(q.y));
            }
        }
        if (tru.x > -0.3)
            res.max_head = std::max(res.max_head, std::abs(tru.th) * 180 / M_PI);
        if (pl.finished(est))
            res.finished = true;
    }
    if (c.img && trail) {
        const double S = 400; // px/m
        cv::Mat im(int(0.8 * S), int((x_end + 1.0) * S), CV_8UC3, cv::Scalar(60, 60, 60));
        auto P = [&](double x, double y) { return cv::Point(int((x + 1.0) * S), int((0.4 - y) * S)); };
        cv::line(im, P(-1, LANE), P(x_end, LANE), cv::Scalar(0, 210, 255), 2);
        cv::line(im, P(-1, -LANE), P(x_end, -LANE), cv::Scalar(235, 235, 235), 2);
        for (const auto &r : bricks)
            cv::rectangle(im, P(r.x0, r.y1), P(r.x1, r.y0), cv::Scalar(200, 220, 0), cv::FILLED);
        for (size_t i = 1; i < trail->size(); ++i)
            cv::line(im, P((*trail)[i - 1].x, (*trail)[i - 1].y), P((*trail)[i].x, (*trail)[i].y), cv::Scalar(0, 220, 0), 2);
        cv::imwrite("construction_sim.png", im);
    }
    return res;
}

void print(const char *name, const Cfg &c)
{
    std::vector<cv::Point2d> tr;
    const Result r = run(c, &tr);
    std::printf("%-34s 벽돌 여유 %5.1fcm  선 여유 %5.1fcm  최대 방향 %3.0f°  %s\n", name, r.clear * 100, r.lane * 100,
                r.max_head, r.finished ? "끝 판단 O" : "끝 판단 X");
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
        if (k == "v") c.v = v; else if (k == "gap") c.gap = v; else if (k == "y0") c.y0 = v;
        else if (k == "th0") c.th0 = v; else if (k == "hint") c.hint_err = v; else if (k == "es") c.es = v;
        else if (k == "ew") c.ew = v; else if (k == "noise") c.noise = v; else if (k == "seed") c.seed = int(v);
        else if (k == "img") c.img = int(v); else if (k == "ld") c.ld = v; else if (k == "ldsharp") c.ld_sharp = v;
    }
    if (single) { print("지정 조건", c); return 0; }

    // 기본 조건 여러 개 × 시드 8개 → 가장 나쁜 값
    struct Case { const char *name; Cfg c; };
    std::vector<Case> cases;
    auto add = [&](const char *n, auto f) { Cfg x; f(x); cases.push_back({n, x}); };
    add("기본 (0.10m/s)", [](Cfg &) {});
    add("0.15m/s", [](Cfg &x) { x.v = 0.15; });
    add("0.20m/s", [](Cfg &x) { x.v = 0.20; });
    add("빈 공간 35cm", [](Cfg &x) { x.gap = 0.35; });
    add("진입 방향 15° 틀어짐 + 옆 5cm", [](Cfg &x) { x.th0 = 15; x.y0 = 0.05; x.hint_err = 10; });
    add("이동량 오차 5%", [](Cfg &x) { x.es = 0.05; x.ew = 0.05; });
    add("관측 잡음 2배", [](Cfg &x) { x.noise = 2.0; });
    int bad = 0;
    for (auto &cs : cases) {
        double clear = 1e9, lane = 1e9, head = 0;
        bool fin = true;
        for (int s = 1; s <= 8; ++s) {
            cs.c.seed = s;
            const Result r = run(cs.c);
            clear = std::min(clear, r.clear); lane = std::min(lane, r.lane);
            head = std::max(head, r.max_head); fin = fin && r.finished;
        }
        const bool ok = clear > 0.0;
        bad += !ok;
        std::printf("%-34s 벽돌 여유(최소) %5.1fcm  선 여유 %5.1fcm  최대 방향 %3.0f°  끝 판단 %s  %s\n", cs.name,
                    clear * 100, lane * 100, head, fin ? "O" : "X", ok ? "" : "★충돌");
    }
    std::printf("충돌 %d건\n", bad);
    return bad;
}
