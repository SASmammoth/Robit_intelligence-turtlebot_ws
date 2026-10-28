// 공사 구간 같은 쪽 벽돌 잇기 (path.con_bridge_mm) 시험: r-r-l / r-l 배치
//   빌드: cd tb_path && g++ -O2 -std=c++17 -Iinclude test/bridge_test.cpp src/laneSkeleton.cpp src/lanePath.cpp \
//         src/obstacles.cpp $(pkg-config --cflags --libs opencv4) -o bridge_test && ./bridge_test
//   1번(오른쪽)은 이미 화면 아래로 빠져 기억만 있고, 2번이 앞에 보이는 상황.
//   잇기 없이: 1·2번 사이 흰선 옆 빈 곳으로 경로가 들어갈 수 있음 / 잇기: 막힘
#include "tb_path/laneSkeleton.hpp"
#include "tb_path/lanePath.hpp"
#include "tb_path/obstacles.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <cstdio>
#include <string>

namespace {
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

void fill(cv::Mat &m, const obstacles::Quad &q, const lane_path::Geo &g)
{
    std::vector<cv::Point> px;
    for (const auto &p : q)
        px.push_back(cv::Point(lane_path::robot_to_bev(p, g)));
    cv::fillConvexPoly(m, px, 255);
}

int fails = 0;
void check(bool ok, const char *what)
{
    std::printf("  %-58s %s\n", what, ok ? "OK" : "FAIL");
    if (!ok)
        ++fails;
}

// 시나리오 하나: 흰선 wy, 1번(기억) 오른쪽, 2번 오른쪽/왼쪽(second_right), 2번 앞면 x2
// 반환: 경로가 2번 앞면 앞 구간에서 길 쪽 끝(yin)보다 선 쪽으로 들어갔는지
bool run(const char *name, bool bridge, bool second_right, double x2, double wy, double yin1)
{
    const lane_path::Geo g = geo();
    const float w = 640, h = 480, cx = w / 2, ty = h * 0.45f, by = h - 1, tw = w * 0.16f, bw = w * 0.5f;
    const std::vector<cv::Point2f> src = {{cx - tw, ty}, {cx + tw, ty}, {cx + bw, by}, {cx - bw, by}};
    const float S = 320, m = S * 0.25f;
    const std::vector<cv::Point2f> dst = {{m, 0}, {S - m, 0}, {S - m, S}, {m, S}};
    const cv::Mat M = cv::getPerspectiveTransform(src, dst);
    const cv::Mat valid = lane_skel::make_valid_mask(M, cv::Size(640, 480), cv::Size(320, 320));

    cv::Mat white = cv::Mat::zeros(320, 320, CV_8U), yellow = white.clone();
    cv::line(white, cv::Point(lane_path::robot_to_bev({0.0, wy}, g)), cv::Point(lane_path::robot_to_bev({1.0, wy}, g)),
             255, int(0.02 / g.mx()));
    white &= valid;

    // 2번 벽돌 벽 (붙은 선 쪽은 화면 밖까지, 길 쪽은 안쪽 끝 - 여유 5cm)
    const double depth = 0.15, margin = 0.05;
    const double yin2 = second_right ? yin1 : wy + 0.56 - 0.20; // 왼쪽 벽돌: 왼쪽 선(흰선 + 56cm)에서 20cm
    cv::Mat brick = cv::Mat::zeros(320, 320, CV_8U);
    if (second_right)
        fill(brick, {cv::Point2d(x2, yin2 + margin), cv::Point2d(x2 + depth, yin2 + margin), cv::Point2d(x2 + depth, -1.0), cv::Point2d(x2, -1.0)}, g);
    else
        fill(brick, {cv::Point2d(x2, yin2 - margin), cv::Point2d(x2 + depth, yin2 - margin), cv::Point2d(x2 + depth, 1.0), cv::Point2d(x2, 1.0)}, g);

    // 잇기: 1번(기억, 바퀴 축 옆 x 0.00~0.10) + 2번
    std::vector<obstacles::SideBrick> sb = {{0.00, 0.10, yin1, true}, {x2, x2 + 0.08, yin2, second_right}};
    const auto br = bridge ? obstacles::bridge_same_side(sb, 0.6, 0.05, 0.15, margin) : std::vector<obstacles::Quad>{};
    cv::Mat bm = cv::Mat::zeros(320, 320, CV_8U);
    for (const auto &q : br)
        fill(bm, q, g);

    const cv::Mat wall = white | brick | bm;
    lane_skel::Params sp;
    sp.lane_w = g.lane_w_px;
    sp.dead_end_len = int(0.25 / g.my());
    const auto sk = lane_skel::build(wall, valid, sp);
    lane_path::Tracker tr;
    tr.p.straight_by_lane = true;
    const auto o = tr.update(sk, sp.scale, white, yellow, valid, g, lane_path::Turn::Straight, {});

    double miny = 1e9;
    for (const auto &q : o.path)
        if (q.x > g.bottom_m && q.x < x2)
            miny = std::min(miny, q.y);
    const bool in_pocket = miny < yin1; // 오른쪽 벽돌 안쪽 끝보다 오른쪽 = 주머니
    std::printf("%s bridge=%d n=%zu bridges=%zu  path min y (x<%.2f)=%+.3f  yin=%+.2f  %s\n", name, bridge, o.path.size(),
                br.size(), x2, miny, yin1, o.path.empty() ? "경로 없음 (선이 화면 밖)" : in_pocket ? "→ 1번 벽돌 안쪽 끝보다 선 쪽 (r-r이면 주머니, r-l이면 정상)" : "길 쪽 유지");

    cv::Mat view(320, 320, CV_8UC3, cv::Scalar(70, 70, 70));
    lane_skel::draw_debug(view, valid, sk, sp.scale);
    view.setTo(cv::Scalar(0, 0, 200), brick);
    view.setTo(cv::Scalar(180, 60, 200), bm);
    view.setTo(cv::Scalar(235, 235, 235), white);
    lane_path::draw_path(view, o, g);
    cv::imwrite(std::string("bridge_") + name + (bridge ? "_on" : "_off") + ".png", view);
    return in_pocket;
}
} // namespace

int main()
{
    // 함수 단위
    std::printf("[bridge_same_side]\n");
    {
        using obstacles::SideBrick;
        auto q = obstacles::bridge_same_side({{0.0, 0.10, -0.10, true}, {0.45, 0.53, -0.11, true}}, 0.6);
        check(q.size() == 1, "같은 쪽 r-r, 간격 35cm → 이음");
        q = obstacles::bridge_same_side({{0.0, 0.10, -0.10, true}, {0.45, 0.53, 0.10, false}}, 0.6);
        check(q.empty(), "r-l → 안 이음");
        q = obstacles::bridge_same_side({{0.0, 0.10, -0.10, true}, {0.90, 0.98, -0.10, true}}, 0.6);
        check(q.empty(), "같은 쪽이지만 80cm 떨어짐 → 안 이음");
        q = obstacles::bridge_same_side({{0.0, 0.10, -0.10, true}, {0.02, 0.10, -0.10, true}}, 0.6);
        check(q.empty(), "기억과 화면이 같은 벽돌 (겹침) → 안 이음");
        q = obstacles::bridge_same_side({{0.0, 0.10, -0.10, true}, {0.45, 0.53, -0.40, true}}, 0.6);
        check(q.empty(), "같은 쪽 판정이지만 안쪽 끝이 30cm 다름 → 안 이음");
        q = obstacles::bridge_same_side({{0.0, 0.10, -0.10, true}, {0.45, 0.53, -0.11, true}, {0.98, 1.06, -0.10, true}}, 0.6);
        check(q.size() == 2, "r-r-r → 두 군데 이음");
        q = obstacles::bridge_same_side({{0.0, 0.10, -0.10, true}, {0.45, 0.53, -0.10, true}}, 0.0);
        check(q.empty(), "con_bridge_mm = 0 → 끔");
        q = obstacles::bridge_same_side({{0.0, 0.10, -0.10, true}, {0.45, 0.53, -0.10, true}}, 0.6, 0.05, 0.15, 0.03, 0.6, true);
        check(q.size() == 1 && std::abs(q[0][0].x - 0.10) < 1e-9 && std::abs(q[0][0].y - (-0.07)) < 1e-9,
              "from_back + margin 3cm → 뒷면(0.10)부터, 안쪽 끝 + 3cm");
    }

    // 경로 (스켈레톤 + 추적) — 흰선이 보이는 경우와 화면 밖인 경우
    std::printf("[경로]\n");
    for (double wy : {-0.22, -0.30})
    {
        const double yin1 = wy + 0.20; // 오른쪽 벽돌 20cm
        char nm[32];
        std::snprintf(nm, sizeof(nm), "rr_w%02d", int(-wy * 100));
        const bool off = run(nm, false, true, 0.45, wy, yin1);
        const bool on = run(nm, true, true, 0.45, wy, yin1);
        check(!on, "r-r 잇기 켬 → 1·2번 사이로 안 들어감");
        (void)off;
        std::snprintf(nm, sizeof(nm), "rl_w%02d", int(-wy * 100));
        const bool a = run(nm, false, false, 0.45, wy, yin1);
        const bool b = run(nm, true, false, 0.45, wy, yin1);
        check(a == b, "r-l 은 잇기 켜도 경로 그대로");
    }
    std::printf("%s\n", fails ? "FAIL" : "ALL OK");
    return fails ? 1 : 0;
}
