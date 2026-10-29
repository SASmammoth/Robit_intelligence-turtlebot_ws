// 공사 구간 출구: 노란선만 보이다 왼쪽으로 꺾이는 곳에서 바깥(직진)으로 새지 않는지 (path.con_yellow_left)
//   녹화 2026-10-29 09:57 22s: 오른쪽 흰선이 화면 밖(공사 구간 폭 56cm)이라 노란선 오른쪽 바닥이 앞쪽 흰 가로선과
//   이어진 도로로 잡히고, 공사 모드 직진 기준(straight_by_lane)이 바깥 가지를 골라 선 밖으로 나감
//
//   g++ -O2 -std=c++17 -Iinclude src/laneSkeleton.cpp src/lanePath.cpp test/yellow_left_test.cpp \
//       $(pkg-config --cflags --libs opencv4) -o yellow_left_test && ./yellow_left_test
//   결과: 켬/끔 각각 출구를 왼쪽으로 돌았는지 + yl_*_map.png
#include "tb_path/laneSkeleton.hpp"
#include "tb_path/lanePath.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <cstdio>
#include <string>

using PL = std::vector<cv::Point>;
constexpr double RES = 0.002; // 월드 1px = 2mm
constexpr int WW = 1200, WH = 1600;
constexpr int T = 10;         // 선 두께 20mm

struct World { cv::Mat white, yellow; };

// 노란선: x=530 직진 → 반지름 r 원호로 왼쪽 → 가로(왼쪽으로). 흰선: 오른쪽 멀리(x=820, 공사 폭) → 출구 바깥 가로선
// 그 너머(트랙 밖)에 다른 흰 가로선 (녹화에서 앞쪽에 보이던 다른 구간)
World make_exit(int r_px)
{
    World w{cv::Mat::zeros(WH, WW, CV_8U), cv::Mat::zeros(WH, WW, CV_8U)};
    const int corner_y = 760 + r_px; // 원호 시작 높이 (영상 행)
    PL y{{530, 1600}, {530, corner_y}};
    for (int a = 1; a <= 18; ++a) { // 중심 (530 - r, corner_y)에서 0° → 90°
        const double t = a * M_PI / 36.0;
        y.push_back({cvRound(530 - r_px + r_px * std::cos(t)), cvRound(corner_y - r_px * std::sin(t))});
    }
    y.push_back({0, 760});
    cv::polylines(w.yellow, y, false, 255, T);
    cv::polylines(w.white, PL{{820, 1600}, {820, 620}, {0, 620}}, false, 255, T); // 출구 바깥 경계 (안쪽 폭 26cm)
    cv::line(w.white, {300, 380}, {1200, 380}, 255, T);                          // 트랙 밖 다른 구간
    return w;
}

cv::Mat bev_valid_320()
{
    const float w = 640, h = 480, cx = w / 2;
    const float ty = h * 0.45f, by = h - 1, tw = w * 0.15f, bw = w * 0.5f;
    std::vector<cv::Point2f> src = {{cx - tw, ty}, {cx + tw, ty}, {cx + bw, by}, {cx - bw, by}};
    const float S = 320, m = S * 0.25f;
    std::vector<cv::Point2f> dst = {{m, 0}, {S - m, 0}, {S - m, S}, {m, S}};
    return lane_skel::make_valid_mask(cv::getPerspectiveTransform(src, dst), cv::Size(640, 480), cv::Size(320, 320));
}

cv::Point w2px(double x, double y) { return {cvRound(x / RES), cvRound(WH - y / RES)}; }

void render(const World &w, const lane_path::Pose2D &p, const lane_path::Geo &g, const cv::Mat &valid,
            cv::Mat &white, cv::Mat &yellow)
{
    const int S = g.bev_size;
    cv::Mat mx(S, S, CV_32F), my(S, S, CV_32F);
    const double c = std::cos(p.th), s = std::sin(p.th);
    for (int v = 0; v < S; ++v)
        for (int u = 0; u < S; ++u) {
            const cv::Point2d r = lane_path::bev_to_robot({double(u), double(v)}, g);
            const double wx = p.x + c * r.x - s * r.y, wy = p.y + s * r.x + c * r.y;
            mx.at<float>(v, u) = float(wx / RES);
            my.at<float>(v, u) = float(WH - wy / RES);
        }
    cv::remap(w.white, white, mx, my, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    cv::remap(w.yellow, yellow, mx, my, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    white &= valid;
    yellow &= valid;
}

// 0: 출구를 왼쪽으로 돎, 1: 바깥으로 나감/잃음
int run(const std::string &name, bool yl, int r_px, double x0, double v)
{
    const World world = make_exit(r_px);
    const cv::Mat valid = bev_valid_320();
    lane_path::Geo g;
    lane_skel::Params sp;
    lane_path::Tracker tr;
    // 공사 모드 값 (1029 params)
    tr.p.override_frames = 1;
    tr.p.cross_override_frames = 15;
    tr.p.straight_by_lane = true;
    tr.p.yellow_left = yl;

    lane_path::Pose2D pose{x0 * RES, (WH - 1550) * RES, M_PI / 2};
    const double dt = 1.0 / 15, Ld = 0.21;
    cv::Mat map;
    cv::cvtColor(world.white, map, cv::COLOR_GRAY2BGR);
    map.setTo(cv::Scalar(0, 210, 255), world.yellow);
    cv::Point last = w2px(pose.x, pose.y), pos(-1, -1);
    int used = 0, verdict = 1;
    const char *why = "시간 초과";
    for (int k = 0; k < 600; ++k) {
        cv::Mat wm, ym;
        render(world, pose, g, valid, wm, ym);
        const lane_skel::Result sk = lane_skel::build(wm | ym, valid, sp, &pos);
        pos = sk.ok ? sk.seed : cv::Point(-1, -1);
        const lane_path::Output o = tr.update(sk, sp.scale, wm, ym, valid, g, lane_path::Turn::Straight, pose);
        used += o.yellow_left_used;
        if (o.path.size() < 2) { why = "경로 잃음"; break; }
        cv::Point2d tgt = o.path.back();
        for (const auto &q : o.path)
            if (std::hypot(q.x, q.y) >= Ld) { tgt = q; break; }
        const double w = v * 2 * tgt.y / std::max(tgt.dot(tgt), 1e-6);
        pose.x += v * dt * std::cos(pose.th);
        pose.y += v * dt * std::sin(pose.th);
        pose.th += w * dt;
        const cv::Point np = w2px(pose.x, pose.y);
        cv::line(map, last, np, cv::Scalar(0, 200, 0), 4);
        last = np;
        if (np.y < 615) { why = "바깥 흰선 넘음"; break; }
        if (np.y > 765 && np.x < 520) { why = "노란선 넘음"; break; }
        if (np.x < 250 && np.y > 625 && np.y < 755) { why = "출구 왼쪽으로 돎"; verdict = 0; break; }
    }
    cv::imwrite(name + "_map.png", map);
    std::printf("%-22s %s  (노란선 왼쪽 유지 사용 %d프레임)\n", name.c_str(), why, used);
    return verdict;
}

int main()
{
    int fail_on = 0, fail_off = 0;
    for (int r : {0, 60, 120})          // 출구 모서리 반지름 0 / 12 / 24cm
        for (double x0 : {600.0, 620.0, 640.0}) // 노란선에서 13 / 17 / 21cm 오른쪽에서 출발
            for (double v : {0.20, 0.35})
            {
                const std::string n = "r" + std::to_string(r) + "_x" + std::to_string(int(x0)) + "_v" + std::to_string(int(v * 100));
                fail_off += run("yl_off_" + n, false, r, x0, v);
                fail_on += run("yl_on_" + n, true, r, x0, v);
            }
    std::printf("실패: 끔 %d / 켬 %d (각 18가지)\n", fail_off, fail_on);
    return fail_on;
}
