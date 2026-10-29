// T자에 수직으로 들어갈 때 가로선(위 경계)을 넘어가는지 + 차선 넘기 방지(guard_*) 켜고 끈 비교
//   빌드: cd tb_path && g++ -O2 -std=c++17 -Iinclude test/t_cross.cpp src/laneSkeleton.cpp src/lanePath.cpp \
//         $(pkg-config --cflags --libs opencv4) -o t_cross && ./t_cross
//   가로선 너머: open(바닥만) / road(바로 위에 다른 도로) / gap(정면 가로선이 8cm 끊김)
//   좌/우 명령을 가로 도로 아래 경계 앞 switch_m에서 줌 (none = 끝까지 직진 명령)
//   OK: 가로 도로로 빠져나감, STOP: 경로를 잃고 멈춤(넘지는 않음), CROSS: 가로선을 넘어감
//   ./t_cross [v] [save 0/1] [open|road|gap: 하나만]
#include "tb_path/laneSkeleton.hpp"
#include "tb_path/lanePath.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

using PL = std::vector<cv::Point>;
constexpr double RES = 0.002; // 월드 1px = 2mm
constexpr int WW = 1600, WH = 1600, T = 10;
constexpr int BAR_LO = 600, BAR_HI = 460; // 가로 도로 아래·위 경계선 (px y). 안쪽 간격 130px = 260mm

struct World { cv::Mat white, yellow; };
World make_t(const char *beyond)
{
    World w{cv::Mat::zeros(WH, WW, CV_8U), cv::Mat::zeros(WH, WW, CV_8U)};
    cv::polylines(w.yellow, PL{{730, 1600}, {730, BAR_LO}, {0, BAR_LO}}, false, 255, T);
    cv::polylines(w.white, PL{{870, 1600}, {870, BAR_LO}, {1600, BAR_LO}}, false, 255, T);
    cv::line(w.white, {0, BAR_HI}, {800, BAR_HI}, 255, T);
    cv::line(w.yellow, {800, BAR_HI}, {1600, BAR_HI}, 255, T);
    if (!std::strcmp(beyond, "road")) // 가로선 바로 위에 나란한 다른 도로 (트랙 다른 구간)
        cv::line(w.white, {0, BAR_HI - 140}, {1600, BAR_HI - 140}, 255, T);
    if (!std::strcmp(beyond, "gap"))  // 정면 가로선이 흐려서 8cm 끊김
        cv::line(w.white, {780, BAR_HI}, {820, BAR_HI}, 0, T + 2), cv::line(w.yellow, {780, BAR_HI}, {820, BAR_HI}, 0, T + 2);
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

void render(const World &w, const lane_path::Pose2D &p, const lane_path::Geo &g, const cv::Mat &valid,
            cv::Mat &white, cv::Mat &yellow)
{
    const int S = g.bev_size;
    cv::Mat mx(S, S, CV_32F), my(S, S, CV_32F);
    const double c = std::cos(p.th), s = std::sin(p.th);
    for (int v = 0; v < S; ++v)
        for (int u = 0; u < S; ++u)
        {
            const cv::Point2d r = lane_path::bev_to_robot({double(u), double(v)}, g);
            mx.at<float>(v, u) = float((p.x + c * r.x - s * r.y) / RES);
            my.at<float>(v, u) = float(WH - (p.y + s * r.x + c * r.y) / RES);
        }
    cv::remap(w.white, white, mx, my, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    cv::remap(w.yellow, yellow, mx, my, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    white &= valid;
    yellow &= valid;
}

struct Guard { bool start = false, mem = false, gap = false; };
enum Result { OK, STOP, CROSS };

// path_node처럼: 기억과 선 너머 새 경로 의심(guard_mem)은 cross_override_frames로
Result run(const char *beyond, lane_path::Turn turn, double switch_m, double v, const Guard &gd, bool save)
{
    const World world = make_t(beyond);
    const cv::Mat valid = bev_valid_320();
    lane_path::Geo g;
    lane_skel::Params sp;
    sp.guard_start = gd.start;
    sp.robot_y = cvRound(lane_path::robot_to_bev({0, 0}, g).y);
    lane_path::Tracker tr;
    tr.p.cross_override_frames = gd.mem ? 20 : 0;
    tr.p.guard_gap = gd.gap;
    lane_path::Pose2D pose{800 * RES, (WH - 1550) * RES, M_PI / 2};
    const double dt = 1.0 / 15, Ld = 0.20;
    const double bar_y = (WH - BAR_LO) * RES;
    cv::Mat map;
    cv::cvtColor(world.white, map, cv::COLOR_GRAY2BGR);
    map.setTo(cv::Scalar(0, 210, 255), world.yellow);
    cv::Point last(cvRound(pose.x / RES), cvRound(WH - pose.y / RES));
    cv::Point seed(-1, -1);
    lane_path::Turn cur = lane_path::Turn::Straight;
    Result res = STOP;
    for (int k = 0; k < 600; ++k)
    {
        if (cur == lane_path::Turn::Straight && turn != lane_path::Turn::Straight && bar_y - pose.y < switch_m)
        {
            cur = turn;
            tr.drop_memory(); // path.turn_drop_memory
        }
        cv::Mat wm, ym;
        render(world, pose, g, valid, wm, ym);
        cv::Point pred;
        const bool has_pred = tr.predict_seed(pose, g, pred);
        if (has_pred) seed = pred;
        const lane_skel::Result sk = lane_skel::build(wm | ym, valid, sp, &seed, has_pred ? &pred : nullptr);
        seed = sk.ok ? sk.seed : cv::Point(-1, -1);
        const lane_path::Output o = tr.update(sk, sp.scale, wm, ym, valid, g, cur, pose);
        if (o.path.size() < 2)
            break; // 경로 잃음 → 정지
        cv::Point2d tgt = o.path.back();
        for (const auto &q : o.path)
            if (std::hypot(q.x, q.y) >= Ld) { tgt = q; break; }
        if (std::hypot(tgt.x, tgt.y) < 0.03)
            break; // 경로 끝에 다 옴 → 정지
        const double w = v * 2 * tgt.y / std::max(tgt.x * tgt.x + tgt.y * tgt.y, 1e-6);
        pose.x += v * dt * std::cos(pose.th);
        pose.y += v * dt * std::sin(pose.th);
        pose.th += w * dt;
        const cv::Point np(cvRound(pose.x / RES), cvRound(WH - pose.y / RES));
        cv::line(map, last, np, o.src == lane_path::Source::Memory ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 200, 0), 4);
        last = np;
        if (np.y < BAR_HI) { res = CROSS; break; } // 바퀴 축이 가로선을 넘음
        if (np.y > BAR_HI && np.y < BAR_LO && (np.x < 450 || np.x > 1150)) { res = OK; break; }
        if (np.x < 0 || np.x >= WW || np.y >= WH)
            break;
    }
    if (save)
    {
        const char *tn = turn == lane_path::Turn::Left ? "left" : turn == lane_path::Turn::Right ? "right" : "none";
        cv::imwrite(std::string("cross_") + beyond + "_" + tn + "_" + std::to_string(int(switch_m * 100)) +
                        (gd.start || gd.mem || gd.gap ? "_guard" : "") + ".png", map);
    }
    return res;
}

int main(int argc, char **argv)
{
    const double v = argc > 1 ? std::atof(argv[1]) : 0.15;
    const bool save = argc > 2 && std::atoi(argv[2]) != 0;
    static const char *names[] = {"OK", "STOP", "CROSS"};
    struct Cfg { const char *name; Guard g; };
    const Cfg cfgs[] = {{"off", {}}, {"a", {true, false, false}}, {"b", {false, true, false}},
                        {"c", {false, false, true}}, {"abc", {true, true, true}}};
    for (const char *beyond : {"open", "road", "gap"})
    {
        if (argc > 3 && std::strcmp(argv[3], beyond))
            continue;
        std::printf("== beyond: %s (v %.2f)\n%-12s", beyond, v, "turn/switch");
        for (const auto &c : cfgs) std::printf(" %-6s", c.name);
        std::printf("\n");
        for (auto t : {lane_path::Turn::Left, lane_path::Turn::Right, lane_path::Turn::Straight})
            for (double s : {0.30, 0.15, 0.08})
            {
                if (t == lane_path::Turn::Straight && s != 0.30) continue; // none은 한 번만
                char label[32];
                std::snprintf(label, sizeof label, "%s %2.0fcm", t == lane_path::Turn::Left ? "left" : t == lane_path::Turn::Right ? "right" : "none", t == lane_path::Turn::Straight ? 0.0 : s * 100);
                std::printf("%-12s", label);
                for (const auto &c : cfgs)
                    std::printf(" %-6s", names[run(beyond, t, s, v, c.g, save)]);
                std::printf("\n");
            }
    }
    return 0;
}
