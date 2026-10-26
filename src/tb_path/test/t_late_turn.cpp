// T자 갈림길에서 회전 명령(/mission/turn)이 늦게 들어올 때 (표지판이 교차로 바로 앞에서 보이는 경우)
//   빌드: cd tb_path && g++ -O2 -std=c++17 -Iinclude test/t_late_turn.cpp src/laneSkeleton.cpp src/lanePath.cpp \
//         $(pkg-config --cflags --libs opencv4) -o t_late_turn && ./t_late_turn
//   직진 명령으로 T자 가로선 앞 switch_m 까지 와서 좌/우 명령으로 바꿈 → 가로 도로로 빠져나가는지
//   ./t_late_turn [v] [속도 오차] [회전 오차] [drop 0/1]   예: ./t_late_turn 0.15 -0.1 0.5 0
#include "tb_path/laneSkeleton.hpp"
#include "tb_path/lanePath.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <cstdio>
#include <string>

using PL = std::vector<cv::Point>;
constexpr double RES = 0.002; // 월드 1px = 2mm
constexpr int WW = 1600, WH = 1600, T = 10;
constexpr int BAR_LO = 600, BAR_HI = 460; // 가로 도로 아래·위 경계선 (px y). 안쪽 간격 130px = 260mm

struct World { cv::Mat white, yellow; };
World make_t()
{
    World w{cv::Mat::zeros(WH, WW, CV_8U), cv::Mat::zeros(WH, WW, CV_8U)};
    // 세로 도로: 왼쪽 노란선, 오른쪽 흰선 (트랙 규칙)
    cv::polylines(w.yellow, PL{{730, 1600}, {730, BAR_LO}, {0, BAR_LO}}, false, 255, T);
    cv::polylines(w.white, PL{{870, 1600}, {870, BAR_LO}, {1600, BAR_LO}}, false, 255, T);
    // 가로 도로 위 경계: 왼쪽 흰선, 오른쪽 노란선 (실차 영상처럼 색이 섞임)
    cv::line(w.white, {0, BAR_HI}, {800, BAR_HI}, 255, T);
    cv::line(w.yellow, {800, BAR_HI}, {1600, BAR_HI}, 255, T);
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
            const double wx = p.x + c * r.x - s * r.y, wy = p.y + s * r.x + c * r.y;
            mx.at<float>(v, u) = float(wx / RES);
            my.at<float>(v, u) = float(WH - wy / RES);
        }
    cv::remap(w.white, white, mx, my, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    cv::remap(w.yellow, yellow, mx, my, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    white &= valid;
    yellow &= valid;
}

bool g_drop = true;
double g_es = 0, g_ew = 0; // 추정 자세 오차 (cmd_vel 적분: 속도·회전 배율)
int run(lane_path::Turn turn, double switch_m, double v, bool save)
{
    const World world = make_t();
    const cv::Mat valid = bev_valid_320();
    lane_path::Geo g;
    lane_skel::Params sp;
    lane_path::Tracker tr;
    lane_path::Pose2D pose{800 * RES, (WH - 1550) * RES, M_PI / 2};
    lane_path::Pose2D est = pose;
    const double dt = 1.0 / 15, Ld = 0.20;
    const double bar_y = (WH - BAR_LO) * RES; // 가로선 (아래 경계) 월드 y
    cv::Mat map;
    cv::cvtColor(world.white, map, cv::COLOR_GRAY2BGR);
    map.setTo(cv::Scalar(0, 210, 255), world.yellow);
    cv::Point last(cvRound(pose.x / RES), cvRound(WH - pose.y / RES));
    cv::Point seed(-1, -1);
    int mem_after = 0, frames_after = 0;
    lane_path::Turn cur = lane_path::Turn::Straight;
    bool ok = false;
    for (int k = 0; k < 600; ++k)
    {
        if (cur == lane_path::Turn::Straight && bar_y - pose.y < switch_m)
        {
            cur = turn;
            if (g_drop)
                tr.drop_memory(); // path_node: path.turn_drop_memory
        }
        cv::Mat wm, ym;
        render(world, pose, g, valid, wm, ym);
        const lane_skel::Result sk = lane_skel::build(wm | ym, valid, sp, &seed);
        seed = sk.ok ? sk.seed : cv::Point(-1, -1);
        const lane_path::Output o = tr.update(sk, sp.scale, wm, ym, valid, g, cur, est);
        if (cur != lane_path::Turn::Straight)
        {
            ++frames_after;
            if (o.src == lane_path::Source::Memory)
                ++mem_after;
        }
        if (o.path.size() < 2)
            break;
        cv::Point2d tgt = o.path.back();
        for (const auto &q : o.path)
            if (std::hypot(q.x, q.y) >= Ld) { tgt = q; break; }
        const double w = v * 2 * tgt.y / std::max(tgt.x * tgt.x + tgt.y * tgt.y, 1e-6);
        pose.x += v * dt * std::cos(pose.th);
        pose.y += v * dt * std::sin(pose.th);
        pose.th += w * dt;
        est.x += v * (1 + g_es) * dt * std::cos(est.th);
        est.y += v * (1 + g_es) * dt * std::sin(est.th);
        est.th += w * (1 + g_ew) * dt;
        const cv::Point np(cvRound(pose.x / RES), cvRound(WH - pose.y / RES));
        cv::line(map, last, np, o.src == lane_path::Source::Memory ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 200, 0), 4);
        last = np;
        const bool in_bar = np.y > BAR_HI && np.y < BAR_LO;
        if (in_bar && ((turn == lane_path::Turn::Left && np.x < 450) || (turn == lane_path::Turn::Right && np.x > 1150)))
        { ok = true; break; }
        if (np.y < BAR_HI - 20 || np.x < 0 || np.x >= WW || np.y >= WH)
            break;
    }
    if (save)
        cv::imwrite(std::string("t_") + (turn == lane_path::Turn::Left ? "left_" : "right_") + std::to_string(int(switch_m * 100)) + ".png", map);
    std::printf("%-5s switch %2.0fcm before bar: %-6s memory %2d/%2d frames after turn\n",
                turn == lane_path::Turn::Left ? "left" : "right", switch_m * 100, ok ? "OK" : "FAIL", mem_after, frames_after);
    return ok ? 0 : 1;
}

int main(int argc, char **argv)
{
    const double v = argc > 1 ? std::atof(argv[1]) : 0.15;
    g_es = argc > 2 ? std::atof(argv[2]) : 0.0;
    g_ew = argc > 3 ? std::atof(argv[3]) : 0.0;
    g_drop = argc > 4 ? std::atoi(argv[4]) != 0 : true;
    int f = 0;
    for (auto t : {lane_path::Turn::Left, lane_path::Turn::Right})
        for (double s : {0.60, 0.45, 0.35, 0.30, 0.25, 0.20})
            f += run(t, s, v, true);
    std::printf("failures: %d\n", f);
    return f;
}
