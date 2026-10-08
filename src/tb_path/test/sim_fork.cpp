// 가상 트랙 폐루프 시뮬레이션
// 월드 트랙을 로봇 자세에서 BEV로 렌더링 → 스켈레톤 → 경로 추적 → pure pursuit → 로봇 이동
#include "tb_path/laneSkeleton.hpp"
#include "tb_path/lanePath.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cmath>
#include <cstdio>
#include <string>

using PL = std::vector<cv::Point>;
constexpr double RES = 0.002;          // 월드 1px = 2mm
constexpr int WW = 1200, WH = 1600;
constexpr int T = 10;                  // 차선 두께 20mm

struct World { cv::Mat white, yellow; };

World make_island()
{
    World w{cv::Mat::zeros(WH, WW, CV_8U), cv::Mat::zeros(WH, WW, CV_8U)};
    // 안쪽 간격 260mm = 130px, 선 중심은 안쪽 경계에서 5px 바깥
    cv::polylines(w.yellow, PL{{530, 1600}, {530, 1100}, {400, 1100}, {400, 500}, {530, 500}, {530, 0}}, false, 255, T);
    cv::polylines(w.white,  PL{{670, 1600}, {670, 1100}, {800, 1100}, {800, 500}, {670, 500}, {670, 0}}, false, 255, T);
    // 섬: 왼쪽 절반 흰색, 오른쪽 절반 노란색
    cv::polylines(w.white,  PL{{600, 950}, {540, 950}, {540, 650}, {600, 650}}, false, 255, T);
    cv::polylines(w.yellow, PL{{600, 950}, {660, 950}, {660, 650}, {600, 650}}, false, 255, T);
    return w;
}

float g_bw = 0.5f;
World make_eo()
{
    // ㅓ 구간: 본선 왼쪽으로 복도, 복도 위아래에 주차 칸 (점선으로 구분)
    World w{cv::Mat::zeros(WH, WW, CV_8U), cv::Mat::zeros(WH, WW, CV_8U)};
    cv::polylines(w.yellow, PL{{530, 1600}, {530, 1030}, {400, 1030}, {400, 1170}, {250, 1170}, {250, 760}, {400, 760}, {400, 900}, {530, 900}, {530, 0}}, false, 255, T);
    for (int x = 255; x < 395; x += 34) {
        cv::line(w.yellow, {x, 900}, {x + 20, 900}, 255, T);
        cv::line(w.yellow, {x, 1030}, {x + 20, 1030}, 255, T);
    }
    cv::line(w.white, {670, 1600}, {670, 0}, 255, T);
    return w;
}
bool g_eo = false;

cv::Mat bev_valid_320()
{
    const float w = 640, h = 480, cx = w / 2;
    const float ty = h * 0.45f, by = h - 1, tw = w * 0.15f, bw = w * g_bw;
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

struct Var { double dx = 0, dth = 0, v = 0.08, fps = 15, mem = 0.6, es = 0, ew = 0; };
int run(const std::string &name, lane_path::Turn turn, Var var = {})
{
    const World world = g_eo ? make_eo() : make_island();
    const cv::Mat valid = bev_valid_320();
    lane_path::Geo g;                 // lane_w_px 160, 260mm, depth 520mm, bottom 150mm
    lane_skel::Params sp;
    lane_path::Tracker tr;
    tr.p.mem_travel_m = var.mem;

    lane_path::Pose2D pose{600 * RES + var.dx, (WH - 1550) * RES, M_PI / 2 + var.dth};
    lane_path::Pose2D est = pose;   // 추정 자세 (cmd_vel 적분 오차 흉내)
    const double v = var.v, dt = 1.0 / var.fps, Ld = 0.20;

    cv::Mat map;
    cv::cvtColor(world.white, map, cv::COLOR_GRAY2BGR);
    map.setTo(cv::Scalar(0, 210, 255), world.yellow);
    cv::Point robot_last = w2px(pose.x, pose.y);

    cv::Point pos(-1, -1);
    int counts[4] = {0, 0, 0, 0};
    double min_x_island = 1e9, max_x_island = -1e9;
    bool exited = false;
    int saved = 0;
    std::vector<cv::Mat> ring;

    for (int k = 0; k < 2000; ++k) {
        cv::Mat wm, ym;
        render(world, pose, g, valid, wm, ym);
        const lane_skel::Result sk = lane_skel::build(wm | ym, valid, sp, &pos);
        pos = sk.ok ? sk.seed : cv::Point(-1, -1);
        const lane_path::Output o = tr.update(sk, sp.scale, wm, ym, valid, g, turn, est);
        counts[int(o.src)]++;

        // 마지막 프레임들 저장 (6프레임 간격)
        const cv::Point rp = w2px(pose.x, pose.y);
        if (k % 3 == 0 && rp.x > 700 && rp.y < 1000 && saved < 8) {
            cv::Mat view(320, 320, CV_8UC3, cv::Scalar(60, 60, 60));
            view.setTo(cv::Scalar(235, 235, 235), wm);
            view.setTo(cv::Scalar(0, 210, 255), ym);
            lane_skel::draw_debug(view, valid, sk, sp.scale);
            view.setTo(cv::Scalar(235, 235, 235), wm);
            view.setTo(cv::Scalar(0, 210, 255), ym);
            lane_path::draw_path(view, o, g);
            cv::putText(view, std::to_string(rp.x) + "," + std::to_string(rp.y), {6, 310}, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255,255,255), 1); ring.push_back(view); ++saved;
        }

        if (o.path.size() < 2 || (g_eo && std::hypot(o.path.back().x, o.path.back().y) < 0.05)) { if (!g_eo) std::printf("%s: lost at step %d\n", name.c_str(), k); break; }

        // pure pursuit
        cv::Point2d tgt = o.path.back();
        for (const auto &q : o.path)
            if (std::hypot(q.x, q.y) >= Ld) { tgt = q; break; }
        const double L2 = tgt.x * tgt.x + tgt.y * tgt.y;
        const double w = v * 2 * tgt.y / std::max(L2, 1e-6);
        pose.x += v * dt * std::cos(pose.th);
        pose.y += v * dt * std::sin(pose.th);
        pose.th += w * dt;
        est.x += v * (1 + var.es) * dt * std::cos(est.th);
        est.y += v * (1 + var.es) * dt * std::sin(est.th);
        est.th += w * (1 + var.ew) * dt;

        const cv::Scalar col = o.src == lane_path::Source::Live     ? cv::Scalar(0, 200, 0)
                               : o.src == lane_path::Source::Memory ? cv::Scalar(255, 0, 255)
                                                                    : cv::Scalar(0, 120, 255);
        const cv::Point np = w2px(pose.x, pose.y);
        cv::line(map, robot_last, np, col, 4);
        robot_last = np;
        if (np.y > 650 && np.y < 950) { min_x_island = std::min(min_x_island, double(np.x)); max_x_island = std::max(max_x_island, double(np.x)); }
        if (np.y < 60) { exited = true; break; }
        if (np.x < 0 || np.x >= WW || np.y >= WH) break;
    }
    cv::imwrite(name + "_map.png", map);
    if (!ring.empty()) { cv::Mat all; cv::hconcat(ring, all); cv::imwrite(name + "_last.png", all); }

    if (g_eo) {
        const cv::Point fp = w2px(pose.x, pose.y);
        const bool in_corr = fp.x < 450 && fp.x > 250 && fp.y > 900 && fp.y < 1030;
        std::printf("%-6s end=(%d,%d) %s  frames live=%d memory=%d color=%d lost=%d\n", name.c_str(), fp.x, fp.y,
                    in_corr ? "IN CORRIDOR" : "NOT in corridor", counts[0], counts[1], counts[2], counts[3]);
        return in_corr ? 0 : 1;
    }
    const char *side = max_x_island < 540 ? "LEFT of island" : min_x_island > 660 ? "RIGHT of island" : "hit island?";
    std::printf("%-6s exited=%d  side=%s  (x range %.0f..%.0f)  frames live=%d memory=%d color=%d lost=%d\n",
                name.c_str(), exited, side, min_x_island, max_x_island, counts[0], counts[1], counts[2], counts[3]);
    const bool right_side = turn == lane_path::Turn::Left ? max_x_island < 540 : min_x_island > 660;
    return exited && right_side ? 0 : 1;
}

int main()
{
    int f = 0;
    for (auto t : {lane_path::Turn::Left, lane_path::Turn::Right}) {
        const std::string n = t == lane_path::Turn::Left ? "left" : "right";
        f += run(n, t);
        f += run(n + "_off", t, Var{0.04, 0.1});
        f += run(n + "_off2", t, Var{-0.04, -0.1});
        f += run(n + "_fast", t, Var{0, 0, 0.15, 10});
        f += run(n + "_odomerr+", t, Var{0, 0, 0.08, 15, 0.6, 0.10, 0.15});
        f += run(n + "_odomerr-", t, Var{0, 0, 0.08, 15, 0.6, -0.10, -0.15});
        g_bw = 0.38f;
        f += run(n + "_narrow", t, Var{});
        f += run(n + "_narrow_err", t, Var{0.03, 0.05, 0.12, 12, 0.6, 0.1, -0.15});
        g_bw = 0.5f;
    }
    g_eo = true;
    run("eo_left", lane_path::Turn::Left);   // 참고용 (ㅓ 막다른 복도: 주차 기동은 별도)
    g_eo = false;
    std::printf("failures: %d\n", f);
    return f;
}
