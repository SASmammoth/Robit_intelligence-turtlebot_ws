// 녹화로 path_node 경로 만들기 재현 (공사 구간 포함, 지난 벽돌 피하기 제외)
//   python3 tb_path/test/bag_dump_full.py ~/bags/run_1027 /tmp/rp tb_interfaces/msg   (src 폴더에서)
//   g++ -O2 -std=c++17 -Itb_path/include tb_path/test/bag_replay_con.cpp tb_path/src/laneSkeleton.cpp \
//       tb_path/src/lanePath.cpp tb_path/src/obstacles.cpp $(pkg-config --cflags --libs opencv4) -o bag_replay_con
//   cd /tmp/rp && ~/turtlebot_ws/src/bag_replay_con 6.4 8.2 1 1 1 out.png
//   인자: t0 t1 con_override_frames con_straight_by_lane con_offscreen_side [출력 png]
//   벽돌 파라미터는 아래 상수 (tb_params_261027_con2 값). 로봇은 녹화된 대로 움직이므로 경로가 갈라진 뒤는 참고만
#include "tb_path/laneSkeleton.hpp"
#include "tb_path/lanePath.hpp"
#include "tb_path/obstacles.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
static void smooth_path(std::vector<cv::Point2d> &path, double half)
{
    const size_t n = path.size();
    if (n < 5) return;
    std::vector<double> s(n, 0.0);
    for (size_t i = 1; i < n; ++i) s[i] = s[i - 1] + cv::norm(path[i] - path[i - 1]);
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<cv::Point2d> out(path);
        for (size_t i = 1; i + 1 < n; ++i) {
            const double h = std::min({half, s[i] - s[0], s[n - 1] - s[i]});
            cv::Point2d sum(0, 0); int cnt = 0;
            for (size_t j = i; j-- > 0 && s[i] - s[j] <= h;) sum += path[j], ++cnt;
            for (size_t j = i; j < n && s[j] - s[i] <= h; ++j) sum += path[j], ++cnt;
            if (cnt > 0) out[i] = sum * (1.0 / cnt);
        }
        path.swap(out);
    }
}
int main(int argc, char **argv)
{
    const double t0 = atof(argv[1]), t1 = atof(argv[2]);
    const int ovr = atoi(argv[3]); const bool bylane = atoi(argv[4]), offs = atoi(argv[5]);
    const char *png = argc > 6 ? argv[6] : "replay.png";
    // 사용자 파라미터 (tb_params_261027_con2)
    const double depth = 0.130, margin = 0.050, gap = 0.140, edge = 0.150, bw = 0.300, attach = 0.300, extend = 0.400, dead = 0.250, smooth = 0.125;
    std::ifstream in("frames.txt");
    std::string line;
    lane_path::Tracker tr;
    tr.p.max_dev_m = 0.05; tr.p.min_len_m = 0.10; tr.p.lookahead_m = 0.04; tr.p.blind_max_m = 0.25; tr.p.fill_blind = 1; tr.p.fill_join_m = 0.04; tr.p.mem_travel_m = 0.6;
    cv::Point last_seed(-1, -1);
    std::vector<cv::Mat> shots;
    FILE *csv = fopen("replay_paths.txt", "w");
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        int k, nb, con; double t; lane_path::Pose2D pose; double mx, my, bot; double h[9]; int lw;
        ss >> k >> t >> pose.x >> pose.y >> pose.th >> mx >> my >> bot >> lw;
        for (double &v : h) ss >> v;
        ss >> con >> nb;
        std::vector<obstacles::Box> boxes(nb);
        for (auto &b : boxes) ss >> b.x >> b.y >> b.w >> b.h;
        char fn[64]; std::snprintf(fn, sizeof fn, "mask_%03d.png", k);
        const cv::Mat label = cv::imread(fn, 0);
        const cv::Mat white = label == 1, yellow = label == 2, valid = label != 255;
        lane_path::Geo g; g.bev_size = label.cols; g.lane_w_px = lw; g.m_per_px_x = mx; g.m_per_px_y = my; g.bottom_m = bot;
        cv::Mat wall = white | yellow;
        const cv::Mat lines = wall.clone();
        const cv::Mat H(3, 3, CV_64F, h);
        cv::Mat bm = cv::Mat::zeros(wall.size(), CV_8U), ext = bm.clone();
        if (!boxes.empty()) {
            if (con) {
                obstacles::draw_boxes(bm, H, boxes, depth / my, margin / mx, &lines, gap / mx, edge / mx, bw / mx, attach / mx, false, offs);
                obstacles::extend_occluded(lines, bm, ext, 0.08 / my, extend / my, std::max(3, int(std::lround(0.012 / mx))));
                wall |= ext; wall |= bm;
            } else {
                obstacles::draw_boxes(wall, H, boxes, 0.135 / my, 0.100 / mx, &lines, gap / mx, 0, 0, 0, true);
            }
        }
        lane_skel::Params sp; sp.lane_w = std::max(8, lw); sp.seal_len = 30; sp.prune_len = 20; sp.max_lane_thick = 20;
        sp.dead_end_len = con ? int(std::lround(dead / my)) : 0;
        cv::Point seed_hint = last_seed, pred;
        const bool hp = tr.predict_seed(pose, g, pred);
        if (hp) seed_hint = pred;
        const auto sk = lane_skel::build(wall, valid, sp, &seed_hint, hp ? &pred : nullptr);
        last_seed = sk.ok ? sk.seed : cv::Point(-1, -1);
        tr.p.override_frames = con ? ovr : 10;
        tr.p.straight_by_lane = con && bylane;
        auto o = tr.update(sk, sp.scale, white, yellow, valid, g, lane_path::Turn::Straight, pose);
        if (con) smooth_path(o.path, smooth);
        fprintf(csv, "%.4f %s", t, lane_path::source_name(o.src));
        for (auto &q : o.path) fprintf(csv, " %.4f %.4f", q.x, q.y);
        fprintf(csv, "\n");
        if (t < t0 || t > t1) continue;
        double ymax = -9; for (auto &q : o.path) ymax = std::max(ymax, q.y);
        std::printf("%.2f con=%d src=%-6s n=%2zu ymax=%+.2f\n", t, con, lane_path::source_name(o.src), o.path.size(), ymax);
        cv::Mat view(label.size(), CV_8UC3, cv::Scalar(70, 70, 70));
        lane_skel::draw_debug(view, valid, sk, sp.scale);
        view.setTo(cv::Scalar(0, 0, 200), bm);
        view.setTo(cv::Scalar(170, 170, 170), ext);
        view.setTo(cv::Scalar(235, 235, 235), white);
        view.setTo(cv::Scalar(0, 210, 255), yellow);
        lane_path::draw_path(view, o, g);
        char tx[32]; std::snprintf(tx, sizeof tx, "%.2f", t);
        cv::putText(view, tx, {4, 316}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
        shots.push_back(view);
    }
    fclose(csv);
    if (shots.empty()) return 0;
    std::vector<cv::Mat> sel;
    for (size_t i = 0; i < shots.size(); i += std::max<size_t>(1, shots.size() / 12)) sel.push_back(shots[i]);
    while (sel.size() % 6) sel.push_back(cv::Mat::zeros(shots[0].size(), shots[0].type()));
    std::vector<cv::Mat> rows;
    for (size_t i = 0; i < sel.size(); i += 6) { cv::Mat r; cv::hconcat(std::vector<cv::Mat>(sel.begin() + i, sel.begin() + i + 6), r); rows.push_back(r); }
    cv::Mat all; cv::vconcat(rows, all);
    cv::imwrite(png, all);
}
