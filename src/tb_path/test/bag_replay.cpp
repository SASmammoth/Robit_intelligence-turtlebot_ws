// 녹화(bag_dump.py 결과)로 path_node 공사 구간 경로 만들기를 다시 돌림 (지난 벽돌 피하기·모서리 둥글게 제외)
//   g++ -O2 -std=c++17 -Itb_path/include tb_path/test/bag_replay.cpp tb_path/src/laneSkeleton.cpp tb_path/src/lanePath.cpp
//       tb_path/src/obstacles.cpp $(pkg-config --cflags --libs opencv4) -o bag_replay      (src 폴더에서)
//   cd /tmp/replay && ~/ros2_ws/src/bag_replay 1.8 2.4      # 시작~끝 시간(s), 그 구간 경로 요약 + replay_skel.png
//   인자: t0 t1 [con_dead_end_mm 250] [brick_gap_mm 140] [con_brick_w_mm 300] [출력 png] [blind_max_m 0.25]
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
int main(int argc, char **argv)
{
    const double t0 = atof(argv[1]), t1 = atof(argv[2]);
    const int dead_mm = argc > 3 ? atoi(argv[3]) : 250, gap_mm = argc > 4 ? atoi(argv[4]) : 140;
    const int width_mm = argc > 5 ? atoi(argv[5]) : 300;
    std::ifstream in("frames.txt");
    std::string line;
    lane_path::Tracker tr;
    tr.p.max_dev_m = 0.05; tr.p.override_frames = 10; tr.p.min_len_m = 0.10; tr.p.lookahead_m = 0.04;
    tr.p.blind_max_m = argc > 7 ? atof(argv[7]) : 0.25; tr.p.fill_blind = 1; tr.p.fill_join_m = 0.04; tr.p.mem_travel_m = 0.6;
    cv::Point last_seed(-1, -1);
    std::vector<cv::Mat> shots;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        int k, nb; double t; lane_path::Pose2D pose; double mx, my, bot; double h[9];
        int lw; ss >> k >> t >> pose.x >> pose.y >> pose.th >> mx >> my >> bot >> lw;
        for (double &v : h) ss >> v;
        ss >> nb;
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
            obstacles::draw_boxes(bm, H, boxes, 0.150 / my, 0.060 / mx, &lines, gap_mm / 1000.0 / mx, 0.060 / mx,
                                  width_mm / 1000.0 / mx, 0.300 / mx, false);
            obstacles::extend_occluded(lines, bm, ext, 0.08 / my, 0.40 / my, std::max(3, int(std::lround(0.012 / mx))));
            wall |= ext; wall |= bm;
        }
        lane_skel::Params sp; sp.lane_w = lw; sp.seal_len = 30; sp.prune_len = 20; sp.max_lane_thick = 20;
        sp.dead_end_len = dead_mm > 0 ? int(std::lround(dead_mm / 1000.0 / my)) : 0;
        cv::Point seed_hint = last_seed, pred;
        const bool hp = tr.predict_seed(pose, g, pred);
        if (hp) seed_hint = pred;
        const auto sk = lane_skel::build(wall, valid, sp, &seed_hint, hp ? &pred : nullptr);
        last_seed = sk.ok ? sk.seed : cv::Point(-1, -1);
        const auto o = tr.update(sk, sp.scale, white, yellow, valid, g, lane_path::Turn::Straight, pose);
        if (t < t0 || t > t1) continue;
        double ymin = 9, ymax = -9, xmax = -9;
        for (auto &q : o.path) ymin = std::min(ymin, q.y), ymax = std::max(ymax, q.y), xmax = std::max(xmax, q.x);
        std::printf("%.2f src=%s n=%zu x<=%.2f y %+.2f..%+.2f ends=%zu junc=%zu\n", t, lane_path::source_name(o.src), o.path.size(), xmax, ymin, ymax, sk.ends.size(), sk.junctions.size());
        cv::Mat view(label.size(), CV_8UC3, cv::Scalar(70, 70, 70));
        lane_skel::draw_debug(view, valid, sk, sp.scale);
        view.setTo(cv::Scalar(0, 0, 200), bm);
        view.setTo(cv::Scalar(170, 170, 170), ext);
        view.setTo(cv::Scalar(235, 235, 235), white);
        if (sk.ok) { std::vector<cv::Point> pts; cv::findNonZero(sk.skel, pts); for (auto &p : pts) cv::circle(view, p * sp.scale, 1, cv::Scalar(200, 255, 100), -1); }
        lane_path::draw_path(view, o, g);
        char tx[32]; std::snprintf(tx, sizeof tx, "%.2f", t);
        cv::putText(view, tx, {4, 316}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
        shots.push_back(view);
    }
    std::vector<cv::Mat> sel;
    for (size_t i = 0; i < shots.size(); i += std::max<size_t>(1, shots.size() / 8)) sel.push_back(shots[i]);
    while (sel.size() % 4) sel.push_back(cv::Mat::zeros(shots[0].size(), shots[0].type()));
    std::vector<cv::Mat> rows;
    for (size_t i = 0; i < sel.size(); i += 4) { cv::Mat r; cv::hconcat(std::vector<cv::Mat>(sel.begin() + i, sel.begin() + i + 4), r); rows.push_back(r); }
    cv::Mat all; cv::vconcat(rows, all);
    cv::imwrite(argc > 6 ? argv[6] : "replay_skel.png", all);
}
