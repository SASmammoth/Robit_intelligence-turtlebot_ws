// 공사 구간 좌커브 안쪽 벽돌 (claude12 10초): 벽돌을 지나자 안쪽이 벽돌 박스로 막혀
// 스켈레톤이 바깥 흰선 너머 바닥에 생김 → con_override_frames=1이면 바로 그 경로를 따라 선 밖으로 나감.
// cross_override_frames: 새 경로와 기억 사이에 선이 있으면 이만큼 연속으로 보여야 믿음
#include "tb_path/laneSkeleton.hpp"
#include "tb_path/lanePath.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cstdio>
int main(int argc, char **argv) {
    const int S = 320;
    cv::Mat white(S, S, CV_8U, cv::Scalar(0)), yellow = white.clone(), valid(S, S, CV_8U, cv::Scalar(255));
    // 바깥 흰선: 오른쪽 아래에서 왼쪽 위로 (좌커브 바깥)
    std::vector<cv::Point> wl;
    for (int y = S - 1; y >= 0; y -= 4) { const double t = double(S - 1 - y) / S; wl.push_back({int(260 - 330 * t * t), y}); }
    cv::polylines(white, wl, false, 255, 12);
    lane_skel::Params sp; sp.lane_w = 172;
    lane_path::Geo g; g.lane_w_px = 172; g.m_per_px_x = 0.26 / 172; g.m_per_px_y = 0.38 / 320; g.bottom_m = 0.19;

    cv::Mat brick(S, S, CV_8U, cv::Scalar(0));
    { // 안쪽(로봇 쪽)이 벽돌 박스로 다 막힘 → 열린 곳은 흰선 너머 바닥뿐
        std::vector<cv::Point> poly = wl; poly.push_back({0, 0}); poly.push_back({0, S - 1});
        cv::fillPoly(brick, std::vector<std::vector<cv::Point>>{poly}, 255);
        brick.setTo(0, white);
    }

    int fails = 0;
    for (int cross : {0, 15}) {
        lane_path::Tracker tr;
        tr.p.override_frames = 1;
        tr.p.cross_override_frames = cross;
        lane_path::Pose2D pose;
        cv::Point none(-1, -1);
        // 1) 벽돌 없음: 안쪽 경로를 기억
        for (int i = 0; i < 3; ++i) {
            auto sk = lane_skel::build(white, valid, sp, &none);
            tr.update(sk, sp.scale, white, yellow, valid, g, lane_path::Turn::Straight, pose);
        }
        // 2) 안쪽이 막힘
        const cv::Mat wall = white | brick;
        int live = 0, mem = 0, first_live = -1;
        for (int i = 0; i < 20; ++i) {
            auto sk = lane_skel::build(wall, valid, sp, &none);
            auto o = tr.update(sk, sp.scale, white, yellow, valid, g, lane_path::Turn::Straight, pose);
            if (i == 0 && argc > 1) {
                cv::Mat v; cv::Mat sk2; cv::resize(sk.skel, sk2, brick.size(), 0, 0, cv::INTER_NEAREST); cv::merge(std::vector<cv::Mat>{brick, sk2, white}, v);
                lane_path::draw_path(v, o, g); cv::imwrite(argv[1], v);
            }
            if (o.src == lane_path::Source::Live) { ++live; if (first_live < 0) first_live = i; }
            if (o.src == lane_path::Source::Memory) ++mem;
            if (i == 0 && o.src == lane_path::Source::Live && !o.path.empty()) {
                const auto b = lane_path::robot_to_bev(o.path.front(), g);
                std::printf("  선 너머 경로 시작 BEV (%.0f,%.0f)\n", b.x, b.y);
            }
        }
        std::printf("cross_override_frames=%2d: live %d, memory %d, 처음 live 프레임 %d\n", cross, live, mem, first_live);
        if (cross == 0 && first_live != 0) ++fails;   // 재현: 바로 넘어감
        if (cross == 15 && (first_live >= 0 && first_live < 14)) ++fails;
    }
    std::printf(fails ? "FAIL\n" : "OK\n");
    return fails;
}
