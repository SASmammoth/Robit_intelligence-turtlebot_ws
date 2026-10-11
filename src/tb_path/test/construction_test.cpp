// 빌드: cd tb_path && g++ -O2 -std=c++17 -Iinclude test/construction_test.cpp src/laneSkeleton.cpp src/lanePath.cpp src/obstacles.cpp $(pkg-config --cflags --libs opencv4) -o construction_test
// 실행: ./construction_test 1 200   (1=공사 구간 설정, 200=벽돌 앞면 BEV 줄)
// 공사 구간 화면 재현: 흰 선만 오른쪽에, 벽돌이 왼쪽(화면 밖 선에 붙음) → 경로가 벽돌 오른쪽 틈으로 가는지
#include "tb_path/laneSkeleton.hpp"
#include "tb_path/lanePath.hpp"
#include "tb_path/obstacles.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cstdio>
#include <cstdlib>
int main(int argc, char **argv)
{
    const bool con = argc > 1 && atoi(argv[1]);
    const int face_y = argc > 2 ? atoi(argv[2]) : 200;
    const float w = 640, h = 480, cx = w / 2, ty = h * 0.40f, by = h - 1, tw = w * 0.16f, bw = w * 0.5f;
    const std::vector<cv::Point2f> src = {{cx - tw, ty}, {cx + tw, ty}, {cx + bw, by}, {cx - bw, by}};
    const float S = 320, m = S * 0.25f;
    const std::vector<cv::Point2f> dst = {{m, 0}, {S - m, 0}, {S - m, S}, {m, S}};
    const cv::Mat H = cv::getPerspectiveTransform(src, dst);
    cv::Mat valid;
    cv::warpPerspective(cv::Mat(480, 640, CV_8U, cv::Scalar(255)), valid, H, cv::Size(320, 320), cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    lane_path::Geo g;
    // 공사 구간 56cm: 로봇이 가운데면 선은 ±0.28m → 화면(±0.26m) 밖. 로봇이 왼쪽으로 4cm 치우쳐 오른선만 보임
    cv::Mat white = cv::Mat::zeros(320, 320, CV_8U), yellow = white.clone();
    const double rx = lane_path::robot_to_bev({0.3, -0.24}, g).x;
    cv::line(white, {int(rx), 0}, {int(rx), 319}, 255, 10);
    white &= valid;
    // 왼쪽 벽돌 20cm: 왼선(y=+0.32)에 붙음 → 박스 왼끝은 화면 밖
    const double xf = lane_path::bev_to_robot({0, double(face_y)}, g).x;
    std::vector<cv::Point2f> bev = {lane_path::robot_to_bev({xf, 0.32}, g), lane_path::robot_to_bev({xf, 0.12}, g)}, img;
    cv::perspectiveTransform(bev, img, H.inv());
    const obstacles::Box box{int(img[0].x), int(img[0].y) - 40, int(img[1].x - img[0].x), 40};
    cv::Mat wall = white | yellow;
    const cv::Mat lines = wall.clone();
    const double depth = con ? 0.15 : 0.10, margin = con ? 0.06 : 0.05;
    obstacles::draw_boxes(wall, H, {box}, depth / g.my(), margin / g.mx(), &lines, 0.12 / g.mx(), con ? 0.06 / g.mx() : 0);
    lane_skel::Params sp;
    sp.lane_w = con ? int(0.56 / g.mx()) : 160;
    const auto sk = lane_skel::build(wall, valid, sp);
    lane_path::Tracker tr;
    const auto o = tr.update(sk, sp.scale, white, yellow, valid, g, lane_path::Turn::Straight, {});
    // 벽돌 앞면 줄에서 경로 y (벽돌 안쪽 끝 0.12-0.06=0.06 보다 오른쪽(작은 y)이어야)
    double y_at = 99, best = 1e9;
    for (const auto &q : o.path)
        if (std::abs(q.x - (xf + 0.04)) < best) best = std::abs(q.x - (xf + 0.04)), y_at = q.y;
    double maxy = -1;
    for (const auto &q : o.path) maxy = std::max(maxy, q.y);
    std::printf("con=%d face_y=%d src=%s  벽돌 옆 경로 y=%+.3f (벽돌 끝 +0.12, 선 -0.24)  경로 최대 y=%+.3f  n=%zu %s\n", con, face_y,
                lane_path::source_name(o.src), y_at, maxy, o.path.size(), (best < 0.03 && y_at < 0.06 && y_at > -0.2) ? "OK" : "FAIL");
    cv::Mat view(320, 320, CV_8UC3, cv::Scalar(70, 70, 70));
    lane_skel::draw_debug(view, valid, sk, sp.scale);
    view.setTo(cv::Scalar(0, 0, 200), wall & ~lines);
    view.setTo(cv::Scalar(235, 235, 235), white);
    lane_path::draw_path(view, o, g);
    cv::imwrite(argc > 3 ? argv[3] : "con.png", view);
}
