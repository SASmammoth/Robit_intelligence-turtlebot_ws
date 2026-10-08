// 원본 영상의 벽돌 박스 → BEV 벽 → 스켈레톤 경로가 벽돌을 피해 가는지 확인
#include "tb_path/laneSkeleton.hpp"
#include "tb_path/lanePath.hpp"
#include "tb_path/obstacles.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cstdio>

int main()
{
    // tb_line 기본값과 같은 BEV (640x480, top_y 40, top_w 32, bot_w 100)
    const float w = 640, h = 480, cx = w / 2, ty = h * 0.40f, by = h - 1, tw = w * 0.16f, bw = w * 0.5f;
    const std::vector<cv::Point2f> src = {{cx - tw, ty}, {cx + tw, ty}, {cx + bw, by}, {cx - bw, by}};
    const float S = 320, m = S * 0.25f;
    const std::vector<cv::Point2f> dst = {{m, 0}, {S - m, 0}, {S - m, S}, {m, S}};
    const cv::Mat H = cv::getPerspectiveTransform(src, dst);
    cv::Mat valid;
    cv::warpPerspective(cv::Mat(480, 640, CV_8U, cv::Scalar(255)), valid, H, cv::Size(320, 320),
                        cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);

    // 직선 차선: 노랑 x=75, 흰 x=245 (안쪽 간격 160)
    cv::Mat white = cv::Mat::zeros(320, 320, CV_8U), yellow = white.clone();
    cv::line(yellow, {75, 0}, {75, 319}, 255, 10);
    cv::line(white, {245, 0}, {245, 319}, 255, 10);
    white &= valid; yellow &= valid;

    // 벽돌: BEV에서 아랫변이 (165..230, y=150) 인 위치 → 원본 영상 박스로 역변환
    std::vector<cv::Point2f> bev_pts = {{165, 150}, {230, 150}}, img_pts;
    cv::perspectiveTransform(bev_pts, img_pts, H.inv());
    const obstacles::Box box{int(img_pts[0].x), int(img_pts[0].y) - 40, int(img_pts[1].x - img_pts[0].x), 40};
    std::printf("brick box in camera image: x=%d y=%d w=%d h=%d\n", box.x, box.y, box.w, box.h);

    lane_path::Geo g;   // 0.26/160 m/px, 0.52/320 m/px, bottom 0.15 m
    cv::Mat wall = white | yellow;
    const int n = obstacles::draw_boxes(wall, H, {box}, 0.06 / g.my(), 0.03 / g.mx());
    const cv::Mat brick = wall & ~(white | yellow);
    const cv::Rect bb = cv::boundingRect(brick);
    std::printf("drawn=%d  brick wall in BEV: x %d..%d  y %d..%d\n", n, bb.x, bb.br().x, bb.y, bb.br().y);

    int fails = 0;
    if (n != 1 || bb.y > 150 || bb.br().y < 140 || bb.x > 165) { std::printf("FAIL: 벽 위치\n"); ++fails; }

    lane_skel::Params sp;
    const auto sk = lane_skel::build(wall, valid, sp);
    lane_path::Tracker tr;
    const auto o = tr.update(sk, sp.scale, white, yellow, valid, g, lane_path::Turn::Straight, {});

    // 벽돌 옆(BEV y=150 부근 = 로봇 앞 x) 에서 경로가 왼쪽(+y)으로 비켜 있는지
    const double x_brick = lane_path::bev_to_robot({160, 140}, g).x;
    double y_at = 0, best = 1e9;
    for (const auto &q : o.path)
        if (std::abs(q.x - x_brick) < best) { best = std::abs(q.x - x_brick); y_at = q.y; }
    const double lane_left_edge = lane_path::bev_to_robot({80, 0}, g).y;
    const double brick_left = lane_path::bev_to_robot({double(bb.x), 0}, g).y;
    std::printf("src=%s  path at %.2fm ahead: y=%+.3f m  (gap between lane %+.3f and brick %+.3f)\n",
                lane_path::source_name(o.src), x_brick, y_at, lane_left_edge, brick_left);
    if (o.src != lane_path::Source::Live || !(y_at < lane_left_edge && y_at > brick_left)) {
        std::printf("FAIL: 경로가 빈 틈을 지나지 않음\n"); ++fails;
    }

    cv::Mat view(320, 320, CV_8UC3, cv::Scalar(70, 70, 70));
    lane_skel::draw_debug(view, valid, sk, sp.scale);
    view.setTo(cv::Scalar(0, 0, 200), brick);
    view.setTo(cv::Scalar(235, 235, 235), white);
    view.setTo(cv::Scalar(0, 210, 255), yellow);
    lane_path::draw_path(view, o, g);
    cv::imwrite("brick_test.png", view);
    std::printf("%s\n", fails ? "FAILED" : "ALL OK");
    return fails;
}
