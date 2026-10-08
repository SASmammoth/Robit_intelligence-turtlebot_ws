#include "tb_path/obstacles.hpp"

#include <opencv2/imgproc.hpp>
#include <cmath>

namespace obstacles {

int draw_boxes(cv::Mat &wall, const cv::Mat &H, const std::vector<Box> &boxes,
               double depth_px, double margin_px)
{
    int n = 0;
    const cv::Rect canvas(0, 0, wall.cols, wall.rows);
    for (const auto &b : boxes) {
        // 아랫변 양 끝 (원본 영상)
        std::vector<cv::Point2f> src = {{float(b.x), float(b.y + b.h)},
                                        {float(b.x + b.w), float(b.y + b.h)}};
        std::vector<cv::Point2f> dst;
        cv::perspectiveTransform(src, dst, H);
        const cv::Point2f a = dst[0], c = dst[1];

        // 둘 다 BEV 밖이면 무시 (너무 멀거나 옆)
        if (!canvas.contains(cv::Point(a)) && !canvas.contains(cv::Point(c))) continue;

        // 아랫변 방향과, 로봇에서 멀어지는 방향(BEV 위쪽에 가까운 법선)
        cv::Point2f along = c - a;
        const float L = std::sqrt(along.dot(along));
        if (L < 1e-3f) continue;
        along *= 1.0f / L;
        cv::Point2f away(along.y, -along.x);
        if (away.y > 0) away = -away;           // BEV에서 위쪽(먼 쪽)을 향하게

        const cv::Point2f m = along * float(margin_px);
        const cv::Point2f d = away * float(depth_px);
        const std::vector<cv::Point> quad = {
            cv::Point(a - m), cv::Point(c + m), cv::Point(c + m + d), cv::Point(a - m + d)};
        cv::fillConvexPoly(wall, quad, cv::Scalar(255));
        ++n;
    }
    return n;
}

} // namespace obstacles
