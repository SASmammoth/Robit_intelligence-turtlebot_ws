#include "tb_path/obstacles.hpp"

#include <opencv2/imgproc.hpp>
#include <cmath>
#include <algorithm>

namespace obstacles {

namespace {

// 벽돌 하나(q)의 줄마다 양 끝에서 바깥으로 gap 이내에 선이 있으면 그 사이를 채움
void close_gaps(cv::Mat &wall, const cv::Mat &q, const cv::Mat &lines, int gap)
{
    const cv::Rect bb = cv::boundingRect(q);
    for (int y = bb.y; y < bb.y + bb.height; ++y)
    {
        const uchar *qr = q.ptr<uchar>(y);
        const uchar *lr = lines.ptr<uchar>(y);
        uchar *wr = wall.ptr<uchar>(y);
        int xl = -1, xr = -1;
        for (int x = bb.x; x < bb.x + bb.width; ++x)
            if (qr[x])
            {
                if (xl < 0) xl = x;
                xr = x;
            }
        if (xl < 0)
            continue;
        for (int x = xr + 1; x <= std::min(wall.cols - 1, xr + gap); ++x)
            if (lr[x])
            {
                std::fill(wr + xr + 1, wr + x, uchar(255));
                break;
            }
        for (int x = xl - 1; x >= std::max(0, xl - gap); --x)
            if (lr[x])
            {
                std::fill(wr + x + 1, wr + xl, uchar(255));
                break;
            }
    }
}

} // namespace

int draw_boxes(cv::Mat &wall, const cv::Mat &H, const std::vector<Box> &boxes,
               double depth_px, double margin_px, const cv::Mat *lines, double gap_px)
{
    const bool close = lines && !lines->empty() && gap_px >= 1.0;
    cv::Mat q;
    int n = 0;
    const cv::Rect canvas(0, 0, wall.cols, wall.rows);
    for (const auto &b : boxes) {
        // 아랫변 양 끝 (원본 영상)
        std::vector<cv::Point2f> src = {{float(b.x), float(b.y + b.h)},
                                        {float(b.x + b.w), float(b.y + b.h)}};
        std::vector<cv::Point2f> dst;
        cv::perspectiveTransform(src, dst, H);
        cv::Point2f a = dst[0], c = dst[1];

        // 위쪽(멀리)으로 벗어난 건 맨 윗줄로 당겨서 그림. 옆으로 벗어난 것만 무시
        const bool beyond_top = a.y < 0 && c.y < 0;
        if (beyond_top)
        {
            if (std::max(a.x, c.x) < 0 || std::min(a.x, c.x) >= wall.cols)
                continue; // 멀리 있으면서 옆으로도 벗어남 → 무시
            a.y = 0;
            c.y = 0;
        }
        else if (!canvas.contains(cv::Point(a)) && !canvas.contains(cv::Point(c)))
            continue;

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
        if (close)
        {
            q.create(wall.size(), CV_8U);
            q.setTo(0);
            cv::fillConvexPoly(q, quad, cv::Scalar(255));
            close_gaps(wall, q, *lines, int(std::lround(gap_px)));
        }
        ++n;
    }
    return n;
}

} // namespace obstacles
