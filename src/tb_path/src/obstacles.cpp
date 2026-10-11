#include "tb_path/obstacles.hpp"

#include <opencv2/imgproc.hpp>
#include <cmath>
#include <algorithm>

namespace obstacles {

namespace {

// 벽돌 하나(q: 아직 wall에 안 그린 사각형)를 wall에 그림
//  - 줄마다 양 끝에서 바깥으로 gap 이내에 선이 있으면 그 사이를 채움.
//    한쪽에서 찾은 (벽돌 끝, 선) 점들을 볼록 다각형으로 통째로 채워 얇은 틈이 안 남게
//  - width > 0이고 한쪽만 선에 닿았으면: 벽돌은 그 선에 붙은 것. 박스는 옆면·곡선 때문에 실제보다 넓게
//    잡히므로, 반대쪽 끝을 "닿은 선에서 width"까지로 줄임 (반대쪽은 지나갈 길)
//  - edge > 0이고 양쪽 다 선에 안 닿았으면 (= 화면 밖 선에 붙은 벽돌): 화면 끝에 더 가까운 쪽 하나만
//    화면 끝까지 막고, 그 아래 화면 끝도 막음 (화면 밖 선 대신)
void draw_brick(cv::Mat &wall, cv::Mat &q, const cv::Mat &lines, int gap, int edge, int width)
{
    struct Row { int y, xl, xr, cl, cr; }; // cl/cr: 왼쪽/오른쪽에 닿은 선 x (-1이면 없음)
    std::vector<Row> rows;
    const cv::Rect bb = cv::boundingRect(q);
    for (int y = bb.y; y < bb.y + bb.height; ++y)
    {
        const uchar *qr = q.ptr<uchar>(y);
        int xl = -1, xr = -1;
        for (int x = bb.x; x < bb.x + bb.width; ++x)
            if (qr[x])
            {
                if (xl < 0) xl = x;
                xr = x;
            }
        if (xl < 0)
            continue;
        Row r{y, xl, xr, -1, -1};
        if (gap > 0)
        {
            const uchar *lr = lines.ptr<uchar>(y);
            // 박스 안쪽에 선이 겹쳐 있어도 닿은 것 (벽돌이 선을 가림)
            for (int x = std::max(0, xl - gap); x <= std::min(wall.cols - 1, xr + gap); ++x)
                if (lr[x])
                {
                    if (x <= (xl + xr) / 2 && x >= xl - gap)
                        r.cl = x; // 왼쪽: 가장 안쪽(오른쪽) 선 픽셀
                    else if (x > (xl + xr) / 2 && r.cr < 0)
                        r.cr = x; // 오른쪽: 가장 안쪽(왼쪽) 선 픽셀
                }
        }
        rows.push_back(r);
    }
    if (rows.empty())
        return;

    int nl = 0, nr = 0;
    std::vector<int> cls, crs;
    for (const auto &r : rows)
    {
        if (r.cl >= 0) ++nl, cls.push_back(r.cl);
        if (r.cr >= 0) ++nr, crs.push_back(r.cr);
    }
    const bool att_r = nr > 0 && nl == 0, att_l = nl > 0 && nr == 0;
    auto median = [](std::vector<int> v)
    {
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
    };
    const int mcr = crs.empty() ? -1 : median(crs), mcl = cls.empty() ? -1 : median(cls);

    std::vector<cv::Point> side_pts[2]; // [0] 오른쪽, [1] 왼쪽
    for (auto &r : rows)
    {
        if (width > 0 && att_r)
            r.xl = std::max(r.xl, (r.cr >= 0 ? r.cr : mcr) - width);
        if (width > 0 && att_l)
            r.xr = std::min(r.xr, (r.cl >= 0 ? r.cl : mcl) + width);
        if (r.xl > r.xr)
            continue;
        uchar *wr = wall.ptr<uchar>(r.y);
        std::fill(wr + r.xl, wr + r.xr + 1, uchar(255));
        if (r.cr > r.xr)
        {
            std::fill(wr + r.xr + 1, wr + r.cr, uchar(255));
            side_pts[0].push_back({r.xr, r.y});
            side_pts[0].push_back({r.cr, r.y});
        }
        if (r.cl >= 0 && r.cl < r.xl)
        {
            std::fill(wr + r.cl + 1, wr + r.xl, uchar(255));
            side_pts[1].push_back({r.xl, r.y});
            side_pts[1].push_back({r.cl, r.y});
        }
    }
    for (const auto &pts : side_pts)
        if (pts.size() >= 4)
        {
            std::vector<cv::Point> hull;
            cv::convexHull(pts, hull);
            cv::fillConvexPoly(wall, hull, cv::Scalar(255));
        }

    if (edge <= 0 || nl > 0 || nr > 0)
        return;
    int xmin = wall.cols, xmax = -1;
    for (const auto &r : rows)
        xmin = std::min(xmin, r.xl), xmax = std::max(xmax, r.xr);
    const int dl = xmin, dr = wall.cols - 1 - xmax; // 각 끝에서 화면 끝까지
    const bool left = dl <= edge && dl <= dr;
    const bool right = !left && dr <= edge;
    for (const auto &r : rows)
    {
        uchar *wr = wall.ptr<uchar>(r.y);
        if (right)
            std::fill(wr + r.xr, wr + wall.cols, uchar(255));
        if (left)
            std::fill(wr, wr + r.xl + 1, uchar(255));
    }
    const int strip = 4, y0 = rows.front().y;
    if (left)
        cv::rectangle(wall, cv::Rect(0, y0, strip, wall.rows - y0), cv::Scalar(255), cv::FILLED);
    if (right)
        cv::rectangle(wall, cv::Rect(wall.cols - strip, y0, strip, wall.rows - y0), cv::Scalar(255), cv::FILLED);
}

} // namespace

int draw_boxes(cv::Mat &wall, const cv::Mat &H, const std::vector<Box> &boxes,
               double depth_px, double margin_px, const cv::Mat *lines, double gap_px, double edge_px,
               double width_px)
{
    const bool close = lines && !lines->empty() && (gap_px >= 1.0 || edge_px >= 1.0);
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
        if (close)
        {
            q.create(wall.size(), CV_8U);
            q.setTo(0);
            cv::fillConvexPoly(q, quad, cv::Scalar(255));
            draw_brick(wall, q, *lines, int(std::lround(gap_px)), int(std::lround(edge_px)),
                       width_px >= 1.0 ? int(std::lround(width_px + margin_px)) : 0);
        }
        else
            cv::fillConvexPoly(wall, quad, cv::Scalar(255));
        ++n;
    }
    return n;
}

} // namespace obstacles
