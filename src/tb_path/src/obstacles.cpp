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
//  - attach > gap이면: 한쪽만 선에 닿았을 때 그쪽은 벽돌의 모든 줄을 attach 안의 선까지 막음
//    (곡선에서 선이 벽돌 아래쪽 줄에서 멀어지며 생기는 주머니로 경로가 파고들지 않게)
//  - off_side: 박스 아랫변이 화면 밖으로 나감 (-1 왼쪽, +1 오른쪽, 0 아님) = 그쪽 화면 밖 선에 붙은 벽돌.
//    반대쪽 선에 닿아 보여도(곡선에서 먼 줄만 닿음) 그쪽에 붙은 걸로 보지 않음 → 반대쪽 틈은 지나갈 길
//    (녹화 run_1027 6.8s: 왼쪽 벽돌의 먼 줄이 휘어 들어온 흰 선에 닿아 오른쪽 틈까지 막고, 경로가 벽돌 앞으로 감)
// 반환: 벽돌이 붙은 쪽 (+1 오른쪽 선, -1 왼쪽 선, 0 모름: 양쪽 다 닿음 / 선도 화면 끝도 아님)
int draw_brick(cv::Mat &wall, cv::Mat &q, const cv::Mat &lines, int gap, int edge, int width, int attach, int off_side)
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
        return 0;

    if (off_side != 0)
        for (auto &r : rows)
            (off_side < 0 ? r.cr : r.cl) = -1; // 화면 밖 선에 붙은 벽돌: 반대쪽 선과는 안 붙음
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

    // 붙은 쪽: 선이 gap보다 멀리 있는 줄도 attach 안이면 선까지
    if (attach > gap && (att_r || att_l))
        for (auto &r : rows)
        {
            const uchar *lr = lines.ptr<uchar>(r.y);
            if (att_r && r.cr < 0)
                for (int x = r.xr + 1; x <= std::min(wall.cols - 1, r.xr + attach); ++x)
                    if (lr[x]) { r.cr = x; break; }
            if (att_l && r.cl < 0)
                for (int x = r.xl - 1; x >= std::max(0, r.xl - attach); --x)
                    if (lr[x]) { r.cl = x; break; }
        }

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
    {
        if (att_r || att_l)
            return att_r ? 1 : -1;
        // 양쪽 다 닿음: 곡선에서 붙은 선이 벽돌 먼 줄 아래로 휘어 반대쪽에서도 닿음.
        // 로봇에 가장 가까운 줄(앞면)에서 한쪽만 닿은 쪽이 붙은 선
        for (auto it = rows.rbegin(); it != rows.rend(); ++it)
        {
            if (it->cr >= 0 && it->cl < 0)
                return 1;
            if (it->cl >= 0 && it->cr < 0)
                return -1;
        }
        return 0;
    }
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
    return right ? 1 : left ? -1 : 0;
}

} // namespace

int draw_boxes(cv::Mat &wall, const cv::Mat &H, const std::vector<Box> &boxes,
               double depth_px, double margin_px, const cv::Mat *lines, double gap_px, double edge_px,
               double width_px, double attach_px, bool keep_far, bool off_screen, std::vector<int> *sides)
{
    const bool close = lines && !lines->empty() && (gap_px >= 1.0 || edge_px >= 1.0);
    cv::Mat q;
    int n = 0;
    const cv::Rect canvas(0, 0, wall.cols, wall.rows);
    if (sides)
        sides->assign(boxes.size(), 0);
    for (size_t bi = 0; bi < boxes.size(); ++bi) {
        const Box &b = boxes[bi];
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
            if (!keep_far)
                continue;
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
        // 아랫변이 화면 옆 끝(edge_px의 절반 안)까지 가거나 넘어간 쪽 (멀리 있어 위로 당긴 것은 판단 안 함)
        //   오른쪽 선에 붙은 벽돌(20cm)이 화면 왼쪽 끝 7cm 안까지 오는 일은 거의 없음
        int off_side = 0;
        if (off_screen && !beyond_top)
        {
            const float o = float(edge_px) * 0.5f;
            const bool l = std::min(a.x, c.x) < o, r = std::max(a.x, c.x) >= wall.cols - o;
            if (l && !r)
                off_side = -1;
            else if (r && !l)
                off_side = 1;
        }
        if (close)
        {
            q.create(wall.size(), CV_8U);
            q.setTo(0);
            cv::fillConvexPoly(q, quad, cv::Scalar(255));
            // 화면 위로 당겨 그린 먼 벽돌은 선이 안 보여 붙은 쪽을 모르므로 화면 끝 막기는 안 함
            const int side = draw_brick(wall, q, *lines, int(std::lround(gap_px)), beyond_top ? 0 : int(std::lround(edge_px)),
                                        width_px >= 1.0 ? int(std::lround(width_px + margin_px)) : 0, int(std::lround(attach_px)), off_side);
            if (sides && !beyond_top)
                (*sides)[bi] = side;
        }
        else
            cv::fillConvexPoly(wall, quad, cv::Scalar(255));
        ++n;
    }
    return n;
}

int extend_occluded(const cv::Mat &lines, const cv::Mat &bricks, cv::Mat &out,
                    double probe_px, double len_px, int thick)
{
    cv::Mat labels, stats, cents;
    const int n = cv::connectedComponentsWithStats(lines, labels, stats, cents, 8, CV_32S);
    int count = 0;
    for (int k = 1; k < n; ++k)
    {
        if (stats.at<int>(k, cv::CC_STAT_AREA) < 30)
            continue;
        const int top = stats.at<int>(k, cv::CC_STAT_TOP);
        if (top <= 2)
            continue; // 화면 끝까지 이어진 선
        // 위쪽 끝 근처 점들로 방향 (끝 40px), 끝점 (끝 4px 평균)
        std::vector<cv::Point2f> seg;
        cv::Point2f tip(0, 0);
        int nt = 0;
        const int y1 = std::min(lines.rows - 1, top + 40);
        for (int y = top; y <= y1; ++y)
        {
            const int *lr = labels.ptr<int>(y);
            for (int x = 0; x < lines.cols; ++x)
                if (lr[x] == k)
                {
                    seg.emplace_back(float(x), float(y));
                    if (y <= top + 4)
                        tip += cv::Point2f(float(x), float(y)), ++nt;
                }
        }
        if (seg.size() < 20 || nt == 0)
            continue;
        tip *= 1.0f / nt;
        cv::Vec4f fl;
        cv::fitLine(seg, fl, cv::DIST_L2, 0, 0.01, 0.01);
        cv::Point2f d(fl[0], fl[1]);
        if (d.y > 0)
            d = -d; // 위쪽(먼 쪽)으로
        if (-d.y < 0.3f)
            continue; // 거의 가로선은 안 이음
        // 끝에서 probe 안에 벽돌이 있어야 (= 벽돌이 선을 가림)
        bool hit = false;
        for (float t = 0; t <= probe_px && !hit; t += 1.0f)
        {
            const cv::Point2f p = tip + d * t;
            for (int o = -thick; o <= thick && !hit; ++o)
            {
                const cv::Point q(cvRound(p.x + o), cvRound(p.y));
                if (q.x >= 0 && q.y >= 0 && q.x < bricks.cols && q.y < bricks.rows && bricks.at<uchar>(q))
                    hit = true;
            }
        }
        if (!hit)
            continue;
        cv::line(out, tip, tip + d * float(len_px), cv::Scalar(255), thick, cv::LINE_8);
        ++count;
    }
    return count;
}

void keep_clear(std::vector<cv::Point2d> &path, const std::vector<Quad> &bricks,
                double front, double rear, double half, double clear, double hold, bool soft)
{
    for (const auto &q : bricks)
    {
        double x0 = 1e9, x1 = -1e9, yl = -1e9, yr = 1e9;
        for (const auto &p : q)
        {
            x0 = std::min(x0, p.x);
            x1 = std::max(x1, p.x);
            yl = std::max(yl, p.y);
            yr = std::min(yr, p.y);
        }
        const bool right = yl < 0.0, left = yr > 0.0; // 벽돌이 로봇 오른쪽 / 왼쪽
        if (!right && !left)
            continue;
        const double a = x0 - front - clear, b = x1 + rear + clear + hold;
        double lim_r = yl + half + clear, lim_l = yr - half - clear;
        if (soft)
            lim_r = std::min(lim_r, 0.0), lim_l = std::max(lim_l, 0.0);
        for (auto &p : path)
        {
            if (p.x < a || p.x > b)
                continue;
            if (right)
                p.y = std::max(p.y, lim_r);
            else
                p.y = std::min(p.y, lim_l);
        }
    }
}

std::vector<Quad> bridge_same_side(std::vector<SideBrick> bricks, double max_gap, double min_gap,
                                   double max_dy, double margin, double far, bool from_back, double far_dy)
{
    std::vector<Quad> out;
    if (max_gap <= 0.0)
        return out;
    std::sort(bricks.begin(), bricks.end(), [](const SideBrick &a, const SideBrick &b) { return a.x0 < b.x0; });
    for (const bool right : {true, false})
    {
        const SideBrick *prev = nullptr;
        for (const auto &b : bricks)
        {
            if (b.right != right)
                continue;
            if (prev)
            {
                const double gap = b.x0 - prev->x1;
                const double lim = (b.far || prev->far) ? far_dy : max_dy;
                if (gap > min_gap && gap <= max_gap && (lim <= 0.0 || std::abs(b.yin - prev->yin) <= lim))
                {
                    // 길 쪽 끝: 두 벽돌 중 더 길 쪽으로 나온 것 + 여유. 선 쪽은 far만큼 (화면 밖까지)
                    // 두 벽돌 안쪽 끝을 잇는 선 (곡선에서는 비스듬함) + 여유, 선 쪽으로 far만큼
                    const double s = right ? 1.0 : -1.0; // 길 쪽 = +y(오른쪽 벽돌) / -y(왼쪽 벽돌)
                    const double xs = from_back ? prev->x1 : prev->x0, xe = b.x0 + 0.02;
                    const double ya = prev->yin + s * margin, yb = b.yin + s * margin;
                    out.push_back({cv::Point2d(xs, ya), cv::Point2d(xe, yb),
                                   cv::Point2d(xe, yb - s * far), cv::Point2d(xs, ya - s * far)});
                }
            }
            if (!prev || b.x1 > prev->x1)
                prev = &b; // 겹친 것(같은 벽돌)은 더 뒤까지 간 쪽을 기준으로
        }
    }
    return out;
}

int hug_line(std::vector<cv::Point2d> &path, const cv::Mat &lines, const cv::Mat &bricks,
             const lane_path::Geo &g, const HugParams &p)
{
    const size_t n = path.size();
    if (n < 3 || lines.empty() || bricks.empty())
        return 0;
    const cv::Rect canvas(0, 0, lines.cols, lines.rows);
    // 0: 없음, 1: 선, 2: 벽돌, -1: 화면 밖
    auto at = [&](const cv::Point2d &q)
    {
        const cv::Point2d b = lane_path::robot_to_bev(q, g);
        const cv::Point c(cvRound(b.x), cvRound(b.y));
        if (!canvas.contains(c))
            return -1;
        if (bricks.at<uchar>(c))
            return 2;
        if (lines.at<uchar>(c))
            return 1;
        return 0;
    };
    std::vector<double> sh(n, 0.0);
    std::vector<cv::Point2d> nrm(n);
    const double step = 0.005;
    for (size_t i = 0; i < n; ++i)
    {
        cv::Point2d t = path[std::min(i + 1, n - 1)] - path[i > 0 ? i - 1 : 0];
        const double L = std::hypot(t.x, t.y);
        if (L < 1e-6)
            continue;
        t *= 1.0 / L;
        nrm[i] = {-t.y, t.x}; // 왼쪽
        if (at(path[i]) != 0)
            continue; // 화면 밖(사각지대 포함) 또는 이미 벽·선 위
        double d_brick[2] = {1e9, 1e9}, d_line[2] = {1e9, 1e9}; // [0] 왼쪽, [1] 오른쪽
        for (int s = 0; s < 2; ++s)
        {
            const double sg = s == 0 ? 1.0 : -1.0;
            for (double d = step; d <= p.reach; d += step)
            {
                const int v = at(path[i] + nrm[i] * (sg * d));
                if (v == -1)
                    break;
                if (v == 2) { d_brick[s] = d; break; }
                if (v == 1) { d_line[s] = d; break; }
            }
        }
        for (int s = 0; s < 2; ++s)
        {
            const int o = 1 - s; // s 쪽에 벽돌, o 쪽에 선
            if (d_brick[s] > p.zone || d_brick[o] < 1e8 || d_line[o] > 1e8)
                continue;
            const double move = std::min(p.max_shift, d_line[o] - p.keep);
            if (move > 0.0)
                sh[i] = (o == 0 ? 1.0 : -1.0) * move;
        }
    }
    // 부드럽게 (앞뒤 ±5점 평균, 2cm 간격이면 ±10cm) + 시작 부분 줄이기
    std::vector<double> sm(n, 0.0);
    for (size_t i = 0; i < n; ++i)
    {
        double sum = 0.0;
        int cnt = 0;
        for (size_t j = (i >= 5 ? i - 5 : 0); j <= std::min(n - 1, i + 5); ++j)
            sum += sh[j], ++cnt;
        sm[i] = sum / cnt;
    }
    int moved = 0;
    double s_len = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        if (i > 0)
            s_len += cv::norm(path[i] - path[i - 1]);
        const double w = p.taper > 1e-6 ? std::min(1.0, s_len / p.taper) : 1.0;
        const double d = sm[i] * w;
        if (std::abs(d) > 1e-4)
        {
            path[i] += nrm[i] * d;
            ++moved;
        }
    }
    return moved;
}

} // namespace obstacles
