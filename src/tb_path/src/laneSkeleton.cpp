#include "tb_path/laneSkeleton.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cfloat>
#include <cmath>

namespace lane_skel {
namespace {

// ───────── 스켈레톤 기본 연산 ─────────

// 8방향 이웃 순서: N, NE, E, SE, S, SW, W, NW
constexpr int DX[8] = {0, 1, 1, 1, 0, -1, -1, -1};
constexpr int DY[8] = {-1, -1, 0, 1, 1, 1, 0, -1};

// 교차수: 이웃을 한 바퀴 돌 때 0→1 전환 횟수
// 1 = 끝점, 2 = 선 중간, 3 이상 = 분기점 (계단 모양에서도 오판 없음)
int crossing(const cv::Mat &s, int x, int y)
{
    int n = 0;
    for (int i = 0; i < 8; ++i) {
        const bool a = s.at<uchar>(y + DY[i], x + DX[i]);
        const bool b = s.at<uchar>(y + DY[(i + 1) % 8], x + DX[(i + 1) % 8]);
        if (!a && b) ++n;
    }
    return n;
}

cv::Mat crossing_map(const cv::Mat &s)
{
    cv::Mat A(s.size(), CV_8U, cv::Scalar(0));
    for (int y = 1; y < s.rows - 1; ++y)
        for (int x = 1; x < s.cols - 1; ++x)
            if (s.at<uchar>(y, x)) A.at<uchar>(y, x) = uchar(crossing(s, x, y));
    return A;
}

// Zhang-Suen 세선화 한 픽셀 판정
inline bool zs_deletable(const cv::Mat &m, int x, int y, int pass)
{
    const uchar *u = m.ptr(y - 1), *c = m.ptr(y), *d = m.ptr(y + 1);
    const int p2 = u[x], p3 = u[x + 1], p4 = c[x + 1], p5 = d[x + 1],
              p6 = d[x], p7 = d[x - 1], p8 = c[x - 1], p9 = u[x - 1];
    const int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
    if (B < 2 || B > 6) return false;
    const int A = (!p2 && p3) + (!p3 && p4) + (!p4 && p5) + (!p5 && p6) +
                  (!p6 && p7) + (!p7 && p8) + (!p8 && p9) + (!p9 && p2);
    if (A != 1) return false;
    return pass == 0 ? !((p2 && p4 && p6) || (p4 && p6 && p8))
                     : !((p2 && p4 && p8) || (p2 && p6 && p8));
}

// Zhang-Suen 세선화. 입력/출력 0·255, 테두리 1px은 0이어야 함
// 전체 영상을 매번 훑지 않고 경계 픽셀 목록만 검사함 (결과는 표준 방식과 동일)
void thin_zhang_suen(cv::Mat &img)
{
    cv::Mat m = img / 255, mark(img.size(), CV_8U, cv::Scalar(0));
    std::vector<cv::Point> cand, next, dels;

    for (int y = 1; y < m.rows - 1; ++y)                    // 처음 후보: 배경과 닿은 픽셀
        for (int x = 1; x < m.cols - 1; ++x) {
            if (!m.at<uchar>(y, x)) continue;
            for (int i = 0; i < 8; ++i)
                if (!m.at<uchar>(y + DY[i], x + DX[i])) {
                    cand.push_back({x, y});
                    mark.at<uchar>(y, x) = 1;
                    break;
                }
        }

    for (bool changed = true; changed;) {
        changed = false;
        for (int pass = 0; pass < 2; ++pass) {
            dels.clear();
            for (const auto &p : cand)
                if (m.at<uchar>(p) && zs_deletable(m, p.x, p.y, pass)) dels.push_back(p);
            if (dels.empty()) continue;
            changed = true;
            for (const auto &p : dels) m.at<uchar>(p) = 0;

            // 다음 후보 = 남은 후보 + 지워진 픽셀의 이웃
            for (const auto &p : cand) mark.at<uchar>(p) = 0;
            next.clear();
            for (const auto &p : cand)
                if (m.at<uchar>(p)) { next.push_back(p); mark.at<uchar>(p) = 1; }
            for (const auto &p : dels)
                for (int i = 0; i < 8; ++i) {
                    const cv::Point q(p.x + DX[i], p.y + DY[i]);
                    if (m.at<uchar>(q) && !mark.at<uchar>(q)) { next.push_back(q); mark.at<uchar>(q) = 1; }
                }
            cand.swap(next);
        }
    }
    img = m * 255;
}

// 끝점에서 스켈레톤을 따라 BFS. 분기점에 닿거나 max_len만큼 가면 멈춤
struct Walk {
    std::vector<cv::Point> pts;   // 지나온 픽셀 (끝점 포함, 분기점 제외)
    cv::Point2f far{0, 0};        // 마지막 층의 중심 (방향 계산용)
    int len = 0;
    bool hit_junction = false;
};

Walk walk_from(const cv::Mat &s, const cv::Mat &A, cv::Point e, int max_len)
{
    Walk w;
    w.pts.push_back(e);
    std::vector<cv::Point> layer{e};
    auto visited = [&](const cv::Point &q) {
        return std::find(w.pts.begin(), w.pts.end(), q) != w.pts.end();
    };

    while (w.len < max_len && !layer.empty()) {
        std::vector<cv::Point> next;
        for (const auto &p : layer)
            for (int i = 0; i < 8; ++i) {
                const cv::Point q(p.x + DX[i], p.y + DY[i]);
                if (!s.at<uchar>(q) || visited(q)) continue;
                if (A.at<uchar>(q) >= 3) { w.hit_junction = true; continue; }
                next.push_back(q);
                w.pts.push_back(q);
            }
        if (w.hit_junction || next.empty()) break;
        layer = std::move(next);
        ++w.len;
    }
    for (const auto &p : layer) w.far += cv::Point2f(p);
    w.far *= 1.0f / float(layer.size());
    return w;
}

std::vector<cv::Point> find_ends(const cv::Mat &s, const cv::Mat &A)
{
    std::vector<cv::Point> e;
    for (int y = 1; y < s.rows - 1; ++y)
        for (int x = 1; x < s.cols - 1; ++x)
            if (A.at<uchar>(y, x) == 1) e.push_back({x, y});
    return e;
}

// 끝점에서 분기점까지 max_len 이하인 잔가지 삭제
void prune_spurs(cv::Mat &s, int max_len)
{
    const cv::Mat A = crossing_map(s);
    for (const auto &e : find_ends(s, A)) {
        if (!s.at<uchar>(e)) continue;
        const Walk w = walk_from(s, A, e, max_len);
        if (w.hit_junction && w.len < max_len)
            for (const auto &p : w.pts) s.at<uchar>(p) = 0;
    }
}

// ───────── 벽(차선) 끝 연장 ─────────

// tip에서 dir 방향으로 선을 그림.
//  cut = true : 캔버스 끝까지. 단, 사각지대를 지나 다시 보이는 영역에 들어오면 멈춤
//  cut = false: len 만큼
void extend_wall(cv::Mat &wall, const cv::Mat &valid, cv::Point2f tip, cv::Point2f dir,
                 bool cut, float len)
{
    cv::Point2f last = tip;
    const float max_t = cut ? float(wall.cols + wall.rows) : len;
    bool was_blind = false;
    for (float t = 1; t <= max_t; t += 1.0f) {
        const cv::Point2f q = tip + dir * t;
        const cv::Point qi(cvRound(q.x), cvRound(q.y));
        if (qi.x < 0 || qi.y < 0 || qi.x >= wall.cols || qi.y >= wall.rows) break;
        if (cut) {
            const bool vis = valid.at<uchar>(qi) != 0;
            if (!vis) was_blind = true;
            else if (was_blind) break;          // 사각지대 건너편 도로를 막지 않음
        }
        last = q;
    }
    cv::line(wall, tip, last, 255, 2);
}

void seal_lane_ends(const cv::Mat &lane, const cv::Mat &valid, cv::Mat &wall,
                    int seal_len, float max_half_thick)
{
    cv::Mat ls = lane.clone();
    thin_zhang_suen(ls);                                    // 차선의 중심선
    prune_spurs(ls, std::max(3, int(max_half_thick * 2)));  // 두꺼운 끝에서 생긴 잔가지

    cv::Mat half_thick;
    cv::distanceTransform(lane, half_thick, cv::DIST_L2, 3);

    constexpr int BACK = 8;                                 // 방향 계산에 쓸 길이
    const cv::Mat A = crossing_map(ls);

    for (const auto &e : find_ends(ls, A)) {
        const Walk w = walk_from(ls, A, e, BACK);
        if (w.len < BACK / 2) continue;                     // 너무 짧은 조각 (점, 분기 직전)
        const cv::Point fc(cvRound(w.far.x), cvRound(w.far.y));
        if (half_thick.at<float>(fc) > max_half_thick) continue;  // 차선이 아닌 두꺼운 덩어리

        const cv::Point2f tip(e);
        cv::Point2f dir = tip - w.far;
        const float n = float(cv::norm(dir));
        if (n < 1e-3f) continue;
        dir *= 1.0f / n;

        const cv::Point probe(cvRound(tip.x + dir.x * 4), cvRound(tip.y + dir.y * 4));
        const bool cut = probe.x < 0 || probe.y < 0 || probe.x >= valid.cols ||
                         probe.y >= valid.rows || valid.at<uchar>(probe) == 0;

        extend_wall(wall, valid, tip, dir, cut, float(seal_len));
    }
}

// ───────── 영역 정리 ─────────

// 영역 안의 구멍 중 벽이 하나도 없는 것(넓은 곳에서 생긴 가짜 구멍)만 메움. 섬은 유지
void fill_fake_holes(cv::Mat &region, const cv::Mat &wall)
{
    cv::Mat holes = (region == 0), lbl;
    const int n = cv::connectedComponents(holes, lbl, 4, CV_32S);
    std::vector<uchar> keep(n, 0);
    for (int y = 0; y < lbl.rows; ++y)
        for (int x = 0; x < lbl.cols; ++x) {
            const int l = lbl.at<int>(y, x);
            if (l && (wall.at<uchar>(y, x) || x == 0 || y == 0 ||
                      x == lbl.cols - 1 || y == lbl.rows - 1))
                keep[l] = 1;
        }
    for (int y = 0; y < lbl.rows; ++y)
        for (int x = 0; x < lbl.cols; ++x) {
            const int l = lbl.at<int>(y, x);
            if (l && !keep[l]) region.at<uchar>(y, x) = 255;
        }
}

// anchor에 가장 가까운 연결 성분만 남기고, 그 점을 돌려줌
cv::Point keep_nearest_component(cv::Mat &s, cv::Point anchor)
{
    std::vector<cv::Point> pts;
    cv::findNonZero(s, pts);
    if (pts.empty()) return {-1, -1};
    cv::Point best = pts[0];
    double bd = DBL_MAX;
    for (const auto &p : pts) {
        const double d = cv::norm(p - anchor);
        if (d < bd) { bd = d; best = p; }
    }
    cv::Mat lbl;
    if (cv::connectedComponents(s, lbl, 8, CV_32S) > 2)
        s = (lbl == lbl.at<int>(best));
    return best;
}

bool find_seed(const cv::Mat &cand, const cv::Rect &img, const cv::Point *hint, cv::Point &seed)
{
    if (hint) {                                             // 직전 프레임 위치 주변
        for (int r = 0; r <= 4; ++r)
            for (int dy = -r; dy <= r; ++dy)
                for (int dx = -r; dx <= r; ++dx) {
                    const cv::Point q(hint->x + dx, hint->y + dy);
                    if (img.contains(q) && cand.at<uchar>(q)) { seed = q; return true; }
                }
    }
    const int cx = img.x + img.width / 2;                   // 하단 중앙에서 바깥쪽으로
    for (int y = img.br().y - 1; y >= img.br().y - 10; --y)
        for (int dx = 0; dx < img.width / 2; ++dx)
            for (int s : {1, -1})
                if (cand.at<uchar>(y, cx + s * dx)) { seed = {cx + s * dx, y}; return true; }
    return false;
}

} // namespace

// ───────── 공개 함수 ─────────

cv::Mat make_valid_mask(const cv::Mat &M, const cv::Size &src, const cv::Size &dst, int erode_px)
{
    cv::Mat ones(src, CV_8U, cv::Scalar(255)), valid;
    cv::warpPerspective(ones, valid, M, dst, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    if (erode_px > 1)
        cv::erode(valid, valid, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erode_px, erode_px)));
    return valid;
}

void remove_dashes(cv::Mat &mask, int max_len)
{
    std::vector<std::vector<cv::Point>> cs;
    cv::findContours(mask, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < cs.size(); ++i) {
        const cv::RotatedRect r = cv::minAreaRect(cs[i]);
        if (std::max(r.size.width, r.size.height) < max_len)
            cv::drawContours(mask, cs, int(i), 0, cv::FILLED);
    }
}

Result build(const cv::Mat &lane, const cv::Mat &valid, const Params &p, const cv::Point *seed_hint)
{
    Result r;
    const int K = std::max(1, p.scale);
    const int W = std::max(p.lane_w / K, 4);
    const int P = W + 2;                                    // 여백 > 영역이 퍼질 수 있는 최대 거리
    const cv::Size small(lane.cols / K, lane.rows / K);

    // 0) 축소 + 여백 (여백 = 차선 없음, 사각지대)
    cv::Mat ln, vd;
    cv::resize(lane, ln, small, 0, 0, cv::INTER_AREA);
    ln = ln > 0;
    cv::resize(valid, vd, small, 0, 0, cv::INTER_NEAREST);
    cv::copyMakeBorder(ln, ln, P, P, P, P, cv::BORDER_CONSTANT, 0);
    cv::copyMakeBorder(vd, vd, P, P, P, P, cv::BORDER_CONSTANT, 0);
    const cv::Rect img(P, P, small.width, small.height);

    // 1) 벽 = 차선 + 끝 연장
    cv::Mat wall = ln.clone();
    seal_lane_ends(ln, vd, wall, p.seal_len / K, p.max_lane_thick / 2.0f / K);

    // 2) 벽까지 거리 (사각지대·화면 끝은 벽이 아님)
    if (cv::countNonZero(wall) == 0) return r;
    cv::Mat free_px = (wall == 0), dist;
    cv::distanceTransform(free_px, dist, cv::DIST_L2, 5);

    // 3) 도로 후보: 벽에서 W 미만 → 한쪽 차선만 보여도 중심선이 W/2에 생김
    cv::Mat cand = (dist > 0.5f) & (dist < float(W));

    // 4) 로봇과 이어진 영역만
    cv::Point seed;
    cv::Point hint_small;
    const cv::Point *hint = nullptr;
    // 직전 시드는 화면 아래쪽 30% 안에 있을 때만 믿음 (엉뚱한 도로로 옮겨가는 것 방지)
    if (seed_hint && seed_hint->x >= 0 && seed_hint->y > lane.rows * 7 / 10) {
        hint_small = cv::Point(seed_hint->x / K + P, seed_hint->y / K + P);
        hint = &hint_small;
    }
    if (!find_seed(cand, img, hint, seed)) return r;
    cv::floodFill(cand, seed, cv::Scalar(128), nullptr, cv::Scalar(0), cv::Scalar(0), 4);
    cv::Mat region = (cand == 128);
    fill_fake_holes(region, wall);
    cv::morphologyEx(region, region, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    cv::rectangle(region, cv::Rect(0, 0, region.cols, region.rows), 0, 1);  // 테두리 0 (세선화 조건)

    // 5) 세선화 → 잔가지 제거
    cv::Mat skel = region.clone();
    thin_zhang_suen(skel);
    prune_spurs(skel, std::max(2, p.prune_len / K));

    // 6) 여백 제거 → 로봇에 가장 가까운 줄기만
    //    사각지대 안의 중심선은 남겨둠: 영역 자체가 차선 거리로 추정한 것이라 믿을 만하고,
    //    잘라내면 회전 중에 경로가 사각지대에서 끊겨 짧은 조각만 남음
    cv::Mat out = skel(img).clone();
    // 기준은 항상 로봇 위치(화면 하단 중앙). 직전 시드를 기준으로 하면 프레임마다 옆으로 밀려남
    const cv::Point anchor(out.cols / 2, out.rows - 1);
    const cv::Point nearest = keep_nearest_component(out, anchor);
    if (nearest.x < 0) return r;

    // 7) 결과 정리 (끝점/분기점은 BEV 좌표로)
    cv::Mat pad;
    cv::copyMakeBorder(out, pad, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0);
    const cv::Mat A = crossing_map(pad);
    cv::Mat jmask = (A >= 3), jl, stats, cents;
    const int nj = cv::connectedComponentsWithStats(jmask, jl, stats, cents, 8);
    for (int i = 1; i < nj; ++i)                            // 붙어 있는 분기 픽셀은 하나로
        r.junctions.emplace_back(cvRound((cents.at<double>(i, 0) - 1) * K),
                                 cvRound((cents.at<double>(i, 1) - 1) * K));
    for (const auto &e : find_ends(pad, A))
        r.ends.emplace_back((e.x - 1) * K, (e.y - 1) * K);

    cv::resize(region(img), r.region, lane.size(), 0, 0, cv::INTER_NEAREST);
    r.skel = out;
    r.seed = nearest * K;
    r.ok = true;
    return r;
}

void draw_debug(cv::Mat &view, const cv::Mat &valid, const Result &r, int scale)
{
    view.setTo(cv::Scalar(40, 40, 40), valid == 0);                       // 사각지대: 회색
    if (!r.ok) return;
    cv::Mat tint = view * 0.6 + cv::Scalar(100, 60, 0);
    tint.copyTo(view, r.region);                                           // 도로: 푸른 틴트

    std::vector<cv::Point> pts;
    cv::findNonZero(r.skel, pts);
    for (const auto &p : pts)
        cv::circle(view, p * scale, 1, cv::Scalar(200, 255, 100), -1);     // 중심선: 민트
    for (const auto &j : r.junctions)
        cv::circle(view, j, 5, cv::Scalar(0, 140, 255), -1);               // 분기점: 주황
    for (const auto &e : r.ends)
        cv::circle(view, e, 4, cv::Scalar(60, 60, 255), 1);                // 끝점: 빨강
}

} // namespace lane_skel
