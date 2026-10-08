#include "tb_path/lanePath.hpp"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <functional>

namespace lane_path {
namespace {

using Pts = std::vector<cv::Point2d>;

constexpr int DX[8] = {0, 1, 1, 1, 0, -1, -1, -1};
constexpr int DY[8] = {-1, -1, 0, 1, 1, 1, 0, -1};

// ───────── 좌표 변환 ─────────

cv::Point2d to_world(const cv::Point2d &r, const Pose2D &p)
{
    const double c = std::cos(p.th), s = std::sin(p.th);
    return {p.x + c * r.x - s * r.y, p.y + s * r.x + c * r.y};
}

cv::Point2d to_robot(const cv::Point2d &w, const Pose2D &p)
{
    const double c = std::cos(p.th), s = std::sin(p.th);
    const double dx = w.x - p.x, dy = w.y - p.y;
    return {c * dx + s * dy, -s * dx + c * dy};
}

// ───────── 폴리라인 도구 ─────────

double seg_dist(const cv::Point2d &q, const cv::Point2d &a, const cv::Point2d &b)
{
    const cv::Point2d ab = b - a;
    const double L2 = ab.dot(ab);
    const double t = L2 > 0 ? std::clamp((q - a).dot(ab) / L2, 0.0, 1.0) : 0.0;
    return cv::norm(q - (a + ab * t));
}

double poly_dist(const cv::Point2d &q, const Pts &poly)
{
    if (poly.empty()) return DBL_MAX;
    if (poly.size() == 1) return cv::norm(q - poly[0]);
    double d = DBL_MAX;
    for (size_t i = 1; i < poly.size(); ++i) d = std::min(d, seg_dist(q, poly[i - 1], poly[i]));
    return d;
}

double poly_len(const Pts &p)
{
    double L = 0;
    for (size_t i = 1; i < p.size(); ++i) L += cv::norm(p[i] - p[i - 1]);
    return L;
}

// 일정 간격으로 다시 찍기
Pts resample(const Pts &in, double step)
{
    if (in.size() < 2) return in;
    Pts out{in[0]};
    double carry = 0;
    for (size_t i = 1; i < in.size(); ++i) {
        const cv::Point2d a = in[i - 1], b = in[i];
        const double L = cv::norm(b - a);
        double t = step - carry;
        while (t <= L) {
            out.push_back(a + (b - a) * (t / L));
            t += step;
        }
        carry = L - (t - step);
    }
    if (cv::norm(out.back() - in.back()) > step * 0.3) out.push_back(in.back());
    return out;
}

// 이동평균으로 스켈레톤의 계단 모양을 펴줌 (양 끝은 고정)
Pts smooth(const Pts &in, int half)
{
    if ((int)in.size() <= 2 * half) return in;
    Pts out = in;
    for (int i = half; i < (int)in.size() - half; ++i) {
        cv::Point2d s(0, 0);
        for (int k = -half; k <= half; ++k) s += in[i + k];
        out[i] = s * (1.0 / (2 * half + 1));
    }
    return out;
}

// ───────── 스켈레톤 따라가기 ─────────

// 들어온 방향 기준 회전각 (왼쪽 +, 영상 좌표 → 수학 좌표로 y 뒤집음)
double turn_angle(cv::Point2d din, cv::Point2d dout)
{
    din.y = -din.y;
    dout.y = -dout.y;
    return std::atan2(din.x * dout.y - din.y * dout.x, din.dot(dout));
}


// first에서 시작해 분기 없이 n픽셀 따라감 (방문 표시는 건드리지 않음)
std::vector<cv::Point> preview(const cv::Mat &s, const cv::Mat &vis, cv::Point from,
                               cv::Point first, int n)
{
    std::vector<cv::Point> pts{first};
    cv::Point prev = from, cur = first;
    for (int k = 0; k < n; ++k) {
        cv::Point nxt(-1, -1);
        double best = -1;
        for (int i = 0; i < 8; ++i) {
            const cv::Point q(cur.x + DX[i], cur.y + DY[i]);
            if (q.x < 0 || q.y < 0 || q.x >= s.cols || q.y >= s.rows) continue;
            if (!s.at<uchar>(q) || vis.at<uchar>(q) || q == prev) continue;
            if (std::find(pts.begin(), pts.end(), q) != pts.end()) continue;
            const double d = cv::norm(q - prev);   // 직진에 가까운 쪽 우선
            if (d > best) { best = d; nxt = q; }
        }
        if (nxt.x < 0) break;
        prev = cur;
        cur = nxt;
        pts.push_back(cur);
    }
    return pts;
}

// 분기에서 고를 가지 번호를 돌려주는 함수
//  cur: 분기 위치, dir_in: 들어온 방향, ends: 각 가지를 미리 본 끝점 (스켈레톤 픽셀)
using Chooser = std::function<int(cv::Point cur, cv::Point2d dir_in,
                                  const std::vector<cv::Point> &ends)>;

std::vector<cv::Point> trace(const cv::Mat &s, cv::Point start, int look, int max_len,
                             const Chooser &choose)
{
    cv::Mat vis(s.size(), CV_8U, cv::Scalar(0));
    std::vector<cv::Point> path{start};
    vis.at<uchar>(start) = 1;

    while ((int)path.size() < max_len) {
        const cv::Point cur = path.back();
        std::vector<cv::Point> cand;
        for (int i = 0; i < 8; ++i) {
            const cv::Point q(cur.x + DX[i], cur.y + DY[i]);
            if (q.x < 0 || q.y < 0 || q.x >= s.cols || q.y >= s.rows) continue;
            if (s.at<uchar>(q) && !vis.at<uchar>(q)) cand.push_back(q);
        }
        if (cand.empty()) break;

        cv::Point next = cand[0];
        if (cand.size() > 1) {
            // 각 후보를 미리 따라가 봄. 3픽셀도 못 가는 건 계단 모양의 찌꺼기
            std::vector<std::vector<cv::Point>> pv;
            std::vector<cv::Point> firsts;
            for (const auto &c : cand) {
                auto v = preview(s, vis, cur, c, look);
                if ((int)v.size() >= 3 || cand.size() == 1) { pv.push_back(std::move(v)); firsts.push_back(c); }
            }
            if (pv.empty()) {
                next = cand[0];
            } else {
                // 끝점이 가까운 후보끼리는 같은 가지 (계단 모양)
                std::vector<int> rep;                       // 가지별 대표 후보
                for (int i = 0; i < (int)pv.size(); ++i) {
                    bool same = false;
                    for (int &r : rep)
                        if (cv::norm(pv[i].back() - pv[r].back()) <= 3.0) {
                            if (pv[i].size() > pv[r].size()) r = i;
                            same = true;
                            break;
                        }
                    if (!same) rep.push_back(i);
                }
                // 들어온 방향. 시작점에서는 위쪽(로봇 전방)으로 들어왔다고 봄
                const cv::Point back = path[path.size() - std::min<size_t>(path.size(), 8)];
                cv::Point2d din(cur - back);
                if (cv::norm(din) < 1e-6) din = {0, -1};

                // 135°보다 크게 꺾이는 가지는 되돌아가는 방향 → 후보에서 뺌
                //  (시작점이 선 중간일 때의 뒤쪽 방향, 계단 모양 옆 픽셀에서 생기는 가짜 가지)
                std::vector<int> fwd;
                for (int r : rep)
                    if (std::abs(turn_angle(din, cv::Point2d(pv[r].back() - cur))) < 2.36) fwd.push_back(r);
                if (fwd.empty()) fwd = rep;

                int pick = fwd[0];
                if (fwd.size() > 1) {
                    std::vector<cv::Point> ends;
                    for (int r : fwd) ends.push_back(pv[r].back());
                    pick = fwd[choose(cur, din, ends)];

                    // 고른 가지를 미리 본 만큼 한 번에 전진.
                    // 분기점은 여러 픽셀 뭉치라서, 한 칸씩 가면 바로 옆 픽셀에서 다시 판단해 선택이 뒤집힘
                    for (const auto &q : pv[pick]) {
                        if (vis.at<uchar>(q)) break;
                        vis.at<uchar>(q) = 1;
                        path.push_back(q);
                    }
                    continue;
                }
                next = firsts[pick];
            }
        }
        vis.at<uchar>(next) = 1;
        path.push_back(next);
    }
    return path;
}

// ───────── 색 규칙 (갈림길 입구) ─────────

// 좌회전: 흰 덩어리의 왼쪽 끝에서 반 차선폭 왼쪽, 우회전: 노란 덩어리 오른쪽 끝에서 반 차선폭 오른쪽
bool color_rule_target(const cv::Mat &white, const cv::Mat &yellow, const cv::Mat &valid,
                       const Geo &g, Turn turn, cv::Point2d &target_px)
{
    if (turn == Turn::Straight) return false;
    cv::Mat m;
    cv::bitwise_and(turn == Turn::Left ? white : yellow, valid, m);
    std::vector<std::vector<cv::Point>> cs;
    cv::findContours(m, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    int best = -1;
    double area = 50;                                   // 너무 작은 건 무시
    for (int i = 0; i < (int)cs.size(); ++i) {
        const double a = cv::contourArea(cs[i]);
        if (a > area) { area = a; best = i; }
    }
    if (best < 0) return false;

    cv::Point e = cs[best][0];
    for (const auto &q : cs[best])
        if (turn == Turn::Left ? q.x < e.x : q.x > e.x) e = q;
    const double off = g.lane_w_px / 2.0;
    target_px = {e.x + (turn == Turn::Left ? -off : off), double(e.y)};
    return true;
}

} // namespace

// ───────── 공개 함수 ─────────

cv::Point2d bev_to_robot(const cv::Point2d &px, const Geo &g)
{
    const double S = g.bev_size;
    return {g.bottom_m + (S - 1 - px.y) * g.my(), (S / 2.0 - px.x) * g.mx()};
}

cv::Point2d robot_to_bev(const cv::Point2d &p, const Geo &g)
{
    const double S = g.bev_size;
    return {S / 2.0 - p.y / g.mx(), S - 1 - (p.x - g.bottom_m) / g.my()};
}

const char *source_name(Source s)
{
    switch (s) {
    case Source::Live:      return "live";
    case Source::Memory:    return "memory";
    case Source::ColorRule: return "color_rule";
    default:                return "lost";
    }
}

Output Tracker::update(const lane_skel::Result &sk, int K,
                       const cv::Mat &white, const cv::Mat &yellow, const cv::Mat &valid,
                       const Geo &g, Turn turn, const Pose2D &pose)
{
    Output out;
    K = std::max(1, K);

    // 기억을 현재 로봇 좌표로 옮기고, 이미 지나간 앞부분은 버림
    Pts mem_r;
    for (const auto &w : mem_) mem_r.push_back(to_robot(w, pose));
    size_t cut = 0;
    while (cut + 1 < mem_r.size() && mem_r[cut + 1].x < 0.02) ++cut;
    mem_r.erase(mem_r.begin(), mem_r.begin() + cut);
    const double travelled = std::hypot(pose.x - mem_pose_.x, pose.y - mem_pose_.y);

    auto px_small_to_robot = [&](const cv::Point &q) {
        return bev_to_robot(cv::Point2d(q.x * K + K / 2.0, q.y * K + K / 2.0), g);
    };

    // ── 1) 실시간 경로
    Pts live;
    if (sk.ok && !sk.skel.empty() && sk.seed.x >= 0) {
        bool turn_used = false;
        // 분기 선택: 기억한 경로가 이 분기 너머까지 있으면 그것과 가장 가까운 가지,
        // 없으면 이번 경로의 첫 분기는 turn 명령, 그다음 분기는 가장 곧은 가지
        const Chooser choose = [&](cv::Point cur, cv::Point2d din, const std::vector<cv::Point> &ends) {
            if (mem_r.size() >= 2) {
                std::vector<double> d;
                for (const auto &e : ends) d.push_back(poly_dist(px_small_to_robot(e), mem_r));
                std::vector<int> idx(d.size());
                for (size_t i = 0; i < idx.size(); ++i) idx[i] = int(i);
                std::sort(idx.begin(), idx.end(), [&](int a, int b) { return d[a] < d[b]; });
                const double b0 = d[idx[0]], b1 = d[idx[1]];
                if (b0 < 0.025 && (b1 > 2 * b0 || b1 > 0.04)) return idx[0];
            }
            const Turn t = turn_used ? Turn::Straight : turn;
            turn_used = true;
            int best = 0;
            double bv = -DBL_MAX;
            for (int i = 0; i < (int)ends.size(); ++i) {
                const double a = turn_angle(din, cv::Point2d(ends[i] - cur));
                const double v = t == Turn::Left ? a : t == Turn::Right ? -a : -std::abs(a);
                if (v > bv) { bv = v; best = i; }
            }
            return best;
        };

        const cv::Point start(sk.seed.x / K, sk.seed.y / K);
        const int look = std::max(3, int(p.lookahead_m / (g.my() * K)));
        auto px = trace(sk.skel, start, look, sk.skel.rows * sk.skel.cols, choose);

        // 사각지대 안을 오래 지나는 경로는 거기서 자름.
        // 사각지대에서는 차선이 안 보여서 영역이 벽 너머로 샐 수 있음 (막다른 길 끝 등).
        // 짧게 지나는 것(회전 중 화면 아래 모서리)은 허용
        const int blind_max = std::max(3, int(p.blind_max_m / (g.my() * K)));
        int run = 0;
        for (size_t i = 0; i < px.size(); ++i) {
            const cv::Point b(std::min(px[i].x * K, valid.cols - 1), std::min(px[i].y * K, valid.rows - 1));
            run = valid.at<uchar>(b) ? 0 : run + 1;
            if (run > blind_max) { px.resize(i + 1 - run); break; }
        }
        for (const auto &q : px) live.push_back(px_small_to_robot(q));
        live = resample(smooth(live, 2), p.step_m);
    }
    bool live_ok = live.size() >= 2 && poly_len(live) >= p.min_len_m;

    // ── 2) 기억과 비교 (갈림길 입구처럼 화면이 애매할 때 엉뚱한 경로 거르기)
    if (live_ok && mem_r.size() >= 2) {
        const double mem_far = mem_r.back().x;
        double sum = 0;
        int n = 0;
        for (const auto &q : live)
            if (q.x <= mem_far) { sum += poly_dist(q, mem_r); ++n; }
        if (n > 0 && sum / n > p.max_dev_m) {
            if (++suspicious_ < p.override_frames) live_ok = false;
        } else {
            suspicious_ = 0;
        }
    }

    if (live_ok) {
        suspicious_ = 0;
        mem_.clear();
        for (const auto &q : live) mem_.push_back(to_world(q, pose));
        mem_pose_ = pose;
        out.src = Source::Live;
        out.path = live;
        return out;
    }

    // ── 3) 기억으로 버티기
    if (mem_r.size() >= 2 && poly_len(mem_r) >= p.min_len_m * 0.5 && travelled < p.mem_travel_m) {
        out.src = Source::Memory;
        out.path = resample(mem_r, p.step_m);
        return out;
    }

    // ── 4) 색 규칙
    cv::Point2d tpx;
    if (color_rule_target(white, yellow, valid, g, turn, tpx)) {
        const cv::Point2d t = bev_to_robot(tpx, g);
        out.src = Source::ColorRule;
        out.path = resample(Pts{{0, 0}, t}, p.step_m);
        return out;
    }

    out.src = Source::Lost;
    return out;
}

void draw_path(cv::Mat &bev, const Output &o, const Geo &g)
{
    if (o.path.size() < 2) return;
    const cv::Scalar col = o.src == Source::Live     ? cv::Scalar(0, 220, 0)
                           : o.src == Source::Memory ? cv::Scalar(255, 0, 255)
                                                     : cv::Scalar(0, 140, 255);
    std::vector<cv::Point> px;
    for (const auto &q : o.path) {
        const cv::Point2d b = robot_to_bev(q, g);
        px.emplace_back(cvRound(b.x), cvRound(b.y));
    }
    cv::polylines(bev, px, false, col, 2, cv::LINE_AA);
    cv::putText(bev, source_name(o.src), {6, 18}, cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv::LINE_AA);
}

} // namespace lane_path
