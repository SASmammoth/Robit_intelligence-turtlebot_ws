#include "tb_path/construction.hpp"

#include <algorithm>
#include <cmath>

namespace construction {
namespace {

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

double wrap(double a) { return std::remainder(a, 2 * M_PI); }

} // namespace

void Planner::start(double lane_heading, const cv::Point2d &lane_point)
{
    active_ = true;
    h0_ = h_ = lane_heading;
    o_ = lane_point;
    bricks_.clear();
    has_last_ = false;
}

void Planner::reset()
{
    active_ = false;
    bricks_.clear();
    has_last_ = false;
}

double Planner::s_of(const cv::Point2d &w) const
{
    return (w.x - o_.x) * std::cos(h_) + (w.y - o_.y) * std::sin(h_);
}

double Planner::l_of(const cv::Point2d &w) const
{
    return -(w.x - o_.x) * std::sin(h_) + (w.y - o_.y) * std::cos(h_);
}

cv::Point2d Planner::lane_to_world(double s, double l) const
{
    const double c = std::cos(h_), sn = std::sin(h_);
    return {o_.x + c * s - sn * l, o_.y + sn * s + c * l};
}

void Planner::lane_hint(const std::vector<cv::Point2d> &cl, const Pose2D &pose)
{
    if (!active_ || !bricks_.empty())
        return;
    // 로봇 앞 0.15~0.6m의 중심선으로 방향과 옆 위치를 조금씩 따라감
    std::vector<cv::Point2d> pts;
    for (const auto &q : cl)
        if (q.x > 0.15 && q.x < 0.6)
            pts.push_back(to_world(q, pose));
    if (pts.size() < 5)
        return;
    const cv::Point2d d = pts.back() - pts.front();
    if (cv::norm(d) < 0.15)
        return;
    const double h = std::atan2(d.y, d.x);
    h_ = wrap(h_ + 0.3 * wrap(h - h_));
    h0_ = h_;
    cv::Point2d mid(0, 0);
    for (const auto &q : pts)
        mid += q;
    mid *= 1.0 / pts.size();
    const double dl = l_of(mid);
    o_ += cv::Point2d(-std::sin(h_), std::cos(h_)) * (0.3 * dl);
}

void Planner::observe(const cv::Point2d &front, const Pose2D &pose)
{
    if (!active_)
        return;
    const cv::Point2d t(std::cos(h_), std::sin(h_));
    const cv::Point2d c = to_world(front, pose) + t * (p.brick_d / 2); // 앞면 → 중심
    const int side = l_of(c) >= 0 ? 1 : -1;
    const double s = s_of(c);

    Brick *hit = nullptr;
    for (auto &b : bricks_)
        if (b.side == side && std::abs(s_of(b.c) - s) < p.merge_dist)
            hit = &b;
    if (hit) {
        const int n = std::min(hit->n, 20); // 오래 본 벽돌도 새 관측이 조금은 반영되게
        hit->c = (hit->c * n + c) * (1.0 / (n + 1));
        ++hit->n;
    } else {
        bricks_.push_back({c, side, 1});
    }
    refit();

    // 지나간 벽돌 정리 (가장 앞 벽돌 위치는 last_c_로 기억해 끝 판단에 씀)
    const double sr = s_of({pose.x, pose.y});
    for (const auto &b : bricks_)
        if (!has_last_ || s_of(b.c) > s_of(last_c_)) {
            last_c_ = b.c;
            has_last_ = true;
        }
    bricks_.erase(std::remove_if(bricks_.begin(), bricks_.end(),
                                 [&](const Brick &b) { return s_of(b.c) < sr - p.keep_behind; }),
                  bricks_.end());
}

// 벽돌은 차선에 붙어 있으므로 벽돌 중심에서 차선 중심선 위의 점을 거꾸로 구함
void Planner::refit()
{
    if (bricks_.empty())
        return;
    // 차선 기준이 바뀌면서 가까워진 같은 쪽 벽돌은 하나로 합침
    for (size_t i = 0; i < bricks_.size(); ++i)
        for (size_t j = i + 1; j < bricks_.size();) {
            Brick &a = bricks_[i], &b = bricks_[j];
            if (a.side == b.side && cv::norm(a.c - b.c) < p.merge_dist) {
                a.c = (a.c * a.n + b.c * b.n) * (1.0 / (a.n + b.n));
                a.n += b.n;
                bricks_.erase(bricks_.begin() + j);
            } else
                ++j;
        }
    const double inset = p.lane_w / 2 - p.brick_w / 2;
    for (int it = 0; it < 3; ++it) {
        const cv::Point2d n(-std::sin(h_), std::cos(h_));
        std::vector<cv::Point2d> P;
        std::vector<double> W;
        double wsum = 0;
        cv::Point2d m(0, 0);
        for (const auto &b : bricks_) {
            P.push_back(b.c - n * (b.side * inset));
            W.push_back(std::min(b.n, 10));
            m += P.back() * W.back();
            wsum += W.back();
        }
        m *= 1.0 / wsum;

        double smin = 1e9, smax = -1e9;
        for (const auto &q : P) {
            smin = std::min(smin, s_of(q));
            smax = std::max(smax, s_of(q));
        }
        if (P.size() >= 2 && smax - smin > 0.2) {
            // 가중 주성분으로 중심선 방향
            double sxx = 0, sxy = 0, syy = 0;
            for (size_t i = 0; i < P.size(); ++i) {
                const cv::Point2d d = P[i] - m;
                sxx += W[i] * d.x * d.x;
                sxy += W[i] * d.x * d.y;
                syy += W[i] * d.y * d.y;
            }
            double h = 0.5 * std::atan2(2 * sxy, sxx - syy);
            if (std::abs(wrap(h - h_)) > M_PI / 2)
                h = wrap(h + M_PI);
            if (std::abs(wrap(h - h0_)) < p.max_turn)
                h_ = h;
        }
        o_ = m;
    }
}

double Planner::offset(double s) const
{
    // 노드: (중심 s, 목표 옆 위치). 첫 벽돌 앞·마지막 벽돌 뒤에는 차선 중심(0)인 가상 노드
    std::vector<std::pair<double, double>> nodes;
    for (const auto &b : bricks_)
        nodes.push_back({s_of(b.c), -b.side * p.pass_off});
    std::sort(nodes.begin(), nodes.end());
    nodes.insert(nodes.begin(), {nodes.front().first - p.spacing, 0.0});
    nodes.push_back({nodes.back().first + p.spacing, 0.0});

    if (s <= nodes.front().first)
        return 0.0;
    for (size_t k = 0; k + 1 < nodes.size(); ++k) {
        const auto &a = nodes[k], &b = nodes[k + 1];
        if (s > b.first)
            continue;
        const double m = 0.5 * (a.first + b.first);
        const double L = std::max(0.05, std::min(p.trans_len, b.first - a.first - p.brick_d));
        if (s < m - L / 2)
            return a.second;
        if (s > m + L / 2)
            return b.second;
        const double u = (s - (m - L / 2)) / L;
        return a.second + (b.second - a.second) * (1 - std::cos(M_PI * u)) / 2;
    }
    return 0.0;
}

bool Planner::path(const Pose2D &pose, std::vector<cv::Point2d> &out) const
{
    out.clear();
    if (!active_ || bricks_.empty())
        return false;
    const double sr = s_of({pose.x, pose.y});
    for (double s = sr - p.behind; s <= sr + p.ahead + 1e-9; s += p.step)
        out.push_back(to_robot(lane_to_world(s, offset(s)), pose));
    return true;
}

bool Planner::finished(const Pose2D &pose) const
{
    return active_ && has_last_ && s_of({pose.x, pose.y}) > s_of(last_c_) + p.exit_after;
}

} // namespace construction
