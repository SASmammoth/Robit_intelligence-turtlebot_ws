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

double cos_blend(double a, double b, double u)
{
    u = std::clamp(u, 0.0, 1.0);
    return a + (b - a) * (1 - std::cos(M_PI * u)) / 2;
}

} // namespace

void Planner::start(const Pose2D &pose)
{
    active_ = true;
    h_ = pose.th;
    targets_.clear();
    passed_ = 0;
    has_last_ = false;
}

void Planner::reset()
{
    active_ = false;
    targets_.clear();
    passed_ = 0;
    has_last_ = false;
}

// 차선 방향: 가장 가까운 벽돌 앞면 근처(조금 앞쪽 위주)의 선 한 줄을 주성분으로
void Planner::lane_dir(const std::vector<cv::Point2d> &lines, const Pose2D &pose)
{
    const double hr = wrap(h_ - pose.th);
    const cv::Point2d t(std::cos(hr), std::sin(hr)), n(-t.y, t.x);

    double s0 = 0.10, s1 = 0.70;
    double best = 1e9;
    for (const auto &tg : targets_)
    {
        const double s = to_robot(tg.face, pose).dot(t);
        if (s > 0.05 && s < best)
            best = s;
    }
    if (best > 1e8 && passed_ > 0)
        return; // 벽돌을 다 지나는 중: 나가는 꺾임을 따라 방향이 돌면 마지막 벽돌 옆에서 일찍 꺾임
    if (best < 1e8)
    {
        s0 = best - 0.05; // 첫 벽돌은 꺾임 바로 뒤라 앞면보다 앞쪽(직선)을 봄
        s1 = best + 0.25;
    }

    std::vector<cv::Point2d> pts;
    for (const auto &q : lines)
    {
        const double s = q.dot(t);
        if (s > s0 && s < s1)
            pts.push_back(q);
    }
    if (pts.size() < 12)
        return;

    // 선이 두 줄 보이면 점이 더 많은 쪽 한 줄만 (옆 위치로 나눔)
    std::sort(pts.begin(), pts.end(), [&](const cv::Point2d &a, const cv::Point2d &b) { return a.dot(n) < b.dot(n); });
    size_t g0 = 0, bg0 = 0, bg1 = 0;
    for (size_t i = 1; i <= pts.size(); ++i)
        if (i == pts.size() || pts[i].dot(n) - pts[i - 1].dot(n) > 0.10)
        {
            if (i - g0 > bg1 - bg0)
            {
                bg0 = g0;
                bg1 = i;
            }
            g0 = i;
        }
    if (bg1 - bg0 < 12)
        return;

    cv::Point2d m(0, 0);
    for (size_t i = bg0; i < bg1; ++i)
        m += pts[i];
    m *= 1.0 / (bg1 - bg0);
    double sxx = 0, sxy = 0, syy = 0;
    for (size_t i = bg0; i < bg1; ++i)
    {
        const cv::Point2d d = pts[i] - m;
        sxx += d.x * d.x;
        sxy += d.x * d.y;
        syy += d.y * d.y;
    }
    const double tr = sxx + syy, det = sxx * syy - sxy * sxy;
    const double l1 = tr / 2 + std::sqrt(std::max(0.0, tr * tr / 4 - det)), l2 = tr - l1;
    const double N = double(bg1 - bg0);
    if (l1 / N < 0.03 * 0.03 || l1 < 12 * std::max(l2, 1e-12)) // 너무 짧거나 굽음
        return;
    double a = 0.5 * std::atan2(2 * sxy, sxx - syy); // 로봇 기준 선 방향
    if (std::abs(a) > M_PI / 2)
        a = wrap(a + M_PI);
    if (std::abs(a) > p.max_turn) // 로봇 앞으로 뻗지 않은 선 (가로선 등)
        return;
    h_ = wrap(h_ + 0.3 * wrap(a - hr));
}

void Planner::observe(const BrickObs &b, const std::vector<cv::Point2d> &lines, const Pose2D &pose)
{
    const double hr = wrap(h_ - pose.th);
    const cv::Point2d t(std::cos(hr), std::sin(hr)), n(-t.y, t.x);
    const cv::Point2d mid = (b.l + b.r) * 0.5;
    // 박스 아랫변 줄 = 벽돌에서 가장 가까운 모서리. 로봇이 차선에 비스듬하면 앞면 한쪽 끝이 그 모서리
    // (차선이 로봇 기준 왼쪽으로 틀어져 있으면 왼쪽 끝이 더 가까움). 가운데를 쓰면 앞뒤로 수 cm 틀림
    cv::Point2d corner = mid;
    bool rough = false;
    if (std::abs(hr) > 0.05)
    {
        // 가까운 끝이 화면 밖에서 잘렸으면 앞뒤 위치가 부정확 → 가운데로 대충 (이미 아는 벽돌이면 앞뒤는 안 고침)
        if ((hr > 0 && b.l_clip) || (hr < 0 && b.r_clip))
            rough = true;
        else
            corner = hr > 0 ? b.l : b.r;
    }
    const double sf = corner.dot(t);
    double la = b.l.dot(n), lb = b.r.dot(n);
    if (la < lb)
        std::swap(la, lb);
    const double gc = (p.lane_w - p.brick_w) / 2; // 틈 가운데까지 (벽돌 안쪽 끝에서)
    const double att = p.brick_w + gc;            // 벽돌이 붙은 선에서 틈 가운데까지

    // 벽돌 앞면 줄 근처의 선 (옆 위치 l)
    bool lc = false, rc = false, lf = false, rf = false;
    double cl = 1e9, cr = -1e9, fl = 1e9, fr = -1e9;
    for (const auto &q : lines)
    {
        if (std::abs(q.dot(t) - sf) > 0.05)
            continue;
        const double l = q.dot(n);
        if (l > la - 0.05 && l < la + p.contact) { lc = true; cl = std::min(cl, l); }
        else if (l >= la + p.contact) { lf = true; fl = std::min(fl, l); }
        if (l < lb + 0.05 && l > lb - p.contact) { rc = true; cr = std::max(cr, l); }
        else if (l <= lb - p.contact) { rf = true; fr = std::max(fr, l); }
    }

    int side = 0, q = 3;
    double lp = 0;
    if (lc && rc)
        side = (cl - la) <= (lb - cr) ? 1 : -1, q = 0;
    else if (lc)
        side = 1, q = 0;
    else if (rc)
        side = -1, q = 0;
    else if (rf && !lf)
        side = 1, q = 1;
    else if (lf && !rf)
        side = -1, q = 1;
    else if (b.l_clip != b.r_clip)
        side = b.l_clip ? 1 : -1, q = 2;

    if (side == 0)
    {
        // 근거가 없으면: 바로 앞 벽돌의 반대쪽, 그것도 없으면 로봇 기준 어느 쪽인지
        const Target *prev = nullptr;
        double ps = -1e9;
        for (const auto &tg : targets_)
        {
            const double s = to_robot(tg.face, pose).dot(t);
            if (s < sf - p.merge_dist && s > ps)
                ps = s, prev = &tg;
        }
        side = prev ? -prev->side() : (mid.dot(n) >= 0 ? 1 : -1);
        q = 3;
    }

    // 지날 점의 옆 위치 (붙은 선 기준과 반대쪽 선 기준이 둘 다 있으면 평균)
    if (side > 0)
    {
        double sum = 0;
        int k = 0;
        if (lc) sum += cl - att, ++k;
        if (rf) sum += fr + gc, ++k;
        if (k == 0) sum = b.l_clip ? lb - gc : la - att, k = 1; // 왼쪽이 잘렸으면 박스 안쪽 끝 기준 (여유 쪽으로 치우침)
        lp = sum / k;
    }
    else
    {
        double sum = 0;
        int k = 0;
        if (rc) sum += cr + att, ++k;
        if (lf) sum += fl - gc, ++k;
        if (k == 0) sum = b.r_clip ? la + gc : lb + att, k = 1;
        lp = sum / k;
    }

    const cv::Point2d face = to_world(corner, pose);
    const cv::Point2d pass = to_world(t * sf + n * lp, pose);
    const cv::Point2d tw(std::cos(h_), std::sin(h_));
    const int idx = side > 0 ? 0 : 1;
    static const double qw[4] = {4.0, 3.0, 1.0, 0.3};

    Target *hit = nullptr;
    double hd = p.merge_dist;
    for (auto &tg : targets_)
    {
        const double d = std::abs((face - tg.face).dot(tw));
        if (d < hd)
            hd = d, hit = &tg;
    }
    if (!hit)
    {
        Target tg;
        tg.face = face;
        tg.face_rough = rough;
        tg.pass[idx] = pass;
        tg.n[idx] = 1;
        tg.vote[idx] = qw[q];
        tg.quality[idx] = q;
        targets_.push_back(tg);
        return;
    }
    if (!rough)
    {
        hit->face += (face - hit->face) * (hit->face_rough ? 1.0 : 0.5);
        hit->face_rough = false;
    }
    else if (hit->face_rough)
        hit->face += (face - hit->face) * 0.5;
    hit->miss = -1; // 이번 프레임에 봄 (update에서 0으로)
    hit->vote[0] *= 0.9;
    hit->vote[1] *= 0.9;
    hit->vote[idx] += qw[q] * (rough ? 0.5 : 1.0);
    if (rough && !hit->face_rough && hit->n[idx] > 0)
        return; // 정확한 값이 이미 있으면 잘린 관측으로 옆 위치도 안 고침
    if (hit->n[idx] == 0)
        hit->pass[idx] = pass, hit->quality[idx] = q;
    else
    {
        // 가까워질수록 정확하므로 새 관측을 많이 반영
        const double a = q < hit->quality[idx] ? 0.7 : q == hit->quality[idx] ? 0.5 : 0.15;
        hit->pass[idx] += (pass - hit->pass[idx]) * a;
        hit->quality[idx] = std::min(hit->quality[idx], q);
    }
    ++hit->n[idx];
}

void Planner::update(const std::vector<cv::Point2d> &lines, const std::vector<BrickObs> &bricks, bool fresh,
                     const Pose2D &pose, double view_x0, double view_x1)
{
    if (!active_)
        return;
    lane_dir(lines, pose);
    const double hr = wrap(h_ - pose.th);
    const cv::Point2d t(std::cos(hr), std::sin(hr)), tw(std::cos(h_), std::sin(h_));

    if (fresh)
    {
        for (const auto &b : bricks)
            observe(b, lines, pose);
        // 탐지 범위 안인데 안 보이는 벽돌은 놓친 횟수를 셈 (잘못 탐지한 것 정리)
        for (auto &tg : targets_)
        {
            if (tg.miss < 0)
            {
                tg.miss = 0;
                continue;
            }
            const double s = to_robot(tg.face, pose).dot(t);
            if (s > view_x0 && s < view_x1)
                ++tg.miss;
        }
        targets_.erase(std::remove_if(targets_.begin(), targets_.end(),
                                      [](const Target &tg) { return tg.miss > 8 && tg.miss > tg.n[0] + tg.n[1]; }),
                       targets_.end());
    }

    // 차선 방향이 바뀌면서 앞뒤로 가까워진 것은 하나로 (표가 많은 쪽을 남김)
    for (size_t i = 0; i < targets_.size(); ++i)
        for (size_t j = i + 1; j < targets_.size();)
        {
            Target &a = targets_[i], &b = targets_[j];
            if (std::abs((a.face - b.face).dot(tw)) < p.merge_dist)
            {
                const double wa = a.vote[0] + a.vote[1], wb = b.vote[0] + b.vote[1];
                Target keep = wa >= wb ? a : b;
                const Target &other = wa >= wb ? b : a;
                for (int k = 0; k < 2; ++k)
                {
                    if (keep.n[k] == 0 && other.n[k] > 0)
                        keep.pass[k] = other.pass[k], keep.quality[k] = other.quality[k];
                    keep.vote[k] += other.vote[k];
                    keep.n[k] += other.n[k];
                }
                keep.passed = a.passed || b.passed;
                a = keep;
                targets_.erase(targets_.begin() + j);
            }
            else
                ++j;
        }

    // 지나간 벽돌: 뒷면이 로봇 뒤 15cm를 지나면 셈, keep_behind 지나면 지움
    for (auto it = targets_.begin(); it != targets_.end();)
    {
        const double s = to_robot(it->face, pose).dot(t);
        if (s + p.brick_d < -p.rear_clear && !it->passed && it->n[0] + it->n[1] >= 3) // 몇 번 안 본 건 안 셈
        {
            it->passed = true;
            ++passed_;
            has_last_ = true;
            last_back_ = it->face + tw * p.brick_d;
        }
        if (s < -p.keep_behind)
            it = targets_.erase(it);
        else
            ++it;
    }
}

bool Planner::path(const Pose2D &pose, std::vector<cv::Point2d> &out) const
{
    out.clear();
    if (!active_ || targets_.empty())
        return false;
    const double hr = wrap(h_ - pose.th);
    const cv::Point2d t(std::cos(hr), std::sin(hr)), n(-t.y, t.x);

    struct Node { double s, l; int side; };
    std::vector<Node> nodes;
    for (const auto &tg : targets_)
    {
        const int side = tg.side();
        const cv::Point2d pr = to_robot(tg.pass_pt(), pose);
        nodes.push_back({to_robot(tg.face, pose).dot(t), pr.dot(n), side});
    }
    std::sort(nodes.begin(), nodes.end(), [](const Node &a, const Node &b) { return a.s < b.s; });

    const double pass_off = p.brick_w + (p.lane_w - p.brick_w) / 2 - p.lane_w / 2;
    const Node &f = nodes.front(), &e = nodes.back();
    const double f_s = f.s - p.first_lead;                                      // 첫 벽돌: 이 앞에서 옆 위치 도착
    const double a0 = std::min(std::max(0.0, f_s - p.trans_len), f_s - 0.10); // 넘어가기 시작
    const double e0 = e.s + p.brick_d + 0.05, e1 = e0 + 0.35;
    const double lc = e.l + e.side * pass_off; // 마지막 벽돌 뒤: 차선 가운데로

    auto lat = [&](double s)
    {
        if (s < f_s)
            return cos_blend(0.0, f.l, (s - a0) / std::max(1e-6, f_s - a0));
        for (size_t k = 0; k < nodes.size(); ++k)
        {
            const double hold_end = nodes[k].s + p.brick_d +
                                    (k + 1 < nodes.size() ? std::min(p.hold_after, 0.5 * (nodes[k + 1].s - nodes[k].s - p.brick_d)) : 0.0);
            if (s <= hold_end)
                return nodes[k].l;
            if (k + 1 < nodes.size() && s < nodes[k + 1].s)
            {
                const double g0 = hold_end, g1 = nodes[k + 1].s;
                return cos_blend(nodes[k].l, nodes[k + 1].l, (s - g0) / std::max(1e-6, g1 - g0));
            }
        }
        if (s < e0)
            return e.l;
        return cos_blend(e.l, lc, (s - e0) / (e1 - e0));
    };
    for (double s = -p.behind; s <= p.ahead + 1e-9; s += p.step)
        out.push_back(t * s + n * lat(s));
    return true;
}

bool Planner::finished(const Pose2D &pose) const
{
    if (!active_)
        return false;
    if (p.count > 0 && passed_ >= p.count)
        return true;
    if (!has_last_)
        return false;
    const double hr = wrap(h_ - pose.th);
    const cv::Point2d t(std::cos(hr), std::sin(hr));
    for (const auto &tg : targets_)
        if (!tg.passed) // 아직 안 지난 벽돌
            return false;
    return to_robot(last_back_, pose).dot(t) < -p.exit_after;
}

} // namespace construction
