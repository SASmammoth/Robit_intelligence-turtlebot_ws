#include "tb_drive/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

namespace tb_drive {
namespace {
double norm(const Pt &p) { return std::hypot(p.x, p.y); }
} // namespace

Target find_target(const std::vector<Pt> &path, double ld)
{
    Target t;

    size_t s = 0;
    while (s < path.size() && path[s].x <= 0.0)
        ++s;
    if (s >= path.size())
        return t;

    // 남은 길이: 로봇 → 첫 점 → ... → 마지막 점
    double len = norm(path[s]);
    for (size_t i = s + 1; i < path.size(); ++i)
        len += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    t.end_dist = len;

    if (norm(path[s]) >= ld)
    {
        t.p = path[s]; // 사각지대 때문에 경로가 ld 밖에서 시작
    }
    else
    {
        bool found = false;
        for (size_t i = s + 1; i < path.size() && !found; ++i)
        {
            // 선분 a→b와 반지름 ld 원의 교점 (바깥으로 나가는 쪽)
            const Pt a = path[i - 1], b = path[i];
            const double dx = b.x - a.x, dy = b.y - a.y;
            const double A = dx * dx + dy * dy;
            if (A < 1e-12)
                continue;
            const double B = 2.0 * (a.x * dx + a.y * dy);
            const double C = a.x * a.x + a.y * a.y - ld * ld;
            const double D = B * B - 4.0 * A * C;
            if (D < 0.0)
                continue;
            const double u = (-B + std::sqrt(D)) / (2.0 * A);
            if (u >= 0.0 && u <= 1.0)
            {
                t.p = {a.x + u * dx, a.y + u * dy};
                found = true;
            }
        }
        if (!found)
        {
            t.p = path.back();
            t.at_end = true;
        }
    }

    const double d = norm(t.p);
    if (d < 1e-3)
        return t;
    t.ld = d;
    t.kappa = 2.0 * t.p.y / (d * d);
    t.ok = true;
    return t;
}

void command(const Target &t, const Params &p, double limit, double &v, double &w)
{
    v = w = 0.0;
    if (!t.ok || t.end_dist <= p.stop_dist)
        return;
    limit = std::min(limit, p.v_max);
    if (limit <= 0.0)
        return;

    double vc = limit / (1.0 + p.curv_slow * std::abs(t.kappa));            // 커브 감속
    vc = std::min(vc, std::sqrt(2.0 * p.decel * (t.end_dist - p.stop_dist))); // 경로 끝 감속
    vc = std::max(vc, std::min(p.v_min, limit));

    double wc = vc * t.kappa;
    if (std::abs(wc) > p.w_max)
    {
        // 각속도가 모자라면 곡률을 지키도록 속도를 낮춤
        wc = std::copysign(p.w_max, wc);
        vc = p.w_max / std::abs(t.kappa);
    }
    v = vc;
    w = wc;
}

void Limiter::step(double v_des, double w_des, double dt, const Params &p)
{
    const bool slowing = std::abs(v_des) < std::abs(v) || v_des * v < 0.0;
    const double a = (slowing ? p.decel : p.accel) * dt;
    v += std::clamp(v_des - v, -a, a);
    const double aw = p.w_accel * dt;
    w += std::clamp(w_des - w, -aw, aw);
}

void MotionHistory::push(double t, double v, double w)
{
    h_.push_back({t, v, w});
    // 1초보다 오래된 기록 삭제 (구간 시작값은 남김)
    while (h_.size() > 2 && h_[1].t <= t - 1.0)
        h_.pop_front();
}

void MotionHistory::delta(double t0, double t1, double &dx, double &dy, double &dth) const
{
    dx = dy = dth = 0.0;
    for (size_t i = 0; i < h_.size(); ++i)
    {
        const double a = std::max(h_[i].t, t0);
        const double b = std::min(i + 1 < h_.size() ? h_[i + 1].t : t1, t1);
        if (b <= a)
            continue;
        const double dt = b - a, v = h_[i].v, w = h_[i].w;
        dx += v * dt * std::cos(dth + 0.5 * w * dt);
        dy += v * dt * std::sin(dth + 0.5 * w * dt);
        dth += w * dt;
    }
}

std::vector<Pt> shift_path(const std::vector<Pt> &in, double dx, double dy, double dth)
{
    const double c = std::cos(dth), s = std::sin(dth);
    std::vector<Pt> out;
    out.reserve(in.size());
    for (const auto &p : in)
    {
        const double px = p.x - dx, py = p.y - dy;
        out.push_back({c * px + s * py, -s * px + c * py});
    }
    return out;
}

void Follower::reset()
{
    hist_.clear();
    cur_.clear();
    good_.clear();
    cur_stamp_ = good_stamp_ = -1.0;
}

void Follower::set_path(std::vector<Pt> pts, double stamp)
{
    if (!pts.empty())
    {
        good_ = pts;
        good_stamp_ = stamp;
    }
    cur_ = std::move(pts);
    cur_stamp_ = stamp;
}

Follower::Out Follower::follow(double t, double v_now, double limit) const
{
    Out o;
    const bool cur_fresh = cur_stamp_ >= 0.0 && t - cur_stamp_ <= path_timeout;
    const bool good_alive = good_stamp_ >= 0.0 && t - good_stamp_ <= path_timeout + hold_time;

    const std::vector<Pt> *src = nullptr;
    double stamp = 0.0;
    bool shift = latency_comp;
    if (cur_fresh && !cur_.empty())
    {
        src = &cur_;
        stamp = cur_stamp_;
    }
    else if (good_alive)
    {
        // 끝이 사각지대로 들어갔거나 몇 프레임 놓침 → 마지막 경로를 움직인 만큼 당겨서 사용
        src = &good_;
        stamp = good_stamp_;
        shift = true;
        o.held = true;
    }
    else
    {
        o.why = cur_fresh ? "path_empty" : "no_path";
        return o;
    }

    std::vector<Pt> pts = *src;
    if (shift && t > stamp)
    {
        double dx, dy, dth;
        hist_.delta(stamp, t, dx, dy, dth);
        pts = shift_path(pts, dx, dy, dth);
    }

    const double ld = std::clamp(p.ld_time * std::abs(v_now), p.ld_min, p.ld_max);
    o.tg = find_target(pts, ld);
    if (!o.tg.ok)
    {
        o.why = "path_behind";
        return o;
    }
    command(o.tg, p, limit, o.v, o.w);
    if (o.v == 0.0)
        o.why = "path_end";
    return o;
}

} // namespace tb_drive
