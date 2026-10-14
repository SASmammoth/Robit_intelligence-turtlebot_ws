#include "tb_drive/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>



namespace tb_drive {
namespace {
double norm(const Pt &p) { return std::hypot(p.x, p.y); }
// 경로를 따라 s번째 점에서 arc 만큼 간 점
Pt point_along(const std::vector<Pt> &path, size_t s, double arc)
{
    for (size_t i = s + 1; i < path.size(); ++i)
    {
        const double L = std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
        if (L >= arc)
        {
            const double r = arc / L;
            return {path[i - 1].x + (path[i].x - path[i - 1].x) * r,
                    path[i - 1].y + (path[i].y - path[i - 1].y) * r};
        }
        arc -= L;
    }
    return path.back();
}
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
        t.p = point_along(path, s, ld); // 사각지대 때문에 경로가 ld 밖에서 시작
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

bool find_corner(const std::vector<Pt> &p, double min_turn, Pt &corner, double &turn, double min_x)
{
    const size_t n = p.size();
    const size_t C = 3;   // 방향은 6cm 현(chord)으로 잼: 점 하나하나의 방향은 스켈레톤 흔들림에 민감해서
                          // 45° 꺾임이 80°로 재지는 등 꺾인 각도가 크게 부풀었음
    const size_t win = 4; // 8cm 떨어진 두 현의 방향을 비교
    if (n < win + C + 2)
        return false;
    auto chord = [&](size_t k)
    {
        const size_t e = std::min(k + C, n - 1);
        return std::atan2(p[e].y - p[k].y, p[e].x - p[k].x);
    };
    auto step = [&](size_t k) // 점 하나 사이 방향 (코너 위치 찾기용)
    { return std::atan2(p[k + 1].y - p[k].y, p[k + 1].x - p[k].x); };

    for (size_t i = 0; i + win + C < n; ++i)
    {
        if (p[i].x < min_x)
            continue; // 사각지대(채운 구간)와 이미 지나간 부분
        const double d = std::remainder(chord(i + win) - chord(i), 2 * M_PI);
        if (std::abs(d) <= min_turn)
            continue;

        // 꺾임이 끝날 때까지 따라감: 4cm 앞 현과 방향이 6° 넘게 다르면 아직 도는 중
        size_t j = i + win;
        while (j + 2 + C < n && std::abs(std::remainder(chord(j + 2) - chord(j), 2 * M_PI)) > 0.1)
            ++j;
        turn = std::remainder(chord(j) - chord(i), 2 * M_PI);

        // 가장 급하게 꺾이는 점을 코너로 (창 가운데로 하면 급한 모서리보다 몇 cm 일찍 잡힘)
        size_t kb = i;
        double best = -1.0;
        for (size_t k = i; k < std::min(i + win + C, n - 1); ++k)
        {
            const double dk = std::abs(std::remainder(step(k + 1 < n - 1 ? k + 1 : k) - step(k), 2 * M_PI));
            if (dk > best) { best = dk; kb = k; }
        }
        corner = p[std::min(kb + 1, n - 1)];
        return true;
    }
    return false;
}

void command(const Target &t, const Params &p, double limit, double &v, double &w)
{
    v = w = 0.0;
    if (!t.ok || t.end_dist <= p.stop_dist)
        return;
    limit = std::min(limit, p.v_max);
    if (limit <= 0.0)
        return;

    const double kappa = t.kappa * p.turn_gain; // 회전 세기 배율 (lookahead는 그대로, 도는 반경만 줄임)
    double vc = limit / (1.0 + p.curv_slow * std::abs(kappa));              // 커브 감속
    vc = std::min(vc, std::sqrt(2.0 * p.decel * (t.end_dist - p.stop_dist))); // 경로 끝 감속
    vc = std::max(vc, std::min(p.v_min, limit));

    double wc = vc * kappa;
    if (std::abs(wc) > p.w_max)
    {
        // 각속도가 모자라면 곡률을 지키도록 속도를 낮춤
        wc = std::copysign(p.w_max, wc);
        vc = p.w_max / std::abs(kappa);
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
    sharp_t_ = -1e9;
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

    o.has_corner = find_corner(pts, 0.6, o.corner, o.corner_turn, p.corner_min_x); // 8cm 안에서 35° 넘게 꺾이면 코너


    double ld = std::clamp(p.ld_time * std::abs(v_now), p.ld_min, p.ld_max);
    if (p.ld_sharp > 0.0 && p.ld_sharp < ld)
    {
        // 앞쪽 경로가 로봇 방향에서 가장 많이 틀어진 각도 (6cm 현으로 재서 흔들림에 둔감하게)
        double dev = 0.0, run = 0.0;
        for (size_t i = 0; i + 3 < pts.size() && run < p.sharp_look; ++i)
        {
            if (pts[i].x <= 0.0)
                continue;
            dev = std::max(dev, std::abs(std::atan2(pts[i + 3].y - pts[i].y, pts[i + 3].x - pts[i].x)));
            run += std::hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y);
        }
        // sharp_start 아래는 줄이지 않고, sharp_turn에서 ld_sharp를 다 씀 (그 사이는 비례)
        double r = std::clamp((dev - p.sharp_start) / std::max(1e-3, p.sharp_turn - p.sharp_start), 0.0, 1.0);
        if (r >= 1.0)
            sharp_t_ = t;            // 급한 곳: 이 시각부터 sharp_hold 동안 짧게 유지
        else if (t - sharp_t_ < p.sharp_hold)
            r = 1.0;
        ld -= (ld - p.ld_sharp) * r;
    }
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

void Pivoter::reset()
{
    phase_ = Phase::Follow;
    seen_ = 0;
    seen_sign_ = 0;
}

bool Pivoter::step(const Follower::Out &o, double v_now, double w_now, double dt,
                   double v_approach, double &v, double &w, const char *&why, bool corner_ok)
{
    // 코너를 지난 주기 이동만큼 옮김 (사각지대로 들어가도 추적)
    if (phase_ == Phase::Approach)
        corner_ = shift_path({corner_}, v_now * dt, 0.0, w_now * dt)[0];

    const double tg_ang = o.tg.ok ? std::atan2(o.tg.p.y, o.tg.p.x) : 0.0;

    if (phase_ == Phase::Follow)
    {
        // 코너가 같은 방향으로 연속해서 보이는지 셈
        const bool sharp = corner_ok && o.has_corner && std::abs(o.corner_turn) > p.enter && o.corner.x < p.reach;
        const int sign = o.corner_turn > 0 ? 1 : -1;
        if (sharp && (seen_ == 0 || sign == seen_sign_))
        {
            ++seen_;
            seen_sign_ = sign;
        }
        else
            seen_ = 0;

        if (seen_ >= p.confirm && std::abs(o.corner_turn) - p.keep > 0.17) // 초과분이 10° 넘을 때만
        {
            phase_ = Phase::Approach;
            corner_ = o.corner;
            turn_ = o.corner_turn;
            target_ = std::abs(turn_) - p.keep;
            from_fallback_ = false;
            seen_ = 0;
        }
        else if (o.tg.ok && std::abs(tg_ang) > p.fallback)
        {
            phase_ = Phase::Pivot;
            turn_ = tg_ang;
            target_ = std::abs(tg_ang) - p.keep;
            from_fallback_ = true;
            done_ = 0.0;
            seen_ = 0;
        }
        else
            return false;
    }

    if (phase_ == Phase::Approach) // 코너까지 전진
    {
        if (corner_.x <= 0.01)
        {
            phase_ = Phase::Pivot;
            done_ = 0.0;
        }
        else if (corner_.x > 0.5 || std::abs(corner_.y) > 0.3)
        {
            phase_ = Phase::Follow; // 코너가 엉뚱한 곳으로 가면(추측항법 오차) 포기
            return false;
        }
        else
        {
            v = v_approach;
            const double d2 = std::max(0.0025, corner_.x * corner_.x + corner_.y * corner_.y);
            w = v * 2.0 * corner_.y / d2;
            why = "approach";
            return true;
        }
    }

    // Pivot: 초과분만 제자리에서
    done_ += std::abs(w_now) * dt;
    const bool aligned = o.tg.ok && std::abs(tg_ang) < (from_fallback_ ? p.keep : p.exit);
    if (done_ >= target_ || (from_fallback_ && aligned) || done_ > target_ + 0.5)
    {
        phase_ = Phase::Follow;
        return false;
    }
    v = 0.0;
    w = std::copysign(p.w, turn_);
    why = "pivot";
    return true;
}

} // namespace tb_drive
