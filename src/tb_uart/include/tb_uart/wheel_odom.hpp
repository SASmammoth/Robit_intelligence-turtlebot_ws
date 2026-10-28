#ifndef TB_UART__WHEEL_ODOM_HPP_
#define TB_UART__WHEEL_ODOM_HPP_

// 다이나믹셀 velocity 피드백(원시값) → 로봇 속도 (v, w) → 위치 (x, y, th) 적분
// ROS 의존 없음 (테스트에서 그대로 씀)

#include <algorithm>
#include <cmath>

namespace tb {

class WheelOdom {
public:
  // 하드웨어 값 (tb_drive drive.uart.* 와 같아야 함)
  double wheel_radius = 0.0475;   // [m]
  double wheel_sep = 0.1933;      // 두 바퀴 같은 방향으로 구를 때 간격 [m]
  double pivot_sep = 0.1933;      // 제자리 회전 때 실측 간격 [m] (미끄러져 더 작게 나옴)
  double rpm_per_unit = 0.229;    // MX-64 Protocol 2.0 velocity 단위
  double max_gap = 0.1;           // 피드백 사이가 이보다 길면 이만큼만 적분 [s]

  double x = 0.0, y = 0.0, th = 0.0;   // [m, m, rad] 시작 위치 기준
  double v = 0.0, w = 0.0;             // 마지막 피드백 속도 [m/s, rad/s]
  double dist = 0.0;                   // 누적 이동 거리 (|v| 적분) [m]

  // 원시값 → 바퀴 선속도 [m/s]
  double raw_to_mps(double raw) const { return raw * rpm_per_unit * 2.0 * M_PI * wheel_radius / 60.0; }

  // 바퀴 선속도 → (v, w). tb_drive publish_uart()의 역변환
  //   drive:  sep = ws                                       (|v| >= |w|*ws/2, 두 바퀴 같은 방향)
  //           sep = ps + (ws-ps) * |v| / (|w|*ws/2)           (안쪽 바퀴가 거꾸로 돌 때)
  //           l = v - w*sep/2,  r = v + w*sep/2
  //   역:     v = (l+r)/2,  d = (r-l)/2 = w*sep/2
  //           |v| >= |d| 이면 sep = ws
  //           아니면 sep = ps / (1 - (ws-ps)*|v| / (|d|*ws))   (|v| = |d|에서 ws로 이어짐)
  void wheels_to_twist(double l, double r, double &v_out, double &w_out) const
  {
    v_out = 0.5 * (l + r);
    const double d = 0.5 * (r - l);
    double sep = wheel_sep;
    if (std::abs(d) > 1e-9 && std::abs(v_out) < std::abs(d) && wheel_sep > 1e-6) {
      const double k = 1.0 - (wheel_sep - pivot_sep) * std::abs(v_out) / (std::abs(d) * wheel_sep);
      if (k > 1e-6) sep = pivot_sep / k;
    }
    w_out = (sep > 1e-6) ? 2.0 * d / sep : 0.0;
  }

  // 피드백 한 번. l_raw, r_raw: 전진이면 둘 다 양수가 되도록 방향을 맞춘 원시값, t: 받은 시각 [s]
  void update(double l_raw, double r_raw, double t)
  {
    double vn, wn;
    wheels_to_twist(raw_to_mps(l_raw), raw_to_mps(r_raw), vn, wn);
    if (last_t_ >= 0.0) {
      const double dt = std::clamp(t - last_t_, 0.0, max_gap);
      // 사다리꼴: 지난 피드백과 이번 피드백의 평균 속도로 dt 동안 움직였다고 봄
      const double va = 0.5 * (v + vn), wa = 0.5 * (w + wn);
      const double mid = th + 0.5 * wa * dt;
      x += va * dt * std::cos(mid);
      y += va * dt * std::sin(mid);
      th = std::remainder(th + wa * dt, 2.0 * M_PI);
      dist += std::abs(va) * dt;
    }
    v = vn;
    w = wn;
    last_t_ = t;
  }

  void reset_pose() { x = y = th = dist = 0.0; }
  bool started() const { return last_t_ >= 0.0; }

private:
  double last_t_ = -1.0;
};

}  // namespace tb

#endif  // TB_UART__WHEEL_ODOM_HPP_
