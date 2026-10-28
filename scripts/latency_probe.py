#!/usr/bin/env python3
# 파이프라인 단계별 지연 측정 (Nano에서 로봇 실행 중에 다른 터미널로)
#   python3 ~/turtlebot_ws/scripts/latency_probe.py            # 2초마다 표 출력
#   python3 ~/turtlebot_ws/scripts/latency_probe.py --period 5
#
# 모든 토픽의 header.stamp는 camera_node가 찍은 값이 그대로 이어진다.
# 그래서 "받은 시각 - stamp" = 카메라 publish부터 그 단계까지 걸린 시간.
#   /lane/mask        카메라 → lineDetect 끝
#   /lane/path        카메라 → path 끝
#   /signs/detections 카메라 → YOLO 끝 (compressed 인코딩·디코딩 포함)
#   /drive/state age= 카메라 → 모터 명령 계산 (drive 타이머 대기 포함)
# 카메라 안쪽 지연(노출 → USB → 디코딩)은 camera_node 로그 "[latency]" 줄을 볼 것.
#
# 반드시 Nano에서 실행 (노트북은 시계가 달라서 값이 틀림). 이 스크립트 자체도 1ms 정도 더함.
import argparse
import re
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path
from std_msgs.msg import String
from tb_interfaces.msg import LaneMask, DetectionArray


def stats(xs):
    if not xs:
        return '      -'
    xs = sorted(xs)
    p95 = xs[min(len(xs) - 1, int(len(xs) * 0.95))]
    return f'{statistics.fmean(xs):6.1f} {xs[len(xs) // 2]:6.1f} {p95:6.1f} {xs[-1]:6.1f}'


class Probe(Node):
    def __init__(self, period):
        super().__init__('latency_probe')
        self.period = period
        self.age = {k: [] for k in ('mask', 'path', 'signs', 'drive')}
        self.mask_arrival = {}     # stamp(ns) → 받은 시각(ns)
        self.stage_path = []       # 같은 프레임: path 도착 - mask 도착
        self.n_mask = self.n_path = 0
        qos = qos_profile_sensor_data  # best effort: 발행 쪽에 부담 안 줌
        self.create_subscription(LaneMask, '/lane/mask', self.on_mask, qos)
        self.create_subscription(Path, '/lane/path', self.on_path, qos)
        self.create_subscription(DetectionArray, '/signs/detections', self.on_signs, qos)
        self.create_subscription(String, '/drive/state', self.on_drive, 10)
        self.create_timer(period, self.report)
        self.age_re = re.compile(r'\bage=(-?\d+)')

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    @staticmethod
    def stamp_ns(h):
        return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec

    def on_mask(self, m):
        t = self.now_ns()
        s = self.stamp_ns(m.header)
        self.age['mask'].append((t - s) / 1e6)
        self.mask_arrival[s] = t
        self.n_mask += 1
        if len(self.mask_arrival) > 200:
            for k in sorted(self.mask_arrival)[:100]:
                del self.mask_arrival[k]

    def on_path(self, m):
        t = self.now_ns()
        s = self.stamp_ns(m.header)
        if s == 0:
            return
        self.age['path'].append((t - s) / 1e6)
        self.n_path += 1
        if s in self.mask_arrival:
            self.stage_path.append((t - self.mask_arrival[s]) / 1e6)

    def on_signs(self, m):
        s = self.stamp_ns(m.header)
        if s:
            self.age['signs'].append((self.now_ns() - s) / 1e6)

    def on_drive(self, m):
        g = self.age_re.search(m.data)
        if g and int(g.group(1)) >= 0:
            self.age['drive'].append(float(g.group(1)))

    def report(self):
        print(f'\n--- {self.period:.0f}s ---  [ms]           mean    p50    p95    max   Hz')
        rows = [('카메라→line 끝 (/lane/mask)', 'mask'),
                ('카메라→path 끝 (/lane/path)', 'path'),
                ('카메라→YOLO 끝 (/signs/detections)', 'signs'),
                ('카메라→모터 명령 (/drive/state age)', 'drive')]
        for name, k in rows:
            xs = self.age[k]
            print(f'{name:<34} {stats(xs)} {len(xs) / self.period:5.1f}')
        print(f'{"  path 단계만 (path 도착-mask 도착)":<34} {stats(self.stage_path)}')
        if self.n_mask:
            print(f'  mask {self.n_mask}개 중 path로 나온 것 {self.n_path}개'
                  + ('  ← path가 프레임을 건너뜀' if self.n_path < self.n_mask * 0.95 else ''))
        if not any(self.age.values()):
            print('  (받은 메시지 없음: 로봇 실행 중인지, ROS_DOMAIN_ID 확인)')
        for v in self.age.values():
            v.clear()
        self.stage_path.clear()
        self.n_mask = self.n_path = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--period', type=float, default=2.0)
    a = ap.parse_args()
    rclpy.init()
    try:
        rclpy.spin(Probe(a.period))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
