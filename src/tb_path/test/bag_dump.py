#!/usr/bin/env python3
# ros2 bag(.mcap) → BEV 마스크 PNG + frames.txt (bag_replay.cpp 입력). ROS 없이 동작
#   pip install rosbags opencv-python numpy
#   python3 tb_path/test/bag_dump.py ~/bags/con_run4 /tmp/replay   (src 폴더에서)
# 녹화: ros2 bag record -o ~/bags/xxx /lane/mask /signs/detections /lane/path /drive/state
import sys, math, json
from pathlib import Path
import numpy as np, cv2
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

bag, out = Path(sys.argv[1]), Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)
msgdir = Path(__file__).resolve().parents[2] / 'tb_interfaces' / 'msg'
ts = get_typestore(Stores.ROS2_JAZZY)
add = {}
for n in ['Detection', 'DetectionArray', 'LaneMask', 'DriveCmd']:
    p = msgdir / (n + '.msg')
    if p.exists():
        add.update(get_types_from_msg(p.read_text(), 'tb_interfaces/msg/' + n))
ts.register(add)

ev = []
with AnyReader([bag], default_typestore=ts) as r:
    t0 = None
    for c, t, raw in r.messages():
        m = r.deserialize(raw, c.msgtype)
        t0 = t if t0 is None else t0
        tt = (t - t0) / 1e9
        if c.topic == '/drive/state':
            ev.append(('state', tt, m.data))
        elif c.topic == '/signs/detections':
            ev.append(('det', tt, [(d.label, float(d.score), d.x, d.y, d.w, d.h) for d in m.detections]))
        elif c.topic == '/lane/path':
            ev.append(('path', tt, [(p.pose.position.x, p.pose.position.y) for p in m.poses]))
        elif c.topic == '/lane/mask':
            k = sum(1 for e in ev if e[0] == 'mask')
            img = np.frombuffer(m.label.data, np.uint8).reshape(m.label.height, m.label.width)
            cv2.imwrite(str(out / ('mask_%03d.png' % k)), img)
            ev.append(('mask', tt, k, m.m_per_px_x, m.m_per_px_y, m.bottom_offset_m, m.lane_w_px, list(m.homography)))
json.dump(ev, open(out / 'events.json', 'w'))

def last(kind, t):
    b = None
    for e in ev:
        if e[0] == kind and e[1] <= t + 1e-3:
            b = e
    return b

# 이동량: /drive/state 의 v, w 적분 (odom이 없을 때)
pose, lt = [0.0, 0.0, 0.0], 0.0
with open(out / 'frames.txt', 'w') as f:
    for e in ev:
        if e[0] != 'mask':
            continue
        t = e[1]
        st = last('state', t)
        v = w = 0.0
        if st:
            kv = dict(p.split('=') for p in st[2].split()[1:] if '=' in p)
            v, w = float(kv.get('v', 0)), float(kv.get('w', 0))
        dt, lt = t - lt, t
        pose[0] += v * dt * math.cos(pose[2] + 0.5 * w * dt)
        pose[1] += v * dt * math.sin(pose[2] + 0.5 * w * dt)
        pose[2] += w * dt
        dd = last('det', t)
        bx = [b for b in (dd[2] if dd else []) if b[0] == 'brick']
        f.write('%d %.4f %.6f %.6f %.6f %.6f %.6f %.6f %d %s %d %s\n' % (
            e[2], t, pose[0], pose[1], pose[2], e[3], e[4], e[5], e[6], ' '.join('%.9g' % x for x in e[7]),
            len(bx), ' '.join('%d %d %d %d' % tuple(b[2:6]) for b in bx)))
print('frames:', sum(1 for e in ev if e[0] == 'mask'), '->', out / 'frames.txt')
