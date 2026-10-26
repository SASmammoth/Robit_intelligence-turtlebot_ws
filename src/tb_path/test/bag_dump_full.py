#!/usr/bin/env python3
# ros2 bag(.mcap) → BEV 마스크 PNG, 카메라 JPG, events.json, frames.txt (bag_replay_con.cpp 입력). ROS 없이 동작
#   pip install rosbags opencv-python numpy
#   python3 tb_path/test/bag_dump_full.py ~/bags/run_1027 /tmp/rp tb_interfaces/msg   (src 폴더에서)
# 녹화: ros2 bag record -s mcap -o ~/bags/xxx /lane/mask /signs/detections /lane/path /lane/path_source #       /lane/construction /drive/state /drive/target /cmd_vel /mission/turn /mission/construction /mission/pause #       /drive/cmd /brain/run /brain/inject /camera/image_raw/compressed
# frames.txt 자세는 /cmd_vel 적분 (path_node와 같음), 벽돌은 마스크 stamp와 300ms 안의 검출
import sys, json, math, bisect
from pathlib import Path
import numpy as np, cv2
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
bag, out, msgdir = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
out.mkdir(parents=True, exist_ok=True); (out/'cam').mkdir(exist_ok=True)
ts = get_typestore(Stores.ROS2_JAZZY); add = {}
for n in ['Detection', 'DetectionArray', 'LaneMask', 'DriveCmd']:
    add.update(get_types_from_msg((msgdir / (n + '.msg')).read_text(), 'tb_interfaces/msg/' + n))
ts.register(add)
ev = []; k = 0; kc = 0
with AnyReader([bag], default_typestore=ts) as r:
    t0 = None
    for c, t, raw in r.messages():
        m = r.deserialize(raw, c.msgtype)
        t0 = t if t0 is None else t0
        tt = (t - t0) / 1e9
        tp = c.topic
        if tp == '/drive/state': ev.append(['state', tt, m.data])
        elif tp == '/signs/detections':
            st = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
            ev.append(['det', tt, st, [(d.label, float(d.score), d.x, d.y, d.w, d.h) for d in m.detections]])
        elif tp == '/lane/path': ev.append(['path', tt, [(p.pose.position.x, p.pose.position.y) for p in m.poses]])
        elif tp == '/lane/path_source': ev.append(['src', tt, m.data])
        elif tp == '/lane/construction': ev.append(['con', tt, bool(m.data)])
        elif tp == '/cmd_vel': ev.append(['cmd', tt, m.linear.x, m.angular.z])
        elif tp == '/drive/target': ev.append(['tgt', tt, m.point.x, m.point.y])
        elif tp in ('/mission/turn','/mission/construction','/mission/pause','/brain/run'): ev.append(['mis', tt, tp, str(m.data)])
        elif tp == '/drive/cmd': pass
        elif tp == '/lane/mask':
            img = np.frombuffer(m.label.data, np.uint8).reshape(m.label.height, m.label.width)
            cv2.imwrite(str(out / ('mask_%03d.png' % k)), img)
            st = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
            ev.append(['mask', tt, k, m.m_per_px_x, m.m_per_px_y, m.bottom_offset_m, m.lane_w_px, list(m.homography), st]); k += 1
        elif tp == '/camera/image_raw/compressed':
            img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
            cv2.imwrite(str(out/'cam'/('%04d_%07.3f.jpg' % (kc, tt))), img); kc += 1
json.dump(ev, open(out / 'events.json', 'w'))
print('masks', k, 'cams', kc, 'dur', ev[-1][1])
D = str(out)

# ───── frames.txt ─────
import bisect

cmds = [e for e in ev if e[0] == 'cmd']; dets = [e for e in ev if e[0] == 'det']; paths = [e for e in ev if e[0] == 'path']
cons = [e for e in ev if e[0] == 'con']
# 자세: /cmd_vel 적분 (path_node와 같게)
pose = [0.0, 0.0, 0.0]; poses = []; lt = None; lv = lw = 0.0
ci = 0
masks = [e for e in ev if e[0] == 'mask']
def last(lst, t):
    ts = [e[1] for e in lst]; i = bisect.bisect_right(ts, t) - 1
    return lst[i] if i >= 0 else None
with open(D + '/frames.txt', 'w') as f:
    for m in masks:
        t = m[1]
        # integrate cmd_vel up to t
        while ci < len(cmds) and cmds[ci][1] <= t:
            c = cmds[ci]
            if lt is not None:
                dt = min(c[1] - lt, 0.2)
                pose[0] += lv * dt * math.cos(pose[2] + 0.5 * lw * dt); pose[1] += lv * dt * math.sin(pose[2] + 0.5 * lw * dt); pose[2] += lw * dt
            lt, lv, lw = c[1], c[2], c[3]; ci += 1
        d = last(dets, t + 0.0)
        bx = []
        if d and abs(d[2] - m[8]) * 1000 <= 300:
            bx = [b for b in d[3] if b[0] == 'brick']
        con = last(cons, t); con = 1 if (con and con[2]) else 0
        f.write('%d %.4f %.6f %.6f %.6f %.6f %.6f %.6f %d %s %d %d %s\n' % (
            m[2], t, pose[0], pose[1], pose[2], m[3], m[4], m[5], m[6], ' '.join('%.9g' % x for x in m[7]), con,
            len(bx), ' '.join('%d %d %d %d' % tuple(b[2:6]) for b in bx)))
print('ok')
