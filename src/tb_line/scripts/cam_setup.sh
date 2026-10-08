#!/bin/bash
# 카메라 노출 고정 (빛번짐 방지)
DEV=${1:-/dev/video0}

v4l2-ctl -d "$DEV" -c auto_exposure=1               # 수동 노출 (먼저 해야 아래가 먹힘)
v4l2-ctl -d "$DEV" -c exposure_time_absolute=85
v4l2-ctl -d "$DEV" -c exposure_dynamic_framerate=0  # 노출 때문에 fps 떨어지는 것 방지
v4l2-ctl -d "$DEV" -c gain=0
v4l2-ctl -d "$DEV" -c backlight_compensation=0
v4l2-ctl -d "$DEV" -c white_balance_automatic=0
v4l2-ctl -d "$DEV" -c white_balance_temperature=6200

echo "[cam_setup] $DEV 설정 완료"