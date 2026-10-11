# tb_brain → tb_drive 주행 명령
# tb_brain은 원하는 동작이 유지되는 동안 10Hz 이상으로 계속 발행한다.
# tb_drive는 drive.cmd_timeout_ms 동안 명령이 안 오면 마지막 명령을 버린다.

uint8 FOLLOW = 0   # /lane/path 추종 (speed_limit 적용)
uint8 STOP   = 1   # 정지 (decel 제한으로 부드럽게)
uint8 TWIST  = 2   # 경로 무시, v·w 그대로 (후진, 제자리 회전 등)

std_msgs/Header header
uint8   mode
float32 speed_limit   # FOLLOW: 속도 상한 [m/s], 0 이하면 drive.v_max 사용
float32 v             # TWIST 전용 [m/s], 음수면 후진
float32 w             # TWIST 전용 [rad/s], 양수면 좌회전
