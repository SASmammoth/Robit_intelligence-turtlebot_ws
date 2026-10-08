# tb_line → tb_path : BEV 차선 라벨 영상과 거리 환산값
std_msgs/Header header

# mono8 BEV 영상. 0 = 차선 없음, 1 = 흰선, 2 = 노란선, 255 = 사각지대(카메라가 못 보는 곳)
sensor_msgs/Image label

float32 m_per_px_x        # 가로 1px당 거리 [m]
float32 m_per_px_y        # 세로 1px당 거리 [m]
float32 bottom_offset_m   # 로봇 바퀴 축 ~ BEV 맨 아랫줄 거리 [m]
int32   lane_w_px         # BEV에서 두 차선 안쪽 간격 [px]

# 원본 카메라 영상 픽셀 → BEV 픽셀 변환 행렬 (3x3, 행 우선)
float64[9] homography
