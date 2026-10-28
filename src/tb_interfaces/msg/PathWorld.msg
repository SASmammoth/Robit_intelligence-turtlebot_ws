# tb_path → tb_gui: 경로를 만들 때 쓴 것과 기억하고 있는 것 (GUI "주행+" 탭 표시용, 제어에는 안 씀)
# 좌표: 로봇 기준 base_link [m] (x 앞, y 왼쪽, 바퀴 축 원점). header.stamp = 카메라 stamp
std_msgs/Header header

# 이번 프레임 BEV에서 본 선 (줄마다 선 조각의 가운데 점, 성기게)
geometry_msgs/Point[] white
geometry_msgs/Point[] yellow
geometry_msgs/Point[] red             # 정지선 모드일 때만
geometry_msgs/Point[] ext             # 공사 구간: 벽돌에 가려 끊긴 선을 이은 부분

geometry_msgs/Polygon[] bricks        # 이번 프레임 벽돌 벽 (스켈레톤이 피한 빨간 영역 윤곽)
geometry_msgs/Polygon[] held          # 기억 중인 지난 벽돌 (화면 아래로 빠진 것, 이동 거리만큼 뒤로 옮김)
geometry_msgs/Polygon[] bridges       # 같은 쪽 벽돌 사이를 이어 막은 곳 (path.con_bridge_mm)
geometry_msgs/Polygon[] keep          # 경로를 떼어 놓은 벽돌 (path.con_keep > 0일 때만)

geometry_msgs/Point[] memory          # 기억 경로 (화면이 애매할 때 쓰는 직전 경로)
geometry_msgs/Point[] trail           # 사각지대 채움용 지나온 경로

float32 stop_x                        # 기억한 정지선까지 거리 [m] (-1 = 없음)
float32 stop_cut_x                    # 정지선 때문에 경로를 자른 x [m] (-1 = 안 자름)
bool construction
uint8 turn                            # 지금 쓰는 /mission/turn (0 직진, 1 좌, 2 우)
