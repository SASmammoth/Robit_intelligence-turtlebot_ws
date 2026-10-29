# 노트북에서 ROS 없이 알고리즘만 테스트

    cd tb_path
    g++ -O2 -std=c++17 -Iinclude src/laneSkeleton.cpp src/lanePath.cpp test/sim_fork.cpp \
        $(pkg-config --cflags --libs opencv4) -o sim_fork && ./sim_fork      # 섬 갈림길 좌/우 16가지
    g++ -O2 -std=c++17 -Iinclude src/laneSkeleton.cpp src/lanePath.cpp src/obstacles.cpp test/brick_test.cpp $(pkg-config --cflags --libs opencv4) -o brick_test && ./brick_test  # 벽돌 회피

결과 궤적 이미지(*_map.png)가 현재 폴더에 생깁니다.
    g++ -O2 -std=c++17 -Iinclude src/laneSkeleton.cpp src/lanePath.cpp test/curve_seed_test.cpp \
        $(pkg-config --cflags --libs opencv4) -o curve_seed_test && ./curve_seed_test   # 커브에서 선 너머로 시작점이 넘어가는지
    g++ -O2 -std=c++17 -Iinclude src/laneSkeleton.cpp src/lanePath.cpp test/cross_line_test.cpp \
        $(pkg-config --cflags --libs opencv4) -o cross_line_test && ./cross_line_test   # 공사 구간: 안쪽이 막혔을 때 흰선 너머 경로를 바로 따라가는지

## 녹화(bag)로 공사 구간 경로 다시 만들기

    pip install rosbags opencv-python numpy
    python3 tb_path/test/bag_dump_full.py ~/bags/run_1027 /tmp/rp tb_interfaces/msg      # src 폴더에서
    g++ -O2 -std=c++17 -Itb_path/include tb_path/test/bag_replay_con.cpp tb_path/src/laneSkeleton.cpp \
        tb_path/src/lanePath.cpp tb_path/src/obstacles.cpp $(pkg-config --cflags --libs opencv4) -o bag_replay_con
    cd /tmp/rp && ~/turtlebot_ws/src/bag_replay_con 6.4 8.2 1 1 1 out.png   # t0 t1 override by_lane offscreen

`con_side_test`: 화면 밖 선에 붙은 벽돌 + 흰 선만 보일 때 (`BYLANE=1 ./con_side_test 0 0.36`)


`bridge_test`: 같은 쪽 벽돌 잇기 (`path.con_bridge_mm`). r-r 배치에서 앞 벽돌이 화면 아래로 빠졌을 때 1·2번 사이로 안 들어가는지, r-l은 그대로인지

    cd src/tb_path && g++ -O2 -std=c++17 -Iinclude test/bridge_test.cpp src/laneSkeleton.cpp src/lanePath.cpp \
        src/obstacles.cpp $(pkg-config --cflags --libs opencv4) -o bridge_test && ./bridge_test

`hug_test`: 벽돌 옆에서 차선 쪽으로 붙기 (`path.con_line_hug`). 선 쪽으로 옮기는지, 선에 keep보다 안 붙는지, 벽돌·선이 없으면 그대로인지

    cd src/tb_path && g++ -O2 -std=c++17 -Iinclude test/hug_test.cpp src/laneSkeleton.cpp src/lanePath.cpp \
        src/obstacles.cpp $(pkg-config --cflags --libs opencv4) -o hug_test && ./hug_test

`yellow_left_test`: 공사 구간 출구 (`path.con_yellow_left`). 노란선만 보이다 왼쪽으로 꺾이고 오른쪽 흰선은 화면 밖, 앞에 출구 바깥 흰 가로선이 보일 때
바깥(직진)으로 새지 않고 왼쪽으로 도는지. 녹화 1029 09:57 22s 재현. 켬/끔 각 18가지 (모서리 반지름 0/12/24cm × 출발 위치 × 속도)

    cd src/tb_path && g++ -O2 -std=c++17 -Iinclude src/laneSkeleton.cpp src/lanePath.cpp test/yellow_left_test.cpp \
        $(pkg-config --cflags --libs opencv4) -o yellow_left_test && ./yellow_left_test
