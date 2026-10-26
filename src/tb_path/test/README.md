# 노트북에서 ROS 없이 알고리즘만 테스트

    cd tb_path
    g++ -O2 -std=c++17 -Iinclude src/laneSkeleton.cpp src/lanePath.cpp test/sim_fork.cpp \
        $(pkg-config --cflags --libs opencv4) -o sim_fork && ./sim_fork      # 섬 갈림길 좌/우 16가지
    g++ -O2 -std=c++17 -Iinclude src/laneSkeleton.cpp src/lanePath.cpp src/obstacles.cpp test/brick_test.cpp $(pkg-config --cflags --libs opencv4) -o brick_test && ./brick_test  # 벽돌 회피

결과 궤적 이미지(*_map.png)가 현재 폴더에 생깁니다.
    g++ -O2 -std=c++17 -Iinclude src/laneSkeleton.cpp src/lanePath.cpp test/curve_seed_test.cpp \
        $(pkg-config --cflags --libs opencv4) -o curve_seed_test && ./curve_seed_test   # 커브에서 선 너머로 시작점이 넘어가는지

## 녹화(bag)로 공사 구간 경로 다시 만들기

    pip install rosbags opencv-python numpy
    python3 tb_path/test/bag_dump_full.py ~/bags/run_1027 /tmp/rp tb_interfaces/msg      # src 폴더에서
    g++ -O2 -std=c++17 -Itb_path/include tb_path/test/bag_replay_con.cpp tb_path/src/laneSkeleton.cpp \
        tb_path/src/lanePath.cpp tb_path/src/obstacles.cpp $(pkg-config --cflags --libs opencv4) -o bag_replay_con
    cd /tmp/rp && ~/turtlebot_ws/src/bag_replay_con 6.4 8.2 1 1 1 out.png   # t0 t1 override by_lane offscreen

`con_side_test`: 화면 밖 선에 붙은 벽돌 + 흰 선만 보일 때 (`BYLANE=1 ./con_side_test 0 0.36`)

