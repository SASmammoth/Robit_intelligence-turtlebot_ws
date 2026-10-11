# 노트북에서 ROS 없이 알고리즘만 테스트

    cd tb_path
    g++ -O2 -std=c++17 -Iinclude src/laneSkeleton.cpp src/lanePath.cpp test/sim_fork.cpp \
        $(pkg-config --cflags --libs opencv4) -o sim_fork && ./sim_fork      # 섬 갈림길 좌/우 16가지
    g++ -O2 -std=c++17 -Iinclude src/laneSkeleton.cpp src/lanePath.cpp src/obstacles.cpp test/brick_test.cpp $(pkg-config --cflags --libs opencv4) -o brick_test && ./brick_test  # 벽돌 회피

결과 궤적 이미지(*_map.png)가 현재 폴더에 생깁니다.
    g++ -O2 -std=c++17 -Iinclude src/laneSkeleton.cpp src/lanePath.cpp test/curve_seed_test.cpp \
        $(pkg-config --cflags --libs opencv4) -o curve_seed_test && ./curve_seed_test   # 커브에서 선 너머로 시작점이 넘어가는지
