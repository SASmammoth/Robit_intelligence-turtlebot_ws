// 전처리(letterbox) ↔ 후처리(decode) 좌표가 맞는지, NMS 동작 확인
#include "tb_signs/yolo.hpp"
#include <cstdio>
int fails = 0;
#define CHECK(c, m) do { if (!(c)) { std::printf("FAIL: %s\n", m); ++fails; } } while (0)

int main()
{
    const int W = 640, H = 640, NC = 7, N = 8400;
    cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::rectangle(frame, cv::Rect(400, 100, 60, 40), cv::Scalar(0, 0, 255), cv::FILLED);   // 빨강 (BGR)

    std::vector<float> in(3 * W * H);
    const yolo::Letterbox lb = yolo::preprocess(frame, W, H, in.data());
    std::printf("letterbox scale=%.3f pad=(%.0f,%.0f)\n", lb.scale, lb.pad_x, lb.pad_y);
    CHECK(lb.pad_x == 0 && lb.pad_y == 80, "640x480 → 640x640 pad 80 위아래");

    // 빨간 사각형 중심이 입력 텐서 R 채널(0번)에 1.0, B 채널(2번)에 0
    const int ix = int(430 * lb.scale + lb.pad_x), iy = int(120 * lb.scale + lb.pad_y);
    CHECK(std::abs(in[0 * W * H + iy * W + ix] - 1.0f) < 1e-3, "R 채널 = 1");
    CHECK(std::abs(in[2 * W * H + iy * W + ix]) < 1e-3, "B 채널 = 0 (BGR→RGB)");
    CHECK(std::abs(in[1 * W * H + 10 * W + 10] - 114 / 255.f) < 1e-3, "여백 = 114");

    // 가짜 출력: 같은 위치에 'brick'(6) 두 개 (겹침) + 'left'(1) 하나
    auto make = [&](bool chan_first) {
        std::vector<float> o((4 + NC) * N, 0.f);
        auto set = [&](int i, float cx, float cy, float w, float h, int c, float s) {
            const float v[4] = {cx, cy, w, h};
            for (int k = 0; k < 4; ++k) (chan_first ? o[k * N + i] : o[i * (4 + NC) + k]) = v[k];
            (chan_first ? o[(4 + c) * N + i] : o[i * (4 + NC) + 4 + c]) = s;
        };
        set(10, 430 + lb.pad_x, 120 + lb.pad_y, 60, 40, 6, 0.9f);   // 입력 좌표 (scale 1)
        set(11, 432 + lb.pad_x, 121 + lb.pad_y, 58, 40, 6, 0.8f);   // 중복
        set(500, 100 + lb.pad_x, 300 + lb.pad_y, 50, 50, 1, 0.7f);
        set(600, 200 + lb.pad_x, 200 + lb.pad_y, 50, 50, 2, 0.3f);  // conf 미만
        return o;
    };
    for (bool cf : {true, false}) {
        const auto o = make(cf);
        auto d = yolo::decode(o.data(), cf ? 4 + NC : N, cf ? N : 4 + NC, NC, 0.5f, lb, frame.size());
        CHECK(d.size() == 3, "conf 통과 3개");
        d = yolo::nms(d, 0.45f);
        CHECK(d.size() == 2, "NMS 후 2개");
        CHECK(d[0].cls == 6 && std::abs(d[0].box.x - 400) < 1 && std::abs(d[0].box.y - 100) < 1 &&
              std::abs(d[0].box.width - 60) < 1, "brick 박스 원본 좌표 복원");
        CHECK(d[1].cls == 1, "left 유지");
        std::printf("[%s] brick box=(%.0f,%.0f,%.0f,%.0f) score=%.2f\n", cf ? "[4+nc,N]" : "[N,4+nc]",
                    d[0].box.x, d[0].box.y, d[0].box.width, d[0].box.height, d[0].score);
    }
    std::printf("%s\n", fails ? "FAILED" : "ALL OK");
    return fails;
}
