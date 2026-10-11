// 우커브 진입: 차선이 오른쪽으로 휘어 BEV 하단 중앙(x=160)이 노란선 왼쪽(차선 밖)에 놓인 상황
#include "tb_path/laneSkeleton.hpp"
#include "tb_path/lanePath.hpp"
#include <opencv2/imgproc.hpp>
#include <cstdio>
int main() {
    const int S = 320;
    cv::Mat white(S, S, CV_8U, cv::Scalar(0)), yellow = white.clone(), valid(S, S, CV_8U, cv::Scalar(255));
    // 노란선(차선 왼쪽 경계): 아래 x=190 → 위로 갈수록 오른쪽으로 휨, 흰선은 그보다 172px 오른쪽(화면 밖으로 나감)
    std::vector<cv::Point> yl, wl;
    for (int y = S - 1; y >= 0; y -= 4) {
        const double t = double(S - 1 - y) / S;
        const int xc = 190 + int(170 * t * t * 2);
        yl.push_back({xc, y}); wl.push_back({xc + 172, y});
    }
    cv::polylines(yellow, yl, false, 255, 18);
    cv::polylines(white, wl, false, 255, 18);
    lane_skel::Params sp; sp.lane_w = 172;
    const cv::Mat lane = white | yellow;

    auto side = [&](const lane_skel::Result &r) {
        if (!r.ok) return "실패";
        return r.seed.x > 190 ? "차선 안 (노란선 오른쪽)" : "차선 밖 (노란선 왼쪽)";
    };
    cv::Point none(-1, -1), lane_center(190 + 86, S - 4);   // 기억 경로가 예측한 점: 차선 중앙
    const auto a = lane_skel::build(lane, valid, sp, &none);
    const auto b = lane_skel::build(lane, valid, sp, &lane_center, &lane_center);
    std::printf("하단 중앙에서 탐색 : seed=(%d,%d) %s\n", a.seed.x, a.seed.y, side(a));
    std::printf("기억 경로 예측 시드: seed=(%d,%d) %s\n", b.seed.x, b.seed.y, side(b));
}
