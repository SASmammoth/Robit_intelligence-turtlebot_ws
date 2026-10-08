// 합성 트랙으로 lane_skeleton 동작 확인
#include "tb_path/laneSkeleton.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cstdio>
#include <string>
#include <vector>

using PL = std::vector<cv::Point>;
constexpr int T = 10;   // 차선 두께 (BEV px)

static void dashed(cv::Mat &m, cv::Point a, cv::Point b, int on, int off)
{
    const cv::Point2f d = cv::Point2f(b - a) * (1.0f / cv::norm(b - a));
    const float L = cv::norm(b - a);
    for (float t = 0; t < L; t += on + off)
        cv::line(m, cv::Point2f(a) + d * t, cv::Point2f(a) + d * std::min(L, t + on), 255, T);
}

struct Scene {
    std::string name;
    cv::Mat white, yellow, valid;
    bool parking = false;
};

static cv::Mat bev_valid_320()
{
    // 실제 노드와 같은 BEV 설정: 640x480 → 320x320, top_y 45, top_w 30, bot_w 100
    const float w = 640, h = 480, cx = w / 2;
    const float ty = h * 0.45f, by = h - 1, tw = w * 0.15f, bw = w * 0.5f;
    std::vector<cv::Point2f> src = {{cx - tw, ty}, {cx + tw, ty}, {cx + bw, by}, {cx - bw, by}};
    const float S = 320, m = S * 0.25f;
    std::vector<cv::Point2f> dst = {{m, 0}, {S - m, 0}, {S - m, S}, {m, S}};
    const cv::Mat M = cv::getPerspectiveTransform(src, dst);
    return lane_skel::make_valid_mask(M, cv::Size(640, 480), cv::Size(320, 320));
}

static cv::Mat out_view(const Scene &s, const lane_skel::Result &r, int scale)
{
    cv::Mat v(s.white.size(), CV_8UC3, cv::Scalar(70, 70, 70));
    v.setTo(cv::Scalar(235, 235, 235), s.white);
    v.setTo(cv::Scalar(0, 210, 255), s.yellow);
    lane_skel::draw_debug(v, s.valid, r, scale);
    v.setTo(cv::Scalar(235, 235, 235), s.white);
    v.setTo(cv::Scalar(0, 210, 255), s.yellow);
    std::vector<cv::Point> pts;
    cv::findNonZero(r.skel, pts);
    for (auto &p : pts) cv::circle(v, p * scale, 2, cv::Scalar(170, 230, 60), -1);
    for (auto &j : r.junctions) cv::circle(v, j, 7, cv::Scalar(0, 120, 255), -1);
    for (auto &e : r.ends) cv::circle(v, e, 6, cv::Scalar(60, 60, 255), 2);
    cv::putText(v, s.name, {8, 22}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    return v;
}

int main()
{
    std::vector<Scene> scenes;
    const cv::Mat full640(640, 640, CV_8U, cv::Scalar(255));

    // 1. 직선: 흰선에 구멍 + 위쪽은 흰선 없음 (한쪽만 보임), 사각지대 있음
    {
        Scene s{"1 hole + one side", cv::Mat::zeros(320, 320, CV_8U), cv::Mat::zeros(320, 320, CV_8U), bev_valid_320()};
        cv::line(s.yellow, {75, 0}, {75, 320}, 255, T);
        cv::line(s.white, {245, 320}, {245, 200}, 255, T);
        cv::line(s.white, {245, 172}, {245, 100}, 255, T);
        scenes.push_back(s);
    }
    // 2. 왼쪽 곡선, 사각지대 있음
    {
        Scene s{"2 curve", cv::Mat::zeros(320, 320, CV_8U), cv::Mat::zeros(320, 320, CV_8U), bev_valid_320()};
        cv::ellipse(s.yellow, {-40, 320}, {115, 260}, 0, -90, 0, 255, T);
        cv::ellipse(s.white, {-40, 320}, {285, 400}, 0, -90, 0, 255, T);
        scenes.push_back(s);
    }
    // 3. 섬 갈림길 (그림 1)
    {
        Scene s{"3 island fork", cv::Mat::zeros(640, 640, CV_8U), cv::Mat::zeros(640, 640, CV_8U), full640};
        cv::polylines(s.yellow, PL{{234, 640}, {234, 470}, {60, 470}, {60, 170}, {234, 170}, {234, 0}}, false, 255, T);
        cv::polylines(s.white,  PL{{406, 640}, {406, 470}, {580, 470}, {580, 170}, {406, 170}, {406, 0}}, false, 255, T);
        cv::polylines(s.white,  PL{{320, 250}, {260, 250}, {260, 390}, {320, 390}}, false, 255, T);
        cv::polylines(s.yellow, PL{{320, 250}, {380, 250}, {380, 390}, {320, 390}}, false, 255, T);
        scenes.push_back(s);
    }
    // 4, 5. ㅓ 구간 (그림 2): 주행 모드 / 주차 모드
    for (bool park : {false, true}) {
        Scene s{park ? "5 park mode" : "4 drive mode", cv::Mat::zeros(640, 640, CV_8U), cv::Mat::zeros(640, 640, CV_8U), full640, park};
        cv::polylines(s.yellow, PL{{300, 0}, {300, 260}, {200, 260}, {200, 120}, {40, 120}, {40, 560}, {190, 560}, {190, 420}, {300, 420}, {300, 640}}, false, 255, T);
        dashed(s.yellow, {48, 260}, {192, 260}, 22, 12);
        dashed(s.yellow, {48, 420}, {182, 420}, 22, 12);
        cv::line(s.white, {460, 0}, {460, 640}, 255, T);
        scenes.push_back(s);
    }

    lane_skel::Params p;   // 기본값: lane_w 160, seal 30, prune 20
    cv::Mat row1, row2;
    std::vector<cv::Mat> big;
    for (auto &s : scenes) {
        cv::Mat yw = s.yellow.clone();
        if (s.parking) lane_skel::remove_dashes(yw, 40);
        cv::Mat lane = s.white | yw;
        const lane_skel::Result r = lane_skel::build(lane, s.valid, p);
        std::printf("%-18s ok=%d ends=%zu junctions=%zu seed=(%d,%d)\n", s.name.c_str(), r.ok,
                    r.ends.size(), r.junctions.size(), r.seed.x, r.seed.y);
        cv::Mat v = out_view(s, r, p.scale);
        cv::imwrite("out_" + s.name.substr(0, 1) + ".png", v);
    }
    return 0;
}
