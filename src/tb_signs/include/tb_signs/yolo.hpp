#pragma once
// YOLO(ultralytics v8/11 detect) 전처리·후처리. TensorRT와 무관한 부분만 모아서 단독 테스트 가능
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <numeric>
#include <vector>

namespace yolo {

struct Letterbox {
    float scale = 1.f;   // 원본 → 입력 배율
    float pad_x = 0.f;   // 입력 영상의 왼쪽 여백
    float pad_y = 0.f;   // 위쪽 여백
};

struct Det {
    int cls;
    float score;
    cv::Rect2f box;      // 원본 영상 좌표
};

// BGR 원본 → 비율 유지 리사이즈 + 회색(114) 여백 → RGB float [0,1] CHW 로 dst에 기록
// dst: in_w * in_h * 3 개 float
inline Letterbox preprocess(const cv::Mat &bgr, int in_w, int in_h, float *dst)
{
    Letterbox lb;
    lb.scale = std::min(in_w / float(bgr.cols), in_h / float(bgr.rows));
    const int nw = int(std::round(bgr.cols * lb.scale));
    const int nh = int(std::round(bgr.rows * lb.scale));
    lb.pad_x = (in_w - nw) / 2.f;
    lb.pad_y = (in_h - nh) / 2.f;

    cv::Mat resized, padded, rgb, f;
    cv::resize(bgr, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    const int top = int(std::round(lb.pad_y - 0.1f)), left = int(std::round(lb.pad_x - 0.1f));
    cv::copyMakeBorder(resized, padded, top, in_h - nh - top, left, in_w - nw - left,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(f, CV_32F, 1.0 / 255.0);

    // HWC → CHW: dst 메모리를 가리키는 3개 평면에 바로 split
    std::vector<cv::Mat> planes = {
        cv::Mat(in_h, in_w, CV_32F, dst),
        cv::Mat(in_h, in_w, CV_32F, dst + in_w * in_h),
        cv::Mat(in_h, in_w, CV_32F, dst + 2 * in_w * in_h)};
    cv::split(f, planes);
    lb.pad_x = float(left);
    lb.pad_y = float(top);
    return lb;
}

// 출력 텐서 해석. ultralytics detect 출력은 [1, 4+nc, N] (cx, cy, w, h, class scores...)
// 일부 export는 [1, N, 4+nc] 로 전치되어 있어서 둘 다 처리
inline std::vector<Det> decode(const float *out, int d1, int d2, int nc, float conf,
                               const Letterbox &lb, const cv::Size &img)
{
    const bool chan_first = (d1 == 4 + nc);          // [4+nc, N]
    const int N = chan_first ? d2 : d1;
    auto at = [&](int k, int i) { return chan_first ? out[k * N + i] : out[i * (4 + nc) + k]; };

    std::vector<Det> dets;
    for (int i = 0; i < N; ++i) {
        int best = 0;
        float bs = at(4, i);
        for (int c = 1; c < nc; ++c) {
            const float s = at(4 + c, i);
            if (s > bs) { bs = s; best = c; }
        }
        if (bs < conf) continue;
        const float cx = (at(0, i) - lb.pad_x) / lb.scale;
        const float cy = (at(1, i) - lb.pad_y) / lb.scale;
        const float w = at(2, i) / lb.scale, h = at(3, i) / lb.scale;
        cv::Rect2f r(cx - w / 2, cy - h / 2, w, h);
        r &= cv::Rect2f(0, 0, float(img.width), float(img.height));
        if (r.width < 1 || r.height < 1) continue;
        dets.push_back({best, bs, r});
    }
    return dets;
}

inline float iou(const cv::Rect2f &a, const cv::Rect2f &b)
{
    const float inter = (a & b).area();
    const float uni = a.area() + b.area() - inter;
    return uni > 0 ? inter / uni : 0.f;
}

// 클래스별 NMS
inline std::vector<Det> nms(std::vector<Det> dets, float iou_thres, int max_det = 50)
{
    std::sort(dets.begin(), dets.end(), [](const Det &a, const Det &b) { return a.score > b.score; });
    std::vector<Det> keep;
    std::vector<char> removed(dets.size(), 0);
    for (size_t i = 0; i < dets.size() && (int)keep.size() < max_det; ++i) {
        if (removed[i]) continue;
        keep.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j)
            if (!removed[j] && dets[j].cls == dets[i].cls && iou(dets[i].box, dets[j].box) > iou_thres)
                removed[j] = 1;
    }
    return keep;
}

} // namespace yolo
