#pragma once
#include "uavloc/localizer.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace demo {
inline cv::Mat scene(unsigned seed = 12345) {
    cv::RNG rng(seed);
    cv::Mat gray(1024, 1024, CV_8U);
    rng.fill(gray, cv::RNG::UNIFORM, 45, 100);
    cv::GaussianBlur(gray, gray, {5, 5}, 1.2);
    for (int i = 0; i < 140; ++i) {
        int x = rng.uniform(30, 950), y = rng.uniform(30, 950);
        cv::rectangle(gray, {x, y, rng.uniform(15, 65), rng.uniform(15, 65)}, cv::Scalar(rng.uniform(110, 240)), -1);
        cv::line(gray, {x + 3, y + 3}, {x + 13, y + 13}, cv::Scalar(25), 2);
    }
    for (int i = 0; i < 12; ++i) {
        cv::Point a(rng.uniform(0, 1024), rng.uniform(0, 1024));
        cv::Point b(rng.uniform(0, 1024), rng.uniform(0, 1024));
        cv::line(gray, a, b, cv::Scalar(25), 13, cv::LINE_AA);
        cv::line(gray, a, b, cv::Scalar(180), 3, cv::LINE_AA);
    }
    for (int i = 0; i < 180; ++i)
        cv::circle(gray, {rng.uniform(10, 1010), rng.uniform(10, 1010)}, rng.uniform(3, 12), cv::Scalar(rng.uniform(20, 240)), -1, cv::LINE_AA);
    return gray;
}
inline cv::Mat view(const cv::Mat& map, cv::Mat& query_to_map) {
    std::vector<cv::Point2f> from{{210, 245}, {700, 210}, {740, 710}, {175, 745}};
    std::vector<cv::Point2f> to{{0, 0}, {479, 0}, {479, 479}, {0, 479}};
    const auto h = cv::getPerspectiveTransform(from, to);
    query_to_map = h.inv();
    cv::Mat query;
    cv::warpPerspective(map, query, h, {480, 480});
    return query;
}
inline uavloc::GeoPoint truth(const uavloc::Tile& tile, const cv::Mat& query_to_map) {
    std::vector<cv::Point2f> center{{239.5f, 239.5f}};
    cv::perspectiveTransform(center, center, query_to_map);
    return uavloc::pixelToGeo(tile, center.front(), {1024, 1024});
}
} // namespace demo
