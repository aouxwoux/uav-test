#include "uavloc/localizer.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace uavloc {
namespace {
constexpr double pi = 3.14159265358979323846;
double radians(double x) { return x * pi / 180.0; }
double score(const Result& r) { return r.inliers * r.inlier_ratio / (1.0 + r.rmse_px); }
bool finite(cv::Point2f p) { return std::isfinite(p.x) && std::isfinite(p.y); }
}

double distanceMeters(GeoPoint a, GeoPoint b) {
    double x = std::pow(std::sin(radians(b.latitude - a.latitude) / 2), 2) +
               std::cos(radians(a.latitude)) * std::cos(radians(b.latitude)) *
               std::pow(std::sin(radians(b.longitude - a.longitude) / 2), 2);
    return 6371008.8 * 2 * std::asin(std::sqrt(std::clamp(x, 0.0, 1.0)));
}

GeoPoint pixelToGeo(const Tile& tile, cv::Point2d pixel, cv::Size image) {
    if (image.width <= 0 || image.height <= 0) throw std::runtime_error("Invalid image size for georeferencing");
    // Bounds describe outer image edges; x/y refer to pixel centres.
    return {tile.north - (pixel.y + 0.5) / image.height * (tile.north - tile.south),
            tile.west + (pixel.x + 0.5) / image.width * (tile.east - tile.west)};
}

bool intersectsPrior(const Tile& tile, const Query& query) {
    // Conservative spherical distance lower bound. Longitude wraps around +/-180.
    const double lat_gap = std::max({tile.south - query.prior.latitude, query.prior.latitude - tile.north, 0.0});
    double lon_gap = 180;
    for (double shifted : {query.prior.longitude - 360, query.prior.longitude, query.prior.longitude + 360})
        lon_gap = std::min(lon_gap, std::max({tile.west - shifted, shifted - tile.east, 0.0}));
    const double extreme_lat = std::max({std::abs(tile.south), std::abs(tile.north), std::abs(query.prior.latitude)});
    const double a = std::pow(std::sin(radians(lat_gap) / 2), 2) +
                     std::pow(std::cos(radians(extreme_lat)), 2) * std::pow(std::sin(radians(lon_gap) / 2), 2);
    return 6371008.8 * 2 * std::asin(std::sqrt(std::clamp(a, 0.0, 1.0))) <= query.radius_m;
}

cv::Mat loadGray(const fs::path& file, int max_side) {
    cv::Mat image = cv::imread(file.string(), cv::IMREAD_GRAYSCALE);
    if (image.empty()) throw std::runtime_error("Cannot read image: " + file.string());
    if (std::min(image.rows, image.cols) < 64) throw std::runtime_error("Image is too small (minimum 64 pixels per side)");
    const double scale = std::min(1.0, static_cast<double>(max_side) / std::max(image.cols, image.rows));
    if (scale < 1) cv::resize(image, image, {}, scale, scale, cv::INTER_AREA);
    return image;
}

cv::Mat normalizeImage(const cv::Mat& gray, bool gradient) {
    cv::Mat normalized;
    if (gradient) {
        cv::Mat smoothed, dx, dy, magnitude;
        cv::GaussianBlur(gray, smoothed, {3, 3}, 0.8);
        cv::Sobel(smoothed, dx, CV_32F, 1, 0);
        cv::Sobel(smoothed, dy, CV_32F, 0, 1);
        cv::magnitude(dx, dy, magnitude);
        cv::normalize(magnitude, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    } else normalized = gray;
    cv::Mat out;
    cv::createCLAHE(2.0, {8, 8})->apply(normalized, out);
    return out;
}

Features extractFeatures(const cv::Mat& gray, bool gradient, int count) {
    Features out;
    auto image = normalizeImage(gray, gradient);
    cv::ORB::create(count, 1.2f, 8, 19, 0, 2, cv::ORB::HARRIS_SCORE, 31, 12)
        ->detectAndCompute(image, cv::noArray(), out.keypoints, out.descriptors);
    return out;
}

Localizer::Localizer(fs::path folder, std::vector<Tile> tiles, Config config)
    : folder_(std::move(folder)), tiles_(std::move(tiles)), config_(config) {
    if (config_.cache_tiles == 0 || config_.max_side < 64 || config_.features < 32 ||
        config_.min_inliers < 4 || config_.min_inlier_ratio <= 0 || config_.min_inlier_ratio > 1 ||
        config_.min_query_coverage <= 0 || config_.min_query_coverage > 1 ||
        config_.max_rmse_px <= 0 || config_.ransac_px <= 0 || config_.ratio_test <= 0 ||
        config_.ratio_test >= 1 || config_.ambiguity_ratio <= 0 || config_.ambiguity_ratio > 1 ||
        config_.distinct_location_m < 0) throw std::runtime_error("Invalid localizer configuration");
}

const Localizer::CachedTile& Localizer::getTile(std::size_t index) {
    auto it = std::find_if(cache_.begin(), cache_.end(), [index](const CachedTile& t) { return t.index == index; });
    if (it != cache_.end()) { cache_.splice(cache_.begin(), cache_, it); return cache_.front(); }
    auto image = loadGray(imagePath(folder_, tiles_.at(index).file), config_.max_side);
    CachedTile tile{index, image.size(), extractFeatures(image, false, config_.features),
                   extractFeatures(image, true, config_.features)};
    cache_.push_front(std::move(tile));
    while (cache_.size() > config_.cache_tiles) cache_.pop_back();
    return cache_.front();
}

Result Localizer::verify(const Features& q, cv::Size qs, const Features& m, cv::Size ms,
                         const Tile& tile, const Query& query) {
    Result result;
    if (q.descriptors.rows < 2 || m.descriptors.rows < 2) return result;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> forward, reverse;
    matcher.knnMatch(q.descriptors, m.descriptors, forward, 2);
    matcher.knnMatch(m.descriptors, q.descriptors, reverse, 2);
    std::vector<cv::Point2f> source, destination;
    for (const auto& pair : forward) {
        if (pair.size() < 2 || pair[0].distance >= config_.ratio_test * pair[1].distance) continue;
        const auto& best = pair[0];
        const auto& back = reverse.at(static_cast<std::size_t>(best.trainIdx));
        if (back.size() < 2 || back[0].trainIdx != best.queryIdx || back[0].distance >= config_.ratio_test * back[1].distance) continue;
        source.push_back(q.keypoints.at(best.queryIdx).pt);
        destination.push_back(m.keypoints.at(best.trainIdx).pt);
    }
    if (source.size() < static_cast<std::size_t>(config_.min_inliers)) return result;
    cv::Mat mask;
    cv::setRNGSeed(42);
    auto h = cv::findHomography(source, destination, cv::RANSAC, config_.ransac_px, mask, 3000, 0.995);
    if (h.empty() || !cv::checkRange(h)) return result;
    const int count = cv::countNonZero(mask);
    const double ratio = static_cast<double>(count) / source.size();
    if (count < config_.min_inliers || ratio < config_.min_inlier_ratio) return result;
    std::vector<cv::Point2f> projected, accepted;
    cv::perspectiveTransform(source, projected, h);
    double squared_error = 0;
    for (std::size_t i = 0; i < source.size(); ++i) if (mask.at<unsigned char>(static_cast<int>(i))) {
        if (!finite(projected[i])) return result;
        accepted.push_back(source[i]);
        squared_error += std::pow(cv::norm(projected[i] - destination[i]), 2);
    }
    const double rmse = std::sqrt(squared_error / count);
    if (rmse > config_.max_rmse_px) return result;
    std::vector<cv::Point2f> hull;
    cv::convexHull(accepted, hull);
    if (std::abs(cv::contourArea(hull)) / qs.area() < config_.min_query_coverage) return result;
    // A homography with a horizon through the footprint can project plausible corners
    // but an unbounded interior. Require all corner denominators to share a sign.
    std::vector<cv::Point2f> footprint{{0, 0}, {static_cast<float>(qs.width - 1), 0},
        {static_cast<float>(qs.width - 1), static_cast<float>(qs.height - 1)}, {0, static_cast<float>(qs.height - 1)}};
    double previous_denominator = 0;
    for (auto p : footprint) {
        double denominator = h.at<double>(2, 0) * p.x + h.at<double>(2, 1) * p.y + h.at<double>(2, 2);
        if (std::abs(denominator) < 1e-8 || (previous_denominator != 0 && denominator * previous_denominator <= 0)) return result;
        previous_denominator = denominator;
    }
    cv::perspectiveTransform(footprint, footprint, h);
    for (auto p : footprint)
        if (!finite(p) || p.x < -0.05 * ms.width || p.y < -0.05 * ms.height || p.x > 1.05 * ms.width || p.y > 1.05 * ms.height) return result;
    const double area = std::abs(cv::contourArea(footprint));
    if (!cv::isContourConvex(footprint) || area < 100 || area > 1.2 * ms.area()) return result;
    std::vector<cv::Point2f> center{{(qs.width - 1) / 2.0f, (qs.height - 1) / 2.0f}};
    cv::perspectiveTransform(center, center, h);
    auto p = center.front();
    if (!finite(p) || p.x < 0 || p.y < 0 || p.x >= ms.width || p.y >= ms.height) return result;
    auto geo = pixelToGeo(tile, p, ms);
    if (distanceMeters(query.prior, geo) > query.radius_m) return result;
    result.matched = true;
    result.reason = "verified";
    result.position = geo;
    result.inliers = count;
    result.inlier_ratio = ratio;
    result.rmse_px = rmse;
    result.tile = tile.file;
    result.homography = h;
    result.confidence = std::min(1.0, count / 40.0) * ratio * std::exp(-rmse / 4.0);
    return result;
}

Result Localizer::locate(const Query& query, const fs::path& input_folder) {
    const auto started = std::chrono::steady_clock::now();
    Result result;
    result.name = query.file;
    try {
        const auto image = loadGray(imagePath(input_folder, query.file), config_.max_side);
        const auto intensity = extractFeatures(image, false, config_.features);
        const auto gradient = extractFeatures(image, true, config_.features);
        if (std::max(intensity.keypoints.size(), gradient.keypoints.size()) < static_cast<std::size_t>(config_.min_inliers)) {
            result.reason = "insufficient_texture";
        } else {
            std::vector<Result> candidates;
            std::size_t shortlisted = 0, readable = 0;
            for (std::size_t i = 0; i < tiles_.size(); ++i) {
                if (!intersectsPrior(tiles_[i], query)) continue;
                ++shortlisted;
                try {
                    const auto& map = getTile(i);
                    ++readable;
                    Result best;
                    // Both modalities are attempted for every query. IR tries structure first.
                    for (bool use_gradient : {query.modality == "IR", query.modality != "IR"}) {
                        auto candidate = verify(use_gradient ? gradient : intensity, image.size(),
                            use_gradient ? map.gradient : map.intensity, map.size, tiles_[i], query);
                        candidate.branch = use_gradient ? "gradient" : "intensity";
                        if (candidate.matched && (!best.matched || score(candidate) > score(best))) best = std::move(candidate);
                    }
                    if (best.matched) candidates.push_back(std::move(best));
                } catch (const std::exception& e) {
                    std::cerr << "Reference tile " << tiles_[i].file << ": " << e.what() << '\n';
                }
            }
            if (shortlisted == 0) result.reason = "outside_reference_coverage";
            else if (readable == 0) result.reason = "reference_unreadable";
            else if (!candidates.empty()) {
                std::sort(candidates.begin(), candidates.end(), [](const Result& a, const Result& b) { return score(a) > score(b); });
                result = candidates.front();
                result.name = query.file;
                for (std::size_t i = 1; i < candidates.size(); ++i)
                    if (distanceMeters(result.position, candidates[i].position) > config_.distinct_location_m &&
                        score(candidates[i]) >= config_.ambiguity_ratio * score(result)) {
                        result = Result{};
                        result.name = query.file;
                        result.reason = "ambiguous_locations";
                        break;
                    }
            }
        }
    } catch (const std::exception& e) {
        result.reason = "input_error";
        std::cerr << "Query " << query.file << ": " << e.what() << '\n';
    }
    result.processing_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    return result;
}
} // namespace uavloc
