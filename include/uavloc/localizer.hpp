#pragma once
#include <filesystem>
#include <list>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace uavloc {
namespace fs = std::filesystem;
struct GeoPoint { double latitude = 0, longitude = 0; };
struct Tile {
    std::string file;
    double north = 0, south = 0, east = 0, west = 0;
};
struct Query {
    std::string file;
    GeoPoint prior;
    double radius_m = 5000;
    std::string modality = "EO";
};
struct Config {
    int max_side = 1600;
    int features = 4000;
    std::size_t cache_tiles = 4;
    int min_inliers = 12;
    double min_inlier_ratio = 0.45;
    double min_query_coverage = 0.04;
    double max_rmse_px = 3.0;
    double ratio_test = 0.75;
    double ransac_px = 4.0;
    double ambiguity_ratio = 0.90;
    double distinct_location_m = 100.0;
};
struct Result {
    std::string name, reason = "no_verified_match", tile, branch;
    bool matched = false;
    GeoPoint position;
    double confidence = 0, inlier_ratio = 0, rmse_px = 0, processing_ms = 0;
    int inliers = 0;
    cv::Mat homography;
};
struct Features { std::vector<cv::KeyPoint> keypoints; cv::Mat descriptors; };

double distanceMeters(GeoPoint a, GeoPoint b);
GeoPoint pixelToGeo(const Tile& tile, cv::Point2d pixel, cv::Size image);
bool intersectsPrior(const Tile& tile, const Query& query);
cv::Mat loadGray(const fs::path& file, int max_side);
cv::Mat normalizeImage(const cv::Mat& gray, bool gradient);
Features extractFeatures(const cv::Mat& gray, bool gradient, int count);
std::vector<std::vector<std::string>> readCsv(const fs::path& file);
std::vector<Tile> readTiles(const fs::path& folder);
std::vector<Query> readQueries(const fs::path& folder);
fs::path imagePath(const fs::path& folder, const std::string& relative);
std::string csvEscape(const std::string& text);
void writeResults(const fs::path& output, const std::vector<Result>& results);

class Localizer {
public:
    Localizer(fs::path reference_folder, std::vector<Tile> tiles, Config config = {});
    Result locate(const Query& query, const fs::path& input_folder);
private:
    struct CachedTile {
        std::size_t index;
        cv::Size size;
        Features intensity, gradient;
    };
    const CachedTile& getTile(std::size_t index);
    Result verify(const Features& query_features, cv::Size query_size,
                  const Features& map_features, cv::Size map_size,
                  const Tile& tile, const Query& query);
    fs::path folder_;
    std::vector<Tile> tiles_;
    Config config_;
    std::list<CachedTile> cache_;
};
} // namespace uavloc
