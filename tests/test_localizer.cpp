#include "uavloc/localizer.hpp"
#include "../tools/demo_scene.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <opencv2/imgcodecs.hpp>

namespace {
int checks = 0;
void check(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}
template<class F> void rejects(F operation, const std::string& message) {
    bool threw = false;
    try { operation(); } catch (const std::exception&) { threw = true; }
    check(threw, message);
}
}
int main() {
    const auto unique = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto root = uavloc::fs::temp_directory_path() / ("uavloc-tests-" + std::to_string(unique));
    try {
        cv::setNumThreads(1);
        uavloc::fs::create_directories(root / "reference");
        uavloc::fs::create_directories(root / "input");
        const auto map = demo::scene();
        cv::Mat h;
        const auto day = demo::view(map, h);
        cv::imwrite((root / "reference/map.png").string(), map);
        cv::imwrite((root / "reference/copy.png").string(), map);
        cv::imwrite((root / "input/day.png").string(), day);
        cv::imwrite((root / "input/blank.png").string(), cv::Mat(480,480,CV_8U,cv::Scalar(80)));
        cv::imwrite((root / "input/unrelated.png").string(), demo::scene(98765));
        cv::Mat ir;
        cv::bitwise_not(day, ir);
        cv::GaussianBlur(ir, ir, {3,3}, 0.7);
        cv::imwrite((root / "input/ir.png").string(), ir);
        const uavloc::Tile tile{"map.png", 12.01, 12.0, 77.01, 77.0};
        const auto truth = demo::truth(tile, h);
        const auto center = uavloc::pixelToGeo(tile, {511.5,511.5}, {1024,1024});
        check(std::abs(center.latitude - 12.005) < 1e-10 && std::abs(center.longitude - 77.005) < 1e-10, "Geographic pixel centre convention");
        check(uavloc::distanceMeters({0,0}, {0,1}) > 111000 && uavloc::distanceMeters({0,0}, {0,1}) < 111300, "Haversine distance");
        uavloc::Query query{"day.png", {12.006,77.006}, 5000, "EO"};
        check(uavloc::intersectsPrior(tile, query), "Prior includes map");
        uavloc::Localizer localizer(root / "reference", {tile});
        const auto eo = localizer.locate(query, root / "input");
        check(eo.matched, "Perspective day query must match");
        const auto eo_error = uavloc::distanceMeters(eo.position, truth);
        check(eo_error < 8, "Day query centre must be within eight synthetic metres");
        query.file = "ir.png"; query.modality = "IR";
        const auto thermal = localizer.locate(query, root / "input");
        check(thermal.matched && thermal.branch == "gradient", "Contrast-inverted proxy must use gradient matching");
        const auto ir_error = uavloc::distanceMeters(thermal.position, truth);
        check(ir_error < 8, "IR proxy centre must be within eight synthetic metres");
        query.file = "blank.png";
        check(localizer.locate(query, root / "input").reason == "insufficient_texture", "Blank input must abstain");
        query.file = "unrelated.png";
        check(!localizer.locate(query, root / "input").matched, "Unrelated scene must abstain");
        query.file = "missing.png";
        check(localizer.locate(query, root / "input").reason == "input_error", "Missing query must produce a row-level error");
        query.file = "day.png"; query.modality = "EO"; query.prior = {14,79}; query.radius_m = 100;
        check(localizer.locate(query, root / "input").reason == "outside_reference_coverage", "Distant prior must reject map");
        query.prior = {12.0095,77.0095}; query.radius_m = 30;
        check(!localizer.locate(query, root / "input").matched, "Visually valid location outside prior must abstain");
        query.prior = {12.006,77.006}; query.radius_m = 5000;
        auto duplicate = tile;
        duplicate.file = "copy.png"; duplicate.west += 0.007; duplicate.east += 0.007;
        uavloc::Localizer ambiguous(root / "reference", {tile,duplicate});
        check(ambiguous.locate(query, root / "input").reason == "ambiguous_locations", "Two equally strong distant matches must abstain");
        duplicate.west = tile.west; duplicate.east = tile.east;
        uavloc::Localizer overlapping(root / "reference", {tile,duplicate});
        check(overlapping.locate(query, root / "input").matched, "Overlapping tiles at the same location are not ambiguous");
        auto missing_tile = tile; missing_tile.file = "missing-map.png";
        uavloc::Localizer missing_map(root / "reference", {missing_tile});
        check(missing_map.locate(query, root / "input").reason == "reference_unreadable", "Missing references must be explicit");
        auto large_map = map.clone();
        cv::resize(map, large_map, {2048,2048});
        cv::imwrite((root / "reference/large.png").string(), large_map);
        auto large_tile = tile; large_tile.file = "large.png";
        uavloc::Localizer resized(root / "reference", {large_tile});
        const auto scaled = resized.locate(query, root / "input");
        check(scaled.matched && uavloc::distanceMeters(scaled.position, truth) < 8, "Resized map must preserve georeferencing");
        std::ofstream(root / "quoted.csv") << "file,value\n\"a,b\"\"c.png\",1\n";
        const auto csv = uavloc::readCsv(root / "quoted.csv");
        check(csv.size() == 2 && csv[1][0] == "a,b\"c.png", "Quoted CSV fields");
        rejects([&] { uavloc::imagePath(root, "../outside.png"); }, "Reject path traversal");
        std::ofstream(root / "input/queries.csv") << "file,latitude,longitude,radius_m,modality\nx.png,nan,77,100,EO\n";
        rejects([&] { uavloc::readQueries(root / "input"); }, "Reject nonfinite coordinates");
        std::ofstream(root / "input/queries.csv") << "file,latitude,longitude,radius_m,modality\nx.png,12,77,100,EO\nx.png,12,77,100,EO\n";
        rejects([&] { uavloc::readQueries(root / "input"); }, "Reject duplicate query identifiers");
        std::ofstream(root / "reference/tiles.csv") << "file,north,south,east,west\nmap.png,12,13,77.01,77\n";
        rejects([&] { uavloc::readTiles(root / "reference"); }, "Reject inverted bounds");
        uavloc::Result no_match; no_match.name = "unresolved,query.png";
        uavloc::writeResults(root / "results.csv", {eo, no_match});
        const auto rows = uavloc::readCsv(root / "results.csv");
        check(rows.size() == 3 && rows[2].size() == 12 && rows[2][2].empty() && rows[2][3].empty(), "One row per query, blank coordinates on abstention");
        check(rows[2][0] == no_match.name && rows[2][1] == "no_match", "Output CSV escaping and status");
        std::cout << checks << " checks passed; synthetic EO error " << eo_error << " m, IR proxy error " << ir_error << " m\n";
        uavloc::fs::remove_all(root);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << "\nFixtures retained at " << root << '\n';
        return 1;
    }
}
