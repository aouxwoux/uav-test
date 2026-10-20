#include "demo_scene.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/imgcodecs.hpp>

namespace {
void save(const uavloc::fs::path& path, const cv::Mat& image) {
    if (!cv::imwrite(path.string(), image)) throw std::runtime_error("Could not write " + path.string());
}
}
int main(int argc, char** argv) {
    if (argc != 2) { std::cerr << "Usage: make_demo OUTPUT_FOLDER\n"; return 2; }
    try {
        cv::setNumThreads(1);
        const uavloc::fs::path folder(argv[1]);
        if (uavloc::fs::exists(folder)) throw std::runtime_error("Output folder already exists; choose a new folder");
        uavloc::fs::create_directories(folder / "reference");
        uavloc::fs::create_directories(folder / "input");
        const auto map = demo::scene();
        cv::Mat h;
        auto eo = demo::view(map, h);
        eo.convertTo(eo, CV_8U, 0.8, 20);
        cv::Mat ir;
        cv::bitwise_not(demo::view(map, h), ir);
        cv::GaussianBlur(ir, ir, {3, 3}, 0.7);
        save(folder / "reference/map.png", map);
        save(folder / "input/day.png", eo);
        save(folder / "input/thermal_proxy.png", ir);
        save(folder / "input/blank.png", cv::Mat(480, 480, CV_8U, cv::Scalar(90)));
        save(folder / "input/unrelated.png", demo::scene(98765));
        const uavloc::Tile tile{"map.png", 12.01, 12.0, 77.01, 77.0};
        const auto point = demo::truth(tile, h);
        std::ofstream(folder / "reference/tiles.csv") << "file,north,south,east,west\nmap.png,12.01,12.0,77.01,77.0\n";
        std::ofstream queries(folder / "input/queries.csv");
        queries << "file,latitude,longitude,radius_m,modality\n"
                   "day.png,12.006,77.006,5000,EO\n"
                   "thermal_proxy.png,12.006,77.006,5000,IR\n"
                   "blank.png,12.006,77.006,5000,EO\n"
                   "unrelated.png,12.006,77.006,5000,IR\n";
        // Truth stays outside input/ so the localizer never reads the answers.
        std::ofstream truth(folder / "truth.csv");
        truth << std::fixed << std::setprecision(10)
              << "name,expected_status,latitude,longitude\n"
              << "day.png,matched," << point.latitude << ',' << point.longitude << '\n'
              << "thermal_proxy.png,matched," << point.latitude << ',' << point.longitude << '\n'
              << "blank.png,no_match,,\nunrelated.png,no_match,,\n";
        cv::Mat map_color, eo_color, ir_color;
        cv::cvtColor(map, map_color, cv::COLOR_GRAY2BGR);
        cv::cvtColor(eo, eo_color, cv::COLOR_GRAY2BGR);
        cv::cvtColor(ir, ir_color, cv::COLOR_GRAY2BGR);
        const std::vector<cv::Point> outline{{210,245},{700,210},{740,710},{175,745}};
        cv::polylines(map_color, outline, true, {70, 220, 110}, 5, cv::LINE_AA);
        std::vector<cv::Point2f> center{{239.5f,239.5f}};
        cv::perspectiveTransform(center, center, h);
        cv::drawMarker(map_color, center.front(), {0, 80, 255}, cv::MARKER_CROSS, 35, 5);
        cv::Mat montage(610, 1080, CV_8UC3, cv::Scalar(24, 28, 34));
        cv::Mat map_small, eo_small, ir_small;
        cv::resize(map_color, map_small, {560,560});
        cv::resize(eo_color, eo_small, {240,240});
        cv::resize(ir_color, ir_small, {240,240});
        map_small.copyTo(montage(cv::Rect(10,40,560,560)));
        eo_small.copyTo(montage(cv::Rect(585,100,240,240)));
        ir_small.copyTo(montage(cv::Rect(835,100,240,240)));
        cv::putText(montage, "SYNTHETIC MAP / KNOWN FOOTPRINT", {14,27}, cv::FONT_HERSHEY_SIMPLEX, 0.65, {220,230,240}, 1, cv::LINE_AA);
        cv::putText(montage, "DAY QUERY", {590,85}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {220,230,240}, 1, cv::LINE_AA);
        cv::putText(montage, "IR PROXY", {840,85}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {220,230,240}, 1, cv::LINE_AA);
        cv::putText(montage, "MATCH: pixel center -> latitude / longitude", {585,390}, cv::FONT_HERSHEY_SIMPLEX, 0.52, {100,225,145}, 1, cv::LINE_AA);
        cv::putText(montage, "ABSTAIN: blank / unrelated / ambiguous", {585,425}, cv::FONT_HERSHEY_SIMPLEX, 0.52, {150,195,245}, 1, cv::LINE_AA);
        cv::putText(montage, "IR proxy = inverted grayscale, not real thermal.", {585,485}, cv::FONT_HERSHEY_SIMPLEX, 0.48, {180,185,195}, 1, cv::LINE_AA);
        save(folder / "overview.png", montage);
        if (!queries || !truth) throw std::runtime_error("Could not write demo manifests");
        std::cout << "Synthetic demo created at " << folder << '\n';
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
