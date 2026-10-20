#include "uavloc/localizer.hpp"
#include <iostream>

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: uav_localizer REFERENCE_FOLDER INPUT_FOLDER OUTPUT.csv\n"
                     "Reads tiles.csv and queries.csv; writes one row per query. See README.md.\n";
        return 0;
    }
    if (argc != 4) {
        std::cerr << "Usage: uav_localizer REFERENCE_FOLDER INPUT_FOLDER OUTPUT.csv\n";
        return 2;
    }
    try {
        // Fix OpenCV CPU threading and seed for repeatable demos; timing still varies.
        cv::setNumThreads(1);
        cv::setRNGSeed(42);
        auto tiles = uavloc::readTiles(argv[1]);
        auto queries = uavloc::readQueries(argv[2]);
        const auto output = uavloc::fs::weakly_canonical(argv[3]);
        // A mistyped output path must not overwrite an input or reference asset.
        for (const auto& folder : {uavloc::fs::path(argv[1]), uavloc::fs::path(argv[2])}) {
            const auto relative = output.lexically_relative(uavloc::fs::weakly_canonical(folder));
            if (!relative.empty() && *relative.begin() != "..")
                throw std::runtime_error("Output must be outside the reference and input folders");
        }
        uavloc::Localizer localizer(argv[1], std::move(tiles));
        std::vector<uavloc::Result> results;
        int matched = 0;
        for (const auto& query : queries) {
            auto result = localizer.locate(query, argv[2]);
            matched += result.matched ? 1 : 0;
            std::cerr << query.file << ": " << (result.matched ? "matched" : "no_match") << " (" << result.reason << ")\n";
            results.push_back(std::move(result));
        }
        uavloc::writeResults(argv[3], results);
        std::cerr << matched << '/' << queries.size() << " matched; results: " << argv[3] << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
