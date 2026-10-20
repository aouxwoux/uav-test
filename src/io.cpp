#include "uavloc/localizer.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <stdexcept>

namespace uavloc {
std::string csvEscape(const std::string& text) {
    if (text.find_first_of(",\"\r\n") == std::string::npos) return text;
    std::string out = "\"";
    for (char c : text) { if (c == '"') out += '"'; out += c; }
    return out + '"';
}

std::vector<std::vector<std::string>> readCsv(const fs::path& file) {
    std::ifstream stream(file);
    if (!stream) throw std::runtime_error("Cannot open manifest: " + file.string());
    std::vector<std::vector<std::string>> rows;
    std::string line;
    std::size_t number = 0;
    while (std::getline(stream, line)) {
        ++number;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (number == 1 && line.compare(0, 3, "\xEF\xBB\xBF") == 0) line.erase(0, 3);
        if (line.empty()) continue;
        std::vector<std::string> row;
        std::string field;
        bool quoted = false, closed = false;
        for (std::size_t i = 0; i < line.size(); ++i) {
            const char c = line[i];
            if (quoted) {
                if (c == '"') {
                    if (i + 1 < line.size() && line[i + 1] == '"') { field += '"'; ++i; }
                    else { quoted = false; closed = true; }
                } else field += c;
            } else if (c == ',') {
                row.push_back(field); field.clear(); closed = false;
            } else if (c == '"' && field.empty() && !closed) quoted = true;
            else {
                if (closed || c == '"') throw std::runtime_error("Malformed CSV at line " + std::to_string(number));
                field += c;
            }
        }
        if (quoted) throw std::runtime_error("Unclosed CSV quote at line " + std::to_string(number));
        row.push_back(field);
        rows.push_back(row);
    }
    if (stream.bad()) throw std::runtime_error("Error reading manifest: " + file.string());
    return rows;
}

fs::path imagePath(const fs::path& folder, const std::string& relative) {
    const fs::path p(relative);
    if (relative.empty() || p.is_absolute() || p.has_root_name() || p.has_root_directory())
        throw std::runtime_error("Image path must be relative: " + relative);
    for (const auto& component : p)
        if (component == "..") throw std::runtime_error("Image path cannot contain '..'");
    return folder / p;
}

namespace {
double number(const std::string& s) {
    std::size_t end = 0;
    double value = std::stod(s, &end);
    if (end != s.size() || !std::isfinite(value)) throw std::runtime_error("Invalid finite number: " + s);
    return value;
}
void header(const std::vector<std::vector<std::string>>& rows, const std::vector<std::string>& expected) {
    if (rows.empty() || rows.front() != expected) throw std::runtime_error("Unexpected CSV header; see README for exact columns");
    for (std::size_t i = 1; i < rows.size(); ++i)
        if (rows[i].size() != expected.size()) throw std::runtime_error("Wrong column count at data row " + std::to_string(i));
    if (rows.size() < 2) throw std::runtime_error("Manifest contains no data rows");
}
void coordinate(GeoPoint p) {
    if (std::abs(p.latitude) > 85 || std::abs(p.longitude) > 180)
        throw std::runtime_error("Coordinate outside supported latitude [-85,85] / longitude [-180,180]");
}
}

std::vector<Tile> readTiles(const fs::path& folder) {
    auto rows = readCsv(folder / "tiles.csv");
    header(rows, {"file", "north", "south", "east", "west"});
    std::vector<Tile> out;
    std::set<std::string> names;
    for (std::size_t i = 1; i < rows.size(); ++i) {
        const auto& r = rows[i];
        imagePath(folder, r[0]);
        if (!names.insert(r[0]).second) throw std::runtime_error("Duplicate tile filename: " + r[0]);
        Tile tile{r[0], number(r[1]), number(r[2]), number(r[3]), number(r[4])};
        coordinate({tile.north, tile.east}); coordinate({tile.south, tile.west});
        if (tile.north <= tile.south || tile.east <= tile.west || tile.east - tile.west > 2 || tile.north - tile.south > 2)
            throw std::runtime_error("Tiles must be small north-up geographic rectangles, without antimeridian crossing");
        out.push_back(tile);
    }
    return out;
}

std::vector<Query> readQueries(const fs::path& folder) {
    auto rows = readCsv(folder / "queries.csv");
    header(rows, {"file", "latitude", "longitude", "radius_m", "modality"});
    std::vector<Query> out;
    std::set<std::string> names;
    for (std::size_t i = 1; i < rows.size(); ++i) {
        const auto& r = rows[i];
        imagePath(folder, r[0]);
        if (!names.insert(r[0]).second) throw std::runtime_error("Duplicate query filename: " + r[0]);
        Query q{r[0], {number(r[1]), number(r[2])}, number(r[3]), r[4]};
        coordinate(q.prior);
        if (q.radius_m <= 0 || q.radius_m > 10000) throw std::runtime_error("radius_m must be in (0,10000]");
        if (q.modality != "EO" && q.modality != "IR") throw std::runtime_error("modality must be EO or IR");
        out.push_back(q);
    }
    return out;
}

void writeResults(const fs::path& output, const std::vector<Result>& results) {
    if (output.has_parent_path()) fs::create_directories(output.parent_path());
    std::ofstream stream(output);
    if (!stream) throw std::runtime_error("Cannot write output: " + output.string());
    stream << "name,status,latitude,longitude,confidence,reason,inliers,inlier_ratio,reprojection_error_px,processing_ms,tile,branch\n";
    stream << std::fixed << std::setprecision(8);
    for (const auto& r : results) {
        stream << csvEscape(r.name) << ',' << (r.matched ? "matched" : "no_match") << ',';
        if (r.matched) stream << r.position.latitude << ',' << r.position.longitude;
        else stream << ',';
        stream << ',' << r.confidence << ',' << csvEscape(r.reason) << ',' << r.inliers << ',' << r.inlier_ratio << ',';
        if (r.matched) stream << r.rmse_px;
        stream << ',' << r.processing_ms << ',' << csvEscape(r.tile) << ',' << r.branch << '\n';
    }
    stream.flush();
    if (!stream) throw std::runtime_error("Failed while writing output: " + output.string());
}
} // namespace uavloc
