#include "demo_data.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {
std::string escape(const std::string& s) {
    std::string out;
    for (const auto c : s) {
        if (c == '&') out += "&amp;";
        else if (c == '<') out += "&lt;";
        else if (c == '>') out += "&gt;";
        else if (c == '"') out += "&quot;";
        else out += c;
    }
    return out;
}
std::string fixed(double n, int precision) {
    std::ostringstream s; s << std::fixed << std::setprecision(precision) << n; return s.str();
}
void report(const uavloc::fs::path& folder, const std::vector<uavloc::Result>& results,
            const std::vector<double>& errors) {
    std::ofstream html(folder / "report.html");
    if (!html) throw std::runtime_error("Cannot write report");
    html << R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>UAV localizer / demo</title><style>
*{box-sizing:border-box}body{margin:0;background:#10181d;color:#ecf0e9;font:16px system-ui,sans-serif}
main{max-width:1160px;margin:auto;padding:32px 30px}header{display:flex;justify-content:space-between;align-items:center;color:#aebbb5;font-size:13px}
header span{border:1px solid #3b4b44;padding:8px 12px;border-radius:20px}h1{font-size:clamp(27px,4vw,42px);letter-spacing:-1.5px;margin:28px 0 10px}
.intro{color:#b1beb7;margin:0 0 24px;line-height:1.6}nav{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:22px}
button{font:inherit;background:#1d292d;color:#c5d1ca;border:1px solid #3e4e48;border-radius:8px;padding:10px 18px;cursor:pointer}
button[aria-selected=true]{background:#c7edb0;color:#132118;border-color:#c7edb0}.case[hidden]{display:none!important}
.images{display:grid;grid-template-columns:1fr 1fr;gap:20px}.panel{background:#182329;border:1px solid #33433d;border-radius:12px;overflow:hidden}
.label{display:flex;justify-content:space-between;padding:14px 16px;font-size:13px;color:#b8c8bf}.panel img{display:block;width:100%;aspect-ratio:1;object-fit:contain;background:#202827}
.metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:14px;margin-top:20px}.metric{background:#182329;border:1px solid #33433d;border-radius:9px;padding:16px;min-width:0}
.metric small{display:block;color:#a6b6ac;font-size:12px;margin-bottom:8px}.metric strong{font-size:19px;overflow-wrap:anywhere}.yes{color:#bde4a6}.no{color:#f5c58c}
.explain{color:#b1beb7;font-size:14px;line-height:1.7;margin:20px 0}footer{display:flex;gap:20px;flex-wrap:wrap;border-top:1px solid #33433d;padding-top:18px;color:#90a399;font-size:12px}a{color:#c7edb0}
@media(min-width:900px){main{padding-top:20px;padding-bottom:20px}h1{font-size:32px;margin:18px 0 10px}.intro{margin-bottom:14px}nav{margin-bottom:12px}.panel img{height:clamp(220px,34vh,360px);aspect-ratio:auto}.metrics{margin-top:12px}.metric{padding:12px}.metric strong{font-size:17px}.explain{margin:12px 0;font-size:13px}footer{padding-top:12px}}
@media(max-width:700px){main{padding:22px 16px}.images{grid-template-columns:1fr}.metrics{grid-template-columns:1fr 1fr}}
</style></head><body><main>
<header><b>UAV / VISUAL LOCALIZER</b><span>C++17 + OpenCV / offline</span></header>
<h1>Find the image. Or say no match.</h1>
<p class="intro">Four generated inputs, processed by the C++ matcher. Pick a case to see what it found.</p>
<nav role="tablist" aria-label="Demo cases">
)HTML";
    const std::vector<std::string> labels{"Daylight", "IR proxy", "Blank image", "Unrelated image"};
    for (std::size_t i = 0; i < results.size(); ++i)
        html << "<button role=\"tab\" id=\"tab" << i << "\" aria-controls=\"case" << i
             << "\" aria-selected=\"" << (i == 0 ? "true" : "false") << "\" onclick=\"show(" << i << ")\">" << labels.at(i) << "</button>\n";
    html << "</nav>\n";
    const auto map = cv::imread((folder / "reference/map.png").string());
    if (map.empty()) throw std::runtime_error("Cannot load demo map for report");
    for (std::size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        auto overlay = map.clone();
        if (r.matched) {
            auto query = cv::imread((folder / "input" / r.name).string());
            std::vector<cv::Point2f> corners{{0,0},{static_cast<float>(query.cols-1),0},
                {static_cast<float>(query.cols-1),static_cast<float>(query.rows-1)},{0,static_cast<float>(query.rows-1)}};
            cv::perspectiveTransform(corners, corners, r.homography);
            std::vector<cv::Point> outline;
            for (auto p : corners) outline.emplace_back(cvRound(p.x),cvRound(p.y));
            cv::polylines(overlay, outline, true, {140,235,170}, 5, cv::LINE_AA);
            std::vector<cv::Point2f> centre{{(query.cols-1)/2.0f,(query.rows-1)/2.0f}};
            cv::perspectiveTransform(centre, centre, r.homography);
            cv::drawMarker(overlay, centre.front(), {50,140,255}, cv::MARKER_CROSS, 36, 5);
        }
        const auto asset = "prediction-" + std::to_string(i) + ".png";
        if (!cv::imwrite((folder / asset).string(), overlay)) throw std::runtime_error("Cannot write prediction image");
        html << "<section role=\"tabpanel\" id=\"case" << i << "\" aria-labelledby=\"tab" << i << "\" class=\"case\"" << (i ? " hidden" : "") << ">\n"
             << "<div class=\"images\"><div class=\"panel\"><div class=\"label\"><span>INPUT</span><span>" << escape(r.name) << "</span></div>"
             << "<img alt=\"" << labels[i] << " input\" src=\"input/" << escape(r.name) << "\"></div>"
             << "<div class=\"panel\"><div class=\"label\"><span>REFERENCE MAP</span><span>" << (r.matched ? "green = predicted footprint" : "no position returned") << "</span></div>"
             << "<img alt=\"Reference map with matching result\" src=\"" << asset << "\"></div></div>\n"
             << "<div class=\"metrics\"><div class=\"metric\"><small>RESULT</small><strong class=\"" << (r.matched ? "yes" : "no") << "\">" << (r.matched ? "matched" : "no match") << "</strong></div>"
             << "<div class=\"metric\"><small>LAT / LON</small><strong>" << (r.matched ? fixed(r.position.latitude,6) + " / " + fixed(r.position.longitude,6) : "no fix") << "</strong></div>"
             << "<div class=\"metric\"><small>INLIERS / RUNTIME</small><strong>" << r.inliers << " / " << fixed(r.processing_ms,0) << " ms</strong></div>"
             << "<div class=\"metric\"><small>SYNTHETIC CENTRE ERROR</small><strong>" << (r.matched ? fixed(errors.at(i),2) + " m" : "n/a") << "</strong></div></div>\n"
             << "<p class=\"explain\">";
        if (i == 0) html << "A perspective-warped, brightness-adjusted view. Consistent feature matches locate its centre on the map.";
        if (i == 1) html << "A blurred, contrast-inverted view. The gradient branch recovers structure. This is a proxy, not real thermal imagery.";
        if (i == 2) html << "No usable texture, so there is no reliable visual evidence for a location.";
        if (i == 3) html << "This image has features, but they do not agree with the reference map. The matcher returns no fix.";
        html << " Decision: <code>" << escape(r.reason) << "</code>";
        if (r.matched) html << " / branch: <code>" << escape(r.branch) << "</code>";
        html << ".</p></section>\n";
    }
    html << R"HTML(<footer><span>Generated imagery. No field-accuracy claim. Jetson performance untested.</span><a href="results.csv">Results CSV</a><a href="truth.csv">Synthetic truth</a></footer>
</main><script>
function show(n){document.querySelectorAll('.case').forEach((el,i)=>{el.hidden=i!==n});document.querySelectorAll('[role=tab]').forEach((el,i)=>{el.setAttribute('aria-selected',String(i===n))})}
document.querySelector('nav').addEventListener('keydown',e=>{if(e.key!=='ArrowLeft'&&e.key!=='ArrowRight')return;const tabs=[...document.querySelectorAll('[role=tab]')];let n=tabs.findIndex(el=>el.getAttribute('aria-selected')==='true');n=(n+(e.key==='ArrowRight'?1:-1)+tabs.length)%tabs.length;show(n);tabs[n].focus();e.preventDefault()});
</script></body></html>)HTML";
    html.flush();
    if (!html) throw std::runtime_error("Failed to write demo report");
}
}

int main(int argc, char** argv) {
    if (argc != 2) { std::cerr << "Usage: run_demo NEW_OUTPUT_FOLDER\n"; return 2; }
    try {
        std::cout << std::unitbuf;
        const auto folder = uavloc::fs::absolute(argv[1]);
        std::cout << "[1/3] Generate four synthetic inputs\n";
        demo::generate(folder);
        auto tiles = uavloc::readTiles(folder / "reference");
        auto queries = uavloc::readQueries(folder / "input");
        uavloc::Localizer localizer(folder / "reference", std::move(tiles));
        std::vector<uavloc::Result> results;
        std::cout << "[2/3] Run the C++ matcher\n";
        for (const auto& q : queries) {
            auto r = localizer.locate(q, folder / "input");
            std::cout << "  " << q.file << ": " << (r.matched ? "matched" : "no_match") << " / " << r.reason << '\n';
            results.push_back(std::move(r));
        }
        uavloc::writeResults(folder / "results.csv", results);
        // Read answers only after localization, for demonstration assertions/reporting.
        const auto truth = uavloc::readCsv(folder / "truth.csv");
        std::vector<double> errors;
        for (std::size_t i = 0; i < results.size(); ++i) {
            const auto& r = results[i];
            const bool expected = truth.at(i+1).at(1) == "matched";
            if (r.matched != expected) throw std::runtime_error("Unexpected synthetic result for " + r.name);
            double error = 0;
            if (r.matched) {
                error = uavloc::distanceMeters(r.position, {std::stod(truth.at(i+1).at(2)), std::stod(truth.at(i+1).at(3))});
                if (error > 8) throw std::runtime_error("Synthetic location error exceeds demo tolerance");
            }
            errors.push_back(error);
        }
        std::cout << "[3/3] Write visual report\n";
        report(folder, results, errors);
        std::cout << "PASS: 2 matches, 2 no-match cases. Synthetic errors below 8 m.\n"
                  << "Open in your browser: " << (folder / "report.html").string() << '\n';
        return 0;
    } catch (const std::exception& e) { std::cerr << "Demo failed: " << e.what() << '\n'; return 1; }
}
