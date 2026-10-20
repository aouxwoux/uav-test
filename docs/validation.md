# Prototype validation

Recorded on 20 October 2026. These are synthetic functional checks, not a UAV localization benchmark.

## Environment

- Windows Subsystem for Linux, distribution `Ubuntu-24.04`, x86_64.
- GNU C++ 13.3.0, CMake 3.28.3, OpenCV 4.6.0.
- Release build, OpenCV CPU threads set to one, fixed procedural image seed.
- No Jetson, CUDA, TensorRT, real EO/IR dataset, or competition imagery was used.

## Automated checks

Build and `ctest --test-dir build --output-on-failure` passed. CTest runs the functional test executable, generates demo fixtures, runs the batch entry point, and cleans up those fixtures. All four stages passed.

The functional executable passed **23 assertions**, covering:

- Geographic pixel-centre conversion and metre-distance calculation.
- Perspective image localization and an eight-synthetic-metre error bound.
- Gradient matching of the contrast-inverted IR proxy.
- Abstention on blank and unrelated images.
- Missing query and reference images.
- Prior-based shortlist rejection and rejection of a visual fix outside the prior radius.
- Ambiguity between duplicated imagery assigned to different geographic locations.
- Acceptance of duplicate overlapping tiles representing the same geographic location.
- Georeferencing after resizing a reference image.
- Quoted CSV fields and escaped output identifiers.
- Rejection of path traversal, nonfinite coordinates, duplicate query identifiers, and inverted map bounds.
- One output row per result, with empty coordinates for abstention.

The functional fixture's unmodified EO view had 0.20844 m synthetic centre error; the IR proxy had 0.190126 m. These subpixel results on images derived from the same generated map are **not evidence of field accuracy**.

## Standalone four-image demonstration

The separately generated demo applies a brightness change to its EO query. Its [observed output](demo-results.csv) and [synthetic truth](demo-truth.csv) are included for inspection. The truth file is never read by the localizer.

| Input | Result | Selected branch | Centre error in synthetic coordinates |
| --- | --- | --- | --- |
| `day.png` | matched | gradient | 0.0872 m |
| `thermal_proxy.png` | matched | gradient | 0.1903 m |
| `blank.png` | no_match / insufficient_texture | none | not applicable |
| `unrelated.png` | no_match / no_verified_match | none | not applicable |

The four statuses, row count, blank coordinates on abstention, and the accepted errors below eight metres were independently checked against `truth.csv`. Either branch can win on EO: modality controls attempt order, not a forced branch selection.

The recorded per-query processing times were approximately 1,127 ms, 867 ms, 20 ms, and 1,008 ms. These include preprocessing and tile descriptor extraction on cache misses, but exclude manifest parsing and final CSV writing. They come from one WSL run and are **not an end-to-end latency benchmark or a Jetson estimate**. Runtime and small floating-point differences can vary across OpenCV versions and machines.

## Still unvalidated

- Real thermal-to-daylight matching and seasonal change.
- Error tails, false-fix rate, confidence calibration, and availability across modality/altitude groups.
- Runtime and memory for the full reference region.
- ARM64/Jetson build, memory, thermals, batch latency, and offline container execution.
- Organizer-specific input/output schema and required container base.

The starter Dockerfile was not locally built because a running Docker daemon was unavailable. The C++ executables were built and run directly in WSL. GitHub Actions is provided for subsequent reproducible Linux checks; its status is separate from the local results above.
