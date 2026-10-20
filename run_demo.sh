#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if ! command -v cmake >/dev/null || ! command -v c++ >/dev/null; then
  printf '%s\n' 'Install the build tools first: sudo apt-get install build-essential cmake libopencv-dev' >&2
  exit 1
fi
cmake -S "$root" -B "$root/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$root/build" --target run_demo -j2
if (( $# > 1 )); then
  printf '%s\n' 'Usage: bash run_demo.sh [NEW_OUTPUT_FOLDER]' >&2
  exit 2
fi
output="${1:-$(mktemp -d "$root/build/demo-XXXXXX")/run}"
"$root/build/run_demo" "$output"
