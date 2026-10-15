#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:?Usage: run_cloud_snapshot_test.sh OUTPUT_DIRECTORY}"
mkdir -p "$OUT"
g++ -std=c++17 -O2 -pthread -I "$ROOT/src" -I "$ROOT/thirdparty" \
  -I /usr/include/pcl-1.10 -I /usr/include/eigen3 \
  "$ROOT/tests/cloud_snapshot_test.cpp" -o "$OUT/cloud_snapshot_test" -lpcl_common
"$OUT/cloud_snapshot_test"
