#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "${ROOT_DIR}"

cmake -S . -B build/host -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host
ctest --test-dir build/host --output-on-failure

if rg -n '#include[[:space:]]*[<"](zephyr/|Arduino|board_pinmap)' modules/domain modules/contracts; then
  echo "Domain dependency boundary violation" >&2
  exit 1
fi

build/host/nura_host_sim build/host/traces/flight_trace.csv
build/host/nura_app_catalog build/host/traces/app_catalog.md
cmp documents/runtime_app_catalog.generated.md build/host/traces/app_catalog.md
python3 tools/tracemap_export.py build/host/traces/flight_trace.csv \
  --out-dir build/host/traces/report
rg -q 'FSM transitions: `8`' build/host/traces/report/summary.md

if [[ -x .venv-zephyr/bin/west && -d .zephyr-workspace/zephyr ]]; then
  scripts/build-zephyr.sh native_sim
  scripts/build-zephyr.sh teensy41
fi
