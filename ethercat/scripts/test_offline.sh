#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
python3 scripts/import_pmac.py --check
cmake -S . -B build/offline -G Ninja -DCMAKE_BUILD_TYPE=Debug -DENABLE_SANITIZERS=ON
cmake --build build/offline -j "${IGH_BUILD_JOBS:-4}"
ctest --test-dir build/offline --output-on-failure --output-junit results.xml
python3 scripts/doctor.py --output build/host-report.json
