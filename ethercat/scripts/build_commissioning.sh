#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
# The instrumented offline build remains separate and is still tested.
cmake -S . -B build/commissioning -G Ninja -DCMAKE_BUILD_TYPE=Release -DENABLE_SANITIZERS=OFF
cmake --build build/commissioning --target igh_disabled_probe -j "${IGH_BUILD_JOBS:-4}"
