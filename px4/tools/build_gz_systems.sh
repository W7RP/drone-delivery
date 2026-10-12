#!/usr/bin/env bash
# Build this repo's Gazebo systems (px4/sim/systems, Phase 8 chase camera) against the host's
# Gazebo Harmonic into px4/sim/systems/build; lib/sitl.sh puts that on GZ_SIM_SYSTEM_PLUGIN_PATH.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$HERE/sim/systems" -B "$HERE/sim/systems/build" -DCMAKE_BUILD_TYPE=Release > /dev/null
cmake --build "$HERE/sim/systems/build" -j "$(nproc)"
