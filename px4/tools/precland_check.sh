#!/usr/bin/env bash
# Phase 4 step 5 check: one precision landing on A's IN spot from the approach height, with the
# marker detector feeding PX4's landing target. Prints the touchdown error from ground truth.
# Usage: tools/precland_check.sh <run_dir> [seed]
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../lib/sitl.sh
source "$HERE/lib/sitl.sh"
run="$1"; seed="${2:-1}"
TIMEOUT_S=240
mkdir -p "$run"
"$HERE/py.sh" tools/make_vehicle.py --out "$run" --payload-kg 5 >/dev/null
"$HERE/py.sh" tools/make_ros_params.py --run "$run" --seed "$seed"
trap sitl_stop EXIT
ros_env
sitl_env "$run"
sitl_start_gz "$run" 1.0
sitl_spawn_scene "$run"
sitl_start_agent "$run"
sitl_start_px4 "$run" 1
ros_bridge "$run"
ros_node "$run" marker_detector_node marker_detector
ros_node "$run" precland_check_node precland_check
for _ in $(seq 1 "$TIMEOUT_S"); do
  grep -q PRECLAND_RESULT "$run/precland_check.log" && break
  sleep 1
done
grep PRECLAND_RESULT "$run/precland_check.log" || { echo "no result in ${TIMEOUT_S} s"; exit 1; }
