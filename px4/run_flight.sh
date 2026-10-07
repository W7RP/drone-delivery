#!/usr/bin/env bash
# One delivery visit in PX4 SITL + Gazebo, headless, end to end:
#   px4/run_flight.sh --payload-kg X --seed N --out <dir> [--speed F] [--timeout S]
# Takes off from A's OUT with a pod of X kg, flies lane 3 to B, lands on IN, releases, hops to
# OUT, latches the pod waiting there, and parks. Writes the logs into <dir>.
# --seed sets only the injected offsets (start position, detector corner noise); Gazebo and PX4
# are not seeded, so two runs with one seed are not bit-identical.
# --speed runs Gazebo and PX4 lockstep at F times real time (default 1).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/sitl.sh
source "$HERE/lib/sitl.sh"

payload="" seed="" out="" speed=1 timeout_s=1200
while [[ $# -gt 0 ]]; do
  case "$1" in
    --payload-kg) payload="$2"; shift ;;
    --seed) seed="$2"; shift ;;
    --out) out="$2"; shift ;;
    --speed) speed="$2"; shift ;;
    --timeout) timeout_s="$2"; shift ;;
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
[[ -n "$payload" && -n "$seed" && -n "$out" ]] || { echo "need --payload-kg, --seed and --out" >&2; exit 2; }
[[ -x "$HERE/ros2_ws/install/dd_flight/lib/dd_flight/drone_agent_node" ]] \
  || { echo "workspace not built: px4/setup_ws.sh" >&2; exit 1; }
mkdir -p "$out"
out="$(cd "$out" && pwd)"
rm -f "$out/run_status"

"$HERE/py.sh" tools/make_vehicle.py --out "$out" --payload-kg "$payload" > /dev/null
"$HERE/py.sh" tools/make_ros_params.py --run "$out" --seed "$seed"
printf '{"payload_kg": %s, "seed": %s, "speed": %s, "started_utc": "%s"}\n' \
  "$payload" "$seed" "$speed" "$(date -u +%FT%TZ)" > "$out/run.json"

python3 "$HERE/tools/rss_monitor.py" $$ "$out/rss.json" &
monitor=$!
finish() {
  sitl_stop
  kill "$monitor" 2>/dev/null || true
  wait "$monitor" 2>/dev/null || true
}
trap finish EXIT

ros_env
sitl_env "$out"
sitl_start_gz "$out" "$speed"
sitl_spawn_scene "$out"
sitl_start_agent "$out"
sitl_start_px4 "$out" "$speed"
ros_bridge "$out"
ros_node "$out" flight_logger_node flight_logger
ros_node "$out" marker_detector_node marker_detector
ros_node "$out" pad_controller_node pad_a
ros_node "$out" pad_controller_node pad_b
ros_node "$out" fleet_manager_node fleet_manager
ros_node "$out" drone_agent_node drone_agent

start=$SECONDS
while [[ ! -f "$out/run_status" ]]; do
  if (( SECONDS - start > timeout_s )); then
    echo "TIMEOUT after ${timeout_s} s" > "$out/run_status"
    break
  fi
  sleep 1
done
status="$(cat "$out/run_status")"
echo "$((SECONDS - start))" > "$out/wall_s"
echo "$(basename "$out"): $status ($((SECONDS - start)) s wall)"
[[ "$status" == COMPLETE ]]
