#!/usr/bin/env bash
# One delivery visit in PX4 SITL + Gazebo, headless, end to end:
#   px4/run_flight.sh --payload-kg X --seed N --out <dir> [--speed F] [--timeout S] [--lane-id L]
#                     [--airframe quad|vtol] [--wind-mps W --wind-from-deg D --gust-frac G]
#                     [--world-dir DIR] [--cameras]
# Takes off from A's OUT with a pod of X kg, flies lane 3 to B, lands on IN, releases, hops to
# OUT, latches the pod waiting there, and parks. Writes the logs into <dir>.
# --seed sets only the injected offsets (start position, detector corner noise); Gazebo and PX4
# are not seeded, so two runs with one seed are not bit-identical.
# --speed runs Gazebo and PX4 lockstep at F times real time (default 1).
# --lane-id (or --lane) flies any directed lane of the Pittsburgh catalog instead of lane 3: its lane record,
# world and scene are generated into <dir>.
# --airframe vtol flies the scaled standard VTOL (Phase 7): fixed-wing lane, multicopter at the
# pads. --timeout defaults to three times the visit's expected wall time; a run that outlasts
# it is stopped (Gazebo, PX4 and every node) and marked TIMEOUT. The run refuses to start with
# less than MIN_FREE_GB free. --wind-mps, even 0, flies with Gazebo's wind systems on (A94).
# --world-dir flies a generated terrain world (Phase 8, `pipeline/run.sh hero world`, default
# px4/sim/hero): its lane.json, scene.json and world_src.sdf replace the flat lane's.
# --cameras adds the clip's three render cameras (tools/hero_cameras.py) and streams them to
# <dir>/video as H.264 (tools/frame_recorder.py); nothing uncompressed reaches the disk.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MIN_FREE_GB=20
# Expected visit time: fixed pad and climb time plus the lane at cruise speed (s, sim).
EXPECTED_FIXED_S=200
EXPECTED_CRUISE_MPS=15

payload="" seed="" out="" speed=1 timeout_s="" lane_id="" airframe=quad world_dir="" cameras=false
wind=false wind_mps=0 wind_from=0 gust_frac=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --airframe) airframe="$2"; shift ;;
    --wind-mps) wind=true; wind_mps="$2"; shift ;;
    --wind-from-deg) wind_from="$2"; shift ;;
    --gust-frac) gust_frac="$2"; shift ;;
    --payload-kg) payload="$2"; shift ;;
    --seed) seed="$2"; shift ;;
    --out) out="$2"; shift ;;
    --speed) speed="$2"; shift ;;
    --timeout) timeout_s="$2"; shift ;;
    --lane-id|--lane) lane_id="$2"; shift ;;
    --world-dir) world_dir="$2"; shift ;;
    --cameras) cameras=true ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
[[ -n "$payload" && -n "$seed" && -n "$out" ]] || { echo "need --payload-kg, --seed and --out" >&2; exit 2; }
export AIRFRAME="$airframe"
# shellcheck source=lib/sitl.sh
source "$HERE/lib/sitl.sh"
free_gb=$(df -BG --output=avail "$HERE" | tail -1 | tr -dc 0-9)
(( free_gb >= MIN_FREE_GB )) || { echo "only ${free_gb} GB free; need ${MIN_FREE_GB}" >&2; exit 1; }
[[ -x "$HERE/ros2_ws/install/dd_flight/lib/dd_flight/drone_agent_node" ]] \
  || { echo "workspace not built: px4/setup_ws.sh" >&2; exit 1; }
mkdir -p "$out"
out="$(cd "$out" && pwd)"
rm -f "$out/run_status"
[[ -z "$lane_id" || -z "$world_dir" ]] || { echo "--lane-id and --world-dir exclude each other" >&2; exit 2; }
world_git="" world_sha=""
if [[ -n "$world_dir" ]]; then
  world_dir="$(cd "$world_dir" && pwd)"
  for f in lane.json scene.json world_src.sdf; do
    [[ -f "$world_dir/$f" ]] || { echo "$world_dir/$f missing: pipeline/run.sh hero world" >&2; exit 1; }
    cp "$world_dir/$f" "$out/$f"
  done
  # The generator's commit, marked dirty when pipeline/hero differs from it.
  world_git="$(git -C "$HERE" rev-parse --short=12 HEAD)"
  [[ -z "$(git -C "$HERE/.." status --porcelain -- pipeline/hero)" ]] || world_git="$world_git-dirty"
  world_sha="$(sha256sum "$out/world_src.sdf" | cut -c1-16)"
fi
if [[ -n "$lane_id" ]]; then
  "$HERE/py.sh" tools/select_lane.py --lane-id "$lane_id" --out "$out/lane.json"
  "$HERE/py.sh" tools/make_world.py --lane "$out/lane.json" --out-world "$out/world_src.sdf" \
    --out-scene "$out/scene.json" > /dev/null
fi

wind_flag=()
if [[ "$wind" == true ]]; then
  export DD_WIND="$wind_mps $wind_from $gust_frac"
  wind_flag=(--wind)
fi
"$HERE/py.sh" tools/make_vehicle.py --out "$out" --payload-kg "$payload" --airframe "$airframe" \
  "${wind_flag[@]}" > /dev/null
if [[ -z "$timeout_s" ]]; then
  lane_file="$out/lane.json"; [[ -f "$lane_file" ]] || lane_file="$HERE/config/lane.json"
  timeout_s=$(python3 -c "import json,sys; L=json.load(open(sys.argv[1]))['length_m']; print(int(3*($EXPECTED_FIXED_S + L/$EXPECTED_CRUISE_MPS)/$speed))" "$lane_file")
fi
"$HERE/py.sh" tools/make_ros_params.py --run "$out" --seed "$seed"
rm -rf "$out/video" "$out/cameras.json" "$out/server_cameras.config"
if [[ "$cameras" == true ]]; then
  [[ -f "$HERE/sim/systems/build/libdd_chase_camera.so" ]] \
    || { echo "chase camera system not built: px4/tools/build_gz_systems.sh" >&2; exit 1; }
  "$HERE/py.sh" tools/hero_cameras.py --run "$out" --server-config "$HERE/sim/server.config" > /dev/null
fi
# The px4/ code that flew this run: its commit, marked dirty when px4/ differs from it, so a
# batch can be checked to be all one version.
script_git="$(git -C "$HERE" rev-parse --short=12 HEAD)"
[[ -z "$(git -C "$HERE" status --porcelain -- . ':!runs')" ]] || script_git="$script_git-dirty"
# run_flight.sh's own last commit, and the world generator's (empty on a flat world).
run_flight_git="$(git -C "$HERE" log -1 --format=%h --abbrev=12 -- run_flight.sh)"
printf '{"payload_kg": %s, "seed": %s, "speed": %s, "airframe": "%s", "timeout_s": %s, "wind": %s, "wind_mps": %s, "wind_from_deg": %s, "gust_frac": %s, "script_git": "%s", "run_flight_git": "%s", "world_dir": "%s", "world_git": "%s", "world_sdf_sha256": "%s", "cameras": %s, "started_utc": "%s"}\n' \
  "$payload" "$seed" "$speed" "$airframe" "$timeout_s" "$wind" "$wind_mps" "$wind_from" "$gust_frac" \
  "$script_git" "$run_flight_git" "$world_dir" "$world_git" "$world_sha" "$cameras" "$(date -u +%FT%TZ)" > "$out/run.json"

python3 "$HERE/tools/rss_monitor.py" $$ "$out/rss.json" &
monitor=$!
finish() {
  sitl_stop_recorder
  [[ -d "$out/video" ]] && du -sb "$out/video" | cut -f1 > "$out/video_bytes"
  sitl_stop
  kill "$monitor" 2>/dev/null || true
  wait "$monitor" 2>/dev/null || true
}
trap finish EXIT

ros_env
sitl_env "$out"
sitl_start_gz "$out" "$speed"
sitl_spawn_scene "$out"
if [[ "$cameras" == true ]]; then
  sitl_spawn_cameras "$out"
  sitl_start_recorder "$out"
fi
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
