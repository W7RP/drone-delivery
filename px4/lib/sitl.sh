# Shared by run_flight.sh: start and stop Gazebo, PX4 SITL and the uXRCE-DDS agent for one run.
# Source it; it defines functions only. Every process started is recorded in SITL_PIDS and
# stopped by sitl_stop, which also kills any Gazebo server left behind (they can ignore
# SIGTERM).

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
DD_PX4="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_NAME="dd_corridor"
# AIRFRAME=quad (PX4 4001, x500) or vtol (4004, standard VTOL; Phase 7). The model name is
# the one tools/make_vehicle.py writes for it.
AIRFRAME="${AIRFRAME:-quad}"
case "$AIRFRAME" in
  quad) PX4_AIRFRAME=4001; DRONE_MODEL=dd_quad ;;
  vtol) PX4_AIRFRAME=4004; DRONE_MODEL=dd_vtol ;;
  *) echo "unknown AIRFRAME $AIRFRAME" >&2; exit 2 ;;
esac
SITL_PIDS=()

sitl_log() { printf '[sitl %s] %s\n' "$(date +%H:%M:%S)" "$*" >&2; }

# sitl_env <run_dir>: Gazebo resource and plugin paths, PX4 attach variables.
sitl_env() {
  local run="$1"
  set +u
  # shellcheck disable=SC1091
  source "$PX4_BUILD/rootfs/gz_env.sh"
  set -u
  # Per-run models first (vehicle, pods), then this repo's world and markers, then PX4's.
  export GZ_SIM_RESOURCE_PATH="$run/models:$DD_PX4/sim/models:$DD_PX4/sim/worlds:$GZ_SIM_RESOURCE_PATH"
  export GZ_SIM_SERVER_CONFIG_PATH="$DD_PX4/sim/server.config"
  export GZ_IP=127.0.0.1
  export PX4_GZ_WORLD="$WORLD_NAME"
  export PX4_GZ_MODEL_NAME="$DRONE_MODEL"
  export PX4_SYS_AUTOSTART="$PX4_AIRFRAME"
  export HEADLESS=1
  unset PX4_SIM_MODEL
}

sitl_spawn() {  # <name> <sdf path> <x> <y> <z> <yaw>
  gz service -s "/world/$WORLD_NAME/create" --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean --timeout 10000 \
    --req "name: \"$1\", allow_renaming: false, sdf_filename: \"$2\", pose: {position: {x: $3, y: $4, z: $5}, orientation: {z: $(python3 -c "import math;print(math.sin($6/2))"), w: $(python3 -c "import math;print(math.cos($6/2))")}}" \
    | grep -q "data: true"
}

# sitl_start_gz <run_dir> <real_time_factor>
sitl_start_gz() {
  local run="$1" rtf="$2"
  if pgrep -f '^gz sim' >/dev/null; then
    sitl_log "a Gazebo server is already running; stop it first: pkill -KILL -f '^gz sim'"
    return 1
  fi
  # A run flown on its own lane brings its world (world_src.sdf); else the default lane's.
  local src="$DD_PX4/sim/worlds/$WORLD_NAME.sdf"
  [[ -f "$run/world_src.sdf" ]] && src="$run/world_src.sdf"
  sed "s|<real_time_factor>1.0</real_time_factor>|<real_time_factor>$rtf</real_time_factor>|" \
    "$src" > "$run/$WORLD_NAME.sdf"
  # Phase 7 wind check: DD_WIND="<m/s> <from deg> <gust fraction>" adds Gazebo wind (A94).
  if [[ -n "${DD_WIND:-}" ]]; then
    read -r w_mps w_from w_gust <<< "$DD_WIND"
    python3 "$DD_PX4/tools/add_wind.py" "$run/$WORLD_NAME.sdf" "$GZ_SIM_SERVER_CONFIG_PATH" \
      "$run/server.config" --mps "$w_mps" --from-deg "$w_from" --gust-frac "$w_gust"
    export GZ_SIM_SERVER_CONFIG_PATH="$run/server.config"
  fi
  gz sim --verbose=1 -s -r --headless-rendering "$run/$WORLD_NAME.sdf" > "$run/gz_server.log" 2>&1 &
  SITL_PIDS+=($!)
  local pid=$! i
  for i in $(seq 1 120); do
    kill -0 "$pid" 2>/dev/null || { sitl_log "Gazebo exited (see $run/gz_server.log)"; return 1; }
    gz service -i --service "/world/$WORLD_NAME/scene/info" 2>/dev/null | grep -q "Service providers" && return 0
    sleep 0.5
  done
  sitl_log "Gazebo did not come up"
  return 1
}

# sitl_start_agent <run_dir>
sitl_start_agent() {
  MicroXRCEAgent udp4 -p 8888 > "$1/xrce_agent.log" 2>&1 &
  SITL_PIDS+=($!)
}

# sitl_start_px4 <run_dir> <speed_factor>: PX4 attaches to the spawned dd_quad.
# Parameters go in twice: as PX4_PARAM_* at boot (rcS applies them before most modules start)
# and with px4-param once boot is done, because a later `param set-default` in the airframe
# scripts can override a boot-time value equal to the compiled default (quad-autonomy-sim,
# docs/environment.md issue 1).
sitl_start_px4() {
  local run="$1" speed="$2" name value i
  local params=()
  while read -r name value _; do
    [[ -z "$name" || "$name" == \#* ]] && continue
    params+=("$name=$value")
    export "PX4_PARAM_$name=$value"
  done < <(cat "$DD_PX4/params/sitl.params"; [[ "$AIRFRAME" == vtol ]] && cat "$DD_PX4/params/sitl_vtol.params")
  mkdir -p "$run/px4"
  export PX4_SIM_SPEED_FACTOR="$speed"
  ( cd "$run/px4" && exec "$PX4_BUILD/bin/px4" -d "$PX4_BUILD/etc" ) > "$run/px4.log" 2>&1 &
  SITL_PIDS+=($!)
  for i in $(seq 1 240); do
    grep -q "Startup script returned successfully" "$run/px4.log" 2>/dev/null && break
    sleep 0.5
  done
  local kv
  for kv in "${params[@]}"; do
    "$PX4_BUILD/bin/px4-param" set "${kv%%=*}" "${kv#*=}" >/dev/null 2>&1 \
      || { sitl_log "could not set ${kv%%=*}"; return 1; }
  done
  for i in $(seq 1 240); do
    grep -q "Ready for takeoff" "$run/px4.log" 2>/dev/null && break
    sleep 0.5
  done
  grep -q "Ready for takeoff" "$run/px4.log" || { sitl_log "PX4 not ready (see $run/px4.log)"; return 1; }
  sitl_log "PX4 ready, ${#params[@]} parameters applied"
}

# sitl_spawn_scene <run_dir>: drone, then pod 2 (detached: a detachable joint starts attached,
# and pod 2 waits on B's OUT), then pod 1, whose joint forms in the carry pose.
# Pod 2's detach is confirmed on the joint's state topic. A detach sent before the joint exists
# is lost and leaves pod 2 welded to the drone from across the lane; on a 441 m lane that
# showed up as a drone pinned off its climb with motors at 150 and 985 rad/s.
sitl_spawn_scene() {
  local run="$1" spec name i
  local state="$run/pod2_joint_state.txt"
  timeout 30 gz topic -e -t /dd/dd_pod_2/state > "$state" 2>/dev/null &
  sleep 0.5
  local names="$DRONE_MODEL dd_pod_2 dd_pod_1"
  [[ "$(python3 -c "import json,sys; print(len(json.load(open(sys.argv[1]))['pods']))" "$run/vehicle.json")" == 0 ]] \
    && names="$DRONE_MODEL"
  for name in $names; do
    spec="$(python3 -c "import json,sys; print(*json.load(open(sys.argv[1]))['spawn'][sys.argv[2]])" "$run/vehicle.json" "$name")"
    # shellcheck disable=SC2086
    sitl_spawn "$name" "$run/models/$name/model.sdf" $spec || { sitl_log "could not spawn $name"; return 1; }
    if [[ "$name" == dd_pod_2 ]]; then
      for i in $(seq 1 50); do grep -q '"attached"' "$state" && break; sleep 0.1; done
      for i in $(seq 1 10); do
        gz topic -t /dd/dd_pod_2/detach -m gz.msgs.Empty -p ' '
        sleep 0.5
        grep -q '"detached"' "$state" && break
      done
      grep -q '"detached"' "$state" || { sitl_log "pod 2 did not detach"; return 1; }
    fi
  done
  sitl_log "spawned $names"
}

sitl_stop() {
  local pid
  for pid in "${SITL_PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${SITL_PIDS[@]}"; do kill -KILL "$pid" 2>/dev/null || true; done
  pkill -KILL -f "^gz sim .*$WORLD_NAME.sdf" 2>/dev/null || true
  SITL_PIDS=()
}

# ROS 2 side. ROS setup scripts reference unset variables, so relax nounset while sourcing.
ros_env() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$DD_PX4/ros2_ws/install/setup.bash"
  set -u
  # The XRCE agent was installed into ~/.local by quad-autonomy-sim's setup (no root).
  export PATH="$HOME/.local/bin:$PATH"
  export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"
}

# ros_node <run_dir> <executable> <node name>: start one of our nodes with the run's params.
ros_node() {
  local run="$1" exe="$2" name="$3"
  "$DD_PX4/ros2_ws/install/dd_flight/lib/dd_flight/$exe" --ros-args \
    -r __node:="$name" --params-file "$run/ros_params.yaml" > "$run/$name.log" 2>&1 &
  SITL_PIDS+=($!)
}

ros_bridge() {
  local run="$1"
  /opt/ros/humble/lib/ros_gz_bridge/parameter_bridge --ros-args \
    -p config_file:="$run/bridge.yaml" > "$run/bridge.log" 2>&1 &
  SITL_PIDS+=($!)
}
