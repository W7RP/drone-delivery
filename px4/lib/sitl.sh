# Shared by run_flight.sh: start and stop Gazebo, PX4 SITL and the uXRCE-DDS agent for one run.
# Source it; it defines functions only. Every process started is recorded in SITL_PIDS and
# stopped by sitl_stop, which also kills any Gazebo server left behind (they can ignore
# SIGTERM).

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD="$PX4_DIR/build/px4_sitl_default"
DD_PX4="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_NAME="dd_corridor"
X500_AIRFRAME=4001
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
  export PX4_GZ_MODEL_NAME="dd_quad"
  export PX4_SYS_AUTOSTART="$X500_AIRFRAME"
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
  sed "s|<real_time_factor>1.0</real_time_factor>|<real_time_factor>$rtf</real_time_factor>|" \
    "$DD_PX4/sim/worlds/$WORLD_NAME.sdf" > "$run/$WORLD_NAME.sdf"
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
  done < "$DD_PX4/params/sitl.params"
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

# sitl_spawn_scene <run_dir>: drone, then pod 2 (detached at once: a detachable joint starts
# attached, and pod 2 waits on B's OUT), then pod 1, whose joint forms in the carry pose.
sitl_spawn_scene() {
  local run="$1" spec name
  for name in dd_quad dd_pod_2 dd_pod_1; do
    spec="$(python3 -c "import json,sys; print(*json.load(open(sys.argv[1]))['spawn'][sys.argv[2]])" "$run/vehicle.json" "$name")"
    # shellcheck disable=SC2086
    sitl_spawn "$name" "$run/models/$name/model.sdf" $spec || { sitl_log "could not spawn $name"; return 1; }
    if [[ "$name" == dd_pod_2 ]]; then
      gz topic -t /dd/dd_pod_2/detach -m gz.msgs.Empty -p ' '
    fi
  done
  sitl_log "spawned dd_quad, dd_pod_2 (detached), dd_pod_1 (latched)"
}

sitl_stop() {
  local pid
  for pid in "${SITL_PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${SITL_PIDS[@]}"; do kill -KILL "$pid" 2>/dev/null || true; done
  pkill -KILL -f "^gz sim .*$WORLD_NAME.sdf" 2>/dev/null || true
  SITL_PIDS=()
}
