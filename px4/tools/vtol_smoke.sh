#!/usr/bin/env bash
# Phase 7 step 4: can the scaled VTOL fly? No ROS nodes, no pods: PX4's own commander takes
# off, transitions to fixed wing and loiters, transitions back, and lands, while this script
# samples vehicle_status, local position and airspeed. Bounded by --timeout (s, wall).
#   px4/tools/vtol_smoke.sh --out <dir> [--payload-kg 5] [--loiter-s 60] [--alt-m 61] [--timeout 600]
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export AIRFRAME=vtol
# shellcheck source=../lib/sitl.sh
source "$HERE/lib/sitl.sh"
out="" payload=5 loiter_s=60 timeout_s=600 alt_m=61
while [[ $# -gt 0 ]]; do
  case "$1" in
    --out) out="$2"; shift ;;
    --payload-kg) payload="$2"; shift ;;
    --loiter-s) loiter_s="$2"; shift ;;
    --timeout) timeout_s="$2"; shift ;;
    --alt-m) alt_m="$2"; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
[[ -n "$out" ]] || { echo "need --out" >&2; exit 2; }
free_gb=$(df -BG --output=avail "$HERE" | tail -1 | tr -dc 0-9)
(( free_gb >= 20 )) || { echo "only ${free_gb} GB free; need 20" >&2; exit 1; }
mkdir -p "$out"; out="$(cd "$out" && pwd)"
# The carried pod's mass rides in the latch for this smoke (no pod models).
"$HERE/py.sh" tools/make_vehicle.py --out "$out" --payload-kg 0 --pods 0 --airframe vtol \
  --latch-kg "$(python3 -c "print(0.6 + $payload)")" > /dev/null
( sleep "$timeout_s"; echo "TIMEOUT" > "$out/status"; kill -TERM $$ ) &
watchdog=$!
trap 'kill $watchdog 2>/dev/null || true; sitl_stop' EXIT
sitl_env "$out"
sitl_start_gz "$out" 1
sitl_spawn_scene "$out"
sitl_start_px4 "$out" 1
cmd() { "$PX4_BUILD/bin/px4-commander" "$@" >> "$out/commander.log" 2>&1; }
probe() {  # <label>: one line of state
  local st pos arsp
  st=$("$PX4_BUILD/bin/px4-listener" vehicle_status -n 1 2>/dev/null | grep -E "^\s+(vehicle_type|in_transition_mode|nav_state|arming_state):" | tr -s ' ' | tr '\n' ' ')
  pos=$("$PX4_BUILD/bin/px4-listener" vehicle_local_position -n 1 2>/dev/null | grep -E "^\s+(x|y|z|vx|vy|vz):" | tr -s ' ' | tr '\n' ' ')
  arsp=$("$PX4_BUILD/bin/px4-listener" airspeed_validated -n 1 2>/dev/null | grep -E "true_airspeed_m_s" | tr -s ' ' | tr '\n' ' ')
  echo "$(date +%s) $1 | $st| $pos| $arsp" >> "$out/probe.log"
}
"$PX4_BUILD/bin/px4-param" set MIS_TAKEOFF_ALT "$alt_m" > /dev/null
probe boot
cmd takeoff
for i in $(seq 1 45); do sleep 2; probe takeoff; done
cmd transition
for i in $(seq 1 $((loiter_s / 3))); do sleep 3; probe fw; done
cmd transition
for i in $(seq 1 10); do sleep 2; probe back; done
cmd land
for i in $(seq 1 60); do
  sleep 2; probe land
  "$PX4_BUILD/bin/px4-listener" vehicle_land_detected -n 1 2>/dev/null | grep -q "landed: True" && break
done
echo DONE > "$out/status"
