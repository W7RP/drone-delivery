#!/usr/bin/env bash
# Whole verification suite; exits non-zero on the first failure.
#  1. Spec, schemas and configs agree (check_spec.py, via uv; pinned deps in uv's cache only).
#  2. Offline pipeline unit tests (pipeline/run.sh test; creates pipeline/.venv on first use),
#     including the committed weather series against spec 9.7 and its 1 MB limit.
#  3. PX4 phase offline checks (log reader tests, node unit tests, the committed sample log).
#     Skip with VERIFY_SKIP_PX4=1.
#  4. Rust core: formatting, clippy with warnings as errors, all tests (debug build, so the
#     invariants run after every event). Includes loading the committed Pittsburgh network files.
#  5. Release budget runs (spec A39: 60 s wall, 200 MB peak RSS each): the 200-drone 7-day scale
#     case; the 7-day Pittsburgh scenario with each flight profile (fitted, placeholder, and fitted
#     with the 1370 Wh battery), at 1370 Wh with the Phase 6a pad layout and repair keys, and
#     the coupled 900 Wh layout of Phase 6b, and the Phase 6b demo with weather and traffic;
#     the Phase 7 VTOL scenario, its layout and its demo (every 1.9.0 key on), whose order
#     outcomes must account for every request; and the placeholder Pittsburgh at three times its
#     demand, which must turn requests away at the backlog cap instead of queueing them without
#     limit. Skip with VERIFY_SKIP_SCALE=1.
#  6. The Phase 7 VTOL fit reruns from its committed table to byte-identical parameters
#     (pipeline/run.sh calib vtol_fit, then git diff). Skip with VERIFY_SKIP_SCALE=1.
set -euo pipefail
cd "$(dirname "$0")/.."
[ -d "$HOME/.cargo/bin" ] && PATH="$HOME/.cargo/bin:$PATH"

echo "== spec and schema checks"
uv run --quiet scripts/check_spec.py

echo "== pipeline unit tests"
pipeline/run.sh test 2>&1 | tail -n 3

if [ "${VERIFY_SKIP_PX4:-0}" != "1" ]; then
  # Offline checks of the PX4 phase (no simulator): log reader tests, the flight nodes' unit
  # tests, and the committed sample flight log against the validator. Needs ROS 2 Humble and a
  # built px4/ros2_ws (px4/setup_ws.sh); VERIFY_SKIP_PX4=1 skips them.
  echo "== px4: log reader tests, node unit tests, sample log"
  px4/py.sh tests/test_flightlog.py 2>&1 | tail -n 1
  (
    set +u
    source /opt/ros/humble/setup.bash
    source px4/ros2_ws/install/setup.bash
    set -u
    cd px4/ros2_ws
    colcon test --packages-select dd_flight --event-handlers console_direct- > /dev/null
    colcon test-result --all | tail -n 1
    colcon test-result > /dev/null
  )
  px4/py.sh tools/flightlog.py validate px4/samples/*/
fi

echo "== rust: fmt, clippy, tests"
(cd sim && cargo fmt --check && cargo clippy --quiet --all-targets -- -D warnings && cargo test --quiet)

# budget_run <label> <scenario.toml> <out dir>: release run within the A39 budget.
budget_run() {
  /usr/bin/time -v sim/target/release/sim run "$2" --out "$3" > /dev/null 2> "$3.time"
  local wall rss_kb secs
  wall="$(awk -F': ' '/Elapsed \(wall clock\)/ {print $2}' "$3.time")"
  rss_kb="$(awk -F': ' '/Maximum resident set size/ {print $2}' "$3.time")"
  secs="$(echo "$wall" | awk -F: '{ if (NF == 3) print $1*3600 + $2*60 + $3; else print $1*60 + $2 }')"
  echo "$1: wall ${secs} s, peak RSS $((rss_kb / 1024)) MB (budget 60 s, 200 MB)"
  awk -v s="$secs" 'BEGIN { exit !(s < 60) }' || { echo "FAIL: $1 wall time over budget"; exit 1; }
  [ "$rss_kb" -lt $((200 * 1024)) ] || { echo "FAIL: $1 peak RSS over budget"; exit 1; }
}

if [ "${VERIFY_SKIP_SCALE:-0}" != "1" ]; then
  echo "== release budget runs"
  out="$(mktemp -d)"
  trap 'rm -rf "$out"' EXIT
  (cd sim && cargo build --quiet --release)
  budget_run scale_200 configs/scale_200/scenario.toml "$out/scale_200"
  budget_run pittsburgh configs/parts_logistics_pittsburgh/scenario.toml "$out/pittsburgh"
  budget_run pittsburgh_placeholder configs/parts_logistics_pittsburgh_placeholder/scenario.toml \
    "$out/pittsburgh_placeholder"
  budget_run pittsburgh_battery_1370wh configs/parts_logistics_pittsburgh_battery_1370wh/scenario.toml \
    "$out/pittsburgh_battery_1370wh"
  budget_run pittsburgh_1370wh_layout configs/parts_logistics_pittsburgh_1370wh_layout/scenario.toml \
    "$out/pittsburgh_1370wh_layout"
  budget_run pittsburgh_900wh_layout configs/parts_logistics_pittsburgh_900wh_layout/scenario.toml \
    "$out/pittsburgh_900wh_layout"
  budget_run pittsburgh_demo configs/parts_logistics_pittsburgh_demo/scenario.toml \
    "$out/pittsburgh_demo"
  budget_run pittsburgh_vtol configs/parts_logistics_pittsburgh_vtol/scenario.toml "$out/pittsburgh_vtol"
  budget_run pittsburgh_vtol_layout configs/parts_logistics_pittsburgh_vtol_layout/scenario.toml \
    "$out/pittsburgh_vtol_layout"
  budget_run pittsburgh_vtol_demo configs/parts_logistics_pittsburgh_vtol_demo/scenario.toml \
    "$out/pittsburgh_vtol_demo"
  python3 - "$out/pittsburgh_vtol_demo/metrics.json" <<'PY'
import json, sys
o = json.load(open(sys.argv[1]))["order_outcomes"]
classes = ["delivered_drone", "delivered_truck", "rejected_cap", "blocked_weather", "blocked_energy",
           "in_flight", "waiting", "diverted_or_failed"]
total = sum(o[c] for c in classes)
print(f"pittsburgh_vtol_demo: {total} of {o['requests']} requests in an outcome class, "
      f"completion {o['completion_rate']:.1%}")
sys.exit(0 if total == o["requests"] else "FAIL: order outcomes do not account for every request")
PY
  triple="$(python3 scripts/scale_demand.py configs/parts_logistics_pittsburgh_placeholder/scenario.toml 3 "$out/x3")"
  budget_run pittsburgh_3x "$triple" "$out/pittsburgh_3x"
  python3 - "$out/pittsburgh_3x/metrics.json" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
rejected, requests = m["requests_rejected"], m["shipment_requests_in_window"]
print(f"pittsburgh_3x: {rejected} of {requests} requests rejected at the backlog cap")
sys.exit(0 if rejected > 0 else "FAIL: 3x demand should overflow the backlog cap")
PY

  echo "== VTOL fit reruns identically"
  pipeline/run.sh calib vtol_fit > /dev/null
  git diff --exit-code --stat -- pipeline/calib/vtol_fitted_params.json pipeline/calib/out/vtol_fit.json \
    || { echo "FAIL: the VTOL fit changed on rerun"; exit 1; }
fi
echo "verify: all checks passed"
