#!/usr/bin/env bash
# Phase 4 step 1: render a downward 320x240 camera at 10 Hz in a headless Gazebo server and check
# the frames (tools/camera_smoke.py). Set LIBGL_ALWAYS_SOFTWARE=1 to try software rendering.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STARTUP_S=10

if pgrep -f '^gz sim' >/dev/null; then
  echo "a Gazebo server is already running; stop it first: pkill -KILL -f '^gz sim'" >&2
  exit 1
fi
gz sim -s -r --headless-rendering "$HERE/sim/smoke/camera_smoke.sdf" >/dev/null 2>&1 &
gz_pid=$!
cleanup() {
  kill "$gz_pid" 2>/dev/null || true
  # gz servers have been seen ignoring SIGTERM; make sure none survives.
  sleep 1
  pkill -KILL -f '^gz sim -s -r --headless-rendering .*camera_smoke.sdf' 2>/dev/null || true
}
trap cleanup EXIT
sleep "$STARTUP_S"
python3 "$HERE/tools/camera_smoke.py"
