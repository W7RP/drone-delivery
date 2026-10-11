#!/usr/bin/env bash
# Phase 8 step 1: render three 1280x720 cameras at 30 Hz sim time plus the 320x240 marker camera
# in a headless Gazebo server, stream the three to H.264 (tools/frame_recorder.py) and report
# frames per wall second, the achieved real-time factor and any missing sim-time frames.
#   px4/tools/render_check.sh <out dir> [wall seconds, default 60]
# Set LIBGL_ALWAYS_SOFTWARE=1 to force software rendering.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out="$1"; seconds="${2:-60}"
mkdir -p "$out"; out="$(cd "$out" && pwd)"
STARTUP_S=15

if pgrep -f '^gz sim' >/dev/null; then
  echo "a Gazebo server is already running; stop it first: pkill -KILL -f '^gz sim'" >&2
  exit 1
fi
python3 "$HERE/tools/render_check_world.py" "$out/render_check.sdf"
ffmpeg_exe="$("$HERE/py.sh" tools/ffmpeg_path.py)"
gz sim --verbose=3 -s -r --headless-rendering "$out/render_check.sdf" > "$out/gz_server.log" 2>&1 &
gz_pid=$!
cleanup() {
  kill "$gz_pid" 2>/dev/null || true
  sleep 1
  pkill -KILL -f '^gz sim .*render_check.sdf' 2>/dev/null || true
}
trap cleanup EXIT
sleep "$STARTUP_S"
python3 "$HERE/tools/frame_recorder.py" --out "$out" --ffmpeg "$ffmpeg_exe" --seconds "$seconds" \
  --camera chase=/render_check/chase --camera pad_close=/render_check/pad_close \
  --camera high_wide=/render_check/high_wide --camera marker_down=/render_check/marker_down
