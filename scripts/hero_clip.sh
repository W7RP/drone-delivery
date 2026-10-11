#!/usr/bin/env bash
# Phase 8: regenerate the corridor world and the hero clip offline from the pipeline cache, in
# one command (docs/hero-clip.md). Picks the lane, builds the world, computes the drone-vs-truck
# panel, flies the camera visit in PX4 SITL at 1x, validates and checks it, cuts the clip and
# tests its HUD against the logs.
#   scripts/hero_clip.sh [--seed N] [--run DIR] [--video FILE]
# Defaults: seed 1, run px4/runs/hero/clip_<UTC time>, video video/hero_clip.mp4.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
seed=1 run="" video="$ROOT/video/hero_clip.mp4"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --seed) seed="$2"; shift ;;
    --run) run="$2"; shift ;;
    --video) video="$2"; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done
[[ -n "$run" ]] || run="$ROOT/px4/runs/hero/clip_$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$(dirname "$run")" "$(dirname "$video")"
run="$(cd "$(dirname "$run")" && pwd)/$(basename "$run")"
video="$(cd "$(dirname "$video")" && pwd)/$(basename "$video")"
echo "free disk before: $(df -BG --output=avail "$ROOT" | tail -1 | tr -d ' ')"

"$ROOT/pipeline/run.sh" hero lane
"$ROOT/pipeline/run.sh" hero world > /dev/null
"$ROOT/pipeline/run.sh" hero panel > /dev/null
"$ROOT/px4/tools/build_gz_systems.sh" > /dev/null
"$ROOT/px4/run_flight.sh" --airframe vtol --payload-kg 5 --seed "$seed" --world-dir "$ROOT/px4/sim/hero" \
  --cameras --out "$run"
# The world this run flew, for comparing regenerations.
cp "$ROOT/pipeline/hero/out/world_manifest.json" "$run/world_manifest.json"
"$ROOT/px4/py.sh" tools/flightlog.py validate "$run"
"$ROOT/px4/py.sh" tools/hero_check.py "$run" --out "$run/hero_check.json" > /dev/null
"$ROOT/px4/py.sh" tools/hero_edit.py --run "$run" --out "$video"
"$ROOT/px4/py.sh" tests/test_hero_hud.py "$run"
echo "clip: $video ($(du -h "$video" | cut -f1))"
echo "peak disk of the camera streams: $(cat "$run/video_bytes") bytes"
echo "free disk after: $(df -BG --output=avail "$ROOT" | tail -1 | tr -d ' ')"
