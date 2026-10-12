#!/usr/bin/env bash
# The Phase 4 acceptance runs, one after another, into px4/runs/<batch>/:
#   payload_<kg>   payload 0, 2, 5 and 10 kg, seed 100
#   repeat_<n>     5 kg, seeds 1..10
#   speed2_seed1   5 kg, seed 1, at 2x real time (compare with repeat_1)
# then validates every run and writes the summary (tools/summarize.py).
# Usage: tools/run_batch.sh <batch name> [run names...]   (default: all of the above)
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
batch="$HERE/runs/$1"; shift
mkdir -p "$batch"
runs=("$@")
if [[ ${#runs[@]} -eq 0 ]]; then
  runs=(payload_0 payload_2 payload_5 payload_10)
  for n in $(seq 1 10); do runs+=("repeat_$n"); done
  runs+=(speed2_seed1)
fi
for name in "${runs[@]}"; do
  case "$name" in
    payload_*) args=(--payload-kg "${name#payload_}" --seed 100) ;;
    repeat_*) args=(--payload-kg 5 --seed "${name#repeat_}") ;;
    speed2_seed1) args=(--payload-kg 5 --seed 1 --speed 2) ;;
    *) echo "unknown run $name" >&2; exit 2 ;;
  esac
  rm -rf "${batch:?}/$name"
  "$HERE/run_flight.sh" "${args[@]}" --out "$batch/$name" 2>&1 | grep -v "Segmentation fault\|Killed" | tail -1
done
"$HERE/py.sh" tools/flightlog.py validate "$batch"/*/ | tee "$batch/validation.txt"
"$HERE/py.sh" tools/summarize.py "$batch"
