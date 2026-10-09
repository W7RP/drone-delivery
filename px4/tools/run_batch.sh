#!/usr/bin/env bash
# The Phase 4 acceptance runs, one after another, into px4/runs/<batch>/:
#   payload_<kg>   payload 0, 2, 5 and 10 kg, seed 100
#   repeat_<n>     5 kg, seeds 1..10
#   speed2_seed1   5 kg, seed 1, at 2x real time (compare with repeat_1)
# and, only when named, the Phase 5 held-out flights (5 kg, 2x real time, seeds 1 and 2):
#   heldout_short_<seed>  lane 14, UPMC Oakland to UPMC Magee-Womens, 441 m (shortest legal lane)
#   heldout_long_<seed>   lane 4, Strip District hub to Edgar Thomson, 12 896 m (longest)
#   fitlane_<seed>        the fit lane (3) at 5 kg and 2x, to check a SITL change against the fit
# and the Phase 7 VTOL runs (airframe vtol):
#   vtol_<kg>_<seed>      the fit lane (3), payload <kg>, at 2x real time (calibration)
#   vtol1x_<kg>_<seed>    the same at 1x (speed-factor check)
#   vtolheld_<lane>_<seed> 5 kg on catalog lane <lane> at 2x (held-out validation)
#   vtolwind_<m/s>_<seed>  5 kg on the fit lane at 2x in a crosswind of <m/s>, gusts 50% (A94)
# then validates every run and writes the summary (tools/summarize.py).
# Usage: tools/run_batch.sh <batch name> [run names...]   (default: all of the above)
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The fit lane (3) runs on a bearing of 112 deg (ENU 1938 m east, 791 m south); a wind from
# 22 deg blows square across it from the left.
CROSSWIND_FROM_DEG=22
GUST_FRAC=0.5
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
    heldout_short_*) args=(--payload-kg 5 --seed "${name##*_}" --speed 2 --lane-id 14) ;;
    heldout_long_*) args=(--payload-kg 5 --seed "${name##*_}" --speed 2 --lane-id 4) ;;
    fitlane_*) args=(--payload-kg 5 --seed "${name##*_}" --speed 2) ;;
    vtol_*) IFS=_ read -r _ kg seed <<< "$name"
      args=(--airframe vtol --payload-kg "$kg" --seed "$seed" --speed 2) ;;
    vtol1x_*) IFS=_ read -r _ kg seed <<< "$name"
      args=(--airframe vtol --payload-kg "$kg" --seed "$seed") ;;
    vtolheld_*) IFS=_ read -r _ lane seed <<< "$name"
      args=(--airframe vtol --payload-kg 5 --seed "$seed" --speed 2 --lane-id "$lane") ;;
    vtolwind_*) IFS=_ read -r _ mps seed <<< "$name"
      args=(--airframe vtol --payload-kg 5 --seed "$seed" --speed 2 --wind-mps "$mps"
            --wind-from-deg "$CROSSWIND_FROM_DEG" --gust-frac "$GUST_FRAC") ;;
    *) echo "unknown run $name" >&2; exit 2 ;;
  esac
  rm -rf "${batch:?}/$name"
  "$HERE/run_flight.sh" "${args[@]}" --out "$batch/$name" 2>&1 | grep -v "Segmentation fault\|Killed" | tail -1
done
"$HERE/py.sh" tools/flightlog.py validate "$batch"/*/ | tee "$batch/validation.txt"
if ls "$batch"/repeat_* > /dev/null 2>&1; then
  "$HERE/py.sh" tools/summarize.py "$batch"
fi
