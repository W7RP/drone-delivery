#!/usr/bin/env bash
# Whole verification suite; exits non-zero on the first failure.
#  1. Spec, schemas and configs agree (check_spec.py, via uv; pinned deps in uv's cache only).
#  2. Offline pipeline unit tests (pipeline/run.sh test; creates pipeline/.venv on first use).
#  3. Rust core: formatting, clippy with warnings as errors, all tests (debug build, so the
#     invariants run after every event). Includes loading the committed Pittsburgh network files.
#  4. Release budget runs (spec A39: 60 s wall, 200 MB peak RSS each): the 200-drone 7-day scale
#     case, the 7-day Pittsburgh scenario, and Pittsburgh at three times its demand, which must
#     turn requests away at the backlog cap instead of queueing them without limit.
#     Skip with VERIFY_SKIP_SCALE=1.
set -euo pipefail
cd "$(dirname "$0")/.."
[ -d "$HOME/.cargo/bin" ] && PATH="$HOME/.cargo/bin:$PATH"

echo "== spec and schema checks"
uv run --quiet scripts/check_spec.py

echo "== pipeline unit tests"
pipeline/run.sh test 2>&1 | tail -n 3

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
  triple="$(python3 scripts/scale_demand.py configs/parts_logistics_pittsburgh/scenario.toml 3 "$out/x3")"
  budget_run pittsburgh_3x "$triple" "$out/pittsburgh_3x"
  python3 - "$out/pittsburgh_3x/metrics.json" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
rejected, requests = m["requests_rejected"], m["shipment_requests_in_window"]
print(f"pittsburgh_3x: {rejected} of {requests} requests rejected at the backlog cap")
sys.exit(0 if rejected > 0 else "FAIL: 3x demand should overflow the backlog cap")
PY
fi
echo "verify: all checks passed"
