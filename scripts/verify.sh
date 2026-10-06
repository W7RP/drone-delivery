#!/usr/bin/env bash
# Whole verification suite; exits non-zero on the first failure.
#  1. Spec, schemas and configs agree (check_spec.py, via uv; pinned deps in uv's cache only).
#  2. Rust core: formatting, clippy with warnings as errors, all tests (debug build, so the
#     invariants run after every event).
#  3. Scale budget (spec A39): the 200-drone, 7-day case in release within 60 s and 200 MB.
#     Skip with VERIFY_SKIP_SCALE=1.
set -euo pipefail
cd "$(dirname "$0")/.."
[ -d "$HOME/.cargo/bin" ] && PATH="$HOME/.cargo/bin:$PATH"

echo "== spec and schema checks"
uv run --quiet scripts/check_spec.py

echo "== rust: fmt, clippy, tests"
(cd sim && cargo fmt --check && cargo clippy --quiet --all-targets -- -D warnings && cargo test --quiet)

if [ "${VERIFY_SKIP_SCALE:-0}" != "1" ]; then
  echo "== scale budget"
  out="$(mktemp -d)"
  trap 'rm -rf "$out"' EXIT
  (cd sim && cargo build --quiet --release)
  /usr/bin/time -v sim/target/release/sim run configs/scale_200/scenario.toml --out "$out" 2> "$out/time.txt"
  wall="$(awk -F': ' '/Elapsed \(wall clock\)/ {print $2}' "$out/time.txt")"
  rss_kb="$(awk -F': ' '/Maximum resident set size/ {print $2}' "$out/time.txt")"
  secs="$(echo "$wall" | awk -F: '{ if (NF == 3) print $1*3600 + $2*60 + $3; else print $1*60 + $2 }')"
  echo "wall ${secs} s, peak RSS $((rss_kb / 1024)) MB (budget 60 s, 200 MB)"
  awk -v s="$secs" 'BEGIN { exit !(s < 60) }' || { echo "FAIL: wall time over budget"; exit 1; }
  [ "$rss_kb" -lt $((200 * 1024)) ] || { echo "FAIL: peak RSS over budget"; exit 1; }
fi
echo "verify: all checks passed"
