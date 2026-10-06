#!/usr/bin/env bash
# Phase-1 verification: schema validation, config rules, spec/schema cross-checks.
# Needs uv; dependencies are pinned in check_spec.py and installed into uv's cache only.
set -euo pipefail
cd "$(dirname "$0")/.."
uv run --quiet scripts/check_spec.py
