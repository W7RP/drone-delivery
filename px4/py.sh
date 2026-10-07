#!/usr/bin/env bash
# Run a px4/tools Python script in px4/.venv (Python 3.12, pinned hashed wheels), creating the
# venv on first use. Usage: px4/py.sh tools/<script>.py [args]
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/.venv"
STAMP="$VENV/.requirements.sha256"
command -v uv >/dev/null || { echo "uv is required (https://docs.astral.sh/uv/)" >&2; exit 1; }
[[ -x "$VENV/bin/python" ]] || uv venv --quiet --python 3.12 "$VENV"
want="$(sha256sum "$HERE/requirements.txt" | cut -d' ' -f1)"
if [[ "$(cat "$STAMP" 2>/dev/null)" != "$want" ]]; then
  VIRTUAL_ENV="$VENV" uv pip install --quiet --only-binary :all: --require-hashes -r "$HERE/requirements.txt"
  echo "$want" > "$STAMP"
fi
script="$1"; shift
# -I: ignore PYTHONPATH (ROS sources put Python 3.10 packages on it) and the user site.
exec "$VENV/bin/python" -I "$HERE/$script" "$@"
