#!/usr/bin/env bash
# Offline data pipeline, one command: Python environment, fetch (or reuse the cache), build the
# Pittsburgh network files and scenario, sanity-check them. Arguments go to `python -m ddpipe`;
# `run.sh test` runs the pipeline's unit tests instead.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV="$HERE/.venv"
PYTHON_VERSION="3.12"
STAMP="$VENV/.requirements.sha256"

if ! command -v uv >/dev/null; then
  echo "uv is required (https://docs.astral.sh/uv/); it creates the venv without root" >&2
  exit 1
fi
if [[ ! -x "$VENV/bin/python" ]]; then
  uv venv --quiet --python "$PYTHON_VERSION" "$VENV"
fi
want="$(sha256sum "$HERE/requirements.txt" | cut -d' ' -f1)"
if [[ "$(cat "$STAMP" 2>/dev/null)" != "$want" ]]; then
  VIRTUAL_ENV="$VENV" uv pip install --quiet --only-binary :all: --require-hashes \
    -r "$HERE/requirements.txt"
  echo "$want" > "$STAMP"
fi

# The package lives in pipeline/; downloaded files stay under pipeline/data/, off sys.path.
cd "$HERE"
if [[ "${1:-}" == "test" ]]; then
  exec "$VENV/bin/python" -m unittest discover -s tests -t . "${@:2}"
fi
exec "$VENV/bin/python" -m ddpipe "$@"
