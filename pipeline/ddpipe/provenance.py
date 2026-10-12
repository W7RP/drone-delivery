"""Which pipeline code produced a file: the git commit, marked dirty when the pipeline's own
sources differ from it."""

import subprocess

from .paths import PIPELINE, REPO

# Inputs that change what the pipeline produces. Outputs and downloads are excluded.
_CODE_PATHS = ["ddpipe", "sources.yaml", "sites.yaml", "params.yaml", "scenario.toml.in"]


def _git(*args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(REPO), *args], check=True, capture_output=True, text=True
    ).stdout.strip()


def pipeline_revision() -> str:
    head = _git("rev-parse", "--short=12", "HEAD")
    changes = _git("status", "--porcelain", "--", *[str(PIPELINE / p) for p in _CODE_PATHS])
    return f"{head}-dirty" if changes else head
