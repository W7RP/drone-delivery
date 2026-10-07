"""Shared by the calibration scripts: where the SITL logs live and how to read them. The px4/
tools reader (flightlog.py) is the one source for the log formats; it is imported, not copied."""

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "px4" / "tools"))
import flightlog  # noqa: E402

RUNS = REPO / "px4" / "runs"
FIT_BATCH = RUNS / "accept"
CALIB = REPO / "pipeline" / "calib"
OUT = CALIB / "out"
GRAVITY = 9.80665


def fit_runs() -> list[Path]:
    """The Phase 4 acceptance runs: the training data of the fit (validated, COMPLETE)."""
    runs = sorted(p for p in FIT_BATCH.iterdir() if (p / "run.json").exists())
    for r in runs:
        problems = flightlog.check(r)
        if problems:
            raise RuntimeError(f"{r.name} fails validation: {problems}")
    return runs


def run_meta(run: Path) -> dict:
    meta = json.loads((run / "run.json").read_text())
    meta.update(json.loads((run / "vehicle.json").read_text()))
    meta["name"] = run.name
    return meta
