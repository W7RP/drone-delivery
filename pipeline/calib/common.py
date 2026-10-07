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

SIM = REPO / "sim" / "target" / "release" / "sim"
# The placeholder profile's Pittsburgh scenario: the model before calibration, and the base
# scenario_with() substitutes profile values into (it selects no flight_profile of its own).
PITTSBURGH = REPO / "configs" / "parts_logistics_pittsburgh_placeholder" / "scenario.toml"
PHASE_KEYS = {"takeoff": "takeoff", "cruise": "cruise", "approach_landing": "approach", "hop": "hop"}


def sim_flight_model(lane_m: float, payload_kg: float, config: Path = PITTSBURGH) -> dict:
    """The core's own per-phase estimate (`sim flight-model`), so nothing here reimplements it."""
    import subprocess
    if not SIM.exists():
        raise RuntimeError(f"{SIM} missing: cd sim && cargo build --release")
    out = subprocess.run([str(SIM), "flight-model", "--lane-m", repr(lane_m), "--payload-kg",
                          repr(payload_kg), "--config", str(config)],
                         capture_output=True, text=True, check=True).stdout
    return json.loads(out)

# The flight-model keys a profile sets (spec 2.7 and the drone timing keys of 8.2).
PROFILE_KEYS = {"cruise_speed_mps": "m/s", "cruise_power_ref_w": "W", "hover_power_ref_w": "W",
                "power_ref_mass_kg": "kg", "takeoff_s": "s", "approach_s": "s", "hop_s": "s"}


def scenario_with(values: dict, tag: str) -> Path:
    """A copy of the Pittsburgh scenario with the given drone flight-model values, for
    `sim flight-model`: out/models/<tag>/scenario.toml, network files by absolute path."""
    import re
    text = PITTSBURGH.read_text()
    for key, value in values.items():
        unit = PROFILE_KEYS[key]
        text, n = re.subn(rf"^{key} = \{{ value = [^,]+, unit = \"{re.escape(unit)}\"",
                          f'{key} = {{ value = {float(value)!r}, unit = "{unit}"', text, flags=re.M)
        if n != 1:
            raise ValueError(f"{key} not found once in {PITTSBURGH}")
    text = text.replace('dir = "derived"', f'dir = "{PITTSBURGH.parent / "derived"}"')
    path = OUT / "models" / tag / "scenario.toml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path
