"""Shared helpers for the Phase 6a analysis scripts: read a committed scenario, change keys in a
copy, write the copy where `sim run` can load it, and run the release sim.

A copy keeps its scenario's network and flight profile: `network_files.dir` and
`flight_profile.file` are rewritten as absolute paths, so the copy can live anywhere. The copy's
directory is named after its `meta.name` (spec R14). Committed scenario files are never edited.
"""

import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import tomli
import tomli_w

ROOT = Path(__file__).resolve().parent.parent
CONFIGS = ROOT / "configs"
SIM = ROOT / "sim" / "target" / "release" / "sim"
OUT = ROOT / "analysis" / "out"
J_PER_WH = 3600.0
# A sweep refuses to start with less free space than this on the repo drive or the temp drive.
MIN_FREE_BYTES = 20 * 1024**3


def load(name: str) -> dict:
    return tomli.loads((CONFIGS / name / "scenario.toml").read_text())


def absolute_paths(cfg: dict, source_dir: Path) -> dict:
    if "network_files" in cfg:
        cfg["network_files"]["dir"] = str((source_dir / cfg["network_files"]["dir"]).resolve())
    if "flight_profile" in cfg:
        cfg["flight_profile"]["file"] = str((source_dir / cfg["flight_profile"]["file"]).resolve())
    return cfg


def quantity(value, unit: str) -> dict:
    return {"value": value, "unit": unit, "assumption": True}


def set_value(cfg: dict, path: str, value) -> None:
    """Set the `value` of the quantity at a dotted path; a dict replaces the whole entry and
    None removes it."""
    node = cfg
    *parents, last = path.split(".")
    for key in parents:
        node = node.setdefault(key, {})
    if value is None:
        node.pop(last, None)
    elif isinstance(value, dict) or last not in node or not isinstance(node[last], dict):
        node[last] = value
    else:
        node[last]["value"] = value


def write(cfg: dict, root: Path, name: str, description: str) -> Path:
    cfg["meta"]["name"] = name
    cfg["meta"]["description"] = description
    path = root / name / "scenario.toml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(tomli_w.dumps(cfg))
    return path


def variant(source: str, root: Path, name: str, edits: dict, description: str = "") -> Path:
    """A copy of committed scenario `source` with `edits` ({dotted path: value}) applied."""
    cfg = absolute_paths(load(source), CONFIGS / source)
    for path, value in edits.items():
        set_value(cfg, path, value)
    return write(cfg, root, name, description or f"{source} with {sorted(edits)}")


def run(scenario: Path, seed: int, out: Path) -> dict:
    """`sim run` in release, without an event log; returns metrics.json."""
    subprocess.run(
        [str(SIM), "run", str(scenario), "--seed", str(seed), "--out", str(out), "--no-event-log"],
        check=True,
        capture_output=True,
    )
    return json.loads((out / "metrics.json").read_text())


def sim_json(*args: str) -> dict:
    done = subprocess.run([str(SIM), *args], check=True, capture_output=True, text=True)
    return json.loads(done.stdout)


def require_free_disk() -> None:
    """Stop before a sweep can fill a disk (Phase 6a's runaway runs once did)."""
    for where in (ROOT, Path(tempfile.gettempdir())):
        free = shutil.disk_usage(where).free
        if free < MIN_FREE_BYTES:
            raise SystemExit(f"{where}: {free / 1024**3:.1f} GB free, a sweep needs 20 GB")


def build_sim() -> None:
    require_free_disk()
    subprocess.run(["cargo", "build", "--quiet", "--release"], cwd=ROOT / "sim", check=True)
