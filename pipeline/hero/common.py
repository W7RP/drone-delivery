"""Paths and readers shared by the hero steps."""

import csv
import subprocess
import tomllib
from pathlib import Path

import yaml

from ddpipe import binfmt
from ddpipe.paths import CACHE, PIPELINE, RAW, REPO

HERO = PIPELINE / "hero"
CONFIG = HERO / "hero.yaml"
OUT = HERO / "out"
DEMO = REPO / "configs" / "parts_logistics_pittsburgh_vtol_demo"
SIM = REPO / "sim" / "target" / "release" / "sim"
OSM_FILE = "pennsylvania-latest.osm.pbf"
__all__ = ["CACHE", "RAW", "REPO", "HERO", "CONFIG", "OUT", "DEMO", "SIM", "OSM_FILE"]


def config() -> dict:
    return yaml.safe_load(CONFIG.read_text())


def demo_network() -> tuple[Path, list[dict], binfmt.LaneGraph, binfmt.TruckFile]:
    """The demo scenario's derived directory, pads (pads.toml order = pad id), lanes and truck
    matrix."""
    scenario = tomllib.loads((DEMO / "scenario.toml").read_text())
    derived = (DEMO / scenario["network_files"]["dir"]).resolve()
    pads = tomllib.loads((derived / "pads.toml").read_text())["pads"]
    lanes, _ = binfmt.read_lanes((derived / "lanes.csr.bin").read_bytes())
    truck, _ = binfmt.read_truck((derived / "truck_matrix.bin").read_bytes())
    return derived, pads, lanes, truck


def lane_index(lanes: binfmt.LaneGraph, a: int, b: int) -> int | None:
    lo, hi = int(lanes.row_offsets[a]), int(lanes.row_offsets[a + 1])
    for i in range(lo, hi):
        if int(lanes.to_pad[i]) == b:
            return i
    return None


def run_demo(seed: int, out: Path) -> list[dict]:
    """Run the demo scenario (no event log) and return its deliveries.csv rows."""
    done = subprocess.run([str(SIM), "run", str(DEMO / "scenario.toml"), "--seed", str(seed),
                           "--out", str(out), "--no-event-log"], capture_output=True, text=True,
                          timeout=120)
    if done.returncode != 0:
        raise RuntimeError(f"demo seed {seed}: {done.stderr.strip()}")
    with open(out / "deliveries.csv") as f:
        return list(csv.DictReader(f))
