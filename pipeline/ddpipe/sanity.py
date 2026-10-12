"""Acceptance checks on the files in derived/, read back from disk: pads clear of water and
exclusion zones, lanes no shorter than the straight line, road times no faster than the
straight line at the top truck speed, a finite truck matrix, and the spec 9 reader rules.
Raises on the first failed group, after logging every offending item in it."""

import json
from pathlib import Path
import logging
import tomllib

import numpy as np
import shapely

from . import binfmt, constraints
from .geo import great_circle_m
from .outputs import LANES_FILE, PADS_FILE, REPORT_FILE, TRUCK_FILE
from .truck import MAX_PLAUSIBLE_SPEED_MPS

log = logging.getLogger(__name__)


class SanityError(RuntimeError):
    pass


def _expect(problems: list[str], what: str) -> None:
    for p in problems:
        log.error("sanity: %s: %s", what, p)
    if problems:
        raise SanityError(f"{what}: {len(problems)} problem(s)")
    log.info("sanity ok: %s", what)


def check(params: dict, water_polygons: list, derived: Path) -> None:
    catalog = (derived / PADS_FILE).read_bytes()
    pads = tomllib.loads(catalog.decode())["pads"]
    names = [p["name"] for p in pads]
    lat = np.array([p["lat_deg"]["value"] for p in pads])
    lon = np.array([p["lon_deg"]["value"] for p in pads])
    n = len(pads)
    cons = constraints.build(params, water_polygons)

    problems = []
    for name, x, y in zip(names, lon, lat):
        if shapely.contains_xy(cons.water, x, y):
            problems.append(f"{name} is in water")
        if shapely.contains_xy(cons.exclusion, x, y):
            problems.append(f"{name} is in an airspace exclusion zone")
    _expect(problems, "pads clear of water and exclusion zones")

    expected_hash = binfmt.fnv1a64(catalog)
    lanes, lanes_hash = binfmt.read_lanes((derived / LANES_FILE).read_bytes())
    truck_data = (derived / TRUCK_FILE).read_bytes()
    problems = []
    for fname, h in ((LANES_FILE, lanes_hash),):
        if h != expected_hash:
            problems.append(f"{fname} source_hash {h:016x} != pads.toml {expected_hash:016x}")
    if lanes.n_pads != n:
        problems.append(f"{LANES_FILE} has {lanes.n_pads} pads, catalog has {n}")
    _expect(problems, "lane graph valid per spec 9.1 and keyed to the catalog")

    frm, to = lanes.from_pad, lanes.to_pad.astype(np.int64)
    straight = great_circle_m(lat[frm], lon[frm], lat[to], lon[to])
    problems = [f"{names[a]} -> {names[b]}: {l} m < straight {s:.3f} m"
                for a, b, l, s in zip(frm, to, lanes.length_m, straight) if float(l) < s]
    _expect(problems, "every lane length >= straight-line distance")

    # Read the raw matrix first so unreachable pairs are listed by name, not just rejected.
    _, _, count_a, _, truck_hash = binfmt._header(truck_data, binfmt.TRUCK_MAGIC,
                                                    binfmt.TRUCK_MINOR)
    problems = []
    if count_a != n:
        problems.append(f"{TRUCK_FILE} has {count_a} pads, catalog has {n}")
    if truck_hash != expected_hash:
        problems.append(f"{TRUCK_FILE} source_hash {truck_hash:016x} != pads.toml")
    if not problems:
        raw = np.frombuffer(truck_data, dtype="<f4", count=2 * n * n, offset=binfmt.HEADER_BYTES)
        for k, (i, j) in enumerate((i, j) for i in range(n) for j in range(n)):
            for label, v in (("free_flow_s", raw[k]), ("road_m", raw[n * n + k])):
                if i != j and not np.isfinite(v):
                    problems.append(f"{label} {names[i]} -> {names[j]} unreachable ({v})")
    _expect(problems, "truck matrix finite for every pad pair")
    truck, _ = binfmt.read_truck(truck_data)
    free_flow, road = truck.free_flow_s, truck.road_m
    _expect([] if truck.class_frac is not None else ["no road-class fractions"],
            "truck matrix has road-class and bridge/tunnel fractions (9.2 version_minor 2)")

    max_speed = json.loads((derived / REPORT_FILE).read_text())["truck"]["max_speed_mps"]
    problems = []
    if not 0 < max_speed <= MAX_PLAUSIBLE_SPEED_MPS:
        problems.append(f"max truck speed {max_speed} m/s implausible")
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            s = float(great_circle_m(lat[i], lon[i], lat[j], lon[j]))
            if free_flow[i, j] < s / max_speed:
                problems.append(f"{names[i]} -> {names[j]}: {free_flow[i, j]} s < "
                                f"{s:.0f} m / {max_speed} m/s")
            if road[i, j] < s:
                problems.append(f"{names[i]} -> {names[j]}: road {road[i, j]} m < straight {s:.0f} m")
    _expect(problems, "every road time >= straight-line distance / max truck speed")
