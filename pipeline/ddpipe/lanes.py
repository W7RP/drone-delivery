"""Step 3: lanes between pads. Candidates are all pad pairs within the drone's range at the
lane payload; a lane is rejected if its straight path crosses an airspace exclusion zone or if
the sim's dispatch guard (R12) would refuse it. Kept lanes get length, highest terrain and a
ground-risk score along the path (sampling method in spec section 10, A49)."""

import logging
from dataclasses import dataclass

import numpy as np
import shapely

from . import energy
from .geo import great_circle_m
from .pads import Pad
from .params import q

log = logging.getLogger(__name__)


@dataclass
class Lane:
    a: int
    b: int
    length_m: float
    max_terrain_m: float
    ground_risk_per_km2: float


@dataclass
class LaneRejection:
    a: str
    b: str
    length_m: float
    reason: str


def lane_length_m(pa: Pad, pb: Pad) -> float:
    return float(great_circle_m(pa.lat_deg, pa.lon_deg, pb.lat_deg, pb.lon_deg))


def sample_path(pa: Pad, pb: Pad, spacing_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Points every `spacing_m` or closer along the straight path, both ends included. Linear
    in lon/lat; over these distances that departs from the great circle by centimetres."""
    n = max(2, int(np.ceil(lane_length_m(pa, pb) / spacing_m)) + 1)
    t = np.linspace(0.0, 1.0, n)
    return pa.lon_deg + t * (pb.lon_deg - pa.lon_deg), pa.lat_deg + t * (pb.lat_deg - pa.lat_deg)


def _energy_filter(pads, kept, model, payload_kg, has_charger, rejections):
    """Drop lanes the sim's R12 check would refuse, until none fails. Removing a lane can only
    lengthen routes, so this ends."""
    while True:
        route = energy.routes(len(pads), [(l.a, l.b, l.length_m) for l in kept])
        usable = model.usable_j()
        failing = []
        for lane in kept:
            needs = [energy.dispatch_need_j(model, route, a, b, payload_kg, has_charger)
                     for a, b in ((lane.a, lane.b), (lane.b, lane.a))]
            if max(needs) > usable:
                failing.append((lane, max(needs)))
        if not failing:
            return kept
        # Remove the worst lane only; others may pass once routes are recomputed.
        lane, need = max(failing, key=lambda f: (f[1], f[0].a, f[0].b))
        kept.remove(lane)
        rejections.append(LaneRejection(
            pads[lane.a].name, pads[lane.b].name, lane.length_m,
            f"energy guard (R12): needs {need:.0f} J above reserve, battery gives {usable:.0f} J"))


def _connected(n: int, lanes: list[Lane]) -> bool:
    route = energy.routes(n, [(l.a, l.b, l.length_m) for l in lanes])
    return bool(np.all(np.isfinite(route[0])))


def build(pads: list[Pad], cons, model: energy.FlightModel, params: dict, terrain, population):
    payload_kg = q(params["lanes"]["payload_kg"], "kg")
    spacing_m = q(params["lanes"]["sample_spacing_m"], "m")
    max_range_m = model.max_lane_m(payload_kg)
    log.info("lane range at %.1f kg payload: %.0f m", payload_kg, max_range_m)
    rejections: list[LaneRejection] = []
    kept: list[Lane] = []
    for i in range(len(pads)):
        for j in range(i + 1, len(pads)):
            pa, pb = pads[i], pads[j]
            length = lane_length_m(pa, pb)
            if length > max_range_m:
                rejections.append(LaneRejection(pa.name, pb.name, length,
                                                f"beyond range {max_range_m:.0f} m"))
                continue
            line = shapely.LineString([(pa.lon_deg, pa.lat_deg), (pb.lon_deg, pb.lat_deg)])
            crossed = cons.cells_crossed(line)
            if crossed:
                airports = sorted({c.airport for c in crossed})
                lowest = min(c.ceiling_ft for c in crossed)
                rejections.append(LaneRejection(
                    pa.name, pb.name, length,
                    f"crosses {len(crossed)} exclusion cell(s) of {', '.join(airports)} "
                    f"(lowest ceiling {lowest:.0f} ft)"))
                continue
            kept.append(Lane(i, j, length, 0.0, 0.0))
    has_charger = (lambda p: True) if model.out_contacts else (lambda p: pads[p].is_hub)
    kept = _energy_filter(pads, kept, model, payload_kg, has_charger, rejections)
    if not _connected(len(pads), kept):
        raise RuntimeError("lane graph is not connected after rejections (R3)")
    for lane in kept:
        lon, lat = sample_path(pads[lane.a], pads[lane.b], spacing_m)
        lane.max_terrain_m = float(terrain.sample(lon, lat).max())
        on_water = shapely.contains_xy(cons.water, lon, lat)
        lane.ground_risk_per_km2 = float(np.where(on_water, 0.0, population.density(lon, lat)).mean())
    for lane in kept:
        log.info("lane %s - %s: %.0f m, terrain max %.0f m, ground risk %.0f /km2",
                 pads[lane.a].name, pads[lane.b].name, lane.length_m, lane.max_terrain_m,
                 lane.ground_risk_per_km2)
    for r in rejections:
        log.info("lane rejected: %s - %s (%.0f m): %s", r.a, r.b, r.length_m, r.reason)
    log.info("lanes: %d kept, %d rejected", len(kept), len(rejections))
    return kept, rejections, max_range_m
