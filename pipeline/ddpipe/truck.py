"""Step 4: the truck travel matrix (spec 9.2) over the OSM drive network. Free-flow speed per
way comes from `maxspeed`, else a per-class default (params.yaml). One-way rules follow OSM:
`oneway=yes|true|1`, `oneway=-1|reverse`, roundabouts and motorways are one-way unless
`oneway=no`. Each pad snaps to the nearest drivable node of the largest strongly connected
component, and the snap leg is a straight connector at a fixed speed in both directions."""

import logging
import re
from dataclasses import dataclass

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import connected_components, dijkstra
from scipy.spatial import cKDTree

from .geo import Enu, great_circle_m
from .osm import RoadWay
from .pads import Pad
from .params import q

log = logging.getLogger(__name__)

MPS_PER_MPH = 0.44704
MPS_PER_KMH = 1 / 3.6
# Tagged speeds outside this range are treated as tagging errors and use the class default.
MAX_PLAUSIBLE_SPEED_MPS = 45.0
# Two OSM nodes at the same position give a zero-length edge; scipy needs a positive weight.
MIN_EDGE_S = 1e-3
FORWARD = {"yes", "true", "1"}
BACKWARD = {"-1", "reverse"}
SPEED_RE = re.compile(r"^\s*(\d+(?:\.\d+)?)\s*(mph|km/h|kmh|kph)?\s*$")


def parse_maxspeed(text: str | None) -> float | None:
    """`maxspeed` in m/s, the lowest of a `;` list, or None if nothing parses. A bare number is
    km/h (OSM default unit)."""
    if not text:
        return None
    speeds = []
    for part in text.split(";"):
        m = SPEED_RE.match(part)
        if not m:
            continue
        value = float(m.group(1))
        speeds.append(value * (MPS_PER_MPH if m.group(2) == "mph" else MPS_PER_KMH))
    speeds = [s for s in speeds if 0 < s <= MAX_PLAUSIBLE_SPEED_MPS]
    return min(speeds) if speeds else None


def direction(tags: dict) -> tuple[bool, bool]:
    """(forward allowed, backward allowed) along the way's node order."""
    oneway = tags.get("oneway")
    if oneway in BACKWARD:
        return False, True
    if oneway in FORWARD:
        return True, False
    if oneway == "no":
        return True, True
    implied = tags.get("junction") in ("roundabout", "circular") or tags.get("highway") == "motorway"
    return True, not implied


@dataclass
class RoadGraph:
    node_ids: np.ndarray
    lon_deg: np.ndarray
    lat_deg: np.ndarray
    time_s: csr_matrix
    length_m: csr_matrix
    snappable: np.ndarray
    max_speed_mps: float
    tagged_ways: int
    default_ways: int


def build_graph(roads: list[RoadWay], params: dict) -> RoadGraph:
    defaults = {k: q(v, "m/s") for k, v in params["truck"]["default_speed_mps"].items()}
    no_snap = set(params["truck"]["no_snap_classes"])
    all_ids = np.concatenate([r.node_ids for r in roads])
    all_lon = np.concatenate([r.lon_deg for r in roads])
    all_lat = np.concatenate([r.lat_deg for r in roads])
    node_ids, first, inverse = np.unique(all_ids, return_index=True, return_inverse=True)
    lon, lat = all_lon[first], all_lat[first]
    snappable = np.zeros(len(node_ids), dtype=bool)
    us, vs, ts, ls = [], [], [], []
    tagged = defaulted = 0
    max_speed = 0.0
    offset = 0
    for r in roads:
        idx = inverse[offset:offset + len(r.node_ids)]
        offset += len(r.node_ids)
        cls = r.tags["highway"]
        speed = parse_maxspeed(r.tags.get("maxspeed"))
        if speed is None:
            speed = defaults[cls]
            defaulted += 1
        else:
            tagged += 1
        max_speed = max(max_speed, speed)
        if cls not in no_snap:
            snappable[idx] = True
        seg = great_circle_m(r.lat_deg[:-1], r.lon_deg[:-1], r.lat_deg[1:], r.lon_deg[1:])
        t = np.maximum(seg / speed, MIN_EDGE_S)
        fwd, bwd = direction(r.tags)
        if fwd:
            us.append(idx[:-1]); vs.append(idx[1:]); ts.append(t); ls.append(seg)
        if bwd:
            us.append(idx[1:]); vs.append(idx[:-1]); ts.append(t); ls.append(seg)
    u, v, t, length = map(np.concatenate, (us, vs, ts, ls))
    # Parallel edges (two ways over the same node pair): keep the fastest.
    order = np.lexsort((t, v, u))
    u, v, t, length = u[order], v[order], t[order], length[order]
    keep = np.ones(len(u), dtype=bool)
    keep[1:] = (u[1:] != u[:-1]) | (v[1:] != v[:-1])
    u, v, t, length = u[keep], v[keep], t[keep], length[keep]
    n = len(node_ids)
    time_s = csr_matrix((t, (u, v)), shape=(n, n))
    length_m = csr_matrix((length, (u, v)), shape=(n, n))
    log.info("roads: %d nodes, %d directed edges; maxspeed used on %d ways, class default on %d",
             n, len(u), tagged, defaulted)
    return RoadGraph(node_ids, lon, lat, time_s, length_m, snappable, max_speed, tagged, defaulted)


def _edge_length(g: RoadGraph, a: int, b: int) -> float:
    lo, hi = g.length_m.indptr[a], g.length_m.indptr[a + 1]
    k = lo + np.searchsorted(g.length_m.indices[lo:hi], b)
    return float(g.length_m.data[k])


@dataclass
class TruckMatrix:
    free_flow_s: np.ndarray
    road_m: np.ndarray
    snap_m: np.ndarray
    snap_node: np.ndarray
    max_speed_mps: float


def build(pads: list[Pad], g: RoadGraph, enu: Enu, params: dict) -> TruckMatrix:
    snap_speed = q(params["truck"]["snap_speed_mps"], "m/s")
    _, labels = connected_components(g.time_s, directed=True, connection="strong")
    largest = np.bincount(labels).argmax()
    candidates = np.flatnonzero((labels == largest) & g.snappable)
    x, y = enu.xy(g.lon_deg[candidates], g.lat_deg[candidates])
    tree = cKDTree(np.column_stack([x, y]))
    px, py = enu.xy([p.lon_deg for p in pads], [p.lat_deg for p in pads])
    _, k = tree.query(np.column_stack([px, py]))
    snap_node = candidates[k]
    snap_m = great_circle_m([p.lat_deg for p in pads], [p.lon_deg for p in pads],
                            g.lat_deg[snap_node], g.lon_deg[snap_node])
    dist_s, pred = dijkstra(g.time_s, directed=True, indices=snap_node, return_predecessors=True)
    n = len(pads)
    free_flow = np.zeros((n, n))
    road = np.zeros((n, n))
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            node, length = snap_node[j], 0.0
            while node != snap_node[i]:
                prev = pred[i, node]
                if prev < 0:
                    raise RuntimeError(f"no road path {pads[i].name} -> {pads[j].name}")
                length += _edge_length(g, prev, node)
                node = prev
            free_flow[i, j] = (snap_m[i] + snap_m[j]) / snap_speed + dist_s[i, snap_node[j]]
            road[i, j] = snap_m[i] + length + snap_m[j]
    for p, d in zip(pads, snap_m):
        log.info("truck snap %-32s %5.0f m", p.name, d)
    return TruckMatrix(free_flow, road, snap_m, snap_node, max(g.max_speed_mps, snap_speed))


def detour_ratios(pads: list[Pad], tm: TruckMatrix, pairs) -> np.ndarray:
    """Road distance over straight-line distance for the given (a, b) pad pairs, both ways."""
    out = []
    for a, b in pairs:
        straight = great_circle_m(pads[a].lat_deg, pads[a].lon_deg, pads[b].lat_deg, pads[b].lon_deg)
        out += [tm.road_m[a, b] / straight, tm.road_m[b, a] / straight]
    return np.array(out)
