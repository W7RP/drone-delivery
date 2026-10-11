"""Phase 8 step 4: checks of one flight over the terrain world (docs/hero-clip.md 4).

- Height: PX4's EKF altitude (barometric, A67) against the ground truth, both from the start
  on pad A's OUT. Their difference is the EKF's height error relative to the DEM-built world
  (world z is DEM elevation minus pad A's). Reported over the whole airborne flight and over
  the lane.
- Clearance: the ground-truth path against the terrain mesh (vertical) and the building prisms
  (3D distance to the nearest prism), over the lane (EN_ROUTE) and over the whole airborne
  flight outside the pads' clear circles, against the placeholder margins in
  pipeline/hero/hero.yaml (`flight`).

The world must be the one the flight flew: the run's world_src.sdf hash must match the world
directory's. Usage: hero_check.py <run dir> [--world-dir DIR] [--out FILE]; prints JSON."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import flightlog  # noqa: E402

PX4 = Path(__file__).resolve().parents[1]
HERO_CFG = PX4.parent / "pipeline" / "hero" / "hero.yaml"
AIRBORNE = {"TAKEOFF", "EN_ROUTE", "HOLDING", "APPROACH", "GO_AROUND", "HOPPING"}
# Building distances are computed on samples this far apart in time (the path moves at most
# 15 m/s, so 0.5 s is 7.5 m).
BUILDING_STEP_S = 0.5
# Only prisms whose bounding box is this close to a sample are measured exactly.
NEAR_M = 400.0


def surface(grid: np.ndarray, x0: float, y0: float, g: float, x, y) -> np.ndarray:
    """The terrain mesh's height at (x, y): cells split along (i, j)-(i+1, j+1), as
    pipeline/hero/world.py builds it."""
    ny, nx = grid.shape
    u = np.clip((np.asarray(x) - x0) / g, 0, nx - 1 - 1e-9)
    v = np.clip((np.asarray(y) - y0) / g, 0, ny - 1 - 1e-9)
    i, j = np.floor(u).astype(np.int64), np.floor(v).astype(np.int64)
    u, v = u - i, v - j
    z00, z10, z01, z11 = grid[j, i], grid[j, i + 1], grid[j + 1, i], grid[j + 1, i + 1]
    return np.where(u >= v, z00 + u * (z10 - z00) + v * (z11 - z10), z00 + v * (z01 - z00) + u * (z11 - z01))


def horizontal_distance(px: np.ndarray, py: np.ndarray, ring: np.ndarray) -> np.ndarray:
    """Distance from points to a closed polygon ring, 0 inside (even-odd rule)."""
    a, b = ring[:-1], ring[1:]
    d = b - a
    seg_len2 = np.maximum((d ** 2).sum(1), 1e-12)
    t = np.clip(((px[:, None] - a[:, 0]) * d[:, 0] + (py[:, None] - a[:, 1]) * d[:, 1]) / seg_len2, 0, 1)
    cx, cy = a[:, 0] + t * d[:, 0], a[:, 1] + t * d[:, 1]
    dist = np.sqrt((px[:, None] - cx) ** 2 + (py[:, None] - cy) ** 2).min(1)
    crosses = ((a[:, 1] > py[:, None]) != (b[:, 1] > py[:, None])) & (
        px[:, None] < (b[:, 0] - a[:, 0]) * (py[:, None] - a[:, 1]) / np.where(d[:, 1] == 0, 1e-12, d[:, 1]) + a[:, 0])
    inside = crosses.sum(1) % 2 == 1
    return np.where(inside, 0.0, dist)


def building_clearance(prisms: list[dict], x: np.ndarray, y: np.ndarray, z: np.ndarray) -> tuple[np.ndarray, list]:
    best = np.full(len(x), np.inf)
    who = [None] * len(x)
    for p in prisms:
        ring = np.asarray(p["exterior_m"], dtype=np.float64)
        lo, hi = ring.min(0), ring.max(0)
        box = np.hypot(np.maximum(0, np.maximum(lo[0] - x, x - hi[0])), np.maximum(0, np.maximum(lo[1] - y, y - hi[1])))
        near = box < np.minimum(best, NEAR_M)
        if not near.any():
            continue
        dh = horizontal_distance(x[near], y[near], ring)
        dz = np.maximum(0.0, z[near] - p["top_z_m"])
        d = np.hypot(dh, dz)
        idx = np.nonzero(near)[0]
        better = d < best[idx]
        best[idx[better]] = d[better]
        for k in idx[better]:
            who[k] = p["osm_id"]
    return best, who


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", type=Path)
    ap.add_argument("--world-dir", type=Path, default=PX4 / "sim" / "hero")
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()
    meta = json.loads((a.run / "run.json").read_text())
    want = hashlib.sha256((a.world_dir / "world_src.sdf").read_bytes()).hexdigest()[:16]
    if meta.get("world_sdf_sha256") != want:
        raise SystemExit(f"{a.run} flew world {meta.get('world_sdf_sha256')}, {a.world_dir} holds {want}")
    margins = yaml.safe_load(HERO_CFG.read_text())["flight"]
    obst = json.loads((a.world_dir / "obstacles" / "obstacles.json").read_text())
    tg = obst["terrain"]
    grid = np.load(a.world_dir / "obstacles" / tg["file"]).astype(np.float64)
    scene = json.loads((a.run / "scene.json").read_text())
    s = flightlog.read_samples(a.run / "flight_samples.bin")
    t = s["t_us"] * 1e-6
    modes = np.array([flightlog.DRONE_MODES[m] for m in s["drone_mode"]])
    air = np.isin(modes, list(AIRBORNE))
    lane = modes == "EN_ROUTE"
    truth = s["truth_enu_m"].astype(np.float64)
    x, y, z = truth[:, 0], truth[:, 1], truth[:, 2]
    # Height error: EKF altitude above its origin against the truth above the start.
    ekf_up = -s["pos_ned_m"][:, 2].astype(np.float64)
    drift = ekf_up - (z - z[0])
    ground = surface(grid, tg["x0_m"], tg["y0_m"], tg["spacing_m"], x, y)
    agl = z - ground
    pads = [np.mean([scene["spots"][f"{p}_IN"]["enu_m"], scene["spots"][f"{p}_OUT"]["enu_m"]], axis=0) for p in "AB"]
    clear_r = yaml.safe_load(HERO_CFG.read_text())["world"]["pad_clear_radius_m"]
    away = np.all([np.hypot(x - px, y - py) > clear_r for px, py in pads], axis=0)
    step = max(1, int(round(BUILDING_STEP_S / np.median(np.diff(t)))))
    sub = np.zeros(len(t), bool)
    sub[::step] = True
    sel = sub & air & away
    bdist, bwho = building_clearance(obst["buildings"], x[sel], y[sel], z[sel])
    b_lane = lane[sel]

    def at(mask, values):
        k = int(np.argmin(np.where(mask, values, np.inf)))
        return k

    def terrain_min(mask):
        k = at(mask, agl)
        return {"m": round(float(agl[k]), 2), "t_s": round(float(t[k]), 1),
                "enu_m": [round(float(x[k]), 1), round(float(y[k]), 1), round(float(z[k]), 1)]}

    def building_min(mask):
        if not mask.any():
            return None
        k = int(np.argmin(np.where(mask, bdist, np.inf)))
        idx = np.nonzero(sel)[0][k]
        return {"m": round(float(bdist[k]), 2), "t_s": round(float(t[idx]), 1), "osm_id": bwho[k],
                "enu_m": [round(float(x[idx]), 1), round(float(y[idx]), 1), round(float(z[idx]), 1)]}

    doc = {
        "run": a.run.name, "world_sdf_sha256": want,
        "height": {"max_abs_drift_airborne_m": round(float(np.abs(drift[air]).max()), 3),
                   "max_abs_drift_lane_m": round(float(np.abs(drift[lane]).max()), 3),
                   "drift_at_end_m": round(float(drift[-1]), 3),
                   "mean_drift_lane_m": round(float(drift[lane].mean()), 3)},
        "terrain_clearance": {"lane": terrain_min(lane), "airborne_off_pad": terrain_min(air & away),
                              "margin_m": margins["terrain_margin_m"]},
        "building_clearance": {"lane": building_min(b_lane), "airborne_off_pad": building_min(np.ones(len(bdist), bool)),
                               "margin_m": margins["building_margin_m"]},
        "cruise_altitude_m": round(float(np.median(z[lane])), 2),
    }
    doc["terrain_clearance"]["lane_ok"] = doc["terrain_clearance"]["lane"]["m"] >= margins["terrain_margin_m"]
    bl = doc["building_clearance"]["lane"]
    doc["building_clearance"]["lane_ok"] = bl is None or bl["m"] >= margins["building_margin_m"]
    text = json.dumps(doc, indent=1)
    if a.out:
        a.out.write_text(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
