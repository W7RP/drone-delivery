"""Phase 8: map of the clip flight for the README (docs/media/hero_track.png).

Top: the drone's ground-truth track over the corridor world it flew in: the DEM grid shaded,
the corridor (lane buffer and pad buffers of hero.yaml) left bright, water, roads and buildings
from the generated world. Bottom: height along the lane, the drone against the terrain and the
buildings under the straight A-B line. Reads <run>/flight_samples.bin and the world the run
names in run.json, and refuses a world that is not the one the run flew."""

import hashlib
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import shapely  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402
from matplotlib.colors import LightSource, ListedColormap  # noqa: E402
from PIL import Image  # noqa: E402

from .common import REPO, config  # noqa: E402

sys.path.insert(0, str(REPO / "px4" / "tools"))
import flightlog  # noqa: E402

MEDIA = REPO / "docs" / "media"
DPI = 110
PROFILE_STEP_M = 10.0
# Colour range of the corridor's ground, metres NAVD88 (the lane runs 205 to 351 m).
ELEV_MIN_M = 200.0
ELEV_MAX_M = 360.0
# The README image is quantized to this many colours to stay small.
PNG_COLOURS = 256
AIRBORNE = {"TAKEOFF", "EN_ROUTE", "HOLDING", "APPROACH", "HOPPING"}
WATER_RGB = (0.36, 0.52, 0.68)
ROAD_RGB = (0.25, 0.25, 0.27)
BUILDING_RGB = (0.62, 0.6, 0.57)
TRACK_RGB = (0.85, 0.2, 0.1)


def read_obj_triangles(path: Path) -> np.ndarray:
    """Triangles of a world mesh as (n, 3, 2) ENU x, y; faces are `f a//na b//nb c//nc`."""
    verts, faces = [], []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            verts.append([float(v) for v in line.split()[1:3]])
        elif line.startswith("f "):
            faces.append([int(p.split("/")[0]) - 1 for p in line.split()[1:4]])
    return np.asarray(verts)[np.asarray(faces)]


def corridor(cfg: dict, a_xy, b_xy) -> shapely.Geometry:
    lane = shapely.LineString([a_xy, b_xy]).buffer(cfg["lane_buffer_m"])
    pads = [shapely.Point(p).buffer(cfg["pad_buffer_m"]) for p in (a_xy, b_xy)]
    return shapely.union_all([lane, *pads])


def surface(grid: np.ndarray, x0: float, y0: float, g: float, x, y) -> np.ndarray:
    """The terrain mesh's height at (x, y), cells split along (i, j)-(i+1, j+1) as world.py
    builds it (same as px4/tools/hero_check.py)."""
    ny, nx = grid.shape
    u = np.clip((np.asarray(x) - x0) / g, 0, nx - 1 - 1e-9)
    v = np.clip((np.asarray(y) - y0) / g, 0, ny - 1 - 1e-9)
    i, j = np.floor(u).astype(np.int64), np.floor(v).astype(np.int64)
    u, v = u - i, v - j
    z00, z10, z01, z11 = grid[j, i], grid[j, i + 1], grid[j + 1, i], grid[j + 1, i + 1]
    return np.where(u >= v, z00 + u * (z10 - z00) + v * (z11 - z10), z00 + v * (z01 - z00) + u * (z11 - z01))


def building_tops_along(buildings: list[dict], px: np.ndarray, py: np.ndarray) -> np.ndarray:
    """Highest building roof over each point, NaN where no footprint covers it."""
    polys = [shapely.Polygon(b["exterior_m"]) for b in buildings]
    tree = shapely.STRtree(polys)
    pts = shapely.points(px, py)
    tops = np.full(len(px), np.nan)
    for k, i in zip(*tree.query(pts, predicate="within")):
        tops[k] = np.fmax(tops[k], buildings[i]["top_z_m"])
    return tops


def main(run: Path) -> Path:
    meta = json.loads((run / "run.json").read_text())
    world = Path(meta["world_dir"])
    sha = hashlib.sha256((world / "world_src.sdf").read_bytes()).hexdigest()[:16]
    if sha != meta["world_sdf_sha256"]:
        raise RuntimeError(f"{run.name} flew world {meta['world_sdf_sha256']}, {world} holds {sha}")
    lane = json.loads((world / "lane.json").read_text())
    obst = json.loads((world / "obstacles" / "obstacles.json").read_text())
    tg = obst["terrain"]
    grid = np.load(world / "obstacles" / tg["file"]).astype(np.float64)
    meshes = world / "models" / "dd_hero_world" / "meshes"
    elev_a = lane["terrain"]["pad_a_elevation_m"]
    a_xy = np.array(lane["from_pad"]["enu_m"])
    b_xy = np.array(lane["to_pad"]["enu_m"])

    s = flightlog.read_samples(run / "flight_samples.bin")
    modes = np.array([flightlog.DRONE_MODES[m] for m in s["drone_mode"]])
    truth = s["truth_enu_m"].astype(np.float64)
    air = np.isin(modes, list(AIRBORNE))
    x, y, z = truth[air, 0], truth[air, 1], truth[air, 2]

    fig, (ax, prof) = plt.subplots(2, 1, figsize=(10, 9.4), gridspec_kw={"height_ratios": [3.2, 1]})
    ny, nx = grid.shape
    x1, y1 = tg["x0_m"] + (nx - 1) * tg["spacing_m"], tg["y0_m"] + (ny - 1) * tg["spacing_m"]
    extent = [tg["x0_m"] / 1e3, x1 / 1e3, tg["y0_m"] / 1e3, y1 / 1e3]
    light = LightSource(azdeg=315, altdeg=40)
    land = ListedColormap(plt.get_cmap("terrain")(np.linspace(0.25, 0.9, 256)))
    shaded = light.shade(grid + elev_a, cmap=land, vert_exag=3, dx=tg["spacing_m"], dy=tg["spacing_m"],
                         blend_mode="soft", vmin=ELEV_MIN_M, vmax=ELEV_MAX_M)[..., :3]
    relief = light.hillshade(grid, vert_exag=3, dx=tg["spacing_m"], dy=tg["spacing_m"])
    gx, gy = np.meshgrid(tg["x0_m"] + tg["spacing_m"] * np.arange(nx), tg["y0_m"] + tg["spacing_m"] * np.arange(ny))
    # Outside the corridor the world has no water, roads or buildings: grey relief for context only.
    inside = shapely.contains_xy(corridor(config()["world"], a_xy, b_xy), gx, gy)
    shaded[~inside] = (0.6 + 0.35 * relief[~inside])[:, None]
    ax.imshow(shaded, origin="lower", extent=extent, interpolation="bilinear")

    for name, rgb, lw in (("water.obj", WATER_RGB, 0.3), ("roads.obj", ROAD_RGB, 0.2)):
        tris = read_obj_triangles(meshes / name) / 1e3
        ax.add_collection(PolyCollection(tris, facecolors=rgb, edgecolors=rgb, linewidths=lw))
    footprints = [np.asarray(b["exterior_m"]) / 1e3 for b in obst["buildings"]]
    ax.add_collection(PolyCollection(footprints, facecolors=BUILDING_RGB, edgecolors="0.3", linewidths=0.2))
    ax.plot([a_xy[0] / 1e3, b_xy[0] / 1e3], [a_xy[1] / 1e3, b_xy[1] / 1e3], ls="--", lw=0.8, color="k",
            label="straight A-B line")
    ax.plot(x / 1e3, y / 1e3, color=TRACK_RGB, lw=1.8, label="drone track (Gazebo ground truth)")
    for xy, tag, pad, dx, ha in ((a_xy, "A", lane["from_pad"], 0.25, "left"),
                                 (b_xy, "B", lane["to_pad"], -0.25, "right")):
        ax.plot(*(xy / 1e3), marker="o", ms=8, mfc="white", mec="k", zorder=5)
        ax.annotate(f"pad {tag}: {pad['osm_name']}", xy / 1e3, xytext=(xy[0] / 1e3 + dx, xy[1] / 1e3),
                    ha=ha, va="center", fontsize=9, bbox={"fc": "white", "ec": "0.6", "alpha": 0.85})
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.set_xlabel("east of pad A (km)")
    ax.set_ylabel("north of pad A (km)")
    ax.legend(loc="lower left", fontsize=8)
    ax.set_title(f"Lane {lane['lane_id']}, {lane['length_m'] / 1e3:.1f} km, VTOL with a 5 kg pod in PX4 SITL "
                 f"(run {run.name}, seed {meta['seed']}, px4/ at {meta['script_git']})", fontsize=10)

    unit = (b_xy - a_xy) / np.linalg.norm(b_xy - a_xy)
    along = np.arange(0.0, np.linalg.norm(b_xy - a_xy) + PROFILE_STEP_M, PROFILE_STEP_M)
    lx, ly = a_xy[0] + along * unit[0], a_xy[1] + along * unit[1]
    ground = surface(grid, tg["x0_m"], tg["y0_m"], tg["spacing_m"], lx, ly) + elev_a
    tops = building_tops_along(obst["buildings"], lx, ly) + elev_a
    d_km = along / 1e3
    prof.fill_between(d_km, 150, ground, color=(0.55, 0.6, 0.45), lw=0, label="terrain under the line")
    prof.vlines(d_km[~np.isnan(tops)], ground[~np.isnan(tops)], tops[~np.isnan(tops)], color=BUILDING_RGB,
                lw=1.5, label="buildings under the line")
    drone_along = ((x - a_xy[0]) * unit[0] + (y - a_xy[1]) * unit[1]) / 1e3
    prof.plot(drone_along, z + elev_a, color=TRACK_RGB, lw=1.5, label="drone")
    prof.set_xlim(-0.2, d_km[-1] + 0.2)
    prof.set_ylim(150, max(float(np.max(z)) + elev_a, float(np.nanmax(tops))) + 30)
    prof.set_xlabel("distance along the A-B line (km)")
    prof.set_ylabel("height NAVD88 (m)")
    prof.legend(loc="lower center", fontsize=8, ncol=3)
    fig.tight_layout()
    MEDIA.mkdir(parents=True, exist_ok=True)
    out = MEDIA / "hero_track.png"
    fig.savefig(out, dpi=DPI)
    plt.close(fig)
    Image.open(out).convert("RGB").quantize(PNG_COLOURS).save(out, optimize=True)
    print(f"wrote {out} ({out.stat().st_size // 1024} KB)")
    return out
