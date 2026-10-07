"""Step 8: one debug figure, out/pittsburgh_overview.png: pads, lanes coloured by ground risk,
rejected lanes, airspace exclusion cells and water. For checking the pipeline by eye only."""

import logging

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import shapely  # noqa: E402
from matplotlib.collections import LineCollection, PathCollection  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
from matplotlib.path import Path as MplPath  # noqa: E402

from .paths import OUT  # noqa: E402

log = logging.getLogger(__name__)

PNG = OUT / "pittsburgh_overview.png"
MARGIN_DEG = 0.03
DPI = 150
FIG_IN = (10, 11)


def _polygon_paths(geom: shapely.Geometry) -> list[MplPath]:
    """Matplotlib paths with holes kept, so islands in a river stay land."""
    out = []
    for poly in getattr(geom, "geoms", [geom]):
        if poly.geom_type != "Polygon" or poly.is_empty:
            continue
        rings = [np.asarray(poly.exterior.coords)] + [np.asarray(r.coords) for r in poly.interiors]
        verts = np.concatenate(rings)
        codes = np.concatenate([
            [MplPath.MOVETO] + [MplPath.LINETO] * (len(r) - 2) + [MplPath.CLOSEPOLY]
            for r in rings
        ])
        out.append(MplPath(verts, codes))
    return out


def overview(pads, lane_list, lane_rejections, cons, snapshot_date: str) -> None:
    lon = np.array([p.lon_deg for p in pads])
    lat = np.array([p.lat_deg for p in pads])
    box = shapely.box(lon.min() - MARGIN_DEG, lat.min() - MARGIN_DEG,
                      lon.max() + MARGIN_DEG, lat.max() + MARGIN_DEG)
    fig, ax = plt.subplots(figsize=FIG_IN)
    water = shapely.intersection(cons.water, box)
    water_style = {"facecolor": "#a6cee3", "edgecolor": "none"}
    ax.add_collection(PathCollection(_polygon_paths(water), **water_style))
    excl = shapely.intersection(cons.exclusion, box)
    excl_style = {"facecolor": "#e31a1c", "alpha": 0.35, "edgecolor": "#e31a1c", "linewidth": 0.4}
    ax.add_collection(PathCollection(_polygon_paths(excl), **excl_style))
    index = {p.name: i for i, p in enumerate(pads)}
    rejected = [[(lon[index[r.a]], lat[index[r.a]]), (lon[index[r.b]], lat[index[r.b]])]
                for r in lane_rejections]
    ax.add_collection(LineCollection(rejected, colors="0.55", linewidths=0.7, linestyles="--",
                                     label="rejected lane"))
    segments = [[(lon[l.a], lat[l.a]), (lon[l.b], lat[l.b])] for l in lane_list]
    risk = np.array([l.ground_risk_per_km2 for l in lane_list])
    lanes = LineCollection(segments, array=risk, cmap="plasma", linewidths=2.2, zorder=3)
    ax.add_collection(lanes)
    fig.colorbar(lanes, ax=ax, shrink=0.6,
                 label="lane ground risk: mean population density along the lane, people/km²")
    hub = np.array([p.is_hub for p in pads])
    ax.scatter(lon[~hub], lat[~hub], s=45, c="black", zorder=4, label="pad")
    ax.scatter(lon[hub], lat[hub], s=160, marker="*", c="black", zorder=4, label="hub pad")
    for p in pads:
        ax.annotate(p.osm_name, (p.lon_deg, p.lat_deg), xytext=(5, 4),
                    textcoords="offset points", fontsize=7, zorder=5)
    minx, miny, maxx, maxy = box.bounds
    ax.set_xlim(minx, maxx)
    ax.set_ylim(miny, maxy)
    ax.set_aspect(1 / np.cos(np.radians(lat.mean())))
    ax.set_xlabel("longitude, deg")
    ax.set_ylabel("latitude, deg")
    ax.set_title(f"Pittsburgh parts logistics: {len(pads)} pads, {len(lane_list)} lanes, "
                 f"{len(lane_rejections)} rejected\nOSM, FAA, USGS and Census data retrieved "
                 f"{snapshot_date} (pipeline debug view)")
    # Collections of paths get no legend entry of their own; add proxies.
    handles, _ = ax.get_legend_handles_labels()
    handles += [Patch(label="water (OSM)", **water_style),
                Patch(label="airspace exclusion (FAA facility map cell below cruise altitude)",
                      **excl_style)]
    ax.legend(handles=handles, loc="lower left", fontsize=8)
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(PNG, dpi=DPI, metadata={"Software": None})
    plt.close(fig)
    log.info("plot: %s", PNG)
