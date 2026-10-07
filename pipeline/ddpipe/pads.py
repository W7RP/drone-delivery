"""Step 2: pick pads from OSM site footprints by the rules in sites.yaml. Deterministic: the
order of candidates depends only on footprint area and OSM id."""

import logging
import re
from dataclasses import dataclass

import shapely

from .constraints import Constraints
from .geo import Enu
from .osm import Site
from .params import q

log = logging.getLogger(__name__)

MAX_NAME_LEN = 32
# Pad coordinates are rounded once, here, to 1e-7 deg (about 1 cm), so every later step and the
# sim compute from exactly the numbers written to pads.toml.
COORD_DECIMALS = 7


@dataclass
class Pad:
    name: str
    osm_name: str
    osm_key: str
    cluster: str
    role: str
    is_hub: bool
    lat_deg: float
    lon_deg: float
    site_area_m2: float


def config_name(osm_name: str, taken: set[str]) -> str:
    """OSM name to a pad name the config accepts: ASCII letters, digits and underscores,
    starting with a letter, at most 32 characters, unique."""
    base = re.sub(r"[^A-Za-z0-9]+", "_", osm_name).strip("_")
    if not base or not base[0].isalpha():
        base = "P_" + base
    base = base[:MAX_NAME_LEN].rstrip("_")
    name, n = base, 2
    while name in taken:
        suffix = f"_{n}"
        name = base[: MAX_NAME_LEN - len(suffix)] + suffix
        n += 1
    return name


def _matches(site: Site, match: dict) -> bool:
    return any(site.tags.get(k) in values for k, values in match.items())


def site_keys(sites_cfg: dict) -> tuple[str, ...]:
    keys = sorted({k for c in sites_cfg["clusters"] for k in c["match"]})
    return tuple(keys)


@dataclass
class Rejection:
    cluster: str
    osm_key: str
    osm_name: str
    reason: str


def select(sites_cfg: dict, sites: list[Site], constraints: Constraints):
    """Return (pads, rejections). Pads are in cluster order, then rank order."""
    min_area = q(sites_cfg["min_site_area_m2"], "m2")
    min_sep = q(sites_cfg["min_separation_m"], "m")
    pads: list[Pad] = []
    rejections: list[Rejection] = []
    taken: set[str] = set()
    chosen_points: list[shapely.Point] = []
    for cluster in sites_cfg["clusters"]:
        lat_c, lon_c = cluster["center_deg"]
        enu = Enu(lat_c, lon_c)
        radius = q(cluster["search_radius_m"], "m")
        candidates = []
        for s in sites:
            if s.geom.geom_type not in ("Polygon", "MultiPolygon") or not _matches(s, cluster["match"]):
                continue
            footprint = enu.to_enu(s.geom)
            point = s.geom.point_on_surface()
            px, py = enu.xy(point.x, point.y)
            if footprint.area < min_area or float((px**2 + py**2) ** 0.5) > radius:
                continue
            candidates.append((-footprint.area, s.osm_key, s, point, footprint.area))
        candidates.sort(key=lambda c: (c[0], c[1]))
        picked = 0
        for _, key, s, point, area in candidates:
            if picked == cluster["pads"]:
                break
            reason = None
            if constraints.water.contains(point):
                reason = "pad point in water"
            elif constraints.exclusion.contains(point):
                reason = "pad point in an airspace exclusion zone"
            else:
                for other in chosen_points:
                    ox, oy = enu.xy(other.x, other.y)
                    px, py = enu.xy(point.x, point.y)
                    if float(((px - ox) ** 2 + (py - oy) ** 2) ** 0.5) < min_sep:
                        reason = f"within {min_sep:.0f} m of a chosen pad"
                        break
            if reason:
                rejections.append(Rejection(cluster["name"], key, s.name, reason))
                continue
            name = config_name(s.name, taken)
            taken.add(name)
            chosen_points.append(point)
            pads.append(Pad(
                name=name, osm_name=s.name, osm_key=key, cluster=cluster["name"],
                role=cluster["role"], is_hub=bool(cluster.get("hub")) and picked == 0,
                lat_deg=round(point.y, COORD_DECIMALS), lon_deg=round(point.x, COORD_DECIMALS),
                site_area_m2=area,
            ))
            picked += 1
        if picked < cluster["pads"]:
            log.warning("cluster %s: only %d of %d pads found", cluster["name"], picked,
                        cluster["pads"])
    for p in pads:
        log.info("pad %-32s %-14s %s (%s, %.0f m2)%s", p.name, p.cluster, p.osm_name, p.osm_key,
                 p.site_area_m2, " hub" if p.is_hub else "")
    for r in rejections:
        log.info("pad rejected: %s %s %s: %s", r.cluster, r.osm_key, r.osm_name, r.reason)
    return pads, rejections
