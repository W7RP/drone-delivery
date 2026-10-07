"""One pass over the OSM extract: drivable road ways, water polygons and named site features
inside the pipeline area. Everything outside the area bounding box is dropped here."""

import logging
from dataclasses import dataclass, field

import numpy as np
import osmium
import shapely

log = logging.getLogger(__name__)

# Ways a delivery truck may use (osmnx "drive" in spirit). Service roads stay, except parking
# aisles and private driveways; a pad behind a gate is reached by the snap leg (truck.py).
DRIVABLE = {
    "motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link",
    "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified",
    "residential", "living_street", "service",
}
EXCLUDED_SERVICE = {"parking_aisle", "driveway", "drive-through", "emergency_access"}
NO_ACCESS = {"no", "private"}
ROAD_TAGS = ("highway", "oneway", "maxspeed", "junction", "service")


@dataclass
class RoadWay:
    way_id: int
    node_ids: np.ndarray
    lon_deg: np.ndarray
    lat_deg: np.ndarray
    tags: dict


@dataclass
class Site:
    osm_key: str
    name: str
    tags: dict
    geom: shapely.Geometry


@dataclass
class Extract:
    roads: list[RoadWay] = field(default_factory=list)
    water: list[shapely.Geometry] = field(default_factory=list)
    sites: list[Site] = field(default_factory=list)


def _is_drivable(tags) -> bool:
    if tags.get("highway") not in DRIVABLE:
        return False
    if tags.get("service") in EXCLUDED_SERVICE:
        return False
    if tags.get("area") == "yes":
        return False
    for key in ("access", "motor_vehicle", "motorcar"):
        if tags.get(key) in NO_ACCESS:
            return False
    return True


def _is_water(tags) -> bool:
    return tags.get("natural") == "water" or tags.get("waterway") == "riverbank"


def _site_tags(tags, keys) -> dict:
    return {k: tags.get(k) for k in keys if k in tags}


def read(pbf_path, area: dict, site_keys: tuple[str, ...]) -> Extract:
    """`area` has south_deg/west_deg/north_deg/east_deg; `site_keys` are the tag keys a site
    rule may test (sites.yaml)."""
    s, w, n, e = area["south_deg"], area["west_deg"], area["north_deg"], area["east_deg"]
    box = shapely.box(w, s, e, n)
    wkb = osmium.geom.WKBFactory()
    out = Extract()
    keys = ("highway", "natural", "waterway", *site_keys)
    proc = (
        osmium.FileProcessor(str(pbf_path))
        .with_locations()
        .with_areas()
        .with_filter(osmium.filter.KeyFilter(*keys))
    )
    for obj in proc:
        tags = obj.tags
        if obj.is_node():
            name = tags.get("name")
            loc = obj.location
            if name and loc.valid() and s <= loc.lat <= n and w <= loc.lon <= e:
                out.sites.append(
                    Site(f"n{obj.id}", name, _site_tags(tags, site_keys),
                         shapely.Point(loc.lon, loc.lat))
                )
        elif obj.is_way():
            if not _is_drivable(tags):
                continue
            try:
                ids = np.fromiter((nd.ref for nd in obj.nodes), np.int64)
                lon = np.fromiter((nd.lon for nd in obj.nodes), np.float64)
                lat = np.fromiter((nd.lat for nd in obj.nodes), np.float64)
            except osmium.InvalidLocationError:
                continue
            if len(ids) < 2 or not np.any((lat >= s) & (lat <= n) & (lon >= w) & (lon <= e)):
                continue
            out.roads.append(RoadWay(obj.id, ids, lon, lat,
                                     {k: tags[k] for k in ROAD_TAGS if k in tags}))
        elif obj.is_area():
            water = _is_water(tags)
            name = tags.get("name")
            if not water and not name:
                continue
            try:
                geom = shapely.from_wkb(wkb.create_multipolygon(obj))
            except RuntimeError:
                continue
            if not geom.intersects(box):
                continue
            if water:
                out.water.append(geom)
            if name:
                kind = "w" if obj.from_way() else "r"
                out.sites.append(Site(f"{kind}{obj.orig_id()}", name,
                                      _site_tags(tags, site_keys), geom))
    log.info("osm: %d road ways, %d water polygons, %d named features",
             len(out.roads), len(out.water), len(out.sites))
    return out
