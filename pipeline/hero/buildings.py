"""OSM building footprints inside a lon/lat box, from the cached Pennsylvania extract. One pass
over the PBF (about a minute), memoised in data/cache by the extract's sha256 and the box: the
cache only saves time, deleting it changes no output."""

import hashlib
import pickle
from dataclasses import dataclass
from pathlib import Path

import osmium
import shapely

# The tags the clip's extrusion reads (world.building_height).
HEIGHT_TAGS = ("height", "building:levels")


@dataclass
class Building:
    osm_id: str
    geom: shapely.Geometry  # lon/lat, Polygon or MultiPolygon
    tags: dict


def read_cached(pbf: Path, pbf_sha256: str, cache_dir: Path, box: tuple[float, float, float, float]
                ) -> list[Building]:
    """`box` is (west, south, east, north) in degrees."""
    box = tuple(round(v, 6) for v in box)
    key = hashlib.sha256(repr((pbf_sha256, box, HEIGHT_TAGS)).encode()).hexdigest()
    path = cache_dir / f"buildings_{key[:16]}.pickle"
    if path.exists():
        with open(path, "rb") as f:
            return pickle.load(f)
    found = read(pbf, box)
    cache_dir.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".part")
    with open(tmp, "wb") as f:
        pickle.dump(found, f, protocol=pickle.HIGHEST_PROTOCOL)
    tmp.rename(path)
    return found


def read(pbf: Path, box: tuple[float, float, float, float]) -> list[Building]:
    """Closed ways and multipolygon relations tagged building=* (any value but "no") whose
    geometry intersects the box, sorted by OSM id."""
    bbox = shapely.box(*box)
    wkb = osmium.geom.WKBFactory()
    out = []
    proc = (osmium.FileProcessor(str(pbf))
            .with_locations()
            .with_areas()
            .with_filter(osmium.filter.KeyFilter("building")))
    for obj in proc:
        if not obj.is_area() or obj.tags.get("building") in (None, "no"):
            continue
        try:
            geom = shapely.from_wkb(wkb.create_multipolygon(obj))
        except RuntimeError:
            continue
        if not geom.intersects(bbox):
            continue
        kind = "w" if obj.from_way() else "r"
        out.append(Building(f"{kind}{obj.orig_id()}", geom,
                            {k: obj.tags[k] for k in HEIGHT_TAGS if k in obj.tags}))
    out.sort(key=lambda b: (b.osm_id[0], int(b.osm_id[1:])))
    return out
