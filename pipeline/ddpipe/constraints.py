"""Where pads may not stand and lanes may not pass: water polygons from OSM and airspace
exclusion zones from the FAA UAS facility map. Geometries are WGS84 lon/lat."""

import json
import logging
from dataclasses import dataclass

import shapely

from .params import q
from .paths import RAW

log = logging.getLogger(__name__)

FACILITY_MAP_FILE = "faa_uas_facility_map.geojson"


@dataclass
class Constraints:
    water: shapely.Geometry
    exclusion: shapely.Geometry
    exclusion_cells: int


def airspace_exclusion(params: dict) -> tuple[shapely.Geometry, int]:
    ceiling_min_ft = q(params["airspace"]["cruise_altitude_agl_ft"], "ft")
    doc = json.loads((RAW / FACILITY_MAP_FILE).read_text())
    cells = []
    for f in doc["features"]:
        p = f["properties"]
        if p["UNIT"] != "Feet":
            raise ValueError(f"facility map cell in {p['UNIT']}, expected Feet")
        if p["CEILING"] < ceiling_min_ft:
            cells.append(shapely.geometry.shape(f["geometry"]))
    log.info("airspace: %d of %d facility-map cells below %.0f ft are excluded",
             len(cells), len(doc["features"]), ceiling_min_ft)
    return shapely.union_all(cells), len(cells)


def build(params: dict, water_polygons: list) -> Constraints:
    water = shapely.union_all([shapely.make_valid(g) for g in water_polygons])
    exclusion, n = airspace_exclusion(params)
    return Constraints(water=water, exclusion=exclusion, exclusion_cells=n)
