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
class ExclusionCell:
    geom: shapely.Geometry
    airport: str
    ceiling_ft: float


@dataclass
class Constraints:
    water: shapely.Geometry
    exclusion: shapely.Geometry
    cells: list[ExclusionCell]

    def cells_crossed(self, line: shapely.Geometry) -> list[ExclusionCell]:
        return [c for c in self.cells if c.geom.intersects(line)]


def airspace_exclusion(params: dict) -> list[ExclusionCell]:
    ceiling_min_ft = q(params["airspace"]["cruise_altitude_agl_ft"], "ft")
    doc = json.loads((RAW / FACILITY_MAP_FILE).read_text())
    cells = []
    for f in doc["features"]:
        p = f["properties"]
        if p["UNIT"] != "Feet":
            raise ValueError(f"facility map cell in {p['UNIT']}, expected Feet")
        if p["CEILING"] < ceiling_min_ft:
            cells.append(ExclusionCell(shapely.geometry.shape(f["geometry"]), p["APT1_FAAID"],
                                       float(p["CEILING"])))
    log.info("airspace: %d of %d facility-map cells below %.0f ft are excluded",
             len(cells), len(doc["features"]), ceiling_min_ft)
    return cells


def build(params: dict, water_polygons: list) -> Constraints:
    water = shapely.union_all([shapely.make_valid(g) for g in water_polygons])
    cells = airspace_exclusion(params)
    exclusion = shapely.union_all([c.geom for c in cells])
    return Constraints(water=water, exclusion=exclusion, cells=cells)
