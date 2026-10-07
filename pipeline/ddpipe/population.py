"""Population density per 2020 Census block group: the sum of POP20 over the group's blocks
divided by the group's land area (ALAND). People per km2."""

import logging

import numpy as np
import pyogrio
import shapely

from .paths import RAW

log = logging.getLogger(__name__)

BLOCK_GROUPS = "tl_2020_42_bg.zip"
BLOCKS = "tl_2020_42_tabblock20.zip"
M2_PER_KM2 = 1e6


class Population:
    def __init__(self, area: dict):
        bbox = (area["west_deg"], area["south_deg"], area["east_deg"], area["north_deg"])
        _, _, geoms, fields = pyogrio.raw.read(
            f"/vsizip/{RAW / BLOCK_GROUPS}", columns=["GEOID", "ALAND"], bbox=bbox
        )
        geoid, aland_m2 = fields
        _, _, _, bfields = pyogrio.raw.read(
            f"/vsizip/{RAW / BLOCKS}", columns=["GEOID20", "POP20"], read_geometry=False
        )
        pop_by_bg: dict[str, int] = {}
        for block, pop in zip(bfields[0], bfields[1]):
            key = block[:12]
            pop_by_bg[key] = pop_by_bg.get(key, 0) + int(pop)
        pop = np.array([pop_by_bg.get(g, 0) for g in geoid], dtype=np.float64)
        land = np.asarray(aland_m2, dtype=np.float64)
        self.density_per_km2 = np.where(land > 0, pop / np.maximum(land, 1.0) * M2_PER_KM2, 0.0)
        self.polygons = shapely.from_wkb(geoms)
        self.tree = shapely.STRtree(self.polygons)
        log.info("population: %d block groups, %.0f people in the area",
                 len(self.polygons), pop.sum())

    def density(self, lon_deg: np.ndarray, lat_deg: np.ndarray) -> np.ndarray:
        """Density at each point; 0 outside every block group."""
        points = shapely.points(lon_deg, lat_deg)
        idx_pt, idx_poly = self.tree.query(points, predicate="within")
        out = np.zeros(len(points))
        # A point on a shared boundary can fall in two groups; keep the first, deterministically.
        order = np.lexsort((idx_poly, idx_pt))
        idx_pt, idx_poly = idx_pt[order], idx_poly[order]
        first = np.ones(len(idx_pt), dtype=bool)
        first[1:] = idx_pt[1:] != idx_pt[:-1]
        out[idx_pt[first]] = self.density_per_km2[idx_poly[first]]
        return out
