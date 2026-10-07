"""Terrain elevation from the USGS 3DEP 1 arc-second tiles (metres, NAVD88), mosaicked over the
pipeline area once and sampled by nearest pixel."""

import numpy as np
import rasterio
from rasterio.merge import merge

from .paths import RAW

DEM_FILES = ("USGS_1_n41w080.tif", "USGS_1_n41w081.tif")


class Terrain:
    def __init__(self, area: dict):
        srcs = [rasterio.open(RAW / f) for f in DEM_FILES]
        try:
            bounds = (area["west_deg"], area["south_deg"], area["east_deg"], area["north_deg"])
            mosaic, self.transform = merge(srcs, bounds=bounds)
            self.nodata = srcs[0].nodata
        finally:
            for s in srcs:
                s.close()
        self.elev_m = mosaic[0]
        self._inv = ~self.transform

    def sample(self, lon_deg: np.ndarray, lat_deg: np.ndarray) -> np.ndarray:
        cols, rows = self._inv * (np.asarray(lon_deg), np.asarray(lat_deg))
        r = np.floor(rows).astype(np.int64)
        c = np.floor(cols).astype(np.int64)
        if (r.min() < 0 or c.min() < 0 or r.max() >= self.elev_m.shape[0]
                or c.max() >= self.elev_m.shape[1]):
            raise ValueError("terrain sample outside the DEM mosaic")
        z = self.elev_m[r, c].astype(np.float64)
        if self.nodata is not None and np.any(z == self.nodata):
            raise ValueError("terrain sample on a DEM nodata pixel")
        return z
