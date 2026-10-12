"""Geodesy shared by every step. The sim's own projection (spec section 0: equirectangular,
R = 6 371 000 m, about the scenario's geo origin) is the only one used, so positions and
lengths here are the ones the sim would compute. "Straight-line distance" is the great-circle
distance on the same sphere."""

import numpy as np
import shapely

EARTH_RADIUS_M = 6_371_000.0


class Enu:
    """Local ENU metres (x east, y north) about a fixed origin."""

    def __init__(self, lat0_deg: float, lon0_deg: float):
        self.lat0_deg = lat0_deg
        self.lon0_deg = lon0_deg
        self._kx = EARTH_RADIUS_M * np.cos(np.radians(lat0_deg))

    def xy(self, lon_deg, lat_deg):
        x = self._kx * np.radians(np.asarray(lon_deg) - self.lon0_deg)
        y = EARTH_RADIUS_M * np.radians(np.asarray(lat_deg) - self.lat0_deg)
        return x, y

    def lonlat(self, x_m, y_m):
        lon = self.lon0_deg + np.degrees(np.asarray(x_m) / self._kx)
        lat = self.lat0_deg + np.degrees(np.asarray(y_m) / EARTH_RADIUS_M)
        return lon, lat

    def to_enu(self, geom: shapely.Geometry) -> shapely.Geometry:
        def f(coords):
            x, y = self.xy(coords[:, 0], coords[:, 1])
            return np.column_stack([x, y])

        return shapely.transform(geom, f)


def great_circle_m(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    """Haversine distance on the sphere of radius EARTH_RADIUS_M. Accepts arrays."""
    p1, p2 = np.radians(lat1_deg), np.radians(lat2_deg)
    dp = p2 - p1
    dl = np.radians(np.asarray(lon2_deg) - np.asarray(lon1_deg))
    h = np.sin(dp / 2) ** 2 + np.cos(p1) * np.cos(p2) * np.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * np.arcsin(np.sqrt(np.clip(h, 0.0, 1.0)))
