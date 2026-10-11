"""Corridor world rules where a mistake would be silent: heights read from OSM tags, the terrain
surface that roads and buildings sit on, and triangle winding (a clockwise face is culled and
vanishes from the clip)."""

import unittest

import numpy as np
import shapely

from hero.world import Grid, building_height, flat_triangles, parse_height_m

CFG = {"level_height_m": 3.0, "default_height_m": 8.0}


def ccw(tri: np.ndarray) -> bool:
    (x0, y0), (x1, y1), (x2, y2) = tri
    return (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0) > 0


class Heights(unittest.TestCase):
    def test_units(self):
        self.assertEqual(parse_height_m("12"), 12.0)
        self.assertEqual(parse_height_m("12.5 m"), 12.5)
        self.assertAlmostEqual(parse_height_m("40 ft"), 12.192)
        self.assertAlmostEqual(parse_height_m("40'"), 12.192)
        for bad in ("", "tall", "12;14", "0", "-3"):
            self.assertIsNone(parse_height_m(bad), bad)

    def test_fallbacks(self):
        self.assertEqual(building_height({"height": "20", "building:levels": "2"}, CFG), (20.0, "height"))
        self.assertEqual(building_height({"height": "n/a", "building:levels": "4"}, CFG), (12.0, "levels"))
        self.assertEqual(building_height({"building:levels": "x"}, CFG), (8.0, "default"))
        self.assertEqual(building_height({}, CFG), (8.0, "default"))


class Surface(unittest.TestCase):
    # z = 10 at (0, 0), 20 at (10, 0), 30 at (0, 10), 0 at (10, 10): not a plane, so the
    # diagonal the cell is split along matters.
    GRID = Grid(0.0, 0.0, 10.0, np.array([[10.0, 20.0], [30.0, 0.0]]), np.ones((1, 1), bool))

    def test_vertices(self):
        z = self.GRID.surface([0, 10, 0, 10], [0, 0, 10, 10])
        # The far edge is clamped 1e-9 cells inside the grid.
        np.testing.assert_allclose(z, [10, 20, 30, 0], atol=1e-6)

    def test_on_the_mesh_triangles(self):
        # Below the (0,0)-(10,10) diagonal the face is (0,0), (10,0), (10,10).
        self.assertAlmostEqual(float(self.GRID.surface(7.5, 2.5)), 10 + 0.75 * 10 + 0.25 * -20)
        # Above it, (0,0), (10,10), (0,10).
        self.assertAlmostEqual(float(self.GRID.surface(2.5, 7.5)), 10 + 0.75 * 20 + 0.25 * -30)
        # On the diagonal both faces agree: halfway between 10 and 0.
        self.assertAlmostEqual(float(self.GRID.surface(5, 5)), 5.0)


class Winding(unittest.TestCase):
    def test_polygon_with_hole_is_counter_clockwise(self):
        cw_outer = shapely.Polygon([(0, 0), (0, 10), (10, 10), (10, 0)], [[(4, 4), (6, 4), (6, 6), (4, 6)]])
        tris = flat_triangles(cw_outer)
        self.assertTrue(all(ccw(t) for t in tris))
        area = sum(abs(shapely.Polygon(t).area) for t in tris)
        self.assertAlmostEqual(area, 96.0)


if __name__ == "__main__":
    unittest.main()
