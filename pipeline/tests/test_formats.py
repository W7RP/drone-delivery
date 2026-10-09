"""Parsers and format rules where a mistake would be silent: the spec 9 binary layouts, OSM
speed and one-way tags, and pad names."""

import struct
import unittest
from types import SimpleNamespace

import numpy as np

from ddpipe import binfmt
from ddpipe.pads import config_name
from ddpipe.truck import direction, parse_maxspeed


def lane(a, b, length, terrain=300.0, risk=1000.0):
    return SimpleNamespace(a=a, b=b, length_m=length, max_terrain_m=terrain,
                           ground_risk_per_km2=risk)


TRIANGLE = [lane(0, 1, 1000.0), lane(1, 2, 2000.5), lane(0, 2, 2500.25)]


class LanesFile(unittest.TestCase):
    def test_round_trip(self):
        data = binfmt.write_lanes(binfmt.lane_graph(3, TRIANGLE), 0xABCDEF)
        g, h = binfmt.read_lanes(data)
        self.assertEqual(h, 0xABCDEF)
        self.assertEqual(list(g.row_offsets), [0, 2, 4, 6])
        self.assertEqual(list(g.to_pad), [1, 2, 0, 2, 0, 1])
        self.assertEqual(list(g.reverse_lane), [2, 4, 0, 5, 1, 3])
        self.assertEqual(float(g.length_m[1]), 2500.25)

    def test_arrays_start_on_8_byte_boundaries(self):
        data = binfmt.write_lanes(binfmt.lane_graph(3, TRIANGLE), 0)
        # 32 header + 16 row_offsets, to_pad (12 B) padded to 16, then f32 lengths at 64.
        self.assertEqual(struct.unpack_from("<f", data, 64)[0], 1000.0)

    def test_length_rounds_up_to_f32(self):
        x = 1000.00001
        self.assertGreaterEqual(float(binfmt.f32_at_least([x])[0]), x)

    def corrupt(self, offset, fmt, value):
        data = bytearray(binfmt.write_lanes(binfmt.lane_graph(3, TRIANGLE), 0))
        struct.pack_into(fmt, data, offset, value)
        return bytes(data)

    def test_rejects(self):
        cases = {
            "magic": self.corrupt(0, "<8s", b"DDTRUCK\0"),
            "major": self.corrupt(8, "<H", 2),
            "minor 0 lacks terrain": self.corrupt(10, "<H", 0),
            "reverse": self.corrupt(88, "<I", 1),
            "nan length": self.corrupt(64, "<f", float("nan")),
            "unequal reverse length": self.corrupt(64, "<f", 999.0),
        }
        for name, data in cases.items():
            with self.subTest(name), self.assertRaises(binfmt.FormatError):
                binfmt.read_lanes(data)
        whole = binfmt.write_lanes(binfmt.lane_graph(3, TRIANGLE), 0)
        with self.assertRaises(binfmt.FormatError):
            binfmt.read_lanes(whole[:-4])
        with self.assertRaises(binfmt.FormatError):
            binfmt.read_lanes(whole + bytes(8))

    def test_higher_minor_may_append(self):
        data = bytearray(binfmt.write_lanes(binfmt.lane_graph(3, TRIANGLE), 0))
        struct.pack_into("<H", data, 10, 2)
        binfmt.read_lanes(bytes(data) + bytes(16))


class TruckFile(unittest.TestCase):
    FF = np.array([[0, 60], [70, 0]], dtype=float)
    ROAD = np.array([[0, 900], [950, 0]], dtype=float)
    # Off-diagonal 0.5 motorway, 0.25 primary, 0.25 other; 0.1 on a bridge.
    CLASSES = np.array([[[0, 0.5], [0.5, 0]], [[0, 0.25], [0.25, 0]], [[0, 0.25], [0.25, 0]]])
    STRUCTURE = np.array([[0, 0.1], [0.1, 0]])

    def write(self, ff=FF, classes=CLASSES):
        return binfmt.write_truck(ff, self.ROAD, classes, self.STRUCTURE, 7)

    def test_round_trip_and_rejects(self):
        got, h = binfmt.read_truck(self.write())
        self.assertEqual(h, 7)
        np.testing.assert_array_equal(got.free_flow_s, self.FF)
        np.testing.assert_array_equal(got.road_m, self.ROAD)
        np.testing.assert_array_equal(got.class_frac, self.CLASSES.astype(np.float32))
        for bad in (np.array([[0, np.inf], [70, 0]]), np.array([[1, 60], [70, 0]]),
                    np.array([[0, 0], [70, 0]])):
            with self.subTest(bad=bad), self.assertRaises(binfmt.FormatError):
                binfmt.read_truck(self.write(ff=bad))
        short = self.CLASSES.copy()
        short[2, 0, 1] = 0.1
        with self.assertRaises(binfmt.FormatError):
            binfmt.read_truck(self.write(classes=short))

    def test_minor_1_file_still_reads(self):
        old = binfmt._pack(binfmt.TRUCK_MAGIC, 1, 2, 0, 7,
                           [self.FF.astype(np.float32).ravel(), self.ROAD.astype(np.float32).ravel()])
        got, _ = binfmt.read_truck(old)
        self.assertIsNone(got.class_frac)
        np.testing.assert_array_equal(got.road_m, self.ROAD)


class OsmTags(unittest.TestCase):
    def test_maxspeed(self):
        self.assertAlmostEqual(parse_maxspeed("25 mph"), 25 * 0.44704)
        self.assertAlmostEqual(parse_maxspeed("50"), 50 / 3.6)
        self.assertAlmostEqual(parse_maxspeed("35 mph;25 mph"), 25 * 0.44704)
        for text in (None, "", "signals", "none", "0", "500 mph"):
            self.assertIsNone(parse_maxspeed(text))

    def test_oneway(self):
        self.assertEqual(direction({"highway": "residential"}), (True, True))
        self.assertEqual(direction({"oneway": "yes"}), (True, False))
        self.assertEqual(direction({"oneway": "-1"}), (False, True))
        self.assertEqual(direction({"junction": "roundabout"}), (True, False))
        self.assertEqual(direction({"highway": "motorway"}), (True, False))
        self.assertEqual(direction({"highway": "motorway", "oneway": "no"}), (True, True))


class PadNames(unittest.TestCase):
    def test_config_name(self):
        taken: set[str] = set()
        self.assertEqual(config_name("UPMC Magee-Womens Hospital", taken), "UPMC_Magee_Womens_Hospital")
        self.assertEqual(config_name("84 Lumber", taken), "P_84_Lumber")
        long = config_name("Mon Valley Works - Edgar Thomson Plant", taken)
        self.assertLessEqual(len(long), 32)
        taken.add(long)
        self.assertNotEqual(config_name("Mon Valley Works - Edgar Thomson Plant", taken), long)


class Hash(unittest.TestCase):
    def test_fnv1a64_reference_values(self):
        self.assertEqual(binfmt.fnv1a64(b""), 0xCBF29CE484222325)
        self.assertEqual(binfmt.fnv1a64(b"a"), 0xAF63DC4C8601EC8C)


if __name__ == "__main__":
    unittest.main()
