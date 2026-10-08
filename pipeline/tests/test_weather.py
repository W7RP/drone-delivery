"""The hourly weather series (spec 9.7): report binning, gap filling, unit conversion, the
present-weather flags and the reader's rejection rules."""

import datetime as dt
import struct
import unittest

import numpy as np

from ddpipe import binfmt
from ddpipe.weather import parse, wx_flags

HEADER = "station,valid,sknt,drct,gust,tmpf,vsby,p01i,wxcodes\n"
START = dt.datetime(2025, 1, 1)


def csv(*rows):
    return HEADER + "".join(f"AGC,{r}\n" for r in rows)


def series(text, hours=6, carry_h=3):
    return parse(text, START, START + dt.timedelta(hours=hours), carry_h)


class Binning(unittest.TestCase):
    def test_report_goes_to_the_nearest_hour_and_the_closer_one_wins(self):
        w, c = series(csv("2024-12-31 23:53,10.00,180.00,M,32.00,10.00,0.00,M",
                          "2025-01-01 00:20,20.00,180.00,M,32.00,10.00,0.00,M",
                          "2025-01-01 00:53,4.00,90.00,M,32.00,10.00,0.00,M"), hours=2)
        self.assertAlmostEqual(w.wind_mps[0], 10 * 0.514444, places=5)
        self.assertAlmostEqual(w.wind_mps[1], 4 * 0.514444, places=5)
        self.assertEqual(c.reports_not_nearest, 1)

    def test_si_conversion_and_defaults(self):
        w, _ = series(csv("2025-01-01 00:00,10.00,270.00,M,50.00,2.00,0.10,-SN BR"), hours=1)
        self.assertAlmostEqual(w.gust_mps[0], w.wind_mps[0])
        self.assertAlmostEqual(w.temp_c[0], 10.0, places=5)
        self.assertAlmostEqual(w.visibility_m[0], 2 * 1609.344, places=3)
        self.assertAlmostEqual(w.precip_mm_h[0], 2.54, places=5)
        self.assertEqual(w.wx_flags[0], binfmt.WX_SNOW)
        self.assertEqual(w.wind_from_deg[0], 270.0)


class GapFill(unittest.TestCase):
    ROWS = ("2025-01-01 00:00,10.00,180.00,M,32.00,10.00,0.00,M",
            "2025-01-01 05:00,4.00,90.00,M,40.00,10.00,0.00,M")

    def test_carries_forward_up_to_the_limit_then_marks_missing(self):
        w, c = series(csv(*self.ROWS), hours=6, carry_h=3)
        self.assertEqual(list(w.missing), [0, 0, 0, 0, 1, 0])
        # A missing hour keeps the last value, so the sim still has numbers.
        self.assertAlmostEqual(w.wind_mps[4], w.wind_mps[0])
        self.assertEqual((c.carried_hours, c.missing_hours, c.hours_without_report), (3, 1, 4))

    def test_a_missing_start_takes_the_first_report_and_is_marked(self):
        w, _ = series(csv("2025-01-01 05:00,4.00,90.00,M,40.00,10.00,0.00,M"), hours=6)
        self.assertEqual(list(w.missing), [1, 1, 1, 1, 1, 0])
        self.assertAlmostEqual(w.temp_c[0], w.temp_c[5])

    def test_variable_wind_keeps_the_last_direction_without_marking(self):
        rows = ["2025-01-01 00:00,5.00,200.00,M,32.00,10.00,0.00,M"] + [
            f"2025-01-01 0{h}:00,3.00,M,M,32.00,10.00,0.00,M" for h in range(1, 6)]
        w, c = series(csv(*rows), hours=6)
        self.assertTrue(np.all(w.wind_from_deg == 200.0))
        self.assertEqual(int(w.missing.sum()), 0)
        self.assertEqual(c.variable_wind_hours, 5)

    def test_speed_at_direction_zero_is_dropped(self):
        w, c = series(csv("2025-01-01 00:00,5.00,180.00,M,32.00,10.00,0.00,M",
                          "2025-01-01 01:00,90.00,0.00,M,32.00,10.00,0.00,M"), hours=2)
        self.assertAlmostEqual(w.wind_mps[1], w.wind_mps[0])
        self.assertEqual(c.invalid_values, 1)


class Flags(unittest.TestCase):
    def test_codes(self):
        self.assertEqual(wx_flags("-TSRA BR"), binfmt.WX_THUNDER | binfmt.WX_RAIN)
        self.assertEqual(wx_flags("VCTS HZ"), 0)
        self.assertEqual(wx_flags("FZFG"), 0)
        self.assertEqual(wx_flags("-FZRASN"),
                         binfmt.WX_FREEZING | binfmt.WX_RAIN | binfmt.WX_SNOW)
        self.assertEqual(wx_flags("UP"), binfmt.WX_RAIN)


class WeatherFile(unittest.TestCase):
    def data(self):
        w, _ = series(csv("2025-01-01 00:00,10.00,270.00,15.00,50.00,2.00,0.10,-RA"), hours=3)
        return w, binfmt.write_weather(w, 9)

    def test_round_trip(self):
        w, data = self.data()
        got, h = binfmt.read_weather(data)
        self.assertEqual((h, got.start_unix_h), (9, int(START.replace(tzinfo=dt.UTC).timestamp()) // 3600))
        np.testing.assert_array_equal(got.gust_mps, w.gust_mps.astype(np.float32))
        np.testing.assert_array_equal(got.missing, w.missing)

    def test_rejects(self):
        _, data = self.data()
        gust_at = 32 + 2 * 16  # header, then wind and direction, 12 B each padded to 16
        cases = {
            "magic": (0, "<8s", b"DDTRUCK\0"),
            "gust below wind": (gust_at, "<f", 0.0),
            "nan temperature": (gust_at + 16, "<f", float("nan")),
            "direction 360": (32 + 16, "<f", 360.0),
            "unknown flag": (32 + 6 * 16, "<B", 0x80),
        }
        for name, (offset, fmt, value) in cases.items():
            bad = bytearray(data)
            struct.pack_into(fmt, bad, offset, value)
            with self.subTest(name), self.assertRaises(binfmt.FormatError):
                binfmt.read_weather(bytes(bad))
        with self.assertRaises(binfmt.FormatError):
            binfmt.read_weather(data + bytes(8))
        with self.assertRaises(binfmt.FormatError):
            binfmt.read_weather(data[:-1])

if __name__ == "__main__":
    unittest.main()
