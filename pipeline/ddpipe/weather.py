"""Hourly weather series (spec 9.7) from IEM ASOS routine reports: one value per UTC hour for
wind, gust, temperature, visibility, 1 h precipitation and present weather, converted to SI.

Each report goes to the nearest whole hour; when two land on one hour the one closer to it
wins (the earlier on a tie). Values a report cannot have are dropped as missing (wind at
direction 0 with a speed, or out of range). A field still missing takes the last value for at
most `carry_forward_h` hours; past that the hour is marked missing and keeps the last value, so
the sim always has numbers to fly with. No gust group means no gusts (gust = wind), no 1 h
precip group or a trace means 0, and a variable wind (speed without direction) keeps the last
direction without limit, since it has none of its own."""

import csv
import datetime as dt
import io
import json
import logging
from dataclasses import dataclass

import numpy as np

from . import binfmt
from .params import q

log = logging.getLogger(__name__)

KNOT_MPS = 0.514444
MILE_M = 1609.344
INCH_MM = 25.4
MAX_WIND_KT = 120.0
MAX_VISIBILITY_MI = 100.0
HOUR_S = 3600
TIME_FORMAT = "%Y-%m-%d %H:%M"

# Present-weather phenomena (two-letter METAR codes) behind each flag. Vicinity codes (VC...)
# are not at the station and are skipped; FG, BR and HZ show up in visibility instead.
WX_CODES = {
    binfmt.WX_RAIN: {"RA", "DZ", "UP"},
    binfmt.WX_SNOW: {"SN", "SG", "PL", "GS", "GR", "IC"},
    binfmt.WX_FREEZING: {"FZ"},
    binfmt.WX_THUNDER: {"TS"},
}


def wx_flags(text: str) -> int:
    """Flag bits for an IEM `wxcodes` field such as "-TSRA BR"."""
    flags = 0
    for token in text.split():
        body = token.lstrip("+-")
        if body.startswith("VC"):
            continue
        parts = {body[i:i + 2] for i in range(0, len(body) - 1, 2)}
        for bit, codes in WX_CODES.items():
            if parts & codes:
                flags |= bit
    if flags & binfmt.WX_FREEZING and not flags & binfmt.WX_RAIN:
        # Freezing fog (FZFG) is not precipitation.
        flags &= ~binfmt.WX_FREEZING
    return flags


@dataclass
class Counts:
    reports: int = 0
    reports_outside_span: int = 0
    reports_not_nearest: int = 0
    hours_without_report: int = 0
    invalid_values: int = 0
    variable_wind_hours: int = 0
    carried_hours: int = 0
    missing_hours: int = 0


def _number(text: str, counts: Counts, high: float | None = None, low: float = 0.0):
    if text == "M":
        return np.nan
    value = float(text)
    if not np.isfinite(value) or value < low or (high is not None and value > high):
        counts.invalid_values += 1
        return np.nan
    return value


def _carry(a: np.ndarray, hours: int) -> np.ndarray:
    """Fill NaN runs with the last finite value for at most `hours` steps."""
    out = a.copy()
    last, age = np.nan, 0
    for i, v in enumerate(a):
        if np.isfinite(v):
            last, age = v, 0
        else:
            age += 1
            if age <= hours:
                out[i] = last
    return out


def _hold(a: np.ndarray) -> np.ndarray:
    """Fill every NaN with the last finite value, or the first one before any."""
    finite = np.flatnonzero(np.isfinite(a))
    if finite.size == 0:
        raise ValueError("no observed value at all")
    idx = np.maximum.accumulate(np.where(np.isfinite(a), np.arange(len(a)), -1))
    idx[idx < 0] = finite[0]
    return a[idx]


def parse(text: str, start: dt.datetime, end: dt.datetime, carry_h: int):
    """The hourly series over [start, end) UTC from IEM CSV text, and what filling it took."""
    n = int((end - start).total_seconds()) // HOUR_S
    counts = Counts()
    best: dict[int, tuple[float, dt.datetime, dict]] = {}
    for row in csv.DictReader(io.StringIO(text)):
        counts.reports += 1
        t = dt.datetime.strptime(row["valid"], TIME_FORMAT)
        offset_s = (t - start).total_seconds()
        hour = int(np.floor(offset_s / HOUR_S + 0.5))
        if not 0 <= hour < n:
            counts.reports_outside_span += 1
            continue
        key = (abs(offset_s - hour * HOUR_S), t)
        if hour in best:
            counts.reports_not_nearest += 1
            if best[hour][:2] <= key:
                continue
        best[hour] = (*key, row)
    counts.hours_without_report = n - len(best)

    fields = {k: np.full(n, np.nan) for k in ("wind", "dir", "gust", "temp", "vis", "precip", "wx")}
    for hour, (_, _, row) in best.items():
        wind = _number(row["sknt"], counts, MAX_WIND_KT)
        direction = _number(row["drct"], counts, 360.0)
        if direction == 0 and wind > 0:
            # Direction 0 is reserved for calm; a speed with it is a bad report.
            counts.invalid_values += 1
            wind = direction = np.nan
        if np.isnan(direction) and wind > 0:
            counts.variable_wind_hours += 1
        gust = _number(row["gust"], counts, MAX_WIND_KT)
        fields["wind"][hour] = wind * KNOT_MPS
        fields["dir"][hour] = direction % 360.0
        fields["gust"][hour] = (wind if np.isnan(gust) else gust) * KNOT_MPS
        tmpf = _number(row["tmpf"], counts, high=binfmt.TEMP_MAX_C * 9 / 5 + 32,
                       low=binfmt.TEMP_MIN_C * 9 / 5 + 32)
        fields["temp"][hour] = (tmpf - 32.0) * 5.0 / 9.0
        fields["vis"][hour] = _number(row["vsby"], counts, MAX_VISIBILITY_MI) * MILE_M
        precip = row["p01i"]
        fields["precip"][hour] = 0.0 if precip in ("M", "T") else _number(precip, counts) * INCH_MM
        fields["wx"][hour] = 0 if row["wxcodes"] == "M" else wx_flags(row["wxcodes"])

    observed = np.isfinite(fields["wind"]) & np.isfinite(fields["temp"]) & np.isfinite(fields["vis"])
    variable = np.isnan(fields["dir"]) & np.isfinite(fields["wind"])
    fields["dir"][variable] = _hold(fields["dir"])[variable]
    carried = {k: _carry(v, carry_h) for k, v in fields.items()}
    missing = np.zeros(n, dtype=bool)
    for v in carried.values():
        missing |= np.isnan(v)
    counts.carried_hours = int(np.sum(~observed & ~missing))
    counts.missing_hours = int(missing.sum())
    held = {k: _hold(v) for k, v in carried.items()}
    series = binfmt.WeatherSeries(
        start_unix_h=int(start.replace(tzinfo=dt.UTC).timestamp()) // HOUR_S,
        wind_mps=held["wind"],
        wind_from_deg=held["dir"],
        gust_mps=np.maximum(held["gust"], held["wind"]),
        temp_c=held["temp"],
        visibility_m=held["vis"],
        precip_mm_h=held["precip"],
        wx_flags=held["wx"].astype(np.uint8),
        missing=missing.astype(np.uint8),
    )
    return series, counts


def _summary(w: binfmt.WeatherSeries) -> dict:
    def pct(a, p):
        return round(float(np.percentile(a, p)), 2)

    flags = w.wx_flags
    return {
        "wind_10m_mps": {"p50": pct(w.wind_mps, 50), "p90": pct(w.wind_mps, 90),
                         "max": round(float(w.wind_mps.max()), 2)},
        "gust_10m_mps": {"p90": pct(w.gust_mps, 90), "max": round(float(w.gust_mps.max()), 2)},
        "temp_c": {"min": round(float(w.temp_c.min()), 2), "p50": pct(w.temp_c, 50),
                   "max": round(float(w.temp_c.max()), 2)},
        "hours_visibility_below_1600_m": int(np.sum(w.visibility_m < MILE_M)),
        "hours_precip_above_0": int(np.sum(w.precip_mm_h > 0)),
        "hours_by_flag": {name: int(np.sum(flags & bit > 0)) for name, bit in
                          (("rain", binfmt.WX_RAIN), ("snow", binfmt.WX_SNOW),
                           ("freezing", binfmt.WX_FREEZING), ("thunder", binfmt.WX_THUNDER))},
    }


def build(params: dict, sources: list[dict], manifest: dict, raw_dir, out_dir,
          revision: str) -> None:
    """Write weather.bin, report.json and manifest.json for the chosen station into out_dir."""
    cfg = params["weather"]
    start = dt.datetime.fromisoformat(cfg["start_utc"])
    end = dt.datetime.fromisoformat(cfg["end_utc"])
    carry_h = int(q(cfg["carry_forward_h"], "h"))
    max_missing = q(cfg["max_missing_fraction"], "1")
    by_name = {s["name"]: s for s in sources}
    tried = []
    for name in (cfg["primary_source"], cfg["fallback_source"]):
        raw = (raw_dir / by_name[name]["file"]).read_bytes()
        series, counts = parse(raw.decode("ascii"), start, end, carry_h)
        fraction = counts.missing_hours / len(series.wind_mps)
        log.info("%s: %d reports, %d hours carried forward, %d missing (%.2f%%), %d invalid values",
                 name, counts.reports, counts.carried_hours, counts.missing_hours, 100 * fraction,
                 counts.invalid_values)
        tried.append({"source": name, "counts": vars(counts), "missing_fraction": round(fraction, 5)})
        if fraction <= max_missing:
            break
    else:
        raise RuntimeError(f"no weather station has at most {max_missing:.0%} missing hours")
    source_hash = binfmt.fnv1a64(raw)
    data = binfmt.write_weather(series, source_hash)
    binfmt.read_weather(data)
    report = {
        "pipeline_git": revision,
        "source": name,
        "span_utc": [cfg["start_utc"], cfg["end_utc"]],
        "hours": len(series.wind_mps),
        "stations_tried": tried,
        "summary": _summary(series),
        "assumptions": [
            "The station is not at the lanes: one series stands for every pad and lane (spec A76).",
            "Wind and gust are at the 10 m anemometer; the sim scales them to cruise height.",
        ],
    }
    files = {
        "weather.bin": data,
        "report.json": (json.dumps(report, indent=1, sort_keys=True) + "\n").encode(),
        "manifest.json": (json.dumps({"pipeline_git": revision,
                                      "sources": {name: manifest["files"][name]}},
                                     indent=1, sort_keys=True) + "\n").encode(),
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    for file, content in files.items():
        tmp = out_dir / (file + ".part")
        tmp.write_bytes(content)
        tmp.rename(out_dir / file)
    log.info("weather: %s, %d hours from %s, weather.bin %d B (source_hash %016x)", name,
             len(series.wind_mps), cfg["start_utc"], len(data), source_hash)
