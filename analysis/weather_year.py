# /// script
# requires-python = ">=3.10"
# dependencies = ["tomli==2.0.1", "tomli-w==1.0.0"]
# ///
"""Phase 6b step 7: the demo scenario over every complete week of the weather series, one
command.

Base: configs/parts_logistics_pittsburgh_demo (900 Wh layout, every realism key on). A week
starts at local midnight on a Monday (America/New_York) and runs 168 hours of the series; the
demo's `sim.start_time_of_day_s` is 0 and `traffic.start_weekday` Monday. Every week that fits
in the series runs with seeds 1 to 3 under each hold policy (WAIT, TRUCK_FALLBACK). Drone and
truck see the same orders: the truck time is each order's baseline (spec 7) on the same run.

Per run, over SHIPMENT orders requested in the window and completed (flown and DELIVERED, or
TRUCKED): ratio = mean door-to-door of the network / mean truck baseline. Orders not completed
(still open at the end, diverted, failed) and requests rejected at the backlog cap are not in
the ratio; the completed share reports them. A week's value is the mean over its seeds; mean,
p10 and p90 are over weeks.

(a) the year table per policy; (b) three weeks chosen by rule: the week of median mean wind,
the windiest (highest mean wind) and the coldest (lowest mean temperature), each with the
door-to-door split next to the truck; (c) one-at-a-time sensitivity of the ratio (WAIT) to
+-50% on the wind limits, the cold-derate depth (1 - factor), the landing-error gust term k
and the traffic peak delays.

Writes analysis/out/weather_year.json and weather_year.md. Every run's outputs are deleted once
read. Run: uv run analysis/weather_year.py"""

import csv
import datetime as dt
import json
import os
import shutil
import statistics
import struct
import subprocess
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from zoneinfo import ZoneInfo

from variants import CONFIGS, OUT, SIM, build_sim, variant

BASE = "parts_logistics_pittsburgh_demo"
SERIES = CONFIGS / "weather" / "pittsburgh" / "weather.bin"
LOCAL = ZoneInfo("America/New_York")
SEEDS = [1, 2, 3]
POLICIES = ["WAIT", "TRUCK_FALLBACK"]
WEEK_H = 168
RUN_TIMEOUT_S = 60
MAX_WORKERS = 16
SCALES = [0.5, 1.5]
SNOW_BIT = 2
HEADER = struct.Struct("<8sHHIIIQ")


def read_series() -> dict:
    """weather.bin (spec 9.7): start hour and the per-hour arrays this script needs."""
    data = SERIES.read_bytes()
    _, _, _, start, n, start_unix_h, _ = HEADER.unpack_from(data)
    at, arrays = start, {}
    for name, fmt in [("wind", "f"), ("dir", "f"), ("gust", "f"), ("temp", "f"), ("vis", "f"),
                      ("precip", "f"), ("flags", "B"), ("missing", "B")]:
        at += -at % 8
        size = struct.calcsize(fmt) * n
        arrays[name] = struct.unpack_from(f"<{n}{fmt}", data, at)
        at += size
    arrays["start_unix_h"] = start_unix_h
    arrays["n"] = n
    return arrays


def weeks(series: dict) -> list[dict]:
    """Every local-Monday-midnight week whose 168 hours fit in the series, with its weather."""
    start = dt.datetime.fromtimestamp(series["start_unix_h"] * 3600, dt.timezone.utc)
    day = start.astimezone(LOCAL).date()
    day += dt.timedelta(days=(7 - day.weekday()) % 7)
    out = []
    while True:
        local = dt.datetime.combine(day, dt.time(0), LOCAL)
        hour = int((local.astimezone(dt.timezone.utc) - start).total_seconds()) // 3600
        if hour + WEEK_H > series["n"]:
            return out
        if hour >= 0:
            span = slice(hour, hour + WEEK_H)
            out.append({
                "monday": day.isoformat(), "start_hour": hour,
                "mean_wind_mps": round(statistics.fmean(series["wind"][span]), 3),
                "max_gust_mps": round(max(series["gust"][span]), 2),
                "mean_temp_c": round(statistics.fmean(series["temp"][span]), 2),
                "min_temp_c": round(min(series["temp"][span]), 2),
                "snow_hours": sum(1 for f in series["flags"][span] if f & SNOW_BIT),
            })
        day += dt.timedelta(days=7)


def read_run(out: Path, window_ms) -> dict:
    m = json.loads((out / "metrics.json").read_text())
    done, flown = [], []
    with open(out / "deliveries.csv") as f:
        for r in csv.DictReader(f):
            if r["kind"] != "SHIPMENT" or not window_ms[0] <= int(r["requested_t_ms"]) < window_ms[1]:
                continue
            if r["status"] in ("DELIVERED", "TRUCKED"):
                done.append(r)
            if r["status"] == "DELIVERED":
                flown.append(r)
    mean = lambda rows, k: statistics.fmean(float(r[k]) for r in rows) if rows else None
    network, truck = mean(done, "drone_door_to_door_s"), mean(done, "truck_door_to_door_s")
    return {
        "ratio": network / truck if done else None,
        "network_s": network,
        "truck_s": truck,
        "completed_share": len(done) / m["shipment_requests_in_window"],
        "flown_truck_s": mean(flown, "truck_door_to_door_s"),
        "split_s": {k: mean(flown, k) for k in ("pod_wait_s", "queue_s", "pad_service_s", "flight_s")},
        "flyable": m["weather"]["flyable_hour_fraction"],
        "trucked_share": m["weather"]["trucked_share"],
        "weather_aborts": m["weather"]["weather_aborts"],
        "forced_landings": m["alerts"]["DRONE_GROUNDED"],
    }


def tree_bytes(root: Path) -> int:
    """Bytes under `root`; files other runs delete meanwhile count as gone."""
    total = 0
    for folder, _, files in os.walk(root):
        for f in files:
            try:
                total += os.stat(os.path.join(folder, f)).st_size
            except FileNotFoundError:
                pass
    return total


class Runner:
    """Runs scenarios in parallel, deletes each run's outputs once read, tracks peak temp use."""

    def __init__(self, root: Path):
        self.root, self.peak_bytes = root, 0

    def one(self, scenario: Path, seed: int, window_ms) -> dict:
        out = scenario.parent / f"out_{seed}"
        try:
            done = subprocess.run([str(SIM), "run", str(scenario), "--seed", str(seed), "--out",
                                   str(out), "--no-event-log"],
                                  capture_output=True, text=True, timeout=RUN_TIMEOUT_S)
            if done.returncode != 0:
                raise RuntimeError(f"{scenario} seed {seed}: {done.stderr.strip()}")
            self.peak_bytes = max(self.peak_bytes, tree_bytes(self.root))
            return read_run(out, window_ms)
        finally:
            shutil.rmtree(out, ignore_errors=True)

    def many(self, jobs: list[tuple[Path, int]], window_ms) -> list[dict]:
        workers = min(MAX_WORKERS, os.cpu_count() or 1)
        with ThreadPoolExecutor(workers) as pool:
            return list(pool.map(lambda j: self.one(j[0], j[1], window_ms), jobs))


def over_seeds(runs: list[dict]) -> dict:
    """Mean over seeds of every number; None where a seed completed no order."""
    def avg(values):
        return None if None in values else statistics.fmean(values)
    out = {k: avg([r[k] for r in runs]) for k in runs[0] if k != "split_s"}
    out["split_s"] = {k: avg([r["split_s"][k] for r in runs]) for k in runs[0]["split_s"]}
    return out


def summary(values: list) -> dict:
    """Mean, p10, median, p90 over weeks, ignoring weeks without a value (counted)."""
    v = sorted(x for x in values if x is not None)
    q = statistics.quantiles(v, n=10) if len(v) >= 2 else [None] * 9
    return {"mean": statistics.fmean(v) if v else None, "p10": q[0], "p50": q[4], "p90": q[8],
            "weeks": len(v), "weeks_without_value": len(values) - len(v)}


def sweep(runner: Runner, root: Path, tag: str, wks: list[dict], edits: dict, window_ms) -> list[dict]:
    """Per week, the mean over seeds of one variant of the demo."""
    jobs = []
    for w in wks:
        e = {**edits, "weather.start_hour": w["start_hour"]}
        name = f"{tag}_{w['start_hour']}"
        path = variant(BASE, root, name, e)
        jobs += [(path, s) for s in SEEDS]
    runs = runner.many(jobs, window_ms)
    per_week = [over_seeds(runs[i * len(SEEDS):(i + 1) * len(SEEDS)]) for i in range(len(wks))]
    shutil.rmtree(root, ignore_errors=True)
    root.mkdir()
    return per_week


def sensitivity_edits(cfg: dict) -> dict[str, dict]:
    w, tr = cfg["weather"], cfg["traffic"]
    out = {}
    for s in SCALES:
        out[f"wind limits x{s}"] = {
            f"weather.{k}": w[k]["value"] * s
            for k in ("max_wind_mps", "max_gust_mps", "landing_max_wind_mps", "landing_max_gust_mps")}
        out[f"cold derate depth x{s}"] = {"weather.battery_derate_factor": [
            {**f, "value": 1 - (1 - f["value"]) * s} for f in w["battery_derate_factor"]]}
        out[f"landing gust k x{s}"] = {"weather.latch_gust_k": w["latch_gust_k"]["value"] * s}
        out[f"traffic peak delay x{s}"] = {
            f"traffic.{k}": tr[k]["value"] * s
            for k in ("peak_delay_motorway", "peak_delay_primary", "peak_delay_other",
                      "peak_delay_structure")}
    return out


def fmt(x, nd=2, pct=False):
    if x is None:
        return "-"
    return f"{100 * x:.0f}%" if pct else f"{x:.{nd}f}"


def main() -> None:
    import tomli
    build_sim()
    started = time.monotonic()
    cfg = tomli.loads((CONFIGS / BASE / "scenario.toml").read_text())
    window_ms = (cfg["sim"]["warmup_s"]["value"] * 1000, cfg["sim"]["duration_s"]["value"] * 1000)
    wks = weeks(read_series())
    result = {"base": BASE, "seeds": SEEDS, "weeks": wks, "policies": {}, "sensitivity": {}}
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp) / "runs"
        root.mkdir()
        runner = Runner(Path(tmp))
        for policy in POLICIES:
            per_week = sweep(runner, root, policy, wks, {"weather.hold_policy": policy}, window_ms)
            result["policies"][policy] = {
                "per_week": per_week,
                "ratio": summary([p["ratio"] for p in per_week]),
                "flyable": summary([p["flyable"] for p in per_week]),
                "trucked_share": summary([p["trucked_share"] for p in per_week]),
                "completed_share": summary([p["completed_share"] for p in per_week]),
                "forced_landings_total": sum(p["forced_landings"] for p in per_week) * len(SEEDS),
            }
        base_ratio = result["policies"]["WAIT"]["ratio"]["mean"]
        for n, (label, edits) in enumerate(sensitivity_edits(cfg).items()):
            per_week = sweep(runner, root, f"sens{n}", wks,
                             {"weather.hold_policy": "WAIT", **edits}, window_ms)
            r = summary([p["ratio"] for p in per_week])
            result["sensitivity"][label] = {
                "ratio": r, "change": None if r["mean"] is None else r["mean"] - base_ratio,
                "completed_share": summary([p["completed_share"] for p in per_week])["mean"]}
        peak_bytes = runner.peak_bytes
    pick = named_weeks(wks)
    result["named_weeks"] = pick
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "weather_year.json").write_text(json.dumps(result, indent=1) + "\n")
    (OUT / "weather_year.md").write_text(report(result, wks, pick))
    # Wall time and temp use vary run to run, so they are printed, not written.
    print(report(result, wks, pick))
    print(f"wall {time.monotonic() - started:.0f} s, peak temp {peak_bytes / 1e6:.1f} MB")


def named_weeks(wks: list[dict]) -> dict[str, int]:
    """Indices of the median-wind, windiest and coldest weeks (earliest on a tie)."""
    by_wind = sorted(range(len(wks)), key=lambda i: (wks[i]["mean_wind_mps"], i))
    return {"median wind": by_wind[len(by_wind) // 2],
            "windiest": max(range(len(wks)), key=lambda i: (wks[i]["mean_wind_mps"], -i)),
            "coldest": min(range(len(wks)), key=lambda i: (wks[i]["mean_temp_c"], i))}


def report(r: dict, wks: list[dict], pick: dict) -> str:
    lines = [f"{len(wks)} weeks ({wks[0]['monday']} to {wks[-1]['monday']}), seeds {r['seeds']}.", "",
             "| Policy | Ratio mean | p10 | Median | p90 | Flyable hours | Trucked share | Completed share | Weeks without a completed order |",
             "|---|---|---|---|---|---|---|---|---|"]
    for name, p in r["policies"].items():
        ra = p["ratio"]
        lines.append(f"| {name} | {fmt(ra['mean'])} | {fmt(ra['p10'])} | {fmt(ra['p50'])} | {fmt(ra['p90'])} | "
                     f"{fmt(p['flyable']['mean'], pct=True)} | {fmt(p['trucked_share']['mean'], pct=True)} | "
                     f"{fmt(p['completed_share']['mean'], pct=True)} | {ra['weeks_without_value']} |")
    lines += ["", "| Week | Rule | Wind mean (m/s) | Gust max | Temp mean (C) | Snow h | Policy | Network (s) | Pod wait | Queue | Pad service | Flight | Truck, same flown orders (s) | Ratio | Flyable | Trucked | Completed |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for rule, i in pick.items():
        w = wks[i]
        for name, p in r["policies"].items():
            x = p["per_week"][i]
            s = x["split_s"]
            lines.append(
                f"| {w['monday']} | {rule} | {w['mean_wind_mps']:.1f} | {w['max_gust_mps']:.1f} | "
                f"{w['mean_temp_c']:.1f} | {w['snow_hours']} | {name} | {fmt(x['network_s'], 0)} | "
                f"{fmt(s['pod_wait_s'], 0)} | {fmt(s['queue_s'], 0)} | {fmt(s['pad_service_s'], 0)} | "
                f"{fmt(s['flight_s'], 0)} | {fmt(x['flown_truck_s'], 0)} | {fmt(x['ratio'])} | "
                f"{fmt(x['flyable'], pct=True)} | {fmt(x['trucked_share'], pct=True)} | "
                f"{fmt(x['completed_share'], pct=True)} |")
    lines += ["", "| Sensitivity (WAIT) | Ratio mean | Change | p10 | Median | p90 | Completed share |",
              "|---|---|---|---|---|---|---|"]
    for label, s in sorted(r["sensitivity"].items(), key=lambda kv: -abs(kv[1]["change"] or 0)):
        lines.append(f"| {label} | {fmt(s['ratio']['mean'])} | {fmt(s['change'], 3)} | "
                     f"{fmt(s['ratio']['p10'])} | {fmt(s['ratio']['p50'])} | {fmt(s['ratio']['p90'])} | "
                     f"{fmt(s['completed_share'], pct=True)} |")
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    main()
