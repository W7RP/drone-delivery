# /// script
# requires-python = ">=3.10"
# dependencies = ["tomli==2.0.1", "tomli-w==1.0.0"]
# ///
"""Phase 7 step 13: completion-aware results of the VTOL demo against the multirotor demo, one
command.

Both demos run over every complete week of the weather series (the weeks of
analysis/weather_year.py), seeds 1 and 2, under both hold policies, with M11 order outcomes on.
The multirotor run is the 6b demo with only `metrics.order_outcomes` added (the 6b rerun,
completion-aware); the VTOL run is configs/parts_logistics_pittsburgh_vtol_demo with its
`weather.start_hour` and `traffic.start_unix_s` moved to each week's local Monday midnight.

A week pools its seeds (outcome counts and time sums added, as in analysis/completion.py), so
both ratio forms are order-weighted: ratio_delivered over delivered orders, ratio_lower_bound
with every unfinished order counted at its time to the end of the run (spec M11). Median, p10
and p90 are over weeks.

(a) the year per base and policy: both ratio forms, completion rate, outcome breakdown (share
of all requests in the year), flyable hours; (b) the three named weeks of weather_year.json with
the door-to-door split of drone-delivered orders next to their truck baseline; (c) the
comparison with the 6b numbers as published (weather_year.json), and per Mon Valley flow of the
VTOL demo (a flow to or from a pad the VTOL network has and the multirotor's lacks: Edgar Thomson,
Josh Steel, Clairton, Old Koppers), drone against truck over the year.

Writes analysis/out/vtol_year.json and vtol_year.md; every run's outputs are deleted once read.
Run: uv run analysis/vtol_year.py"""

import csv
import json
import os
import shutil
import subprocess
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import tomli

from variants import CONFIGS, OUT, SIM, build_sim, variant
from weather_year import read_series, summary, weeks

BASES = {"multirotor": "parts_logistics_pittsburgh_demo", "vtol": "parts_logistics_pittsburgh_vtol_demo"}
SEEDS = [1, 2]
POLICIES = ["WAIT", "TRUCK_FALLBACK"]
CLASSES = ["delivered_drone", "delivered_truck", "rejected_cap", "blocked_weather",
           "blocked_energy", "in_flight", "waiting", "diverted_or_failed"]
SUMS = ["delivered_network", "delivered_truck", "unfinished_elapsed", "unfinished_truck"]
SPLIT = ["pod_wait_s", "queue_s", "pad_service_s", "flight_s"]
RUN_TIMEOUT_S = 60
MAX_WORKERS = 16
S_PER_H = 3600


def pad_names(base: str) -> list[str]:
    cfg = tomli.loads((CONFIGS / base / "scenario.toml").read_text())
    pads = CONFIGS / base / cfg["network_files"]["dir"] / "pads.toml"
    return [p["name"] for p in tomli.loads(pads.read_text())["pads"]]


def read_run(out: Path, window_ms: tuple[float, float]) -> dict:
    """Order outcomes, flyable hours, and per-flow sums over the SHIPMENT orders requested in
    the window: drone-delivered (door-to-door, split, truck baseline) and trucked."""
    m = json.loads((out / "metrics.json").read_text())
    flows: dict[str, dict] = {}
    with open(out / "deliveries.csv") as f:
        for r in csv.DictReader(f):
            if r["kind"] != "SHIPMENT" or not window_ms[0] <= int(r["requested_t_ms"]) < window_ms[1]:
                continue
            if r["status"] not in ("DELIVERED", "TRUCKED"):
                continue
            k = f"{r['origin_pad']}>{r['dest_pad']}"
            s = flows.setdefault(k, {"drone_n": 0, "drone_s": 0.0, "drone_truck_s": 0.0, "trucked_n": 0,
                                     "trucked_s": 0.0, **{x: 0.0 for x in SPLIT}})
            if r["status"] == "DELIVERED":
                s["drone_n"] += 1
                s["drone_s"] += float(r["drone_door_to_door_s"])
                s["drone_truck_s"] += float(r["truck_door_to_door_s"])
                for x in SPLIT:
                    s[x] += float(r[x])
            else:
                s["trucked_n"] += 1
                s["trucked_s"] += float(r["drone_door_to_door_s"])
    return {"outcomes": m["order_outcomes"], "flyable": m["weather"]["flyable_hour_fraction"],
            "flows": flows}


def add_flows(into: dict, flows: dict) -> None:
    for k, s in flows.items():
        t = into.setdefault(k, dict.fromkeys(s, 0))
        for x, v in s.items():
            t[x] += v


def pooled(runs: list[dict]) -> dict:
    """Seeds pooled: counts and sums added, ratios of sums."""
    o = [r["outcomes"] for r in runs]
    p = {c: sum(x[c] for x in o) for c in ["requests", *CLASSES]}
    s = {k: sum(x["sums_s"][k] for x in o) for k in SUMS}
    p["sums_s"] = s
    p["completion_rate"] = (p["delivered_drone"] + p["delivered_truck"]) / p["requests"]
    p["ratio_delivered"] = s["delivered_network"] / s["delivered_truck"] if s["delivered_truck"] else None
    p["ratio_lower_bound"] = ((s["delivered_network"] + s["unfinished_elapsed"])
                              / (s["delivered_truck"] + s["unfinished_truck"]))
    p["flyable"] = sum(r["flyable"] for r in runs) / len(runs)
    p["flows"] = {}
    for r in runs:
        add_flows(p["flows"], r["flows"])
    return p


def run_one(scenario: Path, seed: int, window_ms) -> dict:
    out = scenario.parent / f"out_{seed}"
    try:
        done = subprocess.run([str(SIM), "run", str(scenario), "--seed", str(seed), "--out", str(out),
                               "--no-event-log"], capture_output=True, text=True, timeout=RUN_TIMEOUT_S)
        if done.returncode != 0:
            raise RuntimeError(f"{scenario} seed {seed}: {done.stderr.strip()}")
        return read_run(out, window_ms)
    finally:
        shutil.rmtree(out, ignore_errors=True)


def week_edits(kind: str, policy: str, week: dict, start_unix_h: int) -> dict:
    e = {"schema_version": "1.9.0", "metrics.order_outcomes": True,
         "weather.hold_policy": policy, "weather.start_hour": week["start_hour"]}
    if kind == "vtol":
        e["traffic.start_unix_s"] = float((start_unix_h + week["start_hour"]) * S_PER_H)
    return e


def year(root: Path, kind: str, policy: str, wks: list[dict], start_unix_h: int, window_ms) -> list[dict]:
    jobs = []
    for w in wks:
        path = variant(BASES[kind], root, f"{kind}_{policy}_{w['start_hour']}",
                       week_edits(kind, policy, w, start_unix_h))
        jobs += [(path, s) for s in SEEDS]
    with ThreadPoolExecutor(min(MAX_WORKERS, os.cpu_count() or 1)) as pool:
        runs = list(pool.map(lambda j: run_one(*j, window_ms), jobs))
    shutil.rmtree(root, ignore_errors=True)
    root.mkdir()
    k = len(SEEDS)
    return [pooled(runs[i * k:(i + 1) * k]) for i in range(len(wks))]


def year_summary(per_week: list[dict]) -> dict:
    requests = sum(p["requests"] for p in per_week)
    flows: dict = {}
    for p in per_week:
        add_flows(flows, p["flows"])
    return {"ratio_delivered": summary([p["ratio_delivered"] for p in per_week]),
            "ratio_lower_bound": summary([p["ratio_lower_bound"] for p in per_week]),
            "completion_rate": summary([p["completion_rate"] for p in per_week]),
            "flyable": summary([p["flyable"] for p in per_week]),
            "requests": requests,
            "outcome_share": {c: sum(p[c] for p in per_week) / requests for c in CLASSES},
            "flows": flows}


def named_split(p: dict) -> dict:
    """Door-to-door split of the week's drone-delivered orders, and their truck baseline."""
    n = sum(f["drone_n"] for f in p["flows"].values())
    mean = lambda key: sum(f[key] for f in p["flows"].values()) / n if n else None
    return {"drone_delivered": n, "door_to_door_s": mean("drone_s"), "truck_s": mean("drone_truck_s"),
            **{x: mean(x) for x in SPLIT}, "ratio_delivered": p["ratio_delivered"],
            "ratio_lower_bound": p["ratio_lower_bound"], "completion_rate": p["completion_rate"],
            "delivered_truck": p["delivered_truck"], "requests": p["requests"]}


def mon_valley(flows: dict, names: list[str], restored: set[str]) -> list[dict]:
    rows = []
    for k, f in sorted(flows.items(), key=lambda kv: tuple(map(int, kv[0].split(">")))):
        o, d = (names[int(i)] for i in k.split(">"))
        if o not in restored and d not in restored:
            continue
        rows.append({"origin": o, "destination": d, "drone_n": f["drone_n"], "trucked_n": f["trucked_n"],
                     "drone_s": f["drone_s"] / f["drone_n"] if f["drone_n"] else None,
                     "truck_s": f["drone_truck_s"] / f["drone_n"] if f["drone_n"] else None,
                     "trucked_s": f["trucked_s"] / f["trucked_n"] if f["trucked_n"] else None})
    return rows


def main() -> None:
    build_sim()
    started = time.monotonic()
    series = read_series()
    wks = weeks(series)
    published = json.loads((OUT / "weather_year.json").read_text())
    if [w["start_hour"] for w in published["weeks"]] != [w["start_hour"] for w in wks]:
        raise RuntimeError("weather_year.json weeks differ from the series: rerun analysis/weather_year.py")
    cfg = tomli.loads((CONFIGS / BASES["vtol"] / "scenario.toml").read_text())
    window_ms = (cfg["sim"]["warmup_s"]["value"] * 1000, cfg["sim"]["duration_s"]["value"] * 1000)
    restored = set(pad_names(BASES["vtol"])) - set(pad_names(BASES["multirotor"]))
    result = {"seeds": SEEDS, "mon_valley_pads": sorted(restored), "weeks": len(wks), "named_weeks": published["named_weeks"], "bases": {}}
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp) / "runs"
        root.mkdir()
        for kind in BASES:
            result["bases"][kind] = {}
            for policy in POLICIES:
                per_week = year(root, kind, policy, wks, series["start_unix_h"], window_ms)
                y = year_summary(per_week)
                y["named"] = {rule: {"monday": wks[i]["monday"], **named_split(per_week[i])}
                              for rule, i in published["named_weeks"].items()}
                y["mon_valley"] = mon_valley(y.pop("flows"), pad_names(BASES[kind]), restored)
                result["bases"][kind][policy] = y
    result["published_6b"] = {p: {"ratio": v["ratio"], "completed_share": v["completed_share"]}
                              for p, v in published["policies"].items()}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "vtol_year.json").write_text(json.dumps(result, indent=1) + "\n")
    text = report(result)
    (OUT / "vtol_year.md").write_text(text)
    print(text)
    print(f"wall {time.monotonic() - started:.0f} s")


def fmt(x, nd=2, pct=False) -> str:
    if x is None:
        return "-"
    return f"{100 * x:.1f}%" if pct else f"{x:.{nd}f}"


def report(r: dict) -> str:
    lines = [f"{r['weeks']} weeks, seeds {r['seeds']} pooled per week; median (p10 to p90) over weeks.", "",
             "(a) Year.", "",
             "| Base | Policy | Ratio, delivered | Ratio, lower bound | Completion | Flyable hours |",
             "|---|---|---|---|---|---|"]
    q = lambda s, pct=False: f"{fmt(s['p50'], pct=pct)} ({fmt(s['p10'], pct=pct)} to {fmt(s['p90'], pct=pct)})"
    for kind, pol in r["bases"].items():
        for policy, y in pol.items():
            lines.append(f"| {kind} | {policy} | {q(y['ratio_delivered'])} | {q(y['ratio_lower_bound'])} | "
                         f"{q(y['completion_rate'], True)} | {q(y['flyable'], True)} |")
    lines += ["", "Outcome breakdown, share of every request in the year.", "",
              "| Base | Policy | Requests | " + " | ".join(CLASSES) + " |",
              "|---|---|---|" + "---|" * len(CLASSES)]
    for kind, pol in r["bases"].items():
        for policy, y in pol.items():
            lines.append(f"| {kind} | {policy} | {y['requests']} | "
                         + " | ".join(fmt(y["outcome_share"][c], pct=True) for c in CLASSES) + " |")
    lines += ["", "(b) Named weeks: drone-delivered orders, mean door-to-door split (s) next to their truck "
              "baseline.", "",
              "| Week | Rule | Base | Policy | Drone | Trucked | Completion | Pod wait | Queue | Pad service | "
              "Flight | Door-to-door | Truck | Ratio, delivered | Ratio, lower bound |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for rule in r["named_weeks"]:
        for kind, pol in r["bases"].items():
            for policy, y in pol.items():
                n = y["named"][rule]
                lines.append(f"| {n['monday']} | {rule} | {kind} | {policy} | {n['drone_delivered']} | "
                             f"{n['delivered_truck']} | {fmt(n['completion_rate'], pct=True)} | "
                             + " | ".join(fmt(n[x], 0) for x in SPLIT)
                             + f" | {fmt(n['door_to_door_s'], 0)} | {fmt(n['truck_s'], 0)} | "
                             f"{fmt(n['ratio_delivered'])} | {fmt(n['ratio_lower_bound'])} |")
    lines += ["", "(c) Multirotor demo (6b as published, and rerun completion-aware) against the VTOL demo: "
              "median (p10 to p90) over weeks.", "",
              "| Policy | 6b published ratio | 6b completed share | Multirotor ratio, delivered | "
              "Multirotor ratio, lower bound | Multirotor completion | VTOL ratio, delivered | "
              "VTOL ratio, lower bound | VTOL completion |",
              "|---|---|---|---|---|---|---|---|---|"]
    for policy in POLICIES:
        p6, m, v = r["published_6b"][policy], r["bases"]["multirotor"][policy], r["bases"]["vtol"][policy]
        lines.append(f"| {policy} | {q(p6['ratio'])} | {q(p6['completed_share'], True)} | "
                     f"{q(m['ratio_delivered'])} | {q(m['ratio_lower_bound'])} | {q(m['completion_rate'], True)} | "
                     f"{q(v['ratio_delivered'])} | {q(v['ratio_lower_bound'])} | {q(v['completion_rate'], True)} |")
    lines += ["", "Mon Valley flows of the VTOL demo over the year (the multirotor network does not reach "
              "Mon Valley; its flows there are left out). Drone: drone-delivered orders, mean door-to-door "
              "against their truck baseline; trucked: orders the fallback sent by truck.", "",
              "| Policy | Origin | Destination | Drone orders | Drone (s) | Truck (s) | Ratio | Trucked orders | "
              "Trucked (s) |", "|---|---|---|---|---|---|---|---|---|"]
    for policy in POLICIES:
        for f in r["bases"]["vtol"][policy]["mon_valley"]:
            ratio = f["drone_s"] / f["truck_s"] if f["drone_n"] else None
            lines.append(f"| {policy} | {f['origin']} | {f['destination']} | {f['drone_n']} | "
                         f"{fmt(f['drone_s'], 0)} | {fmt(f['truck_s'], 0)} | {fmt(ratio)} | {f['trucked_n']} | "
                         f"{fmt(f['trucked_s'], 0)} |")
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    main()
