# /// script
# requires-python = ">=3.10"
# dependencies = ["tomli==2.0.1", "tomli-w==1.0.0"]
# ///
"""Phase 7 step 1: completion-aware results for the Phase 6b demo, one command.

Reruns the three named weeks of analysis/weather_year.py (median wind, windiest, coldest) on
configs/parts_logistics_pittsburgh_demo with M11 order outcomes on (spec 7), seeds 1 to 3, under
both hold policies; then the truck-fallback timeout at 1800, 3600 and 7200 s on the windiest and
coldest weeks. Each cell pools its seeds: outcome counts are summed, and both ratios are sums of
door-to-door over sums of truck baseline (order-weighted). The 6b ratio of the same cell is shown
beside it.

Writes analysis/out/completion.json and completion.md. The demo file is not edited: every run is
a copy with `metrics.order_outcomes` on. Run: uv run analysis/completion.py"""

import json
import os
import shutil
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from variants import OUT, SIM, build_sim, variant

BASE = "parts_logistics_pittsburgh_demo"
SEEDS = [1, 2, 3]
TIMEOUTS_S = [1800.0, 3600.0, 7200.0]
CLASSES = ["delivered_drone", "delivered_truck", "rejected_cap", "blocked_weather",
           "blocked_energy", "in_flight", "waiting", "diverted_or_failed"]
SUMS = ["delivered_network", "delivered_truck", "unfinished_elapsed", "unfinished_truck"]
RUN_TIMEOUT_S = 60
MAX_WORKERS = 16
# Keys every run adds to the demo; `extra` adds a step's own.
OUTCOMES_ON = {"schema_version": "1.9.0", "metrics.order_outcomes": True}


def run_one(scenario: Path, seed: int) -> dict:
    out = scenario.parent / f"out_{seed}"
    try:
        done = subprocess.run([str(SIM), "run", str(scenario), "--seed", str(seed), "--out", str(out),
                               "--no-event-log"], capture_output=True, text=True,
                              timeout=RUN_TIMEOUT_S)
        if done.returncode != 0:
            raise RuntimeError(f"{scenario} seed {seed}: {done.stderr.strip()}")
        return json.loads((out / "metrics.json").read_text())["order_outcomes"]
    finally:
        shutil.rmtree(out, ignore_errors=True)


def pooled(outcomes: list[dict]) -> dict:
    """Seeds summed: counts, completion rate, and both order-weighted ratios."""
    p = {c: sum(o[c] for o in outcomes) for c in ["requests", *CLASSES]}
    s = {k: sum(o["sums_s"][k] for o in outcomes) for k in SUMS}
    p["completion_rate"] = (p["delivered_drone"] + p["delivered_truck"]) / p["requests"]
    p["ratio_delivered"] = (s["delivered_network"] / s["delivered_truck"]
                            if s["delivered_truck"] else None)
    p["ratio_lower_bound"] = ((s["delivered_network"] + s["unfinished_elapsed"])
                              / (s["delivered_truck"] + s["unfinished_truck"]))
    return p


def cells(root: Path, wanted: list[tuple[str, dict]]) -> list[dict]:
    jobs = []
    for n, (_, edits) in enumerate(wanted):
        path = variant(BASE, root, f"cell{n}", {**OUTCOMES_ON, **edits})
        jobs += [(path, s) for s in SEEDS]
    with ThreadPoolExecutor(min(MAX_WORKERS, os.cpu_count() or 1)) as pool:
        runs = list(pool.map(lambda j: run_one(*j), jobs))
    k = len(SEEDS)
    return [pooled(runs[i * k:(i + 1) * k]) for i in range(len(wanted))]


def main(extra_cells=None) -> None:
    build_sim()
    year = json.loads((OUT / "weather_year.json").read_text())
    named = {rule: year["weeks"][i] | {"index": i} for rule, i in year["named_weeks"].items()}
    wanted, labels = [], []
    for rule, w in named.items():
        for policy in ("WAIT", "TRUCK_FALLBACK"):
            labels.append({"part": "named", "rule": rule, "monday": w["monday"], "policy": policy,
                           "ratio_6b": year["policies"][policy]["per_week"][w["index"]]["ratio"]})
            wanted.append((rule, {"weather.start_hour": w["start_hour"],
                                  "weather.hold_policy": policy}))
    for rule in ("windiest", "coldest"):
        w = named[rule]
        for timeout in TIMEOUTS_S:
            labels.append({"part": "timeout", "rule": rule, "monday": w["monday"],
                           "policy": "TRUCK_FALLBACK", "timeout_s": timeout})
            wanted.append((rule, {"weather.start_hour": w["start_hour"],
                                  "weather.hold_policy": "TRUCK_FALLBACK",
                                  "weather.truck_fallback_timeout_s": timeout}))
    for label, edits in extra_cells or []:
        labels.append(label)
        wanted.append((label["rule"], edits))
    with tempfile.TemporaryDirectory() as tmp:
        results = cells(Path(tmp), wanted)
    rows = [lab | res for lab, res in zip(labels, results)]
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "completion.json").write_text(json.dumps({"base": BASE, "seeds": SEEDS, "cells": rows},
                                                    indent=1) + "\n")
    (OUT / "completion.md").write_text(report(rows))
    print(report(rows))


def pct(x):
    return "-" if x is None else f"{100 * x:.1f}%"


def num(x):
    return "-" if x is None else f"{x:.2f}"


def report(rows: list[dict]) -> str:
    head = ("| Drone | Truck fallback | Rejected | Blocked weather | Blocked energy | In flight "
            "| Waiting | Diverted/failed | Completion | Ratio, delivered | Ratio, lower bound |")
    rule = "|---" * 11 + "|"
    def counts(r):
        return " | ".join(str(r[c]) for c in CLASSES)
    lines = ["Named weeks of the 6b demo, seeds 1 to 3 pooled (requests summed over seeds).", "",
             "| Week | Rule | Policy | 6b ratio " + head, "|---|---|---|---" + rule]
    for r in rows:
        if r["part"] == "named":
            lines.append(f"| {r['monday']} | {r['rule']} | {r['policy']} | {num(r['ratio_6b'])} | "
                         f"{counts(r)} | {pct(r['completion_rate'])} | {num(r['ratio_delivered'])} | "
                         f"{num(r['ratio_lower_bound'])} |")
    for part, title in (("timeout", "Truck-fallback timeout, TRUCK_FALLBACK."),
                        ("energy", "Truck fallback also on energy-blocked orders (step 2).")):
        sel = [r for r in rows if r["part"] == part]
        if not sel:
            continue
        lines += ["", title, "", "| Week | Rule | Timeout (s) " + head, "|---|---|---" + rule]
        for r in sel:
            lines.append(f"| {r['monday']} | {r['rule']} | {r['timeout_s']:.0f} | {counts(r)} | "
                         f"{pct(r['completion_rate'])} | {num(r['ratio_delivered'])} | "
                         f"{num(r['ratio_lower_bound'])} |")
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    main()
