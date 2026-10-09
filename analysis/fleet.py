# /// script
# requires-python = ">=3.10"
# dependencies = ["tomli==2.0.1", "tomli-w==1.0.0"]
# ///
"""Phase 6a step 8: pods and fleet at the chosen 1370 Wh layout, one command.

Base: configs/parts_logistics_pittsburgh_1370wh_layout (the layout analysis/sweep.py chose).
Grid: `demand.pods_per_origin` (pods at each warehouse, spec A73) against `drone.fleet_size`, five
seeds each; mean and standard deviation over seeds of door-to-door, pod wait and the rejected
fraction (requests turned away at the backlog cap, M9). A fleet larger than the layout's slots
cannot load (R5) and is reported as infeasible.

The smallest combination with under 5% rejection is the one with the fewest drones, then the
fewest pods per warehouse, whose mean rejected fraction is below 5% (A73). Rejection alone lets
a slow network pass, so two more minima are reported on the same ordering: the smallest also
faster than the truck on the same orders, and the smallest also within 10% of the best mean
door-to-door.

Writes analysis/out/fleet.json and fleet.md; `--base 900` runs at the 900 Wh layout of Phase 6b
and writes fleet_900wh.json/.md; `--base vtol` runs at the VTOL layout of Phase 7 and writes
fleet_vtol.json/.md. Run: uv run analysis/fleet.py [--base 1370|900|vtol]"""

import argparse
import json
import tempfile
from itertools import product
from pathlib import Path

import tomli

from sweep import load_error, run_one, stats_over
from variants import CONFIGS, OUT, build_sim, quantity, variant

# Chosen layout scenario and output name per base (analysis/sweep.py --base).
BASES = {
    "1370": ("parts_logistics_pittsburgh_1370wh_layout", "fleet"),
    "900": ("parts_logistics_pittsburgh_900wh_layout", "fleet_900wh"),
    "vtol": ("parts_logistics_pittsburgh_vtol_layout", "fleet_vtol"),
}
PODS = [2, 4, 6, 8, 12, 16]
FLEET = [2, 4, 6, 8, 10, 12, 14, 16, 18]
SEEDS = [1, 2, 3, 4, 5]
MAX_REJECTED = 0.05
WITHIN = 0.10


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--base", choices=sorted(BASES), default="1370")
    base, out_name = BASES[parser.parse_args().base]
    build_sim()
    cfg = tomli.loads((CONFIGS / base / "scenario.toml").read_text())
    window = (cfg["sim"]["warmup_s"]["value"] * 1000, cfg["sim"]["duration_s"]["value"] * 1000)
    points = []
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        for fleet, pods in product(FLEET, PODS):
            name = f"f{fleet}_p{pods}"
            path = variant(base, root, name, {
                "drone.fleet_size": fleet,
                "demand.pods_per_origin": quantity(pods, "count"),
            })
            runs = [run_one(path, s, root / f"{name}_{s}", window) for s in SEEDS]
            point = {"fleet_size": fleet, "pods_per_warehouse": pods, "feasible": None not in runs}
            if point["feasible"]:
                point |= stats_over(runs)
                point["max_rejected_fraction"] = max(r["rejected_fraction"] for r in runs)
            else:
                point["infeasible"] = load_error(path)
            points.append(point)
    def smallest_of(candidates):
        return min(candidates, key=lambda p: (p["fleet_size"], p["pods_per_warehouse"]), default=None)

    ok = [p for p in points if p["feasible"] and p["rejected_fraction"]["mean"] < MAX_REJECTED]
    timed = [p for p in ok if p["door_to_door_s"]["mean"] is not None]
    best = min(p["door_to_door_s"]["mean"] for p in timed)
    smallest = smallest_of(ok)
    result = {
        "base": base, "seeds": SEEDS, "max_rejected_fraction": MAX_REJECTED,
        "points": points, "smallest": smallest,
        "best_door_to_door_s": best,
        "smallest_faster_than_truck": smallest_of(
            p for p in timed if p["door_to_door_s"]["mean"] < p["truck_s"]["mean"]),
        "smallest_within_10pct_of_best": smallest_of(
            p for p in timed if p["door_to_door_s"]["mean"] <= (1 + WITHIN) * best),
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{out_name}.json").write_text(json.dumps(result, indent=1) + "\n")
    fmt = lambda x: (f"{x['mean']:.0f} ± {x['std']:.0f}" if x["mean"] is not None
                     else f"none delivered on {x['seeds_without_deliveries']} seed(s)")
    lines = [
        "| Fleet | Pods per warehouse | Door-to-door (s) | Pod wait (s) | Rejected (mean ± std, max) |",
        "|---|---|---|---|---|",
    ]
    for p in points:
        head = f"| {p['fleet_size']} | {p['pods_per_warehouse']} |"
        if not p["feasible"]:
            lines.append(f"{head} infeasible: {p['infeasible'][:60]} | | |")
            continue
        r = p["rejected_fraction"]
        mark = " (smallest under 5%)" if p is smallest else ""
        lines.append(
            f"{head} {fmt(p['door_to_door_s'])} | {fmt(p['pod_wait_s'])} | "
            f"{r['mean']:.1%} ± {r['std']:.1%}, {p['max_rejected_fraction']:.1%}{mark} |"
        )
    (OUT / f"{out_name}.md").write_text("\n".join(lines) + "\n")
    for key in ("smallest", "smallest_faster_than_truck", "smallest_within_10pct_of_best"):
        p = result[key]
        if p is None:
            print(f"{key}: none")
            continue
        print(f"{key}: fleet {p['fleet_size']}, {p['pods_per_warehouse']} pods per warehouse, "
              f"door-to-door {fmt(p['door_to_door_s'])}, rejected {p['rejected_fraction']['mean']:.1%}")


if __name__ == "__main__":
    main()
