# /// script
# requires-python = ">=3.10"
# dependencies = ["tomli==2.0.1", "tomli-w==1.0.0"]
# ///
"""Phase 6a step 2: drone against truck on the same orders.

Audit (spec 7, `truck_baseline` in sim/src/fm.rs): both clocks start at the shipment request.
The truck adds `truck.dispatch_delay_s`, `truck.load_s`, the drive and `truck.unload_s`; the
drone's time holds the pod wait (with `pad_ops.pod_place_s`), queue, pad service and flight. So
the truck already carries its loading and dispatch dwell, and no truck dwell key is added.

Comparison: every Pittsburgh scenario, restricted to the order flows the fitted 900 Wh network
serves, over the delivered shipments requested inside the metrics window. Drone and truck are
averaged over the same orders, with the drone's time split (M2). Five seeds; mean and standard
deviation over seeds. Writes analysis/out/truck_fairness.json and .md.

Run: uv run analysis/truck_fairness.py"""

import csv
import json
import statistics
import tempfile
from pathlib import Path

import tomli

from variants import CONFIGS, OUT, build_sim, run

SCENARIOS = [
    ("placeholder", "parts_logistics_pittsburgh_placeholder"),
    ("fitted 900 Wh", "parts_logistics_pittsburgh"),
    ("fitted 1370 Wh", "parts_logistics_pittsburgh_battery_1370wh"),
]
SERVED_BY = "parts_logistics_pittsburgh"
SEEDS = [1, 2, 3, 4, 5]
SPLIT = ["pod_wait_s", "queue_s", "pad_service_s", "flight_s"]


def pad_names(name: str) -> list[str]:
    cfg = tomli.loads((CONFIGS / name / "scenario.toml").read_text())
    derived = CONFIGS / name / cfg["network_files"]["dir"]
    return [p["name"] for p in tomli.loads((derived / "pads.toml").read_text())["pads"]]


def served_flows() -> set[tuple[str, str]]:
    cfg = tomli.loads((CONFIGS / SERVED_BY / "scenario.toml").read_text())
    return {
        (f["origin"], f["destination"])
        for f in cfg["demand"]["flows"]
        if f["rate_per_h"]["value"] > 0
    }


def window_ms(name: str) -> tuple[float, float]:
    sim = tomli.loads((CONFIGS / name / "scenario.toml").read_text())["sim"]
    return sim["warmup_s"]["value"] * 1000, sim["duration_s"]["value"] * 1000


def orders(run_dir: Path, names: list[str], window, flows=None) -> list[dict]:
    rows = []
    with open(run_dir / "deliveries.csv") as f:
        for r in csv.DictReader(f):
            if r["kind"] != "SHIPMENT" or r["status"] != "DELIVERED":
                continue
            if not window[0] <= int(r["requested_t_ms"]) <= window[1]:
                continue
            flow = (names[int(r["origin_pad"])], names[int(r["dest_pad"])])
            if flows is None or flow in flows:
                rows.append(r)
    return rows


def summarise(rows: list[dict]) -> dict:
    mean = lambda key: statistics.fmean(float(r[key]) for r in rows)
    drone, truck = mean("drone_door_to_door_s"), mean("truck_door_to_door_s")
    return {
        "orders": len(rows),
        "drone_s": drone,
        "truck_s": truck,
        "speedup": truck / drone,
        **{k: mean(k) for k in SPLIT},
    }


def over_seeds(per_seed: list[dict]) -> dict:
    out = {}
    for key in per_seed[0]:
        values = [s[key] for s in per_seed]
        out[key] = {"mean": statistics.fmean(values), "std": statistics.stdev(values)}
    return out


def main() -> None:
    build_sim()
    flows = served_flows()
    result = {"served_flows": sorted(f"{a} -> {b}" for a, b in flows), "seeds": SEEDS, "scenarios": {}}
    with tempfile.TemporaryDirectory() as tmp:
        for label, name in SCENARIOS:
            names, window = pad_names(name), window_ms(name)
            same, every = [], []
            for seed in SEEDS:
                out = Path(tmp) / f"{name}_{seed}"
                metrics = run(CONFIGS / name / "scenario.toml", seed, out)
                all_rows = orders(out, names, window)
                # The unrestricted drone mean must equal M2's, or the filter is wrong.
                m2 = metrics["door_to_door_s"]["mean"]
                assert abs(summarise(all_rows)["drone_s"] - m2) < 1e-6, (name, seed)
                same.append(summarise(orders(out, names, window, flows)))
                every.append(summarise(all_rows))
            result["scenarios"][label] = {
                "scenario": name,
                "same_flows": over_seeds(same),
                "all_flows": over_seeds(every),
            }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "truck_fairness.json").write_text(json.dumps(result, indent=1) + "\n")
    lines = [
        "| Scenario | Orders | Drone (s) | Truck (s) | Speedup | Pod wait / queue / pad service / flight (s) |",
        "|---|---|---|---|---|---|",
    ]
    for label, s in result["scenarios"].items():
        x = s["same_flows"]
        split = " / ".join(f"{x[k]['mean']:.0f}" for k in SPLIT)
        lines.append(
            f"| {label} | {x['orders']['mean']:.0f} | {x['drone_s']['mean']:.0f} ± {x['drone_s']['std']:.0f} "
            f"| {x['truck_s']['mean']:.0f} ± {x['truck_s']['std']:.0f} | {x['speedup']['mean']:.2f} | {split} |"
        )
    (OUT / "truck_fairness.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
