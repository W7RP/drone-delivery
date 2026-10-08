# /// script
# requires-python = ">=3.10"
# dependencies = ["tomli==2.0.1", "tomli-w==1.0.0"]
# ///
"""Phase 6a step 7: charging and pad layout sweep at 1370 Wh, one command.

Base: configs/parts_logistics_pittsburgh_battery_1370wh, the one 1370 Wh scenario. No coupled
battery serves its longest lane (analysis/battery.py), so the sweep keeps its fixed mass.
Grid (spec A72), every pad alike:

- `charging.power_w`: 1.5, 3, 6, 12 kW;
- parking: `pad_ops.parking_capacity` 0 to 3, with parked charging on whenever there is parking;
- `pad_ops.out_spots`: 1 to 3.

Every point repairs a pad grounded by a release fault (spec A69, mean 2 h, spread 1 h): without
it a single release fault closes a pad for the rest of the week and swamps the layout's effect.
Five seeds per point; mean and standard deviation over seeds of drone door-to-door (M2), pod
wait (M2 split), rejected fraction (M9), and the truck's time on the same delivered orders. A
point the core rejects at load (too few slots for the fleet, R4/R5) is reported as infeasible.

Hardware-cost proxy per pad, placeholders in `cu`: base + per spot x (IN + OUT spots) + per
kW x charger kW x charging points (OUT spots, plus parking slots when they charge) + per
parking slot x slots; the network's cost is the sum over its pads.

Before/after: the base scenario as it is, the base with repair only, and the chosen layout.

Writes analysis/out/sweep.json and sweep.md: the table, the Pareto set (door-to-door against
cost), and the chosen layout, the cheapest within 10% of the best mean door-to-door, which is
also written as configs/parts_logistics_pittsburgh_1370wh_layout/scenario.toml.

Phase 6b adds `--base 900`: the same grid on the coupled 900 Wh network (fitted profile, 5 pads,
4.5 kg airframe at 200 Wh/kg, spec A68) with the yield fix on (A74) at every point; it writes
sweep_900wh.json/.md and configs/parts_logistics_pittsburgh_900wh_layout.

Run: uv run analysis/sweep.py [--base 1370|900]"""

import argparse
import csv
import json
import re
import shutil
import statistics
import subprocess
import tempfile
from dataclasses import dataclass, field
from itertools import product
from pathlib import Path

import tomli

from variants import CONFIGS, OUT, SIM, build_sim, quantity, variant

# Coupled battery mass (A68) and the yield fix (A74), for the 900 Wh base of Phase 6b.
AIRFRAME_KG = 4.5
SPECIFIC_ENERGY_WH_PER_KG = 200.0


@dataclass(frozen=True)
class Base:
    scenario: str
    chosen: str
    label: str
    out_name: str
    schema: str
    # Keys every sweep point and the "with repair" reference also get.
    extra: dict = field(default_factory=dict)
    coupled: bool = False


BASES = {
    "1370": Base("parts_logistics_pittsburgh_battery_1370wh", "parts_logistics_pittsburgh_1370wh_layout",
                 "1370 Wh", "sweep", "1.6.0"),
    "900": Base("parts_logistics_pittsburgh", "parts_logistics_pittsburgh_900wh_layout",
                "900 Wh", "sweep_900wh", "1.7.0",
                extra={
                    "schema_version": "1.7.0",
                    "drone.empty_mass_kg": None,
                    "drone.airframe_mass_kg": quantity(AIRFRAME_KG, "kg"),
                    "drone.battery_specific_energy_wh_per_kg": quantity(SPECIFIC_ENERGY_WH_PER_KG, "Wh/kg"),
                    "policy.yield_on_request": True,
                },
                coupled=True),
}
POWERS_W = [1500.0, 3000.0, 6000.0, 12000.0]
PARKING = [0, 1, 2, 3]
OUT_SPOTS = [1, 2, 3]
SEEDS = [1, 2, 3, 4, 5]
WITHIN = 0.10
# A 7-day run takes well under a second; a run this long is stuck.
RUN_TIMEOUT_S = 60
CONFIG_ERROR_EXIT = 2
REPAIR_MEAN_S = 7200.0
REPAIR_SPREAD_S = 3600.0
COST_CU = {"pad_base": 100.0, "per_spot": 20.0, "per_charger_kw": 10.0, "per_parking_slot": 15.0}


def cost_proxy(n_pads: int, power_w: float, parking: int, out_spots: int) -> float:
    charging_points = out_spots + (parking if parking > 0 else 0)
    per_pad = (
        COST_CU["pad_base"]
        + COST_CU["per_spot"] * (1 + out_spots)
        + COST_CU["per_charger_kw"] * power_w / 1000.0 * charging_points
        + COST_CU["per_parking_slot"] * parking
    )
    return n_pads * per_pad


REPAIR = {
    "schema_version": "1.6.0",
    "pad_ops.repair_mean_s": quantity(REPAIR_MEAN_S, "s"),
    "pad_ops.repair_spread_s": quantity(REPAIR_SPREAD_S, "s"),
}


def edits(base: Base, power_w: float, parking: int, out_spots: int) -> dict:
    return {
        **REPAIR,
        **base.extra,
        "charging.power_w": power_w,
        "charging.charge_while_parked": parking > 0,
        "pad_ops.parking_capacity": quantity(parking, "count"),
        "pad_ops.out_spots": quantity(out_spots, "count"),
    }


def run_one(scenario: Path, seed: int, out: Path, window_ms) -> dict | None:
    """Metrics of one run; None if the core rejects the config at load (exit code 2). A run that
    fails otherwise, or outlasts RUN_TIMEOUT_S, stops the sweep: that is a bug, not a result.
    The run's outputs are deleted once read, so a sweep never leaves event logs behind."""
    try:
        done = subprocess.run(
            [str(SIM), "run", str(scenario), "--seed", str(seed), "--out", str(out), "--no-event-log"],
            capture_output=True, text=True, timeout=RUN_TIMEOUT_S,
        )
        if done.returncode == CONFIG_ERROR_EXIT:
            return None
        if done.returncode != 0:
            raise RuntimeError(f"{scenario} seed {seed} failed: {done.stderr.strip()}")
        return read_run(out, window_ms)
    except subprocess.TimeoutExpired as e:
        raise RuntimeError(f"{scenario} seed {seed} did not finish in {RUN_TIMEOUT_S} s") from e
    finally:
        shutil.rmtree(out, ignore_errors=True)


def read_run(out: Path, window_ms) -> dict:
    m = json.loads((out / "metrics.json").read_text())
    rows = []
    with open(out / "deliveries.csv") as f:
        for r in csv.DictReader(f):
            if r["kind"] == "SHIPMENT" and r["status"] == "DELIVERED" and window_ms[0] <= int(r["requested_t_ms"]) <= window_ms[1]:
                rows.append(r)
    # A run that delivers nothing in the window has no times to average.
    mean = lambda k: statistics.fmean(float(r[k]) for r in rows) if rows else None
    return {
        "door_to_door_s": mean("drone_door_to_door_s"),
        "pod_wait_s": mean("pod_wait_s"),
        "truck_s": mean("truck_door_to_door_s"),
        "rejected_fraction": m["requests_rejected"] / m["shipment_requests_in_window"],
    }


def load_error(scenario: Path) -> str:
    probe = scenario.parent / "probe"
    done = subprocess.run([str(SIM), "run", str(scenario), "--seed", "1", "--out", str(probe), "--no-event-log"],
                          capture_output=True, text=True, timeout=RUN_TIMEOUT_S)
    shutil.rmtree(probe, ignore_errors=True)
    return done.stderr.strip().removeprefix("config error: ")


def stats_over(runs: list[dict]) -> dict:
    """Mean and standard deviation over seeds; None for a time when any seed delivered nothing."""
    out = {}
    for k in runs[0]:
        values = [r[k] for r in runs]
        if None in values:
            out[k] = {"mean": None, "std": None, "seeds_without_deliveries": values.count(None)}
        else:
            out[k] = {"mean": statistics.fmean(values), "std": statistics.stdev(values)}
    return out


def ranked(points: list[dict]) -> list[dict]:
    """Points that loaded and delivered on every seed: the only ones with a mean to rank."""
    return [p for p in points if p["feasible"] and p["door_to_door_s"]["mean"] is not None]


def pareto(points: list[dict]) -> list[dict]:
    feasible = ranked(points)
    front = []
    for p in sorted(feasible, key=lambda p: (p["cost_cu"], p["door_to_door_s"]["mean"])):
        if not front or p["door_to_door_s"]["mean"] < front[-1]["door_to_door_s"]["mean"]:
            front.append(p)
    return front


def write_chosen(base: Base, point: dict) -> Path:
    """The chosen layout as a committed scenario: the base file's text with the layout keys set,
    so its comments and every other value stay as they are."""
    text = (CONFIGS / base.scenario / "scenario.toml").read_text()
    header = (
        f"# The {base.label} Pittsburgh scenario with the charging and pad layout chosen by\n"
        f"# analysis/sweep.py, which wrote this file from the {base.label} scenario and shares its network\n"
        "# files. Every number is a placeholder (assumption = true), not real data.\n"
    )
    text = header + text[text.index("\n\n") + 1:]
    parking = "no parking" if point["parking"] == 0 else f"{point['parking']} charging parking slots"
    subs = [
        (r'^schema_version = "[^"]+"$', f'schema_version = "{base.schema}"'),
        (r'^name = "[^"]+"$', f'name = "{base.chosen}"'),
        (r'^description = "[^"]+"$',
         f'description = "The {base.label} scenario with the charging and pad layout chosen by analysis/sweep.py: '
         f'{point["power_w"] / 1000:g} kW chargers, {point["out_spots"]} OUT spots and {parking} at every pad, seven days."'),
        (r'^dir = "derived"$', f'dir = "../{base.scenario}/derived"'),
        (r'^power_w = \{ value = [0-9.]+,', f'power_w = {{ value = {point["power_w"]},'),
        (r'^charge_while_parked = (true|false)$', f'charge_while_parked = {str(point["parking"] > 0).lower()}'),
    ]
    for pattern, new in subs:
        text, n = re.subn(pattern, new, text, count=1, flags=re.MULTILINE)
        assert n == 1, pattern
    layout = (
        "# Pad layout chosen by analysis/sweep.py (spec A72): every pad alike.\n"
        f'out_spots = {{ value = {point["out_spots"]}, unit = "count", assumption = true }}\n'
        f'parking_capacity = {{ value = {point["parking"]}, unit = "count", assumption = true }}\n'
    )
    layout += (
        "# Repair after a release fault (spec A69), as in the sweep.\n"
        f'repair_mean_s = {{ value = {REPAIR_MEAN_S}, unit = "s", assumption = true }}\n'
        f'repair_spread_s = {{ value = {REPAIR_SPREAD_S}, unit = "s", assumption = true }}\n'
    )
    text, n = re.subn(r"^(pc_fault_duration_s = .*\n)", r"\1" + layout.replace("\\", "\\\\"), text, count=1, flags=re.MULTILINE)
    assert n == 1
    if base.coupled:
        coupled = (
            "# Battery mass follows capacity (spec A68): 4.5 kg + 900 Wh / 200 Wh/kg = 9.0 kg.\n"
            f'airframe_mass_kg = {{ value = {AIRFRAME_KG}, unit = "kg", assumption = true }}\n'
            f'battery_specific_energy_wh_per_kg = {{ value = {SPECIFIC_ENERGY_WH_PER_KG}, unit = "Wh/kg", assumption = true }}\n'
        )
        text, n = re.subn(r"^empty_mass_kg = .*\n", coupled.replace("\\", "\\\\"), text, count=1, flags=re.MULTILINE)
        assert n == 1
        text, n = re.subn(r"^\[policy\]\n", "[policy]\n# Idle drones make room on request (spec A74).\nyield_on_request = true\n", text, count=1, flags=re.MULTILINE)
        assert n == 1
    path = CONFIGS / base.chosen / "scenario.toml"
    path.parent.mkdir(exist_ok=True)
    path.write_text(text)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--base", choices=sorted(BASES), default="1370")
    base = BASES[parser.parse_args().base]
    build_sim()
    base_cfg = tomli.loads((CONFIGS / base.scenario / "scenario.toml").read_text())
    window = (base_cfg["sim"]["warmup_s"]["value"] * 1000, base_cfg["sim"]["duration_s"]["value"] * 1000)
    derived = CONFIGS / base.scenario / base_cfg["network_files"]["dir"]
    n_pads = len(tomli.loads((derived / "pads.toml").read_text())["pads"])
    points = []
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        before = [run_one(CONFIGS / base.scenario / "scenario.toml", s, root / f"base_{s}", window) for s in SEEDS]
        repaired = variant(base.scenario, root, "base_repair", {**REPAIR, **base.extra})
        before_repair = [run_one(repaired, s, root / f"base_repair_{s}", window) for s in SEEDS]
        for power_w, parking, out_spots in product(POWERS_W, PARKING, OUT_SPOTS):
            name = f"p{int(power_w)}_k{parking}_o{out_spots}"
            path = variant(base.scenario, root, name, edits(base, power_w, parking, out_spots))
            runs = [run_one(path, s, root / f"{name}_{s}", window) for s in SEEDS]
            point = {"power_w": power_w, "parking": parking, "out_spots": out_spots,
                     "cost_cu": cost_proxy(n_pads, power_w, parking, out_spots), "feasible": None not in runs}
            if point["feasible"]:
                point |= stats_over(runs)
            else:
                point["infeasible"] = load_error(path)
            points.append(point)
    feasible = ranked(points)
    best = min(p["door_to_door_s"]["mean"] for p in feasible)
    chosen = min((p for p in feasible if p["door_to_door_s"]["mean"] <= (1 + WITHIN) * best),
                 key=lambda p: (p["cost_cu"], p["door_to_door_s"]["mean"]))
    front = pareto(points)
    result = {
        "base": base.scenario, "seeds": SEEDS, "cost_cu": COST_CU, "n_pads": n_pads,
        "before": stats_over(before),
        "before_with_repair": stats_over(before_repair),
        "points": points,
        "pareto": [{k: p[k] for k in ("power_w", "parking", "out_spots")} for p in front],
        "best_door_to_door_s": best,
        "chosen": chosen,
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{base.out_name}.json").write_text(json.dumps(result, indent=1) + "\n")
    write_chosen(base, chosen)
    on_front = {(p["power_w"], p["parking"], p["out_spots"]) for p in front}
    fmt = lambda x: f"{x['mean']:.0f} ± {x['std']:.0f}"
    lines = [
        "| Charger (kW) | Parking (charging) | OUT spots | Cost (cu) | Door-to-door (s) | Pod wait (s) | Rejected | Truck (s) | Pareto |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for p in points:
        head = f"| {p['power_w'] / 1000:g} | {p['parking']} | {p['out_spots']} | {p['cost_cu']:.0f} |"
        if not p["feasible"]:
            lines.append(f"{head} infeasible: {p['infeasible'][:60]} | | | | |")
            continue
        mark = "yes" if (p["power_w"], p["parking"], p["out_spots"]) in on_front else ""
        r = p["rejected_fraction"]
        lines.append(f"{head} {fmt(p['door_to_door_s'])} | {fmt(p['pod_wait_s'])} | {r['mean']:.1%} ± {r['std']:.1%} | {fmt(p['truck_s'])} | {mark} |")
    (OUT / f"{base.out_name}.md").write_text("\n".join(lines) + "\n")
    for label in ("before", "before_with_repair"):
        b = result[label]
        print(f"{label}: door-to-door {fmt(b['door_to_door_s'])}, pod wait {fmt(b['pod_wait_s'])}, "
              f"rejected {b['rejected_fraction']['mean']:.1%}, truck {fmt(b['truck_s'])}")
    print(f"best mean door-to-door {best:.0f} s; chosen {chosen['power_w'] / 1000:g} kW, parking {chosen['parking']}, "
          f"OUT {chosen['out_spots']}, cost {chosen['cost_cu']:.0f} cu, door-to-door {fmt(chosen['door_to_door_s'])}")
    print(f"pareto: {result['pareto']}")


if __name__ == "__main__":
    main()
