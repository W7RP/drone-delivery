"""Phase 5 step 7b and 7c: the three Pittsburgh variants side by side. 7b: their lanes against
Phase 3's (placeholder) network: lanes gained and lost, pads left out, whether Clairton is
served. 7c: a 7-day run of each (seed 42, release sim) with drone door-to-door time and its
split (pod wait, queue, pad service, flight) next to the truck. Writes out/scenarios.json; run
outputs go to runs/calib/<variant>/ (git-ignored). Wall time and memory are scripts/verify.sh's
budget check and are not recorded here, so this file reruns byte-identically."""

import json
import subprocess

from common import OUT, REPO, SIM

VARIANTS = ["parts_logistics_pittsburgh_placeholder", "parts_logistics_pittsburgh",
            "parts_logistics_pittsburgh_battery_1370wh"]
SEED = 42
CLAIRTON = ("Mon_Valley_Works_Clairton_Plant", "Old_Koppers_Area")


def lanes_of(variant: str) -> set[frozenset]:
    report = json.loads((REPO / "configs" / variant / "derived" / "report.json").read_text())
    return {frozenset((l["a"], l["b"])) for l in report["lanes"]}


def network(variant: str) -> dict:
    report = json.loads((REPO / "configs" / variant / "derived" / "report.json").read_text())
    base = lanes_of(VARIANTS[0])
    mine = lanes_of(variant)
    flows = report["scenario"]["flows"]
    return {"pads": len(report["pads"]), "lanes": len(mine),
            "lanes_gained": sorted("-".join(sorted(l)) for l in mine - base),
            "lanes_lost": sorted("-".join(sorted(l)) for l in base - mine),
            "pads_left_out": [p["name"] for p in report["variant"]["pads_left_out"]],
            "flows": len(flows), "fleet_size": report["scenario"]["fleet_size"],
            "clairton_served": any(f["destination"] in CLAIRTON for f in flows)}


def run(variant: str) -> dict:
    out = REPO / "runs" / "calib" / variant
    subprocess.run([str(SIM), "run", str(REPO / "configs" / variant / "scenario.toml"),
                    "--seed", str(SEED), "--out", str(out)], check=True, capture_output=True)
    m = json.loads((out / "metrics.json").read_text())
    split = m["door_to_door_split_s"]
    return {
        "requests": m["shipment_requests_in_window"],
        "rejected": m["requests_rejected"],
        "deliveries_per_h": round(m["deliveries_per_h"], 2),
        "drone_door_to_door_s": {k: round(v, 1) for k, v in m["door_to_door_s"].items() if k != "n"},
        "truck_door_to_door_s": {k: round(v, 1) for k, v in m["truck_door_to_door_s"].items() if k != "n"},
        "speedup": round(m["speedup"], 3),
        "split_s": {k: round(split[k], 1) for k in ("pod_wait", "queue", "pad_service", "flight")},
        "drone_cost_per_delivery": round(m["cost"]["drone_cost_per_delivery"], 2),
        "truck_cost_per_delivery": round(m["cost"]["truck_cost_per_delivery"], 2),
    }


def main() -> None:
    result = {v: {"network": network(v), "run": run(v)} for v in VARIANTS}
    (OUT / "scenarios.json").write_text(json.dumps(result, indent=1) + "\n")
    for v, r in result.items():
        n, x = r["network"], r["run"]
        print(f"{v}: {n['pads']} pads, {n['lanes']} lanes (+{len(n['lanes_gained'])} -{len(n['lanes_lost'])}), "
              f"{n['flows']} flows, fleet {n['fleet_size']}, Clairton {n['clairton_served']}; "
              f"drone {x['drone_door_to_door_s']['mean']} s vs truck {x['truck_door_to_door_s']['mean']} s "
              f"(x{x['speedup']}), split {x['split_s']}, rejected {x['rejected']}/{x['requests']}")


if __name__ == "__main__":
    main()
