# /// script
# requires-python = ">=3.10"
# dependencies = ["tomli==2.0.1", "tomli-w==1.0.0"]
# ///
"""Phase 6a step 3: rejection rate at 1x demand once a pad grounded by a release fault is
repaired (spec A69) instead of staying out of service for the rest of the run.

Phase 3 (placeholder profile, seed 42) rejected 187 of 1373 requests, 13.6%, all after one
release fault on day 4.9 closed a pad for good. This reruns that scenario with and without the
repair keys, at seed 42 and over seeds 1 to 5. Writes analysis/out/repair.json and .md.

Run: uv run analysis/repair.py"""

import json
import statistics
import tempfile
from pathlib import Path

from variants import CONFIGS, OUT, build_sim, quantity, run, variant

SCENARIO = "parts_logistics_pittsburgh_placeholder"
REPAIR_MEAN_S = 7200.0
REPAIR_SPREAD_S = 3600.0
SEEDS = [42, 1, 2, 3, 4, 5]


def rejected(m: dict) -> dict:
    return {
        "requests": m["shipment_requests_in_window"],
        "rejected": m["requests_rejected"],
        "rejected_fraction": m["requests_rejected"] / m["shipment_requests_in_window"],
        "drone_mean_s": m["door_to_door_s"]["mean"],
        "deliveries_per_h": m["deliveries_per_h"],
    }


def main() -> None:
    build_sim()
    result = {"scenario": SCENARIO, "repair_mean_s": REPAIR_MEAN_S, "repair_spread_s": REPAIR_SPREAD_S}
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        repaired = variant(SCENARIO, root, "with_repair", {
            "schema_version": "1.6.0",
            "pad_ops.repair_mean_s": quantity(REPAIR_MEAN_S, "s"),
            "pad_ops.repair_spread_s": quantity(REPAIR_SPREAD_S, "s"),
        })
        for label, path in (("without_repair", CONFIGS / SCENARIO / "scenario.toml"), ("with_repair", repaired)):
            runs = {seed: rejected(run(path, seed, root / f"{label}_{seed}")) for seed in SEEDS}
            fractions = [runs[s]["rejected_fraction"] for s in SEEDS[1:]]
            result[label] = {
                "seed_42": runs[42],
                "seeds_1_to_5": {
                    "rejected_fraction_mean": statistics.fmean(fractions),
                    "rejected_fraction_std": statistics.stdev(fractions),
                    "per_seed": {s: runs[s] for s in SEEDS[1:]},
                },
            }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "repair.json").write_text(json.dumps(result, indent=1) + "\n")
    lines = [
        "| Release fault | Seed 42: rejected | Seeds 1-5: rejected fraction |",
        "|---|---|---|",
    ]
    for label in ("without_repair", "with_repair"):
        s42, s = result[label]["seed_42"], result[label]["seeds_1_to_5"]
        lines.append(
            f"| {label.replace('_', ' ')} | {s42['rejected']} of {s42['requests']} ({s42['rejected_fraction']:.1%}) "
            f"| {s['rejected_fraction_mean']:.1%} ± {s['rejected_fraction_std']:.1%} |"
        )
    (OUT / "repair.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
