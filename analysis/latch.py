# /// script
# requires-python = ">=3.10"
# dependencies = ["tomli==2.0.1", "tomli-w==1.0.0", "scipy==1.14.1", "numpy==2.1.3"]
# ///
"""Phase 6a step 6: the latch-fail probability derived from the OUT landing error (spec A71).

The 15 Phase 4 OUT landing errors (analysis/data/phase4_out_landing_error.json) are fitted by
maximum likelihood with two families for the distance of a 2-D Gaussian landing from the spot:
Rayleigh (no bias) and Rice (a bias plus a spread). AIC picks the family. The probability that a
landing misses the latch's 0.15 m capture radius is the derived `p_latch_fault`, against the
placeholder 0.01. The core computes the same value from the fitted parameters
(`landing_miss_probability`, checked against scipy in its unit tests).

The placeholder Pittsburgh week is then run with each value, seeds 1 to 5. Marker detection
stays at 0.97. Writes analysis/out/latch.json and .md. Run: uv run analysis/latch.py"""

import json
import statistics
import tempfile
from pathlib import Path

import numpy as np
from scipy import stats

from variants import CONFIGS, OUT, ROOT, build_sim, quantity, run, variant

DATA = ROOT / "analysis" / "data" / "phase4_out_landing_error.json"
SCENARIO = "parts_logistics_pittsburgh_placeholder"
SEEDS = [1, 2, 3, 4, 5]
CM_PER_M = 100.0


def fit(errors_m: np.ndarray, radius_m: float) -> dict:
    sigma = stats.rayleigh.fit(errors_m, floc=0)[1]
    ll_rayleigh = stats.rayleigh.logpdf(errors_m, 0, sigma).sum()
    b, _, scale = stats.rice.fit(errors_m, floc=0)
    ll_rice = stats.rice.logpdf(errors_m, b, 0, scale).sum()
    families = {
        "rayleigh": {
            "bias_m": 0.0, "sigma_m": sigma, "aic": 2 * 1 - 2 * ll_rayleigh,
            "p_miss": float(stats.rayleigh.sf(radius_m, 0, sigma)),
        },
        "rice": {
            "bias_m": b * scale, "sigma_m": scale, "aic": 2 * 2 - 2 * ll_rice,
            "p_miss": float(stats.rice.sf(radius_m, b, 0, scale)),
        },
    }
    chosen = min(families, key=lambda f: families[f]["aic"])
    return {"families": families, "chosen": chosen}


def week(path: Path, root: Path, label: str) -> dict:
    per_seed = [run(path, s, root / f"{label}_{s}") for s in SEEDS]
    mean = lambda f: statistics.fmean(f(m) for m in per_seed)
    return {
        "latch_abort_count": mean(lambda m: m["latch_abort_count"]),
        "drone_mean_s": mean(lambda m: m["door_to_door_s"]["mean"]),
        "deliveries_per_h": mean(lambda m: m["deliveries_per_h"]),
    }


def main() -> None:
    data = json.loads(DATA.read_text())
    errors_m = np.array(list(data["out_landing_error_cm"].values())) / CM_PER_M
    radius_m = data["capture_radius_m"]
    result = {"n": len(errors_m), "capture_radius_m": radius_m, "current_p_latch_fault": 0.01}
    result |= fit(errors_m, radius_m)
    chosen = result["families"][result["chosen"]]
    build_sim()
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        derived = variant(SCENARIO, root, "derived_latch", {
            "schema_version": "1.6.0",
            "drone.p_latch_fault": None,
            "drone.latch_capture_radius_m": quantity(radius_m, "m"),
            "drone.out_landing_error_bias_m": quantity(chosen["bias_m"], "m"),
            "drone.out_landing_error_sigma_m": quantity(chosen["sigma_m"], "m"),
        })
        result["week_seeds_1_to_5"] = {
            "current": week(CONFIGS / SCENARIO / "scenario.toml", root, "current"),
            "derived": week(derived, root, "derived"),
        }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "latch.json").write_text(json.dumps(result, indent=1) + "\n")
    f, w = result["families"], result["week_seeds_1_to_5"]
    lines = [
        "| Model | Bias (cm) | Spread (cm) | AIC | p_latch_fault per attempt |",
        "|---|---|---|---|---|",
        "| Current placeholder | | | | 0.0100 |",
    ]
    for name in ("rayleigh", "rice"):
        x = f[name]
        mark = " (chosen)" if name == result["chosen"] else ""
        lines.append(
            f"| {name.capitalize()}{mark} | {x['bias_m'] * CM_PER_M:.1f} | {x['sigma_m'] * CM_PER_M:.1f} "
            f"| {x['aic']:.1f} | {x['p_miss']:.4f} |"
        )
    lines += [
        "",
        "| Placeholder week, seeds 1-5 | Latch aborts | Drone mean (s) | Deliveries/h |",
        "|---|---|---|---|",
    ]
    for label in ("current", "derived"):
        x = w[label]
        lines.append(
            f"| {label} | {x['latch_abort_count']:.1f} | {x['drone_mean_s']:.0f} | {x['deliveries_per_h']:.2f} |"
        )
    (OUT / "latch.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
