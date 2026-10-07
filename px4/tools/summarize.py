"""Summary of a batch of SITL runs (tools/run_batch.sh): writes <batch>/summary.json and the
markdown tables docs/flight-runs.md quotes. Usage: summarize.py <batch_dir>"""

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import flightlog  # noqa: E402

PX4 = Path(__file__).resolve().parents[1]


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return (math.nan, math.nan)
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - h) / d, (c + h) / d)


def stats(values: list[float]) -> dict:
    a = np.array(values, dtype=float)
    if len(a) == 0:
        return {}
    return {"n": len(a), "mean": round(float(a.mean()), 2),
            "p95": round(float(np.percentile(a, 95)), 2), "max": round(float(a.max()), 2)}


def run_info(run: Path) -> dict:
    info = json.loads((run / "run.json").read_text())
    status = (run / "run_status").read_text().strip() if (run / "run_status").exists() else "MISSING"
    problems = flightlog.check(run) if status == "COMPLETE" else [status]
    out = {"run": run.name, **info, "status": status, "valid": not problems,
           "problems": problems}
    if (run / "wall_s").exists():
        out["wall_s"] = int((run / "wall_s").read_text())
    if (run / "rss.json").exists():
        out["rss"] = json.loads((run / "rss.json").read_text())
    if status == "COMPLETE":
        out.update(flightlog.summary(run))
    return out


def main() -> int:
    batch = Path(sys.argv[1])
    runs = [run_info(r) for r in sorted(p for p in batch.iterdir() if (p / "run.json").exists())]
    by = {r["run"]: r for r in runs}
    cfg = json.loads((PX4 / "config" / "lane.json").read_text())
    import yaml
    flight = yaml.safe_load((PX4 / "config" / "flight.yaml").read_text())
    capture_cm = flight["pads"]["latch_capture_radius_m"] * 100

    repeats = [r for r in runs if r["run"].startswith("repeat_")]
    landings = {"IN": [], "OUT": []}
    attempts = successes = 0
    for r in repeats:
        # Each repeat makes two landing attempts (IN, then OUT). A landing succeeds when the
        # marker was found and the drone touched down inside the latch capture radius; a run that
        # ended before a landing counts that landing (and any after it) as failed.
        attempts += 2
        errs = r.get("landing_error_cm", {})
        for spot in ("IN", "OUT"):
            if spot in errs:
                landings[spot].append(errs[spot])
                successes += errs[spot] <= capture_cm
    lo, hi = wilson(successes, attempts)
    summary = {
        "batch": batch.name,
        "runs": runs,
        "repeats": {
            "runs": len(repeats),
            "complete_and_valid": sum(r["valid"] for r in repeats),
            "landing_error_cm": {s: stats(v) for s, v in landings.items()},
            "landing_success": {"successes": successes, "attempts": attempts,
                                "rate": successes / attempts if attempts else None,
                                "wilson95": [round(lo, 3), round(hi, 3)],
                                "criterion": f"marker found and touchdown within {capture_cm:.0f} cm"},
            "config_p_marker_detect": cfg["pad_ops"]["p_marker_detect"],
            # Laplace's rule: (k + 1) / (n + 2), so 20 of 20 does not claim certainty.
            "recommended_p_marker_detect": round((successes + 1) / (attempts + 2), 3) if attempts else None,
        },
        "peak_rss_mb": max((r["rss"]["peak_total_mb"] for r in runs if "rss" in r), default=None),
    }
    if "repeat_1" in by and "speed2_seed1" in by:
        summary["speed_comparison"] = {k: {f: by[k].get(f) for f in
                                           ("phases_s", "energy_proxy_j", "landing_error_cm", "wall_s")}
                                       for k in ("repeat_1", "speed2_seed1")}
    (batch / "summary.json").write_text(json.dumps(summary, indent=1, default=str) + "\n")

    lines = ["| Payload (kg) | Status | Airborne total (s) | Takeoff (s) | Cruise (s) | Approach + landing at IN (s) | Hop (s) | Energy proxy total (kJ) | Cruise energy (kJ) | Mean cruise power (W) | Landing error IN / OUT (cm) |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in sorted((r for r in runs if r["run"].startswith("payload_")), key=lambda r: r["payload_kg"]):
        if r["status"] != "COMPLETE":
            lines.append(f"| {r['payload_kg']} | {r['status']} | | | | | | | | | |")
            continue
        p, e = r["phases_s"], r["energy_proxy_j"]
        lines.append(
            f"| {r['payload_kg']} | {'valid' if r['valid'] else 'INVALID'} | {p['airborne_total_s']:.1f} | "
            f"{p['takeoff_s']:.1f} | {p['cruise_s']:.1f} | {p['approach_landing_s']:.1f} | "
            f"{p['hop_s']:.1f} | {e['total_j'] / 1000:.0f} | {e['cruise_j'] / 1000:.0f} | "
            f"{e['cruise_j'] / p['cruise_s']:.0f} | {r['landing_error_cm'].get('IN', math.nan):.1f} / "
            f"{r['landing_error_cm'].get('OUT', math.nan):.1f} |")
    (batch / "summary.md").write_text("\n".join(lines) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "runs"}, indent=1, default=str))
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
