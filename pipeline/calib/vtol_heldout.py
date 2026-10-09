"""Phase 7 step 8: the fitted VTOL profile against the held-out flights (px4/runs/vtol_held:
5 kg at 2x, the longest legal lane under the VTOL profile and the shortest Mon Valley lane,
two seeds each). Nothing is refitted: the prediction is `sim flight-model` with the parameters
committed before these flights were flown (pipeline/calib/vtol_fitted_params.json). Writes
out/vtol_heldout.json; skipped, keeping the committed file, when the raw runs are gone.

Targets (placeholders): visit time within 5%, energy within 15%.

The flights must run the calibration batch's flight code: each run's `script_git` must be
clean, and px4/ at that revision (the committed sample logs aside, which no flight reads) must
equal px4/ at the calibration batch's revision. Commits elsewhere in the repository while the
batch flew move the revision without changing the flight code."""

import json
import subprocess

from common import CALIB, OUT, REPO, RUNS, sim_flight_model
from vtol_fit import scenario_for
from vtol_phases import run_rows

HELDOUT = RUNS / "vtol_held"
TIME_TARGET = 0.05
ENERGY_TARGET = 0.15


def same_flight_code(revision: str, reference: str) -> bool:
    if revision is None or revision.endswith("-dirty"):
        return False
    diff = subprocess.run(["git", "diff", "--quiet", reference, revision, "--", "px4", ":!px4/samples"],
                          cwd=REPO)
    return diff.returncode == 0


def evaluate(run, params) -> dict:
    rows, outcome = run_rows(run)
    r = {x["phase"]: x for x in rows}
    pred = sim_flight_model(outcome["lane_length_m"], outcome["payload_kg"], scenario_for(params, "vtol_heldout"))
    lane_pred = {k: pred["transition"][k] + pred["cruise"][k] for k in ("time_s", "energy_j")}
    measured = {"takeoff": (r["mc_takeoff"]["duration_s"], r["mc_takeoff"]["energy_j"]),
                "lane": (outcome["lane_time_s"], outcome["lane_energy_j"]),
                "approach": (r["approach_landing"]["duration_s"], r["approach_landing"]["energy_j"]),
                "hop": (r["hop"]["duration_s"], r["hop"]["energy_j"])}
    predicted = {"takeoff": pred["takeoff"], "lane": lane_pred, "approach": pred["approach"],
                 "hop": pred["hop"]}
    phases = {k: {"meas_time_s": round(measured[k][0], 2), "pred_time_s": round(predicted[k]["time_s"], 2),
                  "meas_energy_j": round(measured[k][1], 1), "pred_energy_j": round(predicted[k]["energy_j"], 1)}
              for k in measured}
    t = sum(v[0] for v in measured.values())
    e = sum(v[1] for v in measured.values())
    time_err = pred["total"]["time_s"] / t - 1
    energy_err = pred["total"]["energy_j"] / e - 1
    return {**{k: outcome[k] for k in ("run", "status", "script_git", "validation", "wall_s", "peak_rss_mb",
                                       "lane_id", "lane_length_m", "landings_cm")},
            "phases": phases, "visit_time_measured_s": round(t, 2),
            "visit_time_predicted_s": round(pred["total"]["time_s"], 2), "time_error": round(time_err, 4),
            "visit_energy_measured_j": round(e, 1), "visit_energy_predicted_j": round(pred["total"]["energy_j"], 1),
            "energy_error": round(energy_err, 4),
            "within_targets": abs(time_err) <= TIME_TARGET and abs(energy_err) <= ENERGY_TARGET}


def main() -> None:
    runs = sorted(p for p in HELDOUT.iterdir() if (p / "run.json").exists()) if HELDOUT.exists() else []
    if not runs:
        print(f"{HELDOUT} not on disk: keeping the committed out/vtol_heldout.json")
        return
    params = json.loads((CALIB / "vtol_fitted_params.json").read_text())["params"]
    reference = json.loads((CALIB / "vtol_phases.json").read_text())["script_git"]
    for run in runs:
        revision = json.loads((run / "run.json").read_text()).get("script_git")
        if not same_flight_code(revision, reference):
            raise RuntimeError(f"{run.name}: flight code at {revision} differs from the calibration's {reference}")
    results = []
    for run in runs:
        status = (run / "run_status").read_text().strip() if (run / "run_status").exists() else "NO STATUS"
        results.append(evaluate(run, params) if status == "COMPLETE" else {"run": run.name, "status": status})
    done = [r for r in results if r.get("status") == "COMPLETE"]
    out = {"targets": {"time": TIME_TARGET, "energy": ENERGY_TARGET}, "calibration_script_git": reference,
           "runs": results,
           "max_abs_time_error": max((abs(r["time_error"]) for r in done), default=None),
           "max_abs_energy_error": max((abs(r["energy_error"]) for r in done), default=None)}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "vtol_heldout.json").write_text(json.dumps(out, indent=1) + "\n")
    for r in results:
        if r.get("status") == "COMPLETE":
            print(f"{r['run']:22s} lane {r['lane_id']} {r['lane_length_m']:8.1f} m time {r['time_error']:+.2%} "
                  f"energy {r['energy_error']:+.2%}")
        else:
            print(f"{r['run']:22s} {r['status']}")


if __name__ == "__main__":
    main()
