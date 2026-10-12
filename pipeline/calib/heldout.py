"""Phase 5 step 5: the fitted profile against the held-out flights (px4/runs/heldout: 5 kg,
2x speed, the shortest and longest legal lanes, two seeds each). Nothing is refitted: the
prediction is `sim flight-model` with the parameters committed before these flights were flown
(pipeline/calib/fitted_params.json). Writes out/heldout.json, the held-out flight summaries.

It also reports the fit-lane check (px4/runs/fitlane): a 5 kg flight on the fit lane after the
SITL change of spec A67, against the prediction and against the fit's own 5 kg flights."""

import json

from common import OUT, PHASE_KEYS, RUNS, CALIB, flightlog, run_meta, scenario_with, sim_flight_model
from phases import lane_length_m, lane_record, phase_rows

HELDOUT = RUNS / "heldout"
TIME_TARGET = 0.05
ENERGY_TARGET = 0.15


FITLANE = RUNS / "fitlane"


def finished(batch):
    return sorted(p for p in batch.iterdir() if (p / "run_status").exists()) if batch.exists() else []


def evaluate(run, config) -> dict:
    """One flight against the profile in `config`: per-phase and visit time and energy."""
    problems = flightlog.check(run)
    meta = run_meta(run)
    lane = lane_record(run)
    rows = {r["phase"]: r for r in phase_rows(run)}
    pred = sim_flight_model(lane_length_m(run), meta["payload_kg"], config)
    phases = {}
    for phase, key in PHASE_KEYS.items():
        m, p = rows[phase], pred[key]
        phases[phase] = {"meas_time_s": m["duration_s"], "pred_time_s": round(p["time_s"], 2),
                         "meas_energy_j": m["energy_j"], "pred_energy_j": round(p["energy_j"])}
    t = sum(r["duration_s"] for r in rows.values())
    e = sum(r["energy_j"] for r in rows.values())
    cruise = rows["cruise"]
    return {
        "run": run.name, "seed": meta["seed"], "payload_kg": meta["payload_kg"],
        "speed_factor": meta["speed"], "valid": not problems, "problems": problems,
        "lane_id": lane["lane_id"], "from": lane["from_pad"]["name"], "to": lane["to_pad"]["name"],
        "lane_length_m": round(lane["length_m"], 1),
        "along_track_m": cruise["horizontal_m"], "straight_m": cruise["straight_m"],
        "along_track_over_lane": round(cruise["horizontal_m"] / lane["length_m"], 4),
        "meas_time_s": round(t, 2), "pred_time_s": round(pred["total"]["time_s"], 2),
        "time_error": round((pred["total"]["time_s"] - t) / t, 4),
        "meas_energy_j": round(e), "pred_energy_j": round(pred["total"]["energy_j"]),
        "energy_error": round((pred["total"]["energy_j"] - e) / e, 4),
        "phases": phases,
    }


def fit_runs_at(payload_kg: float) -> dict:
    """Mean measured visit time and energy of the fit's flights at one payload."""
    visits = {}
    for r in json.loads((OUT / "phases.json").read_text()):
        if r["payload_kg"] == payload_kg:
            v = visits.setdefault(r["run"], [0.0, 0.0])
            v[0] += r["duration_s"]
            v[1] += r["energy_j"]
    times, energies = zip(*visits.values())
    return {"runs": len(visits), "mean_time_s": round(sum(times) / len(times), 2),
            "mean_energy_j": round(sum(energies) / len(energies))}


def report(f: dict) -> str:
    return (f"{f['run']:16s} lane {f['lane_id']:2d} {f['lane_length_m']:8.1f} m, flown {f['along_track_m']:8.1f} m, "
            f"time {f['meas_time_s']:7.1f} vs {f['pred_time_s']:7.1f} s ({f['time_error']:+.1%}), "
            f"energy {f['meas_energy_j'] / 1000:7.1f} vs {f['pred_energy_j'] / 1000:7.1f} kJ ({f['energy_error']:+.1%})"
            f"{'' if f['valid'] else '  INVALID ' + str(f['problems'])}")


def main() -> None:
    fitted = json.loads((CALIB / "fitted_params.json").read_text())["params"]
    config = scenario_with(fitted, "fitted")
    flights = [evaluate(run, config) for run in finished(HELDOUT)]
    worst = {"time": max(abs(f["time_error"]) for f in flights),
             "energy": max(abs(f["energy_error"]) for f in flights)}
    checks = []
    for run in finished(FITLANE):
        f = evaluate(run, config)
        base = fit_runs_at(f["payload_kg"])
        f["fit_runs"] = base
        f["time_vs_fit_runs"] = round((f["meas_time_s"] - base["mean_time_s"]) / base["mean_time_s"], 4)
        f["energy_vs_fit_runs"] = round((f["meas_energy_j"] - base["mean_energy_j"]) / base["mean_energy_j"], 4)
        checks.append(f)
    result = {"flights": flights, "max_abs_error": worst,
              "targets": {"time": TIME_TARGET, "energy": ENERGY_TARGET},
              "within_targets": worst["time"] <= TIME_TARGET and worst["energy"] <= ENERGY_TARGET,
              "fitlane_check": checks}
    (OUT / "heldout.json").write_text(json.dumps(result, indent=1) + "\n")
    for f in flights:
        print(report(f))
    print(f"max |error|: time {worst['time']:.1%}, energy {worst['energy']:.1%}; "
          f"within targets: {result['within_targets']}")
    for f in checks:
        print(report(f) + f"; against the fit's {f['fit_runs']['runs']} flights at {f['payload_kg']} kg: "
              f"time {f['time_vs_fit_runs']:+.1%}, energy {f['energy_vs_fit_runs']:+.1%}")


if __name__ == "__main__":
    main()
