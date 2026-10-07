"""Phase 5 step 4: fit the sim's flight model (spec 2.7) to the SITL fit logs. Deterministic:
fixed bootstrap seed, inputs sorted. Writes out/fit.json and pipeline/calib/fitted_params.json.

Model, as the core uses it (sim/src/flight.rs, VisitEstimate):
  visit time   = takeoff_s + L / cruise_speed_mps + approach_s + hop_s
  cruise power = cruise_power_ref_w * (m / power_ref_mass_kg)^1.5      (the lane)
  hover power  = hover_power_ref_w  * (m / power_ref_mass_kg)^1.5      (takeoff, approach, hop)
with m the total mass (airframe + latch + pod; the hop is flown empty).

Time constants:
  cruise_speed_mps  the plateau ground speed in the lane. With one lane length in the fit data a
                    speed and a constant overhead cannot both be fitted from cruise times, so the
                    speed is measured directly and the overhead (speed-up and slow-down) is
  takeoff_s         the takeoff duration plus that overhead, since the core times a lane as L/v
  approach_s        the approach and landing duration (D04 to D05)
  hop_s             the hop duration
  Each is a constant unless a regression of the duration on payload has a slope significant at
  the 5% level (t-test on the slope over all fit runs); significance is reported either way.
Power: one coefficient per class by least squares on relative energy residuals
(minimise sum ((E_meas - E_model) / E_meas)^2), linear in the coefficient. A drag term would be
added only if the class's residuals exceeded 10% without it. The hover class does exceed 10%,
but per phase and not with mass (approach +12%, hop -12%, takeoff +3%), so no drag or mass term
can fix it; the spec's one hover power per drone cannot hold a per-phase factor, and none is
added (reported in out/fit.json as hover_class_residual_by_phase).
Validation: leave-one-payload-out (fit without a payload, predict its visits with the core's own
model) and a bootstrap over runs (seeds) for parameter uncertainty."""

import json

import numpy as np
from scipy import stats

from common import CALIB, OUT, flightlog, fit_runs, run_meta, scenario_with, sim_flight_model

SIGNIFICANCE = 0.05
DRAG_TERM_THRESHOLD = 0.10
BOOTSTRAP_DRAWS = 2000
BOOTSTRAP_SEED = 5
POWER_REF_MASS_KG = 14.6  # the scenario's drone.power_ref_mass_kg, kept
HOVER_PHASES = ("takeoff", "approach_landing", "hop")


def plateau_speed(run) -> float:
    """Median horizontal speed over the middle half of the lane's time, where the carrot moves
    at the commanded speed (speed-up and slow-down take a few seconds at each end)."""
    s = flightlog.read_samples(run / "flight_samples.bin")
    t = flightlog.phase_times(flightlog.read_events(run / "events.jsonl"))["_times"]
    ts = s["t_us"] * 1e-6
    quarter = (t["fix"] - t["top_of_climb"]) / 4
    sel = (ts > t["top_of_climb"] + quarter) & (ts < t["fix"] - quarter)
    v = np.linalg.norm(s["vel_ned_mps"][sel, :2].astype(float), axis=1)
    return float(np.median(v))


def power_coefficient(rows: list[dict]) -> tuple[float, float]:
    """Least squares on relative residuals of E = a (m / m_ref)^1.5 t; returns (a, max |rel res|)."""
    x = np.array([(r["mass_kg"] / POWER_REF_MASS_KG) ** 1.5 * r["duration_s"] for r in rows])
    e = np.array([r["energy_j"] for r in rows])
    w = 1.0 / e
    a = float(np.sum(w * w * x * e) / np.sum(w * w * x * x))
    return a, float(np.max(np.abs((e - a * x) / e)))


def fit(rows: list[dict], speeds: dict[str, float]) -> dict:
    by = {p: [r for r in rows if r["phase"] == p] for p in ("takeoff", "cruise", "approach_landing", "hop")}
    v = float(np.mean([speeds[r["run"]] for r in by["cruise"]]))
    overhead = float(np.mean([r["duration_s"] - r["horizontal_m"] / v for r in by["cruise"]]))
    a_c, res_c = power_coefficient(by["cruise"])
    a_h, res_h = power_coefficient([r for p in HOVER_PHASES for r in by[p]])
    return {
        "cruise_speed_mps": v,
        "takeoff_s": float(np.mean([r["duration_s"] for r in by["takeoff"]])) + overhead,
        "approach_s": float(np.mean([r["duration_s"] for r in by["approach_landing"]])),
        "hop_s": float(np.mean([r["duration_s"] for r in by["hop"]])),
        "cruise_power_ref_w": a_c,
        "hover_power_ref_w": a_h,
        "power_ref_mass_kg": POWER_REF_MASS_KG,
        "_lane_overhead_s": overhead,
        "_max_rel_residual": {"cruise": res_c, "hover": res_h},
    }


def hover_residuals(rows: list[dict], a_h: float) -> dict:
    """Mean relative residual of the hover-class fit per phase and payload."""
    out = {}
    for phase in HOVER_PHASES:
        for payload in sorted({r["payload_kg"] for r in rows}):
            rs = [r for r in rows if r["phase"] == phase and r["payload_kg"] == payload]
            res = [(r["energy_j"] - a_h * (r["mass_kg"] / POWER_REF_MASS_KG) ** 1.5 * r["duration_s"])
                   / r["energy_j"] for r in rs]
            out[f"{phase}@{payload:g}kg"] = round(float(np.mean(res)), 4)
    return out


def visit_measured(rows: list[dict], run: str) -> tuple[float, float]:
    rs = [r for r in rows if r["run"] == run]
    return sum(r["duration_s"] for r in rs), sum(r["energy_j"] for r in rs)


def predict(params: dict, lane_m: float, payload_kg: float, tag: str) -> dict:
    values = {k: v for k, v in params.items() if not k.startswith("_")}
    return sim_flight_model(lane_m, payload_kg, scenario_with(values, tag))


def main() -> None:
    rows = json.loads((OUT / "phases.json").read_text())
    runs = fit_runs()
    meta = {r.name: run_meta(r) for r in runs}
    speeds = {r.name: plateau_speed(r) for r in runs}
    names = sorted(meta)
    params = fit(rows, speeds)

    # Payload dependence of each duration.
    tests = {}
    for phase in ("takeoff", "cruise", "approach_landing", "hop"):
        rs = [r for r in rows if r["phase"] == phase]
        lr = stats.linregress([r["payload_kg"] for r in rs], [r["duration_s"] for r in rs])
        tests[phase] = {"slope_s_per_kg": round(lr.slope, 4), "p_value": round(lr.pvalue, 5),
                        "significant": bool(lr.pvalue < SIGNIFICANCE),
                        "effect_0_to_10_kg_s": round(10 * lr.slope, 2)}

    # Leave one payload out.
    cv = []
    lane_m = next(r["lane_length_m"] for r in rows if r["phase"] == "cruise")
    for payload in sorted({m["payload_kg"] for m in meta.values()}):
        train = [r for r in rows if r["payload_kg"] != payload]
        p = fit(train, {k: v for k, v in speeds.items() if meta[k]["payload_kg"] != payload})
        pred = predict(p, lane_m, payload, f"cv_{payload:g}")["total"]
        for name in names:
            if meta[name]["payload_kg"] != payload:
                continue
            t, e = visit_measured(rows, name)
            cv.append({"run": name, "payload_kg": payload, "pred_time_s": round(pred["time_s"], 2),
                       "meas_time_s": round(t, 2), "time_error": round((pred["time_s"] - t) / t, 4),
                       "pred_energy_j": round(pred["energy_j"]), "meas_energy_j": round(e),
                       "energy_error": round((pred["energy_j"] - e) / e, 4)})

    # Bootstrap over runs.
    rng = np.random.default_rng(BOOTSTRAP_SEED)
    draws = {k: [] for k in params if not k.startswith("_")}
    for _ in range(BOOTSTRAP_DRAWS):
        pick = rng.choice(len(names), size=len(names), replace=True)
        chosen = [names[i] for i in pick]
        boot_rows = [dict(r, run=f"{r['run']}#{j}") for j, n in enumerate(chosen) for r in rows if r["run"] == n]
        boot_speeds = {f"{n}#{j}": speeds[n] for j, n in enumerate(chosen)}
        b = fit(boot_rows, boot_speeds)
        for k in draws:
            draws[k].append(b[k])
    uncertainty = {k: {"std": float(np.std(v)), "p2_5": float(np.percentile(v, 2.5)),
                       "p97_5": float(np.percentile(v, 97.5))} for k, v in draws.items()}

    # In-sample check through the core's own model, per payload.
    fitted = predict(params, lane_m, 5.0, "fitted")
    insample = []
    for name in names:
        pred = predict(params, lane_m, meta[name]["payload_kg"], f"in_{meta[name]['payload_kg']:g}")["total"]
        t, e = visit_measured(rows, name)
        insample.append({"run": name, "time_error": round((pred["time_s"] - t) / t, 4),
                         "energy_error": round((pred["energy_j"] - e) / e, 4)})

    def round_params(p):
        return {k: round(v, 3) if isinstance(v, float) else v for k, v in p.items() if not k.startswith("_")}

    result = {
        "source": "px4-sitl-proxy",
        "fit_runs": names,
        "lane_m": lane_m,
        "params": round_params(params),
        "lane_overhead_s": round(params["_lane_overhead_s"], 3),
        "max_relative_residual": {k: round(v, 4) for k, v in params["_max_rel_residual"].items()},
        "residual_over_threshold": {k: v > DRAG_TERM_THRESHOLD for k, v in params["_max_rel_residual"].items()},
        "terms_added": [],
        "hover_class_residual_by_phase": hover_residuals(rows, params["hover_power_ref_w"]),
        "payload_dependence_tests": tests,
        "uncertainty": {k: {kk: round(vv, 3) for kk, vv in v.items()} for k, v in uncertainty.items()},
        "leave_one_payload_out": cv,
        "cv_max_abs_error": {"time": max(abs(c["time_error"]) for c in cv),
                             "energy": max(abs(c["energy_error"]) for c in cv)},
        "in_sample": insample,
        "fitted_visit_5kg": fitted["total"],
        "bootstrap": {"draws": BOOTSTRAP_DRAWS, "seed": BOOTSTRAP_SEED},
    }
    (OUT / "fit.json").write_text(json.dumps(result, indent=1) + "\n")
    (CALIB / "fitted_params.json").write_text(json.dumps(
        {"source": "px4-sitl-proxy", "assumption": True, "params": result["params"],
         "uncertainty_std": {k: v["std"] for k, v in result["uncertainty"].items()},
         "fit": "pipeline/calib/fit.py on px4/runs/accept (Phase 4 acceptance runs)"},
        indent=1) + "\n")
    print(json.dumps({k: result[k] for k in ("params", "lane_overhead_s", "max_relative_residual",
                                              "terms_added", "payload_dependence_tests",
                                              "cv_max_abs_error", "uncertainty")}, indent=1))


if __name__ == "__main__":
    main()
