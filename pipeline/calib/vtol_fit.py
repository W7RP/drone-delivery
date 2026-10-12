"""Phase 7 step 7: fit the VTOL flight profile (spec 2.7, 8.7, A93) to the per-phase table of
the calibration flights (pipeline/calib/vtol_phases.json). Deterministic: fixed bootstrap
seed, inputs sorted. Writes pipeline/calib/vtol_fitted_params.json and out/vtol_fit.json.

Model, as the core uses it (sim/src/flight.rs, VisitEstimate):
  visit time = takeoff_s + transition_s + L / cruise_speed_mps + approach_s + hop_s
  multicopter power = hover_power_ref_w (m / m_ref)^1.5        (takeoff, approach, hop)
  cruise power      = cruise_power_ref_w (m / m_ref)^n          (the lane, fixed wing)
  transition power  = transition_power_ref_w (m / m_ref)^1.5    (for transition_s per flight)
with m the total mass and m_ref = 14.6 kg. Classes and parameters (at most three each):
  multicopter  hover_power_ref_w; takeoff_s, approach_s, hop_s are the measured durations
  cruise       cruise_speed_mps (plateau ground speed), cruise_power_ref_w, n
  transition   transition_s = lane time (D02 to D03) - L / cruise_speed_mps per run, and
               transition_power_ref_w on the lane energy left after the cruise model
Times are constants unless a regression on payload has a slope significant at 5% (t-test
over the fit runs); n is 0 (cruise power independent of mass) unless the regression of log
cruise power on log mass has a significant slope, then that slope. Powers are least squares on
relative energy residuals. Validation: leave-one-payload-out with the core's own model (`sim
flight-model`), and a bootstrap over runs for the parameters' uncertainty. Only the 2x
fit-lane runs are fitted; the 1x run is compared, not used."""

import json

import numpy as np
from scipy import stats

from common import CALIB, OUT, REPO, sim_flight_model

TABLE = CALIB / "vtol_phases.json"
PARAMS = CALIB / "vtol_fitted_params.json"
SIGNIFICANCE = 0.05
BOOTSTRAP_DRAWS = 2000
BOOTSTRAP_SEED = 7
POWER_REF_MASS_KG = 14.6
FIT_SPEED_FACTOR = 2
MTOW_KG = 16.0
MC_PHASES = ("mc_takeoff", "approach_landing", "hop")
TIME_KEYS = {"takeoff_s": "mc_takeoff", "approach_s": "approach_landing", "hop_s": "hop"}
PLACEHOLDER = REPO / "configs" / "parts_logistics_pittsburgh_placeholder" / "scenario.toml"


def significant_slope(x, y) -> tuple[bool, float, float]:
    if len(set(x)) < 2:
        return False, 0.0, 1.0
    r = stats.linregress(x, y)
    return bool(r.pvalue < SIGNIFICANCE), float(r.slope), float(r.pvalue)


def relative_ls(x: np.ndarray, e: np.ndarray) -> float:
    """a minimising sum ((e - a x) / e)^2."""
    w = 1.0 / e
    return float(np.sum(w * w * x * e) / np.sum(w * w * x * x))


def fit(rows: list[dict], runs: list[dict]) -> dict:
    by = lambda phase: [r for r in rows if r["phase"] == phase]
    ratio = lambda r: r["mass_kg"] / POWER_REF_MASS_KG
    params, tests = {}, {}
    for key, phase in TIME_KEYS.items():
        rs = by(phase)
        sig, slope, p = significant_slope([r["payload_kg"] for r in rs], [r["duration_s"] for r in rs])
        tests[key] = {"slope_s_per_kg": slope, "p": p, "significant": sig}
        params[key] = float(np.mean([r["duration_s"] for r in rs]))
    mc = [r for r in rows if r["phase"] in MC_PHASES]
    params["hover_power_ref_w"] = relative_ls(
        np.array([ratio(r) ** 1.5 * r["duration_s"] for r in mc]), np.array([r["energy_j"] for r in mc]))
    speeds = [o["plateau_speed_mps"] for o in runs]
    sig, slope, p = significant_slope([o["payload_kg"] for o in runs], speeds)
    tests["cruise_speed_mps"] = {"slope_mps_per_kg": slope, "p": p, "significant": sig}
    params["cruise_speed_mps"] = float(np.mean(speeds))
    cruise = by("cruise")
    sig, slope, p = significant_slope([np.log(ratio(r)) for r in cruise],
                                      [np.log(r["mean_power_w"]) for r in cruise])
    tests["cruise_mass_exponent"] = {"slope": slope, "p": p, "significant": sig}
    n = slope if sig else 0.0
    params["cruise_mass_exponent"] = float(n)
    params["cruise_power_ref_w"] = relative_ls(
        np.array([ratio(r) ** n * r["duration_s"] for r in cruise]), np.array([r["energy_j"] for r in cruise]))
    over_t = [o["lane_time_s"] - o["lane_length_m"] / params["cruise_speed_mps"] for o in runs]
    sig, slope, p = significant_slope([o["payload_kg"] for o in runs], over_t)
    tests["transition_s"] = {"slope_s_per_kg": slope, "p": p, "significant": sig}
    params["transition_s"] = float(np.mean(over_t))
    mass = {o["run"]: next(r["mass_kg"] for r in cruise if r["run"] == o["run"]) for o in runs}
    cruise_j = [params["cruise_power_ref_w"] * (mass[o["run"]] / POWER_REF_MASS_KG) ** n
                * o["lane_length_m"] / params["cruise_speed_mps"] for o in runs]
    over_e = np.array([o["lane_energy_j"] - c for o, c in zip(runs, cruise_j)])
    params["transition_power_ref_w"] = relative_ls(
        np.array([(mass[o["run"]] / POWER_REF_MASS_KG) ** 1.5 * params["transition_s"] for o in runs]),
        over_e)
    params["power_ref_mass_kg"] = POWER_REF_MASS_KG
    return {"params": params, "tests": tests}


def profile_text(p: dict, name: str = "px4_sitl_vtol_proxy") -> str:
    q = lambda k, unit: f'{k} = {{ value = {round(p[k], 4)!r}, unit = "{unit}", assumption = true }}'
    lines = [f'name = "{name}"', 'source = "px4-sitl-proxy"', "", "[drone]", 'airframe = "VTOL"',
             q("cruise_speed_mps", "m/s"), q("cruise_power_ref_w", "W"), q("hover_power_ref_w", "W"),
             q("power_ref_mass_kg", "kg"), q("takeoff_s", "s"), q("approach_s", "s"), q("hop_s", "s"),
             q("transition_s", "s"), q("transition_power_ref_w", "W"), q("cruise_mass_exponent", "1"),
             f'mtow_kg = {{ value = {MTOW_KG!r}, unit = "kg", assumption = true }}']
    return "\n".join(lines) + "\n"


def scenario_for(p: dict, tag: str):
    """The placeholder Pittsburgh scenario with this profile, for `sim flight-model`."""
    d = OUT / "models" / tag
    d.mkdir(parents=True, exist_ok=True)
    (d / "profile.toml").write_text(profile_text(p, tag))
    text = PLACEHOLDER.read_text()
    text = text.replace('dir = "derived"', f'dir = "{PLACEHOLDER.parent / "derived"}"')
    text = text.replace('schema_version = "1.5.0"', 'schema_version = "1.9.0"', 1)
    text += f'\n[flight_profile]\nfile = "{d / "profile.toml"}"\n'
    (d / "scenario.toml").write_text(text)
    return d / "scenario.toml"


def measured_visit(rows: list[dict], o: dict) -> tuple[float, float]:
    """Visit time and energy as the core models them: takeoff, lane (D02 to D03), approach and
    landing, hop. The hold at the fix is excluded (B granted IN at once)."""
    rs = {r["phase"]: r for r in rows if r["run"] == o["run"]}
    t = rs["mc_takeoff"]["duration_s"] + o["lane_time_s"] + rs["approach_landing"]["duration_s"] + rs["hop"]["duration_s"]
    e = rs["mc_takeoff"]["energy_j"] + o["lane_energy_j"] + rs["approach_landing"]["energy_j"] + rs["hop"]["energy_j"]
    return t, e


def predict(p: dict, lane_m: float, payload_kg: float, tag: str) -> dict:
    return sim_flight_model(lane_m, payload_kg, scenario_for(p, tag))["total"]


def main() -> None:
    data = json.loads(TABLE.read_text())
    runs = sorted((o for o in data["runs"] if o["status"] == "COMPLETE"), key=lambda o: o["run"])
    fit_runs = [o for o in runs if o["speed_factor"] == FIT_SPEED_FACTOR]
    names = {o["run"] for o in fit_runs}
    rows = sorted((r for r in data["phases"] if r["run"] in names), key=lambda r: (r["run"], r["phase"]))
    result = fit(rows, fit_runs)
    params = result["params"]
    # Residuals of each class at the fitted parameters.
    ratio = lambda r: r["mass_kg"] / POWER_REF_MASS_KG
    res = {}
    for r in rows:
        if r["phase"] in MC_PHASES:
            model = params["hover_power_ref_w"] * ratio(r) ** 1.5 * r["duration_s"]
        elif r["phase"] == "cruise":
            model = params["cruise_power_ref_w"] * ratio(r) ** params["cruise_mass_exponent"] * r["duration_s"]
        else:
            continue
        res.setdefault(r["phase"], []).append((r["energy_j"] - model) / r["energy_j"])
    residuals = {k: {"mean": round(float(np.mean(v)), 4), "max_abs": round(float(np.max(np.abs(v))), 4)}
                 for k, v in res.items()}
    # Visits: in-sample and leave-one-payload-out with the core's model.
    def visit_errors(p, subset, tag):
        out = []
        for o in subset:
            t_m, e_m = measured_visit(rows if o["run"] in names else data["phases"], o)
            pr = predict(p, o["lane_length_m"], o["payload_kg"], tag)
            out.append({"run": o["run"], "payload_kg": o["payload_kg"],
                        "time_measured_s": round(t_m, 2), "time_predicted_s": round(pr["time_s"], 2),
                        "time_error": round(pr["time_s"] / t_m - 1, 4),
                        "energy_measured_j": round(e_m, 1), "energy_predicted_j": round(pr["energy_j"], 1),
                        "energy_error": round(pr["energy_j"] / e_m - 1, 4)})
        return out
    in_sample = visit_errors(params, fit_runs, "vtol_fit")
    lopo = []
    for payload in sorted({o["payload_kg"] for o in fit_runs}):
        train = [o for o in fit_runs if o["payload_kg"] != payload]
        tn = {o["run"] for o in train}
        p = fit([r for r in rows if r["run"] in tn], train)["params"]
        lopo += visit_errors(p, [o for o in fit_runs if o["payload_kg"] == payload], f"vtol_lopo_{payload:g}")
    # Bootstrap over runs.
    rng = np.random.default_rng(BOOTSTRAP_SEED)
    draws = []
    for _ in range(BOOTSTRAP_DRAWS):
        pick = [fit_runs[i] for i in rng.integers(0, len(fit_runs), len(fit_runs))]
        if len({o["payload_kg"] for o in pick}) < 2:
            continue
        sel_rows = [r for o in pick for r in rows if r["run"] == o["run"]]
        draws.append(fit(sel_rows, pick)["params"])
    std = {k: float(np.std([d[k] for d in draws])) for k in params}
    # 1x against 2x at the same payload and seed.
    one_x = [o for o in runs if o["speed_factor"] == 1]
    speed_check = []
    for o in one_x:
        twin = next((f for f in fit_runs if f["payload_kg"] == o["payload_kg"] and f["seed"] == o["seed"]), None)
        if twin:
            speed_check.append({"run_1x": o["run"], "run_2x": twin["run"],
                                "lane_time_diff": round(twin["lane_time_s"] / o["lane_time_s"] - 1, 4),
                                "lane_energy_diff": round(twin["lane_energy_j"] / o["lane_energy_j"] - 1, 4),
                                "plateau_speed_diff": round(twin["plateau_speed_mps"] / o["plateau_speed_mps"] - 1, 4)})
    rnd = lambda d: {k: round(v, 4) for k, v in d.items()}
    fitted = {"params": rnd(params), "std": rnd(std), "mtow_kg": MTOW_KG,
              "runs": sorted(names), "bootstrap": {"draws": len(draws), "seed": BOOTSTRAP_SEED}}
    PARAMS.write_text(json.dumps(fitted, indent=1) + "\n")
    report = {**fitted, "tests": result["tests"], "class_residuals": residuals,
              "in_sample": in_sample, "leave_one_payload_out": lopo,
              "lopo_max_abs_error": {"time": max(abs(x["time_error"]) for x in lopo),
                                     "energy": max(abs(x["energy_error"]) for x in lopo)},
              "speed_factor_check": speed_check,
              "cross_check": [{"run": o["run"], **o["cross_check"]} for o in runs]}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "vtol_fit.json").write_text(json.dumps(report, indent=1) + "\n")
    for k in params:
        print(f"{k:24s} {params[k]:10.4f} +- {std[k]:.4f}")
    print("LOPO max |error|: time {time:.2%}, energy {energy:.2%}".format(**report["lopo_max_abs_error"]))


if __name__ == "__main__":
    main()
