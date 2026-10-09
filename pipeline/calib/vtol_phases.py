"""Phase 7 step 6: per-phase table of the VTOL calibration flights (px4/runs/vtol_cal), one
row per run and phase, plus an outcome record for every flight. Writes the committed
pipeline/calib/vtol_phases.json (the fit's only input, so it reruns once the raw logs are
gone) and out/vtol_phases.csv. Skipped, keeping the committed table, when the raw runs are not
on disk.

Phases, from the drone's spec transitions and the logger's VTOL state events:
  mc_takeoff        D01 to D02: multicopter climb to cruise height
  transition        D02 to fixed-wing flight (TRANSITION_FW to FW)
  cruise            fixed wing (FW to TRANSITION_MC)
  back_transition   TRANSITION_MC to the holding fix (D03): the back-transition and the
                    multicopter stretch to the fix
  approach_landing  D04 to D05: descent and precision landing on IN
  hop               IN to OUT, empty (the pod was released on IN)
Distances are on the ground-truth track: along-track is the projection on the line from the
start to the holding fix. Energy is the proxy of spec A57/A92: the four lift rotors by
induced power, the pusher by propeller momentum theory at the speed along the body x axis.

Every flight must carry the same clean px4/ revision (run.json `script_git`); a batch that
mixes versions or was flown from uncommitted code is refused.

Cross-check of the cruise proxy against the simulator: in steady fixed-wing flight the pusher
thrust should balance the wing drag of Gazebo's lift-drag model, D = 1/2 rho V^2 A cd with
cd = cda (alpha + a0) per surface, alpha from the ground-truth velocity in the body frame (no
wind). The reference power is D V / eta with eta = V / (V + v_i) the ideal propeller
efficiency at that thrust; the table reports proxy cruise power over that reference."""

import csv
import json

import numpy as np

from common import CALIB, OUT, RUNS, flightlog, run_meta

BATCH = RUNS / "vtol_cal"
TABLE = CALIB / "vtol_phases.json"
PHASES = [("mc_takeoff", "takeoff_start", "top_of_climb"), ("transition", "top_of_climb", "fw"),
          ("cruise", "fw", "to_mc"), ("back_transition", "to_mc", "fix"),
          ("approach_landing", "grant", "landed_in"), ("hop", "hop_start", "landed_out")]
CLASS = {"mc_takeoff": "multicopter", "approach_landing": "multicopter", "hop": "multicopter",
         "transition": "transition", "back_transition": "transition", "cruise": "cruise"}
# Gazebo lift-drag of PX4's standard VTOL wing (two surfaces), scaled by the vehicle's factor.
WING_AREA_STOCK_M2 = 0.5
WING_SURFACES = 2
WING_CDA = 0.6417112299
WING_A0_RAD = 0.05984281113
GZ_AIR_DENSITY = 1.2041
# The middle half of the cruise, away from the transitions, for the steady-state cross-check.
STEADY_FRACTION = 0.5


def vtol_times(events: list[dict]) -> dict[str, float]:
    t = flightlog.phase_times(events)["_times"]
    v = [(e["t_us"] * 1e-6, e["state"]) for e in events if e["type"] == "vtol"]
    def first(state, after):
        return next(ts for ts, s in v if s == state and ts >= after)
    t["fw"] = first("FW", t["top_of_climb"])
    t["to_mc"] = first("TRANSITION_MC", t["fw"])
    t["mc"] = first("MC", t["to_mc"])
    return t


def body_velocity(samples: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Ground-truth velocity (finite differences of the truth track) in body FRD, and the
    sample times. Truth is world ENU; PX4's attitude rotates FRD into NED."""
    t = samples["t_us"] * 1e-6
    enu = samples["truth_enu_m"].astype(np.float64)
    v_enu = np.gradient(enu, t, axis=0)
    v_ned = np.column_stack([v_enu[:, 1], v_enu[:, 0], -v_enu[:, 2]])
    w, x, y, z = (samples["q_ned_frd"][:, i].astype(np.float64) for i in range(4))
    r = np.empty((len(t), 3, 3))
    r[:, 0] = np.column_stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)])
    r[:, 1] = np.column_stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)])
    r[:, 2] = np.column_stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])
    return np.einsum("nji,nj->ni", r, v_ned), t


def proxy_power_w(samples: np.ndarray, proxy: dict, rho: float, axial: np.ndarray) -> np.ndarray:
    k, r = proxy["hover_motor_constant"], proxy["hover_rotor_radius_m"]
    omega = samples["motor_radps"].astype(np.float64)
    lift = ((k * omega**2) ** 1.5 / np.sqrt(2 * rho * np.pi * r * r)).sum(axis=1)
    kp, rp = proxy["pusher_motor_constant"], proxy["pusher_radius_m"]
    thrust = kp * samples["pusher_radps"].astype(np.float64) ** 2
    v = np.maximum(axial, 0.0)
    vi = -v / 2 + np.sqrt(v * v / 4 + thrust / (2 * rho * np.pi * rp * rp))
    return lift + thrust * (v + vi)


def cross_check(samples, meta, proxy, rho, power, v_body, t, t0, t1) -> dict:
    span = t1 - t0
    sel = (t > t0 + span * (1 - STEADY_FRACTION) / 2) & (t < t1 - span * (1 - STEADY_FRACTION) / 2)
    u, w = v_body[sel, 0], v_body[sel, 2]
    speed2 = u * u + w * w
    alpha = np.arctan2(w, u) + WING_A0_RAD
    area = WING_AREA_STOCK_M2 * meta["thrust_scale"]
    drag = WING_SURFACES * 0.5 * GZ_AIR_DENSITY * speed2 * area * WING_CDA * alpha
    kp, rp = proxy["pusher_motor_constant"], proxy["pusher_radius_m"]
    vi = -u / 2 + np.sqrt(u * u / 4 + drag / (2 * rho * np.pi * rp * rp))
    reference = drag * (u + vi)
    thrust = kp * samples["pusher_radps"][sel].astype(np.float64) ** 2
    return {"steady_cruise_samples": int(sel.sum()),
            "airspeed_mps": round(float(np.mean(np.sqrt(speed2))), 3),
            "wing_drag_n": round(float(np.mean(drag)), 2),
            "pusher_thrust_n": round(float(np.mean(thrust)), 2),
            "reference_power_w": round(float(np.mean(reference)), 1),
            "proxy_power_w": round(float(np.mean(power[sel])), 1),
            "proxy_over_reference": round(float(np.mean(power[sel]) / np.mean(reference)), 4)}


def run_rows(run) -> tuple[list[dict], dict]:
    import yaml
    meta = run_meta(run)
    rho = yaml.safe_load((run / "ros_params.yaml").read_text())["flight_logger"]["ros__parameters"]["air_density_kgpm3"]
    proxy = meta["proxy"]
    events = flightlog.read_events(run / "events.jsonl")
    samples = flightlog.read_samples(run / "flight_samples.bin")
    times = vtol_times(events)
    v_body, t = body_velocity(samples)
    power = proxy_power_w(samples, proxy, rho, v_body[:, 0])
    truth = samples["truth_enu_m"].astype(np.float64)
    at = lambda ts: truth[np.argmin(np.abs(t - ts)), :2]
    start, fix = at(times["takeoff_start"]), at(times["fix"])
    track = (fix - start) / np.linalg.norm(fix - start)
    empty_kg = run_meta(run)["latch_kg"] + 9.0
    lane = json.loads(((run / "lane.json") if (run / "lane.json").exists()
                       else (CALIB.parents[1] / "px4" / "config" / "lane.json")).read_text())
    rows = []
    for name, a, b in PHASES:
        t0, t1 = times[a], times[b]
        sel = (t >= t0) & (t <= t1)
        xy = truth[sel, :2]
        horizontal = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
        along = float((xy[-1] - xy[0]) @ track)
        energy = float(np.trapezoid(power[sel], t[sel]))
        mass = empty_kg + (meta["pod_kg"] if name != "hop" else 0.0)
        rows.append({"run": run.name, "seed": meta["seed"], "payload_kg": meta["payload_kg"],
                     "speed_factor": meta["speed"], "phase": name, "class": CLASS[name],
                     "mass_kg": round(mass, 4), "duration_s": round(t1 - t0, 3),
                     "horizontal_m": round(horizontal, 2), "along_track_m": round(along, 2),
                     "mean_speed_mps": round(horizontal / (t1 - t0), 3),
                     "energy_j": round(energy, 1), "mean_power_w": round(energy / (t1 - t0), 1)})
    lane_s = times["fix"] - times["top_of_climb"]
    outcome = {"run": run.name, "status": (run / "run_status").read_text().strip(),
               "script_git": meta.get("script_git"),
               "wall_s": int((run / "wall_s").read_text()),
               "peak_rss_mb": json.loads((run / "rss.json").read_text())["peak_total_mb"],
               "validation": flightlog.check(run) or "pass",
               "payload_kg": meta["payload_kg"], "seed": meta["seed"], "speed_factor": meta["speed"],
               "lane_id": lane["lane_id"], "lane_length_m": lane["length_m"],
               "lane_time_s": round(lane_s, 3),
               "lane_energy_j": round(float(np.trapezoid(power[(t >= times["top_of_climb"]) & (t <= times["fix"])],
                                                          t[(t >= times["top_of_climb"]) & (t <= times["fix"])])), 1),
               "plateau_speed_mps": plateau_speed(samples, t, times),
               "landings_cm": [round(l["error_cm"], 2) for l in
                               json.loads((run / "visits.json").read_text())["landings"]],
               "cross_check": cross_check(samples, meta, proxy, rho, power, v_body, t,
                                          times["fw"], times["to_mc"])}
    return rows, outcome


def plateau_speed(samples, t, times) -> float:
    """Median ground speed over the middle half of the fixed-wing cruise."""
    quarter = (times["to_mc"] - times["fw"]) / 4
    sel = (t > times["fw"] + quarter) & (t < times["to_mc"] - quarter)
    return round(float(np.median(np.linalg.norm(samples["vel_ned_mps"][sel, :2].astype(float), axis=1))), 3)


def main() -> None:
    if not BATCH.exists():
        print(f"{BATCH} not on disk: keeping the committed {TABLE.name}")
        return
    runs = sorted(p for p in BATCH.iterdir() if (p / "run.json").exists())
    rows, outcomes = [], []
    for run in runs:
        if not (run / "run_status").exists() or (run / "run_status").read_text().strip() != "COMPLETE":
            outcomes.append({"run": run.name, "status": (run / "run_status").read_text().strip()
                             if (run / "run_status").exists() else "NO STATUS"})
            continue
        r, o = run_rows(run)
        rows += r
        outcomes.append(o)
    # Every flight of the batch must come from one committed version of px4/.
    versions = {json.loads((run / "run.json").read_text()).get("script_git") for run in runs}
    if len(versions) != 1 or None in versions or any(v.endswith("-dirty") for v in versions):
        raise RuntimeError(f"{BATCH.name}: flights from mixed or uncommitted px4/ versions: {sorted(map(str, versions))}")
    TABLE.write_text(json.dumps({"script_git": versions.pop(), "phases": rows, "runs": outcomes},
                                indent=1) + "\n")
    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / "vtol_phases.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    for o in outcomes:
        cc = o.get("cross_check", {})
        print(f"{o['run']:12s} {o['status']:8s} lane {o.get('lane_time_s', 0):7.1f} s "
              f"{o.get('lane_energy_j', 0) / 1e3:6.1f} kJ plateau {o.get('plateau_speed_mps', 0):5.2f} m/s "
              f"proxy/ref {cc.get('proxy_over_reference')}")


if __name__ == "__main__":
    main()
