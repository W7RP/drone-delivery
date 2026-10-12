"""Phase 7 step 9: the SITL crosswind check (px4/runs/vtol_wind, spec A94). The fit lane at 5 kg
and 2x, wind from 22 deg (square across the lane from the left) at 0, 5 and 8 m/s with gusts of
50%, seeds 1 and 2: four landings per level. Informs the limitations only; nothing is fitted.

Per run: the outcome, landing errors on IN and OUT, and evidence that the vehicle feels the
wind: the crab angle (heading minus ground track) and ground speed over the middle half of the
fixed-wing cruise, the lane time, and the median tilt during the approach to IN. Writes
out/vtol_wind.json; skipped, keeping the committed file, when the raw runs are gone."""

import json

import numpy as np

from common import OUT, RUNS, flightlog
from vtol_phases import vtol_times

BATCH = RUNS / "vtol_wind"
# The middle half of the cruise, away from the transitions.
STEADY_FRACTION = 0.5
# The latch captures the pod within this distance of the OUT spot (spec A71, placeholder).
CAPTURE_RADIUS_CM = 15.0


def yaw_rad(q: np.ndarray) -> np.ndarray:
    w, x, y, z = (q[:, i].astype(np.float64) for i in range(4))
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def tilt_deg(q: np.ndarray) -> np.ndarray:
    x, y = q[:, 1].astype(np.float64), q[:, 2].astype(np.float64)
    return np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0)))


def wrap_deg(a: np.ndarray) -> np.ndarray:
    return (np.degrees(a) + 180.0) % 360.0 - 180.0


def run_row(run) -> dict:
    meta = json.loads((run / "run.json").read_text())
    status = (run / "run_status").read_text().strip() if (run / "run_status").exists() else "NO STATUS"
    landings = json.loads((run / "visits.json").read_text())["landings"] if (run / "visits.json").exists() else []
    row = {"run": run.name, "wind_mps": meta["wind_mps"], "wind_from_deg": meta["wind_from_deg"],
           "gust_frac": meta["gust_frac"], "seed": meta["seed"], "script_git": meta["script_git"],
           "status": status, "validation": flightlog.check(run) or "pass",
           "landings_cm": {l["spot"]: round(l["error_cm"], 1) for l in landings},
           "latched": status == "COMPLETE"}
    events = flightlog.read_events(run / "events.jsonl")
    samples = flightlog.read_samples(run / "flight_samples.bin")
    t = samples["t_us"] * 1e-6
    times = vtol_times(events)
    span = times["to_mc"] - times["fw"]
    cruise = (t > times["fw"] + span * (1 - STEADY_FRACTION) / 2) & (t < times["to_mc"] - span * (1 - STEADY_FRACTION) / 2)
    vel = samples["vel_ned_mps"].astype(np.float64)
    track = np.arctan2(vel[:, 1], vel[:, 0])
    crab = wrap_deg(yaw_rad(samples["q_ned_frd"]) - track)
    approach = (t >= times["grant"]) & (t <= times["landed_in"])
    row.update({"crab_deg": round(float(np.median(crab[cruise])), 2),
                "cruise_ground_speed_mps": round(float(np.median(np.hypot(vel[cruise, 0], vel[cruise, 1]))), 2),
                "lane_time_s": round(times["fix"] - times["top_of_climb"], 1),
                "approach_tilt_deg": round(float(np.median(tilt_deg(samples["q_ned_frd"])[approach])), 2),
                "approach_s": round(times["landed_in"] - times["grant"], 1)})
    return row


def main() -> None:
    runs = sorted(p for p in BATCH.iterdir() if (p / "run.json").exists()) if BATCH.exists() else []
    if not runs:
        print(f"{BATCH} not on disk: keeping the committed out/vtol_wind.json")
        return
    versions = {json.loads((r / "run.json").read_text())["script_git"] for r in runs}
    if len(versions) != 1 or any(v.endswith("-dirty") for v in versions):
        raise RuntimeError(f"{BATCH.name}: flights from mixed or uncommitted px4/ versions: {sorted(versions)}")
    rows = [run_row(r) for r in runs]
    levels = []
    for mps in sorted({r["wind_mps"] for r in rows}):
        rs = [r for r in rows if r["wind_mps"] == mps]
        errs = {s: [r["landings_cm"][s] for r in rs if s in r["landings_cm"]] for s in ("IN", "OUT")}
        levels.append({"wind_mps": mps, "runs": len(rs), "completed": sum(r["latched"] for r in rs),
                       "landings": sum(len(v) for v in errs.values()),
                       "in_error_cm_max": max(errs["IN"]), "out_error_cm_max": max(errs["OUT"]),
                       "out_beyond_capture": sum(e > CAPTURE_RADIUS_CM for e in errs["OUT"]),
                       "crab_deg_mean": round(float(np.mean([r["crab_deg"] for r in rs])), 2),
                       "cruise_ground_speed_mps_mean": round(float(np.mean([r["cruise_ground_speed_mps"] for r in rs])), 2),
                       "approach_tilt_deg_mean": round(float(np.mean([r["approach_tilt_deg"] for r in rs])), 2)})
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "vtol_wind.json").write_text(json.dumps({"script_git": versions.pop(), "capture_radius_cm": CAPTURE_RADIUS_CM,
                                                    "levels": levels, "runs": rows}, indent=1) + "\n")
    for r in rows:
        print(f"{r['run']:14s} {r['status'][:40]:40s} IN/OUT {r['landings_cm']} crab {r['crab_deg']:+6.1f} deg "
              f"ground {r['cruise_ground_speed_mps']:5.2f} m/s tilt {r['approach_tilt_deg']:4.1f} deg "
              f"lane {r['lane_time_s']} s")


if __name__ == "__main__":
    main()
