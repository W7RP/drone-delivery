"""Phase 5 step 2: one row per run and flight phase (takeoff, cruise, approach + landing, hop)
from every fit log: duration, horizontal and vertical distance flown (ground truth), mean speed,
mean proxy power and proxy energy, and the mass carried. Writes out/phases.csv and
out/phases.json.

Phases come from the drone's spec transitions (px4/tools/flightlog.py phase_times). Along-track
distance is the horizontal path length of the ground-truth track; for the cruise it is set
against the lane length of the catalog (lanes.csr.bin) and against the straight line the drone
actually had to cover (its start on A's OUT to the fix over B's centre). The hop is flown empty:
the pod was released on IN."""

import csv
import json
from pathlib import Path

import numpy as np

from common import OUT, REPO, flightlog, fit_runs, run_meta

PHASES = [("takeoff", "takeoff_start", "top_of_climb"), ("cruise", "top_of_climb", "fix"),
          ("approach_landing", "grant", "landed_in"), ("hop", "hop_start", "landed_out")]


def proxy_power_w(samples: np.ndarray, params: dict) -> np.ndarray:
    k, r, rho = params["motor_constant"], params["rotor_radius_m"], params["air_density_kgpm3"]
    omega = samples["motor_radps"].astype(np.float64)
    return ((k * omega**2) ** 1.5 / np.sqrt(2 * rho * np.pi * r * r)).sum(axis=1)


def phase_rows(run: Path) -> list[dict]:
    import yaml
    meta = run_meta(run)
    params = yaml.safe_load((run / "ros_params.yaml").read_text())["flight_logger"]["ros__parameters"]
    lane = flightlog.read_lanes(run / "flight_log.bin")[0]
    events = flightlog.read_events(run / "events.jsonl")
    samples = flightlog.read_samples(run / "flight_samples.bin")
    times = flightlog.phase_times(events)["_times"]
    t = samples["t_us"] * 1e-6
    power = proxy_power_w(samples, params)
    truth = samples["truth_enu_m"].astype(np.float64)
    empty_kg = 9.0 + meta["latch_kg"]
    rows = []
    for name, a, b in PHASES:
        t0, t1 = times[a], times[b]
        sel = (t >= t0) & (t <= t1)
        xy = truth[sel, :2]
        horiz = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
        vert = float(np.abs(np.diff(truth[sel, 2])).sum())
        energy = float(np.trapezoid(power[sel], t[sel]))
        straight = float(np.linalg.norm(xy[-1] - xy[0]))
        mass = empty_kg + (meta["pod_kg"] if name != "hop" else 0.0)
        rows.append({
            "run": run.name, "seed": meta["seed"], "payload_kg": meta["payload_kg"],
            "speed_factor": meta["speed"], "phase": name, "mass_kg": round(mass, 4),
            "duration_s": round(t1 - t0, 3), "horizontal_m": round(horiz, 2),
            "straight_m": round(straight, 2), "vertical_m": round(vert, 2),
            "mean_speed_mps": round(horiz / (t1 - t0), 3) if t1 > t0 else None,
            "energy_j": round(energy, 1), "mean_power_w": round(energy / (t1 - t0), 1),
            "lane_id": lane.lane_id,
            "lane_length_m": round(lane_length_m(run), 2) if name == "cruise" else None,
        })
    return rows


def lane_record(run: Path) -> dict:
    """The lane the run flew: its own lane.json (runs flown on any lane), else the default lane
    of px4/config (the Phase 4 runs, all on lane 3)."""
    own = run / "lane.json"
    path = own if own.exists() else REPO / "px4" / "config" / "lane.json"
    return json.loads(path.read_text())


def lane_length_m(run: Path) -> float:
    return lane_record(run)["length_m"]


def main() -> None:
    rows = [row for run in fit_runs() for row in phase_rows(run)]
    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / "phases.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    (OUT / "phases.json").write_text(json.dumps(rows, indent=1) + "\n")
    cruise = [r for r in rows if r["phase"] == "cruise"]
    print(f"{len(rows)} phase rows from {len(cruise)} runs")
    for r in cruise:
        print(f"{r['run']:13s} {r['payload_kg']:4.0f} kg cruise {r['duration_s']:6.1f} s "
              f"{r['horizontal_m']:7.1f} m flown, straight {r['straight_m']:7.1f}, lane "
              f"{r['lane_length_m']:7.1f}, {r['mean_speed_mps']:5.2f} m/s, {r['mean_power_w']:6.0f} W")


if __name__ == "__main__":
    main()
