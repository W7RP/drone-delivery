"""Phase 5 step 1: audit the SITL vehicle and the energy proxy before fitting anything.
Writes pipeline/calib/out/audit.json and prints it. Reads the generated vehicle model of each
run, its samples and the PX4 build; changes nothing.

Hover throttle: PX4's gz bridge commands each motor speed as throttle x 1000 rad/s (the motor
model's maxRotVelocity), so throttle = commanded speed / 1000. It is taken from hover samples:
armed, in offboard, above 1 m, speed under 0.3 m/s, and no pod change in between."""

import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from common import GRAVITY, OUT, REPO, flightlog, fit_runs, run_meta

MAX_ROT_VELOCITY = 1000.0
OFFBOARD = 14
ARMED = 2
HOVER_SPEED_MPS = 0.3
HOVER_MIN_ALT_M = 1.0
PX4_DIR = Path.home() / "PX4-Autopilot"
QUAD_PATCHES = Path.home() / "projects" / "quad-autonomy-sim" / "firmware" / "px4_patches"


def model_masses(run: Path) -> dict:
    """Link masses and motor constant of the generated drone, and the pod mass."""
    drone = ET.parse(run / "models" / "dd_quad" / "model.sdf").getroot().find("model")
    links = {l.get("name"): float(l.find("inertial/mass").text) for l in drone.findall("link")}
    k = {float(p.find("motorConstant").text) for p in drone.findall("plugin")
         if p.find("motorConstant") is not None}
    pod = ET.parse(run / "models" / "dd_pod_1" / "model.sdf").getroot().find("model")
    return {"links_kg": links, "motor_constant": k.pop() if len(k) == 1 else sorted(k),
            "pod_kg": float(pod.find("link/inertial/mass").text)}


def hover(samples: np.ndarray) -> dict:
    speed = np.linalg.norm(samples["vel_ned_mps"], axis=1)
    alt = -samples["pos_ned_m"][:, 2]
    m = ((samples["nav_state"] == OFFBOARD) & (samples["arming_state"] == ARMED)
         & (speed < HOVER_SPEED_MPS) & (alt > HOVER_MIN_ALT_M))
    out = {}
    for latched in (0, 1):
        sel = m & (samples["pods_latched"] == latched)
        if sel.sum() == 0:
            continue
        omega = samples["motor_radps"][sel].astype(float)
        out["with_pod" if latched else "empty"] = {
            "samples": int(sel.sum()),
            "throttle_mean": round(float((omega / MAX_ROT_VELOCITY).mean()), 4),
            "omega_rms_radps": round(float(np.sqrt((omega**2).mean())), 1)}
    return out


def cruise(samples: np.ndarray, events: list[dict], k: float, total_kg: float) -> dict:
    """In the lane (D02 to D03, speed above 14 m/s): mean thrust over weight, and how unevenly
    the four rotors share it. Induced power goes with each rotor's thrust^1.5, so uneven rotors
    cost more than four equal ones (ratio > 1)."""
    t = samples["t_us"] * 1e-6
    d = {e["row"]: e["t_us"] * 1e-6 for e in events if e["type"] == "transition" and e["machine"] == "DRONE" and e["row"] in ("D02", "D03")}
    speed = np.linalg.norm(samples["vel_ned_mps"][:, :2], axis=1)
    sel = (t > d["D02"]) & (t < d["D03"]) & (speed > 14.0)
    thrust = k * samples["motor_radps"][sel].astype(float) ** 2
    total = thrust.sum(axis=1)
    uneven = (thrust ** 1.5).sum(axis=1) / (4 * (total / 4) ** 1.5)
    return {"samples": int(sel.sum()), "speed_mps": round(float(speed[sel].mean()), 3),
            "thrust_over_weight": round(float((total / (total_kg * GRAVITY)).mean()), 4),
            "rotor_unevenness": round(float(uneven.mean()), 4)}


def git(*args) -> str:
    return subprocess.run(["git", "-C", str(PX4_DIR), *args], capture_output=True, text=True,
                          check=True).stdout.strip()


def main() -> None:
    import yaml
    runs = [r for r in fit_runs() if r.name.startswith("payload_")]
    flight = yaml.safe_load((REPO / "px4" / "config" / "flight.yaml").read_text())
    logger = yaml.safe_load((runs[0] / "ros_params.yaml").read_text())["flight_logger"]["ros__parameters"]
    scenario = json.loads((REPO / "px4" / "config" / "lane.json").read_text())["drone"]
    rows = []
    for r in sorted(runs, key=lambda r: run_meta(r)["payload_kg"]):
        meta, masses = run_meta(r), model_masses(r)
        samples = flightlog.read_samples(r / "flight_samples.bin")
        airframe = sum(v for k_, v in masses["links_kg"].items() if k_ != "latch_link")
        latch = masses["links_kg"]["latch_link"]
        total = airframe + latch + masses["pod_kg"]
        k = masses["motor_constant"]
        max_thrust = 4 * k * MAX_ROT_VELOCITY**2
        h = hover(samples)
        c = cruise(samples, flightlog.read_events(r / "events.jsonl"), masses["motor_constant"], total)
        loaded = h.get("with_pod", {})
        rows.append({
            "run": r.name, "payload_kg": meta["payload_kg"],
            "gazebo_airframe_kg": round(airframe, 4), "gazebo_latch_kg": latch,
            "gazebo_pod_kg": masses["pod_kg"], "gazebo_total_kg": round(total, 4),
            "motor_constant": k, "max_thrust_n": round(max_thrust, 1),
            "thrust_margin": round(max_thrust / (total * GRAVITY), 3),
            "hover": h,
            "cruise": c,
            # Thrust the measured hover speeds give, against the weight they must carry.
            "hover_thrust_over_weight": round(4 * k * loaded["omega_rms_radps"]**2 / (total * GRAVITY), 3)
            if loaded else None,
        })
    proxy = {"rotor_radius_m": logger["rotor_radius_m"],
             "rotor_disc_area_m2": round(np.pi * logger["rotor_radius_m"]**2, 5),
             "rotors": 4, "air_density_kgpm3": logger["air_density_kgpm3"],
             "figure_of_merit": 1.0, "motor_and_esc_efficiency": 1.0,
             "form": "P = sum over rotors of (k_f omega^2)^1.5 / sqrt(2 rho A), integrated"}
    # Effective empty mass m0 that pure mass^1.5 needs to match the 0 -> 10 kg cruise power ratio.
    p0 = [r for r in rows if r["payload_kg"] == 0][0]
    p10 = [r for r in rows if r["payload_kg"] == 10][0]
    summary = json.loads((REPO / "px4" / "runs" / "accept" / "summary.json").read_text())
    power = {r["payload_kg"]: r["energy_proxy_j"]["cruise_j"] / r["phases_s"]["cruise_s"]
             for r in summary["runs"] if r["run"].startswith("payload_")}
    ratio = power[10.0] / power[0.0]
    m0 = (p10["gazebo_total_kg"] - p0["gazebo_total_kg"]) / (ratio ** (2 / 3) - 1)
    mass_check = {"cruise_power_ratio_10_over_0": round(ratio, 3),
                  "pure_mass_1p5_ratio": round((p10["gazebo_total_kg"] / p0["gazebo_total_kg"]) ** 1.5, 3),
                  "effective_empty_mass_kg": round(m0, 2),
                  "actual_empty_mass_kg": p0["gazebo_total_kg"]}
    patches = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()[:16]
               for p in sorted(QUAD_PATCHES.glob("*.patch"))}
    audit = {
        "scenario_drone": {"empty_mass_kg": scenario["empty_mass_kg"],
                           "latch_mass_kg": scenario["latch_mass_kg"], "mtow_kg": scenario["mtow_kg"]},
        "thrust_to_weight_at_mtow_placeholder": flight["vehicle"]["thrust_to_weight_at_mtow"],
        "per_payload": rows,
        "proxy": proxy,
        "mass_check": mass_check,
        "px4": {"describe": git("describe", "--tags", "--always"), "commit": git("rev-parse", "HEAD"),
                "modified_files": git("diff", "--name-only").splitlines(),
                "patches_from": "quad-autonomy-sim firmware/px4_patches", "patch_sha256_16": patches},
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "audit.json").write_text(json.dumps(audit, indent=1) + "\n")
    print(json.dumps(audit, indent=1))


if __name__ == "__main__":
    main()
