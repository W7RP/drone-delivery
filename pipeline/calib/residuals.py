"""Phase 5 step 3: the current placeholder flight model (`sim flight-model` on the Pittsburgh
scenario) against the measured phases, per payload: predicted and measured time and energy per
phase and for the visit, and the residual (measured - predicted) / measured. Writes
out/residuals_placeholder.json."""

import json
import statistics as st
from collections import defaultdict

from common import OUT, PHASE_KEYS, sim_flight_model


def main() -> None:
    rows = json.loads((OUT / "phases.json").read_text())
    groups = defaultdict(list)
    for r in rows:
        groups[(r["payload_kg"], r["phase"])].append(r)
    lane_m = next(r["lane_length_m"] for r in rows if r["phase"] == "cruise")
    out = []
    for payload in sorted({r["payload_kg"] for r in rows}):
        model = sim_flight_model(lane_m, payload)
        total = {"pred_time_s": 0.0, "meas_time_s": 0.0, "pred_energy_j": 0.0, "meas_energy_j": 0.0}
        for phase, key in PHASE_KEYS.items():
            g = groups[(payload, phase)]
            meas_t = st.mean(r["duration_s"] for r in g)
            meas_e = st.mean(r["energy_j"] for r in g)
            pred_t, pred_e = model[key]["time_s"], model[key]["energy_j"]
            out.append({"payload_kg": payload, "phase": phase, "runs": len(g),
                        "pred_time_s": round(pred_t, 2), "meas_time_s": round(meas_t, 2),
                        "time_residual": round((meas_t - pred_t) / meas_t, 4),
                        "pred_energy_j": round(pred_e), "meas_energy_j": round(meas_e),
                        "energy_residual": round((meas_e - pred_e) / meas_e, 4)})
            total["pred_time_s"] += pred_t
            total["meas_time_s"] += meas_t
            total["pred_energy_j"] += pred_e
            total["meas_energy_j"] += meas_e
        out.append({"payload_kg": payload, "phase": "visit", "runs": len(groups[(payload, "cruise")]),
                    **{k: round(v, 2) for k, v in total.items()},
                    "time_residual": round((total["meas_time_s"] - total["pred_time_s"]) / total["meas_time_s"], 4),
                    "energy_residual": round((total["meas_energy_j"] - total["pred_energy_j"]) / total["meas_energy_j"], 4)})
    (OUT / "residuals_placeholder.json").write_text(json.dumps(out, indent=1) + "\n")
    for r in out:
        print(f"{r['payload_kg']:4.0f} kg {r['phase']:17s} time {r['pred_time_s']:7.1f} vs {r['meas_time_s']:7.1f} "
              f"({r['time_residual']:+.1%})  energy {r['pred_energy_j'] / 1000:7.1f} vs {r['meas_energy_j'] / 1000:7.1f} kJ "
              f"({r['energy_residual']:+.1%})")


if __name__ == "__main__":
    main()
