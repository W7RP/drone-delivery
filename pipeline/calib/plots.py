"""Phase 5 plots for docs/calibration.md (docs/media/):
  calibration_pred_vs_meas.png  predicted against measured visit time and energy:
                                leave-one-payload-out predictions of the fit runs, and the
                                held-out flights (fitted profile, not refitted)
  calibration_residuals.png     per-phase energy residual (measured - predicted) / measured of
                                the fitted profile, fit runs by payload and held-out flights"""

import json
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from common import CALIB, OUT, PHASE_KEYS, REPO, scenario_with, sim_flight_model  # noqa: E402

MEDIA = REPO / "docs" / "media"
DPI = 130


def main() -> None:
    fit = json.loads((OUT / "fit.json").read_text())
    held = json.loads((OUT / "heldout.json").read_text())["flights"]
    rows = json.loads((OUT / "phases.json").read_text())
    params = json.loads((CALIB / "fitted_params.json").read_text())["params"]
    cv = fit["leave_one_payload_out"]

    fig, axes = plt.subplots(1, 2, figsize=(10, 4.6))
    for ax, key, unit, scale in ((axes[0], "time", "s", 1.0), (axes[1], "energy", "kJ", 1e-3)):
        unit_key = "time_s" if key == "time" else "energy_j"
        pts = [(c[f"meas_{unit_key}"] * scale, c[f"pred_{unit_key}"] * scale) for c in cv]
        hp = [(f[f"meas_{unit_key}"] * scale, f[f"pred_{unit_key}"] * scale) for f in held]
        lo = min(min(p) for p in pts + hp) * 0.9
        hi = max(max(p) for p in pts + hp) * 1.05
        ax.plot([lo, hi], [lo, hi], color="0.6", lw=1, label="predicted = measured")
        ax.scatter(*zip(*pts), s=28, label="fit runs, lane 3 (leave one payload out)")
        ax.scatter(*zip(*hp), s=40, marker="D", label="held-out flights, lanes 14 and 4")
        ax.set_xlabel(f"measured visit {key} ({unit})")
        ax.set_ylabel(f"predicted visit {key} ({unit})")
        ax.set_xlim(lo, hi)
        ax.set_ylim(lo, hi)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=7, loc="upper left")
    fig.suptitle("Fitted flight profile against PX4 SITL (energy is the proxy of spec A57)")
    fig.tight_layout()
    MEDIA.mkdir(parents=True, exist_ok=True)
    fig.savefig(MEDIA / "calibration_pred_vs_meas.png", dpi=DPI, metadata={"Software": None})
    plt.close(fig)

    config = scenario_with(params, "fitted")
    lane_m = next(r["lane_length_m"] for r in rows if r["phase"] == "cruise")
    pred = {p: sim_flight_model(lane_m, p, config) for p in sorted({r["payload_kg"] for r in rows})}
    res = defaultdict(list)
    for r in rows:
        e = pred[r["payload_kg"]][PHASE_KEYS[r["phase"]]]["energy_j"]
        res[(r["phase"], f"fit {r['payload_kg']:g} kg")].append((r["energy_j"] - e) / r["energy_j"])
    for f in held:
        for phase, ph in f["phases"].items():
            res[(phase, f"held-out {f['lane_length_m']:.0f} m")].append(
                (ph["meas_energy_j"] - ph["pred_energy_j"]) / ph["meas_energy_j"])
    phases = list(PHASE_KEYS)
    series = sorted({k[1] for k in res}, key=lambda s: (s.startswith("held"), float(s.split()[1])))
    fig, ax = plt.subplots(figsize=(10, 4.6))
    width = 0.8 / len(series)
    for j, name in enumerate(series):
        xs, ys = [], []
        for i, phase in enumerate(phases):
            for v in res.get((phase, name), []):
                xs.append(i - 0.4 + width * (j + 0.5))
                ys.append(100 * v)
        ax.scatter(xs, ys, s=18, label=name, marker="D" if name.startswith("held") else "o")
    ax.axhline(0, color="0.5", lw=1)
    for y in (-15, 15):
        ax.axhline(y, color="0.75", lw=1, ls="--")
    ax.set_xticks(range(len(phases)), ["takeoff", "cruise", "approach + landing", "hop"])
    ax.set_ylabel("energy residual, (measured - predicted) / measured (%)")
    ax.set_title("Per-phase energy residual of the fitted profile (dashed: the 15% visit target)")
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=7, ncol=6, loc="upper center", bbox_to_anchor=(0.5, -0.1))
    fig.tight_layout()
    fig.savefig(MEDIA / "calibration_residuals.png", dpi=DPI, metadata={"Software": None})
    plt.close(fig)
    print("wrote docs/media/calibration_pred_vs_meas.png, calibration_residuals.png")


if __name__ == "__main__":
    main()
