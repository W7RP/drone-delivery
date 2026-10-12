"""Phase 7 plots for docs/vtol.md (docs/media/):
  vtol_pred_vs_meas.png   predicted against measured visit time and energy: the fit runs in
                          sample and leave-one-payload-out, and the held-out flights (fitted
                          profile, not refitted)
  vtol_power_vs_mass.png  mean proxy power per phase class against total mass, with the fitted
                          curves"""

import json

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from common import CALIB, OUT, REPO  # noqa: E402

MEDIA = REPO / "docs" / "media"
DPI = 130
POWER_REF_MASS_KG = 14.6


def main() -> None:
    fit = json.loads((OUT / "vtol_fit.json").read_text())
    held_path = OUT / "vtol_heldout.json"
    held = [r for r in json.loads(held_path.read_text())["runs"] if r.get("status") == "COMPLETE"] \
        if held_path.exists() else []
    params = json.loads((CALIB / "vtol_fitted_params.json").read_text())["params"]
    rows = json.loads((CALIB / "vtol_phases.json").read_text())["phases"]

    fig, axes = plt.subplots(1, 2, figsize=(10, 4.6))
    for ax, key, unit, scale in ((axes[0], "time", "s", 1.0), (axes[1], "energy", "kJ", 1e-3)):
        suffix = "s" if key == "time" else "j"
        sets = [("in sample", fit["in_sample"], "o"), ("leave one payload out", fit["leave_one_payload_out"], "s")]
        for label, pts, marker in sets:
            ax.scatter([p[f"{key}_measured_{suffix}"] * scale for p in pts],
                       [p[f"{key}_predicted_{suffix}"] * scale for p in pts], marker=marker, label=label)
        if held:
            ax.scatter([h[f"visit_{key}_measured_{suffix}"] * scale for h in held],
                       [h[f"visit_{key}_predicted_{suffix}"] * scale for h in held], marker="^",
                       label="held out")
        lo, hi = ax.get_xlim()
        ax.plot([lo, hi], [lo, hi], color="0.5", lw=1)
        ax.set_xlabel(f"measured visit {key} ({unit})")
        ax.set_ylabel(f"predicted visit {key} ({unit})")
        ax.legend(fontsize=8)
    fig.suptitle("VTOL profile against PX4 SITL visits (energy is the proxy of spec A57, A92)")
    fig.tight_layout()
    MEDIA.mkdir(parents=True, exist_ok=True)
    fig.savefig(MEDIA / "vtol_pred_vs_meas.png", dpi=DPI)

    fig, ax = plt.subplots(figsize=(6, 4.4))
    m = np.linspace(9.0, 15.0, 50)
    r = m / POWER_REF_MASS_KG
    curves = {"multicopter": params["hover_power_ref_w"] * r ** 1.5,
              "cruise": params["cruise_power_ref_w"] * r ** params["cruise_mass_exponent"]}
    for cls, color in (("multicopter", "C0"), ("cruise", "C1")):
        pts = [x for x in rows if x["class"] == cls and x["speed_factor"] == 2]
        ax.scatter([x["mass_kg"] for x in pts], [x["mean_power_w"] for x in pts], color=color, s=12,
                   label=f"{cls} phases")
        ax.plot(m, curves[cls], color=color, lw=1)
    ax.set_xlabel("total mass (kg)")
    ax.set_ylabel("mean proxy power (W)")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(MEDIA / "vtol_power_vs_mass.png", dpi=DPI)
    print("wrote docs/media/vtol_pred_vs_meas.png, vtol_power_vs_mass.png")


if __name__ == "__main__":
    main()
