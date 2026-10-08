# /// script
# requires-python = ">=3.10"
# dependencies = ["tomli==2.0.1", "tomli-w==1.0.0"]
# ///
"""Phase 6a step 1: battery-mass coupling (spec A68).

With the mass coupled, a drone's empty mass is airframe + capacity / specific energy. The
placeholders below give exactly the current 9.0 kg at 900 Wh. This script asks the core
(`sim battery`, the `dispatch_ok` guard itself) for:

- range at 5 kg with the fitted 900 Wh scenario (unchanged by construction);
- the battery the longest legal lane (hub to Edgar Thomson, 12.9 km, Phase 3's network) needs at
  5 kg, uncoupled (Phase 5's 1367 Wh) and coupled, iterated to a fixed point because a heavier
  battery needs more energy (or the closest the iteration gets when no fixed point exists);
- the largest battery the 16 kg MTOW allows with a 5 kg pod (spec R6), and the range it gives.

Writes analysis/out/battery.json. Run: uv run analysis/battery.py"""

import json
import math
import tempfile
from pathlib import Path

from variants import J_PER_WH, OUT, build_sim, quantity, sim_json, variant

AIRFRAME_KG = 4.5
SPECIFIC_ENERGY_WH_PER_KG = 200.0
BASE_WH = 900.0
PAYLOAD_KG = 5.0
HUB = "Consumer_Fresh_Produce"
LONGEST = "Mon_Valley_Works_Edgar_Thomson_P"
FULL_NETWORK = "parts_logistics_pittsburgh_battery_1370wh"  # Phase 3's 9 pads and 24 lanes
FITTED_900 = "parts_logistics_pittsburgh"
STEP_WH = 10.0


def coupled_edits(capacity_wh: float) -> dict:
    return {
        "schema_version": "1.6.0",
        "drone.empty_mass_kg": None,
        "drone.airframe_mass_kg": quantity(AIRFRAME_KG, "kg"),
        "drone.battery_specific_energy_wh_per_kg": quantity(SPECIFIC_ENERGY_WH_PER_KG, "Wh/kg"),
        "drone.battery_capacity_j": capacity_wh * J_PER_WH,
    }


def write_coupled(root: Path, source: str, name: str, capacity_wh: float) -> Path:
    return variant(source, root, name, coupled_edits(capacity_wh))


def lane_sizing(config: Path, to: str = LONGEST) -> dict:
    return sim_json(
        "battery", "--config", str(config), "--payload-kg", str(PAYLOAD_KG),
        "--from", HUB, "--to", to,
    )


def main() -> None:
    build_sim()
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        uncoupled = lane_sizing(Path(__file__).resolve().parent.parent / "configs" / FULL_NETWORK / "scenario.toml")
        coupled = lane_sizing(write_coupled(root, FULL_NETWORK, "coupled_900", BASE_WH))
        # Range only; the fitted 900 Wh network has no Edgar Thomson pad, so any of its lanes do.
        fitted = lane_sizing(
            write_coupled(root, FITTED_900, "fitted_coupled_900", BASE_WH), "UPMC_Oakland"
        )
        # The largest battery R6 allows, rounded down to the step so it stays within MTOW.
        limit_wh = math.floor(coupled["mtow_limit_wh"] / STEP_WH) * STEP_WH
        at_limit = lane_sizing(write_coupled(root, FULL_NETWORK, "coupled_limit", limit_wh))
    out = {
        "placeholders": {
            "airframe_mass_kg": AIRFRAME_KG,
            "battery_specific_energy_wh_per_kg": SPECIFIC_ENERGY_WH_PER_KG,
            "empty_mass_kg_at_900_wh": AIRFRAME_KG + BASE_WH / SPECIFIC_ENERGY_WH_PER_KG,
        },
        "payload_kg": PAYLOAD_KG,
        "range_at_900_wh_m": fitted["configured"]["max_lane_m"],
        "longest_legal_lane": uncoupled["lane"],
        "uncoupled_required_wh": uncoupled["required_capacity_without_coupling_wh"],
        "coupled": {
            "fixed_point": coupled["fixed_point"],
            "iterations": coupled["iterations"],
            "sized": coupled["sized"],
            "closest_capacity_wh": coupled["closest"]["capacity_wh"],
            "closest_shortfall_wh": coupled["closest"]["shortfall_wh"],
        },
        "mtow_limit_wh": coupled["mtow_limit_wh"],
        "at_mtow_limit": {
            "capacity_wh": limit_wh,
            "empty_mass_kg": at_limit["configured"]["empty_mass_kg"],
            "range_m": at_limit["configured"]["max_lane_m"],
        },
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "battery.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
