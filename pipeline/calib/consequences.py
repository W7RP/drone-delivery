"""Phase 5 step 7a and 7d: what the fitted profile means for range and battery, through the
spec's energy guard (dispatch_ok, spec 6 and A23) as the pipeline's energy mirror computes it
(ddpipe/energy.py; the sim re-checks the same rule at every scenario load as R12 and R16).

7a  Payload-range: the longest one-way lane dispatch_ok accepts from drone.charge_target_soc
    with the scenario's battery, at 2, 5 and 10 kg, for both profiles. The diversion leg is
    taken as zero (an alternate pad next to the destination), so this is an upper bound; any
    real diversion distance comes off it.
7d  Battery: the capacity the longest legal lane of the catalog (Phase 3's network, lane 4,
    hub to Edgar Thomson, 12 896 m) needs at 5 kg under the fitted profile: dispatch_ok's need
    on that network (its diversion to the destination's nearest pad included) divided by the
    usable fraction (charge_target_soc - reserve_soc).

Writes out/consequences.json."""

import json
import math
import sys
import tomllib

from common import OUT, REPO

sys.path.insert(0, str(REPO / "pipeline"))
from ddpipe import energy, variants  # noqa: E402
from ddpipe.params import load, q  # noqa: E402

PAYLOADS_KG = (2.0, 5.0, 10.0)
J_PER_WH = 3600.0
PLACEHOLDER_NETWORK = REPO / "configs" / "parts_logistics_pittsburgh_placeholder" / "derived"


def legal_lanes() -> tuple[int, list[tuple[int, int, float]], list[str]]:
    """Phase 3's legal lanes (placeholder network): (pad count, undirected lanes, pad names)."""
    report = json.loads((PLACEHOLDER_NETWORK / "report.json").read_text())
    names = [p["name"] for p in report["pads"]]
    index = {n: i for i, n in enumerate(names)}
    lanes = [(index[l["a"]], index[l["b"]], l["length_m"]) for l in report["lanes"]]
    return len(names), lanes, names


def main() -> None:
    with open(REPO / "pipeline" / "scenario.toml.in", "rb") as f:
        template = tomllib.load(f)
    params = load("params.yaml")
    by_name = {v.name: v for v in variants.load(params)}
    models = {
        "placeholder": energy.FlightModel.from_scenario(
            variants.config(template, by_name["parts_logistics_pittsburgh_placeholder"])),
        "px4_sitl_proxy": energy.FlightModel.from_scenario(
            variants.config(template, by_name["parts_logistics_pittsburgh"])),
    }
    d = template["drone"]
    mtow_margin = q(d["mtow_kg"], "kg") - q(d["empty_mass_kg"], "kg") - q(d["latch_mass_kg"], "kg")
    payload_range = []
    for p in PAYLOADS_KG:
        row = {"payload_kg": p, "within_mtow": p <= mtow_margin}
        for name, m in models.items():
            row[f"{name}_max_lane_m"] = round(max(0.0, m.max_lane_m(p)), 1)
        payload_range.append(row)

    n, lanes, names = legal_lanes()
    a, b, length = max(lanes, key=lambda l: l[2])
    route = energy.routes(n, lanes)
    fitted = models["px4_sitl_proxy"]
    need_j = energy.dispatch_need_j(fitted, route, a, b, 5.0, lambda _: True)
    usable_fraction = fitted.charge_target_soc - fitted.reserve_soc
    capacity_j = need_j / usable_fraction
    battery = {
        "lane": {"from": names[a], "to": names[b], "length_m": round(length, 1)},
        "payload_kg": 5.0,
        "need_above_reserve_j": round(need_j),
        "usable_fraction": usable_fraction,
        "required_capacity_j": round(capacity_j),
        "required_capacity_wh": round(capacity_j / J_PER_WH, 1),
        "configured_capacity_j": fitted.capacity_j,
        "configured_capacity_wh": round(fitted.capacity_j / J_PER_WH, 1),
        # The variant's value: rounded up to a whole 10 Wh.
        "variant_capacity_wh": int(math.ceil(capacity_j / J_PER_WH / 10) * 10),
    }
    out = {"payload_range": payload_range, "mtow_payload_margin_kg": mtow_margin,
           "battery_for_longest_legal_lane": battery}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "consequences.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
