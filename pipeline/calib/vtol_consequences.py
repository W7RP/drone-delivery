"""Phase 7 step 11: what the fitted VTOL profile means for battery, range and the network,
through the core's own energy guard (`sim battery`, spec 6, A68).

Battery: the capacity the longest legal lane of Phase 3's catalog (hub to Edgar Thomson,
12 896 m) needs at 5 kg under the VTOL profile with the battery's mass coupled to it (A68:
4.5 kg airframe, 200 Wh/kg), iterated to its fixed point and rounded up to 10 Wh; checked
against the profile's MTOW. Payload-range: the longest one-way lane dispatch_ok accepts at
2 and 5 kg with that battery (zero diversion leg, an upper bound), next to the coupled 900 Wh
multirotor's. Network: once the pipeline has built parts_logistics_pittsburgh_vtol (params.yaml
`scenarios`, offline from its cache), its lanes and pads against the multirotor's 900 Wh
network, and whether Clairton is served.

Writes out/vtol_consequences.json."""

import json
import re
import subprocess

from common import OUT, REPO, SIM

PLACEHOLDER = REPO / "configs" / "parts_logistics_pittsburgh_placeholder"
PROFILE = REPO / "configs" / "flight_profiles" / "px4_sitl_vtol_proxy.toml"
MULTIROTOR = REPO / "configs" / "parts_logistics_pittsburgh_900wh_layout" / "scenario.toml"
VTOL_NET = REPO / "configs" / "parts_logistics_pittsburgh_vtol" / "derived" / "report.json"
MULTI_NET = REPO / "configs" / "parts_logistics_pittsburgh" / "derived" / "report.json"
HUB = "Consumer_Fresh_Produce"
LONGEST = "Mon_Valley_Works_Edgar_Thomson_P"
CLAIRTON = "Mon_Valley_Works_Clairton_Plant"
AIRFRAME_KG = 4.5
SPECIFIC_WH_PER_KG = 200.0
START_WH = 900.0
J_PER_WH = 3600.0
PAYLOADS_KG = (2.0, 5.0)


def vtol_scenario(capacity_wh: float, tag: str):
    """Phase 3's catalog with the VTOL profile and the coupled battery, for `sim battery`."""
    text = (PLACEHOLDER / "scenario.toml").read_text()
    text = re.sub(r'^schema_version = "[^"]*"$', 'schema_version = "1.9.0"', text, count=1, flags=re.M)
    text = re.sub(r"^empty_mass_kg = \{[^}]*\}$",
                  f'airframe_mass_kg = {{ value = {AIRFRAME_KG}, unit = "kg", assumption = true }}\n'
                  f'battery_specific_energy_wh_per_kg = {{ value = {SPECIFIC_WH_PER_KG}, unit = "Wh/kg", assumption = true }}',
                  text, count=1, flags=re.M)
    text = re.sub(r"^battery_capacity_j = \{[^}]*\}$",
                  f'battery_capacity_j = {{ value = {capacity_wh * J_PER_WH!r}, unit = "J", assumption = true }}',
                  text, count=1, flags=re.M)
    text = text.replace('dir = "derived"', f'dir = "{PLACEHOLDER / "derived"}"')
    text += f'\n[flight_profile]\nfile = "{PROFILE}"\n'
    d = OUT / "models" / tag
    d.mkdir(parents=True, exist_ok=True)
    (d / "scenario.toml").write_text(text)
    return d / "scenario.toml"


def battery(config, payload_kg: float, a: str, b: str) -> dict:
    out = subprocess.run([str(SIM), "battery", "--config", str(config), "--payload-kg", repr(payload_kg),
                          "--from", a, "--to", b], capture_output=True, text=True, check=True).stdout
    return json.loads(out)


def lane_set(report: dict) -> set[tuple[str, str]]:
    return {tuple(sorted((l["a"], l["b"]))) for l in report["lanes"]}


def network() -> dict | None:
    if not VTOL_NET.exists():
        return None
    v, m = json.loads(VTOL_NET.read_text()), json.loads(MULTI_NET.read_text())
    lv, lm = lane_set(v), lane_set(m)
    pv, pm = {p["name"] for p in v["pads"]}, {p["name"] for p in m["pads"]}
    flows = v["scenario"]["flows"]
    return {"pads": sorted(pv), "pads_gained": sorted(pv - pm), "pads_lost": sorted(pm - pv),
            "pads_left_out": v["variant"]["pads_left_out"],
            "lanes": len(lv), "lanes_gained": sorted(lv - lm), "lanes_lost": sorted(lm - lv),
            "lane_range_m": v["lane_range_m"],
            "clairton_in_network": CLAIRTON in pv,
            "clairton_flows": [f for f in flows if f["destination"] == CLAIRTON],
            "clairton_flows_left_out": [f for f in v["scenario"]["flows_left_out"]
                                        if f["destination"] == CLAIRTON]}


def main() -> None:
    sized = battery(vtol_scenario(START_WH, "vtol_battery"), 5.0, HUB, LONGEST)
    if not sized["fixed_point"]:
        raise RuntimeError("no coupled battery closes the longest lane under the VTOL profile")
    wh = sized["sized"]["rounded"]["capacity_wh"]
    at = vtol_scenario(wh, "vtol_range")
    payload_range = []
    for p in PAYLOADS_KG:
        v = battery(at, p, HUB, LONGEST)["configured"]
        m = battery(MULTIROTOR, p, HUB, "UPMC_Oakland")["configured"]
        payload_range.append({"payload_kg": p, "vtol_max_lane_m": round(v["max_lane_m"], 1),
                              "vtol_empty_mass_kg": round(v["empty_mass_kg"], 3),
                              "multirotor_900wh_max_lane_m": round(m["max_lane_m"], 1)})
    out = {"battery": {"lane": sized["lane"], "payload_kg": 5.0,
                       "required_capacity_wh": round(sized["sized"]["required_capacity_wh"], 1),
                       "rounded_wh": wh, "rounded_passes": sized["sized"]["rounded_passes"],
                       "within_mtow": sized["sized"]["within_mtow"],
                       "empty_mass_kg": round(sized["sized"]["rounded"]["empty_mass_kg"], 3),
                       "iterations": sized["iterations"]},
           "payload_range": payload_range,
           "network": network()}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "vtol_consequences.json").write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
