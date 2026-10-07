"""Step 7: write a variant's scenario.toml (ddpipe/variants.py) from scenario.toml.in. Demand
runs from each warehouse pad to every other pad the drone can reach under the sim's R16 energy
guard; flows it cannot serve are logged and left out. Fleet size follows one rule (A52):
offered drone-hours per hour over a target utilisation, rounded up."""

import logging
import math

from . import energy, variants
from .params import q
from .paths import PIPELINE

log = logging.getLogger(__name__)

TEMPLATE = PIPELINE / "scenario.toml.in"
FLEET_MARKER = "# @fleet_size: written by the pipeline (scenario.py).\n"
FLOWS_MARKER = "# @flows: written by the pipeline (scenario.py), warehouse pads to every other pad.\n"
WAREHOUSE = "WAREHOUSE"
SECONDS_PER_HOUR = 3600.0


def busy_s_per_order(cfg: dict, model: energy.FlightModel, route_m: float) -> float:
    """Drone time one order takes: out with the loaded pod and back with an empty one (takeoff,
    cruise and approach each way), latch, release and the IN-to-OUT hop with their
    verifications, and the charging time that puts back the energy of both legs."""
    d, ops, ch = cfg["drone"], cfg["pad_ops"], cfg["charging"]
    hover_s = q(d["takeoff_s"], "s") + q(d["approach_s"], "s")
    leg_s = hover_s + route_m / q(d["cruise_speed_mps"], "m/s")
    service_s = sum(q(ops[k], "s") for k in ("latch_s", "latch_verify_s", "release_s",
                                             "release_verify_s")) + q(d["hop_s"], "s")
    energy_j = sum(model.hover_w(m) * hover_s + model.cruise_j_per_m(m) * route_m
                   for m in (q(cfg["payload"]["pod_gross_mass_kg"], "kg"),
                             q(cfg["payload"]["pod_empty_mass_kg"], "kg")))
    charge_s = energy_j / (q(ch["power_w"], "W") * q(ch["efficiency"], "1"))
    return 2 * leg_s + service_s + charge_s


def flows(pads, lane_list, cfg: dict, model: energy.FlightModel, params: dict):
    """(flows, rejected): flows as (origin, dest, rate_per_h); rejected as (origin, dest, need_j)."""
    rate = q(params["demand"]["rate_per_warehouse_per_h"], "1/h")
    payload_kg = q(cfg["payload"]["pod_gross_mass_kg"], "kg")
    route = energy.routes(len(pads), [(l.a, l.b, l.length_m) for l in lane_list])
    has_charger = (lambda p: True) if model.out_contacts else (lambda p: pads[p].is_hub)
    out, rejected = [], []
    for o, origin in enumerate(pads):
        if origin.role != WAREHOUSE:
            continue
        servable = []
        for d, dest in enumerate(pads):
            if dest.role == WAREHOUSE:
                continue
            need = energy.dispatch_need_j(model, route, o, d, payload_kg, has_charger)
            if need > model.usable_j():
                rejected.append((origin.name, dest.name, need))
            else:
                servable.append(d)
        for d in servable:
            out.append((o, d, rate / len(servable)))
    for o, d, need in rejected:
        log.info("flow left out: %s -> %s needs %.0f J above reserve, battery gives %.0f J (R16)",
                 o, d, need, model.usable_j())
    return out, rejected, route


def fleet_size(flow_list, route, cfg: dict, model: energy.FlightModel,
               params: dict) -> tuple[int, float]:
    offered = sum(r * busy_s_per_order(cfg, model, route[o, d])
                  for o, d, r in flow_list) / SECONDS_PER_HOUR
    target = q(params["demand"]["target_utilisation"], "1")
    return max(1, math.ceil(offered / target)), offered


def _quantity(value, unit: str) -> str:
    return f'{{ value = {value!r}, unit = "{unit}", assumption = true }}'


def write(pads, lane_list, cfg: dict, model: energy.FlightModel, params: dict,
          variant: variants.Variant) -> dict:
    flow_list, rejected, route = flows(pads, lane_list, cfg, model, params)
    fleet, offered = fleet_size(flow_list, route, cfg, model, params)
    target = q(params["demand"]["target_utilisation"], "1")
    log.info("fleet size %d: %.3f offered drone-hours per hour / target utilisation %.2f",
             fleet, offered, target)
    text = variants.text(TEMPLATE.read_text(), variant)
    if text.count(FLEET_MARKER) != 1 or text.count(FLOWS_MARKER) != 1:
        raise ValueError(f"{TEMPLATE.name}: each @ marker must appear exactly once")
    text = text.replace(FLEET_MARKER, (
        f"# Rule (spec A52): ceil(offered drone-hours per hour {offered:.3f} / target "
        f"utilisation {target}).\n"
        f"fleet_size = {_quantity(fleet, 'count')}\n"))
    lines = [f"# Each warehouse sends {q(params['demand']['rate_per_warehouse_per_h'], '1/h')} "
             "requests per hour, split evenly over the pads its drones can reach (R16)."]
    for o, d, r in rejected:
        lines.append(f"# Left out, beyond the energy guard: {o} -> {d}.")
    for o, d, r in flow_list:
        lines += ["", "[[demand.flows]]", f'origin = "{pads[o].name}"',
                  f'destination = "{pads[d].name}"', f"rate_per_h = {_quantity(round(r, 6), '1/h')}"]
    text = text.replace(FLOWS_MARKER, "\n".join(lines) + "\n")
    variant.scenario_dir.mkdir(parents=True, exist_ok=True)
    path = variant.scenario_dir / "scenario.toml"
    tmp = path.with_suffix(".toml.part")
    tmp.write_text(text)
    tmp.rename(path)
    log.info("scenario %s: %d flows, %d left out, fleet %d", variant.name, len(flow_list),
             len(rejected), fleet)
    return {"fleet_size": fleet, "offered_drone_hours_per_hour": round(offered, 4),
            "flows": [{"origin": pads[o].name, "destination": pads[d].name,
                       "rate_per_h": round(r, 6)} for o, d, r in flow_list],
            "flows_left_out": [{"origin": o, "destination": d, "need_j": round(n, 1)}
                               for o, d, n in rejected]}
