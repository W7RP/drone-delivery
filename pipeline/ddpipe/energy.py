"""The sim's flight model and energy guard (spec 2.7 and 6, `sim/src/flight.rs`), mirrored so
the pipeline only proposes lanes and flows the sim will accept (rules R12 and R16). The sim
re-checks both at load; if this mirror drifts, the Pittsburgh load test fails."""

from dataclasses import dataclass

import numpy as np
from scipy.sparse.csgraph import shortest_path

from .params import q


@dataclass(frozen=True)
class FlightModel:
    empty_mass_kg: float
    latch_mass_kg: float
    power_ref_mass_kg: float
    cruise_power_ref_w: float
    hover_power_ref_w: float
    cruise_speed_mps: float
    capacity_j: float
    reserve_soc: float
    charge_target_soc: float
    takeoff_s: float
    t_cycle_s: float
    t_contingency_s: float
    out_contacts: bool

    @staticmethod
    def from_scenario(cfg: dict) -> "FlightModel":
        d, ops, pol = cfg["drone"], cfg["pad_ops"], cfg["policy"]
        go_arounds = q(ops["max_go_arounds"], "count")
        t_cycle = (go_arounds + 1) * q(d["approach_s"], "s") + go_arounds * q(d["go_around_s"], "s")
        cycles = q(pol["marker_max_hold_cycles"], "count")
        t_contingency = (cycles + 1) * t_cycle + max(
            q(pol["max_hold_s"], "s"), cycles * q(pol["marker_hold_s"], "s")
        )
        return FlightModel(
            empty_mass_kg=q(d["empty_mass_kg"], "kg"),
            latch_mass_kg=q(d["latch_mass_kg"], "kg"),
            power_ref_mass_kg=q(d["power_ref_mass_kg"], "kg"),
            cruise_power_ref_w=q(d["cruise_power_ref_w"], "W"),
            hover_power_ref_w=q(d["hover_power_ref_w"], "W"),
            cruise_speed_mps=q(d["cruise_speed_mps"], "m/s"),
            capacity_j=q(d["battery_capacity_j"], "J"),
            reserve_soc=q(d["reserve_soc"], "1"),
            charge_target_soc=q(d["charge_target_soc"], "1"),
            takeoff_s=q(d["takeoff_s"], "s"),
            t_cycle_s=t_cycle,
            t_contingency_s=t_contingency,
            out_contacts=cfg["charging"]["mode"] == "OUT_CONTACTS",
        )

    def mass_factor(self, payload_kg: float) -> float:
        total = self.empty_mass_kg + self.latch_mass_kg + payload_kg
        return (total / self.power_ref_mass_kg) ** 1.5

    def cruise_j_per_m(self, payload_kg: float) -> float:
        return self.cruise_power_ref_w * self.mass_factor(payload_kg) / self.cruise_speed_mps

    def hover_w(self, payload_kg: float) -> float:
        return self.hover_power_ref_w * self.mass_factor(payload_kg)

    def usable_j(self) -> float:
        return (self.charge_target_soc - self.reserve_soc) * self.capacity_j

    def fixed_need_j(self, payload_kg: float) -> float:
        """Hover energy dispatch_ok always needs: takeoff, the landing contingency and one
        approach cycle at the diversion pad."""
        return self.hover_w(payload_kg) * (self.takeoff_s + self.t_contingency_s + self.t_cycle_s)

    def max_lane_m(self, payload_kg: float) -> float:
        """Longest lane dispatch_ok could accept from charge_target_soc: everything left after
        the fixed hover energy, spent cruising, with a zero-length diversion."""
        return (self.usable_j() - self.fixed_need_j(payload_kg)) / self.cruise_j_per_m(payload_kg)


def routes(n_pads: int, lanes: list[tuple[int, int, float]]) -> np.ndarray:
    """All-pairs shortest route length over undirected lanes (a, b, length_m); inf if none."""
    w = np.zeros((n_pads, n_pads))
    for a, b, length in lanes:
        w[a, b] = w[b, a] = length
    return shortest_path(w, method="D", directed=False)


def nearest_other(route: np.ndarray, b: int) -> int | None:
    """`alt(b)`: the pad with the shortest route from b, lower ID on ties (flight.rs)."""
    best = None
    for q_ in range(route.shape[0]):
        if q_ != b and np.isfinite(route[b, q_]) and (best is None or route[b, q_] < route[b, best]):
            best = q_
    return best


def dispatch_need_j(model: FlightModel, route: np.ndarray, a: int, b: int, payload_kg: float,
                    has_charger) -> float:
    """`dispatch_need_j` of flight.rs with every pad online."""
    e_m = model.cruise_j_per_m(payload_kg)
    alt = nearest_other(route, b)
    divert = (route[b, alt] * e_m if alt is not None else 0.0)
    charger = 0.0
    if not has_charger(b):
        h = min((q_ for q_ in range(route.shape[0]) if q_ != b and has_charger(q_)),
                key=lambda q_: (route[b, q_], q_), default=None)
        charger = route[b, h] * model.cruise_j_per_m(0.0) if h is not None else 0.0
    return model.fixed_need_j(payload_kg) + route[a, b] * e_m + divert + charger
