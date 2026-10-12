//! Flight model (spec 2.7) and the shared energy rules of spec 6 (`dispatch_ok`, `divert_ok`).
//!
//! Energies in J, powers in W, masses in kg, times in s. The latch mass is always carried.

use crate::config::Config;
use crate::ids::PadId;
use crate::network::Network;

#[derive(Debug, Clone, Copy)]
pub struct FlightModel {
    pub empty_mass_kg: f64,
    pub latch_mass_kg: f64,
    pub power_ref_mass_kg: f64,
    pub cruise_power_ref_w: f64,
    pub hover_power_ref_w: f64,
    pub cruise_speed_mps: f64,
}

impl FlightModel {
    pub fn from_config(cfg: &Config) -> Self {
        let d = &cfg.drone;
        Self {
            empty_mass_kg: d.empty_mass_kg,
            latch_mass_kg: d.latch_mass_kg,
            power_ref_mass_kg: d.power_ref_mass_kg,
            cruise_power_ref_w: d.cruise_power_ref_w,
            hover_power_ref_w: d.hover_power_ref_w,
            cruise_speed_mps: d.cruise_speed_mps,
        }
    }

    fn mass_factor(&self, payload_kg: f64) -> f64 {
        ((self.empty_mass_kg + self.latch_mass_kg + payload_kg) / self.power_ref_mass_kg).powf(1.5)
    }

    /// `flight(lane, payload) -> (time_s, energy_j)` for a straight segment of `length_m`.
    pub fn segment(&self, length_m: f64, payload_kg: f64) -> (f64, f64) {
        let time_s = length_m / self.cruise_speed_mps;
        (
            time_s,
            self.cruise_power_ref_w * self.mass_factor(payload_kg) * time_s,
        )
    }

    pub fn hover_power_w(&self, payload_kg: f64) -> f64 {
        self.hover_power_ref_w * self.mass_factor(payload_kg)
    }
}

/// Derived durations of spec 3.7 and 6 plus the battery limits the energy guards use.
#[derive(Debug, Clone, Copy)]
pub struct EnergyRules {
    pub model: FlightModel,
    pub capacity_j: f64,
    pub reserve_soc: f64,
    pub charge_target_soc: f64,
    pub takeoff_s: f64,
    pub t_cycle_s: f64,
    pub t_contingency_s: f64,
}

impl EnergyRules {
    pub fn from_config(cfg: &Config) -> Self {
        let d = &cfg.drone;
        let go_arounds = f64::from(cfg.pad_ops.max_go_arounds);
        let t_cycle_s = (go_arounds + 1.0) * d.approach_s + go_arounds * d.go_around_s;
        let cycles = f64::from(cfg.policy.marker_max_hold_cycles);
        let t_contingency_s = (cycles + 1.0) * t_cycle_s
            + cfg.policy.max_hold_s.max(cycles * cfg.policy.marker_hold_s);
        Self {
            model: FlightModel::from_config(cfg),
            capacity_j: d.battery_capacity_j,
            reserve_soc: d.reserve_soc,
            charge_target_soc: d.charge_target_soc,
            takeoff_s: d.takeoff_s,
            t_cycle_s,
            t_contingency_s,
        }
    }

    pub fn route_energy(&self, net: &Network, a: PadId, b: PadId, payload_kg: f64) -> f64 {
        self.model.segment(net.route_len(a, b), payload_kg).1
    }

    pub fn route_time_s(&self, net: &Network, a: PadId, b: PadId) -> f64 {
        net.route_len(a, b) / self.model.cruise_speed_mps
    }

    /// `alt(b)`: nearest pad by route energy among `candidates` (ID order breaks ties).
    pub fn nearest_other(
        &self,
        net: &Network,
        b: PadId,
        mut candidate: impl FnMut(PadId) -> bool,
    ) -> Option<PadId> {
        let mut best: Option<(f64, PadId)> = None;
        for q in 0..net.n_pads {
            let q = PadId::try_from(q).expect("pad count fits u16");
            if q == b || !candidate(q) {
                continue;
            }
            let len = net.route_len(b, q);
            if best.is_none_or(|(l, _)| len < l) {
                best = Some((len, q));
            }
        }
        best.map(|(_, q)| q)
    }

    /// `E_divert(b, m)`: the nearest alternate counted among `online` pads; when none is
    /// online the geometric nearest is used so the guard stays conservative (A41).
    pub fn divert_energy(
        &self,
        net: &Network,
        b: PadId,
        payload_kg: f64,
        online: impl FnMut(PadId) -> bool,
    ) -> f64 {
        let alt = self
            .nearest_other(net, b, online)
            .or_else(|| self.nearest_other(net, b, |_| true));
        let route = alt.map_or(0.0, |q| self.route_energy(net, b, q, payload_kg));
        route + self.model.hover_power_w(payload_kg) * self.t_cycle_s
    }

    /// `E_charger(b)`: 0 at a pad with a charger, else the empty route to the nearest charger.
    pub fn charger_energy(
        &self,
        net: &Network,
        b: PadId,
        has_charger: impl Fn(PadId) -> bool,
    ) -> f64 {
        if has_charger(b) {
            return 0.0;
        }
        self.nearest_other(net, b, &has_charger)
            .map_or(0.0, |h| self.route_energy(net, b, h, 0.0))
    }

    /// Energy `dispatch_ok` requires above the reserve for a -> b with `payload_kg`.
    pub fn dispatch_need_j(
        &self,
        net: &Network,
        a: PadId,
        b: PadId,
        payload_kg: f64,
        online: impl FnMut(PadId) -> bool,
        has_charger: impl Fn(PadId) -> bool,
    ) -> f64 {
        let hover = self.model.hover_power_w(payload_kg);
        hover * self.takeoff_s
            + self.route_energy(net, a, b, payload_kg)
            + hover * self.t_contingency_s
            + self.divert_energy(net, b, payload_kg, online)
            + self.charger_energy(net, b, has_charger)
    }

    pub fn usable_j(&self, soc: f64) -> f64 {
        (soc - self.reserve_soc) * self.capacity_j
    }

    pub fn divert_ok(&self, net: &Network, soc: f64, b: PadId, q: PadId, payload_kg: f64) -> bool {
        let need = self.route_energy(net, b, q, payload_kg)
            + self.model.hover_power_w(payload_kg) * self.t_cycle_s;
        self.usable_j(soc) >= need
    }
}
