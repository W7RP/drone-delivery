//! Flight model (spec 2.7) and the shared energy rules of spec 6 (`dispatch_ok`, `divert_ok`).
//!
//! Energies in J, powers in W, masses in kg, times in s. The latch mass is always carried.

use crate::config::Config;
use crate::ids::PadId;
use crate::network::Network;
use crate::weather::CruiseWind;

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

    pub fn cruise_power_w(&self, payload_kg: f64) -> f64 {
        self.cruise_power_ref_w * self.mass_factor(payload_kg)
    }

    pub fn hover_power_w(&self, payload_kg: f64) -> f64 {
        self.hover_power_ref_w * self.mass_factor(payload_kg)
    }
}

/// Simpson intervals for the landing-error integral; ample for a smooth density on [0, R].
const LANDING_ERROR_STEPS: usize = 2000;
/// Terms of the I0 series; enough for arguments far beyond a landing error's.
const BESSEL_I0_TERMS: u32 = 80;

/// Modified Bessel function of the first kind, order 0, by its power series.
fn bessel_i0(x: f64) -> f64 {
    let quarter_x2 = x * x / 4.0;
    let mut term = 1.0;
    let mut sum = 1.0;
    for k in 1..BESSEL_I0_TERMS {
        term *= quarter_x2 / f64::from(k * k);
        sum += term;
        if term < sum * f64::EPSILON {
            break;
        }
    }
    sum
}

/// Probability that a landing misses a capture radius `radius_m` (A71). The horizontal error
/// at OUT is a 2-D Gaussian offset `bias_m` from the spot with per-axis spread `sigma_m`, so its
/// distance from the spot is Rice distributed (Rayleigh when the bias is 0). The density is
/// integrated over [0, R] by Simpson's rule.
pub fn landing_miss_probability(bias_m: f64, sigma_m: f64, radius_m: f64) -> f64 {
    let s2 = sigma_m * sigma_m;
    let pdf = |r: f64| {
        r / s2 * (-(r * r + bias_m * bias_m) / (2.0 * s2)).exp() * bessel_i0(r * bias_m / s2)
    };
    #[allow(clippy::cast_precision_loss)]
    let h = radius_m / LANDING_ERROR_STEPS as f64;
    let mut sum = pdf(0.0) + pdf(radius_m);
    for i in 1..LANDING_ERROR_STEPS {
        #[allow(clippy::cast_precision_loss)]
        let weight = if i % 2 == 1 { 4.0 } else { 2.0 };
        #[allow(clippy::cast_precision_loss)]
        let r = h * i as f64;
        sum += weight * pdf(r);
    }
    (1.0 - sum * h / 3.0).clamp(0.0, 1.0)
}

/// Time and energy of one flight phase.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhaseEstimate {
    pub time_s: f64,
    pub energy_j: f64,
}

/// The flight phases of a normal delivery visit as the core times and charges them (spec 2.7,
/// 4.4): takeoff (D01 to D02) and approach (D04 to D05) at hover power with the pod, the lane at
/// cruise, and the IN-to-OUT hop at hover power without it (the pod was released on IN).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VisitEstimate {
    pub takeoff: PhaseEstimate,
    pub cruise: PhaseEstimate,
    pub approach: PhaseEstimate,
    pub hop: PhaseEstimate,
}

impl VisitEstimate {
    pub fn new(cfg: &Config, lane_m: f64, payload_kg: f64) -> Self {
        let model = FlightModel::from_config(cfg);
        let d = &cfg.drone;
        let hover = |seconds: f64, payload: f64| PhaseEstimate {
            time_s: seconds,
            energy_j: model.hover_power_w(payload) * seconds,
        };
        let (time_s, energy_j) = model.segment(lane_m, payload_kg);
        Self {
            takeoff: hover(d.takeoff_s, payload_kg),
            cruise: PhaseEstimate { time_s, energy_j },
            approach: hover(d.approach_s, payload_kg),
            hop: hover(d.hop_s, 0.0),
        }
    }

    pub fn total(&self) -> PhaseEstimate {
        let all = [self.takeoff, self.cruise, self.approach, self.hop];
        PhaseEstimate {
            time_s: all.iter().map(|p| p.time_s).sum(),
            energy_j: all.iter().map(|p| p.energy_j).sum(),
        }
    }
}

/// Derived durations of spec 3.7 and 6 plus the battery limits the energy guards use. With
/// weather (2.10) the world sets the hour's wind and its usable capacity here, so every guard
/// and every flight sees the same hour.
#[derive(Debug, Clone, Copy)]
pub struct EnergyRules {
    pub model: FlightModel,
    pub capacity_j: f64,
    pub wind: Option<CruiseWind>,
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
            wind: None,
            reserve_soc: d.reserve_soc,
            charge_target_soc: d.charge_target_soc,
            takeoff_s: d.takeoff_s,
            t_cycle_s,
            t_contingency_s,
        }
    }

    /// Time and energy of one directed lane: the fitted model, or in wind (A78) the lane's
    /// own track through the hour's wind.
    pub fn lane_flight(&self, net: &Network, lane: usize, payload_kg: f64) -> (f64, f64) {
        let len = f64::from(net.lane_len_m[lane]);
        let Some(wind) = self.wind else {
            return self.model.segment(len, payload_kg);
        };
        let (xa, ya) = net.pos_m[usize::from(net.lane_from[lane])];
        let (xb, yb) = net.pos_m[usize::from(net.lane_to[lane])];
        let straight = (xb - xa).hypot(yb - ya);
        let (ue, un) = if straight > 0.0 {
            ((xb - xa) / straight, (yb - ya) / straight)
        } else {
            (0.0, 0.0)
        };
        wind.lane(len, ue, un, payload_kg)
    }

    /// Sum of `lane_flight` over the route a -> b.
    fn route_flight(&self, net: &Network, a: PadId, b: PadId, payload_kg: f64) -> (f64, f64) {
        let (mut time_s, mut energy_j) = (0.0, 0.0);
        let mut at = a;
        while at != b {
            let lane = net.next_lane(at, b) as usize;
            let (t, e) = self.lane_flight(net, lane, payload_kg);
            time_s += t;
            energy_j += e;
            at = net.lane_to[lane];
        }
        (time_s, energy_j)
    }

    pub fn route_energy(&self, net: &Network, a: PadId, b: PadId, payload_kg: f64) -> f64 {
        if self.wind.is_some() {
            return self.route_flight(net, a, b, payload_kg).1;
        }
        self.model.segment(net.route_len(a, b), payload_kg).1
    }

    pub fn route_time_s(&self, net: &Network, a: PadId, b: PadId) -> f64 {
        if self.wind.is_some() {
            return self.route_flight(net, a, b, 0.0).0;
        }
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

    /// Longest one-way lane `dispatch_ok` accepts from `soc` into a pad with a charger when the
    /// diversion route is zero (an alternate next to the destination): an upper bound on range.
    pub fn max_lane_m(&self, soc: f64, payload_kg: f64) -> f64 {
        let hover = self.model.hover_power_w(payload_kg);
        let fixed_j = hover * (self.takeoff_s + self.t_contingency_s + self.t_cycle_s);
        let per_m_j = self.model.segment(1.0, payload_kg).1;
        ((self.usable_j(soc) - fixed_j) / per_m_j).max(0.0)
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

#[cfg(test)]
mod tests {
    use super::landing_miss_probability;

    #[test]
    fn rayleigh_tail_has_a_closed_form() {
        // Bias 0: P(r > R) = exp(-R^2 / 2 sigma^2).
        let (sigma, radius): (f64, f64) = (0.066_448_155_228_970_3, 0.15);
        let exact = (-radius * radius / (2.0 * sigma * sigma)).exp();
        assert!((landing_miss_probability(0.0, sigma, radius) - exact).abs() < 1e-9);
    }

    #[test]
    fn rice_tail_matches_scipy() {
        // scipy.stats.rice.sf(0.15, b=nu/sigma, scale=sigma) for the Phase 4 fit.
        let p = landing_miss_probability(0.080_035_232_046_752_18, 0.034_821_452_725_140_145, 0.15);
        assert!((p - 0.032_128_801_048_216_03).abs() < 1e-7, "p = {p}");
    }
}
