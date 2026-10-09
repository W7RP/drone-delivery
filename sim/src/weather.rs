//! Weather (spec 2.10, A77 to A83): the hourly series of spec 9.7 and what one hour of it does
//! to cruise flight, the operating limits, the landing, the battery and the truck. Everything
//! here comes from the optional `[weather]` section (8.2); without it the core never builds
//! any of this and runs exactly as before.

use std::sync::Arc;

use crate::flight::FlightModel;
use crate::ids::{TimeMs, WeatherHoldPolicy};
use crate::netfile::{WX_RAIN, WX_SNOW, WeatherFile};

/// Station anemometer height of the series (spec 9.7), m.
pub const STATION_HEIGHT_M: f64 = 10.0;
pub const MS_PER_HOUR: TimeMs = 3_600_000;
const GRAVITY_MPS2: f64 = 9.806_65;
/// Ground speed never drops below this, so a lane stays finite when the wind at cruise
/// height is stronger than the drone can make headway against. Dispatch limits keep flights
/// far from it; it only matters for a flight already airborne when the hour changes (A78).
const MIN_GROUND_SPEED_MPS: f64 = 1.0;

/// Limits on one hour's weather (A79). Beyond any dispatch limit no flight is dispatched;
/// beyond either landing limit (wind and gust, at or above the dispatch ones) no landing is
/// cleared.
#[derive(Debug, Clone, Copy)]
pub struct Limits {
    pub max_wind_mps: f64,
    pub max_gust_mps: f64,
    pub min_visibility_m: f64,
    pub max_precip_mm_h: f64,
    pub min_temp_c: f64,
    pub max_temp_c: f64,
    pub landing_max_wind_mps: f64,
    pub landing_max_gust_mps: f64,
}

/// `[weather]` of spec 8.2. Speeds and gusts in limits are at the station's 10 m.
#[derive(Debug, Clone)]
pub struct WeatherCfg {
    pub series: Arc<WeatherFile>,
    /// Series hour at sim t = 0.
    pub start_hour: usize,
    pub wind_shear_exponent: f64,
    pub cruise_height_m: f64,
    pub max_airspeed_mps: f64,
    pub rotor_disc_area_m2: f64,
    pub air_density_kgpm3: f64,
    pub limits: Limits,
    pub latch_gust_k: f64,
    pub latch_gust_ref_mps: f64,
    pub marker_penalty_rain: f64,
    pub marker_penalty_snow: f64,
    pub marker_penalty_low_visibility: f64,
    pub low_visibility_m: f64,
    /// Usable-capacity factor against air temperature (deg C), temperatures ascending;
    /// linear between points and flat beyond the ends (A81).
    pub battery_derate: Vec<(f64, f64)>,
    pub truck_factor_rain: f64,
    pub truck_factor_snow: f64,
    pub truck_factor_low_visibility: f64,
    pub hold_policy: WeatherHoldPolicy,
    pub truck_fallback_timeout_ms: TimeMs,
    /// Routes a fully charged drone cannot fly this hour hold up only their own shipments:
    /// FM does not assign them, pads place other shipments first, and TRUCK_FALLBACK takes
    /// them after its timeout (A89).
    pub energy_route_guard: bool,
}

/// One hour of the series, in SI, wind and gust at 10 m.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hour {
    pub wind_mps: f64,
    pub wind_from_deg: f64,
    pub gust_mps: f64,
    pub temp_c: f64,
    pub visibility_m: f64,
    pub precip_mm_h: f64,
    pub flags: u8,
    pub missing: bool,
}

impl WeatherCfg {
    /// Hours of the series a run of `duration_ms` reads, from `start_hour`.
    pub fn hours_needed(duration_ms: TimeMs) -> usize {
        usize::try_from(duration_ms.div_ceil(MS_PER_HOUR)).unwrap_or(usize::MAX)
    }

    /// The hour that holds sim time `t_ms`. The config rules keep it inside the series.
    pub fn hour_index(&self, t_ms: TimeMs) -> usize {
        self.start_hour + usize::try_from(t_ms / MS_PER_HOUR).unwrap_or(usize::MAX)
    }

    /// As `hour_index`, held at the last hour for times past the series (a truck baseline
    /// can depart after the run ends).
    pub fn hour_index_clamped(&self, t_ms: TimeMs) -> usize {
        self.hour_index(t_ms).min(self.series.wind_mps.len() - 1)
    }

    pub fn hour(&self, index: usize) -> Hour {
        let s = &self.series;
        Hour {
            wind_mps: f64::from(s.wind_mps[index]),
            wind_from_deg: f64::from(s.wind_from_deg[index]),
            gust_mps: f64::from(s.gust_mps[index]),
            temp_c: f64::from(s.temp_c[index]),
            visibility_m: f64::from(s.visibility_m[index]),
            precip_mm_h: f64::from(s.precip_mm_h[index]),
            flags: s.wx_flags[index],
            missing: s.missing[index] != 0,
        }
    }

    /// Dispatch is allowed (A79): within every dispatch limit, and observed. An hour the
    /// series marks missing blocks dispatch, since nobody knows what it held.
    pub fn flyable(&self, h: &Hour) -> bool {
        let l = &self.limits;
        !h.missing
            && h.wind_mps <= l.max_wind_mps
            && h.gust_mps <= l.max_gust_mps
            && h.visibility_m >= l.min_visibility_m
            && h.precip_mm_h <= l.max_precip_mm_h
            && (l.min_temp_c..=l.max_temp_c).contains(&h.temp_c)
    }

    /// Landing clearance (A79): wind and gust within the landing limits. Rain, snow and
    /// visibility act on a landing through marker detection, temperature through the battery.
    /// A missing hour does not block a landing: the drone is already in the air and the held
    /// values are the best estimate.
    pub fn landing_ok(&self, h: &Hour) -> bool {
        h.wind_mps <= self.limits.landing_max_wind_mps
            && h.gust_mps <= self.limits.landing_max_gust_mps
    }

    /// Wind at cruise height as the vector it blows toward, ENU (east, north) m/s (A78):
    /// the 10 m wind scaled by the power law `(z / 10 m)^alpha`.
    pub fn cruise_wind_enu(&self, h: &Hour) -> (f64, f64) {
        let speed =
            h.wind_mps * (self.cruise_height_m / STATION_HEIGHT_M).powf(self.wind_shear_exponent);
        let from = h.wind_from_deg.to_radians();
        (-speed * from.sin(), -speed * from.cos())
    }

    /// Multiplier on the per-axis spread of the OUT landing error (A80): `1 + k (gust/ref)^2`.
    pub fn landing_spread_factor(&self, h: &Hour) -> f64 {
        let g = h.gust_mps / self.latch_gust_ref_mps;
        1.0 + self.latch_gust_k * g * g
    }

    /// Multiplier on `pad_ops.p_marker_detect` (A80): one penalty per condition present.
    pub fn marker_factor(&self, h: &Hour) -> f64 {
        let mut f = 1.0;
        if h.flags & WX_RAIN != 0 {
            f *= 1.0 - self.marker_penalty_rain;
        }
        if h.flags & WX_SNOW != 0 {
            f *= 1.0 - self.marker_penalty_snow;
        }
        if h.visibility_m < self.low_visibility_m {
            f *= 1.0 - self.marker_penalty_low_visibility;
        }
        f
    }

    /// Usable-capacity factor at the hour's temperature (A81).
    pub fn capacity_factor(&self, h: &Hour) -> f64 {
        piecewise_linear(&self.battery_derate, h.temp_c)
    }

    /// Truck speed factor (A82): one factor per condition present. Rain counts when the hour
    /// has rain flagged or any precipitation without snow.
    pub fn truck_factor(&self, h: &Hour) -> f64 {
        let snow = h.flags & WX_SNOW != 0;
        let rain = h.flags & WX_RAIN != 0 || (h.precip_mm_h > 0.0 && !snow);
        let mut f = 1.0;
        if rain {
            f *= self.truck_factor_rain;
        }
        if snow {
            f *= self.truck_factor_snow;
        }
        if h.visibility_m < self.low_visibility_m {
            f *= self.truck_factor_low_visibility;
        }
        f
    }
}

/// Linear interpolation over `(x, y)` points with ascending x, flat beyond the ends.
fn piecewise_linear(points: &[(f64, f64)], x: f64) -> f64 {
    let (first, last) = (points[0], points[points.len() - 1]);
    if x <= first.0 {
        return first.1;
    }
    if x >= last.0 {
        return last.1;
    }
    let i = points.partition_point(|p| p.0 <= x);
    let ((x0, y0), (x1, y1)) = (points[i - 1], points[i]);
    y0 + (y1 - y0) * (x - x0) / (x1 - x0)
}

/// Cruise power against airspeed (A77). The shape is ideal momentum theory for the induced
/// power of the rotors (Phase 5 audit: air density and total disc area) plus a parasitic
/// term `c V^3`. `c` is solved so the shape at the reference mass equals the fitted cruise
/// power at the fitted cruise speed, and the shape is then scaled to the fitted power at every
/// mass, so in still air the result is the fitted model exactly.
#[derive(Debug, Clone, Copy)]
pub struct CruiseCurve {
    pub model: FlightModel,
    pub rho_area: f64,
    pub parasitic_w_per_mps3: f64,
    /// A VTOL's fixed-wing cruise (A93): power proportional to airspeed, since the SITL wing's
    /// drag does not change with speed in level flight.
    pub fixed_wing: bool,
}

impl CruiseCurve {
    /// None when the fitted cruise power is below the ideal induced power at the reference
    /// mass: no parasitic term could make the curve pass through the fit.
    pub fn new(model: FlightModel, air_density_kgpm3: f64, disc_area_m2: f64) -> Option<Self> {
        let mut curve = Self {
            model,
            rho_area: air_density_kgpm3 * disc_area_m2,
            parasitic_w_per_mps3: 0.0,
            fixed_wing: false,
        };
        let v = model.cruise_speed_mps;
        let spare_w = model.cruise_power_ref_w - curve.induced_w(model.power_ref_mass_kg, v);
        if spare_w.is_nan() || spare_w < 0.0 {
            return None;
        }
        curve.parasitic_w_per_mps3 = spare_w / (v * v * v);
        Some(curve)
    }

    /// Ideal induced power in level forward flight at `airspeed_mps` (Glauert): with
    /// `v_h^2 = W / (2 rho A)`, the induced velocity solves `v_i^4 + V^2 v_i^2 = v_h^4`.
    fn induced_w(&self, mass_kg: f64, airspeed_mps: f64) -> f64 {
        let weight_n = mass_kg * GRAVITY_MPS2;
        let vh2 = weight_n / (2.0 * self.rho_area);
        let v2 = airspeed_mps * airspeed_mps;
        let vi2 = 0.5 * ((v2 * v2 + 4.0 * vh2 * vh2).sqrt() - v2);
        weight_n * vi2.sqrt()
    }

    fn shape_w(&self, mass_kg: f64, airspeed_mps: f64) -> f64 {
        self.induced_w(mass_kg, airspeed_mps) + self.parasitic_w_per_mps3 * airspeed_mps.powi(3)
    }

    /// The fixed-wing form of a VTOL (A93).
    pub fn fixed_wing(model: FlightModel) -> Self {
        Self {
            model,
            rho_area: 0.0,
            parasitic_w_per_mps3: 0.0,
            fixed_wing: true,
        }
    }

    /// The curve for the scenario's airframe; None as in `new`.
    pub fn for_config(cfg: &crate::config::Config, w: &WeatherCfg) -> Option<Self> {
        let model = FlightModel::from_config(cfg);
        if cfg.drone.airframe == crate::ids::Airframe::Vtol {
            return Some(Self::fixed_wing(model));
        }
        Self::new(model, w.air_density_kgpm3, w.rotor_disc_area_m2)
    }

    pub fn power_w(&self, payload_kg: f64, airspeed_mps: f64) -> f64 {
        let m = &self.model;
        if self.fixed_wing {
            return m.cruise_power_w(payload_kg) * airspeed_mps / m.cruise_speed_mps;
        }
        let mass_kg = m.empty_mass_kg + m.latch_mass_kg + payload_kg;
        let fitted_w = m.cruise_power_w(payload_kg);
        fitted_w * self.shape_w(mass_kg, airspeed_mps) / self.shape_w(mass_kg, m.cruise_speed_mps)
    }
}

/// The wind one flight sees, and how the drone flies through it (A78).
#[derive(Debug, Clone, Copy)]
pub struct CruiseWind {
    pub east_mps: f64,
    pub north_mps: f64,
    pub max_airspeed_mps: f64,
    pub curve: CruiseCurve,
}

impl CruiseWind {
    /// Time and energy of a straight lane of `length_m` along the unit track `(ue, un)`. The
    /// drone holds the fitted cruise speed over the ground unless that needs more than the
    /// maximum airspeed; then it flies at the maximum airspeed and the ground speed drops.
    pub fn lane(&self, length_m: f64, ue: f64, un: f64, payload_kg: f64) -> (f64, f64) {
        let along = self.east_mps * ue + self.north_mps * un;
        let cross = self.east_mps * un - self.north_mps * ue;
        let air_at = |ground: f64| (ground - along).hypot(cross);
        let mut ground = self.curve.model.cruise_speed_mps;
        if air_at(ground) > self.max_airspeed_mps {
            let room = (self.max_airspeed_mps.powi(2) - cross * cross).max(0.0);
            ground = along + room.sqrt();
        }
        let ground = ground.max(MIN_GROUND_SPEED_MPS);
        let time_s = length_m / ground;
        (
            time_s,
            self.curve.power_w(payload_kg, air_at(ground)) * time_s,
        )
    }
}

/// What the current hour does to the run (2.10), refreshed at every `WEATHER_HOUR`.
#[derive(Debug, Clone, Copy)]
pub struct WeatherNow {
    pub hour: Hour,
    pub flyable: bool,
    pub landing_ok: bool,
    /// Start of the current run of hours that block dispatch, if one is open.
    pub blocked_since: Option<TimeMs>,
    pub wind: CruiseWind,
    pub capacity_factor: f64,
    pub marker_factor: f64,
    pub truck_factor: f64,
    /// Derived latch-fail probability at this hour's gust (A80).
    pub p_latch_fault: f64,
    /// Landing requests held for weather (spec 7, M10).
    pub aborts: u64,
}

impl WeatherNow {
    /// The hour holding `t_ms`; `blocked_since` carries over from the previous hour.
    pub fn at(
        w: &WeatherCfg,
        curve: CruiseCurve,
        latch: crate::config::LatchDerivation,
        t_ms: TimeMs,
        previous: Option<&WeatherNow>,
    ) -> Self {
        let hour = w.hour(w.hour_index(t_ms));
        let flyable = w.flyable(&hour);
        let (east_mps, north_mps) = w.cruise_wind_enu(&hour);
        let blocked_since = if flyable {
            None
        } else {
            Some(previous.and_then(|p| p.blocked_since).unwrap_or(t_ms))
        };
        Self {
            hour,
            flyable,
            landing_ok: w.landing_ok(&hour),
            blocked_since,
            wind: CruiseWind {
                east_mps,
                north_mps,
                max_airspeed_mps: w.max_airspeed_mps,
                curve,
            },
            capacity_factor: w.capacity_factor(&hour),
            marker_factor: w.marker_factor(&hour),
            truck_factor: w.truck_factor(&hour),
            p_latch_fault: crate::flight::landing_miss_probability(
                latch.out_error_bias_m,
                latch.out_error_sigma_m * w.landing_spread_factor(&hour),
                latch.capture_radius_m,
            ),
            aborts: previous.map_or(0, |p| p.aborts),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The fitted PX4 SITL profile at 9 kg empty, and the Phase 5 audit's rotors.
    fn fitted() -> CruiseCurve {
        let model = FlightModel {
            empty_mass_kg: 9.0,
            latch_mass_kg: 0.6,
            power_ref_mass_kg: 14.6,
            cruise_power_ref_w: 2107.87,
            hover_power_ref_w: 1830.508,
            cruise_speed_mps: 15.033,
            transition_s: 0.0,
            transition_power_ref_w: 0.0,
            cruise_mass_exponent: 1.5,
        };
        CruiseCurve::new(model, 1.225, 4.0 * 0.061_22).expect("fit above ideal induced power")
    }

    fn wind(east_mps: f64, max_airspeed_mps: f64) -> CruiseWind {
        CruiseWind {
            east_mps,
            north_mps: 0.0,
            max_airspeed_mps,
            curve: fitted(),
        }
    }

    #[test]
    fn still_air_is_the_fitted_model() {
        let c = fitted();
        for payload in [0.0, 5.0] {
            let (t, e) = wind(0.0, 25.0).lane(2000.0, 1.0, 0.0, payload);
            let (t_fit, e_fit) = c.model.segment(2000.0, payload);
            assert!((t - t_fit).abs() < 1e-9 && (e / e_fit - 1.0).abs() < 1e-12);
        }
    }

    #[test]
    fn headwind_costs_more_than_tailwind_and_crosswind_is_symmetric() {
        let w = wind(6.0, 25.0);
        let (t_tail, e_tail) = w.lane(2000.0, 1.0, 0.0, 5.0);
        let (t_head, e_head) = w.lane(2000.0, -1.0, 0.0, 5.0);
        let (t_still, e_still) = wind(0.0, 25.0).lane(2000.0, 1.0, 0.0, 5.0);
        // Ground speed is held, so time is unchanged; power follows airspeed.
        assert!((t_tail - t_still).abs() < 1e-9 && (t_head - t_still).abs() < 1e-9);
        assert!(
            e_head > e_still && e_still > e_tail,
            "{e_head} {e_still} {e_tail}"
        );
        let (_, e_left) = w.lane(2000.0, 0.0, 1.0, 5.0);
        let (_, e_right) = w.lane(2000.0, 0.0, -1.0, 5.0);
        assert!((e_left - e_right).abs() < 1e-6);
    }

    #[test]
    fn airspeed_limit_slows_the_drone_over_the_ground() {
        // 10 m/s headwind and a 20 m/s airspeed limit: 15 m/s over the ground would need 25.
        let (t, _) = wind(10.0, 20.0).lane(2000.0, -1.0, 0.0, 5.0);
        assert!(
            (t - 2000.0 / 10.0).abs() < 1e-9,
            "ground speed 10 m/s, got {}",
            2000.0 / t
        );
        // A wind stronger than the limit leaves the ground-speed floor.
        let (t, _) = wind(30.0, 20.0).lane(2000.0, -1.0, 0.0, 5.0);
        assert!((t - 2000.0 / MIN_GROUND_SPEED_MPS).abs() < 1e-9);
    }

    #[test]
    fn derate_table_interpolates_and_holds_its_ends() {
        let table = [(-20.0, 0.7), (0.0, 0.9), (20.0, 1.0)];
        assert!((piecewise_linear(&table, -30.0) - 0.7).abs() < 1e-12);
        assert!((piecewise_linear(&table, -10.0) - 0.8).abs() < 1e-12);
        assert!((piecewise_linear(&table, 30.0) - 1.0).abs() < 1e-12);
    }
}
