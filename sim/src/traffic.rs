//! Road traffic for the truck (spec 2.11, A84 to A86): a time-of-day delay per road class over
//! each route's class fractions (spec 9.2 version_minor 2), weekday and weekend profiles, and
//! seeded random incidents. Only the truck sees it; without `[traffic]` none of this exists.

use crate::netfile::RouteFractions;
use crate::rng::{StreamKind, Xoshiro256};

pub const HOURS_PER_DAY: usize = 24;
const DAYS_PER_WEEK: u64 = 7;
/// Saturday and Sunday, with day 0 a Monday.
const FIRST_WEEKEND_DAY: u64 = 5;
/// Incidents drawn per pair and hour stop here, so a draw always ends (A86).
const MAX_INCIDENTS_PER_PAIR_HOUR: u32 = 16;
const MS_PER_HOUR: u64 = 3_600_000;
/// Bit offset of the run hour in an incident seed (pair indices stay below it).
const HOUR_SEED_SHIFT: u32 = 24;

/// Delay classes, in the order of `peak_delay`: the three road classes of the route fractions,
/// then bridges and tunnels.
pub const DELAY_CLASSES: usize = 4;

#[derive(Debug, Clone)]
pub struct TrafficCfg {
    /// Day of week at local midnight before t = 0: 0 Monday to 6 Sunday.
    pub start_weekday: u8,
    /// Share of the peak delay at each local hour, in [0, 1].
    pub weekday_shape: [f64; HOURS_PER_DAY],
    pub weekend_shape: [f64; HOURS_PER_DAY],
    /// Extra travel time at a full peak, as a fraction of free flow: motorway and trunk,
    /// primary and secondary, other roads, bridges and tunnels (on top of their class).
    pub peak_delay: [f64; DELAY_CLASSES],
    pub incident_rate_per_pair_h: f64,
    pub incident_delay_median_s: f64,
    pub incident_delay_sigma: f64,
}

impl TrafficCfg {
    /// Whether local day `day` (counted from the day holding t = 0) is a weekend day.
    pub fn is_weekend(&self, day: u64) -> bool {
        (day + u64::from(self.start_weekday)) % DAYS_PER_WEEK >= FIRST_WEEKEND_DAY
    }

    /// Free-flow travel-time multiplier of route `k` at local `hour` (A84):
    /// sum over classes of fraction x (1 + peak delay x shape), plus the bridge and tunnel
    /// share times its own extra delay.
    pub fn multiplier(&self, f: &RouteFractions, k: usize, weekend: bool, hour: usize) -> f64 {
        let shape = if weekend {
            self.weekend_shape[hour]
        } else {
            self.weekday_shape[hour]
        };
        let delay = |c: usize| self.peak_delay[c] * shape;
        let classes: f64 = (0..3)
            .map(|c| f64::from(f.class[c][k]) * (1.0 + delay(c)))
            .sum();
        classes + f64::from(f.structure[k]) * delay(3)
    }

    /// Incident delay, in s, for a truck on route `pair` leaving in run hour `t_ms / 1 h`
    /// (A86): a Poisson number of incidents at `incident_rate_per_pair_h`, each a lognormal
    /// delay with median `incident_delay_median_s` and log-spread `incident_delay_sigma`. Each
    /// (pair, hour) has its own seeded draw, so the answer does not depend on query order.
    pub fn incident_delay_s(&self, master: u64, pair: usize, t_ms: u64) -> f64 {
        if self.incident_rate_per_pair_h <= 0.0 {
            return 0.0;
        }
        let hour = t_ms / MS_PER_HOUR;
        let pair = u64::try_from(pair).expect("pair index fits u64");
        let mut rng = Xoshiro256::from_seed(
            master ^ ((StreamKind::Traffic as u64) << 56) ^ pair ^ (hour << HOUR_SEED_SHIFT),
        );
        // Poisson by multiplying uniforms (Knuth); the rate is small, so few draws.
        let limit = (-self.incident_rate_per_pair_h).exp();
        let mut product = rng.uniform();
        let mut count = 0;
        while product > limit && count < MAX_INCIDENTS_PER_PAIR_HOUR {
            count += 1;
            product *= rng.uniform();
        }
        (0..count)
            .map(|_| {
                self.incident_delay_median_s * (self.incident_delay_sigma * rng.normal()).exp()
            })
            .sum()
    }
}
