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

const SECONDS_PER_DAY: i64 = 86_400;
const SECONDS_PER_HOUR: i64 = 3_600;
/// US Eastern offsets from UTC (America/New_York), s.
const EASTERN_STANDARD_S: i64 = -5 * SECONDS_PER_HOUR;
const EASTERN_DAYLIGHT_S: i64 = -4 * SECONDS_PER_HOUR;
/// Local clock time of both daylight-saving changes, 02:00.
const DST_CHANGE_LOCAL_S: i64 = 2 * SECONDS_PER_HOUR;
/// 1970-01-01 was a Thursday: day 0 of the epoch is weekday 3 with Monday 0.
const EPOCH_WEEKDAY: i64 = 3;
const MARCH: u32 = 3;
const NOVEMBER: u32 = 11;

/// Days since 1970-01-01 of a proleptic Gregorian date (H. Hinnant's `days_from_civil`).
fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let m = i64::from(m);
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + i64::from(d) - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// The year of a day since 1970-01-01.
fn year_of(days: i64) -> i64 {
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    yoe + era * 400 + i64::from(mp >= 10)
}

pub fn weekday(days: i64) -> u64 {
    u64::try_from((days + EPOCH_WEEKDAY).rem_euclid(7)).expect("0..7")
}

/// Day of month of the n-th Sunday (1-based) of a month.
fn nth_sunday(y: i64, m: u32, n: u32) -> u32 {
    let first = days_from_civil(y, m, 1);
    let to_sunday = (6 - weekday(first)) % 7;
    1 + u32::try_from(to_sunday).expect("< 7") + 7 * (n - 1)
}

/// US Eastern local time (A90) of a UTC instant, both in s since 1970-01-01 (local as if UTC):
/// UTC-5, and UTC-4 from 02:00 local on the second Sunday of March to 02:00 local on the
/// first Sunday of November (the rules since 2007).
pub fn eastern_local_s(unix_s: i64) -> i64 {
    let y = year_of((unix_s + EASTERN_STANDARD_S).div_euclid(SECONDS_PER_DAY));
    let start = days_from_civil(y, MARCH, nth_sunday(y, MARCH, 2)) * SECONDS_PER_DAY
        + DST_CHANGE_LOCAL_S
        - EASTERN_STANDARD_S;
    let end = days_from_civil(y, NOVEMBER, nth_sunday(y, NOVEMBER, 1)) * SECONDS_PER_DAY
        + DST_CHANGE_LOCAL_S
        - EASTERN_DAYLIGHT_S;
    let offset = if (start..end).contains(&unix_s) {
        EASTERN_DAYLIGHT_S
    } else {
        EASTERN_STANDARD_S
    };
    unix_s + offset
}

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
    /// `traffic.start_unix_s`: the UTC instant of t = 0. With it, traffic reads US Eastern
    /// local time with daylight saving (A90); without it, `sim.start_time_of_day_s` and
    /// `start_weekday` at a fixed offset.
    pub start_unix_s: Option<i64>,
}

impl TrafficCfg {
    /// (local hour 0..24, weekend) of a truck leaving at `t_dep_ms`, when `start_unix_s` is
    /// set.
    pub fn local_clock(&self, t_dep_ms: u64) -> Option<(usize, bool)> {
        let start = self.start_unix_s?;
        let t = start + i64::try_from(t_dep_ms / 1000).expect("run time fits i64");
        let local = eastern_local_s(t);
        let hour =
            usize::try_from(local.rem_euclid(SECONDS_PER_DAY) / SECONDS_PER_HOUR).expect("0..24");
        let weekend = weekday(local.div_euclid(SECONDS_PER_DAY)) >= FIRST_WEEKEND_DAY;
        Some((hour, weekend))
    }

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eastern_local_time_matches_the_tz_database() {
        // (UTC, local as if UTC) from Python zoneinfo America/New_York.
        for (utc, local) in [
            (1_710_053_999, 1_710_035_999), // 2024-03-10 01:59:59 EST
            (1_710_054_000, 1_710_039_600), // 03:00:00 EDT, spring forward
            (1_730_613_599, 1_730_599_199), // 2024-11-03 01:59:59 EDT
            (1_730_613_600, 1_730_595_600), // 01:00:00 EST, fall back
            (1_688_212_800, 1_688_198_400), // 2023-07-01 08:00 EDT
            (1_736_942_400, 1_736_924_400), // 2025-01-15 07:00 EST
        ] {
            assert_eq!(eastern_local_s(utc), local, "UTC {utc}");
        }
    }

    #[test]
    fn weekdays_count_from_a_thursday_epoch() {
        assert_eq!(weekday(0), 3);
        // 2024-03-04, a Monday.
        assert_eq!(weekday(days_from_civil(2024, 3, 4)), 0);
        assert_eq!(year_of(days_from_civil(2024, 12, 31)), 2024);
        assert_eq!(year_of(days_from_civil(2025, 1, 1)), 2025);
    }
}
