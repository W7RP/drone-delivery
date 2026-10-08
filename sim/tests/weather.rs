//! Weather (spec 2.10, A77 to A83), forced: a wind over the limit stops dispatch, a gust over
//! the limit holds a drone that wants to land, TRUCK_FALLBACK sends blocked shipments by truck
//! after its timeout, and cold cuts range. Each test writes its own small series (spec 9.7).

mod common;
use common::*;
use drone_sim::output::Outputs;
use drone_sim::scenario::Scenario;
use drone_sim::world::World;
use std::path::PathBuf;
use toml::{Table, Value};

const HOUR_MS: u64 = 3_600_000;
const MAX_WIND_MPS: f64 = 10.0;
const MAX_GUST_MPS: f64 = 15.0;
const TIMEOUT_S: f64 = 1800.0;
/// `pad_ops.pod_place_s` of the AB example.
const POD_PLACE_S: f64 = 60.0;

/// One hour of a test series; every field in SI.
#[derive(Clone, Copy)]
struct Wx {
    wind_mps: f32,
    gust_mps: f32,
    temp_c: f32,
}

const CALM: Wx = Wx {
    wind_mps: 2.0,
    gust_mps: 3.0,
    temp_c: 15.0,
};
const GALE: Wx = Wx {
    wind_mps: 20.0,
    gust_mps: 25.0,
    temp_c: 15.0,
};
const GUSTY: Wx = Wx {
    wind_mps: 5.0,
    gust_mps: 25.0,
    temp_c: 15.0,
};

/// `weather.bin` (spec 9.7) for these hours: wind from the west, 10 km visibility, dry.
fn series_bytes(hours: &[Wx]) -> Vec<u8> {
    let n = hours.len();
    let mut b = Vec::new();
    b.extend_from_slice(b"DDWTHR\0\0");
    b.extend_from_slice(&1u16.to_le_bytes());
    b.extend_from_slice(&0u16.to_le_bytes());
    b.extend_from_slice(&32u32.to_le_bytes());
    b.extend_from_slice(&u32::try_from(n).expect("few hours").to_le_bytes());
    b.extend_from_slice(&480_000u32.to_le_bytes());
    b.extend_from_slice(&0u64.to_le_bytes());
    let f32s = |b: &mut Vec<u8>, f: &dyn Fn(&Wx) -> f32| {
        b.resize(b.len().next_multiple_of(8), 0);
        for h in hours {
            b.extend_from_slice(&f(h).to_le_bytes());
        }
    };
    f32s(&mut b, &|h| h.wind_mps);
    f32s(&mut b, &|_| 270.0);
    f32s(&mut b, &|h| h.gust_mps);
    f32s(&mut b, &|h| h.temp_c);
    f32s(&mut b, &|_| 10_000.0);
    f32s(&mut b, &|_| 0.0);
    for _ in 0..2 {
        b.resize(b.len().next_multiple_of(8), 0);
        b.extend(std::iter::repeat_n(0u8, n));
    }
    b
}

/// A directory holding `weather.bin`, unique per test.
fn series_dir(name: &str, hours: &[Wx]) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("dd_weather_{name}_{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    std::fs::write(dir.join("weather.bin"), series_bytes(hours)).expect("write series");
    dir
}

fn q(value: impl Into<Value>, unit: &str) -> Value {
    quantity(value, unit)
}

fn quantities(values: &[f64], unit: &str) -> Value {
    Value::Array(values.iter().map(|&v| q(v, unit)).collect())
}

/// The AB base with schema 1.8.0, a derived latch (R19) that never misses in calm air, and a
/// `[weather]` section; `hours` long, demand A -> B at `rate_per_h`.
fn with_weather(hours: usize, rate_per_h: f64, hold_policy: &str) -> Table {
    let mut t = base();
    set(&mut t, "schema_version", "1.8.0");
    #[allow(clippy::cast_precision_loss)]
    set_q(&mut t, "sim.duration_s", (hours * 3600) as f64);
    set_q(&mut t, "demand.flows.0.rate_per_h", rate_per_h);
    let Some(Value::Table(drone)) = t.get_mut("drone") else {
        panic!("drone section")
    };
    drone.remove("p_latch_fault");
    drone.insert("latch_capture_radius_m".into(), q(0.5, "m"));
    drone.insert("out_landing_error_bias_m".into(), q(0.0, "m"));
    drone.insert("out_landing_error_sigma_m".into(), q(0.01, "m"));
    let mut w = Table::new();
    w.insert("series_file".into(), "weather.bin".into());
    w.insert("start_hour".into(), q(0, "count"));
    w.insert("wind_shear_exponent".into(), q(0.2, "1"));
    w.insert("cruise_height_m".into(), q(60.0, "m"));
    w.insert("max_airspeed_mps".into(), q(25.0, "m/s"));
    // Large enough that the AB example's 1000 W cruise lies above the ideal induced power (R20).
    w.insert("rotor_disc_area_m2".into(), q(1.0, "m2"));
    w.insert("air_density_kgpm3".into(), q(1.225, "kg/m3"));
    w.insert("max_wind_mps".into(), q(MAX_WIND_MPS, "m/s"));
    w.insert("max_gust_mps".into(), q(MAX_GUST_MPS, "m/s"));
    w.insert("min_visibility_m".into(), q(1000.0, "m"));
    w.insert("max_precip_mm_h".into(), q(5.0, "mm/h"));
    w.insert("min_temp_c".into(), q(-30.0, "degC"));
    w.insert("max_temp_c".into(), q(45.0, "degC"));
    w.insert("landing_max_wind_mps".into(), q(MAX_WIND_MPS, "m/s"));
    w.insert("landing_max_gust_mps".into(), q(MAX_GUST_MPS, "m/s"));
    w.insert("latch_gust_k".into(), q(0.0, "1"));
    w.insert("latch_gust_ref_mps".into(), q(10.0, "m/s"));
    w.insert("marker_penalty_rain".into(), q(0.0, "1"));
    w.insert("marker_penalty_snow".into(), q(0.0, "1"));
    w.insert("marker_penalty_low_visibility".into(), q(0.0, "1"));
    w.insert("low_visibility_m".into(), q(1600.0, "m"));
    w.insert(
        "battery_derate_temp_c".into(),
        quantities(&[-20.0, 20.0], "degC"),
    );
    w.insert("battery_derate_factor".into(), quantities(&[0.7, 1.0], "1"));
    w.insert("truck_factor_rain".into(), q(1.0, "1"));
    w.insert("truck_factor_snow".into(), q(1.0, "1"));
    w.insert("truck_factor_low_visibility".into(), q(1.0, "1"));
    w.insert("hold_policy".into(), hold_policy.into());
    w.insert("truck_fallback_timeout_s".into(), q(TIMEOUT_S, "s"));
    t.insert("weather".into(), Value::Table(w));
    t
}

fn world(t: &Table, dir: &std::path::Path) -> World {
    let text = toml::to_string(t).expect("serialise");
    let s = Scenario::from_toml_in(&text, Some(dir)).unwrap_or_else(|e| panic!("rejected: {e}"));
    let hash = s.cfg.source_hash;
    s.into_world(0, Outputs::in_memory(hash))
}

struct Outcome {
    summary: serde_json::Value,
    deliveries: String,
    lanes_flown: usize,
    grants_ms: Vec<u64>,
    holds_ms: Vec<u64>,
}

fn run_in(t: &Table, dir: &std::path::Path) -> Outcome {
    let mut w = world(t, dir);
    let summary = w.run().expect("runs");
    let records = drone_sim::trace::decode(w.out.events.bytes()).expect("valid events.bin");
    // Landing grants: V01 (granted on request) and V04 (granted from the queue).
    let at = |codes: &[u16]| -> Vec<u64> {
        records
            .iter()
            .filter_map(|r| r.transition().map(|t| (r.t_ms, t.code)))
            .filter(|(_, code)| codes.contains(code))
            .map(|(t, _)| t)
            .collect()
    };
    // Landing grants: V01 (on request) and V04 (from the queue); D07: the drone holds.
    let grants_ms = at(&[301, 304]);
    let holds_ms = at(&[407]);
    Outcome {
        summary,
        deliveries: String::from_utf8(w.out.deliveries.bytes().to_vec()).expect("utf8"),
        lanes_flown: w.out.flights.bytes().len() / 48,
        grants_ms,
        holds_ms,
    }
}

#[test]
fn wind_over_the_limit_dispatches_nothing() {
    let dir = series_dir("gale", &[GALE, GALE, GALE]);
    let gale = run_in(&with_weather(3, 6.0, "WAIT"), &dir);
    assert_eq!(gale.lanes_flown, 0, "no lane may be flown in a gale");
    assert_eq!(gale.summary["deliveries_in_window"], 0);
    assert_eq!(gale.summary["weather"]["flyable_hour_fraction"], 0.0);

    let dir = series_dir("calm", &[CALM, CALM, CALM]);
    let calm = run_in(&with_weather(3, 6.0, "WAIT"), &dir);
    assert!(calm.lanes_flown > 0, "the control must fly");
    assert_eq!(calm.summary["weather"]["flyable_hour_fraction"], 1.0);
}

#[test]
fn a_gust_over_the_limit_holds_drones_that_want_to_land() {
    // Calm, then an hour of gusts: a drone airborne at the change must hold, not land. (A
    // drone cannot hold for an hour; this one diverts and is forced down at its reserve.)
    let dir = series_dir("gust", &[CALM, GUSTY, CALM]);
    let r = run_in(&with_weather(3, 30.0, "WAIT"), &dir);
    let gusty = |t: &&u64| (HOUR_MS..2 * HOUR_MS).contains(*t);
    assert!(
        r.summary["weather"]["weather_aborts"]
            .as_u64()
            .expect("count")
            > 0
    );
    assert_eq!(
        r.grants_ms.iter().filter(gusty).count(),
        0,
        "a landing was cleared in gusts"
    );
    assert!(
        r.holds_ms.iter().any(|t| gusty(&t)),
        "the drone must hold in the gusty hour"
    );

    let dir = series_dir("gust_control", &[CALM, CALM, CALM]);
    let calm = run_in(&with_weather(3, 30.0, "WAIT"), &dir);
    assert_eq!(calm.summary["weather"]["weather_aborts"], 0);
}

#[test]
fn truck_fallback_sends_blocked_shipments_by_truck_after_the_timeout() {
    let dir = series_dir("fallback", &[GALE, GALE, GALE]);
    // Midnight start: the truck's time-of-day factor is 1 for every hour this run touches, so
    // a trucked order's door-to-door is exactly the timeout plus its truck baseline.
    let mut t = with_weather(3, 6.0, "TRUCK_FALLBACK");
    set_q(&mut t, "sim.start_time_of_day_s", 0.0);
    let trucked = run_in(&t, &dir);
    let n = trucked.summary["weather"]["orders_trucked"]
        .as_u64()
        .expect("count");
    assert!(n > 0, "blocked shipments must go by truck");
    for row in trucked
        .deliveries
        .lines()
        .filter(|l| l.contains(",TRUCKED,"))
    {
        let f: Vec<&str> = row.split(',').collect();
        let door_s: f64 = f[12].parse().expect("door-to-door");
        let truck_s: f64 = f[13].parse().expect("truck baseline");
        // At the timeout, or when a placement under way at the timeout finishes.
        let late_s = door_s - (TIMEOUT_S + truck_s);
        assert!(
            (-2e-3..POD_PLACE_S + 2e-3).contains(&late_s),
            "not trucked at the timeout: {row}"
        );
    }
    let wait = run_in(&with_weather(3, 6.0, "WAIT"), &dir);
    assert_eq!(wait.summary["weather"]["orders_trucked"], 0);
    assert!(!wait.deliveries.contains(",TRUCKED,"));
}

#[test]
fn cold_cuts_range() {
    let cold = Wx {
        temp_c: -20.0,
        ..CALM
    };
    let range = |hours: &[Wx], name: &str| {
        let dir = series_dir(name, hours);
        let w = world(&with_weather(1, 6.0, "WAIT"), &dir);
        w.energy.max_lane_m(0.95, 5.0)
    };
    let warm_m = range(&[CALM], "warm");
    let cold_m = range(&[cold], "cold");
    assert!(
        cold_m < 0.8 * warm_m,
        "cold {cold_m:.0} m should be well short of warm {warm_m:.0} m"
    );
}
