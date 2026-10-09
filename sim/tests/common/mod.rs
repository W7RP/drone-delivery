//! Shared helpers: build test scenarios from the A-B example, run them in memory, and read the
//! transition trace from the event log.

#![allow(dead_code)]

use drone_sim::event::tcode;
use drone_sim::output::Outputs;
use drone_sim::scenario::Scenario;
use drone_sim::trace::{self, Record, Transition};
use std::path::PathBuf;
use toml::{Table, Value};

pub const EXAMPLE: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../configs/ab_synthetic/scenario.toml"
);

pub fn example() -> Table {
    std::fs::read_to_string(EXAMPLE)
        .expect("example config readable")
        .parse()
        .expect("example config is TOML")
}

fn walk<'a>(t: &'a mut Table, path: &str) -> &'a mut Value {
    let mut parts = path.split('.');
    let first = parts.next().expect("non-empty path");
    let mut cur = t.get_mut(first).unwrap_or_else(|| panic!("no key {first}"));
    for p in parts {
        cur = match cur {
            Value::Table(tt) => tt
                .get_mut(p)
                .unwrap_or_else(|| panic!("no key {p} in {path}")),
            Value::Array(a) => &mut a[p.parse::<usize>().expect("array index")],
            _ => panic!("cannot descend into {p} of {path}"),
        };
    }
    cur
}

/// Set the `value` of a quantity at a dotted path (`pads.1.parking_capacity`).
pub fn set_q(t: &mut Table, path: &str, value: impl Into<Value>) {
    let Value::Table(q) = walk(t, path) else {
        panic!("{path} is not a quantity")
    };
    q.insert("value".into(), value.into());
}

/// Replace any value at a dotted path.
pub fn set(t: &mut Table, path: &str, value: impl Into<Value>) {
    *walk(t, path) = value.into();
}

pub fn quantity(value: impl Into<Value>, unit: &str) -> Value {
    let mut q = Table::new();
    q.insert("value".into(), value.into());
    q.insert("unit".into(), unit.into());
    q.insert("assumption".into(), true.into());
    Value::Table(q)
}

pub fn fault(kind: &str, pad: &str, start_s: f64, duration_s: f64) -> Value {
    let mut f = Table::new();
    f.insert("kind".into(), kind.into());
    f.insert("pad".into(), pad.into());
    f.insert("start_s".into(), quantity(start_s, "s"));
    f.insert("duration_s".into(), quantity(duration_s, "s"));
    Value::Table(f)
}

fn flow(origin: &str, dest: &str, rate_per_h: f64) -> Value {
    let mut f = Table::new();
    f.insert("origin".into(), origin.into());
    f.insert("destination".into(), dest.into());
    f.insert("rate_per_h".into(), quantity(rate_per_h, "1/h"));
    Value::Table(f)
}

/// One drone parked in A's parking area, demand A -> B only, no random faults, no message
/// loss, perfect marker detection: every branch test starts from this and forces one fault.
pub fn base() -> Table {
    let mut t = example();
    set_q(&mut t, "sim.duration_s", 10_800.0);
    set_q(&mut t, "sim.warmup_s", 0.0);
    set_q(&mut t, "drone.fleet_size", 1);
    set(&mut t, "drone.initial_pads", Value::Array(vec!["A".into()]));
    set_q(&mut t, "pads.0.parking_capacity", 2);
    set_q(&mut t, "pads.1.parking_capacity", 2);
    set(&mut t, "faults", Value::Array(vec![]));
    set(
        &mut t,
        "demand.flows",
        Value::Array(vec![flow("A", "B", 6.0)]),
    );
    set_q(&mut t, "drone.p_release_fault", 0.0);
    set_q(&mut t, "drone.p_latch_fault", 0.0);
    set_q(&mut t, "pad_ops.p_marker_detect", 1.0);
    set_q(&mut t, "pad_ops.pc_fault_rate_per_h", 0.0);
    set_q(&mut t, "handshake.p_msg_loss", 0.0);
    t
}

pub fn add_fault(t: &mut Table, f: Value) {
    let Value::Array(a) = walk(t, "faults") else {
        panic!("faults is not an array")
    };
    a.push(f);
}

pub struct Run {
    pub summary: serde_json::Value,
    pub records: Vec<Record>,
    pub events_bin: Vec<u8>,
    pub flights_bin: Vec<u8>,
    pub alerts: String,
    pub deliveries: String,
}

pub fn run_text(text: &str, seed: Option<u64>) -> Run {
    let scenario = Scenario::from_toml(text).unwrap_or_else(|e| panic!("config rejected: {e}"));
    let scenario = match seed {
        Some(s) => scenario.with_seed(s),
        None => scenario,
    };
    let hash = scenario.cfg.source_hash;
    let mut world = scenario.into_world(0, Outputs::in_memory(hash));
    let summary = world.run().expect("in-memory outputs never fail");
    let events_bin = world.out.events.bytes().to_vec();
    Run {
        summary,
        records: trace::decode(&events_bin).expect("valid events.bin"),
        flights_bin: world.out.flights.bytes().to_vec(),
        alerts: String::from_utf8(world.out.alerts.bytes().to_vec()).expect("utf8"),
        deliveries: String::from_utf8(world.out.deliveries.bytes().to_vec()).expect("utf8"),
        events_bin,
    }
}

pub fn run(t: &Table) -> Run {
    run_text(&toml::to_string(t).expect("serialise config"), None)
}

pub fn transitions(r: &Run) -> Vec<Transition> {
    r.records.iter().filter_map(Record::transition).collect()
}

/// Transition labels of one drone (its visits' V rows and its own D rows), in order.
pub fn drone_labels(r: &Run, drone: u16) -> Vec<String> {
    transitions(r)
        .iter()
        .filter(|t| t.drone == drone)
        .map(|t| tcode::label(t.code))
        .collect()
}

/// Labels of one drone at one pad (V rows of that pad's visits, D rows while there).
pub fn drone_pad_labels(r: &Run, drone: u16, pad: u16) -> Vec<String> {
    transitions(r)
        .iter()
        .filter(|t| t.drone == drone && t.pad == pad)
        .map(|t| tcode::label(t.code))
        .collect()
}

/// The labels from the first occurrence of `start` (inclusive), `n` long.
pub fn window(labels: &[String], start: &str, n: usize) -> Vec<String> {
    let i = labels
        .iter()
        .position(|l| l == start)
        .unwrap_or_else(|| panic!("{start} not found in {labels:?}"));
    labels[i..(i + n).min(labels.len())].to_vec()
}

pub fn strs(v: &[&str]) -> Vec<String> {
    v.iter().map(|s| (*s).to_owned()).collect()
}

pub fn first_time(r: &Run, label: &str, drone: u16) -> u64 {
    transitions(r)
        .iter()
        .find(|t| tcode::label(t.code) == label && t.drone == drone)
        .map(|t| t.t_ms)
        .unwrap_or_else(|| panic!("{label} for drone {drone} not found"))
}

/// An alert with this code was written to OPS.
pub fn has_alert(r: &Run, code: &str) -> bool {
    r.alerts
        .lines()
        .any(|l| l.contains(&format!("\"code\":\"{code}\"")))
}

pub fn delivery_rows(r: &Run) -> Vec<Vec<String>> {
    r.deliveries
        .lines()
        .skip(1)
        .map(|l| l.split(',').map(str::to_owned).collect())
        .collect()
}

/// Weather limits of the forced weather tests: sustained wind and gust at 10 m, m/s.
pub const MAX_WIND_MPS: f64 = 10.0;
pub const MAX_GUST_MPS: f64 = 15.0;

/// One hour of a test series; every field in SI.
#[derive(Clone, Copy)]
pub struct Wx {
    pub wind_mps: f32,
    pub gust_mps: f32,
    pub temp_c: f32,
}

pub const CALM: Wx = Wx {
    wind_mps: 2.0,
    gust_mps: 3.0,
    temp_c: 15.0,
};
pub const GALE: Wx = Wx {
    wind_mps: 20.0,
    gust_mps: 25.0,
    temp_c: 15.0,
};
pub const GUSTY: Wx = Wx {
    wind_mps: 5.0,
    gust_mps: 25.0,
    temp_c: 15.0,
};

/// `weather.bin` (spec 9.7) for these hours: wind from the west, 10 km visibility, dry.
pub fn series_bytes(hours: &[Wx]) -> Vec<u8> {
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
pub fn series_dir(name: &str, hours: &[Wx]) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("dd_weather_{name}_{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    std::fs::write(dir.join("weather.bin"), series_bytes(hours)).expect("write series");
    dir
}

/// A `[weather]` section (spec 2.10) with placeholder values that change nothing in calm,
/// warm, clear air, except a 0.7 capacity derate at -20 deg C.
pub fn weather_section(series_file: &str, hold_policy: &str, timeout_s: f64) -> Table {
    let mut w = Table::new();
    w.insert("series_file".into(), series_file.into());
    w.insert("start_hour".into(), quantity(0, "count"));
    w.insert("wind_shear_exponent".into(), quantity(0.2, "1"));
    w.insert("cruise_height_m".into(), quantity(60.0, "m"));
    w.insert("max_airspeed_mps".into(), quantity(25.0, "m/s"));
    // Large enough that the AB example's 1000 W cruise lies above the ideal induced power (R20).
    w.insert("rotor_disc_area_m2".into(), quantity(1.0, "m2"));
    w.insert("air_density_kgpm3".into(), quantity(1.225, "kg/m3"));
    w.insert("max_wind_mps".into(), quantity(MAX_WIND_MPS, "m/s"));
    w.insert("max_gust_mps".into(), quantity(MAX_GUST_MPS, "m/s"));
    w.insert("min_visibility_m".into(), quantity(1000.0, "m"));
    w.insert("max_precip_mm_h".into(), quantity(5.0, "mm/h"));
    w.insert("min_temp_c".into(), quantity(-30.0, "degC"));
    w.insert("max_temp_c".into(), quantity(45.0, "degC"));
    w.insert("landing_max_wind_mps".into(), quantity(MAX_WIND_MPS, "m/s"));
    w.insert("landing_max_gust_mps".into(), quantity(MAX_GUST_MPS, "m/s"));
    w.insert("latch_gust_k".into(), quantity(0.0, "1"));
    w.insert("latch_gust_ref_mps".into(), quantity(10.0, "m/s"));
    w.insert("marker_penalty_rain".into(), quantity(0.0, "1"));
    w.insert("marker_penalty_snow".into(), quantity(0.0, "1"));
    w.insert("marker_penalty_low_visibility".into(), quantity(0.0, "1"));
    w.insert("low_visibility_m".into(), quantity(1600.0, "m"));
    w.insert(
        "battery_derate_temp_c".into(),
        Value::Array(vec![quantity(-20.0, "degC"), quantity(20.0, "degC")]),
    );
    w.insert(
        "battery_derate_factor".into(),
        Value::Array(vec![quantity(0.7, "1"), quantity(1.0, "1")]),
    );
    w.insert("truck_factor_rain".into(), quantity(1.0, "1"));
    w.insert("truck_factor_snow".into(), quantity(1.0, "1"));
    w.insert("truck_factor_low_visibility".into(), quantity(1.0, "1"));
    w.insert("hold_policy".into(), hold_policy.into());
    w.insert("truck_fallback_timeout_s".into(), quantity(timeout_s, "s"));
    w
}
