//! Shared helpers: build test scenarios from the A-B example, run them in memory, and read the
//! transition trace from the event log.

#![allow(dead_code)]

use drone_sim::event::tcode;
use drone_sim::output::Outputs;
use drone_sim::scenario::Scenario;
use drone_sim::trace::{self, Record, Transition};
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

pub fn delivery_rows(r: &Run) -> Vec<Vec<String>> {
    r.deliveries
        .lines()
        .skip(1)
        .map(|l| l.split(',').map(str::to_owned).collect())
        .collect()
}
