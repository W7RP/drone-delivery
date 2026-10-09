//! Cold derate is per lane (spec 2.10, A81): with usable capacity derated between two routes'
//! energy needs, the route that still fits is flown and the one that does not is blocked, not
//! every flight. With `weather.energy_route_guard` (A89) the blocked route's shipments go
//! by truck after the fallback timeout; without it they wait.

mod common;
use common::*;
use drone_sim::flight::EnergyRules;
use drone_sim::ids::PadId;
use drone_sim::output::Outputs;
use drone_sim::scenario::Scenario;
use std::collections::BTreeSet;
use std::path::Path;
use toml::{Table, Value};

const SCENARIO_DIR: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../configs/parts_logistics_pittsburgh"
);
const HOURS: usize = 24;
const TIMEOUT_S: f64 = 1800.0;
const STILL: Wx = Wx {
    wind_mps: 0.0,
    gust_mps: 0.0,
    temp_c: 15.0,
};

fn scenario_table() -> Table {
    std::fs::read_to_string(Path::new(SCENARIO_DIR).join("scenario.toml"))
        .expect("scenario")
        .parse()
        .expect("toml")
}

fn load(t: &Table) -> Scenario {
    let text = toml::to_string(t).expect("serialise");
    Scenario::from_toml_in(&text, Some(Path::new(SCENARIO_DIR)))
        .unwrap_or_else(|e| panic!("rejected: {e}"))
}

/// Energy need of each flow's route over usable capacity at the charge target, still air.
fn flow_need_ratios() -> Vec<((PadId, PadId), f64)> {
    let s = load(&scenario_table());
    let rules = EnergyRules::from_config(&s.cfg);
    let usable = rules.usable_j(s.cfg.drone.charge_target_soc);
    let mut out: Vec<_> = s
        .cfg
        .flows
        .iter()
        .filter(|f| f.rate_per_s > 0.0)
        .map(|f| {
            let need = rules.dispatch_need_j(
                &s.net,
                f.origin,
                f.dest,
                s.cfg.pod_gross_mass_kg,
                |_| true,
                |q| s.cfg.has_charger(q),
            );
            ((f.origin, f.dest), need / usable)
        })
        .collect();
    out.sort_by(|a, b| a.1.total_cmp(&b.1));
    out
}

/// A derate inside the widest gap between the flows' needs, and the routes it blocks.
fn split() -> (f64, BTreeSet<(PadId, PadId)>) {
    let needs = flow_need_ratios();
    let gap = (1..needs.len())
        .max_by(|&i, &j| (needs[i].1 - needs[i - 1].1).total_cmp(&(needs[j].1 - needs[j - 1].1)))
        .expect("at least two flows");
    let factor = 0.5 * (needs[gap - 1].1 + needs[gap].1);
    let blocked = needs[gap..].iter().map(|n| n.0).collect();
    (factor, blocked)
}

/// The Pittsburgh scenario for one day in still air at a flat derate, schema 1.9.0, derived
/// latch (R19), M11 outcomes on.
fn cold_day(factor: f64, hold_policy: &str, guard: bool) -> (Table, std::path::PathBuf) {
    let dir = series_dir(&format!("cold_{hold_policy}_{guard}"), &[STILL; HOURS]);
    let mut t = scenario_table();
    set(&mut t, "schema_version", "1.9.0");
    set_q(&mut t, "sim.duration_s", 86_400.0);
    let Some(Value::Table(drone)) = t.get_mut("drone") else {
        panic!("drone section")
    };
    drone.remove("p_latch_fault");
    drone.insert("latch_capture_radius_m".into(), quantity(0.5, "m"));
    drone.insert("out_landing_error_bias_m".into(), quantity(0.0, "m"));
    drone.insert("out_landing_error_sigma_m".into(), quantity(0.01, "m"));
    let Some(Value::Table(m)) = t.get_mut("metrics") else {
        panic!("metrics section")
    };
    m.insert("order_outcomes".into(), true.into());
    let series = dir.join("weather.bin");
    let mut w = weather_section(series.to_str().expect("utf8"), hold_policy, TIMEOUT_S);
    // The Phase 5 audit's rotors; the fitted cruise power lies above their ideal (R20).
    w.insert("rotor_disc_area_m2".into(), quantity(0.24488, "m2"));
    let flat = Value::Array(vec![quantity(factor, "1"), quantity(factor, "1")]);
    w.insert("battery_derate_factor".into(), flat);
    w.insert("energy_route_guard".into(), guard.into());
    t.insert("weather".into(), Value::Table(w));
    (t, dir)
}

struct Day {
    outcomes: serde_json::Value,
    /// (origin, dest, status) of every row of deliveries.csv.
    rows: Vec<(PadId, PadId, String)>,
}

fn run_day(t: &Table) -> Day {
    let s = load(t).with_seed(4);
    let hash = s.cfg.source_hash;
    let mut w = s.into_world(0, Outputs::in_memory(hash));
    let summary = w.run().expect("runs");
    let csv = String::from_utf8(w.out.deliveries.bytes().to_vec()).expect("utf8");
    let rows = csv
        .lines()
        .skip(1)
        .map(|l| {
            let f: Vec<&str> = l.split(',').collect();
            (
                f[4].parse().expect("origin"),
                f[5].parse().expect("dest"),
                f[2].to_owned(),
            )
        })
        .collect();
    Day {
        outcomes: summary["order_outcomes"].clone(),
        rows,
    }
}

fn count(d: &Day, status: &str, on: impl Fn(&(PadId, PadId)) -> bool) -> usize {
    d.rows
        .iter()
        .filter(|(a, b, s)| s == status && on(&(*a, *b)))
        .count()
}

#[test]
fn with_the_guard_cold_blocks_only_the_routes_it_makes_too_long() {
    let (factor, blocked) = split();
    let is_blocked = |r: &(PadId, PadId)| blocked.contains(r);
    let (t, _dir) = cold_day(factor, "WAIT", true);
    let d = run_day(&t);
    assert_eq!(
        count(&d, "DELIVERED", is_blocked),
        0,
        "a blocked route flew"
    );
    assert!(
        count(&d, "DELIVERED", |r| !is_blocked(r)) > 0,
        "the cold must not stop every flight"
    );
    assert!(d.outcomes["blocked_energy"].as_u64().expect("count") > 0);
}

#[test]
fn without_the_guard_blocked_routes_starve_the_others() {
    // The energy check is per lane, but drones latch pods for blocked routes and strand, and
    // those pods fill the OUT spots ahead of the flyable shipments (Phase 7 finding).
    let (factor, blocked) = split();
    let (t, _dir) = cold_day(factor, "WAIT", false);
    let d = run_day(&t);
    assert_eq!(count(&d, "DELIVERED", |r| !blocked.contains(r)), 0);
}

#[test]
fn energy_blocked_orders_go_by_truck_only_with_the_guard() {
    let (factor, blocked) = split();
    let is_blocked = |r: &(PadId, PadId)| blocked.contains(r);
    let (t, _dir) = cold_day(factor, "TRUCK_FALLBACK", true);
    let on = run_day(&t);
    assert!(
        count(&on, "TRUCKED", is_blocked) > 0,
        "blocked shipments go by truck"
    );
    assert_eq!(
        count(&on, "TRUCKED", |r| !is_blocked(r)),
        0,
        "flyable ones do not"
    );
    assert!(
        count(&on, "DELIVERED", |r| !is_blocked(r)) > 0,
        "and the rest still fly"
    );
    let (t, _dir) = cold_day(factor, "TRUCK_FALLBACK", false);
    let off = run_day(&t);
    assert_eq!(
        count(&off, "TRUCKED", |_| true),
        0,
        "weather fallback alone sees no block"
    );
}
