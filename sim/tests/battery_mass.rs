//! Spec A68: a drone's empty mass can follow its battery (airframe + capacity / specific
//! energy). The coupled form must reproduce the fixed-mass example exactly at its 900 Wh, and a
//! bigger battery must cost mass and so energy.

mod common;
use common::*;
use drone_sim::scenario::Scenario;
use toml::{Table, Value};

const J_PER_WH: f64 = 3600.0;
/// Placeholders of the coupled scenarios: 4.5 kg + 900 Wh / 200 Wh/kg = the example's 9.0 kg.
const AIRFRAME_KG: f64 = 4.5;
const SPECIFIC_ENERGY_WH_PER_KG: f64 = 200.0;

fn coupled(mut t: Table) -> Table {
    set(&mut t, "schema_version", "1.6.0");
    let Value::Table(drone) = t.get_mut("drone").expect("drone section") else {
        panic!("drone is a table")
    };
    drone.remove("empty_mass_kg");
    drone.insert("airframe_mass_kg".into(), quantity(AIRFRAME_KG, "kg"));
    drone.insert(
        "battery_specific_energy_wh_per_kg".into(),
        quantity(SPECIFIC_ENERGY_WH_PER_KG, "Wh/kg"),
    );
    t
}

fn load(t: &Table) -> Result<Scenario, String> {
    Scenario::from_toml(&toml::to_string(t).expect("serialise")).map_err(|e| e.0)
}

fn rejection(t: &Table) -> String {
    match load(t) {
        Ok(_) => panic!("config was accepted"),
        Err(e) => e,
    }
}

#[test]
fn coupled_mass_at_900_wh_reproduces_the_fixed_mass_run() {
    let fixed = example();
    let both = coupled(example());
    let s = load(&both).expect("coupled example loads");
    assert_eq!(
        s.cfg.drone.empty_mass_kg, 9.0,
        "4.5 kg + 900 Wh / 200 Wh/kg"
    );
    let (a, b) = (run(&fixed), run(&both));
    assert_eq!(a.deliveries, b.deliveries);
    assert_eq!(a.alerts, b.alerts);
    // The 32-byte header carries the config hash; the records must match.
    assert_eq!(a.flights_bin[32..], b.flights_bin[32..]);
    let strip = |mut v: serde_json::Value| {
        v["source_hash"] = serde_json::Value::Null;
        v
    };
    assert_eq!(strip(a.summary), strip(b.summary));
}

#[test]
fn a_bigger_battery_is_heavier_and_hovers_at_more_power() {
    let mut t = coupled(example());
    let wh = 1000.0;
    set_q(&mut t, "drone.battery_capacity_j", wh * J_PER_WH);
    let s = load(&t).expect("coupled 1000 Wh loads");
    let expected = AIRFRAME_KG + wh / SPECIFIC_ENERGY_WH_PER_KG;
    assert!((s.cfg.drone.empty_mass_kg - expected).abs() < 1e-9);
    let fixed = load(&example()).expect("example loads");
    let hover =
        |s: &Scenario| drone_sim::flight::FlightModel::from_config(&s.cfg).hover_power_w(5.0);
    // (9.5 + 0.6 + 5) / (9.0 + 0.6 + 5) to the power 1.5.
    let ratio = hover(&s) / hover(&fixed);
    assert!(
        (ratio - (15.1f64 / 14.6).powf(1.5)).abs() < 1e-9,
        "ratio {ratio}"
    );
}

#[test]
fn mass_is_given_one_way_only() {
    let mut both = coupled(example());
    let Value::Table(drone) = both.get_mut("drone").expect("drone") else {
        unreachable!()
    };
    drone.insert("empty_mass_kg".into(), quantity(9.0, "kg"));
    assert!(rejection(&both).contains("give empty_mass_kg, or airframe_mass_kg"));

    let mut partial = coupled(example());
    let Value::Table(drone) = partial.get_mut("drone").expect("drone") else {
        unreachable!()
    };
    drone.remove("battery_specific_energy_wh_per_kg");
    assert!(rejection(&partial).contains("battery_specific_energy_wh_per_kg"));

    let mut old_version = coupled(example());
    set(&mut old_version, "schema_version", "1.5.0");
    assert!(rejection(&old_version).contains("needs schema_version \"1.6.0\""));
}

#[test]
fn heavy_battery_breaks_the_payload_margin() {
    // 16 kg MTOW - 0.6 kg latch - 5 kg pod leaves 10.4 kg: 4.5 kg + 1200 Wh / 200 Wh/kg = 10.5.
    let mut t = coupled(example());
    set_q(&mut t, "drone.battery_capacity_j", 1200.0 * J_PER_WH);
    assert!(rejection(&t).contains("R6"));
}
