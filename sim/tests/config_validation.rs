//! Spec 8: the loader checks every key, unit and range, and rules R1 to R17, and rejects bad
//! configs with an error that names the problem.

mod common;
use common::*;
use drone_sim::scenario::Scenario;
use toml::Value;

fn rejected(t: &toml::Table) -> String {
    match Scenario::from_toml(&toml::to_string(t).expect("serialise")) {
        Ok(_) => panic!("config was accepted"),
        Err(e) => e.0,
    }
}

fn assert_rejected(t: &toml::Table, expect: &str) {
    let msg = rejected(t);
    assert!(
        msg.contains(expect),
        "error {msg:?} does not mention {expect:?}"
    );
}

#[test]
fn example_config_loads_and_matches_its_directory() {
    let s = Scenario::load(std::path::Path::new(EXAMPLE)).expect("example config is valid");
    assert_eq!(s.cfg.pads.len(), 2);
    assert_eq!(
        s.net.lane_to.len(),
        2,
        "one config lane becomes two directed lanes"
    );
    assert!(
        (s.net.route_len(0, 1) - 6000.133).abs() < 0.01,
        "6000.133 m as in docs/paper-run.md"
    );
}

#[test]
fn wrong_unit_is_rejected_not_converted() {
    let mut t = example();
    let Value::Table(q) = t["drone"].as_table_mut().unwrap().get_mut("hop_s").unwrap() else {
        unreachable!()
    };
    q.insert("unit".into(), "ms".into());
    assert_rejected(&t, "drone.hop_s: unit \"ms\", expected \"s\"");
}

#[test]
fn unknown_and_missing_keys_are_rejected() {
    let mut t = example();
    t["drone"]
        .as_table_mut()
        .unwrap()
        .insert("max_speed".into(), quantity(1.0, "m/s"));
    assert_rejected(&t, "drone.max_speed: unknown key");
    let mut t = example();
    t["drone"].as_table_mut().unwrap().remove("takeoff_s");
    assert_rejected(&t, "drone.takeoff_s: missing");
}

#[test]
fn out_of_range_values_are_rejected() {
    let mut t = example();
    set_q(&mut t, "pad_ops.p_marker_detect", 1.5);
    assert_rejected(&t, "pad_ops.p_marker_detect");
    let mut t = example();
    set_q(&mut t, "drone.cruise_speed_mps", 0.0);
    assert_rejected(&t, "drone.cruise_speed_mps");
    let mut t = example();
    set_q(&mut t, "drone.fleet_size", 1.5);
    assert_rejected(&t, "expected an integer count");
}

#[test]
fn stub_and_wrong_version_are_rejected() {
    let stub = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../configs/parts_logistics/scenario.toml"
    ))
    .unwrap();
    let e = Scenario::from_toml(&stub).err().unwrap().0;
    assert!(e.contains("STUB"), "{e}");
    let mut t = example();
    set(&mut t, "schema_version", "1.0.0");
    assert_rejected(&t, "schema_version");
}

#[test]
fn cross_field_rules_are_enforced() {
    let mut t = example();
    set(
        &mut t,
        "drone.initial_pads",
        Value::Array(vec!["A".into(), "A".into()]),
    );
    assert_rejected(&t, "R4");

    let mut t = example();
    set_q(&mut t, "drone.fleet_size", 3);
    set(
        &mut t,
        "drone.initial_pads",
        Value::Array(vec!["A".into(), "B".into(), "B".into()]),
    );
    assert_rejected(&t, "R4");

    let mut t = example();
    set_q(&mut t, "drone.fleet_size", 3);
    t["drone"].as_table_mut().unwrap().remove("initial_pads");
    assert_rejected(&t, "R4");

    let mut t = example();
    set_q(&mut t, "payload.pod_gross_mass_kg", 7.0);
    assert_rejected(&t, "R6");

    let mut t = example();
    set_q(&mut t, "drone.reserve_soc", 0.96);
    assert_rejected(&t, "R7");

    let mut t = example();
    set_q(&mut t, "sim.warmup_s", 30_000.0);
    assert_rejected(&t, "R8");

    let mut t = example();
    add_fault(&mut t, fault("PC_FAULT", "B", 7500.0, 100.0));
    assert_rejected(&t, "R10");

    let mut t = example();
    set(&mut t, "charging.mode", "HUB_ONLY");
    assert_rejected(&t, "R11");

    let mut t = example();
    set_q(&mut t, "drone.battery_capacity_j", 2_000_000.0);
    assert_rejected(&t, "R12");

    let mut t = example();
    set(&mut t, "demand.flows.0.origin", "C");
    assert_rejected(&t, "no pad named \"C\"");
}

#[test]
fn parking_slots_allow_a_larger_fleet() {
    let mut t = example();
    set_q(&mut t, "pads.0.parking_capacity", 3);
    set_q(&mut t, "drone.fleet_size", 4);
    t["drone"].as_table_mut().unwrap().remove("initial_pads");
    let s = Scenario::from_toml(&toml::to_string(&t).unwrap()).expect("4 drones fit 3 + 1 slots");
    assert_eq!(
        s.cfg.drone.initial_pads,
        vec![0, 1, 0, 0],
        "round-robin skips full pads"
    );
}

#[test]
fn network_and_demand_forms_are_exclusive() {
    let mut t = example();
    let mut net = toml::Table::new();
    net.insert("topology".into(), "MESH".into());
    t.insert("synthetic_network".into(), Value::Table(net));
    assert_rejected(&t, "exactly one of");

    let mut t = example();
    t["demand"]
        .as_table_mut()
        .unwrap()
        .insert("rate_per_pad_per_h".into(), quantity(1.0, "1/h"));
    assert_rejected(&t, "demand: give either flows");
}

#[test]
fn file_truck_matrix_needs_network_files_and_no_synthetic_keys() {
    let mut t = example();
    set(&mut t, "truck.matrix_source", "FILE");
    assert_rejected(
        &t,
        "synthetic_detour_factor: only used with matrix_source = SYNTHETIC",
    );
    let truck = t["truck"].as_table_mut().unwrap();
    truck.remove("synthetic_detour_factor");
    truck.remove("synthetic_free_flow_speed_mps");
    assert_rejected(&t, "R17");
}
