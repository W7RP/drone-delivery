//! Spec 5.4: same config, seed and replication give bit-identical outputs; other seeds differ.

mod common;
use common::*;

fn text() -> String {
    std::fs::read_to_string(EXAMPLE).unwrap()
}

#[test]
fn same_seed_gives_byte_identical_outputs() {
    let (a, b) = (run_text(&text(), Some(7)), run_text(&text(), Some(7)));
    assert!(
        a.events_bin.len() > 32 * 1000,
        "a real run, not an empty log"
    );
    assert_eq!(a.events_bin, b.events_bin);
    assert_eq!(a.flights_bin, b.flights_bin);
    assert_eq!(a.alerts, b.alerts);
    assert_eq!(a.deliveries, b.deliveries);
    assert_eq!(
        serde_json::to_string(&a.summary).unwrap(),
        serde_json::to_string(&b.summary).unwrap()
    );
}

#[test]
fn different_seeds_give_different_runs() {
    let (a, b) = (run_text(&text(), Some(7)), run_text(&text(), Some(8)));
    assert_ne!(a.events_bin, b.events_bin);
    assert_ne!(a.deliveries, b.deliveries);
}

#[test]
fn seed_changes_demand_but_not_a_generated_network() {
    let mesh = std::fs::read_to_string(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../configs/mesh_small/scenario.toml"
    ))
    .unwrap();
    let a = drone_sim::scenario::Scenario::from_toml(&mesh)
        .unwrap()
        .with_seed(1);
    let b = drone_sim::scenario::Scenario::from_toml(&mesh)
        .unwrap()
        .with_seed(2);
    assert_eq!(a.cfg.lanes, b.cfg.lanes);
    assert_eq!(a.net.route_len_m, b.net.route_len_m);
}

#[test]
fn event_log_is_well_formed() {
    let r = run_text(&text(), None);
    let mut last = 0;
    for rec in &r.records {
        assert!(rec.t_ms >= last, "records are in time order");
        last = rec.t_ms;
        assert!(rec.status <= 4);
        assert!(rec.kind < 43 || rec.kind == drone_sim::event::TRANSITION_KIND);
    }
    let flights = &r.flights_bin;
    assert_eq!(&flights[0..8], b"DDFLOG\0\0");
    assert_eq!((flights.len() - 32) % 48, 0);
}
