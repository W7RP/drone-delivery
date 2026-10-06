//! The phase-2 synthetic scenarios run end to end with debug invariants on, and produce a
//! drone-versus-truck comparison.

mod common;
use common::*;

fn run_scenario(name: &str) -> Run {
    let path = format!(
        "{}/../configs/{name}/scenario.toml",
        env!("CARGO_MANIFEST_DIR")
    );
    run_text(&std::fs::read_to_string(path).unwrap(), None)
}

fn check(r: &Run, name: &str) {
    let s = &r.summary;
    assert!(
        s["deliveries_in_window"].as_u64().unwrap() > 20,
        "{name}: {}",
        s["deliveries_in_window"]
    );
    assert!(
        s["door_to_door_s"]["mean"].is_number() && s["truck_door_to_door_s"]["mean"].is_number(),
        "{name}"
    );
    assert!(
        s["cost"]["truck_cost_per_delivery"].is_number()
            && s["cost"]["drone_cost_per_delivery"].is_number(),
        "{name}"
    );
    let open = s["open_at_end"].as_u64().unwrap();
    let delivered = s["orders_closed"]["delivered"].as_u64().unwrap();
    assert!(
        open * 10 < delivered,
        "{name}: {open} open against {delivered} delivered"
    );
}

#[test]
fn hub_and_spoke_runs_and_compares_with_truck() {
    let r = run_scenario("hub_spoke_6");
    check(&r, "hub_spoke_6");
    assert_eq!(r.summary["pads"], 6);
}

#[test]
fn mesh_runs_and_compares_with_truck() {
    let r = run_scenario("mesh_small");
    check(&r, "mesh_small");
    // Repositioning and multi-lane routes mean more lane traversals than deliveries.
    let lanes_flown = u64::try_from((r.flights_bin.len() - 32) / 48).unwrap();
    assert!(lanes_flown > r.summary["deliveries_in_window"].as_u64().unwrap());
}

#[test]
fn scale_fleet_keeps_moving_without_forced_landings() {
    // Six hours of the 200-drone, 40-pad scale case with debug invariants on. Regression for
    // two scale bugs: an order to a pad that is out of service for good blocked its origin's
    // OUT forever, and congested neighbours bounced drones until forced landings (spec 1.2.4).
    let path = format!(
        "{}/../configs/scale_200/scenario.toml",
        env!("CARGO_MANIFEST_DIR")
    );
    let mut t: toml::Table = std::fs::read_to_string(path).unwrap().parse().unwrap();
    set_q(&mut t, "sim.duration_s", 21_600.0);
    let r = run(&t);
    assert!(
        r.summary["deliveries_per_h"].as_f64().unwrap() > 80.0,
        "{}",
        r.summary["deliveries_per_h"]
    );
    assert_eq!(r.summary["alerts"]["DRONE_GROUNDED"], 0);
    assert_eq!(r.summary["alerts"]["NO_ALTERNATE"], 0);
}
