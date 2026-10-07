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
    // The overload the bugs were found under (the scenario config itself now runs below
    // capacity): 6 requests per pad-hour and the weakest pod rebalancing.
    set_q(&mut t, "demand.rate_per_pad_per_h", 6.0);
    set_q(&mut t, "policy.pod_min_per_pad", 1);
    let r = run(&t);
    assert!(
        r.summary["deliveries_per_h"].as_f64().unwrap() > 80.0,
        "{}",
        r.summary["deliveries_per_h"]
    );
    assert_eq!(r.summary["alerts"]["DRONE_GROUNDED"], 0);
    assert_eq!(r.summary["alerts"]["NO_ALTERNATE"], 0);
}

/// `VisitEstimate` is spec 2.7 applied to one visit: hover-priced takeoff, approach and empty
/// hop around a cruise lane, all from the config. Worked by hand for the A-B example at 5 kg,
/// where the total mass equals `drone.power_ref_mass_kg` (k = 1).
#[test]
fn visit_estimate_follows_the_flight_model() {
    let s = drone_sim::scenario::Scenario::from_toml(&toml::to_string(&common::example()).unwrap())
        .unwrap();
    let d = &s.cfg.drone;
    let v = drone_sim::flight::VisitEstimate::new(&s.cfg, 3000.0, 5.0);
    let close = |a: f64, b: f64| (a - b).abs() < 1e-6 * b.max(1.0);
    assert!(close(v.cruise.time_s, 3000.0 / d.cruise_speed_mps));
    assert!(close(
        v.cruise.energy_j,
        d.cruise_power_ref_w * 3000.0 / d.cruise_speed_mps
    ));
    assert!(close(v.takeoff.energy_j, d.hover_power_ref_w * d.takeoff_s));
    assert!(close(v.approach.time_s, d.approach_s));
    // The hop is flown empty: lighter than the reference mass, so cheaper per second.
    let k_empty = ((d.empty_mass_kg + d.latch_mass_kg) / d.power_ref_mass_kg).powf(1.5);
    assert!(close(
        v.hop.energy_j,
        d.hover_power_ref_w * k_empty * d.hop_s
    ));
    assert!(close(
        v.total().time_s,
        d.takeoff_s + 3000.0 / d.cruise_speed_mps + d.approach_s + d.hop_s
    ));
}
