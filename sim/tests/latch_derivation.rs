//! Spec A71: the latch-fail probability can be derived from the OUT landing error distribution
//! and the latch capture radius instead of being given.

mod common;
use common::*;
use drone_sim::scenario::Scenario;
use toml::{Table, Value};

fn derived(mut t: Table, bias_m: f64, sigma_m: f64, radius_m: f64) -> Table {
    set(&mut t, "schema_version", "1.6.0");
    let Value::Table(drone) = t.get_mut("drone").expect("drone") else {
        unreachable!()
    };
    drone.remove("p_latch_fault");
    drone.insert("latch_capture_radius_m".into(), quantity(radius_m, "m"));
    drone.insert("out_landing_error_bias_m".into(), quantity(bias_m, "m"));
    drone.insert("out_landing_error_sigma_m".into(), quantity(sigma_m, "m"));
    t
}

fn load(t: &Table) -> Result<Scenario, String> {
    Scenario::from_toml(&toml::to_string(t).expect("serialise")).map_err(|e| e.0)
}

#[test]
fn phase_4_fit_gives_three_percent_per_attempt() {
    let s = load(&derived(example(), 0.080_035, 0.034_821, 0.15)).expect("loads");
    let p = s.cfg.drone.p_latch_fault;
    assert!((p - 0.0321).abs() < 5e-4, "p = {p}");
}

#[test]
fn a_landing_that_always_misses_the_funnel_fails_every_latch() {
    // Spot-on bias of 0.5 m against a 0.15 m funnel: p is 1, so the latch fails, is retried
    // once and the drone leaves empty (V29, V30).
    let r = run(&derived(base(), 0.5, 0.01, 0.15));
    let labels = drone_labels(&r, 0);
    assert_eq!(
        window(&labels, "V28", 4),
        strs(&["V28", "V29", "D15", "D15"])
    );
    assert!(labels.iter().any(|l| l == "V30"));
}

#[test]
fn latch_fault_is_given_one_way_only() {
    let mut both = derived(example(), 0.08, 0.035, 0.15);
    let Value::Table(drone) = both.get_mut("drone").expect("drone") else {
        unreachable!()
    };
    drone.insert("p_latch_fault".into(), quantity(0.01, "1"));
    assert!(
        load(&both)
            .err()
            .expect("rejected")
            .contains("give p_latch_fault")
    );
}
