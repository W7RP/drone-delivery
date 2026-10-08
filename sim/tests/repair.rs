//! Spec A69: with `pad_ops.repair_mean_s` and `repair_spread_s`, a pad closed by a release fault
//! (V20) comes back after the repair, with its drone; the alerts still go to remote ops. Without
//! the keys the pad stays out of service (`failure_branches.rs`).

mod common;
use common::*;
use drone_sim::event::tcode;
use toml::{Table, Value};

const D0: u16 = 0;
const B: u16 = 1;
const REPAIR_MEAN_S: f64 = 600.0;

fn with_repair(mut t: Table, mean_s: f64, spread_s: f64) -> Table {
    set(&mut t, "schema_version", "1.6.0");
    let Value::Table(po) = t.get_mut("pad_ops").expect("pad_ops") else {
        unreachable!()
    };
    po.insert("repair_mean_s".into(), quantity(mean_s, "s"));
    po.insert("repair_spread_s".into(), quantity(spread_s, "s"));
    t
}

fn time_of(r: &Run, label: &str, pad: u16) -> Vec<u64> {
    transitions(r)
        .iter()
        .filter(|x| tcode::label(x.code) == label && x.pad == pad)
        .map(|x| x.t_ms)
        .collect()
}

#[test]
fn a_repaired_pad_and_drone_return_to_service() {
    let mut t = with_repair(base(), REPAIR_MEAN_S, 0.0);
    set_q(&mut t, "drone.p_release_fault", 1.0);
    let r = run(&t);

    let grounded = first_time(&r, "V20", D0);
    let repaired = time_of(&r, "M06", B);
    assert_eq!(
        repaired.first(),
        Some(&(grounded + 600_000)),
        "M06 at B exactly one repair after V20"
    );
    // The stuck pod goes into B's inventory, the drone is back on IN and re-registers.
    assert_eq!(
        window(&drone_pad_labels(&r, D0, B), "V20", 6),
        strs(&["V20", "D13", "P09", "D25", "V42", "V07"])
    );
    // Remote ops still hear about the fault.
    assert!(has_alert(&r, "RELEASE_FAULT") && has_alert(&r, "PAD_OUT_OF_SERVICE"));
    // B serves again: the repaired drone takes off from it and lands at it once more.
    let labels = drone_labels(&r, D0);
    let after_repair = labels.iter().position(|l| l == "D25").expect("D25");
    assert!(
        labels[after_repair..].iter().any(|l| l == "D01"),
        "drone flies again"
    );
    let landings_at_b = time_of(&r, "V06", B);
    assert!(
        landings_at_b.iter().any(|&t| t > repaired[0]),
        "a drone lands with a pod at B after the repair"
    );
}

#[test]
fn repair_time_is_drawn_within_its_spread() {
    let mut t = with_repair(base(), REPAIR_MEAN_S, 300.0);
    set_q(&mut t, "drone.p_release_fault", 1.0);
    for seed in 1..=5 {
        let r = run_text(&toml::to_string(&t).expect("serialise"), Some(seed));
        let grounded = time_of(&r, "V20", B);
        let repaired = time_of(&r, "M06", B);
        assert!(!repaired.is_empty(), "seed {seed}: no repair");
        for (g, m) in grounded.iter().zip(&repaired) {
            let took = m - g;
            assert!(
                (300_000..=900_000).contains(&took),
                "seed {seed}: repair took {took} ms"
            );
        }
    }
}

#[test]
fn repair_keys_come_as_a_pair() {
    let mut t = with_repair(base(), REPAIR_MEAN_S, 0.0);
    let Value::Table(po) = t.get_mut("pad_ops").expect("pad_ops") else {
        unreachable!()
    };
    po.remove("repair_spread_s");
    let err = match drone_sim::scenario::Scenario::from_toml(&toml::to_string(&t).unwrap()) {
        Ok(_) => panic!("accepted"),
        Err(e) => e.0,
    };
    assert!(
        err.contains("repair_mean_s and repair_spread_s together"),
        "{err}"
    );
}
