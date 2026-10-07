//! Spec 3.2 P08 and 7 M9: with `demand.max_backlog_per_pad` a pad that cannot keep up turns
//! requests away instead of queueing them without limit; without the key nothing is rejected.
//! Debug builds also assert after every event that no backlog exceeds the cap.

mod common;
use common::*;

/// One drone and far more requests than it can carry, so the origin's backlog fills.
fn overloaded() -> toml::Table {
    let mut t = base();
    set_q(&mut t, "demand.flows.0.rate_per_h", 60.0);
    t
}

fn label_count(r: &Run, label: &str) -> u64 {
    transitions(r)
        .iter()
        .filter(|t| drone_sim::event::tcode::label(t.code) == label)
        .count() as u64
}

#[test]
fn a_full_backlog_rejects_requests_and_counts_them() {
    let mut t = overloaded();
    t["demand"]
        .as_table_mut()
        .unwrap()
        .insert("max_backlog_per_pad".into(), quantity(5, "count"));
    let r = run(&t);
    let rejected = r.summary["requests_rejected"].as_u64().unwrap();
    let requests = r.summary["shipment_requests_in_window"].as_u64().unwrap();
    assert!(rejected > requests / 2, "{rejected} of {requests} rejected");
    // Warm-up is 0, so every rejection is in the window and has its P08 record.
    assert_eq!(label_count(&r, "P08"), rejected);
    assert_eq!(label_count(&r, "P01") + rejected, requests);
    let fraction = r.summary["rejected_fraction"].as_f64().unwrap();
    #[allow(clippy::cast_precision_loss)]
    let expected = rejected as f64 / requests as f64;
    assert!((fraction - expected).abs() < 1e-12);
}

#[test]
fn without_the_key_requests_queue_without_limit() {
    let r = run(&overloaded());
    assert_eq!(r.summary["requests_rejected"].as_u64(), Some(0));
    assert_eq!(label_count(&r, "P08"), 0);
}

#[test]
fn the_cap_must_be_at_least_one() {
    let mut t = overloaded();
    t["demand"]
        .as_table_mut()
        .unwrap()
        .insert("max_backlog_per_pad".into(), quantity(0, "count"));
    let text = toml::to_string(&t).unwrap();
    let err = drone_sim::scenario::Scenario::from_toml(&text)
        .err()
        .unwrap()
        .0;
    assert!(err.contains("demand.max_backlog_per_pad"), "{err}");
}
