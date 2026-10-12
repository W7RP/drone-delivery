//! Fault injection across seeds: message loss, random pad-controller faults, marker misses,
//! release and latch faults, all at once on the A-B example (2 drones parked on OUT, so the
//! yield logic runs too). Debug builds check the invariants after every event; this test
//! fails on any invariant panic or lockup symptom. It found six liveness and consistency bugs
//! during development (see docs/spec.md section 11, version 1.2.3).

mod common;
use common::*;

fn stressed() -> String {
    let mut t = example();
    set_q(&mut t, "handshake.p_msg_loss", 0.3);
    set_q(&mut t, "pad_ops.pc_fault_rate_per_h", 0.5);
    set_q(&mut t, "pad_ops.p_marker_detect", 0.8);
    set_q(&mut t, "drone.p_release_fault", 0.05);
    set_q(&mut t, "drone.p_latch_fault", 0.05);
    toml::to_string(&t).expect("serialise")
}

#[test]
fn invariants_hold_and_the_network_keeps_moving_under_combined_faults() {
    let text = stressed();
    let mut delivered = 0;
    for seed in 1..=30u64 {
        let r = run_text(&text, Some(seed));
        let closed = &r.summary["orders_closed"];
        delivered += closed["delivered"].as_u64().unwrap();
        // Liveness: every run ends with almost every order closed; a lockup leaves many open.
        assert!(
            r.summary["open_at_end"].as_u64().unwrap() <= 4,
            "seed {seed}: {}",
            r.summary["open_at_end"]
        );
    }
    assert!(
        delivered >= 200,
        "only {delivered} deliveries across 30 stressed seeds"
    );
}
