//! Fault injection across seeds: message loss, random pad-controller faults, marker misses,
//! release and latch faults, all at once on the A-B example (2 drones parked on OUT, so the
//! yield logic runs too). Debug builds check the invariants after every event; this test
//! fails on any invariant panic or lockup symptom. It found six liveness and consistency bugs
//! during development (see docs/spec.md section 11, version 1.2.3). It also checks that every
//! delivered order's door-to-door time splits into four non-negative parts that add up.

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

/// pod_wait + queue + pad_service + flight = drone door-to-door on every delivered row (spec
/// 7 M2). Lost messages and diversions are where a part could overlap another.
fn check_split(r: &Run, seed: u64) {
    let header: Vec<&str> = r.deliveries.lines().next().unwrap().split(',').collect();
    let col = |name: &str| header.iter().position(|h| *h == name).unwrap();
    let total = col("drone_door_to_door_s");
    let parts = ["pod_wait_s", "queue_s", "pad_service_s", "flight_s"].map(col);
    for row in delivery_rows(r) {
        if !matches!(row[col("status")].as_str(), "DELIVERED" | "DIVERTED") {
            assert!(
                parts.iter().all(|&c| row[c].is_empty()),
                "seed {seed}: {row:?}"
            );
            continue;
        }
        let v: Vec<f64> = parts.iter().map(|&c| row[c].parse().unwrap()).collect();
        let sum: f64 = v.iter().sum();
        let want: f64 = row[total].parse().unwrap();
        // A diverted order can be released back at its origin without flying a lane.
        let flew = v[3] > 0.0 || row[col("status")] == "DIVERTED";
        assert!(v.iter().all(|&x| x >= 0.0) && flew, "seed {seed}: {row:?}");
        assert!(
            (sum - want).abs() < 0.01,
            "seed {seed}: parts {sum} != {want}: {row:?}"
        );
    }
}

#[test]
fn invariants_hold_and_the_network_keeps_moving_under_combined_faults() {
    let text = stressed();
    let mut delivered = 0;
    for seed in 1..=30u64 {
        let r = run_text(&text, Some(seed));
        let closed = &r.summary["orders_closed"];
        delivered += closed["delivered"].as_u64().unwrap();
        check_split(&r, seed);
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
