//! Spec 4.1: with `handshake.reorder_window_s`, a radio message that overtakes an earlier one
//! on its link waits for it, so the receiver sees the sender's order. The swap is injected:
//! on a local yield (D20) the drone sends DEPARTED then LANDING_REQUEST to the same pad, and
//! DEPARTED is delivered late.

mod common;
use common::*;
use drone_sim::event::tcode;
use drone_sim::ids::EntityKind;
use drone_sim::output::Outputs;
use drone_sim::scenario::Scenario;
use drone_sim::trace;
use toml::{Table, Value};

const D0: u16 = 0;
const A: u16 = 0;
const DEPARTED: u16 = 7;
/// Past the 50 ms radio latency, so the LANDING_REQUEST sent right after DEPARTED arrives first.
const LATE_MS: u64 = 300;

/// One drone parked on A's OUT: the first shipment at A makes it yield locally (D20).
fn yielding() -> Table {
    let mut t = base();
    set_q(&mut t, "pads.0.parking_capacity", 0);
    t
}

fn with_window(mut t: Table, window_s: f64) -> Table {
    set(&mut t, "schema_version", "1.6.0");
    let Value::Table(h) = t.get_mut("handshake").expect("handshake") else {
        unreachable!()
    };
    h.insert("reorder_window_s".into(), quantity(window_s, "s"));
    t
}

/// Run with drone 0's first DEPARTED delayed; the labels of drone 0 at A from D20 on.
fn swapped(t: &Table) -> Vec<String> {
    let scenario = Scenario::from_toml(&toml::to_string(t).expect("serialise")).expect("loads");
    let hash = scenario.cfg.source_hash;
    let mut world = scenario.into_world(0, Outputs::in_memory(hash));
    world.inject_message_delay((EntityKind::Drone, D0), DEPARTED, LATE_MS);
    world.run().expect("in-memory outputs");
    let records = trace::decode(world.out.events.bytes()).expect("events.bin");
    let labels: Vec<String> = records
        .iter()
        .filter_map(trace::Record::transition)
        .filter(|x| x.drone == D0 && x.pad == A)
        .map(|x| tcode::label(x.code))
        .collect();
    let from = labels
        .iter()
        .position(|l| l == "D20")
        .expect("drone 0 yields locally");
    labels[from..].to_vec()
}

#[test]
fn a_swap_inside_the_window_is_put_back_in_order() {
    let labels = swapped(&with_window(yielding(), 1.0));
    assert_eq!(
        labels[..3].to_vec(),
        strs(&["D20", "V36", "V01"]),
        "the pad closes the departure, then grants the landing at once: {labels:?}"
    );
}

#[test]
fn without_the_window_the_swap_costs_a_retry() {
    let labels = swapped(&yielding());
    assert_eq!(
        labels[..5].to_vec(),
        strs(&["D20", "V38", "V36", "D09", "V01"]),
        "the request arrives mid-departure, is taken as a duplicate, and the drone must retry: \
         {labels:?}"
    );
}

#[test]
fn a_held_message_is_released_when_the_window_ends() {
    // DEPARTED arrives 3 s late, after the 1 s window: the request goes through when the window
    // ends, as if DEPARTED were lost, and the late DEPARTED still closes the departure.
    let mut t = with_window(yielding(), 1.0);
    set_q(&mut t, "handshake.response_timeout_s", 10.0);
    let scenario = Scenario::from_toml(&toml::to_string(&t).expect("serialise")).expect("loads");
    let hash = scenario.cfg.source_hash;
    let mut world = scenario.into_world(0, Outputs::in_memory(hash));
    world.inject_message_delay((EntityKind::Drone, D0), DEPARTED, 3_000);
    world.run().expect("in-memory outputs");
    let records = trace::decode(world.out.events.bytes()).expect("events.bin");
    let at_a: Vec<_> = records
        .iter()
        .filter_map(trace::Record::transition)
        .filter(|x| x.drone == D0 && x.pad == A)
        .collect();
    let d20 = at_a
        .iter()
        .position(|x| tcode::label(x.code) == "D20")
        .expect("D20");
    let next: Vec<String> = at_a[d20..d20 + 3]
        .iter()
        .map(|x| tcode::label(x.code))
        .collect();
    assert_eq!(next, strs(&["D20", "V38", "V36"]), "{next:?}");
    let held_for = at_a[d20 + 1].t_ms - at_a[d20].t_ms;
    assert_eq!(held_for, 50 + 1_000, "released one window after it arrived");
}
