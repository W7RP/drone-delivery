//! Spec A35: a pad with `pad_ops.marker_self_check` reports a marker fault in PAD_STATUS, and
//! FM sends no drone to land there until it clears. A drone already in the air keeps the
//! marker holding policy (hold, then divert).

mod common;
use common::*;
use drone_sim::ids::MissionKind;
use drone_sim::trace::KIND_NAMES;
use toml::{Table, Value};

const D0: u16 = 0;
const D1: u16 = 1;
const B: u16 = 1;
const RUN_S: f64 = 10_800.0;
/// FM learns of the fault one PC -> FM latency after it starts; a mission sent just before
/// that still reaches the drone a little later.
const NOTICE_MS: u64 = 2_000;

fn two_drones_at_a() -> Table {
    let mut t = base();
    set_q(&mut t, "drone.fleet_size", 2);
    set(
        &mut t,
        "drone.initial_pads",
        Value::Array(vec!["A".into(), "A".into()]),
    );
    t
}

fn with_self_check(mut t: Table) -> Table {
    set(&mut t, "schema_version", "1.6.0");
    let Value::Table(po) = t.get_mut("pad_ops").expect("pad_ops") else {
        unreachable!()
    };
    po.insert("marker_self_check".into(), Value::Boolean(true));
    t
}

/// (receipt time, drone, mission, dest) of every MISSION_ASSIGN a drone processed.
fn missions(r: &Run) -> Vec<(u64, u16, u8, u16)> {
    let kind = KIND_NAMES
        .iter()
        .position(|&k| k == "MISSION_ASSIGN")
        .expect("kind");
    r.records
        .iter()
        .filter(|x| usize::from(x.kind) == kind && x.status == 0)
        .map(|x| {
            let u16_at = |i: usize| u16::from_le_bytes([x.args[i], x.args[i + 1]]);
            (x.t_ms, u16_at(0), x.args[2], u16_at(3))
        })
        .collect()
}

fn flies_in(mission: u8) -> bool {
    [
        MissionKind::Deliver,
        MissionKind::Pickup,
        MissionKind::Reposition,
        MissionKind::Divert,
    ]
    .iter()
    .any(|k| k.code() == mission)
}

fn sent_to_b_after(r: &Run, drone: u16, t_ms: u64, until_ms: u64) -> usize {
    missions(r)
        .iter()
        .filter(|&&(t, d, m, dest)| {
            d == drone && dest == B && flies_in(m) && t > t_ms && t < until_ms
        })
        .count()
}

/// The fault starts a minute after drone 0 leaves A for B, so it is in the air when FM learns.
fn fault_start_s() -> f64 {
    let r = run(&two_drones_at_a());
    let departed_ms = first_time(&r, "D02", D0);
    #[allow(clippy::cast_precision_loss)]
    let s = departed_ms as f64 / 1000.0 + 60.0;
    s
}

#[test]
fn no_drone_is_sent_into_a_pad_reporting_a_marker_fault() {
    let start_s = fault_start_s();
    let mut t = with_self_check(two_drones_at_a());
    add_fault(
        &mut t,
        fault("MARKER_NOT_DETECTED", "B", start_s, RUN_S - start_s),
    );
    let r = run(&t);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let notice = (start_s * 1000.0) as u64 + NOTICE_MS;

    // The drone already in the air finds no marker, holds, then diverts (existing policy).
    let d0 = drone_pad_labels(&r, D0, B);
    assert!(d0.iter().any(|l| l == "D12"), "drone 0 diverts: {d0:?}");
    // The idle drone at A is never sent to B while the fault lasts, nor is drone 0 again.
    assert_eq!(sent_to_b_after(&r, D1, notice, u64::MAX), 0);
    let diverted = first_time(&r, "D12", D0);
    assert_eq!(sent_to_b_after(&r, D0, diverted, u64::MAX), 0);
}

#[test]
fn without_self_check_the_second_drone_is_sent_in() {
    let start_s = fault_start_s();
    let mut t = two_drones_at_a();
    add_fault(
        &mut t,
        fault("MARKER_NOT_DETECTED", "B", start_s, RUN_S - start_s),
    );
    let r = run(&t);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let notice = (start_s * 1000.0) as u64 + NOTICE_MS;
    assert!(
        sent_to_b_after(&r, D1, notice, u64::MAX) > 0,
        "the control run must send drone 1 to B, or the test above proves nothing"
    );
}

#[test]
fn dispatch_into_the_pad_resumes_when_the_fault_clears() {
    let start_s = fault_start_s();
    let duration_s = 1800.0;
    let mut t = with_self_check(two_drones_at_a());
    add_fault(
        &mut t,
        fault("MARKER_NOT_DETECTED", "B", start_s, duration_s),
    );
    let r = run(&t);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let (notice, cleared) = (
        (start_s * 1000.0) as u64 + NOTICE_MS,
        ((start_s + duration_s) * 1000.0) as u64,
    );
    assert_eq!(sent_to_b_after(&r, D1, notice, cleared), 0);
    let after = missions(&r)
        .iter()
        .filter(|&&(t, _, m, dest)| dest == B && flies_in(m) && t > cleared)
        .count();
    assert!(after > 0, "orders to B are flown once the marker is back");
}
