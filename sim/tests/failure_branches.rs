//! Spec 3.6: every failure branch, forced to happen (its fault probability set to 1, or a
//! scheduled fault window), asserting the exact transition sequence of the spec rows.

mod common;
use common::*;
use toml::Value;

const A: u16 = 0;
const B: u16 = 1;
const D0: u16 = 0;

fn has_alert(r: &Run, code: &str) -> bool {
    r.alerts
        .lines()
        .any(|l| l.contains(&format!("\"code\":\"{code}\"")))
}

fn order_status(r: &Run, order: usize) -> (String, String) {
    let row = &delivery_rows(r)[order];
    (row[2].clone(), row[3].clone())
}

#[test]
fn normal_delivery_follows_the_visit_rows() {
    let r = run(&base());
    // Pickup from parking at A, latch, fly to B, release, wait for OUT, hop, charge, park.
    let labels = drone_labels(&r, D0);
    assert_eq!(
        window(&labels, "P07", 41),
        strs(&[
            "P07", "D19", "V40", "V34", "D01", "D20", "V36", "V01", "D04", "D05", "V08", "D15",
            "D15", "V28", "V32", "D17", "D19", "V34", "D01", "D02", "V36", "D03", "V01", "D04",
            "D05", "V06", "D15", "D15", "V15", "V18", "V21", "D15", "D15", "V25", "D15", "V33",
            "D17", "P07", "D19", "V34", "D01",
        ])
    );
    assert_eq!(order_status(&r, 0).0, "DELIVERED");
}

#[test]
fn marker_not_detected_goes_around_holds_then_diverts() {
    let mut t = base();
    add_fault(&mut t, fault("MARKER_NOT_DETECTED", "B", 0.0, 10_000.0));
    let r = run(&t);
    let at_b = drone_pad_labels(&r, D0, B);
    // Two approach cycles of (1 + max_go_arounds) attempts, one marker hold, then divert.
    assert_eq!(
        window(&at_b, "D03", 20),
        strs(&[
            "D03", "V01", "D04", "D05", "D06", "D05", "D06", "D05", "V12", "D21", "D08", "V01",
            "D04", "D05", "D06", "D05", "D06", "D05", "V12", "D12",
        ])
    );
    let at_a = drone_pad_labels(&r, D0, A);
    let after_divert: Vec<String> = at_a
        .iter()
        .skip_while(|l| *l != "V36")
        .skip(1)
        .cloned()
        .collect();
    assert_eq!(
        window(&after_divert, "D03", 9),
        strs(&[
            "D03", "V01", "D04", "D05", "V06", "D15", "D15", "V15", "V18"
        ])
    );
    assert_eq!(order_status(&r, 0).0, "DIVERTED");
    assert!(has_alert(&r, "DIVERTED"));
}

#[test]
fn pc_fault_in_the_air_retries_reports_silence_and_diverts() {
    let arrive_b = first_time(&run(&base()), "D03", D0) as f64 / 1000.0;
    let mut t = base();
    add_fault(&mut t, fault("PC_FAULT", "B", arrive_b - 10.0, 600.0));
    let r = run(&t);
    let at_b = drone_pad_labels(&r, D0, B);
    assert_eq!(window(&at_b, "D03", 4), strs(&["D03", "D09", "D10", "D12"]));
    let dropped = r
        .records
        .iter()
        .filter(|x| x.status == 2 && x.kind_name() == "LANDING_REQUEST")
        .count();
    assert_eq!(
        dropped, 2,
        "first request and one retry reach the silent PC"
    );
    assert_eq!(order_status(&r, 0).0, "DIVERTED");
    assert!(has_alert(&r, "DIVERTED"));
    // After the fault ends the network recovers and keeps delivering to B.
    assert!(
        delivery_rows(&r)
            .iter()
            .any(|row| row[2] == "DELIVERED" && row[6] == "1")
    );
}

#[test]
fn pc_fault_on_the_ground_keeps_the_drone_landed_and_resumes() {
    let landed = first_time(&run(&base()), "V08", D0) as f64 / 1000.0;
    let mut t = base();
    add_fault(&mut t, fault("PC_FAULT", "A", landed + 1.0, 120.0));
    let r = run(&t);
    let labels = drone_labels(&r, D0);
    let resends = labels.iter().filter(|l| *l == "D16").count();
    assert!(
        resends >= 3,
        "drone re-sends every response timeout while the PC is silent ({resends})"
    );
    let without_resends: Vec<String> = labels.iter().filter(|l| *l != "D16").cloned().collect();
    // Latch done during the fault; on recovery the held timer re-sends LATCH_CMD (V27), the
    // drone's re-sent report is accepted (V28) and the visit completes.
    assert_eq!(
        window(&without_resends, "V08", 7),
        strs(&["V08", "D15", "D15", "V27", "V28", "V32", "D17"])
    );
    assert!(has_alert(&r, "PC_SILENT"));
    let pad_rows: Vec<String> = transitions(&r)
        .iter()
        .filter(|x| x.pad == A && x.code < 200)
        .map(|x| drone_sim::event::tcode::label(x.code))
        .collect();
    assert_eq!(pad_rows, strs(&["M01", "M02"]));
    // The drone never left the pad without clearance: no takeoff between M01 and M02.
    let tr = transitions(&r);
    let (m01, m02) = (
        tr.iter().find(|x| x.code == 101).unwrap().t_ms,
        tr.iter().find(|x| x.code == 102).unwrap().t_ms,
    );
    assert!(
        !tr.iter()
            .any(|x| x.code == 401 && x.drone == D0 && x.t_ms > m01 && x.t_ms < m02)
    );
}

#[test]
fn in_occupied_hold_ends_in_divert_at_the_hold_limit() {
    let mut t = base();
    set_q(&mut t, "pad_ops.pod_remove_s", 7200.0);
    let r = run(&t);
    let at_b = drone_pad_labels(&r, D0, B);
    assert_eq!(
        window(&at_b, "V02", 6),
        strs(&["V02", "D07", "D21", "D08", "V03", "D07"])
    );
    let tr = transitions(&r);
    let held = tr
        .iter()
        .find(|x| x.code == 302 && x.drone == D0)
        .unwrap()
        .t_ms;
    let diverted = tr
        .iter()
        .find(|x| x.code == 412 && x.drone == D0 && x.t_ms > held)
        .unwrap()
        .t_ms;
    let max_hold_ms = 180_000;
    assert!(
        (max_hold_ms..=max_hold_ms + 1_000).contains(&(diverted - held)),
        "divert after {} ms",
        diverted - held
    );
    assert!(
        at_b.contains(&"V05".to_owned()),
        "the held visit is cancelled at B"
    );
}

#[test]
fn in_occupied_divert_policy_diverts_at_once() {
    let mut t = base();
    set_q(&mut t, "pad_ops.pod_remove_s", 7200.0);
    set(&mut t, "policy.in_occupied_action", Value::from("DIVERT"));
    let r = run(&t);
    assert_eq!(
        window(&drone_pad_labels(&r, D0, B), "V02", 4),
        strs(&["V02", "D07", "D12", "V05"])
    );
}

#[test]
fn release_fault_retries_once_then_grounds_the_drone_and_closes_the_pad() {
    let mut t = base();
    set_q(&mut t, "drone.p_release_fault", 1.0);
    let r = run(&t);
    assert_eq!(
        window(&drone_labels(&r, D0), "V06", 10),
        strs(&[
            "V06", "D15", "D15", "V15", "V19", "D15", "D15", "V15", "V20", "D13"
        ])
    );
    assert_eq!(
        order_status(&r, 0),
        ("FAILED".to_owned(), "RELEASE_FAULT".to_owned())
    );
    assert!(has_alert(&r, "RELEASE_FAULT") && has_alert(&r, "PAD_OUT_OF_SERVICE"));
    assert!(
        transitions(&r).iter().any(|x| x.code == 103 && x.pad == B),
        "M03 at B"
    );
}

#[test]
fn latch_fail_retries_once_then_departs_empty_and_alerts() {
    let mut t = base();
    set_q(&mut t, "drone.p_latch_fault", 1.0);
    let r = run(&t);
    assert_eq!(
        window(&drone_labels(&r, D0), "V08", 10),
        strs(&[
            "V08", "D15", "D15", "V28", "V29", "D15", "D15", "V28", "V30", "D17"
        ])
    );
    assert!(has_alert(&r, "LATCH_FAULT"));
    assert_eq!(r.summary["latch_abort_count"], 1);
    assert_eq!(
        order_status(&r, 0).0,
        "CREATED",
        "the pod stays on OUT for another drone"
    );
}

fn out_wait_ms(r: &Run) -> u64 {
    let tr = transitions(r);
    let v18 = tr.iter().find(|x| x.code == 318).unwrap();
    let v21 = tr
        .iter()
        .find(|x| x.code == 321 && x.t_ms >= v18.t_ms && x.pad == v18.pad)
        .unwrap();
    v21.t_ms - v18.t_ms
}

#[test]
fn out_empty_wait_policy_waits_the_timeout_before_hopping() {
    let r = run(&base());
    assert_eq!(out_wait_ms(&r), 120_000);
}

#[test]
fn out_empty_depart_empty_policy_hops_at_once() {
    let mut t = base();
    set(
        &mut t,
        "pad_ops.out_empty_policy",
        Value::from("DEPART_EMPTY"),
    );
    assert_eq!(out_wait_ms(&run(&t)), 0);
}

#[test]
fn pad_out_of_service_reroutes_inbound_drones() {
    let departed = first_time(&run(&base()), "D02", D0) as f64 / 1000.0;
    let mut t = base();
    add_fault(
        &mut t,
        fault("OUT_OF_SERVICE", "B", departed + 60.0, 3000.0),
    );
    let r = run(&t);
    // New destination mid-lane (D24), recomputed at the lane end (D03 EN_ROUTE), land at A.
    assert_eq!(
        window(&drone_labels(&r, D0), "D24", 7),
        strs(&["D24", "D03", "D03", "V01", "D04", "D05", "V06"])
    );
    assert_eq!(order_status(&r, 0).0, "DIVERTED");
    let modes: Vec<u16> = transitions(&r)
        .iter()
        .filter(|x| x.pad == B && x.code < 200)
        .map(|x| x.code)
        .collect();
    assert_eq!(modes, vec![103, 104], "M03 then M04 at B");
}

#[test]
fn lost_messages_are_retried_and_the_network_keeps_working() {
    let mut t = base();
    set_q(&mut t, "handshake.p_msg_loss", 0.3);
    let r = run(&t);
    let lost = r.records.iter().filter(|x| x.status == 3).count();
    assert!(lost > 10, "{lost} lost messages");
    let labels: Vec<String> = transitions(&r)
        .iter()
        .map(|x| drone_sim::event::tcode::label(x.code))
        .collect();
    for retry in ["D09", "D16", "V38"] {
        assert!(labels.iter().any(|l| l == retry), "{retry} never happened");
    }
    assert!(r.summary["deliveries_in_window"].as_u64().unwrap() > 0);
}

#[test]
fn drone_silence_after_grant_is_resolved_by_the_spot_sensor() {
    // With heavy loss some LANDED reports never arrive; the PC's landing timeout must then
    // use the spot sensor (V13) rather than block the spot.
    let mut t = base();
    set_q(&mut t, "handshake.p_msg_loss", 0.5);
    set_q(&mut t, "sim.duration_s", 21_600.0);
    let r = run(&t);
    let labels: Vec<String> = transitions(&r)
        .iter()
        .map(|x| drone_sim::event::tcode::label(x.code))
        .collect();
    assert!(
        labels.iter().any(|l| l == "V13" || l == "V17"),
        "no silence branch exercised"
    );
}
