//! Spec A72: pad layout keys. `pad_ops.out_spots` gives every pad up to three OUT spots (four from schema 1.9.0), each
//! able to hold a pod and a drone; `pad_ops.parking_capacity` replaces every pad's parking; with
//! `charging.charge_while_parked`, a drone with nothing to latch charges in parking, not on OUT.

mod common;
use common::*;
use drone_sim::event::tcode;
use drone_sim::ids::Spot;
use drone_sim::trace::KIND_NAMES;
use toml::{Table, Value};

const A: u16 = 0;

fn layout(mut t: Table, out_spots: i64, parking: Option<i64>) -> Table {
    set(&mut t, "schema_version", "1.6.0");
    let Value::Table(po) = t.get_mut("pad_ops").expect("pad_ops") else {
        unreachable!()
    };
    po.insert("out_spots".into(), quantity(out_spots, "count"));
    if let Some(c) = parking {
        po.insert("parking_capacity".into(), quantity(c, "count"));
    }
    t
}

fn kind(name: &str) -> u16 {
    u16::try_from(KIND_NAMES.iter().position(|&k| k == name).expect("kind")).expect("u16")
}

/// Spot codes of every LANDED a pad processed.
fn landed_spots(r: &Run, pad: u16) -> Vec<u8> {
    r.records
        .iter()
        .filter(|x| x.kind == kind("LANDED") && x.status == 0)
        .filter(|x| u16::from_le_bytes([x.args[2], x.args[3]]) == pad)
        .map(|x| x.args[4])
        .collect()
}

fn labels_at(r: &Run, pad: u16) -> Vec<(u64, String)> {
    transitions(r)
        .iter()
        .filter(|x| x.pad == pad)
        .map(|x| (x.t_ms, tcode::label(x.code)))
        .collect()
}

fn busy_two_drones(t: Table) -> Table {
    let mut t = t;
    set_q(&mut t, "drone.fleet_size", 2);
    set(
        &mut t,
        "drone.initial_pads",
        Value::Array(vec!["A".into(), "A".into()]),
    );
    set_q(&mut t, "demand.flows.0.rate_per_h", 20.0);
    t
}

/// Two pod placements in progress at A at the same time (two P02 with no P03 between).
fn placements_overlap(r: &Run) -> bool {
    let l: Vec<String> = labels_at(r, A).into_iter().map(|x| x.1).collect();
    let mut placing = 0;
    for x in &l {
        match x.as_str() {
            "P02" => placing += 1,
            "P03" => placing -= 1,
            _ => {}
        }
        if placing >= 2 {
            return true;
        }
    }
    false
}

#[test]
fn a_second_out_spot_takes_a_second_pod_and_a_second_drone() {
    let r = run(&busy_two_drones(layout(base(), 2, None)));
    assert!(placements_overlap(&r), "two pods placed at A at once");
    let out2 = Spot::Out2.code();
    assert!(
        landed_spots(&r, A).contains(&out2),
        "a drone lands on OUT_2 at A"
    );
    assert!(landed_spots(&r, 1).contains(&out2), "and at B");

    let one = run(&busy_two_drones(base()));
    assert!(
        !placements_overlap(&one),
        "one OUT spot places one pod at a time"
    );
    assert!(!landed_spots(&one, A).contains(&out2));
}

/// V09 / V25: a drone with no pickup charging on OUT.
fn charges_on_out(r: &Run) -> usize {
    transitions(r)
        .iter()
        .filter(|x| ["V09", "V25"].contains(&tcode::label(x.code).as_str()))
        .count()
}

#[test]
fn parked_charging_keeps_out_free() {
    let control = run(&base());
    assert!(
        charges_on_out(&control) > 0,
        "the control run charges on OUT"
    );

    let mut t = layout(base(), 1, None);
    set(&mut t, "charging.charge_while_parked", true);
    let r = run(&t);
    assert_eq!(
        charges_on_out(&r),
        0,
        "nothing charges on OUT without a pod"
    );
    let cycles = r.summary["battery_cycles"]["fleet_mean"]
        .as_f64()
        .expect("cycles");
    assert!(cycles > 0.0, "the drone still charges, in parking");
    assert!(r.summary["deliveries_per_h"].as_f64().expect("rate") > 0.0);
}

#[test]
fn layout_keys_are_checked() {
    let reject = |t: &Table| match drone_sim::scenario::Scenario::from_toml(
        &toml::to_string(t).expect("serialise"),
    ) {
        Ok(_) => panic!("accepted"),
        Err(e) => e.0,
    };
    assert!(reject(&layout(base(), 4, None)).contains("pad_ops.out_spots"));
    // No parking and one OUT spot: two drones cannot both start at A (R4).
    let t = busy_two_drones(layout(base(), 1, Some(0)));
    assert!(reject(&t).contains("R4"));
    // With two OUT spots they can.
    let t = busy_two_drones(layout(base(), 2, Some(0)));
    assert!(run(&t).summary["deliveries_per_h"].as_f64().expect("rate") > 0.0);
}

/// One drone parked on A's first OUT spot (no parking area), a second OUT spot for the pod.
fn parked_on_out_two_spots() -> Table {
    let mut t = layout(base(), 2, Some(0));
    set(&mut t, "drone.initial_pads", Value::Array(vec!["A".into()]));
    t
}

#[test]
fn a_drone_on_one_out_spot_hops_to_its_pod_on_another() {
    let r = run(&parked_on_out_two_spots());
    let at_a = drone_pad_labels(&r, 0, A);
    // The parked drone asks to leave for the pickup at its own pad, hops to the pod's spot on
    // the ground (V43) instead of taking off and queueing, lands there and latches (V24).
    assert_eq!(
        window(&at_a, "V43", 4),
        strs(&["V43", "D15", "D15", "V24"]),
        "{at_a:?}"
    );
    assert!(
        !at_a.iter().any(|l| l == "D20"),
        "no local takeoff and re-landing"
    );
    assert!(!has_alert(&r, "DRONE_SILENT"), "the drone answers the hop");
    assert!(r.summary["deliveries_per_h"].as_f64().expect("rate") > 0.0);
}

#[test]
fn pods_per_origin_sets_only_the_pads_that_ship() {
    // base(): one flow, A -> B; both pads start with the example's own pods.
    let own = drone_sim::scenario::Scenario::from_toml(&toml::to_string(&base()).unwrap())
        .expect("base loads");
    let b_own = own.cfg.pads[1].initial_empty_pods;
    let mut t = layout(base(), 1, None);
    let Value::Table(demand) = t.get_mut("demand").expect("demand") else {
        unreachable!()
    };
    demand.insert("pods_per_origin".into(), quantity(5, "count"));
    let s = drone_sim::scenario::Scenario::from_toml(&toml::to_string(&t).unwrap()).expect("loads");
    assert_eq!(s.cfg.pads[0].initial_empty_pods, 5, "A ships");
    assert_eq!(s.cfg.pads[1].initial_empty_pods, b_own, "B only receives");
}

#[test]
fn four_out_spots_need_schema_1_9_and_run() {
    let reject = |t: &Table| match drone_sim::scenario::Scenario::from_toml(
        &toml::to_string(t).expect("serialise"),
    ) {
        Ok(_) => panic!("accepted"),
        Err(e) => e.0,
    };
    let mut four = layout(base(), 4, Some(0));
    assert!(
        reject(&four).contains("1.9.0"),
        "a 1.6.0 file may not use OUT_4"
    );
    set(&mut four, "schema_version", "1.9.0");
    let r = run(&four);
    assert!(r.summary["deliveries_in_window"].as_u64().expect("count") > 0);
    let mut five = layout(base(), 5, Some(0));
    set(&mut five, "schema_version", "1.9.0");
    assert!(reject(&five).contains("pad_ops.out_spots"));
}
