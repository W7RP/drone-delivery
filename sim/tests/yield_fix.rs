//! Spec A74 (`policy.yield_on_request`): an idle drone on a spot makes room for an arriving
//! drone. Phase 6a's 1370 Wh week with 1.5 kW chargers, two OUT spots and no parking lost 7 of
//! its 10 drones to forced landings at seed 2: a drone stranded on IN blocked every loaded
//! arrival until it reached reserve. With the fix there are none.

use drone_sim::output::Outputs;
use drone_sim::scenario::Scenario;
use std::path::Path;

const BASE_DIR: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../configs/parts_logistics_pittsburgh_battery_1370wh"
);

/// The Phase 6a sweep point: 1.5 kW, no parking, two OUT spots, repair on.
fn gridlock_config(yield_on_request: bool) -> String {
    let text = std::fs::read_to_string(Path::new(BASE_DIR).join("scenario.toml")).expect("base");
    let mut t: toml::Table = text.parse().expect("toml");
    t.insert("schema_version".into(), "1.7.0".into());
    let q = |v: toml::Value, unit: &str| {
        let mut q = toml::Table::new();
        q.insert("value".into(), v);
        q.insert("unit".into(), unit.into());
        q.insert("assumption".into(), true.into());
        toml::Value::Table(q)
    };
    let section = |t: &toml::Table, name: &str| -> toml::Table {
        t.get(name)
            .and_then(toml::Value::as_table)
            .cloned()
            .expect(name)
    };
    let mut pad_ops = section(&t, "pad_ops");
    pad_ops.insert("out_spots".into(), q(2.into(), "count"));
    pad_ops.insert("parking_capacity".into(), q(0.into(), "count"));
    pad_ops.insert("repair_mean_s".into(), q(7200.0.into(), "s"));
    pad_ops.insert("repair_spread_s".into(), q(3600.0.into(), "s"));
    t.insert("pad_ops".into(), pad_ops.into());
    let mut charging = section(&t, "charging");
    charging.insert("power_w".into(), q(1500.0.into(), "W"));
    t.insert("charging".into(), charging.into());
    let mut policy = section(&t, "policy");
    policy.insert("yield_on_request".into(), yield_on_request.into());
    t.insert("policy".into(), policy.into());
    toml::to_string(&t).expect("serialise")
}

fn forced_landings(yield_on_request: bool) -> usize {
    let text = gridlock_config(yield_on_request);
    let s = Scenario::from_toml_in(&text, Some(Path::new(BASE_DIR)))
        .expect("loads")
        .with_seed(2);
    let hash = s.cfg.source_hash;
    let mut w = s.into_world(0, Outputs::in_memory(hash));
    w.run().expect("runs");
    String::from_utf8(w.out.alerts.bytes().to_vec())
        .expect("utf8")
        .lines()
        .filter(|l| l.contains("\"code\":\"DRONE_GROUNDED\""))
        .count()
}

#[test]
fn idle_drones_make_room_so_none_is_force_landed() {
    assert_eq!(forced_landings(true), 0);
}

#[test]
fn without_the_fix_the_same_week_force_lands_drones() {
    assert!(
        forced_landings(false) > 0,
        "the control must reproduce Phase 6a's gridlock, or the test above proves nothing"
    );
}
