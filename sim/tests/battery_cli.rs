//! `sim battery` (spec A68): with a battery whose mass is coupled to its capacity, the size a
//! lane needs is iterated to a fixed point. It must close from either side: a configured
//! battery larger than the lane needs shrinks, which also lightens the drone (steps negative,
//! shrinking in size), as reliably as a smaller one grows.

use std::process::Command;

const SCENARIO: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../configs/parts_logistics_pittsburgh_900wh_layout/scenario.toml"
);

fn battery(from: &str, to: &str) -> serde_json::Value {
    let out = Command::new(env!("CARGO_BIN_EXE_sim"))
        .args([
            "battery",
            "--config",
            SCENARIO,
            "--payload-kg",
            "5",
            "--from",
            from,
            "--to",
            to,
        ])
        .output()
        .expect("sim runs");
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    serde_json::from_slice(&out.stdout).expect("json")
}

#[test]
fn a_battery_larger_than_needed_shrinks_to_its_fixed_point() {
    // A 0.9 km Oakland lane needs far less than the configured 900 Wh.
    let v = battery("UPMC_Oakland", "UPMC_Magee_Womens_Hospital");
    assert_eq!(v["fixed_point"], true, "{v:#}");
    let wh = v["sized"]["required_capacity_wh"].as_f64().expect("sized");
    assert!(wh < 900.0, "{wh} Wh");
}
