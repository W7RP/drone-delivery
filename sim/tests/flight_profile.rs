//! Spec 8.7: a scenario selects a flight profile file; its values replace the drone section's
//! flight-model keys, which otherwise stay as the scenario's placeholder profile. A profile with
//! a missing key, wrong unit or no source is rejected.

mod common;
use common::*;
use drone_sim::scenario::Scenario;
use std::path::{Path, PathBuf};
use toml::Value;

const FITTED: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../configs/flight_profiles/px4_sitl_proxy.toml"
);

/// The A-B example with B 1 km from A: its 6 km lane needs more battery than the fitted
/// profile leaves (R12), which is a result of Phase 5, not of this test.
fn short_example() -> toml::Table {
    let mut t = example();
    set_q(&mut t, "pads.1.lon_deg", -79.9841);
    set(&mut t, "schema_version", "1.5.0");
    t
}

fn with_profile(file: &str) -> toml::Table {
    let mut t = short_example();
    let mut p = toml::Table::new();
    p.insert("file".into(), file.into());
    t.insert("flight_profile".into(), Value::Table(p));
    t
}

fn load(t: &toml::Table) -> Result<Scenario, String> {
    Scenario::from_toml_in(&toml::to_string(t).unwrap(), Some(Path::new("/"))).map_err(|e| e.0)
}

fn profile_file(name: &str, text: &str) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("flight_profile");
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join(name);
    std::fs::write(&path, text).unwrap();
    path
}

#[test]
fn the_fitted_profile_replaces_the_flight_model_keys() {
    let placeholder = load(&short_example()).unwrap();
    let fitted = load(&with_profile(FITTED)).unwrap();
    let profile: toml::Table = std::fs::read_to_string(FITTED).unwrap().parse().unwrap();
    let value = |k: &str| profile["drone"][k]["value"].as_float().unwrap();
    let d = &fitted.cfg.drone;
    assert!((d.cruise_speed_mps - value("cruise_speed_mps")).abs() < 1e-12);
    assert!((d.cruise_power_ref_w - value("cruise_power_ref_w")).abs() < 1e-12);
    assert!((d.hover_power_ref_w - value("hover_power_ref_w")).abs() < 1e-12);
    assert!((d.approach_s - value("approach_s")).abs() < 1e-12);
    // Keys a profile does not set keep the scenario's values.
    let p = &placeholder.cfg.drone;
    assert!((d.battery_capacity_j - p.battery_capacity_j).abs() < 1e-12);
    assert!((d.go_around_s - p.go_around_s).abs() < 1e-12);
    let fp = fitted
        .cfg
        .flight_profile
        .as_ref()
        .expect("profile recorded");
    assert_eq!(
        (fp.name.as_str(), fp.source.as_str()),
        ("px4_sitl_proxy", "px4-sitl-proxy")
    );
    assert!(placeholder.cfg.flight_profile.is_none());
}

#[test]
fn a_broken_profile_is_rejected_by_name() {
    let good = std::fs::read_to_string(FITTED).unwrap();
    let cases = [
        (
            "missing_key.toml",
            good.replace("hop_s =", "# hop_s ="),
            "hop_s: missing",
        ),
        (
            "wrong_unit.toml",
            good.replace("unit = \"m/s\"", "unit = \"km/h\""),
            "unit \"km/h\"",
        ),
        (
            "no_source.toml",
            good.replace("source =", "# source ="),
            "source: missing",
        ),
        (
            "extra_key.toml",
            good.replace(
                "[drone]",
                "[drone]\nwing_span_m = { value = 2.0, unit = \"m\", assumption = true }",
            ),
            "wing_span_m: unknown key",
        ),
        (
            "half_transition.toml",
            good.replace(
                "[drone]",
                "[drone]\ntransition_s = { value = 20.0, unit = \"s\", assumption = true }",
            ),
            "transition_power_ref_w: missing",
        ),
    ];
    for (name, text, want) in cases {
        let path = profile_file(name, &text);
        let msg = load(&with_profile(path.to_str().unwrap()))
            .err()
            .expect(name);
        assert!(msg.contains(want) && msg.contains(name), "{name}: {msg}");
    }
}
