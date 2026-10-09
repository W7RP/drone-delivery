//! Spec 8.7 and A93: a VTOL profile's transition overhead is part of the visit estimate, is
//! flown once per flight (with the first lane from a pad), and is counted twice by the energy
//! guard (out and on a diversion). Without the keys nothing changes.

mod common;
use common::*;
use drone_sim::flight::{EnergyRules, VisitEstimate};
use drone_sim::output::Outputs;
use drone_sim::scenario::Scenario;
use std::path::{Path, PathBuf};
use toml::Value;

const TRANSITION_S: f64 = 30.0;
const TRANSITION_POWER_W: f64 = 1500.0;
const LANE_M: f64 = 1000.0;
const PAYLOAD_KG: f64 = 5.0;

/// The placeholder flight model of the A-B example as a profile, plus `extra` drone lines.
fn profile(name: &str, extra: &str) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("vtol_profile");
    std::fs::create_dir_all(&dir).unwrap();
    let t = example();
    let d = t["drone"].as_table().unwrap();
    let mut lines = vec![
        format!("name = \"{name}\""),
        "source = \"test\"".into(),
        "[drone]".into(),
    ];
    for key in [
        "cruise_speed_mps",
        "cruise_power_ref_w",
        "hover_power_ref_w",
        "power_ref_mass_kg",
        "takeoff_s",
        "approach_s",
        "hop_s",
    ] {
        let q = d[key].as_table().unwrap();
        let value = q["value"].as_float().unwrap();
        let unit = q["unit"].as_str().unwrap();
        lines.push(format!(
            "{key} = {{ value = {value:?}, unit = \"{unit}\", assumption = true }}"
        ));
    }
    let text = lines.join("\n") + "\n" + extra;
    let path = dir.join(format!("{name}.toml"));
    std::fs::write(&path, text).unwrap();
    path
}

fn scenario(profile_path: &Path) -> Scenario {
    let mut t = base();
    set(&mut t, "schema_version", "1.9.0");
    // B 1 km from A, as in tests/flight_profile.rs.
    set_q(&mut t, "pads.1.lon_deg", -79.9841);
    let mut p = toml::Table::new();
    p.insert("file".into(), profile_path.to_str().unwrap().into());
    t.insert("flight_profile".into(), Value::Table(p));
    Scenario::from_toml_in(&toml::to_string(&t).unwrap(), Some(Path::new("/")))
        .unwrap_or_else(|e| panic!("rejected: {e}"))
}

fn vtol_lines() -> String {
    format!(
        "airframe = \"VTOL\"\ntransition_s = {{ value = {TRANSITION_S}, unit = \"s\", assumption = true }}\n\
         transition_power_ref_w = {{ value = {TRANSITION_POWER_W}, unit = \"W\", assumption = true }}\n"
    )
}

#[test]
fn the_visit_estimate_carries_one_transition() {
    let plain = scenario(&profile("plain", ""));
    let vtol = scenario(&profile("vtol", &vtol_lines()));
    let a = VisitEstimate::new(&plain.cfg, LANE_M, PAYLOAD_KG);
    let b = VisitEstimate::new(&vtol.cfg, LANE_M, PAYLOAD_KG);
    assert_eq!(a.transition.time_s, 0.0);
    assert!((b.total().time_s - a.total().time_s - TRANSITION_S).abs() < 1e-9);
    let rules = EnergyRules::from_config(&vtol.cfg);
    let e = rules.model.transition_energy_j(PAYLOAD_KG);
    assert!(
        (b.transition.energy_j - e).abs() < 1e-6 && e > TRANSITION_POWER_W * TRANSITION_S * 0.5
    );
    // The guard holds back a transition out and one on a diversion.
    let lost_m = EnergyRules::from_config(&plain.cfg).max_lane_m(0.95, PAYLOAD_KG)
        - rules.max_lane_m(0.95, PAYLOAD_KG);
    let per_m = rules.model.segment(1.0, PAYLOAD_KG).1;
    assert!((lost_m * per_m - 2.0 * e).abs() < 1e-3, "{lost_m} m");
}

/// Lane records of a 3 h run: (t_start, t_end) in ms.
fn lanes(s: Scenario) -> Vec<(u64, u64)> {
    let hash = s.cfg.source_hash;
    let mut w = s.into_world(0, Outputs::in_memory(hash));
    w.run().expect("runs");
    w.out.flights.bytes()[32..]
        .chunks(48)
        .filter(|c| c.len() == 48)
        .map(|c| {
            (
                u64::from_le_bytes(c[0..8].try_into().unwrap()),
                u64::from_le_bytes(c[8..16].try_into().unwrap()),
            )
        })
        .collect()
}

#[test]
fn each_flight_flies_its_transition_once() {
    let plain = lanes(scenario(&profile("plain_run", "")));
    let vtol = lanes(scenario(&profile("vtol_run", &vtol_lines())));
    assert!(!vtol.is_empty());
    let dur = |l: &(u64, u64)| l.1 - l.0;
    // A and B are one lane apart, so every lane starts a flight and carries the transition.
    let lane_ms = dur(&plain[0]);
    assert!(
        vtol.iter().all(|l| dur(l) == lane_ms + 30_000),
        "{:?}",
        &vtol[..3]
    );
}
