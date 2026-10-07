//! Spec 8.4, 9.0 to 9.2: a `network_files` scenario takes pads from `pads.toml`, lanes from
//! `lanes.csr.bin` and the truck matrix from `truck_matrix.bin`, and the loader refuses files
//! that break the reader rules or do not belong to the catalog.

mod common;
use common::*;
use drone_sim::config::fnv1a64;
use drone_sim::output::Outputs;
use drone_sim::scenario::Scenario;
use std::path::{Path, PathBuf};
use toml::Value;

/// Three pads; the lane lengths and truck times below differ from anything the pad positions
/// would give, so a test can tell the file was used.
const CATALOG: &str = r#"[[pads]]
name = "A"
lat_deg = { value = 40.4406, unit = "deg", assumption = false }
lon_deg = { value = -79.9959, unit = "deg", assumption = false }
is_hub = true
service_mode = "AUTO"
pod_return_policy = "KEEP_LOCAL"
initial_empty_pods = { value = 3, unit = "count", assumption = true }
parking_capacity = { value = 2, unit = "count", assumption = true }

[[pads]]
name = "B"
lat_deg = { value = 40.4406, unit = "deg", assumption = false }
lon_deg = { value = -79.9250, unit = "deg", assumption = false }
is_hub = false
service_mode = "AUTO"
pod_return_policy = "KEEP_LOCAL"
initial_empty_pods = { value = 3, unit = "count", assumption = true }
parking_capacity = { value = 2, unit = "count", assumption = true }

[[pads]]
name = "C"
lat_deg = { value = 40.4600, unit = "deg", assumption = false }
lon_deg = { value = -79.9600, unit = "deg", assumption = false }
is_hub = false
service_mode = "AUTO"
pod_return_policy = "KEEP_LOCAL"
initial_empty_pods = { value = 3, unit = "count", assumption = true }
parking_capacity = { value = 2, unit = "count", assumption = true }
"#;

/// Undirected lanes A-B 7000 m, A-C 2500 m; no B-C lane, so A -> B -> ... routes are forced.
const LANE_AB_M: f32 = 7000.0;
const LANE_AC_M: f32 = 2500.0;
const TRUCK_S: [[f32; 3]; 3] = [
    [0.0, 900.0, 400.0],
    [950.0, 0.0, 1100.0],
    [420.0, 1150.0, 0.0],
];
const ROAD_M: [[f32; 3]; 3] = [
    [0.0, 9000.0, 3300.0],
    [9100.0, 0.0, 12000.0],
    [3400.0, 12100.0, 0.0],
];

fn header(magic: &[u8; 8], minor: u16, count_a: u32, count_b: u32, hash: u64) -> Vec<u8> {
    let mut b = Vec::new();
    b.extend_from_slice(magic);
    b.extend_from_slice(&1u16.to_le_bytes());
    b.extend_from_slice(&minor.to_le_bytes());
    b.extend_from_slice(&32u32.to_le_bytes());
    b.extend_from_slice(&count_a.to_le_bytes());
    b.extend_from_slice(&count_b.to_le_bytes());
    b.extend_from_slice(&hash.to_le_bytes());
    b
}

fn align(b: &mut Vec<u8>) {
    b.resize(b.len().next_multiple_of(8), 0);
}

/// Directed lanes sorted by (from, to): 0 A->B, 1 A->C, 2 B->A, 3 C->A.
fn lanes_bin(hash: u64) -> Vec<u8> {
    let mut b = header(b"DDLANES\0", 1, 3, 4, hash);
    for v in [0u32, 2, 3, 4] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    align(&mut b);
    for v in [1u16, 2, 0, 0] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    align(&mut b);
    for v in [LANE_AB_M, LANE_AC_M, LANE_AB_M, LANE_AC_M] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    align(&mut b);
    for v in [2u32, 3, 0, 1] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    for array in [[310.0f32; 4], [1500.0f32; 4]] {
        align(&mut b);
        for v in array {
            b.extend_from_slice(&v.to_le_bytes());
        }
    }
    b
}

fn truck_bin(hash: u64) -> Vec<u8> {
    let mut b = header(b"DDTRUCK\0", 1, 3, 0, hash);
    for m in [TRUCK_S, ROAD_M] {
        align(&mut b);
        for row in m {
            for v in row {
                b.extend_from_slice(&v.to_le_bytes());
            }
        }
    }
    b
}

/// A fresh directory with the catalog and both binary files, each passed through `edit`.
fn network_dir(
    name: &str,
    catalog: &str,
    edit_lanes: impl FnOnce(&mut Vec<u8>),
    edit_truck: impl FnOnce(&mut Vec<u8>),
) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join("network_files")
        .join(name);
    std::fs::create_dir_all(&dir).expect("create test dir");
    let hash = fnv1a64(CATALOG.as_bytes());
    let (mut lanes, mut truck) = (lanes_bin(hash), truck_bin(hash));
    edit_lanes(&mut lanes);
    edit_truck(&mut truck);
    std::fs::write(dir.join("pads.toml"), catalog).unwrap();
    std::fs::write(dir.join("lanes.csr.bin"), lanes).unwrap();
    std::fs::write(dir.join("truck_matrix.bin"), truck).unwrap();
    dir
}

/// The A-B example with its network replaced by `network_files` and a FILE truck matrix.
fn files_scenario(dir: &Path) -> toml::Table {
    let mut t = base();
    t.remove("pads");
    t.remove("lanes");
    let mut nf = toml::Table::new();
    nf.insert("dir".into(), dir.to_str().unwrap().into());
    t.insert("network_files".into(), Value::Table(nf));
    let truck = t["truck"].as_table_mut().unwrap();
    truck.insert("matrix_source".into(), "FILE".into());
    truck.remove("synthetic_detour_factor");
    truck.remove("synthetic_free_flow_speed_mps");
    t
}

fn load(t: &toml::Table) -> Result<Scenario, String> {
    // The directory in the config is absolute, so the scenario directory does not matter.
    Scenario::from_toml_in(&toml::to_string(t).unwrap(), Some(Path::new("/"))).map_err(|e| e.0)
}

fn rejected_with(
    name: &str,
    lanes: impl FnOnce(&mut Vec<u8>),
    truck: impl FnOnce(&mut Vec<u8>),
) -> String {
    let dir = network_dir(name, CATALOG, lanes, truck);
    match load(&files_scenario(&dir)) {
        Ok(_) => panic!("{name}: files were accepted"),
        Err(e) => e,
    }
}

fn set_u16(at: usize, v: u16) -> impl FnOnce(&mut Vec<u8>) {
    move |b: &mut Vec<u8>| b[at..at + 2].copy_from_slice(&v.to_le_bytes())
}

fn set_u32(at: usize, v: u32) -> impl FnOnce(&mut Vec<u8>) {
    move |b: &mut Vec<u8>| b[at..at + 4].copy_from_slice(&v.to_le_bytes())
}

fn keep(_: &mut Vec<u8>) {}

#[test]
fn hand_made_files_round_trip_into_the_network() {
    let dir = network_dir("round_trip", CATALOG, keep, keep);
    let s = load(&files_scenario(&dir)).expect("hand-made files load");
    assert_eq!(s.cfg.pads.len(), 3);
    assert!(s.cfg.pads[0].is_hub && s.cfg.pads[2].name == "C");
    assert_eq!(s.cfg.lanes, vec![(0, 1), (0, 2)]);
    assert_eq!(
        s.net.lane_len_m,
        vec![LANE_AB_M, LANE_AC_M, LANE_AB_M, LANE_AC_M]
    );
    assert_eq!(s.net.reverse_lane, vec![2, 3, 0, 1]);
    // B to C has no lane: the route runs through A.
    let via_a = f64::from(LANE_AB_M) + f64::from(LANE_AC_M);
    assert!((s.net.route_len(1, 2) - via_a).abs() < 1e-9);
    for a in 0..3u16 {
        for b in 0..3u16 {
            let (i, j) = (usize::from(a), usize::from(b));
            assert!((s.net.truck_s(a, b) - f64::from(TRUCK_S[i][j])).abs() < 1e-9);
            assert!((s.net.road(a, b) - f64::from(ROAD_M[i][j])).abs() < 1e-9);
        }
    }
    let mut world = s.into_world(0, Outputs::in_memory(0));
    let summary = world.run().expect("in-memory run");
    assert!(summary["deliveries_per_h"].as_f64().unwrap() > 0.0);
}

#[test]
fn a_later_minor_version_may_append_arrays() {
    let dir = network_dir(
        "later_minor",
        CATALOG,
        |b| {
            set_u16(10, 2)(b);
            b.extend_from_slice(&[0u8; 16]);
        },
        keep,
    );
    load(&files_scenario(&dir)).expect("minor 2 with extra arrays loads");
}

#[test]
fn files_that_break_the_reader_rules_are_rejected() {
    // Lane file offsets: header 32, row_offsets 32..48, to_pad 48..56, length_m 56..72,
    // reverse_lane 72..88.
    let cases: Vec<(&str, String)> = vec![
        ("major", rejected_with("major", set_u16(8, 2), keep)),
        (
            "lanes_minor0",
            rejected_with("lanes_minor0", set_u16(10, 0), keep),
        ),
        (
            "truck_minor0",
            rejected_with("truck_minor0", keep, set_u16(10, 0)),
        ),
        (
            "short",
            rejected_with("short", |b| b.truncate(b.len() - 4), keep),
        ),
        (
            "trailing",
            rejected_with("trailing", |b| b.extend_from_slice(&[0; 8]), keep),
        ),
        (
            "magic",
            rejected_with("magic", |b| b[0..8].copy_from_slice(b"DDTRUCK\0"), keep),
        ),
        (
            "pad_count",
            rejected_with("pad_count", set_u32(16, 4), keep),
        ),
        (
            "truck_pad_count",
            rejected_with("truck_pad_count", keep, set_u32(16, 2)),
        ),
        ("hash", rejected_with("hash", |b| b[24] ^= 1, keep)),
        (
            "nan",
            rejected_with("nan", set_u32(56, f32::NAN.to_bits()), keep),
        ),
        ("reverse", rejected_with("reverse", set_u32(72, 1), keep)),
        (
            "self_lane",
            rejected_with("self_lane", set_u16(48, 0), keep),
        ),
        (
            "diagonal",
            rejected_with("diagonal", keep, set_u32(32, 5.0f32.to_bits())),
        ),
    ];
    let expect = [
        ("major", "version 2.1"),
        ("lanes_minor0", "version 1.0"),
        ("truck_minor0", "version 1.0"),
        ("short", "size mismatch"),
        ("trailing", "size mismatch"),
        ("magic", "magic"),
        ("pad_count", "count_a 4 pads, the catalog has 3"),
        ("truck_pad_count", "count_a 2 pads, the catalog has 3"),
        ("hash", "source_hash"),
        ("nan", "NaN"),
        ("reverse", "reverse_lane"),
        ("self_lane", "self lane"),
        ("diagonal", "diagonal must be 0"),
    ];
    for ((name, msg), (_, want)) in cases.iter().zip(expect) {
        assert!(
            msg.contains(want),
            "{name}: {msg:?} does not mention {want:?}"
        );
        let file = if name.starts_with("truck") || *name == "diagonal" {
            "truck_matrix.bin"
        } else {
            "lanes.csr.bin"
        };
        assert!(msg.contains(file), "{name}: {msg:?} does not name {file}");
    }
}

#[test]
fn an_edited_catalog_no_longer_matches_its_files() {
    let edited = CATALOG.replace("40.4600", "40.4601");
    let dir = network_dir("edited_catalog", &edited, keep, keep);
    let msg = load(&files_scenario(&dir)).err().expect("rejected");
    assert!(
        msg.contains("source_hash") && msg.contains("rebuild"),
        "{msg}"
    );
}

#[test]
fn network_files_excludes_the_other_network_forms_and_needs_a_directory() {
    let dir = network_dir("forms", CATALOG, keep, keep);
    let mut t = files_scenario(&dir);
    t.insert("pads".into(), example()["pads"].clone());
    let msg = load(&t).err().expect("rejected");
    assert!(msg.contains("exactly one of"), "{msg}");

    let t = files_scenario(&dir);
    let msg = Scenario::from_toml(&toml::to_string(&t).unwrap())
        .err()
        .expect("no scenario directory")
        .0;
    assert!(
        msg.contains("network_files: needs the scenario's directory"),
        "{msg}"
    );
}

#[test]
fn a_synthetic_truck_matrix_still_works_with_file_lanes() {
    let dir = network_dir("synthetic_truck", CATALOG, keep, |b| b.clear());
    let mut t = files_scenario(&dir);
    let synthetic = example()["truck"].clone();
    t.insert("truck".into(), synthetic);
    let s = load(&t).expect("FILE lanes with a SYNTHETIC truck matrix");
    assert!(s.net.truck_s(0, 1) > 0.0);
    assert_eq!(s.net.lane_len_m[0], LANE_AB_M);
}
