//! Traffic (spec 2.11, 9.2 version_minor 2, A84 to A86): an old minor-1 truck matrix still
//! loads and gives the same run, peak hours slow the truck by the route's road classes,
//! weekends follow their own profile, and incidents are seeded per pair and hour.

mod common;
use common::quantity;
use drone_sim::fm::truck_baseline;
use drone_sim::ids::ms_from_s;
use drone_sim::output::Outputs;
use drone_sim::scenario::Scenario;
use std::path::{Path, PathBuf};
use toml::{Table, Value};

const SCENARIO_DIR: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../configs/parts_logistics_pittsburgh"
);
const HEADER_BYTES: usize = 32;
const HOUR_MS: u64 = 3_600_000;
const DAY_MS: u64 = 24 * HOUR_MS;
const PEAK_DELAY: [f64; 4] = [0.8, 0.5, 0.3, 1.0];
/// Morning peak at local 08:00 only; every other hour free flow.
const PEAK_HOUR: usize = 8;

fn scenario_table() -> Table {
    std::fs::read_to_string(Path::new(SCENARIO_DIR).join("scenario.toml"))
        .expect("scenario")
        .parse()
        .expect("toml")
}

fn load(t: &Table) -> Scenario {
    let text = toml::to_string(t).expect("serialise");
    Scenario::from_toml_in(&text, Some(Path::new(SCENARIO_DIR)))
        .unwrap_or_else(|e| panic!("rejected: {e}"))
}

/// The derived directory copied, with `truck_matrix.bin` cut back to version_minor 1.
fn minor_1_network() -> PathBuf {
    let dir = std::env::temp_dir().join(format!("dd_truck_minor1_{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("temp dir");
    let src = Path::new(SCENARIO_DIR).join("derived");
    for name in ["pads.toml", "lanes.csr.bin", "truck_matrix.bin"] {
        std::fs::copy(src.join(name), dir.join(name)).expect("copy");
    }
    let path = dir.join("truck_matrix.bin");
    let mut bytes = std::fs::read(&path).expect("read");
    assert_eq!(
        u16::from_le_bytes([bytes[10], bytes[11]]),
        2,
        "committed file is minor 2"
    );
    let n = u32::from_le_bytes(bytes[16..20].try_into().expect("4 bytes")) as usize;
    let array = 4 * n * n;
    let end = (HEADER_BYTES + array).next_multiple_of(8) + array;
    bytes[10..12].copy_from_slice(&1u16.to_le_bytes());
    bytes.truncate(end);
    std::fs::write(&path, bytes).expect("write");
    dir
}

fn one_day_deliveries(t: &Table) -> String {
    let s = load(t).with_seed(3);
    let hash = s.cfg.source_hash;
    let mut w = s.into_world(0, Outputs::in_memory(hash));
    w.run().expect("runs");
    String::from_utf8(w.out.deliveries.bytes().to_vec()).expect("utf8")
}

#[test]
fn an_old_minor_1_truck_matrix_still_loads_and_gives_the_same_run() {
    let mut t = scenario_table();
    common::set_q(&mut t, "sim.duration_s", 86_400.0);
    let current = one_day_deliveries(&t);
    let dir = minor_1_network();
    common::set(
        &mut t,
        "network_files.dir",
        dir.to_str().expect("utf8 path"),
    );
    let old = one_day_deliveries(&t);
    assert!(current.lines().count() > 10);
    assert_eq!(old, current);
}

fn shape(peak: f64) -> Value {
    Value::Array(
        (0..24)
            .map(|h| quantity(if h == PEAK_HOUR { peak } else { 0.0 }, "1"))
            .collect(),
    )
}

/// The Pittsburgh scenario with schema 1.8.0, t = 0 at midnight on a Monday, and traffic.
fn with_traffic(incident_rate_per_h: f64) -> Table {
    let mut t = scenario_table();
    common::set(&mut t, "schema_version", "1.8.0");
    common::set_q(&mut t, "sim.start_time_of_day_s", 0.0);
    for h in 0..24 {
        common::set_q(&mut t, &format!("truck.tod_speed_factors.{h}"), 1.0);
    }
    let mut tr = Table::new();
    tr.insert("start_weekday".into(), quantity(0, "count"));
    tr.insert("weekday_shape".into(), shape(1.0));
    tr.insert("weekend_shape".into(), shape(0.25));
    for (key, d) in [
        "peak_delay_motorway",
        "peak_delay_primary",
        "peak_delay_other",
        "peak_delay_structure",
    ]
    .into_iter()
    .zip(PEAK_DELAY)
    {
        tr.insert(key.into(), quantity(d, "1"));
    }
    tr.insert(
        "incident_rate_per_pair_h".into(),
        quantity(incident_rate_per_h, "1/h"),
    );
    tr.insert("incident_delay_median_s".into(), quantity(600.0, "s"));
    tr.insert("incident_delay_sigma".into(), quantity(0.5, "1"));
    t.insert("traffic".into(), Value::Table(tr));
    t
}

/// Road time of a truck leaving at `t_dep_ms` from pad 0 to pad `to`, in s.
fn drive_s(s: &Scenario, to: u16, t_dep_ms: u64) -> f64 {
    let tk = &s.cfg.truck;
    let lead_ms = ms_from_s(tk.dispatch_delay_s + tk.load_s);
    let unload_ms = ms_from_s(tk.unload_s);
    let (arrive, _) = truck_baseline(&s.cfg, &s.net, 7, 0, to, t_dep_ms - lead_ms);
    (arrive - t_dep_ms - unload_ms) as f64 / 1000.0
}

#[test]
fn the_peak_slows_the_truck_by_its_route_classes_and_weekends_less() {
    let s = load(&with_traffic(0.0));
    let f = s
        .cfg
        .network_files
        .as_ref()
        .unwrap()
        .truck
        .as_ref()
        .unwrap();
    let fr = f.fractions.as_ref().expect("minor 2 matrix");
    let n = s.net.n_pads;
    for to in 1..u16::try_from(n).unwrap() {
        let k = usize::from(to);
        let night = drive_s(&s, to, 3 * HOUR_MS);
        let expect = 1.0
            + (0..3)
                .map(|c| f64::from(fr.class[c][k]) * PEAK_DELAY[c])
                .sum::<f64>()
            + f64::from(fr.structure[k]) * PEAK_DELAY[3];
        let monday_peak = drive_s(&s, to, PEAK_HOUR as u64 * HOUR_MS);
        assert!((monday_peak / night - expect).abs() < 1e-3, "pad {to}");
        assert!(monday_peak > night * 1.2);
        // Saturday (day 5) has a quarter of the peak.
        let saturday_peak = drive_s(&s, to, 5 * DAY_MS + PEAK_HOUR as u64 * HOUR_MS);
        assert!((saturday_peak / night - (1.0 + (expect - 1.0) * 0.25)).abs() < 1e-3);
    }
}

#[test]
fn incidents_are_seeded_per_pair_and_hour_and_only_ever_add_time() {
    let calm = load(&with_traffic(0.0));
    let busy = load(&with_traffic(2.0));
    let (mut delayed, mut total) = (0, 0);
    for hour in 1..49 {
        let t = hour * HOUR_MS + 1;
        let d = drive_s(&busy, 1, t) - drive_s(&calm, 1, t);
        assert!(d >= -1e-3);
        assert_eq!(drive_s(&busy, 1, t), drive_s(&busy, 1, t + 1_000_000));
        total += 1;
        delayed += usize::from(d > 1.0);
    }
    // At 2 per hour, P(no incident) = e^-2, about 14%.
    assert!(
        delayed > total * 3 / 4,
        "{delayed} of {total} hours delayed"
    );
}

#[test]
fn traffic_needs_flat_time_of_day_factors() {
    let mut t = with_traffic(0.0);
    common::set_q(&mut t, "truck.tod_speed_factors.8", 0.7);
    let text = toml::to_string(&t).expect("serialise");
    let e = Scenario::from_toml_in(&text, Some(Path::new(SCENARIO_DIR)))
        .err()
        .expect("rejected");
    assert!(e.to_string().contains("R22"), "{e}");
}

/// Monday 2024-03-04 00:00 EST; the week holds the spring-forward change (Sunday 2024-03-10).
const SPRING_WEEK_UNIX_S: f64 = 1_709_528_400.0;

#[test]
fn peaks_stay_at_the_same_local_clock_time_across_spring_forward() {
    let mut t = with_traffic(0.0);
    let Some(Value::Table(tr)) = t.get_mut("traffic") else {
        panic!("traffic section")
    };
    tr.insert("start_unix_s".into(), quantity(SPRING_WEEK_UNIX_S, "s"));
    common::set(&mut t, "schema_version", "1.9.0");
    let s = load(&t);
    let night = drive_s(&s, 1, 3 * HOUR_MS);
    // 08:00 local is 13:00 UTC on Friday (EST) and 12:00 UTC on Monday (EDT).
    let friday_peak = drive_s(&s, 1, 4 * DAY_MS + PEAK_HOUR as u64 * HOUR_MS);
    let monday_peak = drive_s(&s, 1, 7 * DAY_MS + (PEAK_HOUR as u64 - 1) * HOUR_MS);
    let monday_hour_after = drive_s(&s, 1, 7 * DAY_MS + PEAK_HOUR as u64 * HOUR_MS);
    assert!(friday_peak > night * 1.05);
    assert!(
        (monday_peak - friday_peak).abs() < 1e-3,
        "peak at 08:00 local after the change"
    );
    assert!(
        (monday_hour_after - night).abs() < 1e-3,
        "09:00 local has no peak"
    );
}

#[test]
fn start_unix_s_must_agree_with_start_weekday() {
    let mut t = with_traffic(0.0);
    common::set(&mut t, "schema_version", "1.9.0");
    let Some(Value::Table(tr)) = t.get_mut("traffic") else {
        panic!("traffic section")
    };
    tr.insert("start_weekday".into(), quantity(2, "count"));
    tr.insert("start_unix_s".into(), quantity(SPRING_WEEK_UNIX_S, "s"));
    let text = toml::to_string(&t).expect("serialise");
    let e = Scenario::from_toml_in(&text, Some(Path::new(SCENARIO_DIR)))
        .err()
        .expect("rejected");
    assert!(e.to_string().contains("R23"), "{e}");
}
