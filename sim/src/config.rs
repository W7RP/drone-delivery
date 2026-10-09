//! Scenario config loader (spec section 8). Reads `scenario.toml`, checks every key, unit and
//! range against `schemas/config.schema.json`, expands the synthetic generator (8.6) or reads
//! the pipeline's network files (8.4, 9.1, 9.2) and the weather series (9.7), applies the
//! cross-field rules R1 to R11, R13 and R17 to R21, and converts to SI seconds and integer ms.
//!
//! R12 and R16 need routes, so they are checked in `crate::scenario` after the network is built.

use std::fmt;
use std::path::{Path, PathBuf};

use toml::{Table, Value};

use crate::flight::FlightModel;
use crate::ids::{
    AssignmentPolicyName, ChargingMode, ConfigStatus, DispatchPolicyName, DiversionPolicyName,
    DroneRebalancingPolicyName, FaultKind, HoldingPolicyName, InOccupiedAction, MAX_OUT_SPOTS,
    MatrixSource, NetworkTopology, OutEmptyPolicy, PadId, PodRebalancingPolicyName, Preset,
    ReturnPolicy, ServiceMode, TimeMs, WeatherHoldPolicy, ms_from_s,
};
use crate::netfile::{self, LaneFile, TruckFile};
use crate::network;
use crate::traffic::{DELAY_CLASSES, TrafficCfg};
use crate::weather::{CruiseCurve, Limits, WeatherCfg};

pub const SCHEMA_VERSION: &str = "1.9.0";
/// Earlier minor versions this core still reads. Each later version only added optional keys,
/// which a file may use only when it declares that version (`VERSIONED_KEYS`).
const COMPATIBLE_VERSIONS: [&str; 5] = ["1.3.0", "1.5.0", "1.6.0", "1.7.0", "1.8.0"];
/// Optional keys and the schema version that added them.
const VERSIONED_KEYS: [(&str, &str, &str); 21] = [
    ("", "flight_profile", "1.5.0"),
    ("drone", "airframe_mass_kg", "1.6.0"),
    ("drone", "battery_specific_energy_wh_per_kg", "1.6.0"),
    ("pad_ops", "repair_mean_s", "1.6.0"),
    ("pad_ops", "repair_spread_s", "1.6.0"),
    ("pad_ops", "marker_self_check", "1.6.0"),
    ("handshake", "reorder_window_s", "1.6.0"),
    ("drone", "latch_capture_radius_m", "1.6.0"),
    ("drone", "out_landing_error_bias_m", "1.6.0"),
    ("drone", "out_landing_error_sigma_m", "1.6.0"),
    ("pad_ops", "out_spots", "1.6.0"),
    ("pad_ops", "parking_capacity", "1.6.0"),
    ("demand", "pods_per_origin", "1.6.0"),
    ("sim", "max_events_per_timestamp", "1.7.0"),
    ("sim", "max_event_log_mib", "1.7.0"),
    ("policy", "yield_on_request", "1.7.0"),
    ("", "weather", "1.8.0"),
    ("", "traffic", "1.8.0"),
    ("metrics", "order_outcomes", "1.9.0"),
    ("weather", "energy_route_guard", "1.9.0"),
    ("traffic", "start_unix_s", "1.9.0"),
];
/// Run guards (spec 5.5): events allowed at one sim timestamp, and the `events.bin` size cap.
const DEFAULT_MAX_EVENTS_PER_TIMESTAMP: u32 = 10_000;
const DEFAULT_MAX_EVENT_LOG_MIB: u64 = 256;
const BYTES_PER_MIB: u64 = 1 << 20;
const LATCH_DERIVATION_KEYS: [&str; 3] = [
    "latch_capture_radius_m",
    "out_landing_error_bias_m",
    "out_landing_error_sigma_m",
];

/// Schema versions are "1.<minor>.0"; compare by minor.
fn minor(version: &str) -> u32 {
    version
        .split('.')
        .nth(1)
        .and_then(|m| m.parse().ok())
        .unwrap_or(0)
}

/// A key added after the file's declared version is an error, so a file never relies on
/// behaviour its own version does not have.
fn check_versioned_keys(root: &Table, version: &str) -> Result<()> {
    for (section, key, added) in VERSIONED_KEYS {
        let table = if section.is_empty() {
            Some(root)
        } else {
            root.get(section).and_then(Value::as_table)
        };
        let path = if section.is_empty() {
            key.to_owned()
        } else {
            format!("{section}.{key}")
        };
        if table.is_some_and(|t| t.contains_key(key)) && minor(version) < minor(added) {
            return err(format!(
                "{path}: needs schema_version \"{added}\", the file says \"{version}\""
            ));
        }
    }
    Ok(())
}
const PADS_FILE: &str = "pads.toml";
const LANES_FILE: &str = "lanes.csr.bin";
const TRUCK_FILE: &str = "truck_matrix.bin";
const HOURS_PER_DAY: usize = 24;
const SECONDS_PER_DAY: f64 = 86_400.0;
const SECONDS_PER_HOUR: f64 = 3600.0;
const J_PER_WH: f64 = 3600.0;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigError(pub String);

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ConfigError {}

type Result<T> = std::result::Result<T, ConfigError>;

fn err<T>(msg: impl Into<String>) -> Result<T> {
    Err(ConfigError(msg.into()))
}

#[derive(Debug, Clone)]
pub struct PadCfg {
    pub name: String,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub is_hub: bool,
    pub service_mode: ServiceMode,
    pub return_policy: ReturnPolicy,
    pub initial_empty_pods: u32,
    pub parking_capacity: u16,
    /// OUT spots (`pad_ops.out_spots`, 1 to 3, A72).
    pub out_spots: u16,
}

impl PadCfg {
    /// `slots(p)` of spec 6.4: parking capacity, or the OUT spots when it is 0.
    pub fn slots(&self) -> u16 {
        if self.parking_capacity > 0 {
            self.parking_capacity
        } else {
            self.out_spots
        }
    }
}

#[derive(Debug, Clone)]
pub struct SyntheticNetworkCfg {
    pub topology: NetworkTopology,
    pub n_pads: u16,
    pub radius_m: f64,
    pub mesh_neighbors: u16,
    pub service_mode: ServiceMode,
    pub return_policy: ReturnPolicy,
    pub initial_empty_pods: u32,
    pub parking_capacity: u16,
    pub hub_parking_capacity: u16,
}

#[derive(Debug, Clone, Copy)]
pub struct Flow {
    pub origin: PadId,
    pub dest: PadId,
    pub rate_per_s: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct FaultCfg {
    pub kind: FaultKind,
    pub pad: PadId,
    pub start_ms: TimeMs,
    pub duration_ms: TimeMs,
}

#[derive(Debug, Clone)]
pub struct DroneCfg {
    pub fleet_size: u16,
    pub initial_pads: Vec<PadId>,
    /// Airframe plus battery. Given directly, or derived from the battery when `mass_coupling`
    /// is set (spec 8.2, A68).
    pub empty_mass_kg: f64,
    pub mass_coupling: Option<MassCoupling>,
    pub latch_mass_kg: f64,
    pub mtow_kg: f64,
    pub battery_capacity_j: f64,
    pub initial_soc: f64,
    pub reserve_soc: f64,
    pub charge_target_soc: f64,
    pub cruise_speed_mps: f64,
    pub cruise_power_ref_w: f64,
    pub hover_power_ref_w: f64,
    pub power_ref_mass_kg: f64,
    pub takeoff_s: f64,
    pub approach_s: f64,
    pub go_around_s: f64,
    pub hop_s: f64,
    pub p_release_fault: f64,
    /// Given, or derived from the OUT landing error and the latch capture radius (A71).
    pub p_latch_fault: f64,
    pub latch_derivation: Option<LatchDerivation>,
}

/// Where a derived `p_latch_fault` came from (A71).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LatchDerivation {
    pub capture_radius_m: f64,
    pub out_error_bias_m: f64,
    pub out_error_sigma_m: f64,
}

/// Battery mass that follows capacity (A68): empty mass = airframe + capacity / specific energy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassCoupling {
    pub airframe_mass_kg: f64,
    pub specific_energy_j_per_kg: f64,
}

impl MassCoupling {
    pub fn empty_mass_kg(&self, battery_capacity_j: f64) -> f64 {
        self.airframe_mass_kg + battery_capacity_j / self.specific_energy_j_per_kg
    }
}

impl DroneCfg {
    /// Change the battery, and with it the empty mass when the mass is coupled to it.
    pub fn set_battery_capacity(&mut self, capacity_j: f64) {
        self.battery_capacity_j = capacity_j;
        if let Some(c) = self.mass_coupling {
            self.empty_mass_kg = c.empty_mass_kg(capacity_j);
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ChargingCfg {
    pub mode: ChargingMode,
    pub power_w: f64,
    pub efficiency: f64,
    pub charge_while_parked: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct PadOpsCfg {
    pub pod_place_ms: TimeMs,
    pub pod_remove_ms: TimeMs,
    pub release_ms: TimeMs,
    pub release_verify_ms: TimeMs,
    pub latch_ms: TimeMs,
    pub latch_verify_ms: TimeMs,
    pub manual_release_ms: TimeMs,
    pub manual_latch_ms: TimeMs,
    pub p_marker_detect: f64,
    pub max_go_arounds: u8,
    pub release_max_retries: u8,
    pub latch_max_retries: u8,
    pub out_empty_policy: OutEmptyPolicy,
    pub out_empty_wait_ms: TimeMs,
    pub pc_fault_rate_per_s: f64,
    pub pc_fault_duration_ms: TimeMs,
    /// Repair after a release fault (A69); None keeps the pad out of service for the run.
    pub repair: Option<RepairCfg>,
    /// The pad checks its own fiducial and reports a marker fault to FM (A35); false: only a
    /// drone's failed approach reveals it.
    pub marker_self_check: bool,
    /// OUT spots at every pad (`pad_ops.out_spots`, default 1, A72).
    pub out_spots: u16,
    /// `pad_ops.parking_capacity`: replaces every pad's own parking capacity (A72).
    pub parking_capacity: Option<u16>,
}

/// Repair duration, uniform on [mean - spread, mean + spread] seconds (A69).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RepairCfg {
    pub mean_s: f64,
    pub spread_s: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct HandshakeCfg {
    pub local_latency_ms: TimeMs,
    pub fm_latency_ms: TimeMs,
    pub p_msg_loss: f64,
    pub response_timeout_ms: TimeMs,
    pub max_retries: u8,
    pub hold_retry_ms: TimeMs,
    pub pc_silence_alert_ms: TimeMs,
    /// How long a radio message that overtook an earlier one on its link is held for it (4.1);
    /// 0 when `handshake.reorder_window_s` is absent: deliver on arrival.
    pub reorder_window_ms: TimeMs,
}

#[derive(Debug, Clone, Copy)]
pub struct PolicyCfg {
    pub in_occupied_action: InOccupiedAction,
    pub max_hold_s: f64,
    pub marker_hold_s: f64,
    pub marker_max_hold_cycles: u8,
    pub pod_min_per_pad: u32,
    /// `policy.yield_on_request` (A74): idle drones make room for arriving drones and pods.
    pub yield_on_request: bool,
}

/// `truck.synthetic_*` (spec 9.2 SYNTHETIC); absent when the matrix comes from a file.
#[derive(Debug, Clone, Copy)]
pub struct SyntheticTruckCfg {
    pub detour_factor: f64,
    pub free_flow_speed_mps: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct TruckCfg {
    pub matrix_source: MatrixSource,
    pub synthetic: Option<SyntheticTruckCfg>,
    pub dispatch_delay_s: f64,
    pub load_s: f64,
    pub unload_s: f64,
    pub tod_speed_factors: [f64; HOURS_PER_DAY],
}

#[derive(Debug, Clone, Copy)]
pub struct CostsCfg {
    pub truck_per_m: f64,
    pub truck_per_s: f64,
    pub drone_per_s: f64,
    pub energy_per_j: f64,
}

/// The pipeline's network for a `network_files` scenario (spec 8.4): the lane graph always, the
/// truck matrix when `truck.matrix_source` = FILE.
#[derive(Debug, Clone)]
pub struct NetworkFiles {
    pub dir: PathBuf,
    pub catalog_hash: u64,
    pub lanes: LaneFile,
    pub truck: Option<TruckFile>,
}

/// A flight-model profile file (spec 8.7): its name and where its values came from.
#[derive(Debug, Clone)]
pub struct FlightProfile {
    pub path: PathBuf,
    pub name: String,
    pub source: String,
}

#[derive(Debug, Clone)]
pub struct Config {
    pub name: String,
    pub preset: Preset,
    pub source_hash: u64,
    pub seed: u64,
    pub duration_ms: TimeMs,
    pub warmup_ms: TimeMs,
    pub start_time_of_day_s: f64,
    pub replications: u32,
    /// Run guards (spec 5.5): a run aborts with an error past either.
    pub max_events_per_timestamp: u32,
    pub max_event_log_bytes: u64,
    pub origin_lat_deg: f64,
    pub origin_lon_deg: f64,
    pub pads: Vec<PadCfg>,
    pub lanes: Vec<(PadId, PadId)>,
    pub network_files: Option<NetworkFiles>,
    /// The flight profile applied over the drone section's flight-model keys, if any.
    pub flight_profile: Option<FlightProfile>,
    pub drone: DroneCfg,
    pub pod_gross_mass_kg: f64,
    pub pod_empty_mass_kg: f64,
    pub charging: ChargingCfg,
    pub pad_ops: PadOpsCfg,
    pub handshake: HandshakeCfg,
    pub policy: PolicyCfg,
    pub deadline_offset_ms: TimeMs,
    /// `demand.max_backlog_per_pad`: requests a pad holds before rejecting new ones (P08);
    /// None when the key is absent (no cap).
    pub max_backlog_per_pad: Option<u16>,
    pub flows: Vec<Flow>,
    pub truck: TruckCfg,
    pub hist_bin_ms: TimeMs,
    pub hist_max_ms: TimeMs,
    /// `metrics.order_outcomes` (M11): classify every request at the horizon.
    pub order_outcomes: bool,
    pub costs: CostsCfg,
    pub faults: Vec<FaultCfg>,
    /// `[weather]` (2.10); None runs without weather.
    pub weather: Option<WeatherCfg>,
    /// `[traffic]` (2.11); None runs without traffic.
    pub traffic: Option<TrafficCfg>,
}

impl Config {
    pub fn has_charger(&self, pad: PadId) -> bool {
        match self.charging.mode {
            ChargingMode::OutContacts => true,
            ChargingMode::HubOnly => self.pads[usize::from(pad)].is_hub,
        }
    }

    pub fn total_pods(&self) -> u32 {
        self.pads.iter().map(|p| p.initial_empty_pods).sum()
    }
}

/// FNV-1a 64 of the raw config bytes (spec 9.0 `source_hash`).
pub fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        hash ^= u64::from(b);
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

#[derive(Clone, Copy)]
enum Range {
    NonNeg,
    Pos,
    Prob,
    /// (0, 10], the schema's `q_factor`.
    Factor,
    Lat,
    Lon,
    Finite,
}

impl Range {
    fn check(self, v: f64) -> bool {
        match self {
            Range::NonNeg => v >= 0.0,
            Range::Pos => v > 0.0,
            Range::Prob => (0.0..=1.0).contains(&v),
            Range::Factor => v > 0.0 && v <= 10.0,
            Range::Lat => (-90.0..=90.0).contains(&v),
            Range::Lon => (-180.0..=180.0).contains(&v),
            Range::Finite => true,
        }
    }

    fn describe(self) -> &'static str {
        match self {
            Range::NonNeg => ">= 0",
            Range::Pos => "> 0",
            Range::Prob => "in [0, 1]",
            Range::Factor => "in (0, 10]",
            Range::Lat => "in [-90, 90]",
            Range::Lon => "in [-180, 180]",
            Range::Finite => "a number",
        }
    }
}

/// One TOML table with its dotted path, for precise error messages.
struct Section<'a> {
    table: &'a Table,
    path: String,
}

impl<'a> Section<'a> {
    fn new(table: &'a Table, path: impl Into<String>) -> Self {
        Self {
            table,
            path: path.into(),
        }
    }

    fn key_path(&self, key: &str) -> String {
        if self.path.is_empty() {
            key.to_owned()
        } else {
            format!("{}.{key}", self.path)
        }
    }

    fn allow(&self, keys: &[&str]) -> Result<()> {
        for k in self.table.keys() {
            if !keys.contains(&k.as_str()) {
                return err(format!("{}: unknown key", self.key_path(k)));
            }
        }
        Ok(())
    }

    fn get(&self, key: &str) -> Result<&'a Value> {
        self.table
            .get(key)
            .ok_or_else(|| ConfigError(format!("{}: missing", self.key_path(key))))
    }

    fn has(&self, key: &str) -> bool {
        self.table.contains_key(key)
    }

    fn section(&self, key: &str) -> Result<Section<'a>> {
        match self.get(key)? {
            Value::Table(t) => Ok(Section::new(t, self.key_path(key))),
            _ => err(format!("{}: expected a table", self.key_path(key))),
        }
    }

    fn tables(&self, key: &str) -> Result<Vec<Section<'a>>> {
        let Value::Array(items) = self.get(key)? else {
            return err(format!(
                "{}: expected an array of tables",
                self.key_path(key)
            ));
        };
        items
            .iter()
            .enumerate()
            .map(|(i, v)| match v {
                Value::Table(t) => Ok(Section::new(t, format!("{}[{i}]", self.key_path(key)))),
                _ => err(format!("{}[{i}]: expected a table", self.key_path(key))),
            })
            .collect()
    }

    fn string(&self, key: &str) -> Result<&'a str> {
        match self.get(key)? {
            Value::String(s) => Ok(s),
            _ => err(format!("{}: expected a string", self.key_path(key))),
        }
    }

    fn boolean(&self, key: &str) -> Result<bool> {
        match self.get(key)? {
            Value::Boolean(b) => Ok(*b),
            _ => err(format!("{}: expected true or false", self.key_path(key))),
        }
    }

    fn enumeration<E>(&self, key: &str, parse: fn(&str) -> Option<E>) -> Result<E> {
        let text = self.string(key)?;
        parse(text)
            .ok_or_else(|| ConfigError(format!("{}: unknown value \"{text}\"", self.key_path(key))))
    }

    /// A `{ value, unit, assumption }` table with the exact unit the schema fixes for `key`.
    fn quantity(&self, key: &str, unit: &str, range: Range) -> Result<f64> {
        let path = self.key_path(key);
        quantity_from(self.get(key)?, &path, unit, range)
    }

    fn count(&self, key: &str, min: i64, max: i64) -> Result<i64> {
        let path = self.key_path(key);
        count_from(self.get(key)?, &path, min, max)
    }
}

fn quantity_table<'v>(value: &'v Value, path: &str, unit: &str) -> Result<&'v Value> {
    let Value::Table(t) = value else {
        return err(format!("{path}: expected {{ value, unit, assumption }}"));
    };
    for k in t.keys() {
        if !["value", "unit", "assumption"].contains(&k.as_str()) {
            return err(format!("{path}.{k}: unknown key"));
        }
    }
    match t.get("unit") {
        Some(Value::String(u)) if u == unit => {}
        Some(Value::String(u)) => return err(format!("{path}: unit \"{u}\", expected \"{unit}\"")),
        _ => return err(format!("{path}.unit: missing or not a string")),
    }
    match t.get("assumption") {
        Some(Value::Boolean(_)) => {}
        _ => return err(format!("{path}.assumption: missing or not a boolean")),
    }
    t.get("value")
        .ok_or_else(|| ConfigError(format!("{path}.value: missing")))
}

fn quantity_from(value: &Value, path: &str, unit: &str, range: Range) -> Result<f64> {
    let v = match quantity_table(value, path, unit)? {
        Value::Float(f) => *f,
        #[allow(clippy::cast_precision_loss)]
        Value::Integer(i) => *i as f64,
        _ => return err(format!("{path}.value: expected a number")),
    };
    if !v.is_finite() || !range.check(v) {
        return err(format!(
            "{path}: {v} must be finite and {}",
            range.describe()
        ));
    }
    Ok(v)
}

fn count_from(value: &Value, path: &str, min: i64, max: i64) -> Result<i64> {
    match quantity_table(value, path, "count")? {
        Value::Integer(i) if (min..=max).contains(i) => Ok(*i),
        Value::Integer(i) => err(format!("{path}: {i} must be in [{min}, {max}]")),
        _ => err(format!("{path}.value: expected an integer count")),
    }
}

fn narrow<T: TryFrom<i64>>(v: i64, path: &str) -> Result<T> {
    T::try_from(v).map_err(|_| ConfigError(format!("{path}: {v} out of range")))
}

fn pad_index(names: &[String], name: &str, path: &str) -> Result<PadId> {
    match names.iter().position(|n| n == name) {
        Some(i) => narrow(i64::try_from(i).unwrap_or(i64::MAX), path),
        None => err(format!("{path}: no pad named \"{name}\"")),
    }
}

/// Parse and validate a scenario. `text` is the raw `scenario.toml`. A `network_files` scenario
/// needs `parse_in` with the scenario's directory.
pub fn parse(text: &str) -> Result<Config> {
    parse_in(text, None)
}

/// As `parse`, resolving `network_files.dir` against `scenario_dir`.
pub fn parse_in(text: &str, scenario_dir: Option<&Path>) -> Result<Config> {
    let root: Table = text
        .parse()
        .map_err(|e| ConfigError(format!("not valid TOML: {e}")))?;
    let top = Section::new(&root, "");
    top.allow(&[
        "schema_version",
        "meta",
        "sim",
        "geo",
        "pads",
        "lanes",
        "synthetic_network",
        "network_files",
        "flight_profile",
        "drone",
        "payload",
        "charging",
        "pad_ops",
        "handshake",
        "policy",
        "demand",
        "truck",
        "metrics",
        "costs",
        "faults",
        "weather",
        "traffic",
    ])?;
    let version = top.string("schema_version")?;
    if version != SCHEMA_VERSION && !COMPATIBLE_VERSIONS.contains(&version) {
        return err(format!(
            "schema_version: \"{version}\", this core reads \"{SCHEMA_VERSION}\" and {COMPATIBLE_VERSIONS:?}"
        ));
    }
    check_versioned_keys(&root, version)?;
    let meta = top.section("meta")?;
    meta.allow(&["name", "preset", "status", "city", "description"])?;
    let name = meta.string("name")?.to_owned();
    let preset = meta.enumeration("preset", Preset::from_name)?;
    meta.string("city")?;
    meta.string("description")?;
    if meta.enumeration("status", ConfigStatus::from_name)? == ConfigStatus::Stub {
        return err(format!(
            "meta.status: \"{name}\" is a STUB config and cannot be run"
        ));
    }

    let sim = top.section("sim")?;
    sim.allow(&[
        "seed",
        "duration_s",
        "warmup_s",
        "start_time_of_day_s",
        "replications",
        "max_events_per_timestamp",
        "max_event_log_mib",
    ])?;
    let seed_path = sim.key_path("seed");
    let seed = match quantity_table(sim.get("seed")?, &seed_path, "1")? {
        Value::Integer(i) if *i >= 0 => u64::try_from(*i).unwrap_or(0),
        _ => {
            return err(format!(
                "{seed_path}.value: expected a non-negative integer"
            ));
        }
    };
    let duration_s = sim.quantity("duration_s", "s", Range::Pos)?;
    let warmup_s = sim.quantity("warmup_s", "s", Range::NonNeg)?;
    let start_time_of_day_s = sim.quantity("start_time_of_day_s", "s", Range::NonNeg)?;
    if start_time_of_day_s >= SECONDS_PER_DAY {
        return err("sim.start_time_of_day_s: must be < 86400");
    }
    let replications = narrow(
        sim.count("replications", 1, i64::from(u32::MAX))?,
        "sim.replications",
    )?;
    let max_events_per_timestamp = if sim.has("max_events_per_timestamp") {
        narrow(
            sim.count("max_events_per_timestamp", 1, i64::from(u32::MAX))?,
            "sim.max_events_per_timestamp",
        )?
    } else {
        DEFAULT_MAX_EVENTS_PER_TIMESTAMP
    };
    let max_event_log_mib: u64 = if sim.has("max_event_log_mib") {
        narrow(
            sim.count("max_event_log_mib", 1, i64::from(u32::MAX))?,
            "sim.max_event_log_mib",
        )?
    } else {
        DEFAULT_MAX_EVENT_LOG_MIB
    };

    let geo = top.section("geo")?;
    geo.allow(&["origin_lat_deg", "origin_lon_deg"])?;
    let origin_lat_deg = geo.quantity("origin_lat_deg", "deg", Range::Lat)?;
    let origin_lon_deg = geo.quantity("origin_lon_deg", "deg", Range::Lon)?;

    let (mut pads, lanes, mut network_files) =
        parse_network(&top, origin_lat_deg, origin_lon_deg, seed, scenario_dir)?;
    let names: Vec<String> = pads.iter().map(|p| p.name.clone()).collect();
    // The pad layout keys (A72) apply before drones are placed on the pads' slots.
    let pad_ops = parse_pad_ops(&top.section("pad_ops")?)?;
    for p in &mut pads {
        p.out_spots = pad_ops.out_spots;
        if let Some(c) = pad_ops.parking_capacity {
            p.parking_capacity = c;
        }
    }

    let mut drone = parse_drone(&top.section("drone")?, &names, &pads)?;
    let flight_profile = if top.has("flight_profile") {
        Some(apply_flight_profile(&top, scenario_dir, &mut drone)?)
    } else {
        None
    };
    let payload = top.section("payload")?;
    payload.allow(&["pod_gross_mass_kg", "pod_empty_mass_kg"])?;
    let pod_gross_mass_kg = payload.quantity("pod_gross_mass_kg", "kg", Range::Pos)?;
    let pod_empty_mass_kg = payload.quantity("pod_empty_mass_kg", "kg", Range::Pos)?;

    let ch = top.section("charging")?;
    ch.allow(&["mode", "power_w", "efficiency", "charge_while_parked"])?;
    let charging = ChargingCfg {
        mode: ch.enumeration("mode", ChargingMode::from_name)?,
        power_w: ch.quantity("power_w", "W", Range::Pos)?,
        efficiency: ch.quantity("efficiency", "1", Range::Prob)?,
        charge_while_parked: if ch.has("charge_while_parked") {
            ch.boolean("charge_while_parked")?
        } else {
            false
        },
    };
    if charging.efficiency <= 0.0 {
        return err("charging.efficiency: must be > 0");
    }

    let handshake = parse_handshake(&top.section("handshake")?)?;
    let policy = parse_policy(&top.section("policy")?)?;
    let demand = top.section("demand")?;
    let (deadline_offset_ms, flows) = parse_demand(&demand, &names, &pads)?;
    let max_backlog_per_pad = if demand.has("max_backlog_per_pad") {
        Some(narrow(
            demand.count("max_backlog_per_pad", 1, i64::from(u16::MAX))?,
            "demand.max_backlog_per_pad",
        )?)
    } else {
        None
    };
    // `demand.pods_per_origin` (A73): every pad that ships starts with this many empty pods.
    if demand.has("pods_per_origin") {
        let pods: u32 = narrow(
            demand.count("pods_per_origin", 0, i64::from(u32::MAX))?,
            "demand.pods_per_origin",
        )?;
        for f in flows.iter().filter(|f| f.rate_per_s > 0.0) {
            pads[usize::from(f.origin)].initial_empty_pods = pods;
        }
    }
    let truck = parse_truck(&top.section("truck")?)?;
    if truck.matrix_source == MatrixSource::File {
        let Some(files) = network_files.as_mut() else {
            return err(
                "truck.matrix_source: R17 FILE needs network_files (the matrix is indexed by its pad catalog)",
            );
        };
        let path = files.dir.join(TRUCK_FILE);
        let bytes = read_file(&path)?;
        files.truck = Some(
            netfile::read_truck(&bytes, pads.len(), files.catalog_hash)
                .map_err(|e| ConfigError(format!("{}: {e}", path.display())))?,
        );
    }

    let m = top.section("metrics")?;
    m.allow(&["hist_bin_s", "hist_max_s", "order_outcomes"])?;
    let hist_bin_ms = ms_from_s(m.quantity("hist_bin_s", "s", Range::Pos)?);
    let hist_max_ms = ms_from_s(m.quantity("hist_max_s", "s", Range::Pos)?);
    let order_outcomes = m.has("order_outcomes") && m.boolean("order_outcomes")?;

    let c = top.section("costs")?;
    c.allow(&["truck_per_m", "truck_per_s", "drone_per_s", "energy_per_j"])?;
    let costs = CostsCfg {
        truck_per_m: c.quantity("truck_per_m", "cu/m", Range::NonNeg)?,
        truck_per_s: c.quantity("truck_per_s", "cu/s", Range::NonNeg)?,
        drone_per_s: c.quantity("drone_per_s", "cu/s", Range::NonNeg)?,
        energy_per_j: c.quantity("energy_per_j", "cu/J", Range::NonNeg)?,
    };

    let mut faults = Vec::new();
    for f in top.tables("faults")? {
        f.allow(&["kind", "pad", "start_s", "duration_s"])?;
        faults.push(FaultCfg {
            kind: f.enumeration("kind", FaultKind::from_name)?,
            pad: pad_index(&names, f.string("pad")?, &f.key_path("pad"))?,
            start_ms: ms_from_s(f.quantity("start_s", "s", Range::NonNeg)?),
            duration_ms: ms_from_s(f.quantity("duration_s", "s", Range::Pos)?),
        });
    }

    let traffic = if top.has("traffic") {
        let t = parse_traffic(&top.section("traffic")?)?;
        let has_fractions = network_files
            .as_ref()
            .and_then(|f| f.truck.as_ref())
            .is_some_and(|t| t.fractions.is_some());
        if !has_fractions {
            return err(
                "traffic: R22 needs truck.matrix_source FILE with a truck_matrix.bin of version_minor 2 or above",
            );
        }
        if truck
            .tod_speed_factors
            .iter()
            .any(|&f| (f - 1.0).abs() > 0.0)
        {
            return err(
                "truck.tod_speed_factors: R22 must all be 1 with [traffic], which carries time of day",
            );
        }
        Some(t)
    } else {
        None
    };
    let duration_ms = ms_from_s(duration_s);
    let weather = if top.has("weather") {
        Some(parse_weather(
            &top.section("weather")?,
            scenario_dir,
            duration_ms,
        )?)
    } else {
        None
    };

    let cfg = Config {
        name,
        preset,
        source_hash: fnv1a64(text.as_bytes()),
        seed,
        duration_ms,
        warmup_ms: ms_from_s(warmup_s),
        start_time_of_day_s,
        replications,
        max_events_per_timestamp,
        max_event_log_bytes: max_event_log_mib * BYTES_PER_MIB,
        origin_lat_deg,
        origin_lon_deg,
        pads,
        lanes,
        network_files,
        flight_profile,
        drone,
        pod_gross_mass_kg,
        pod_empty_mass_kg,
        charging,
        pad_ops,
        handshake,
        policy,
        deadline_offset_ms,
        max_backlog_per_pad,
        flows,
        truck,
        hist_bin_ms,
        hist_max_ms,
        order_outcomes,
        costs,
        faults,
        weather,
        traffic,
    };
    check_rules(&cfg)?;
    Ok(cfg)
}

fn parse_pads(top: &Section<'_>) -> Result<Vec<PadCfg>> {
    let mut pads = Vec::new();
    for p in top.tables("pads")? {
        p.allow(&[
            "name",
            "lat_deg",
            "lon_deg",
            "is_hub",
            "service_mode",
            "pod_return_policy",
            "initial_empty_pods",
            "parking_capacity",
        ])?;
        let name = p.string("name")?;
        let valid = name.len() <= 32
            && name.chars().next().is_some_and(|c| c.is_ascii_alphabetic())
            && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_');
        if !valid {
            return err(format!("{}: bad pad name \"{name}\"", p.key_path("name")));
        }
        let parking_capacity = if p.has("parking_capacity") {
            narrow(
                p.count("parking_capacity", 0, i64::from(u16::MAX))?,
                &p.key_path("parking_capacity"),
            )?
        } else {
            0
        };
        pads.push(PadCfg {
            name: name.to_owned(),
            lat_deg: p.quantity("lat_deg", "deg", Range::Lat)?,
            lon_deg: p.quantity("lon_deg", "deg", Range::Lon)?,
            is_hub: p.boolean("is_hub")?,
            service_mode: p.enumeration("service_mode", ServiceMode::from_name)?,
            return_policy: p.enumeration("pod_return_policy", ReturnPolicy::from_name)?,
            initial_empty_pods: narrow(
                p.count("initial_empty_pods", 0, i64::from(u32::MAX))?,
                "pads[].initial_empty_pods",
            )?,
            parking_capacity,
            out_spots: 1,
        });
    }
    if pads.len() < 2 || pads.len() >= usize::from(PadId::MAX) {
        return err("pads: need at least 2 pads and fewer than 65535");
    }
    Ok(pads)
}

type Network = (Vec<PadCfg>, Vec<(PadId, PadId)>, Option<NetworkFiles>);

fn read_file(path: &Path) -> Result<Vec<u8>> {
    std::fs::read(path).map_err(|e| ConfigError(format!("{}: {e}", path.display())))
}

fn check_unique_names(pads: &[PadCfg]) -> Result<()> {
    for (i, p) in pads.iter().enumerate() {
        if pads[..i].iter().any(|q| q.name == p.name) {
            return err(format!("pads: R1 duplicate pad name \"{}\"", p.name));
        }
    }
    Ok(())
}

/// `network_files`: pads from `<dir>/pads.toml`, lanes from `<dir>/lanes.csr.bin`, both checked
/// against each other (spec 8.4, 9.0). The truck matrix is read once the truck section is known.
fn parse_network_files(top: &Section<'_>, scenario_dir: Option<&Path>) -> Result<Network> {
    let s = top.section("network_files")?;
    s.allow(&["dir"])?;
    let Some(base) = scenario_dir else {
        return err(
            "network_files: needs the scenario's directory; load the scenario from its file",
        );
    };
    let dir = base.join(s.string("dir")?);
    let catalog_path = dir.join(PADS_FILE);
    let catalog = read_file(&catalog_path)?;
    let in_catalog = |e: ConfigError| ConfigError(format!("{}: {e}", catalog_path.display()));
    let text =
        std::str::from_utf8(&catalog).map_err(|_| in_catalog(ConfigError("not UTF-8".into())))?;
    let root: Table = text
        .parse()
        .map_err(|e| in_catalog(ConfigError(format!("not valid TOML: {e}"))))?;
    let catalog_top = Section::new(&root, "");
    catalog_top.allow(&["pads"]).map_err(in_catalog)?;
    let pads = parse_pads(&catalog_top).map_err(in_catalog)?;
    check_unique_names(&pads).map_err(in_catalog)?;
    let catalog_hash = fnv1a64(&catalog);
    let lanes_path = dir.join(LANES_FILE);
    let lanes = netfile::read_lanes(&read_file(&lanes_path)?, pads.len(), catalog_hash)
        .map_err(|e| ConfigError(format!("{}: {e}", lanes_path.display())))?;
    if lanes.to_pad.is_empty() {
        return err(format!(
            "{}: at least one lane is required",
            lanes_path.display()
        ));
    }
    let undirected = lanes.undirected();
    Ok((
        pads,
        undirected,
        Some(NetworkFiles {
            dir,
            catalog_hash,
            lanes,
            truck: None,
        }),
    ))
}

fn parse_network(
    top: &Section<'_>,
    lat0: f64,
    lon0: f64,
    seed: u64,
    scenario_dir: Option<&Path>,
) -> Result<Network> {
    let explicit = top.has("pads") || top.has("lanes");
    let forms = [
        explicit,
        top.has("synthetic_network"),
        top.has("network_files"),
    ];
    match forms.iter().filter(|&&f| f).count() {
        0 => {
            return err(
                "missing network: give pads and lanes, synthetic_network, or network_files",
            );
        }
        1 => {}
        _ => {
            return err("give exactly one of pads/lanes, synthetic_network or network_files");
        }
    }
    if top.has("network_files") {
        return parse_network_files(top, scenario_dir);
    }
    if top.has("synthetic_network") {
        let s = top.section("synthetic_network")?;
        s.allow(&[
            "topology",
            "n_pads",
            "radius_m",
            "mesh_neighbors",
            "service_mode",
            "pod_return_policy",
            "initial_empty_pods",
            "parking_capacity",
            "hub_parking_capacity",
        ])?;
        let cfg = SyntheticNetworkCfg {
            topology: s.enumeration("topology", NetworkTopology::from_name)?,
            n_pads: narrow(s.count("n_pads", 2, 65_534)?, "synthetic_network.n_pads")?,
            radius_m: s.quantity("radius_m", "m", Range::Pos)?,
            mesh_neighbors: narrow(
                s.count("mesh_neighbors", 1, 65_534)?,
                "synthetic_network.mesh_neighbors",
            )?,
            service_mode: s.enumeration("service_mode", ServiceMode::from_name)?,
            return_policy: s.enumeration("pod_return_policy", ReturnPolicy::from_name)?,
            initial_empty_pods: narrow(
                s.count("initial_empty_pods", 0, i64::from(u32::MAX))?,
                "synthetic_network.initial_empty_pods",
            )?,
            parking_capacity: narrow(
                s.count("parking_capacity", 0, i64::from(u16::MAX))?,
                "synthetic_network.parking_capacity",
            )?,
            hub_parking_capacity: narrow(
                s.count("hub_parking_capacity", 0, i64::from(u16::MAX))?,
                "synthetic_network.hub_parking_capacity",
            )?,
        };
        let (pads, lanes) = network::generate(&cfg, lat0, lon0, seed);
        return Ok((pads, lanes, None));
    }
    let pads = parse_pads(top)?;
    check_unique_names(&pads)?;
    let names: Vec<String> = pads.iter().map(|p| p.name.clone()).collect();
    let mut lanes: Vec<(PadId, PadId)> = Vec::new();
    for l in top.tables("lanes")? {
        l.allow(&["pad_a", "pad_b"])?;
        let a = pad_index(&names, l.string("pad_a")?, &l.key_path("pad_a"))?;
        let b = pad_index(&names, l.string("pad_b")?, &l.key_path("pad_b"))?;
        let key = (a.min(b), a.max(b));
        if a == b || lanes.contains(&key) {
            return err(format!("{}: R2 self or duplicate lane", l.path));
        }
        lanes.push(key);
    }
    if lanes.is_empty() {
        return err("lanes: at least one lane is required");
    }
    Ok((pads, lanes, None))
}

/// Flight-model keys a profile sets (spec 8.7), with their units.
const PROFILE_KEYS: [(&str, &str); 7] = [
    ("cruise_speed_mps", "m/s"),
    ("cruise_power_ref_w", "W"),
    ("hover_power_ref_w", "W"),
    ("power_ref_mass_kg", "kg"),
    ("takeoff_s", "s"),
    ("approach_s", "s"),
    ("hop_s", "s"),
];

/// `[flight_profile] file`: read the profile (relative to the scenario's directory) and put
/// its values over the drone section's, which stay the scenario's placeholder profile.
fn apply_flight_profile(
    top: &Section<'_>,
    scenario_dir: Option<&Path>,
    drone: &mut DroneCfg,
) -> Result<FlightProfile> {
    let s = top.section("flight_profile")?;
    s.allow(&["file"])?;
    let Some(base) = scenario_dir else {
        return err(
            "flight_profile: needs the scenario's directory; load the scenario from its file",
        );
    };
    let path = base.join(s.string("file")?);
    let text = std::fs::read_to_string(&path)
        .map_err(|e| ConfigError(format!("{}: {e}", path.display())))?;
    let in_file = |e: ConfigError| ConfigError(format!("{}: {e}", path.display()));
    let root: Table = text
        .parse()
        .map_err(|e| in_file(ConfigError(format!("not valid TOML: {e}"))))?;
    let p = Section::new(&root, "");
    p.allow(&["name", "source", "drone"]).map_err(in_file)?;
    let name = p.string("name").map_err(in_file)?.to_owned();
    let source = p.string("source").map_err(in_file)?.to_owned();
    let d = p.section("drone").map_err(in_file)?;
    let keys: Vec<&str> = PROFILE_KEYS.iter().map(|(k, _)| *k).collect();
    d.allow(&keys).map_err(in_file)?;
    let value = |key: &str, unit: &str| d.quantity(key, unit, Range::Pos).map_err(in_file);
    drone.cruise_speed_mps = value("cruise_speed_mps", "m/s")?;
    drone.cruise_power_ref_w = value("cruise_power_ref_w", "W")?;
    drone.hover_power_ref_w = value("hover_power_ref_w", "W")?;
    drone.power_ref_mass_kg = value("power_ref_mass_kg", "kg")?;
    drone.takeoff_s = value("takeoff_s", "s")?;
    drone.approach_s = value("approach_s", "s")?;
    drone.hop_s = value("hop_s", "s")?;
    Ok(FlightProfile { path, name, source })
}

fn parse_drone(d: &Section<'_>, names: &[String], pads: &[PadCfg]) -> Result<DroneCfg> {
    d.allow(&[
        "fleet_size",
        "initial_pads",
        "empty_mass_kg",
        "airframe_mass_kg",
        "battery_specific_energy_wh_per_kg",
        "latch_mass_kg",
        "mtow_kg",
        "battery_capacity_j",
        "initial_soc",
        "reserve_soc",
        "charge_target_soc",
        "cruise_speed_mps",
        "cruise_power_ref_w",
        "hover_power_ref_w",
        "power_ref_mass_kg",
        "takeoff_s",
        "approach_s",
        "go_around_s",
        "hop_s",
        "p_release_fault",
        "p_latch_fault",
        "latch_capture_radius_m",
        "out_landing_error_bias_m",
        "out_landing_error_sigma_m",
    ])?;
    let fleet_size: u16 = narrow(d.count("fleet_size", 1, 65_534)?, "drone.fleet_size")?;
    let initial_pads = if d.has("initial_pads") {
        let Value::Array(items) = d.get("initial_pads")? else {
            return err("drone.initial_pads: expected an array of pad names");
        };
        let mut out = Vec::with_capacity(items.len());
        for (i, v) in items.iter().enumerate() {
            let path = format!("drone.initial_pads[{i}]");
            let Value::String(s) = v else {
                return err(format!("{path}: expected a pad name"));
            };
            out.push(pad_index(names, s, &path)?);
        }
        out
    } else {
        round_robin_placement(pads, fleet_size)
    };
    let battery_capacity_j = d.quantity("battery_capacity_j", "J", Range::Pos)?;
    let coupled = d.has("airframe_mass_kg") || d.has("battery_specific_energy_wh_per_kg");
    if coupled == d.has("empty_mass_kg") {
        return err(
            "drone: give empty_mass_kg, or airframe_mass_kg with battery_specific_energy_wh_per_kg",
        );
    }
    let mass_coupling = if coupled {
        Some(MassCoupling {
            airframe_mass_kg: d.quantity("airframe_mass_kg", "kg", Range::Pos)?,
            specific_energy_j_per_kg: d.quantity(
                "battery_specific_energy_wh_per_kg",
                "Wh/kg",
                Range::Pos,
            )? * J_PER_WH,
        })
    } else {
        None
    };
    let derived = LATCH_DERIVATION_KEYS.iter().filter(|k| d.has(k)).count();
    let latch_derivation = match (derived, d.has("p_latch_fault")) {
        (0, true) => None,
        (3, false) => Some(LatchDerivation {
            capture_radius_m: d.quantity("latch_capture_radius_m", "m", Range::Pos)?,
            out_error_bias_m: d.quantity("out_landing_error_bias_m", "m", Range::NonNeg)?,
            out_error_sigma_m: d.quantity("out_landing_error_sigma_m", "m", Range::Pos)?,
        }),
        _ => {
            return err(
                "drone: give p_latch_fault, or latch_capture_radius_m with out_landing_error_bias_m and out_landing_error_sigma_m",
            );
        }
    };
    let p_latch_fault = match latch_derivation {
        Some(l) => crate::flight::landing_miss_probability(
            l.out_error_bias_m,
            l.out_error_sigma_m,
            l.capture_radius_m,
        ),
        None => d.quantity("p_latch_fault", "1", Range::Prob)?,
    };
    let empty_mass_kg = match mass_coupling {
        Some(c) => c.empty_mass_kg(battery_capacity_j),
        None => d.quantity("empty_mass_kg", "kg", Range::Pos)?,
    };
    Ok(DroneCfg {
        fleet_size,
        initial_pads,
        empty_mass_kg,
        mass_coupling,
        latch_mass_kg: d.quantity("latch_mass_kg", "kg", Range::NonNeg)?,
        mtow_kg: d.quantity("mtow_kg", "kg", Range::Pos)?,
        battery_capacity_j,
        initial_soc: d.quantity("initial_soc", "1", Range::Prob)?,
        reserve_soc: d.quantity("reserve_soc", "1", Range::Prob)?,
        charge_target_soc: d.quantity("charge_target_soc", "1", Range::Prob)?,
        cruise_speed_mps: d.quantity("cruise_speed_mps", "m/s", Range::Pos)?,
        cruise_power_ref_w: d.quantity("cruise_power_ref_w", "W", Range::Pos)?,
        hover_power_ref_w: d.quantity("hover_power_ref_w", "W", Range::Pos)?,
        power_ref_mass_kg: d.quantity("power_ref_mass_kg", "kg", Range::Pos)?,
        takeoff_s: d.quantity("takeoff_s", "s", Range::Pos)?,
        approach_s: d.quantity("approach_s", "s", Range::Pos)?,
        go_around_s: d.quantity("go_around_s", "s", Range::Pos)?,
        hop_s: d.quantity("hop_s", "s", Range::Pos)?,
        p_release_fault: d.quantity("p_release_fault", "1", Range::Prob)?,
        p_latch_fault,
        latch_derivation,
    })
}

/// Spec 8.6: drone i goes to the next pad, round-robin by ID, that still has a free slot.
/// If the fleet exceeds the slots, the extra drones are left unplaced and R5 rejects the config.
fn round_robin_placement(pads: &[PadCfg], fleet_size: u16) -> Vec<PadId> {
    let mut used = vec![0u16; pads.len()];
    let mut out = Vec::with_capacity(usize::from(fleet_size));
    let mut next = 0usize;
    for _ in 0..fleet_size {
        let start = next;
        loop {
            if used[next] < pads[next].slots() {
                used[next] += 1;
                out.push(PadId::try_from(next).unwrap_or(PadId::MAX));
                next = (next + 1) % pads.len();
                break;
            }
            next = (next + 1) % pads.len();
            if next == start {
                return out;
            }
        }
    }
    out
}

fn parse_pad_ops(p: &Section<'_>) -> Result<PadOpsCfg> {
    p.allow(&[
        "pod_place_s",
        "pod_remove_s",
        "release_s",
        "release_verify_s",
        "latch_s",
        "latch_verify_s",
        "manual_release_s",
        "manual_latch_s",
        "p_marker_detect",
        "max_go_arounds",
        "release_max_retries",
        "latch_max_retries",
        "out_empty_policy",
        "out_empty_wait_timeout_s",
        "pc_fault_rate_per_h",
        "pc_fault_duration_s",
        "repair_mean_s",
        "repair_spread_s",
        "marker_self_check",
        "out_spots",
        "parking_capacity",
    ])?;
    let ms = |key: &str, r: Range| p.quantity(key, "s", r).map(ms_from_s);
    let repair = match (p.has("repair_mean_s"), p.has("repair_spread_s")) {
        (false, false) => None,
        (true, true) => {
            let mean_s = p.quantity("repair_mean_s", "s", Range::Pos)?;
            let spread_s = p.quantity("repair_spread_s", "s", Range::NonNeg)?;
            if spread_s > mean_s {
                return err("pad_ops.repair_spread_s: must not exceed repair_mean_s");
            }
            Some(RepairCfg { mean_s, spread_s })
        }
        _ => return err("pad_ops: give repair_mean_s and repair_spread_s together"),
    };
    Ok(PadOpsCfg {
        repair,
        marker_self_check: p.has("marker_self_check") && p.boolean("marker_self_check")?,
        out_spots: if p.has("out_spots") {
            narrow(
                p.count("out_spots", 1, i64::try_from(MAX_OUT_SPOTS).unwrap_or(1))?,
                "pad_ops.out_spots",
            )?
        } else {
            1
        },
        parking_capacity: if p.has("parking_capacity") {
            Some(narrow(
                p.count("parking_capacity", 0, i64::from(u16::MAX))?,
                "pad_ops.parking_capacity",
            )?)
        } else {
            None
        },
        pod_place_ms: ms("pod_place_s", Range::NonNeg)?,
        pod_remove_ms: ms("pod_remove_s", Range::NonNeg)?,
        release_ms: ms("release_s", Range::Pos)?,
        release_verify_ms: ms("release_verify_s", Range::Pos)?,
        latch_ms: ms("latch_s", Range::Pos)?,
        latch_verify_ms: ms("latch_verify_s", Range::Pos)?,
        manual_release_ms: ms("manual_release_s", Range::Pos)?,
        manual_latch_ms: ms("manual_latch_s", Range::Pos)?,
        p_marker_detect: p.quantity("p_marker_detect", "1", Range::Prob)?,
        max_go_arounds: narrow(p.count("max_go_arounds", 0, 255)?, "pad_ops.max_go_arounds")?,
        release_max_retries: narrow(
            p.count("release_max_retries", 0, 255)?,
            "pad_ops.release_max_retries",
        )?,
        latch_max_retries: narrow(
            p.count("latch_max_retries", 0, 255)?,
            "pad_ops.latch_max_retries",
        )?,
        out_empty_policy: p.enumeration("out_empty_policy", OutEmptyPolicy::from_name)?,
        out_empty_wait_ms: ms("out_empty_wait_timeout_s", Range::NonNeg)?,
        pc_fault_rate_per_s: p.quantity("pc_fault_rate_per_h", "1/h", Range::NonNeg)?
            / SECONDS_PER_HOUR,
        pc_fault_duration_ms: ms("pc_fault_duration_s", Range::Pos)?,
    })
}

fn parse_handshake(h: &Section<'_>) -> Result<HandshakeCfg> {
    h.allow(&[
        "local_latency_s",
        "fm_latency_s",
        "p_msg_loss",
        "response_timeout_s",
        "max_retries",
        "hold_retry_s",
        "pc_silence_alert_s",
        "reorder_window_s",
    ])?;
    let ms = |key: &str, r: Range| h.quantity(key, "s", r).map(ms_from_s);
    let p_msg_loss = h.quantity("p_msg_loss", "1", Range::Prob)?;
    if p_msg_loss >= 1.0 {
        return err("handshake.p_msg_loss: must be < 1");
    }
    Ok(HandshakeCfg {
        local_latency_ms: ms("local_latency_s", Range::NonNeg)?,
        fm_latency_ms: ms("fm_latency_s", Range::NonNeg)?,
        p_msg_loss,
        response_timeout_ms: ms("response_timeout_s", Range::Pos)?,
        max_retries: narrow(h.count("max_retries", 0, 255)?, "handshake.max_retries")?,
        hold_retry_ms: ms("hold_retry_s", Range::Pos)?,
        pc_silence_alert_ms: ms("pc_silence_alert_s", Range::Pos)?,
        reorder_window_ms: if h.has("reorder_window_s") {
            ms("reorder_window_s", Range::NonNeg)?
        } else {
            0
        },
    })
}

fn parse_policy(p: &Section<'_>) -> Result<PolicyCfg> {
    p.allow(&[
        "assignment",
        "dispatch",
        "holding",
        "drone_rebalancing",
        "pod_rebalancing",
        "diversion",
        "in_occupied_action",
        "max_hold_s",
        "marker_hold_s",
        "marker_max_hold_cycles",
        "pod_min_per_pad",
        "yield_on_request",
    ])?;
    // One implementation of each in phase 2; the names are still validated.
    p.enumeration("assignment", AssignmentPolicyName::from_name)?;
    p.enumeration("dispatch", DispatchPolicyName::from_name)?;
    p.enumeration("holding", HoldingPolicyName::from_name)?;
    p.enumeration("drone_rebalancing", DroneRebalancingPolicyName::from_name)?;
    p.enumeration("pod_rebalancing", PodRebalancingPolicyName::from_name)?;
    p.enumeration("diversion", DiversionPolicyName::from_name)?;
    Ok(PolicyCfg {
        in_occupied_action: p.enumeration("in_occupied_action", InOccupiedAction::from_name)?,
        max_hold_s: p.quantity("max_hold_s", "s", Range::NonNeg)?,
        marker_hold_s: p.quantity("marker_hold_s", "s", Range::Pos)?,
        marker_max_hold_cycles: narrow(
            p.count("marker_max_hold_cycles", 0, 255)?,
            "policy.marker_max_hold_cycles",
        )?,
        pod_min_per_pad: narrow(
            p.count("pod_min_per_pad", 0, i64::from(u32::MAX))?,
            "policy.pod_min_per_pad",
        )?,
        yield_on_request: p.has("yield_on_request") && p.boolean("yield_on_request")?,
    })
}

fn parse_weather(
    w: &Section<'_>,
    scenario_dir: Option<&Path>,
    duration_ms: TimeMs,
) -> Result<WeatherCfg> {
    w.allow(&[
        "series_file",
        "start_hour",
        "wind_shear_exponent",
        "cruise_height_m",
        "max_airspeed_mps",
        "rotor_disc_area_m2",
        "air_density_kgpm3",
        "max_wind_mps",
        "max_gust_mps",
        "min_visibility_m",
        "max_precip_mm_h",
        "min_temp_c",
        "max_temp_c",
        "landing_max_wind_mps",
        "landing_max_gust_mps",
        "latch_gust_k",
        "latch_gust_ref_mps",
        "marker_penalty_rain",
        "marker_penalty_snow",
        "marker_penalty_low_visibility",
        "low_visibility_m",
        "battery_derate_temp_c",
        "battery_derate_factor",
        "truck_factor_rain",
        "truck_factor_snow",
        "truck_factor_low_visibility",
        "hold_policy",
        "truck_fallback_timeout_s",
        "energy_route_guard",
    ])?;
    let Some(base) = scenario_dir else {
        return err("weather: needs the scenario's directory; load the scenario from its file");
    };
    let path = base.join(w.string("series_file")?);
    let series = netfile::read_weather(&read_file(&path)?)
        .map_err(|e| ConfigError(format!("{}: {e}", path.display())))?;
    let start_hour: usize = narrow(
        w.count("start_hour", 0, i64::from(u32::MAX))?,
        "weather.start_hour",
    )?;
    let needed = WeatherCfg::hours_needed(duration_ms);
    if start_hour.saturating_add(needed) > series.wind_mps.len() {
        return err(format!(
            "weather.start_hour: R18 the run needs hours {start_hour}..{} of a {}-hour series",
            start_hour.saturating_add(needed),
            series.wind_mps.len()
        ));
    }
    let limits = Limits {
        max_wind_mps: w.quantity("max_wind_mps", "m/s", Range::NonNeg)?,
        max_gust_mps: w.quantity("max_gust_mps", "m/s", Range::NonNeg)?,
        min_visibility_m: w.quantity("min_visibility_m", "m", Range::NonNeg)?,
        max_precip_mm_h: w.quantity("max_precip_mm_h", "mm/h", Range::NonNeg)?,
        min_temp_c: w.quantity("min_temp_c", "degC", Range::Finite)?,
        max_temp_c: w.quantity("max_temp_c", "degC", Range::Finite)?,
        landing_max_wind_mps: w.quantity("landing_max_wind_mps", "m/s", Range::NonNeg)?,
        landing_max_gust_mps: w.quantity("landing_max_gust_mps", "m/s", Range::NonNeg)?,
    };
    if limits.min_temp_c > limits.max_temp_c {
        return err("weather.min_temp_c: must not exceed weather.max_temp_c");
    }
    if limits.landing_max_wind_mps < limits.max_wind_mps
        || limits.landing_max_gust_mps < limits.max_gust_mps
    {
        return err(
            "weather.landing_max_wind_mps: R21 landing limits must be at or above the dispatch limits",
        );
    }
    let battery_derate = parse_derate_table(w)?;
    let hold_policy = w.enumeration("hold_policy", WeatherHoldPolicy::from_name)?;
    Ok(WeatherCfg {
        series: std::sync::Arc::new(series),
        start_hour,
        wind_shear_exponent: w.quantity("wind_shear_exponent", "1", Range::NonNeg)?,
        cruise_height_m: w.quantity("cruise_height_m", "m", Range::Pos)?,
        max_airspeed_mps: w.quantity("max_airspeed_mps", "m/s", Range::Pos)?,
        rotor_disc_area_m2: w.quantity("rotor_disc_area_m2", "m2", Range::Pos)?,
        air_density_kgpm3: w.quantity("air_density_kgpm3", "kg/m3", Range::Pos)?,
        limits,
        latch_gust_k: w.quantity("latch_gust_k", "1", Range::NonNeg)?,
        latch_gust_ref_mps: w.quantity("latch_gust_ref_mps", "m/s", Range::Pos)?,
        marker_penalty_rain: w.quantity("marker_penalty_rain", "1", Range::Prob)?,
        marker_penalty_snow: w.quantity("marker_penalty_snow", "1", Range::Prob)?,
        marker_penalty_low_visibility: w.quantity(
            "marker_penalty_low_visibility",
            "1",
            Range::Prob,
        )?,
        low_visibility_m: w.quantity("low_visibility_m", "m", Range::NonNeg)?,
        battery_derate,
        truck_factor_rain: w.quantity("truck_factor_rain", "1", Range::Factor)?,
        truck_factor_snow: w.quantity("truck_factor_snow", "1", Range::Factor)?,
        truck_factor_low_visibility: w.quantity(
            "truck_factor_low_visibility",
            "1",
            Range::Factor,
        )?,
        hold_policy,
        truck_fallback_timeout_ms: ms_from_s(w.quantity(
            "truck_fallback_timeout_s",
            "s",
            Range::Pos,
        )?),
        energy_route_guard: w.has("energy_route_guard") && w.boolean("energy_route_guard")?,
    })
}

fn parse_traffic(t: &Section<'_>) -> Result<TrafficCfg> {
    t.allow(&[
        "start_weekday",
        "weekday_shape",
        "weekend_shape",
        "peak_delay_motorway",
        "peak_delay_primary",
        "peak_delay_other",
        "peak_delay_structure",
        "incident_rate_per_pair_h",
        "incident_delay_median_s",
        "incident_delay_sigma",
        "start_unix_s",
    ])?;
    let shape = |key: &str| -> Result<[f64; HOURS_PER_DAY]> {
        let Value::Array(items) = t.get(key)? else {
            return err(format!(
                "{}: expected an array of 24 quantities",
                t.key_path(key)
            ));
        };
        if items.len() != HOURS_PER_DAY {
            return err(format!(
                "{}: {} entries, expected 24",
                t.key_path(key),
                items.len()
            ));
        }
        let mut out = [0.0; HOURS_PER_DAY];
        for (i, v) in items.iter().enumerate() {
            out[i] = quantity_from(v, &format!("{}[{i}]", t.key_path(key)), "1", Range::Prob)?;
        }
        Ok(out)
    };
    let delay = |key: &str| t.quantity(key, "1", Range::NonNeg);
    let peak_delay: [f64; DELAY_CLASSES] = [
        delay("peak_delay_motorway")?,
        delay("peak_delay_primary")?,
        delay("peak_delay_other")?,
        delay("peak_delay_structure")?,
    ];
    let start_weekday: u8 = narrow(t.count("start_weekday", 0, 6)?, "traffic.start_weekday")?;
    let start_unix_s = if t.has("start_unix_s") {
        #[allow(clippy::cast_possible_truncation)]
        let s = t.quantity("start_unix_s", "s", Range::NonNeg)?.round() as i64;
        let local_day = crate::traffic::eastern_local_s(s).div_euclid(86_400);
        if crate::traffic::weekday(local_day) != u64::from(start_weekday) {
            return err(
                "traffic.start_weekday: R23 must be the US Eastern weekday of traffic.start_unix_s",
            );
        }
        Some(s)
    } else {
        None
    };
    Ok(TrafficCfg {
        start_weekday,
        start_unix_s,
        weekday_shape: shape("weekday_shape")?,
        weekend_shape: shape("weekend_shape")?,
        peak_delay,
        incident_rate_per_pair_h: t.quantity("incident_rate_per_pair_h", "1/h", Range::NonNeg)?,
        incident_delay_median_s: t.quantity("incident_delay_median_s", "s", Range::Pos)?,
        incident_delay_sigma: t.quantity("incident_delay_sigma", "1", Range::NonNeg)?,
    })
}

/// `weather.battery_derate_temp_c` and `_factor`: equal-length arrays, temperatures strictly
/// ascending, factors in (0, 1].
fn parse_derate_table(w: &Section<'_>) -> Result<Vec<(f64, f64)>> {
    let array = |key: &str, unit: &str, range: Range| -> Result<Vec<f64>> {
        let Value::Array(items) = w.get(key)? else {
            return err(format!(
                "{}: expected an array of quantities",
                w.key_path(key)
            ));
        };
        items
            .iter()
            .enumerate()
            .map(|(i, v)| quantity_from(v, &format!("{}[{i}]", w.key_path(key)), unit, range))
            .collect()
    };
    let temps = array("battery_derate_temp_c", "degC", Range::Finite)?;
    let factors = array("battery_derate_factor", "1", Range::Prob)?;
    if temps.is_empty() || temps.len() != factors.len() {
        return err(
            "weather.battery_derate_temp_c: needs as many entries as battery_derate_factor, at least one",
        );
    }
    if temps.windows(2).any(|t| t[0] >= t[1]) {
        return err("weather.battery_derate_temp_c: temperatures must be strictly ascending");
    }
    if factors.iter().any(|&f| f <= 0.0) {
        return err("weather.battery_derate_factor: every factor must be > 0");
    }
    Ok(temps.into_iter().zip(factors).collect())
}

fn parse_demand(d: &Section<'_>, names: &[String], pads: &[PadCfg]) -> Result<(TimeMs, Vec<Flow>)> {
    d.allow(&[
        "deadline_offset_s",
        "max_backlog_per_pad",
        "pods_per_origin",
        "flows",
        "rate_per_pad_per_h",
        "hub_weight",
    ])?;
    let deadline_offset_ms = ms_from_s(d.quantity("deadline_offset_s", "s", Range::Pos)?);
    let generated = d.has("rate_per_pad_per_h") || d.has("hub_weight");
    if generated == d.has("flows") {
        return err("demand: give either flows, or rate_per_pad_per_h with hub_weight");
    }
    if !generated {
        let mut flows = Vec::new();
        for f in d.tables("flows")? {
            f.allow(&["origin", "destination", "rate_per_h"])?;
            let origin = pad_index(names, f.string("origin")?, &f.key_path("origin"))?;
            let dest = pad_index(names, f.string("destination")?, &f.key_path("destination"))?;
            if origin == dest {
                return err(format!("{}: R9 origin equals destination", f.path));
            }
            flows.push(Flow {
                origin,
                dest,
                rate_per_s: f.quantity("rate_per_h", "1/h", Range::NonNeg)? / SECONDS_PER_HOUR,
            });
        }
        if flows.is_empty() {
            return err("demand.flows: at least one flow is required");
        }
        return Ok((deadline_offset_ms, flows));
    }
    let rate_per_s = d.quantity("rate_per_pad_per_h", "1/h", Range::NonNeg)? / SECONDS_PER_HOUR;
    let hub_weight = d.quantity("hub_weight", "1", Range::Factor)?;
    let weight = |p: &PadCfg| if p.is_hub { hub_weight } else { 1.0 };
    let mut flows = Vec::new();
    for o in 0..pads.len() {
        let total: f64 = pads
            .iter()
            .enumerate()
            .filter(|(q, _)| *q != o)
            .map(|(_, p)| weight(p))
            .sum();
        for (q, dest) in pads.iter().enumerate() {
            if q != o {
                flows.push(Flow {
                    origin: PadId::try_from(o).unwrap_or(PadId::MAX),
                    dest: PadId::try_from(q).unwrap_or(PadId::MAX),
                    rate_per_s: rate_per_s * weight(dest) / total,
                });
            }
        }
    }
    Ok((deadline_offset_ms, flows))
}

fn parse_truck(t: &Section<'_>) -> Result<TruckCfg> {
    t.allow(&[
        "matrix_source",
        "synthetic_detour_factor",
        "synthetic_free_flow_speed_mps",
        "dispatch_delay_s",
        "load_s",
        "unload_s",
        "tod_speed_factors",
    ])?;
    let Value::Array(items) = t.get("tod_speed_factors")? else {
        return err("truck.tod_speed_factors: expected an array of 24 quantities");
    };
    if items.len() != HOURS_PER_DAY {
        return err(format!(
            "truck.tod_speed_factors: {} entries, expected 24",
            items.len()
        ));
    }
    let mut tod_speed_factors = [0.0; HOURS_PER_DAY];
    for (i, v) in items.iter().enumerate() {
        tod_speed_factors[i] = quantity_from(
            v,
            &format!("truck.tod_speed_factors[{i}]"),
            "1",
            Range::Factor,
        )?;
    }
    let matrix_source = t.enumeration("matrix_source", MatrixSource::from_name)?;
    let synthetic_keys = ["synthetic_detour_factor", "synthetic_free_flow_speed_mps"];
    let synthetic = match matrix_source {
        MatrixSource::Synthetic => {
            let detour_factor = t.quantity("synthetic_detour_factor", "1", Range::Factor)?;
            if detour_factor < 1.0 {
                return err("truck.synthetic_detour_factor: must be >= 1");
            }
            Some(SyntheticTruckCfg {
                detour_factor,
                free_flow_speed_mps: t.quantity(
                    "synthetic_free_flow_speed_mps",
                    "m/s",
                    Range::Pos,
                )?,
            })
        }
        MatrixSource::File => {
            if let Some(k) = synthetic_keys.iter().find(|k| t.has(k)) {
                return err(format!(
                    "truck.{k}: only used with matrix_source = SYNTHETIC; remove it"
                ));
            }
            None
        }
    };
    Ok(TruckCfg {
        matrix_source,
        synthetic,
        dispatch_delay_s: t.quantity("dispatch_delay_s", "s", Range::NonNeg)?,
        load_s: t.quantity("load_s", "s", Range::NonNeg)?,
        unload_s: t.quantity("unload_s", "s", Range::NonNeg)?,
        tod_speed_factors,
    })
}

/// R3 to R11 and R13 (R1, R2, R9 are checked while parsing; R12 and R16 need routes).
fn check_rules(cfg: &Config) -> Result<()> {
    let n = cfg.pads.len();
    let mut seen = vec![false; n];
    let mut stack = vec![0usize];
    seen[0] = true;
    while let Some(u) = stack.pop() {
        for &(a, b) in &cfg.lanes {
            let (a, b) = (usize::from(a), usize::from(b));
            let v = if a == u {
                b
            } else if b == u {
                a
            } else {
                continue;
            };
            if !seen[v] {
                seen[v] = true;
                stack.push(v);
            }
        }
    }
    if seen.iter().any(|s| !s) {
        return err("lanes: R3 the lane graph is not connected");
    }
    let d = &cfg.drone;
    if d.initial_pads.len() != usize::from(d.fleet_size) {
        return err(
            "drone.initial_pads: R4 one pad per drone is required (or not enough slots for round-robin)",
        );
    }
    let mut per_pad = vec![0u16; n];
    for &p in &d.initial_pads {
        per_pad[usize::from(p)] += 1;
    }
    for (i, &c) in per_pad.iter().enumerate() {
        if c > cfg.pads[i].slots() {
            return err(format!(
                "drone.initial_pads: R4 {c} drones at {} exceed its {} slot(s)",
                cfg.pads[i].name,
                cfg.pads[i].slots()
            ));
        }
    }
    let total_slots: u32 = cfg.pads.iter().map(|p| u32::from(p.slots())).sum();
    if u32::from(d.fleet_size) > total_slots {
        return err(format!(
            "drone.fleet_size: R5 {} drones exceed {total_slots} parking slots",
            d.fleet_size
        ));
    }
    let margin = d.mtow_kg - d.empty_mass_kg - d.latch_mass_kg;
    if cfg.pod_gross_mass_kg > margin || cfg.pod_empty_mass_kg >= cfg.pod_gross_mass_kg {
        return err(format!(
            "payload: R6 pod of {} kg does not fit a margin of {margin} kg",
            cfg.pod_gross_mass_kg
        ));
    }
    if !(d.reserve_soc < d.charge_target_soc && d.reserve_soc < d.initial_soc) {
        return err("drone: R7 reserve_soc must be below charge_target_soc and initial_soc");
    }
    if cfg.warmup_ms >= cfg.duration_ms {
        return err("sim: R8 warmup_s must be below duration_s");
    }
    let mut windows: Vec<(PadId, TimeMs, TimeMs)> = Vec::new();
    for f in &cfg.faults {
        if f.start_ms >= cfg.duration_ms {
            return err("faults: R10 a fault starts after the run ends");
        }
        let end = f.start_ms + f.duration_ms;
        if windows
            .iter()
            .any(|&(p, s, e)| p == f.pad && f.start_ms < e && s < end)
        {
            return err(format!(
                "faults: R10 overlapping windows on {}",
                cfg.pads[usize::from(f.pad)].name
            ));
        }
        windows.push((f.pad, f.start_ms, end));
    }
    if cfg.charging.mode == ChargingMode::HubOnly && !cfg.pads.iter().any(|p| p.is_hub) {
        return err("charging.mode: R11 HUB_ONLY needs at least one hub pad");
    }
    if cfg.hist_max_ms <= cfg.hist_bin_ms {
        return err("metrics: R13 hist_max_s must exceed hist_bin_s");
    }
    if let Some(w) = &cfg.weather {
        if cfg.drone.latch_derivation.is_none() {
            return err(
                "weather: R19 needs the derived latch keys (A71) so gusts can widen the landing error",
            );
        }
        let model = FlightModel::from_config(cfg);
        if CruiseCurve::new(model, w.air_density_kgpm3, w.rotor_disc_area_m2).is_none() {
            return err(
                "weather: R20 drone.cruise_power_ref_w is below the ideal induced power at cruise speed for this disc area and air density",
            );
        }
    }
    Ok(())
}
