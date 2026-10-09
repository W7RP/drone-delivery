//! `sim run <config> --seed N --out <dir> [--replication R]`, `sim dump <events.bin>`,
//! `sim flight-model --lane-m L --payload-kg P [--config <scenario.toml>]`, and
//! `sim battery --config <scenario.toml> --payload-kg P --from <pad> --to <pad>`.

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use drone_sim::config::{Config, ConfigError};
use drone_sim::flight::{EnergyRules, VisitEstimate};
use drone_sim::network::Network;
use drone_sim::output::Outputs;
use drone_sim::scenario::Scenario;

const USAGE: &str = "usage: sim run <scenario.toml> --seed N --out <dir> [--replication R] [--no-event-log]\n       sim dump <events.bin>\n       sim flight-model --lane-m L --payload-kg P [--config <scenario.toml>]\n       sim battery --config <scenario.toml> --payload-kg P --from <pad> --to <pad>";
const J_PER_WH: f64 = 3600.0;
/// `sim battery` rounds the sized battery up to a whole multiple of this.
const BATTERY_STEP_WH: f64 = 10.0;
/// The battery fixed point stops when an iteration moves the capacity by less than this.
const BATTERY_TOLERANCE_J: f64 = 1.0;
const BATTERY_MAX_ITERATIONS: usize = 100;
/// `sim flight-model` without --config uses the anchor scenario's flight model.
const DEFAULT_FLIGHT_CONFIG: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../configs/parts_logistics_pittsburgh/scenario.toml"
);

struct Args {
    config: PathBuf,
    seed: Option<u64>,
    out: PathBuf,
    replication: u32,
    /// Skip `events.bin` (sweeps read only metrics and deliveries).
    event_log: bool,
}

fn parse_args() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    if it.next().as_deref() != Some("run") {
        return Err(USAGE.into());
    }
    let config = it.next().ok_or(USAGE)?;
    let (mut seed, mut out, mut replication, mut event_log) = (None, None, 0, true);
    while let Some(flag) = it.next() {
        if flag == "--no-event-log" {
            event_log = false;
            continue;
        }
        let value = it
            .next()
            .ok_or_else(|| format!("{flag} needs a value\n{USAGE}"))?;
        match flag.as_str() {
            "--seed" => {
                seed = Some(
                    value
                        .parse()
                        .map_err(|_| format!("--seed: not an unsigned integer: {value}"))?,
                )
            }
            "--out" => out = Some(PathBuf::from(value)),
            "--replication" => {
                replication = value
                    .parse()
                    .map_err(|_| format!("--replication: not an unsigned integer: {value}"))?
            }
            _ => return Err(format!("unknown flag {flag}\n{USAGE}")),
        }
    }
    Ok(Args {
        config: PathBuf::from(config),
        seed,
        out: out.ok_or(USAGE)?,
        replication,
        event_log,
    })
}

/// `sim dump <events.bin>`: print the event log as text, one record per line.
fn dump(path: &str) -> ExitCode {
    let bytes = match std::fs::read(path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("{path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    match drone_sim::trace::decode(&bytes) {
        Ok(records) => {
            for r in &records {
                println!("{}", drone_sim::trace::format_record(r));
            }
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("{path}: {e}");
            ExitCode::FAILURE
        }
    }
}

/// `sim flight-model`: the core's own time and energy for one visit's flight phases (takeoff,
/// a lane of `--lane-m`, approach, hop) at `--payload-kg`, as JSON. The calibration scripts call
/// this instead of reimplementing the model.
fn flight_model(args: &[String]) -> ExitCode {
    let (mut lane_m, mut payload_kg, mut config) = (None, None, DEFAULT_FLIGHT_CONFIG.to_owned());
    let mut it = args.iter();
    while let Some(flag) = it.next() {
        let Some(value) = it.next() else {
            eprintln!("{flag} needs a value\n{USAGE}");
            return ExitCode::from(2);
        };
        let number = || {
            value
                .parse::<f64>()
                .ok()
                .filter(|v| v.is_finite() && *v >= 0.0)
        };
        match flag.as_str() {
            "--lane-m" => lane_m = number(),
            "--payload-kg" => payload_kg = number(),
            "--config" => config.clone_from(value),
            _ => {
                eprintln!("unknown flag {flag}\n{USAGE}");
                return ExitCode::from(2);
            }
        }
    }
    let (Some(lane_m), Some(payload_kg)) = (lane_m, payload_kg) else {
        eprintln!("--lane-m and --payload-kg need non-negative numbers\n{USAGE}");
        return ExitCode::from(2);
    };
    let cfg = match parse_only(&config) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("config error: {e}");
            return ExitCode::from(2);
        }
    };
    let v = VisitEstimate::new(&cfg, lane_m, payload_kg);
    let phase = |p: drone_sim::flight::PhaseEstimate| serde_json::json!({"time_s": p.time_s, "energy_j": p.energy_j});
    let out = serde_json::json!({
        "config": config,
        "lane_m": lane_m,
        "payload_kg": payload_kg,
        "takeoff": phase(v.takeoff),
        "transition": phase(v.transition),
        "cruise": phase(v.cruise),
        "approach": phase(v.approach),
        "hop": phase(v.hop),
        "total": phase(v.total()),
    });
    println!(
        "{}",
        serde_json::to_string_pretty(&out).expect("estimate serialises")
    );
    ExitCode::SUCCESS
}

/// Parse only: the flight model and the energy guard need the config and network, not the
/// network's energy rules (R12, R16), which a battery or profile under evaluation may break.
fn parse_only(config: &str) -> Result<Config, ConfigError> {
    let path = std::path::Path::new(config);
    std::fs::read_to_string(path)
        .map_err(|e| ConfigError(format!("{config}: {e}")))
        .and_then(|text| drone_sim::config::parse_in(&text, path.parent()))
}

/// The energy `dispatch_ok` needs above reserve for `from -> to` with `payload_kg`, every pad
/// online, at battery `capacity_j` (which moves the empty mass when it is coupled, A68).
fn lane_need_j(
    cfg: &Config,
    net: &Network,
    lane: (u16, u16),
    payload_kg: f64,
    capacity_j: f64,
) -> f64 {
    let mut cfg = cfg.clone();
    cfg.drone.set_battery_capacity(capacity_j);
    EnergyRules::from_config(&cfg).dispatch_need_j(
        net,
        lane.0,
        lane.1,
        payload_kg,
        |_| true,
        |q| cfg.has_charger(q),
    )
}

fn battery_summary(cfg: &Config, capacity_j: f64, payload_kg: f64) -> serde_json::Value {
    let mut cfg = cfg.clone();
    cfg.drone.set_battery_capacity(capacity_j);
    let rules = EnergyRules::from_config(&cfg);
    serde_json::json!({
        "capacity_wh": capacity_j / J_PER_WH,
        "empty_mass_kg": cfg.drone.empty_mass_kg,
        "max_lane_m": rules.max_lane_m(cfg.drone.charge_target_soc, payload_kg),
    })
}

/// `sim battery`: range at `--payload-kg` with the configured battery, and the battery the lane
/// `--from -> --to` needs under `dispatch_ok`. With a coupled mass (A68) a bigger battery is
/// heavier and needs more energy, so the size is iterated to a fixed point; it is then rounded
/// up to `BATTERY_STEP_WH`.
fn battery(args: &[String]) -> ExitCode {
    let (mut config, mut payload_kg, mut from, mut to) = (None, None, None, None);
    let mut it = args.iter();
    while let Some(flag) = it.next() {
        let Some(value) = it.next() else {
            eprintln!("{flag} needs a value\n{USAGE}");
            return ExitCode::from(2);
        };
        match flag.as_str() {
            "--config" => config = Some(value.clone()),
            "--payload-kg" => {
                payload_kg = value
                    .parse::<f64>()
                    .ok()
                    .filter(|v| v.is_finite() && *v >= 0.0)
            }
            "--from" => from = Some(value.clone()),
            "--to" => to = Some(value.clone()),
            _ => {
                eprintln!("unknown flag {flag}\n{USAGE}");
                return ExitCode::from(2);
            }
        }
    }
    let (Some(config), Some(payload_kg), Some(from), Some(to)) = (config, payload_kg, from, to)
    else {
        eprintln!("--config, --payload-kg, --from and --to are required\n{USAGE}");
        return ExitCode::from(2);
    };
    let cfg = match parse_only(&config) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("config error: {e}");
            return ExitCode::from(2);
        }
    };
    let pad = |name: &str| {
        cfg.pads
            .iter()
            .position(|p| p.name == name)
            .and_then(|i| u16::try_from(i).ok())
    };
    let (Some(a), Some(b)) = (pad(&from), pad(&to)) else {
        eprintln!("--from and --to must name pads of the scenario");
        return ExitCode::from(2);
    };
    let net = Network::build(&cfg);
    let d = &cfg.drone;
    let usable_fraction = d.charge_target_soc - d.reserve_soc;
    // Iterate C <- C + shortfall(C), with shortfall = need / usable fraction - C. The shortfall
    // is convex in C (power grows as mass^1.5), so the steps shrink while it falls; once a step
    // grows again the shortfall is rising and no battery closes it.
    let shortfall_j = |c: f64| lane_need_j(&cfg, &net, (a, b), payload_kg, c) / usable_fraction - c;
    let mut capacity_j = d.battery_capacity_j;
    let mut closest = (capacity_j, shortfall_j(capacity_j));
    let mut previous_step = f64::INFINITY;
    let mut iterations = 0;
    let mut converged = false;
    while iterations < BATTERY_MAX_ITERATIONS {
        iterations += 1;
        let step = shortfall_j(capacity_j);
        if step.abs() < closest.1.abs() {
            closest = (capacity_j, step);
        }
        capacity_j += step;
        if step.abs() < BATTERY_TOLERANCE_J || d.mass_coupling.is_none() {
            converged = true;
            break;
        }
        if step > previous_step {
            break;
        }
        previous_step = step;
    }
    let first_need_j = lane_need_j(&cfg, &net, (a, b), payload_kg, d.battery_capacity_j);
    // R6 with a coupled mass: the battery may grow only until airframe + battery + latch +
    // payload reaches MTOW.
    let mtow_limit_wh = d.mass_coupling.map(|c| {
        (d.mtow_kg - d.latch_mass_kg - payload_kg - c.airframe_mass_kg) * c.specific_energy_j_per_kg
            / J_PER_WH
    });
    let sized = converged.then(|| {
        let rounded_j =
            (capacity_j / J_PER_WH / BATTERY_STEP_WH).ceil() * BATTERY_STEP_WH * J_PER_WH;
        let rounded_need_j = lane_need_j(&cfg, &net, (a, b), payload_kg, rounded_j);
        serde_json::json!({
            "required_capacity_wh": capacity_j / J_PER_WH,
            "rounded": battery_summary(&cfg, rounded_j, payload_kg),
            "rounded_passes": rounded_need_j <= rounded_j * usable_fraction,
            "within_mtow": mtow_limit_wh.is_none_or(|w| rounded_j / J_PER_WH <= w),
        })
    });
    let out = serde_json::json!({
        "config": config,
        "payload_kg": payload_kg,
        "lane": {"from": from, "to": to, "route_m": net.route_len(a, b)},
        "mass_coupled": d.mass_coupling.is_some(),
        "usable_fraction": usable_fraction,
        "configured": battery_summary(&cfg, d.battery_capacity_j, payload_kg),
        "need_above_reserve_at_configured_j": first_need_j,
        "required_capacity_without_coupling_wh": first_need_j / usable_fraction / J_PER_WH,
        "iterations": iterations,
        "fixed_point": converged,
        "sized": sized,
        "closest": {
            "capacity_wh": closest.0 / J_PER_WH,
            "shortfall_wh": closest.1 / J_PER_WH,
        },
        "mtow_limit_wh": mtow_limit_wh,
    });
    println!(
        "{}",
        serde_json::to_string_pretty(&out).expect("summary serialises")
    );
    ExitCode::SUCCESS
}

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().collect();
    if argv.get(1).map(String::as_str) == Some("flight-model") {
        return flight_model(&argv[2..]);
    }
    if argv.get(1).map(String::as_str) == Some("battery") {
        return battery(&argv[2..]);
    }
    if argv.get(1).map(String::as_str) == Some("dump") {
        return match argv.get(2) {
            Some(path) => dump(path),
            None => {
                eprintln!("usage: sim dump <events.bin>");
                ExitCode::from(2)
            }
        };
    }
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::from(2);
        }
    };
    let scenario = match Scenario::load(&args.config) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("config error: {e}");
            return ExitCode::from(2);
        }
    };
    let scenario = match args.seed {
        Some(seed) => scenario.with_seed(seed),
        None => scenario,
    };
    if args.replication >= scenario.cfg.replications {
        eprintln!(
            "--replication {} is outside sim.replications = {}",
            args.replication, scenario.cfg.replications
        );
        return ExitCode::from(2);
    }
    let outputs = match Outputs::to_dir_with(&args.out, scenario.cfg.source_hash, args.event_log) {
        Ok(o) => o,
        Err(e) => {
            eprintln!("cannot create outputs in {}: {e}", args.out.display());
            return ExitCode::FAILURE;
        }
    };
    let started = Instant::now();
    let mut world = scenario.into_world(args.replication, outputs);
    let summary = match world.run() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("run failed: {e}");
            return ExitCode::FAILURE;
        }
    };
    let text = serde_json::to_string_pretty(&summary).expect("summary serialises");
    if let Err(e) = std::fs::write(args.out.join("metrics.json"), format!("{text}\n")) {
        eprintln!("cannot write metrics.json: {e}");
        return ExitCode::FAILURE;
    }
    println!(
        "{}: {} events in {:.2} s wall, deliveries/h {:.2}, outputs in {}",
        summary["scenario"].as_str().unwrap_or("?"),
        summary["events_processed"],
        started.elapsed().as_secs_f64(),
        summary["deliveries_per_h"].as_f64().unwrap_or(0.0),
        args.out.display()
    );
    ExitCode::SUCCESS
}
