//! `sim run <config> --seed N --out <dir> [--replication R]`, `sim dump <events.bin>`, and
//! `sim flight-model --lane-m L --payload-kg P [--config <scenario.toml>]`.

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use drone_sim::flight::VisitEstimate;
use drone_sim::output::Outputs;
use drone_sim::scenario::Scenario;

const USAGE: &str = "usage: sim run <scenario.toml> --seed N --out <dir> [--replication R]\n       sim dump <events.bin>\n       sim flight-model --lane-m L --payload-kg P [--config <scenario.toml>]";
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
}

fn parse_args() -> Result<Args, String> {
    let mut it = std::env::args().skip(1);
    if it.next().as_deref() != Some("run") {
        return Err(USAGE.into());
    }
    let config = it.next().ok_or(USAGE)?;
    let (mut seed, mut out, mut replication) = (None, None, 0);
    while let Some(flag) = it.next() {
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
    // Parse only: the flight model needs the drone section, not the network's energy rules
    // (R12, R16), which a profile under evaluation may not satisfy on every lane.
    let path = std::path::Path::new(&config);
    let cfg = match std::fs::read_to_string(path)
        .map_err(|e| drone_sim::config::ConfigError(format!("{config}: {e}")))
        .and_then(|text| drone_sim::config::parse_in(&text, path.parent()))
    {
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

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().collect();
    if argv.get(1).map(String::as_str) == Some("flight-model") {
        return flight_model(&argv[2..]);
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
    let outputs = match Outputs::to_dir(&args.out, scenario.cfg.source_hash) {
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
            eprintln!("run failed while writing outputs: {e}");
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
