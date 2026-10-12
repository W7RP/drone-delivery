//! `sim run <config> --seed N --out <dir> [--replication R]` and `sim dump <events.bin>`.

use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Instant;

use drone_sim::output::Outputs;
use drone_sim::scenario::Scenario;

const USAGE: &str = "usage: sim run <scenario.toml> --seed N --out <dir> [--replication R]\n       sim dump <events.bin>";

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

fn main() -> ExitCode {
    let argv: Vec<String> = std::env::args().collect();
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
