//! Spec 5.5: run guards. A run aborts with an error, quickly and with a small event log, when
//! too many events fire at one sim timestamp (a zero-time loop), or when its event log reaches
//! its cap. Phase 6a found such a loop write 1.8 GB in three seconds.

mod common;
use common::*;
use drone_sim::event::Ev;
use drone_sim::ids::{EntityKind, TimerKind};
use drone_sim::output::Outputs;
use drone_sim::scenario::Scenario;
use drone_sim::world::RunError;
use std::time::{Duration, Instant};
use toml::Table;

const DEFAULT_MAX_EVENTS_PER_TIMESTAMP: u32 = 10_000;
const DEFAULT_CAP_BYTES: usize = 256 << 20;

fn world(t: &Table) -> drone_sim::world::World {
    let s = Scenario::from_toml(&toml::to_string(t).expect("serialise")).expect("loads");
    let hash = s.cfg.source_hash;
    s.into_world(0, Outputs::in_memory(hash))
}

#[test]
fn a_zero_time_loop_aborts_fast_and_small() {
    let mut w = world(&base());
    // Twice the default limit of events at one timestamp, as a zero-time loop would produce.
    let t_ms = 1_000;
    for _ in 0..2 * DEFAULT_MAX_EVENTS_PER_TIMESTAMP {
        w.schedule(
            t_ms,
            Ev::Timer {
                timer: TimerKind::DHoldRetry,
                owner_kind: EntityKind::Drone,
                owner_id: 0,
                subject: 0,
                generation: u32::MAX,
            },
        );
    }
    let started = Instant::now();
    match w.run() {
        Err(RunError::Watchdog { t_ms: at, events }) => {
            assert_eq!(at, t_ms);
            assert_eq!(events, DEFAULT_MAX_EVENTS_PER_TIMESTAMP + 1);
        }
        other => panic!("expected the watchdog, got {other:?}"),
    }
    assert!(started.elapsed() < Duration::from_secs(5));
    let log = w.out.events.bytes().len();
    assert!(log < DEFAULT_CAP_BYTES / 100, "event log {log} bytes");
}

#[test]
fn the_event_log_cap_stops_a_run() {
    let mut t = base();
    set(&mut t, "schema_version", "1.7.0");
    set_q(&mut t, "sim.duration_s", 604_800.0);
    let toml::Value::Table(sim) = t.get_mut("sim").expect("sim") else {
        unreachable!()
    };
    sim.insert("max_event_log_mib".into(), quantity(1, "count"));
    let mut w = world(&t);
    match w.run() {
        Err(RunError::EventLogCap { bytes, .. }) => assert_eq!(bytes, 1 << 20),
        other => panic!("expected the event-log cap, got {other:?}"),
    }
    assert!(w.out.events.bytes().len() <= 1 << 20);
}

#[test]
fn normal_runs_stay_far_from_the_guards() {
    let r = run(&example());
    assert!(r.events_bin.len() < DEFAULT_CAP_BYTES / 100);
}
