# Running it

## Prerequisites

- Linux or WSL2 (developed on WSL2, kernel 6.18).
- Rust 1.99.0. `sim/rust-toolchain.toml` pins it; with rustup installed, cargo picks it up
  inside `sim/`. Install rustup into your home directory from
  https://rustup.rs (no root needed).
- [uv](https://docs.astral.sh/uv/) for the spec checker. Its dependencies are pinned inside
  `scripts/check_spec.py` and go to uv's cache, not the system Python.
- `/usr/bin/time` (GNU time) for the budget checks in `verify.sh`.
- For the offline pipeline only: about 1 GB of disk for downloads and 1.5 GB of RAM. uv
  installs Python 3.12 and the pinned wheels into `pipeline/.venv`; see `docs/pipeline.md`.

## Build

```
cd sim
cargo build --release
```

## Run a scenario

```
sim/target/release/sim run configs/ab_synthetic/scenario.toml --seed 42 --out runs/ab_synthetic/seed42_rep0
```

The Pittsburgh scenario runs the same way; its network comes from the committed files in
`configs/parts_logistics_pittsburgh/derived/`:

```
sim/target/release/sim run configs/parts_logistics_pittsburgh/scenario.toml --seed 42 --out runs/pittsburgh
```

To rebuild those files and the scenario from public data, run `pipeline/run.sh`
(`docs/pipeline.md`). The first run downloads about 900 MB; every run after that works offline.

`--seed` overrides `sim.seed` (the generated network still comes from the config's own seed,
spec A41). `--replication R` picks replication R of `sim.replications`. A config with an error
is rejected before anything runs, with the key and the rule it broke:

```
config error: drone.hop_s: unit "ms", expected "s"
```

The output directory gets (spec 8.4):

| File | What |
|---|---|
| `metrics.json` | Every section 7 metric, drone against truck, cost and break-even. |
| `deliveries.csv` | One row per order: times, outcome, truck time and cost, and the drone time split into pod wait, queue, pad service and flight. |
| `alerts.jsonl` | Every alert raised for remote ops. |
| `events.bin` | Every event, lost message and state transition, streamed (48 bytes each). |
| `flight_log.bin` | One record per lane flown (spec 9.3). |

`events.bin` is large for long runs (112 MB for the 7-day scale case). To read it:

```
sim/target/release/sim dump runs/ab_synthetic/seed42_rep0/events.bin | less
```

Transition records print as their spec row IDs (`V06`, `D05`, ...), so a run can be read
against spec section 3 and 4.4 line by line.

## Test and verify

```
./scripts/verify.sh
```

runs everything and stops at the first failure:

- the spec/schema/config cross-checks;
- the pipeline unit tests;
- rustfmt and clippy with warnings as errors;
- all Rust tests in a debug build, where invariants are checked after every event;
- release runs of the 200-drone scale case, the Pittsburgh week and Pittsburgh at 3x demand,
  each against the 60 s and 200 MB budget.

`VERIFY_SKIP_SCALE=1` skips the release runs.

Individual suites, from `sim/`:

| Command | Covers |
|---|---|
| `cargo test --test failure_branches` | Each spec 3.6 failure branch forced, exact transition sequence. |
| `cargo test --test fault_sweep` | 30 seeds with every fault type at once. |
| `cargo test --test determinism` | Byte-identical outputs per seed; other seeds differ. |
| `cargo test --test golden` | A-B summary for seed 42. `UPDATE_GOLDEN=1` rewrites it; review the diff. |
| `cargo test --test config_validation` | Bad configs rejected with a named error. |
| `cargo test --test scenarios` | Hub-and-spoke, mesh and a 6-hour scale slice end to end. |
| `cargo test --test network_files` | Hand-made network files round-trip; broken ones are rejected by rule; the Pittsburgh files load and run. |
| `cargo test --test backlog_cap` | A full backlog rejects requests (P08) and counts them (M9). |
