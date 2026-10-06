# drone-delivery

A fully simulated B2B drone delivery network: fixed lanes between autonomous pads, pods that
circulate like the drones do, and a truck baseline serving the same orders over real roads.
No residential delivery. The network is a graph of pads and lanes, so an A-B shuttle, a
hub-and-spoke and a mesh run on the same engine and differ only in config.

Each pad has two passive spots. A drone lands on IN and drops its pod, hops to OUT, latches
the pod a person left there, charges on OUT's contacts and leaves. A pad controller owns the
pad and the spot assignments; the drone does fiducial precision landing and owns its latch; a
fleet manager decides who flies where. People appear only as "pod placed" events and as the
recipients of alerts.

## Status

Phase 1 is done: the specification and interfaces are frozen. There is no simulator code yet.

- `docs/spec.md`: entities, record schemas, pad state machine, pad-drone handshake, event
  model and seeding, fleet manager policy interfaces, metrics, config format, binary file
  layouts, assumptions.
- `docs/paper-run.md`: a normal A-B delivery and a marker-miss, hold, divert case walked
  through the spec step by step, plus the gaps that walk found.
- `schemas/`: JSON Schema (2020-12) for the config, entity records, and events/messages.
- `configs/ab_synthetic/`: a complete example scenario. `configs/<preset>/`: stubs for the
  parts logistics, hospital/lab, retail transfer and remote terrain presets (Pittsburgh).

Every number in `configs/` is a placeholder flagged `assumption = true`. None is real data.

Planned shape: a Rust discrete-event core, a Python offline pipeline that builds the lane
graph and truck matrix, and PX4/Gazebo/ROS 2 flights, decoupled through files, for flight
model calibration and video.

## Checking the contract

`scripts/verify.sh` (needs [uv](https://docs.astral.sh/uv/)) validates every config against
the schema, applies the cross-field rules from spec section 8.3, and cross-checks the spec's
tables, message lists, state names and config key references against the schemas. It exits
non-zero on any mismatch.

## Layout

```
docs/       spec.md, paper-run.md
schemas/    common, config, entities, events (JSON Schema)
configs/    one directory per scenario, each with scenario.toml
scripts/    verify.sh, check_spec.py
```
