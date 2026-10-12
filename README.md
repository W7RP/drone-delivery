# drone-delivery

A fully simulated B2B drone delivery network: fixed lanes between autonomous pads, pods that
circulate like the drones do, and a truck baseline serving the same orders over roads.
No residential delivery. The network is a graph of pads and lanes, so an A-B shuttle, a
hub-and-spoke and a mesh run on the same engine and differ only in config.

Each pad has two passive spots. A drone lands on IN and drops its pod, hops to OUT, latches
the pod a person left there, charges on OUT's contacts and leaves. A pad controller owns the
pad and the spot assignments; the drone does fiducial precision landing and owns its latch; a
fleet manager decides who flies where. People appear only as "pod placed" events and as the
recipients of alerts.

## Status

Phase 2 is done: a Rust discrete-event core runs the spec. It has no fixed time step, flat
arrays indexed by ID, seeded streams, and a streamed event log. The core implements every
pad and drone transition, failure branch and silent-peer rule in the spec, the fleet
manager's six policies with simple defaults, pods as a finite circulating resource, a truck
baseline with costs, and online metrics. There is still no Python pipeline and no PX4/ROS
flight; those are the next phases.

What was learnt building it is in `docs/spec.md` section 11. Forcing each failure branch and
sweeping seeds with every fault at once found ten gaps in the phase-1 spec; each is fixed and
written back as an amendment. For example, a re-sent landing grant that did not restart its
timer let two drones be granted the same spot. The fleet manager also forgot drones it could
not place, and an order to a closed pad blocked its origin forever.

Results on synthetic networks are in `docs/results.md`. Where pods circulate freely the
drone is about 2.7 times faster than the truck door to door. Where pods run short it loses,
because shippers wait for an empty pod; one OUT spot per pad, held through latch and charge,
is the network's real bottleneck. Every number behind those results is a placeholder.

- `docs/spec.md`: entities, records, pad state machine, handshake, event model, policies,
  metrics, config format, binary layouts, assumptions, amendments.
- `docs/paper-run.md`: a normal A-B delivery and a marker-miss, hold, divert case walked
  through the spec by hand.
- `docs/results.md`: drone against truck on four scenarios, and the performance numbers.
- `sim/`: the Rust core (`sim run`, `sim dump`) and its tests.
- `schemas/`, `configs/`: the machine-readable contract and the scenarios. `ab_synthetic`,
  `hub_spoke_6`, `mesh_small` and `scale_200` run; the four presets are still stubs.

## Running it

Build, run, read the event log and run the tests: see `docs/running.md`. One command,
`scripts/verify.sh`, checks everything.

## Layout

```
docs/       spec.md, paper-run.md, results.md, running.md
schemas/    common, config, entities, events (JSON Schema)
configs/    one directory per scenario, each with scenario.toml
sim/        Rust crate: src/ (core), src/bin/sim.rs (CLI), tests/
scripts/    verify.sh, check_spec.py
```
