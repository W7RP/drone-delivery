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

Phase 3 is done: the network can now be real. A Rust discrete-event core runs the spec (phase
2). It has no fixed time step, flat arrays indexed by ID, seeded streams, and a streamed event
log. It implements:

- every pad and drone transition, failure branch and silent-peer rule in the spec;
- the fleet manager's six policies, with simple defaults;
- pods as a finite circulating resource;
- a truck baseline with costs, and online metrics.

An offline Python pipeline (phase 3) builds a Pittsburgh network from public data:

- pads on real hospital, warehouse and industrial sites from OpenStreetMap;
- lanes kept clear of the FAA's no-fly cells around airport runways, with terrain and population
  sampled under each lane;
- a truck matrix routed over the OSM road network.

The sim loads those files instead of a generated network. There is no PX4/ROS flight yet; that
is the next phase.

What was learnt building it is in `docs/spec.md` section 11. Forcing each failure branch and
sweeping seeds with every fault at once found ten gaps in the phase-1 spec; each is fixed and
written back as an amendment. For example, a re-sent landing grant that did not restart its
timer let two drones be granted the same spot. The fleet manager also forgot drones it could
not place, and an order to a closed pad blocked its origin forever.

Results on synthetic networks and on Pittsburgh are in `docs/results.md`. Where pods circulate freely the
drone is about 2.7 times faster than the truck door to door. Where pods run short it loses,
because shippers wait for an empty pod; one OUT spot per pad, held through latch and charge,
is the network's real bottleneck.

On the Pittsburgh parts-logistics scenario, Strip District warehouses ship to Oakland
hospitals and Mon Valley plants. There the drone takes 1287 s door to door against the truck's
2263 s. The largest share of that is not flight but waiting for the pod to reach the OUT spot.

The real map also changed the network. The no-fly cells around the Allegheny County Airport
runways leave Clairton reachable only through Braddock, on a route too long for the energy
guard with a full pod. A warehouse closed by a fault, or three times the demand, grew the
request queue without limit, so pads now cap it and count the requests they turn away.

Every number behind those results is a placeholder.

- `docs/spec.md`: entities, records, pad state machine, handshake, event model, policies,
  metrics, config format, binary layouts, assumptions, amendments.
- `docs/paper-run.md`: a normal A-B delivery and a marker-miss, hold, divert case walked
  through the spec by hand.
- `docs/results.md`: drone against truck on four synthetic scenarios and on Pittsburgh, and the
  performance numbers.
- `docs/pipeline.md`: the data sources, the rules that pick pads and lanes, and what was
  rejected.
- `sim/`: the Rust core (`sim run`, `sim dump`) and its tests.
- `pipeline/`: the offline pipeline (`run.sh`), its rule files and its debug plot.
- `schemas/`, `configs/`: the machine-readable contract and the scenarios. `ab_synthetic`,
  `hub_spoke_6`, `mesh_small`, `scale_200` and `parts_logistics_pittsburgh` run; the other
  three presets are still stubs.

## Running it

Build, run, read the event log and run the tests: see `docs/running.md`. One command,
`scripts/verify.sh`, checks everything.

## Layout

```
docs/       spec.md, paper-run.md, results.md, running.md, pipeline.md
schemas/    common, config, entities, events (JSON Schema)
configs/    one directory per scenario, each with scenario.toml (and derived/ from the pipeline)
sim/        Rust crate: src/ (core), src/bin/sim.rs (CLI), tests/
pipeline/   offline data pipeline: run.sh, ddpipe/ (Python), sources/sites/params YAML, tests/
scripts/    verify.sh, check_spec.py, scale_demand.py
```
