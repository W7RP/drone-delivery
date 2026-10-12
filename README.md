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

Phase 8 is done: one VTOL delivery visit is flown in PX4 SITL over real Pittsburgh terrain and buildings and cut into a clip with a HUD from the logs. Phase 7 flew, fitted and ran a VTOL over three years of weather. A Rust discrete-event core runs the spec (phase
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

The sim loads those files instead of a generated network.

In phase 4, ROS 2 nodes fly one Pittsburgh lane in PX4 SITL and Gazebo, headless. The flight
follows the spec's own handshake:

- a drone agent and two pad controllers exchange the spec's messages under its transition IDs;
- an ArUco detector feeds PX4's precision land, and a scripted latch joint drops and picks up
  pods;
- every flight is logged in the spec's flight-log formats and checked by a validator.

Fifteen visits all completed. Touchdowns landed a mean of 4 cm from the IN spot and 9 cm from
OUT. The motor commands give an energy proxy that rises with payload as the sim's flight model
expects (`docs/flight-runs.md`).

Two things only the simulator showed:

- At first the drone hovered over the wrong spot after 2 km. Gazebo's GPS uses the WGS84
  ellipsoid and PX4's local frame a sphere, so the two disagreed by 5 m over the lane. Pads now
  reach the drone as latitude and longitude, as they would on a real vehicle.
- A 0.4 m marker seen from 8 m was too small to decode reliably.

The sim's flight model is now fitted to those logs (`docs/calibration.md`). Phase times move by a
few seconds, but every visit costs about twice the placeholder energy: the energy here is a
momentum-theory proxy from the motor commands, since SITL has no power model. On four held-out
flights, on a 441 m and a 12.9 km lane, the fit predicts time within 1.2% and energy within 5.1%.
The 12.9 km lane also showed that Gazebo's GPS altitude drifts 13 m from its flat world, so SITL
now takes height from the barometer.

With the fitted profile and the 900 Wh battery, a drone carrying 5 kg reaches 5.7 km instead of
21 km. Pittsburgh shrinks to the Strip District and Oakland pads. A 1370 Wh battery gets every
Phase 3 lane back, but then each recharge holds the OUT spot long enough that the drone becomes
slower than the truck door to door. The Pittsburgh scenario selects its flight profile in its
config; the placeholder profile and every result built on it are kept.

Phase 6a adds realism and then attacks that bottleneck (`docs/network-realism.md`):

- **Battery mass:** once a bigger battery also weighs more, no battery serves the longest
  Pittsburgh lane with a 5 kg pod, and MTOW caps it at 1180 Wh.
- **Repair:** pads grounded by a release fault are repaired, which removes the 13.6% of requests
  Phase 3 turned away.
- **Fault visibility and messaging:** pads can report an unreadable marker, swapped radio
  messages are put back in order, and latch failures follow the measured landing error.
- **Layout:** pads can have up to three OUT spots and charging parking. At 1370 Wh, two OUT
  spots and 6 kW chargers take door-to-door from 9710 s to 735 s, now 3.1 times faster than the
  truck (1370 Wh: infeasible airframe, not for demo).
- **Opt-in:** every new behaviour is set by optional config keys, so existing scenarios and
  their results are unchanged.

Phase 6b moves the demo to the coupled 900 Wh airframe and adds weather and traffic
(`docs/weather-traffic.md`):

- **Guards and yield:** runs abort on a zero-time loop or an oversized event log, and idle
  drones make room for arriving ones, which ends the no-parking gridlock of Phase 6a.
- **900 Wh layout:** 3 kW chargers and three OUT spots per pad take door-to-door from 1659 s to
  463 s, 4.2 times faster than the truck in good conditions.
- **Weather:** three years of hourly AGC airport observations drive wind energy, operating
  limits, landing error, battery derate and truck speed.
- **Traffic:** truck routes are split by road class from the pipeline's own routing, with
  time-of-day profiles and random incidents.
- **Results:** over 156 weeks the median week is 1.8 times faster than the truck (2.4 times
  with a truck fallback), but the worst tenth of weeks are over six times slower. Cold
  derates the marginal battery below the energy guard, and hours over the limits stop
  dispatch. The wind limits matter most, then the cold derate.

Phase 7 first makes the results honest about unfinished orders, then flies a VTOL
(`docs/weather-traffic.md` sections 8 to 10, `docs/vtol.md`):

- **Completion:** every request now ends in one outcome class. The 6b ratios of the windiest
  and coldest weeks rested on 40% and 18% of requests; most of the rest were turned away at the
  backlog cap.
- **Energy route guard:** a cold or windy hour now blocks only the routes it makes infeasible,
  and the truck fallback takes those orders. With it, the multirotor demo completes 98 to 99%
  of those two weeks.
- **VTOL:** PX4's standard VTOL scaled to 16 kg flies the delivery visit in SITL. A profile
  fitted to six flights predicts four held-out flights (1.2 and 12.9 km) within 1.0% in time
  and 1.5% in energy proxy. A coupled 620 Wh battery reaches 14.7 km with 5 kg, which brings
  back Edgar Thomson and Josh Steel; Clairton is in the network but still out of range.
- **Year:** with a truck fallback the VTOL demo's median week is about as fast as the truck
  (ratio 0.98, p10 to p90 0.63 to 1.36, 98.5% completed). The multirotor's is faster (0.42) but
  its worst tenth of weeks are over six times slower. The VTOL battery has no margin for wind
  or cold on the Mon Valley routes, so most of their orders go by truck.
- **Wind:** in a SITL crosswind the OUT landing is the weak point: at 8 m/s both visits
  missed the latch's capture radius.

Phase 8 renders one delivery for a clip (`docs/hero-clip.md`), Gazebo only:

- **Lane and world:** the lane is picked by rule: drone-delivered in the VTOL demo, crossing
  water, with the worst road detour. That is lane 5, 11.1 km from the Strip District hub to
  Josh Steel over the Monongahela. A corridor world is generated from the cached DEM and OSM
  data (terrain, rivers, roads, 1635 buildings), byte-identical on rerun.
- **Flight:** the Phase 7 VTOL flies the full visit over that world at 1x, cruising 190 m above
  the hub to clear the ridge. Two fixes were needed: the barometer read the world's elevation
  only after the EKF origin was set, and baro drift lowered the hop at B, so the hop is now
  measured from the landed position. The path keeps at least 62 m from terrain and 56 m from
  buildings.
- **Clip:** an 82 s cut from three render cameras on massless models, by rules on the event
  log. A HUD shows the states, handshake messages, sim time, height and battery, plus the
  demo's drone-versus-truck figures for the lane (18.2 against 41.3 min). A test checks every
  HUD change against the logs within one frame.
- **Terrain:** over flat ground the fitted flight model predicts the 11.1 km visit within 1% in
  time and energy proxy. Over terrain it is 11% short in time and 25% in energy, because the
  core has no altitude and so misses the climb and descent.

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
- `px4/`: the SITL flight: world and vehicle generators, C++ ROS 2 nodes, `run_flight.sh`, the
  log reader and validator, and a sample log.
- `docs/flight-runs.md`: the fifteen SITL visits: times, energy proxy, landing errors, speed
  factor, memory.
- `docs/network-realism.md`: battery mass, truck fairness, repair, marker faults, message
  reordering, latch failures, and the charging, layout, pod and fleet sweeps (`analysis/`).
- `docs/weather-traffic.md`: run guards, the yield fix, the 900 Wh layout and fleet, the
  weather series, weather and traffic in the sim, the demo scenario, and the year of weather.
- `docs/calibration.md`: the flight model fitted to the SITL logs, held-out flights, and what
  the fit changes for range, lanes, battery and the Pittsburgh week.
- `docs/vtol.md`: the VTOL airframe, its calibration flights and fit, held-out flights, the
  wind check, its network, layout and fleet, and the year compared with the multirotor.
- `docs/hero-clip.md`: the hero clip: render check, lane rule, corridor world, the flight over
  terrain, cameras, edit and HUD, and terrain against flat.
- `docs/merge-plan.md`: how to merge the phase branches into `main`.
- `schemas/`, `configs/`: the machine-readable contract and the scenarios. `ab_synthetic`,
  `hub_spoke_6`, `mesh_small`, `scale_200` and the `parts_logistics_pittsburgh` variants
  (fitted profile, placeholder profile, fitted with a 1370 Wh battery, the 1370 Wh and 900 Wh
  layouts, the Phase 6b demo, and the VTOL network, layout and demo of Phase 7) run; the other three presets are still stubs.

## Running it

Build, run, read the event log and run the tests: see `docs/running.md`. One command,
`scripts/verify.sh`, checks everything.

## Layout

```
docs/       spec.md, paper-run.md, results.md, running.md, pipeline.md, flight-runs.md,
            calibration.md, network-realism.md, weather-traffic.md, vtol.md, hero-clip.md,
            merge-plan.md, media/ (calibration plots, clip stills)
schemas/    common, config, entities, events (JSON Schema)
configs/    one directory per scenario, each with scenario.toml (and derived/ from the pipeline);
            flight_profiles/ (the fitted profiles); weather/ (the hourly series)
sim/        Rust crate: src/ (core), src/bin/sim.rs (CLI), tests/
pipeline/   offline data pipeline: run.sh, ddpipe/ (Python), sources/sites/params YAML, tests/;
            calib/ (flight model calibration); hero/ (Phase 8 lane, world, panel, checks)
px4/        SITL flight: run_flight.sh, ros2_ws/ (C++ nodes), sim/ (worlds, systems/), config/,
            tools/, tests/, samples/
scripts/    verify.sh, check_spec.py, scale_demand.py, hero_clip.sh
video/      the rendered clip (git-ignored)
analysis/   Phase 6a and 6b analysis scripts (uv run) and their results in out/
```
