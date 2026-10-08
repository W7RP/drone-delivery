# Phase 6b: weather and traffic realism

This phase moves the demo network to an airframe that can actually fly it, then adds weather
and road traffic. Every number is a placeholder (spec A01) unless it comes from a cited public
source, and every new behaviour sits behind a config key that is off by default. With the keys
absent, every committed scenario gives byte-identical outputs: all 35 seed-42 output files of
the Phase 5 scenarios, `events.bin` included, are checked against a build of the Phase 5 commit.

## 0. Run guards

Phase 6a met a protocol loop that wrote 1.8 GB of events in three seconds, and runaway probes
filled a disk. A run now aborts with an error (spec 5.5) in two cases:

- more than `sim.max_events_per_timestamp` events (default 10 000) fire at one sim time. Real
  runs peak at 12 (scale_200, Pittsburgh);
- `events.bin` would pass `sim.max_event_log_mib` (default 256; scale_200 writes 107 MB).

`sim run --no-event-log` skips the event log; every analysis script uses it, deletes each run's
outputs once read, and refuses to start with less than 20 GB free on the repo drive or the temp
drive. `sim/tests/run_guards.rs` checks both guards: a burst of 20 000 events at one timestamp,
standing in for a zero-time loop, aborts in 0.03 s with a log under 3 MB.

## 1. Yield fix

The open Phase 6a issue: no-parking layouts gridlocked. In the 1.5 kW sweep point with 2 OUT
spots, a drone diverted to an Oakland pad found both OUT spots charging, finished its service
on IN, and could neither charge there nor afford a flight. It blocked IN for hours, and loaded
drones held over the pad until reserve and made forced landings.

With `policy.yield_on_request` (spec A74), FM moves an idle, empty drone off a spot whenever a
drone holds over its pad, or a shipment or pod return there waits for an OUT spot. It tries, in
order:

- a parking slot with room;
- from IN, a free OUT spot, where the drone can also charge. The pad turns the PARK request
  into a ground hop (V44);
- a REPOSITION by the rebalancing policy;
- for a waiting pod only, the local yield (lift off, let the pod be placed, land again).

A drone is moved once until it reports again, so moves cannot loop.

| 6a sweep point, 1.5 kW, no parking, 2 OUT | Seed 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|
| Forced landings, without the fix | 0 | 7 | 6 | 6 | 3 |
| Forced landings, with the fix | 0 | 0 | 0 | 0 | 0 |
| Requests rejected, without / with | 0 / 0 | 730 / 0 | 52 / 0 | 158 / 0 | 0 / 0 |

`sim/tests/yield_fix.rs` checks seed 2 (7 forced landings without the fix, none with it). The
second 6a gridlock, pods stranded behind parked drones at 18 drones and 2 pods per warehouse,
goes from 97% of requests rejected to 1%, 1% and 11% (seeds 1 to 3).

The first version of V44 also changed behaviour with the key off: two 1370 Wh sweep points with
charging parking moved by up to 1.3%. It is now gated on the key, and both Phase 6a sweeps
reproduce their committed tables byte for byte.

## 2. The 900 Wh network

Phase 6a's layout and fleet results were for 1370 Wh, which no real airframe of this mass can
carry with a 5 kg pod (A68): infeasible airframe, not for demo. Phase 6b re-bases on the coupled
900 Wh network. It has the fitted flight profile, 5 pads (the two Strip District warehouses and
three Oakland hospitals), 5 drones, and a 4.5 kg airframe at 200 Wh/kg, which is exactly 9.0 kg.
Repair and the yield fix are on at every point.

`uv run analysis/sweep.py --base 900` runs the same grid as Phase 6a, with the same cost proxy
and selection rules. It finishes in about 9 s; `analysis/out/sweep_900wh.md` has the table.
Every point loads, and none rejects requests.

| Scenario | Cost (cu) | Door-to-door (s) | Pod wait (s) | Truck (s) |
|---|---|---|---|---|
| 900 Wh as in Phase 5 (1.5 kW, 1 OUT, hub 4 / others 2 parking) | 955 | 1659 ± 863 | 1033 ± 838 | 1939 |
| The same, coupled, with repair and yield | 955 | 1223 ± 165 | 610 ± 148 | 1939 |
| Chosen: 3 kW, 3 OUT spots, no parking | 1350 | 463 ± 9 | 65 ± 1 | 1939 |

The chosen layout is the cheapest within 10% of the best mean (422 s, at 12 kW with 3 OUT
spots). It is `configs/parts_logistics_pittsburgh_900wh_layout`, which `verify.sh` runs within
the release budget. The drone goes from 1.2 to 4.2 times faster than the truck on these short
Oakland flows.

Pareto set:

| Charger (kW) | Parking | OUT spots | Cost (cu) | Door-to-door (s) | Pod wait (s) |
|---|---|---|---|---|---|
| 1.5 | 0 | 1 | 775 | 1536 ± 190 | 886 |
| 1.5 | 0 | 2 | 950 | 686 ± 26 | 117 |
| 3 | 0 | 1 | 850 | 760 ± 56 | 301 |
| 3 | 0 | 2 | 1100 | 541 ± 43 | 107 |
| 3 | 0 | 3 | 1350 | 463 ± 9 | 65 |
| 6 | 0 | 1 | 1000 | 627 ± 42 | 227 |
| 6 | 0 | 3 | 1800 | 439 ± 30 | 75 |
| 12 | 0 | 3 | 2700 | 422 ± 27 | 75 |

As at 1370 Wh, no Pareto point has parking: once idle drones yield on request, OUT spots and
charger power do more per cost unit.

`uv run analysis/fleet.py --base 900` sizes pods and fleet at that layout (about 11 s,
`analysis/out/fleet_900wh.md`). Fleets of 16 and 18 fail to load (R4: a drone needs a starting spot, and 5 pads have 15).

| Criterion (fewest drones, then fewest pods) | Fleet | Pods per warehouse | Door-to-door (s) | Rejected |
|---|---|---|---|---|
| Under 5% rejected (the brief's) | 2 | 6 | 5769 ± 2044 | 1.0% |
| ... and faster than the truck (1939 s) | 4 | 2 | 802 ± 114 | 0.0% |
| ... and within 10% of the best (439 s) | 6 | 6 | 472 ± 26 | 0.0% |

The scenario as written (5 drones, 8 pods) sits between the last two. The yield fix removes
the Phase 6a pattern where more drones meant more rejections: with 2 pods per warehouse and 14
drones, nothing is rejected.
