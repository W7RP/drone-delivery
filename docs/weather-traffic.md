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

## 3. Weather data

`pipeline/run.sh` now also writes `configs/weather/pittsburgh/`: `weather.bin` (spec 9.7, 684
KB), `report.json` and `manifest.json`. The source is the Iowa Environmental Mesonet archive of
ASOS routine reports, 2023 to 2025 in UTC, for AGC (Allegheny County Airport) and PIT; it needs
no key. Both CSVs (1.4 MB each) are cached under `pipeline/data/raw/` with their sha256 in the
manifest, so a rebuild runs offline. Two rebuilds in a row gave byte-identical files, all 19
pipeline outputs included.

Each hour holds the 10 m wind speed and direction, gust, temperature, visibility, 1 h
precipitation and present-weather flags (rain, snow, freezing, thunder), in SI. How reports
become hours (spec A75):

- a report belongs to the nearest whole hour (the hh:53 routine report is hour hh+1);
- a missing value keeps the last one for up to 3 h, after which the hour is marked missing.
  It keeps the last value, so a flight in progress still has numbers;
- no gust group means gust equals wind, a trace or missing 1 h precipitation is 0, and a
  variable wind keeps the last direction;
- a wind speed at direction 0 (the calm code) is dropped as impossible: 33 kt on 2023-06-29
  and 90 kt on 2023-07-01.

| Station | Reports | Hours carried forward | Hours missing | Variable-wind hours |
|---|---|---|---|---|
| AGC (used) | 25 981 | 577 | 198 (0.75%) | 1949 |

AGC is used because under 2% of its hours stay missing (placeholder rule); PIT is only the
fallback. Over the three years at AGC, the 10 m wind has a median of 3.1 m/s, a p90 of 5.7 m/s
and a maximum of 14.4 m/s; the highest gust is 28.3 m/s. Temperature runs from -20.6 to 35.6
deg C. There are 389 hours below 1600 m visibility, 1659 with rain and 603 with snow.

The station is not at the lanes (spec A76): AGC is 10 to 12 km south of the five pads, at an
open airfield. River valleys, building wake and rooftop turbulence are not modelled. Scaling
the 10 m wind to cruise height happens in the sim (section 4).

## 4. Weather in the sim

A scenario with a `[weather]` section (spec 2.10, schema 1.8.0) reads `weather.bin` and steps
through it hour by hour from `weather.start_hour`. Without the section nothing changes: the 35
seed-42 output files of the Phase 5 scenarios and both 900 Wh sweep tables are byte-identical.
Every value in the section is a placeholder (assumption = true); the landing and marker
effects are uncalibrated and should be validated with PX4 wind flights.

What one hour does:

- Flight (A77, A78). The 10 m wind is scaled to cruise height (power law, exponent 0.2) and
  split along and across each lane's track, so the two directions of a lane cost different
  energy. The drone holds its fitted ground speed. If the airspeed that needs exceeds
  `max_airspeed_mps`, it flies at that airspeed and slows over the ground. Cruise power
  against airspeed is ideal momentum theory with the Phase 5 audit's air density and disc
  area, plus a parasitic term chosen so the curve passes through the fitted 2108 W at
  15.03 m/s. In still air it reduces to the fitted model exactly. Takeoff, approach and hop
  stay as fitted. The energy guards use the same numbers, so a headwind can strand a drone
  that a calm hour would have sent.
- Operating limits (A79). Outside the wind, gust, visibility, precipitation or temperature
  limits FM assigns nothing and sends no drone to another pad. An hour missing from the series
  blocks dispatch too. Landing clearance has its own wind and gust limits, at or above the
  dispatch ones. Beyond them pads clear no landings: a drone that wants to land holds, reports
  WEATHER, and the existing hold and divert logic takes over.
- Landing (A80). Gusts widen the OUT landing error that the Phase 6a latch-fail model is
  derived from, and rain, snow or low visibility cut marker detection.
- Battery (A81). Usable capacity follows a derate table against air temperature.
- Truck (A82). Rain, snow and low visibility slow the truck, in the baseline as well, so both
  modes see the same weather.
- Hold policy (A83). `WAIT` keeps blocked orders waiting. With `TRUCK_FALLBACK`, a shipment
  that has waited the timeout during a block goes by truck: from the backlog, or unpacked from
  its pod on an OUT spot if no drone is assigned yet. It is closed TRUCKED with the truck's
  arrival time.
- Metrics (spec M10, only with weather): flyable-hour fraction, orders flown and trucked and
  their shares, and weather aborts (landing requests held for weather).

`sim/tests/weather.rs` forces each behaviour against a control run on a hand-written series.
Each test has a control, so it would fail if the behaviour were missing:

- a gale blocks every lane (a calm series flies);
- a gusty hour clears no landing and makes the drone hold (a calm series has no aborts);
- TRUCK_FALLBACK trucks shipments at the timeout, or when a placement under way at that moment
  finishes (WAIT trucks none);
- at -20 deg C, with a 0.7 derate, range drops below 80% of the warm range.

Separate landing limits are a change from the brief, which asked for one set of limits checked
at dispatch and at landing. Hourly data switch the weather at the top of the hour. With one
set, every drone in the air when a windier hour began could not land. It held, diverted to a
pad with the same weather, and was forced down at its reserve. In a January 2025 week of the
900 Wh layout this grounded all five drones. With landing limits of 14 m/s wind and 20 m/s gust
(dispatch: 10 and 15, all placeholders) that week has no forced landing. A drone still meets
that fate when an hour exceeds the landing limits; the gust test shows it with a one-drone
fleet.

The same trial week (2025-01-06, mean -6 deg C, low -14 deg C) shows how much the 900 Wh drone
depends on usable capacity. With a flat derate and no limits, it delivers normally at 0.9 of
capacity (1319 deliveries, 523 s mean). At 0.85 it manages 615, with 808 diversions. At 0.8 it
delivers nothing: no route passes the energy guard. The derate table is a placeholder, so how
winter weeks come out depends on it; section 7 varies it.
