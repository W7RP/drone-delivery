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

## 5. Traffic

There is no live traffic data. The pipeline splits each truck route by the roads its
free-flow path uses (spec 9.2, version_minor 2): the share of its length on motorway and trunk,
on primary and secondary, and on everything else (the pad snap legs included), plus the share
on a bridge or in a tunnel. Free-flow times and road distances did not change by a byte, and
the sim still reads minor-1 files. Mean shares over the pad pairs:

| Network | Motorway/trunk | Primary/secondary | Other | Bridge/tunnel |
|---|---|---|---|---|
| 900 Wh (5 pads, Strip District and Oakland) | 0.06 | 0.19 | 0.75 | 0.01 |
| 9 pads (with the Mon Valley plants) | 0.17 | 0.50 | 0.34 | 0.07 |

With `[traffic]` (spec 2.11) a truck's road time is its free-flow time times
`sum(class share x (1 + class peak delay x shape(hour)))`, plus the bridge/tunnel share times
its own extra delay, with separate weekday and weekend shapes. The `truck.tod_speed_factors`
must then be flat (R22), so time of day is not counted twice. Incidents add a seeded random
delay: a Poisson count per directed pair and hour, each lognormal. Each pair-hour draws from
its own seed, so a truck's delay does not depend on what else was asked first. Every number
is a placeholder (A84 to A86).

`sim/tests/traffic.rs` checks:

- a minor-1 truck matrix still loads and gives the same day of deliveries as the minor-2 file;
- the peak slows each route by exactly its class mix, and weekends by their own profile;
- incidents only add time, repeat within an hour, and hit most hours at a rate of 2 per hour;
- traffic with time-of-day factors that are not flat is rejected.

## 6. Demo scenario

`configs/parts_logistics_pittsburgh_demo` is the 900 Wh layout (3 kW, 3 OUT spots, no parking,
coupled mass, repair, yield on request) with every realism key on: the derived latch fault
(Phase 4 Rice fit), the marker self-check, a 1 s reorder window, weather from the AGC series
and traffic. It starts at local midnight on Monday 2023-01-02 (series hour 29). The truck's
time-of-day factors are flat because traffic carries time of day. The 0.955 marker value is
not applied (`p_marker_detect` stays 0.97), and no existing scenario file changed.

Seed 42, that week: 7.91 deliveries/h, door-to-door mean 1008 s (p50 460 s, p95 5530 s)
against 1926 s by truck, 97% of hours flyable, no weather aborts and no forced landing.
`verify.sh` runs it within the release budget.

TRUCK_FALLBACK only covers hours the operating limits block. Drones that cold has left too
short of usable capacity to pass the energy guard are not blocked by the limits, so their
orders wait (in the week of 2025-01-06, 6 shipments flown and 431 trucked with fallback on).

## 7. A year of weather, 2023 to 2025

`uv run analysis/weather_year.py` runs the demo over every complete week of the series: 156
weeks from Monday 2023-01-02 to Monday 2025-12-22, each starting at local midnight, seeds 1 to
3, under both hold policies. It then runs the WAIT sensitivity sweep, 4680 runs in all. It
takes about 160 s with 16 runs in parallel. It deletes every run's outputs once read, so temp use peaks
around 17 MB. A second run writes byte-identical tables (`analysis/out/weather_year.md`,
`.json`).

The ratio is the network's mean door-to-door over the truck baseline's on the same orders:
shipments requested in the window and completed, either flown and delivered or trucked. Below
1 the network is faster. Orders still waiting at the end of the week, diverted or failed, and
requests turned away at the backlog cap, are not in the ratio; the completed share counts
them. A week's value is the mean of its three seeds; mean, p10, median and p90 are over weeks.

| Policy | Ratio mean | p10 | Median | p90 | Flyable hours | Trucked share | Completed share | Weeks without a completed order |
|---|---|---|---|---|---|---|---|---|
| WAIT | 3.17 | 0.26 | 0.56 | 6.61 | 94% | 0% | 87% | 2 |
| TRUCK_FALLBACK | 2.14 | 0.26 | 0.41 | 6.31 | 94% | 7% | 91% | 0 |

| Week | Rule | Wind mean (m/s) | Gust max | Temp mean (C) | Snow h | Policy | Network (s) | Pod wait | Queue | Pad service | Flight | Truck, same flown orders (s) | Ratio | Flyable | Trucked | Completed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2023-10-23 | median wind | 3.1 | 10.8 | 15.2 | 0 | WAIT | 569 | 108 | 161 | 149 | 152 | 1919 | 0.30 | 90% | 0% | 91% |
| 2023-10-23 | median wind | 3.1 | 10.8 | 15.2 | 0 | TRUCK_FALLBACK | 920 | 108 | 161 | 149 | 152 | 1919 | 0.48 | 90% | 7% | 98% |
| 2024-01-08 | windiest | 6.0 | 26.2 | 1.4 | 20 | WAIT | 21390 | 18007 | 2994 | 237 | 153 | 1922 | 11.13 | 78% | 0% | 40% |
| 2024-01-08 | windiest | 6.0 | 26.2 | 1.4 | 20 | TRUCK_FALLBACK | 24915 | 10785 | 2682 | 232 | 152 | 1950 | 12.81 | 78% | 50% | 56% |
| 2025-01-20 | coldest | 3.9 | 13.9 | -8.6 | 10 | WAIT | 77039 | 73541 | 3149 | 200 | 149 | 1892 | 40.72 | 51% | 0% | 18% |
| 2025-01-20 | coldest | 3.9 | 13.9 | -8.6 | 10 | TRUCK_FALLBACK | 13220 | 6550 | 1385 | 205 | 148 | 1882 | 6.89 | 51% | 72% | 72% |

| Sensitivity (WAIT) | Ratio mean | Change | p10 | Median | p90 | Completed share |
|---|---|---|---|---|---|---|
| wind limits x0.5 | 8.28 | 5.110 | 0.64 | 3.58 | 15.97 | 72% |
| cold derate depth x1.5 | 7.12 | 3.947 | 0.26 | 0.87 | 16.16 | 76% |
| cold derate depth x0.5 | 0.80 | -2.367 | 0.26 | 0.40 | 1.66 | 96% |
| wind limits x1.5 | 3.02 | -0.149 | 0.26 | 0.53 | 5.96 | 88% |
| traffic peak delay x0.5 | 3.20 | 0.028 | 0.26 | 0.56 | 6.68 | 87% |
| traffic peak delay x1.5 | 3.14 | -0.027 | 0.26 | 0.55 | 6.55 | 87% |
| landing gust k x1.5 | 3.19 | 0.020 | 0.26 | 0.56 | 6.95 | 87% |
| landing gust k x0.5 | 3.16 | -0.010 | 0.25 | 0.55 | 6.48 | 88% |

What it shows, every number resting on placeholders (A77 to A86):

- In a typical week the network beats the truck. The median week takes 0.56 of the truck's
  time under WAIT and 0.41 with TRUCK_FALLBACK, and the best tenth of weeks 0.26 (about four
  times faster). On average 94% of hours are flyable.
- The mean is dominated by bad weeks. The worst tenth of weeks are more than six times slower
  than the truck. There, cold derates the marginal 900 Wh battery below what the energy guard
  accepts, and hours over the limits stop dispatch, so orders wait for days.
- TRUCK_FALLBACK trucks 7% of orders over the three years, lifts the completed share from 87%
  to 91%, and cuts the mean ratio from 3.17 to 2.14. In the coldest week it trucks 72% and
  takes the ratio from 40.7 to 6.9. It cannot help when cold strands drones without crossing a
  limit (section 6). In a mild week it makes the ratio worse (0.30 to 0.48): orders that would
  have waited out a short block and still beaten the truck go by truck instead, while WAIT
  simply leaves the ones it never finishes out of the ratio.
- In the windiest and coldest weeks flight time stays near 150 s and pad service grows from
  149 s to 200-237 s. The losses are pod wait and queue: shipments waiting through blocked
  hours for a pod and a drone.
- Most sensitive: the wind limits. Halving them takes the median week from 0.56 to 3.58 and
  the completed share from 87% to 72%; raising them by half helps little. The cold-derate
  depth comes next: half as deep gives a mean of 0.80 and 96% completed, half again as deep a
  mean of 7.12 and 76% completed. The gust term of the landing error and the traffic peak delays move the mean by
  0.03 or less. Most of the truck's routes on this network run on minor roads (section 5), and
  latch faults cost a retry, not a week.

Week alignment (spec A87): a week is 168 consecutive series hours from a local Monday midnight.
Traffic reads local time from the sim clock with the offset at the start of the week, so in
the two weeks a year with a daylight-saving change its peaks are an hour off after the change.

## 8. Completion-aware results (Phase 7)

The section 7 ratio averaged only finished orders. A week where most requests were turned
away at the backlog cap, or were still waiting at the end, could look no worse than one where
they were served. With `metrics.order_outcomes` (spec M11, schema 1.9.0) the sim now puts every
shipment requested in the window into one class at the horizon: delivered by drone, delivered
by truck fallback, rejected at the cap, blocked by weather, blocked by energy (a fully charged
drone fails the energy guard for that route this hour), in flight, waiting, or diverted/failed.
It reports two order-weighted ratios: over delivered orders, and with every unfinished order
counted at its elapsed time at the horizon (a lower bound on what it will take). The
completion rate sits beside both.

`uv run analysis/completion.py` reruns the three named weeks of section 7 on the 6b demo,
pooling seeds 1 to 3, and then the fallback timeout at three values:

Named weeks of the 6b demo, seeds 1 to 3 pooled (requests summed over seeds).

| Week | Rule | Policy | 6b ratio | Drone | Truck fallback | Rejected | Blocked weather | Blocked energy | In flight | Waiting | Diverted/failed | Completion | Ratio, delivered | Ratio, lower bound |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2023-10-23 | median wind | WAIT | 0.30 | 3603 | 0 | 178 | 129 | 0 | 0 | 0 | 60 | 90.8% | 0.30 | 0.90 |
| 2023-10-23 | median wind | TRUCK_FALLBACK | 0.48 | 3603 | 275 | 0 | 32 | 0 | 0 | 0 | 60 | 97.7% | 0.48 | 0.50 |
| 2024-01-08 | windiest | WAIT | 11.13 | 1586 | 0 | 2216 | 138 | 0 | 0 | 0 | 30 | 39.9% | 11.20 | 19.67 |
| 2024-01-08 | windiest | TRUCK_FALLBACK | 12.81 | 1087 | 1113 | 1708 | 30 | 0 | 0 | 0 | 32 | 55.4% | 12.78 | 12.63 |
| 2025-01-20 | coldest | WAIT | 40.72 | 733 | 0 | 3133 | 0 | 74 | 0 | 15 | 15 | 18.5% | 40.68 | 36.74 |
| 2025-01-20 | coldest | TRUCK_FALLBACK | 6.89 | 776 | 2064 | 1023 | 0 | 74 | 0 | 15 | 18 | 71.5% | 6.89 | 6.81 |

Truck-fallback timeout, TRUCK_FALLBACK.

| Week | Rule | Timeout (s) | Drone | Truck fallback | Rejected | Blocked weather | Blocked energy | In flight | Waiting | Diverted/failed | Completion | Ratio, delivered | Ratio, lower bound |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2024-01-08 | windiest | 1800 | 1612 | 1680 | 628 | 17 | 0 | 0 | 0 | 33 | 82.9% | 5.33 | 5.33 |
| 2024-01-08 | windiest | 3600 | 1087 | 1113 | 1708 | 30 | 0 | 0 | 0 | 32 | 55.4% | 12.78 | 12.63 |
| 2024-01-08 | windiest | 7200 | 1772 | 673 | 1356 | 133 | 0 | 0 | 0 | 36 | 61.6% | 10.80 | 11.33 |
| 2025-01-20 | coldest | 1800 | 772 | 2101 | 986 | 0 | 74 | 0 | 15 | 22 | 72.4% | 6.06 | 6.00 |
| 2025-01-20 | coldest | 3600 | 776 | 2064 | 1023 | 0 | 74 | 0 | 15 | 18 | 71.5% | 6.89 | 6.81 |
| 2025-01-20 | coldest | 7200 | 778 | 1972 | 1114 | 0 | 74 | 0 | 15 | 17 | 69.3% | 8.56 | 8.43 |

- The 6b ratio of the windiest and coldest weeks rested on 40% and 18% of requests. Most of
  the rest were rejected at the backlog cap (pads hold at most so many queued requests),
  not waiting. Even the median-wind week turned away 5% under WAIT.
- Counting unfinished orders shows how much WAIT hid: in the median-wind week the ratio goes
  from 0.30 to 0.90 once the 129 orders still blocked at the end are counted.
- In the coldest week every order still open at the end is energy-blocked or waiting, none
  weather-blocked: the cold has made the routes infeasible without crossing a weather limit, so
  the truck fallback (which acts only on weather blocks) cannot reach them (section 6).
- Timeout: 1800 s gives the best completion and ratio in both bad weeks (82.9% and 5.33 in the
  windiest, 72.4% and 6.06 in the coldest). The effect is not monotonic: at 7200 s the windiest
  week completes more than at 3600 s, because fewer trucked orders change when backlogs fill
  and requests are rejected.
