# Paper run: hand-walking the phase-1 spec

Two walks through `docs/spec.md` with `configs/ab_synthetic/scenario.toml`. Every step names
its owner and the message or event involved, and cites the transition it uses. Times are sim
seconds with millisecond resolution; latencies are 0.050 s on drone-pad links and 0.200 s on
links touching the fleet manager. Numbers come from the placeholder config, so they check the
arithmetic of the spec, not the real world.

Shorthands: PC_A / PC_B = pad controllers of A (pad 0) and B (pad 1); D0, D1 = drones 0 and
1; FM = fleet manager. Lane 0 is A→B, lane 1 is B→A, both 6000.133 m.

Derived values used below (spec 2.7, 3.7, 6):

- Hover power: 639.821 W empty, 1200.000 W with a 5 kg pod. Lane: 400.009 s; 400 009 J with
  the pod, 213 279 J empty.
- T_cycle = 3 x 45 + 2 x 40 = 215 s. T_land_timeout = 217 s. T_contingency = 2 x 215 + 180 =
  610 s.
- `dispatch_ok` A→B with the pod needs 1 826 018 J = 0.5636 of capacity above the reserve, so
  soc >= 0.7636. Rule R12 passes from `drone.charge_target_soc` = 0.95.
- `divert_ok` B→A with the pod needs 658 009 J = 0.2031 above the reserve.

## Initial state (t = 0)

D0 is parked on OUT at A and D1 on OUT at B (`drone.initial_pads`), both at soc 1.0, each with
a visit in READY at its pad. Pods 0 to 2 are at A, 3 to 5 at B, all empty, all KEEP_LOCAL.
No orders exist.

## (a) Normal A-B delivery

A shipment request for B arrives at A at t = 2000.000 (after warmup, so it counts in the
metrics). The draw is shown as a fixed time to keep the walk readable.

| # | t (s) | Owner | Message / event | From → To | Transition | Result |
|---|---|---|---|---|---|---|
| 1 | 2000.000 | DG | `SHIPMENT_REQUEST` (A to B, deadline 5600.000) | DG → PC_A | P01 | Backlog 1. empty_pods 3, so no stockout. OUT is blocked by D0 (READY, no pickup). |
| 2 | 2000.000 | PC_A | `SHIPMENT_PENDING` (out_drone D0) | PC_A → FM | V39 | Arrives 2000.200. |
| 3 | 2000.200 | FM | `MISSION_ASSIGN` (YIELD_LOCAL, dest A) | FM → D0 | 6.4 yield_for, cause placement | Arrives 2000.400. |
| 4 | 2000.400 | D0 | `DEPART_REQUEST` (dest A, YIELD_LOCAL) | D0 → PC_A | D19 | D_RESPONSE_TIMEOUT armed. |
| 5 | 2000.450 | PC_A | `DEPART_CLEAR` | PC_A → D0 | V34 READY → DEPARTING | No other movement at A. Timer 32 s. |
| 6 | 2000.500 | D0 | `DEPART_CLEAR` received; `TAKEOFF_DONE` scheduled at 2030.500 | D0 | D01 PARKED → TAKEOFF | |
| 7 | 2030.500 | D0 | `TAKEOFF_DONE`; sends `DEPARTED` then `LANDING_REQUEST` (no pod, soc 0.99408) | D0 → PC_A | D20 TAKEOFF → HOLDING | Hovering over A. D_ENERGY_RESERVE armed. |
| 8 | 2030.550 | PC_A | `DEPARTED`; sends `PAD_STATUS` | PC_A → FM | V36 DEPARTING → CLOSED (COMPLETED); P02 | OUT drone-free, so placement of pod 0 starts. empty_pods 3 → 2. `POD_PLACE_DONE` at 2090.550. |
| 9 | 2030.550 | PC_A | `LANDING_REQUEST`; sends `LANDING_GRANT` (IN) | PC_A → D0 | V01 → APPROACH | 3.5 rule 2: OUT is placing, IN is free, so IN. PC_LANDING_TIMEOUT at 2247.550. Same-ms ordering: `DEPARTED` was sent first, so it has the lower seq. |
| 10 | 2030.600 | D0 | `LANDING_GRANT`; `APPROACH_DONE` scheduled at 2075.600 | D0 | D04 HOLDING → APPROACH | |
| 11 | 2075.600 | D0 | `APPROACH_DONE` (MARKER draw: detected); sends `LANDED` (IN, no pod, soc 0.98517) and `DRONE_STATUS` (ON_PAD) | D0 → PC_A, FM | D05 APPROACH → ON_PAD | |
| 12 | 2075.650 | PC_A | `LANDED` | D0 → PC_A | V07 APPROACH → WAIT_OUT | PC_OUT_WAIT_TIMEOUT at 2195.650. V21 false: OUT is placing. |
| 13 | 2090.550 | PC_A | `POD_PLACE_DONE`; sends `POD_PLACED` (pod 0, SHIPMENT, dest B, requested 2000.000, deadline 5600.000, 5.0 kg) | PC_A → FM | P03 | out_pod 0. V21 still false: no pickup assigned yet. |
| 14 | 2090.750 | FM | `POD_PLACED`; sends `PICKUP_ASSIGNED` (pod 0, order 0, D0) | FM → PC_A | 6.1 MIN_ETA | Order 0 created, then ASSIGNED. D0 is on the origin pad (ETA 0) and passes `dispatch_ok` (0.98517 >= 0.7636). Truck baseline: departs 3200.000, hour 7, factor 0.6, 1272.755 s, so `truck_delivered_t_ms` = 4772.755. |
| 15 | 2090.950 | PC_A | `PICKUP_ASSIGNED`; sends `HOP_CMD` | PC_A → D0 | P07, V21 WAIT_OUT → HOPPING | OUT reserved. Timer 22 s. |
| 16 | 2091.000 | D0 | `HOP_CMD`; `HOP_DONE` scheduled at 2111.000 | D0 | D15 ON_PAD → HOPPING | |
| 17 | 2111.000 | D0 | `HOP_DONE`; sends `LANDED` (OUT, soc 0.98122) | D0 → PC_A | D15 HOPPING → ON_PAD | |
| 18 | 2111.050 | PC_A | `LANDED`; sends `LATCH_CMD` (pod 0, attempt 1) | PC_A → D0 | V24 HOPPING → LATCHING | IN is empty with no pod, so P04 does nothing. Timer 10 s. |
| 19 | 2111.100 | D0 | `LATCH_CMD`; `LATCH_DONE` scheduled at 2119.100 | D0 | D15 | AUTO: `pad_ops.latch_s` 8 s. |
| 20 | 2119.100 | D0 | `LATCH_DONE` (LATCH draw: locked); sends `LATCH_REPORT` (locked) | D0 → PC_A | D15 | Pod 0 on D0, payload 5.0 kg. |
| 21 | 2119.150 | PC_A | `LATCH_REPORT` | D0 → PC_A | V28 LATCHING → VERIFY_LOCK | Verify done at 2122.150. |
| 22 | 2122.150 | PC_A | `TIMER` (PC_VERIFY_LOCK_DONE); sends `POD_LATCHED` and `SERVICE_COMPLETE` (NORMAL, OUT, pod 0, soc 0.98122) | PC_A → FM, D0 | V32 VERIFY_LOCK → READY | soc >= 0.95, so no charge. |
| 23 | 2122.200 | D0 | `SERVICE_COMPLETE`; sends `MISSION_REQUEST` (NORMAL) | D0 → FM | D17 | |
| 24 | 2122.350 | FM | `POD_LATCHED` | PC_A → FM | 2.1 | Order 0 LATCHED, `latched_t_ms` = 2122.150 (`sent_t_ms`). |
| 25 | 2122.400 | FM | `MISSION_REQUEST`; sends `MISSION_ASSIGN` (DELIVER, B, order 0) to D0 and `MISSION_ASSIGN` (REPOSITION, A) to D1 | FM → D0, D1 | 6.2 IMMEDIATE; 6.4 yield_for D1, cause inbound | D0 now claims B. The only pad unclaimed by D0 is A, so D1 swaps to A. |
| 26 | 2122.600 | D0, D1 | `DEPART_REQUEST` from each | D0 → PC_A, D1 → PC_B | D19 | |
| 27 | 2122.650 | PC_A, PC_B | `DEPART_CLEAR` to each | PC → drone | V34 | |
| 28 | 2152.700 | D0 | `TAKEOFF_DONE`; sends `DEPARTED` and `DRONE_STATUS` (EN_ROUTE, dest B) | D0 → PC_A, FM | D02 TAKEOFF → EN_ROUTE | soc 0.97011. Lane 0, `LANE_DONE` at 2552.709. |
| 29 | 2152.750 | PC_A | `DEPARTED` | D0 → PC_A | V36 → CLOSED (COMPLETED) | A is empty. |
| 30 | 2152.900 | FM | `DRONE_STATUS` | D0 → FM | 2.1 | Order 0 IN_TRANSIT, `departed_t_ms` = 2152.700. |
| 31 | 2152.700 to 2671.886 | D1 | `TAKEOFF_DONE`, `LANE_DONE`, `LANDING_REQUEST`, `LANDING_GRANT` (OUT), `APPROACH_DONE`, `LANDED`, `CHARGE_START`, `SERVICE_COMPLETE`, `MISSION_REQUEST`, `MISSION_ASSIGN` (PARK), `DRONE_STATUS` (PARKED) | D1 ↔ PC_A, FM | D02, D03, D04, D05; V01, V09, V33; D17, D18 | D1 lands straight on OUT at A (empty drone, OUT drone-free, 3.5 rule 2), charges 73.577 s from 0.91934, parks. |
| 32 | 2552.709 | D0 | `LANE_DONE`; sends `LANDING_REQUEST` (pod 0, soc 0.84665) | D0 → PC_B | D03 EN_ROUTE → HOLDING | FlightLogRecord: lane 0, 5.0 kg, 400 009 J, soc 0.97011 → 0.84665. |
| 33 | 2552.759 | PC_B | `LANDING_REQUEST`; sends `LANDING_GRANT` (IN) | PC_B → D0 | V01 → APPROACH | IN free (D1 left B at 2152.700). |
| 34 | 2552.809 | D0 | `LANDING_GRANT`; `APPROACH_DONE` at 2597.809 | D0 | D04 | |
| 35 | 2597.809 | D0 | `APPROACH_DONE` (detected); sends `LANDED` (IN, pod 0, soc 0.82995) and `DRONE_STATUS` | D0 → PC_B, FM | D05 | |
| 36 | 2597.859 | PC_B | `LANDED`; sends `RELEASE_CMD` (attempt 1) | PC_B → D0 | V06 APPROACH → RELEASING | Timer 7 s. |
| 37 | 2597.909 | D0 | `RELEASE_CMD`; `RELEASE_DONE` at 2602.909 | D0 | D15 | |
| 38 | 2602.909 | D0 | `RELEASE_DONE` (RELEASE draw: open); sends `RELEASE_REPORT` (latch_open) | D0 → PC_B | D15 | |
| 39 | 2602.959 | PC_B | `RELEASE_REPORT` | D0 → PC_B | V15 RELEASING → VERIFY_RELEASE | Verify done at 2605.959. |
| 40 | 2605.959 | PC_B | `TIMER` (PC_VERIFY_RELEASE_DONE); sends `POD_DELIVERED` (pod 0, D0) | PC_B → FM | V18 VERIFY_RELEASE → WAIT_OUT | Delivery measurement point. Pod 0 on IN. Out-wait timer at 2725.959. OUT drone-free, no pod, policy WAIT: V21 false. |
| 41 | 2606.159 | FM | `POD_DELIVERED` | PC_B → FM | 2.1 | Order 0 DELIVERED at B. Door-to-door 605.959 s; truck 2772.755 s. |
| 42 | 2725.959 | PC_B | `TIMER` (PC_OUT_WAIT_TIMEOUT); sends `HOP_CMD` | PC_B → D0 | V21 WAIT_OUT → HOPPING | No pod placed at B in the wait. |
| 43 | 2726.009 | D0 | `HOP_CMD`; `HOP_DONE` at 2746.009 | D0 | D15 | |
| 44 | 2746.009 | D0 | `HOP_DONE`; sends `LANDED` (OUT, soc 0.82600) | D0 → PC_B | D15 | |
| 45 | 2746.059 | PC_B | `LANDED`; sends `CHARGE_START` (dwell 297.609 s) | PC_B → D0 | V25 HOPPING → CHARGING; P04 | D0 left IN with pod 0 on it, so `POD_REMOVE_DONE` at 3346.059. |
| 46 | 3043.668 | PC_B | `TIMER` (PC_CHARGE_DONE); sends `SERVICE_COMPLETE` (NO_POD, OUT, soc 0.95) | PC_B → D0 | V33 CHARGING → READY | 401 772 J added: D0 battery cycles +0.124. |
| 47 | 3043.718 | D0 | `SERVICE_COMPLETE`; sends `MISSION_REQUEST` (NO_POD) | D0 → FM | D17 | |
| 48 | 3043.918 | FM | `MISSION_REQUEST`; sends `MISSION_ASSIGN` (PARK) | FM → D0 | 6.2 returns none; 6.4 place_idle | D0 is on OUT and no other drone claims B. |
| 49 | 3044.118 | D0 | `MISSION_ASSIGN`; sends `DRONE_STATUS` (PARKED) | D0 → FM | D18 ON_PAD → PARKED | PC_B keeps the visit in READY. |
| 50 | 3346.059 | PC_B | `POD_REMOVE_DONE`; sends `POD_REMOVED` (pod 0) | PC_B → FM | P05 | Pod 0 empty in B's inventory: B 4, A 2. |
| 51 | 3346.259 | FM | `POD_REMOVED` | PC_B → FM | 6.5 MIN_THRESHOLD | No pad below `policy.pod_min_per_pad` = 1. No return. |

Metric contributions: M1 one delivery. M2 drone 605.959 s against truck 2772.755 s. M4
placement_wait 90.550 s, pickup_wait 31.600 s, landing_hold 0. M5 D0 +0.124 cycles. M6 no
stockout. M7 no abort, 0 go-arounds in 3 approaches.

## (b) Marker miss, hold, divert

The example config schedules a MARKER_NOT_DETECTED window at B from 7200 to 9000 s. D0 has
latched order k (pod p, 5 kg) at A, charged to 0.95, and completed takeoff from A at 7399.991.
D1 is parked on OUT at A after an earlier swap.

| # | t (s) | Owner | Message / event | From → To | Transition | Result |
|---|---|---|---|---|---|---|
| 1 | 7200.000 | FI | `FAULT_START` (MARKER_NOT_DETECTED, B) | FI → PC_B, drones | 3.6 | Marker detection at B is forced to fail until `FAULT_END` at 9000.000. PC_B stays ONLINE. FM is not told (A35). |
| 2 | 7800.000 | D0 | `LANE_DONE`; sends `LANDING_REQUEST` (pod p, soc 0.81543) | D0 → PC_B | D03 EN_ROUTE → HOLDING | D_ENERGY_RESERVE at 9461.659. |
| 3 | 7800.050 | PC_B | `LANDING_REQUEST`; sends `LANDING_GRANT` (IN) | PC_B → D0 | V01 → APPROACH | PC_LANDING_TIMEOUT at 8017.050. |
| 4 | 7800.100 | D0 | `LANDING_GRANT`; `APPROACH_DONE` at 7845.100 | D0 | D04 | |
| 5 | 7845.100 | D0 | `APPROACH_DONE` (MARKER draw consumed, forced miss); `GO_AROUND_DONE` at 7885.100 | D0 | D05 APPROACH → GO_AROUND | go_arounds 1. |
| 6 | 7885.100 | D0 | `GO_AROUND_DONE`; `APPROACH_DONE` at 7930.100 | D0 | D06 | |
| 7 | 7930.100 | D0 | `APPROACH_DONE` (miss); `GO_AROUND_DONE` at 7970.100 | D0 | D05 | go_arounds 2 = `pad_ops.max_go_arounds`. |
| 8 | 7970.100 | D0 | `GO_AROUND_DONE`; `APPROACH_DONE` at 8015.100 | D0 | D06 | |
| 9 | 8015.100 | D0 | `APPROACH_DONE` (miss); sends `APPROACH_ABORT` (MARKER_NOT_DETECTED) and `HOLD_REPORT` (MARKER_NOT_DETECTED, hold_cycles 1, soc 0.73576) | D0 → PC_B, FM | D05 APPROACH → HOLDING | 215 s of hover with the pod: 258 000 J. |
| 10 | 8015.150 | PC_B | `APPROACH_ABORT` | D0 → PC_B | V12 APPROACH → CLOSED (APPROACH_ABORTED) | Arrives 1.900 s before PC_LANDING_TIMEOUT; IN reservation freed. |
| 11 | 8015.300 | FM | `HOLD_REPORT`; sends `HOLD_CONTINUE` (retry_after 60 s) | FM → D0 | 6.3 BOUNDED_HOLD | hold_cycles 1 <= 1. After another 60 + 215 s of hover the divert to A would still leave 0.43082 >= 0.2. CONTINUE. |
| 12 | 8015.500 | D0 | `HOLD_CONTINUE`; `TIMER` (D_HOLD_RETRY) at 8075.500 | D0 | D21 | |
| 13 | 8075.500 | D0 | `TIMER` (D_HOLD_RETRY); sends `LANDING_REQUEST` | D0 → PC_B | D08 | |
| 14 | 8075.550 | PC_B | `LANDING_REQUEST`; sends `LANDING_GRANT` (IN) | PC_B → D0 | V01 → APPROACH (new visit) | |
| 15 | 8075.600 to 8245.600 | D0 | `APPROACH_DONE` (miss) at 8120.600 and 8205.600, `GO_AROUND_DONE` at 8160.600 and 8245.600 | D0 | D04, D05, D06 | Same cycle as steps 4 to 8; soc 0.71336 at grant receipt. |
| 16 | 8290.600 | D0 | `APPROACH_DONE` (miss); sends `APPROACH_ABORT` and `HOLD_REPORT` (MARKER_NOT_DETECTED, hold_cycles 2, soc 0.63373) | D0 → PC_B, FM | D05 APPROACH → HOLDING | |
| 17 | 8290.650 | PC_B | `APPROACH_ABORT` | D0 → PC_B | V12 → CLOSED (APPROACH_ABORTED) | |
| 18 | 8290.800 | FM | `HOLD_REPORT`; sends `MISSION_ASSIGN` (DIVERT, A, order k) to D0 and `MISSION_ASSIGN` (REPOSITION, B) to D1; writes `ALERT` DIVERTED (WARNING) | FM → D0, D1, OPS | 6.3 DIVERT (hold_cycles 2 > 1); 6.6 NEAREST_FEASIBLE; 6.4 yield_for D1, cause inbound | alt(B) = A; `divert_ok` leaves 0.43064 >= 0.2. D0 now claims A, so D1 must yield and the only unclaimed pad is B. |
| 19 | 8291.000 | D0 | `MISSION_ASSIGN`; sends `LANDING_CANCEL` and `DRONE_STATUS` (EN_ROUTE, dest A) | D0 → PC_B, FM | D12 HOLDING → EN_ROUTE | soc 0.63358. Lane 1, `LANE_DONE` at 8691.009. |
| 20 | 8291.050 | PC_B | `LANDING_CANCEL` | D0 → PC_B | 3.4 note: no open visit, ignored | |
| 21 | 8291.000 to 8321.150 | D1, PC_A | `DEPART_REQUEST`, `DEPART_CLEAR`, `TAKEOFF_DONE`, `DEPARTED` | D1 ↔ PC_A | D19, V34, D01, D02, V36 | OUT at A is drone-free from 8321.150. D1 flies empty to B and will meet the same marker fault (finding 3 below); its handling is this same branch and is not traced further. |
| 22 | 8691.009 | D0 | `LANE_DONE`; sends `LANDING_REQUEST` (pod p, soc 0.51012) | D0 → PC_A | D03 → HOLDING | |
| 23 | 8691.059 | PC_A | `LANDING_REQUEST`; sends `LANDING_GRANT` (IN) | PC_A → D0 | V01 → APPROACH | IN free. |
| 24 | 8736.109 | D0 | `APPROACH_DONE` (MARKER draw at A: detected); sends `LANDED` (IN, pod p, soc 0.49341) and `DRONE_STATUS` | D0 → PC_A, FM | D05 | |
| 25 | 8736.159 | PC_A | `LANDED`; sends `RELEASE_CMD` (attempt 1) | PC_A → D0 | V06 → RELEASING | |
| 26 | 8741.209 | D0 | `RELEASE_DONE` (open); sends `RELEASE_REPORT` | D0 → PC_A | D15 | |
| 27 | 8744.259 | PC_A | `TIMER` (PC_VERIFY_RELEASE_DONE); sends `POD_DELIVERED` (pod p, D0) | PC_A → FM | V15 at 8741.259, then V18 → WAIT_OUT | |
| 28 | 8744.459 | FM | `POD_DELIVERED` | PC_A → FM | 2.1, 6.6 | Released at A, not at `dest_pad` B: order k DIVERTED, `delivered_pad` A. Counts in M7 abort_rate. |
| 29 | 8744.259 onward | PC_A, D0, FM | `HOP_CMD`, `LANDED`, `CHARGE_START`, `SERVICE_COMPLETE`, `MISSION_REQUEST`, `MISSION_ASSIGN` (PARK), `POD_REMOVE_DONE`, `POD_REMOVED` | PC_A ↔ D0, FM | V21 (after out-wait), V25, V33, D17, D18, P04, P05 | Same as (a) steps 42 to 51 at A. Pod p becomes an empty pod in A's inventory. |
| 30 | 9000.000 | FI | `FAULT_END` (MARKER_NOT_DETECTED, B) | FI | 3.6 | Marker detection at B back to `pad_ops.p_marker_detect`. |

Metric contributions: M7 one DIVERTED order, 4 go-arounds in D0's 7 approaches, 1 divert. M4
landing_hold 0 (D0 was never HELD; marker holds are airborne, not queue time).

## Findings and fixes made before freeze

Walking these two cases found four gaps. All are fixed in `docs/spec.md` and the schemas; the
walk above is the re-walk against the fixed spec, and no step is left without an owner or a
message.

1. PC timers carried only the pad, not which visit they belong to. Fixed: `TIMER` gained
   `subject_id` (the drone of the visit).
2. A pickup assigned to a drone already in flight (6.1 case b) had no drone-side rule.
   Fixed: D22 updates `pickup_order_id` without changing the route.
3. The default yield in step 18 of (b) sends D1 into the same marker fault at B, because FM
   cannot see marker faults. Kept as a documented limitation (A35) since FM learning from
   marker aborts would be a new feature; the energy guard keeps it safe.
4. Using the full dispatch guard at the moment of diversion would have refused the divert in
   (b) step 18 (soc 0.634 against 0.764 needed) and grounded the drone. Fixed: diversion uses
   the separate `divert_ok`, and the dispatch guard reserves for it in advance.
