# Phase 6a: network realism and the charging bottleneck

This phase adds realism to the sim, then tests charger and layout options against the charging
bottleneck Phase 5 found at 1370 Wh. Every number is a placeholder (spec A01), and every new key
is flagged `assumption = true`. Each new behaviour is set by optional config keys (spec 1.6.0).
When a scenario leaves them out it runs exactly as in 1.5.0, so the committed scenarios and the
golden outputs are unchanged.

The scripts are in `analysis/` and run with `uv run analysis/<script>.py`. They build the
release sim, write variant scenarios into temporary directories, and put their results in
`analysis/out/`.

## 1. Battery mass

Until now `drone.empty_mass_kg` did not depend on `drone.battery_capacity_j`: the 1370 Wh
scenario carried 470 Wh more than the 900 Wh one at no extra mass.

Spec A68 adds the option of an airframe mass plus a battery specific energy. The empty mass is
then airframe + capacity / specific energy. The placeholders, 4.5 kg and 200 Wh/kg, give exactly
9.0 kg at 900 Wh. A test runs the A-B example in both forms and gets identical deliveries,
alerts and flight records (`sim/tests/battery_mass.rs`).

`sim battery` asks the core's own energy guard (`dispatch_ok`) what a lane needs. A coupled
battery is sized by iterating C = need(C) / 0.75 (charge to 0.95, reserve 0.20).
`analysis/battery.py` writes `analysis/out/battery.json`.

| Case (5 kg pod) | Value |
|---|---|
| Range with the fitted profile at 900 Wh | 5662 m, unchanged (9.0 kg either way) |
| Longest legal lane | hub to Edgar Thomson, 12 896 m |
| Battery it needs, mass fixed (Phase 5) | 1367.1 Wh |
| Battery it needs, mass coupled | none: no fixed point |
| Largest battery within MTOW (spec R6) | 1180 Wh (10.4 kg empty), range 8137 m |

No coupled battery serves the longest lane. Each extra kilogram of battery raises the energy
the guard needs, because power grows as mass^1.5. The iteration gets closest at 3893 Wh (a
24 kg drone), still 46 Wh short, and then diverges. MTOW cuts this off much earlier. With a
16 kg MTOW, a 0.6 kg latch and a 5 kg pod, the drone can add only 1.4 kg of battery to the
900 Wh drone. That is 900 + 1.4 x e Wh for a specific energy of e Wh/kg. Even uncoupled, the
lane needs 1367 Wh, so e would have to be at least 334 Wh/kg before the lane could fit.

So no coupled scenario is added: at any capacity that serves the lane, the scenario fails R6
or R12 at load. The 1370 Wh scenario stays as it was (A66), now noted as 2.35 kg too light
under A68. The charging sweep of section 7 uses it, because the bottleneck there comes from
the charger and the OUT spot, not from mass.

## 2. Truck fairness

Both clocks start at the shipment request (spec 7):

- **Truck:** waits `truck.dispatch_delay_s` (900 s) for a truck, loads for `truck.load_s`
  (300 s), drives, and unloads for `truck.unload_s` (300 s).
- **Drone:** its time covers the pod wait, including the shipper placing the pod
  (`pad_ops.pod_place_s`, 60 s), the queue, pad service (latch, charging, takeoff, approach,
  release) and flight. It ends at the verified release on IN.

So the truck already carries its loading and dispatch dwell, about 1500 s of fixed handling
against about 80 s for the drone. No truck dwell key is added. The one asymmetry left favours
the truck: it has an unlimited fleet (A21), so it never queues.

M2 averages the truck over every closed order and the drone over delivered ones. The table
below instead compares the two on the same orders: delivered shipments requested in the
metrics window, on the six flows the fitted 900 Wh network serves (the two Strip District
warehouses to the three Oakland hospitals). Seeds 1 to 5; values are means over seeds, with
the standard deviation. Flow rates differ between scenarios, because the pipeline splits each
warehouse's demand over the pads it can reach (`analysis/truck_fairness.py`,
`analysis/out/truck_fairness.json`).

| Scenario | Orders | Drone (s) | Truck (s) | Speedup | Pod wait / queue / pad service / flight (s) |
|---|---|---|---|---|---|
| Placeholder profile | 768 | 933 ± 65 | 1940 ± 5 | 2.09 | 422 / 120 / 241 / 150 |
| Fitted, 900 Wh | 1250 | 1659 ± 863 | 1939 ± 4 | 1.36 | 1033 / 161 / 314 / 152 |
| Fitted, 1370 Wh | 712 | 9436 ± 1492 | 1941 ± 7 | 0.21 | 8642 / 129 / 515 / 150 |

On the same short Oakland flows, flight takes about 150 s whichever profile flies it. The
differences between scenarios are all on the ground. The fitted 900 Wh network puts all of its
demand on these six flows with five drones, so pod wait grows and varies a lot from seed to seed.
At 1370 Wh, pod wait reaches 2.4 hours (section 7). The truck takes 1940 s in every scenario,
because its time does not depend on the drone network.

## 3. Repair after a release fault

A release fault that outlasts its retries (V20) grounds the drone on IN with its pod and takes
the pad out of service. Before this phase that was for the rest of the run. With
`pad_ops.repair_mean_s` and `pad_ops.repair_spread_s` (A69), the pad is repaired after a time
drawn uniformly from mean ± spread (a new REPAIR random stream, so no other draw moves). At
`REPAIR_DONE`:

- the stuck pod goes into the pad's inventory (P09); its order stays FAILED;
- the pad comes back ONLINE (M06), unless a scheduled out-of-service window is still open;
- the drone is back ON_PAD (D25) and reports its landing again. The PC adopts it (V42) and it
  carries on as an empty drone on IN.

The RELEASE_FAULT and PAD_OUT_OF_SERVICE alerts are raised as before. The test
`sim/tests/repair.rs` forces the fault and checks this sequence, the repair time, and that B
takes deliveries again.

Placeholder Pittsburgh week at 1x demand, repair mean 2 h, spread 1 h (`analysis/repair.py`):

| Release fault | Seed 42: rejected | Seeds 1-5: rejected fraction |
|---|---|---|
| Pad out for good (Phase 3) | 187 of 1373 (13.6%) | 0.0% |
| Repaired | 0 of 1373 (0.0%) | 0.0% |

All 187 rejections at seed 42 came from that one grounding, and with a repair there are none.
None of the other seeds rejects anything either way. Seed 1 has a release fault, and repair
lowers its mean door-to-door from 1316 s to 1220 s.
