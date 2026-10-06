# Phase 2 results

Conditions: commit `4d81103`, release build, seed 42, replication 0, each scenario's own
`configs/<name>/scenario.toml`. Every input number is a placeholder (`assumption = true`), so
these numbers show how the model behaves, not what a real network would do. Times are
door-to-door from the shipment request; the truck serves the same orders over the
synthetic road matrix (straight line x 1.4 at 11 m/s, slowed by the time-of-day factors).

| Scenario | Pads | Drones | Window | Deliveries/h | Drone mean / p95 (s) | Truck mean / p95 (s) | Speedup | Drone cost/delivery (cu) | Truck cost/delivery (cu) | Break-even (deliveries per drone-h) | Actual (per drone-h) | Abort rate | Stockout |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ab_synthetic | 2 | 2 | 7.5 h | 5.5 | 3738 / 9620 | 2462 / 2680 | 0.66 | 2.66 | 25.47 | 0.28 | 2.73 | 0.026 | 0.34 |
| hub_spoke_6 | 6 | 8 | 7.5 h | 14.9 | 2892 / 7200 | 2327 / 2990 | 0.80 | 3.89 | 22.90 | 0.31 | 1.87 | 0.069 | 0.24 |
| mesh_small | 8 | 8 | 7.5 h | 16.0 | 693 / 1080 | 1870 / 2210 | 2.70 | 3.62 | 14.25 | 0.51 | 2.00 | 0.016 | 0.016 |
| scale_200 | 40 | 200 | 167 h | 70.4 | 752 / 1340 | 2042 / 2520 | 2.72 | 20.49 | 17.71 | 0.41 | 0.35 | 0.078 | 0.014 |

Speedup is mean truck time over mean drone time. Break-even is the drone utilisation at
which the drone's cost per delivery equals the truck's (spec 7, M8); above it the drone is
cheaper.

What the numbers say:

- Where pods are plentiful (mesh, scale) the drone is about 2.7 times faster than the truck,
  door to door. Where they are not (A-B, hub-and-spoke) it is slower on average, because
  shippers wait for an empty pod: 34% and 24% of requests found none.
- A-B is pod-starved by design of the example: demand runs 4/h one way and 3/h back with three
  pods per pad and a rebalancing floor of one pod.
- The hub-and-spoke starves the spokes. Spokes send most pods to the hub (`hub_weight` 3), and
  returns have to queue through the hub's single OUT spot behind its own shipments. One OUT
  spot per pad caps a pad at roughly 4.5 launches an hour with these timings (placement,
  pickup flight, latch, charge dwell on OUT); a hub hits that first.
- The scale fleet is cheaper per delivery than the truck only above 0.41 deliveries per
  drone-hour; at 2 requests per pad-hour 200 drones manage 0.35, so the truck wins on cost
  while the drone wins on time. Fewer drones or more demand would cross over.
- Raising `policy.pod_min_per_pad` from 1 to 2 on the scale case cut stockouts from 13% to 1.4%
  and the mean door-to-door from 4237 s to 752 s. Pod circulation, not drone count, was the
  limit.

## Performance

Scale case (200 drones, 40 pads, 7 simulated days), same commit, on an Intel i9-13900K
under WSL2, measured with `/usr/bin/time -v` by `scripts/verify.sh`:

| Measure | Value | Budget (spec A39) |
|---|---|---|
| Events processed | 1 428 035 | |
| Wall time | 0.69 s | 60 s |
| Peak RSS | 8 MB | 200 MB |
| `events.bin` on disk | 112 MB | streamed, never held in memory |

At the overloaded demand the scale case was first written with (6 requests per pad-hour), the
same run processed 2.37 million events in 1.1 s at 10 MB.
