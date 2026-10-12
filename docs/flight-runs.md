# Phase 4 flight runs: one lane in PX4 SITL

These are calibration data only. Nothing here is fitted to the sim's flight model; Phase 5 does
that. Every vehicle and pad number is a placeholder (spec A01, A54 to A61).

## Conditions

| Item | Value |
|---|---|
| Code | commit `d28dcef` (runner); validator as of `41b3f74` |
| Host | WSL2, Ubuntu 22.04, i9-13900K, headless (Gazebo server only) |
| PX4 | v1.17.0 (`d6f12ad1`), SITL, airframe 4001 (x500), parameters in `px4/params/sitl.params`. The build in `~/PX4-Autopilot` already carries quad-autonomy-sim's two patches (extra uXRCE-DDS topics, gz_bridge deferring to PX4's magnetometer simulator); this phase relies on neither and changes nothing in that tree |
| Gazebo | Harmonic, gz-sim 8.15.0, physics step 4 ms |
| Height source | GPS, PX4's default. Phase 5 switched SITL to the barometer after a 12.9 km lane showed GPS height drifting from the flat world (spec A67); over this 2.1 km lane the two differ by 0.34 m at B |
| ROS 2 | Humble; px4_msgs `86d8239`; Micro XRCE-DDS agent v2.4.3; ros_gz 0.244.12 |
| Detector | OpenCV 4.5.4 ArUco 4x4, 320x240 at 10 Hz, 1.2 rad field of view |
| Lane | 3 of the Pittsburgh network: Strip District hub (Consumer Fresh Produce) to VA Pittsburgh Healthcare, 2093.4 m; flown in local ENU at the hub over flat ground |
| Commanded cruise | 15 m/s (scenario `drone.cruise_speed_mps`) at 60.96 m (the pipeline's 200 ft placeholder; the sim has no altitude) |
| Vehicle | x500 body at 9.0 kg plus a 0.6 kg latch; motors and inertia scaled 9.18x for thrust-to-weight 2.0 at the 16 kg MTOW |
| Visit | take off from A's OUT with a pod; cruise; land on B's IN; release; hop 4 m to OUT; land; latch the pod waiting there; park |
| Seed | sets the start offset (0.10 m standard deviation) and detector corner noise (0.3 px); Gazebo and PX4 are not seeded, so runs are not bit-reproducible |

Commands: `px4/tools/run_batch.sh accept` runs all 15 visits into `px4/runs/accept/`, which is
git-ignored. It validates every run and writes `summary.json` and `summary.md`.
`px4/samples/repeat_1/` keeps one run's logs.

## Validation

All 15 runs completed the visit and pass `px4/tools/flightlog.py validate`:

| Check | Requirement |
|---|---|
| Timestamps | monotonic events and samples |
| Drone and pad state sequences | equal the spec's normal visit (`px4/tools/expected.py`) |
| 23 handshake messages | in spec order |
| Sequence numbers | gap-free per sender |
| States | every state has a duration |
| Landings | both landings recorded |
| Lane record | agrees with its D02 and D03 transitions |

The 2x-speed run first failed the order check. DDS had delivered two messages the pad sent in
one sim tick in the opposite order. The check now uses send order, which is the spec's order.

## Per payload (seed 100)

Phase times come from the drone's spec transitions:

- Takeoff: D01 to D02, climbing to cruise height.
- Cruise: the lane, D02 to D03.
- Approach + landing: D04 to D05. This covers the descent from cruise height to 6 m, the wait
  for the marker lock, and precision landing.
- Hop: lifting off IN to touchdown on OUT.

B granted IN at once in every run, so holding time was 0.

| Payload (kg) | Airborne total (s) | Takeoff (s) | Cruise (s) | Approach + landing at IN (s) | Hop (s) | Energy proxy, visit (kJ) | Energy proxy, lane (kJ) | Mean cruise power (W) | Landing error IN / OUT (cm) |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 243.8 | 25.9 | 148.4 | 46.6 | 22.9 | 262 | 164 | 1106 | 3.6 / 2.9 |
| 2 | 245.2 | 26.4 | 146.8 | 49.0 | 23.0 | 345 | 217 | 1479 | 7.1 / 9.9 |
| 5 | 245.0 | 26.8 | 146.5 | 48.9 | 22.9 | 481 | 309 | 2110 | 2.5 / 6.5 |
| 10 | 246.2 | 27.3 | 146.5 | 49.0 | 23.4 | 744 | 486 | 3316 | 4.3 / 9.1 |

For reference, the scenario's timing placeholders are takeoff 30 s, approach 45 s and hop 20 s,
and the lane at 15 m/s is 139.6 s.

- **Times barely change with payload.** The flight is commanded by position and speed, and
  thrust-to-weight stays at least 1.6 even with the 10 kg pod, which is above the scenario's
  6.4 kg payload limit.
- **Energy does change.** Mean cruise power goes from 1106 W empty to 3316 W at 10 kg, a factor
  of 3.0 for a total mass ratio of 2.04 (2.04^1.5 = 2.9).

### Energy proxy

SITL has no power model, so the energy is a proxy (spec A57). Each rotor's thrust is
T = k_f ω², with ω the speed PX4 commands (logged at `/dd_quad/command/motor_speed`) and k_f the
vehicle's motor constant. Its ideal induced power from momentum theory is
P = T^1.5 / sqrt(2 ρ A), with ρ = 1.225 kg/m³ and A the disc of the x500's 0.1396 m propeller.
The four rotors are summed and integrated over time.

It leaves out profile power and motor and ESC losses, and the scaled vehicle keeps the x500's
small propellers. So its absolute value means nothing; its ratios between payloads and phases
are the useful part.

The lane record in `flight_log.bin` integrates each command until the next. The table
integrates the 20 Hz samples, which gives about 1.6% less (309 kJ against 314 kJ at 5 kg).

PX4's simulated battery drains with time, not load, and stops at 50%. The `soc` fields are
logged but say nothing about energy.

## Ten repeats at 5 kg (seeds 1 to 10)

| Measure | IN | OUT |
|---|---|---|
| Landing error mean (cm) | 4.1 | 9.5 |
| Landing error p95 (cm) | 7.6 | 13.5 |
| Landing error max (cm) | 9.0 | 13.6 |

Landing error is the horizontal distance from the vehicle's ground-truth position at touchdown
to the spot centre. The markers sit 1.0 m beside their spots (A56), so the spot centre is the
marker centre plus that fixed offset.

| Measure | Value |
|---|---|
| Landing success | 20 of 20 (marker found and touchdown within the 15 cm latch capture radius; 95% Wilson interval 0.84 to 1.00) |
| Visits completed and valid | 10 of 10 |
| Phase times | takeoff 26.6 to 27.0 s, cruise 146.5 s every run, approach + landing 48.9 to 49.2 s, hop 21.5 to 23.3 s |
| Energy proxy per visit | 480.0 to 482.1 kJ |
| Wall time per visit | 265 to 267 s |

OUT landings are about twice as far off as IN landings. The hop locks on from 2.5 m, where the
offset marker sits near the edge of the image, against 6 m before the IN landing. The worst OUT
landing used 13.6 of the 15 cm the latch funnel allows.

**Marker detection against the sim.** The scenario uses `pad_ops.p_marker_detect` = 0.97 per
approach. SITL found the marker and landed within the capture radius on all 20 approaches.
Twenty trials cannot tell 0.97 from 1.0: at 0.97, 20 of 20 happens 54% of the time.

The recommended value is **0.955**, Laplace's estimate (k + 1) / (n + 2). It moves the sim only
slightly and does not claim more than 20 landings show. It is not applied (Phase 5 decides).
A value worth trusting needs about 100 approaches, and ones with real detection noise: these
runs have no wind, blur or lighting change.

## Speed factor

`--speed 2` runs Gazebo's real-time factor and PX4's lockstep speed factor at 2. Seed 1 at 1x
against seed 1 at 2x:

| Measure | 1x (repeat_1) | 2x |
|---|---|---|
| Wall time (s) | 265 | 134 |
| Takeoff / cruise / approach + landing / hop (s) | 26.75 / 146.5 / 49.1 / 21.5 | 26.94 / 146.5 / 49.0 / 22.7 |
| Energy proxy, visit (kJ) | 480.2 | 482.1 |
| Landing error IN / OUT (cm) | 9.0 / 13.1 | 2.2 / 9.4 |

The 2x run's times and energy are inside the spread of the ten 1x repeats. Its landing errors
differ from the 1x run with the same seed, but by no more than the repeats differ from each
other: Gazebo itself is not seeded. On this host 2x does not change behaviour. The acceptance
runs above were still flown at 1x.

## Resources

Peak resident memory of the whole process tree, sampled every 0.5 s, was 792 to 802 MB per run.
In `repeat_1`, the largest processes peaked at:

| Process | Peak RSS (MB) |
|---|---|
| Gazebo server (`gz sim`, a Ruby process) | 506 |
| Each ROS node | 24 to 51 |
| ros_gz bridge | 49 |
| XRCE agent | 28 |
| PX4 | 21 |

## Not covered

- **Failure branches:** marker misses, go-arounds, faults, lost messages. They come in Phases 6
  and 7.
- **Environment and timing:**
  - no wind, terrain or buildings;
  - pads have no chargers (A58);
  - the approach waits for a marker lock instead of following the scenario's 45 s approach
    timing.
- **Data volume:** one lane and one vehicle. The landing statistics rest on 20 landings.
