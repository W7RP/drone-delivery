# Phase 7: VTOL airframe

The multirotor fitted in Phase 5 reaches 5.7 km with a 5 kg pod on the coupled 900 Wh battery,
so the parts-logistics scenario lost its Mon Valley factories. This phase flies a VTOL in PX4
SITL, fits a flight profile to it, and wires that profile into the sim. Every number is a
placeholder (spec A01) unless it comes from a cited source; SITL has no power model, so all
energies are a proxy (A57, A92).

## 1. Airframe audit

PX4 v1.17.0 ships a Gazebo standard VTOL (`Tools/simulation/gz/models/standard_vtol`, airframe
4004): a flying wing with four lift rotors on booms and a pusher. Read from its model and
airframe files:

| Item | Stock standard VTOL | Scaled for this phase |
|---|---|---|
| Mass | 5.025 kg (body 5, five rotors 0.005 each); no MTOW given | 9.0 kg empty + 0.6 kg latch + pod; design MTOW 16 kg (the scenario's) |
| Wing area | 2 x 0.50 m2 (lift-drag surfaces), elevator 0.01 m2 | 2 x 1.59 m2, elevator 0.032 m2 |
| Lift slope, stall angle | 4.75 /rad, 0.339 rad: CL max 1.61 | unchanged |
| Stall speed | 7.1 m/s at 5.0 kg | 6.8 m/s at 14.6 kg (5 kg pod) |
| Cruise (FW_AIRSPD_TRIM) | 15 m/s | 15 m/s, 2.2 times stall |
| Hover thrust (4 rotors, 1500 rad/s) | 180 N: thrust/weight 3.65 | 573 N: 4.0 at 14.6 kg |
| Pusher thrust (3500 rad/s) | 105 N | 333 N |

The Phase 5 multirotor was PX4's x500 scaled to 9 kg empty and 16 kg MTOW with thrust/weight
2.0 at MTOW.

Scaling (spec A91), the simplest that keeps PX4's 4004 gains valid: one factor s = 16 / 5.025
= 3.18 on body mass and inertia, every motor and rotor-drag constant, and the lift-drag
surface areas. At 16 kg the scaled vehicle then has the stock model's accelerations, stall
speed and thrust/weight; at 14.6 kg it has slightly more margin. Geometry stays the stock
model's, plus two skids under the motor booms. The stock model rests on its wing; the skids
give the carried pod 1 cm of ground clearance. The latch, the down camera (0.25 m forward,
0.15 m below the body) and the pod joints are added as on the x500. `px4/tools/make_vehicle.py
--airframe vtol` writes it from PX4's tree, which is not changed.

`px4/tools/vtol_smoke.sh` flies it with PX4's own commander, with the 5.6 kg of latch and pod
fixed to the body. At the stock 10 m take-off height the transition sank 8 m during the speed
build-up, and the vehicle hit the ground at 17 m/s. At the lane's 61 m it:

- climbs as a multirotor and transitions with a 2.5 m dip;
- loiters as a fixed wing at 61 +- 0.8 m and 15.0 m/s airspeed;
- transitions back and lands.

No tuning was needed beyond the airspeed parameters in `px4/params/sitl_vtol.params`.

## 2. Delivery visit

`px4/run_flight.sh --airframe vtol` flies the Phase 4 visit with the same pad, fleet manager,
detector and logger nodes, the same spec messages and logs. Only the lane changes. After the
multirotor climb (D02) the drone commands a transition to fixed wing and streams an offboard
path setpoint (the destination's holding fix with the track velocity), so PX4's fixed-wing
controller follows the lane line. 150 m before the fix (placeholder:
`px4/config/flight.yaml` `vtol.back_transition_m`) it commands the transition back, and it
covers the rest as a multirotor (D03). Approach, precision landing, release, hop and latch are
the Phase 4 multirotor ones. The runner refuses to start under 20 GB free and stops a visit,
Gazebo, PX4 and nodes included, after three times its expected wall time.

The logger records the VTOL phase changes (MC, TRANSITION_FW, FW, TRANSITION_MC) as events, and
the pusher command in the sample record (spec 9.5 version_minor 1, in the former reserved
bytes).

| Run (5 kg, seed 1) | Lane | Wall | Climb | To FW | FW cruise | Back to MC | MC to fix | Approach + landing IN | Landing error IN / OUT | Peak RSS | Validator |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1x | 3, hub to VA Pittsburgh, 2093 m | 273 s | 26.6 s | 8.2 s | 125.2 s | 6.3 s | 12.9 s | 49.7 s | 8.5 / 4.7 cm | 693 MB | pass |
| 2x | 4, hub to Edgar Thomson, 12 896 m | 499 s | 27.2 s | 8.2 s | 842.7 s | 6.5 s | 12.7 s | 45.2 s | 7.6 / 5.3 cm | 706 MB | pass |

The energy proxy over lane 3 (D02 to D03, transitions included) is 117 kJ, against 314 kJ for
the multirotor flown the same day. The multirotor still completes its fit-lane visit after
the node changes and passes the validator, and so does the committed Phase 4 sample log.

## 3. Calibration flights

`px4/tools/run_batch.sh vtol_cal vtol_0_1 vtol_0_2 vtol_2_1 vtol_2_2 vtol_5_1 vtol_5_2
vtol1x_5_1` flies the fit lane (3, 2093 m) with 0, 2 and 5 kg, two seeds each at 2x real time,
and 5 kg once at 1x. All seven completed, pass the validator, peak at 688 MB RSS and carry one
px4/ revision (`2549e822e0ac`, recorded in each run.json). A first batch mixed two runner
versions after an edit during the batch: one changed only run.json metadata, not the flight. It
was discarded and the whole batch re-flown. `pipeline/calib/vtol_phases.py` refuses a batch with
mixed or uncommitted versions.

That script reduces the logs to `pipeline/calib/vtol_phases.json` (committed; the raw logs are
not), one row per run and phase. Distances are along the line from the start to the holding
fix on the ground-truth track. Energy is the proxy (A57, A92). Mean of the two 2x runs per
payload:

| Phase | 0 kg: s, m/s, m along, kJ | 2 kg | 5 kg |
|---|---|---|---|
| MC takeoff and climb | 25.9, 0.0, 0, 22.6 | 26.2, 0.0, 0, 29.6 | 26.5, 0.0, 0, 41.9 |
| Transition to FW | 5.6, 5.9, 33, 6.5 | 6.7, 6.0, 40, 8.9 | 8.4, 6.2, 49, 13.8 |
| FW cruise | 125.5, 15.2, 1909, 42.2 | 125.8, 15.1, 1901, 52.5 | 125.0, 15.2, 1893, 69.8 |
| Back-transition and MC to the fix | 19.2, 7.8, 149, 17.6 | 19.3, 7.8, 149, 22.2 | 20.4, 7.4, 148, 33.1 |
| Approach and landing on IN | 50.0, 0.1, -2, 42.5 | 49.2, 0.2, -2, 55.8 | 50.5, 0.1, -2, 81.1 |
| Hop IN to OUT (empty) | 23.7, 0.4, 4, 16.4 | 25.4, 0.4, 4, 18.0 | 23.4, 0.4, 4, 16.0 |

Mean cruise power is 336, 418 and 559 W at 0, 2 and 5 kg; multicopter phases run at about
three times that.

Energy proxy. The four lift rotors use A57's induced power from their commanded speeds. The
pusher uses ideal momentum theory for a propeller in axial flight: thrust T = k w^2, induced
velocity v_i = -V/2 + sqrt(V^2/4 + T/(2 rho A)), power T (V + v_i), with V the speed along the
body x axis (no wind) and the scaled stock disc areas (A92).

Cross-check against the simulator. In steady cruise the pusher thrust should balance the
wing drag of Gazebo's lift-drag model. That drag is 1/2 rho V^2 A cd per surface, with
cd = cda (alpha + a0), alpha from the ground-truth velocity in the body frame and PX4's
attitude. The reference power is drag times speed over the ideal propeller efficiency,
D (V + v_i). Over the middle half of each cruise, the pusher thrust from its commands matches
that drag within the noise of the differentiated track: 14.4 N against 13.9 to 16.5 N at 0 kg,
21.7 N against 20.5 to 23.3 N at 5 kg. The proxy cruise power over the reference is 0.81 to
1.24, mean 1.00 over the seven runs. It is still a proxy: SITL has no power model, and
propeller, motor and ESC losses are left out.

1x against 2x (5 kg, seed 1): the lane (D02 to D03, transitions included) took 152.1 s and
112.3 kJ at 1x, and 151.8 s and 115.0 kJ at 2x (-0.2% and +2.3%). The cruise plateau was
14.99 and 15.46 m/s. The second 2x seed took 155.6 s and 118.4 kJ, so the speed factor's
effect is within the seed-to-seed spread.

`px4/samples/vtol_5_1/` keeps the logs of the first 5 kg run; the others are deleted once
summarized.

## 4. Fit

`pipeline/run.sh calib vtol` runs the VTOL steps; `vtol_fit` fits the profile to the committed
table and `vtol_profile` writes `configs/flight_profiles/px4_sitl_vtol_proxy.toml`. Run twice,
the fit gives byte-identical files: fixed bootstrap seed, sorted inputs. Only the six 2x runs
are fitted; the 1x run is compared, not used. The method is Phase 5's (`docs/calibration.md`):

- constants from the data;
- a time depends on payload only if its regression slope is significant at 5%;
- powers by least squares on relative energy residuals;
- at most three parameters per phase class;
- leave-one-payload-out cross-validation through the core's own model (`sim flight-model`);
- a 2000-draw bootstrap over runs for the uncertainty.

The sim's model of a VTOL visit (spec A93) is takeoff_s + transition_s + L / cruise_speed_mps +
approach_s + hop_s. The multicopter phases use the hover power, the lane uses the cruise power,
and the transition overhead (the transitions and the multicopter stretch to the fix) uses its
own power.

| Class | Parameter | Value | Bootstrap std | Payload dependence (p) |
|---|---|---|---|---|
| Multicopter | hover_power_ref_w | 1472.5 W | 4.2 | power with mass^1.5 |
| | takeoff_s | 26.21 s | 0.12 | none (p = 0.051) |
| | approach_s | 49.88 s | 0.63 | none (0.75) |
| | hop_s | 24.16 s | 0.40 | none (0.63) |
| Cruise | cruise_speed_mps | 15.211 m/s | 0.054 | none (0.73) |
| | cruise_power_ref_w | 556.4 W | 4.5 | |
| | cruise_mass_exponent | 1.212 | 0.028 | significant (p < 1e-6) |
| Transition | transition_s | 14.30 s | 0.69 | none (p = 0.064) |
| | transition_power_ref_w | 2658 W | 95 | power with mass^1.5 |

The reference mass is 14.6 kg, as for the multirotor, and the profile's MTOW is 16 kg. The
fitted VTOL hovers at 1472 W at 14.6 kg against the multirotor's 1831 W, because its scaled
rotor discs are larger (A92). It cruises at 556 W against 2108 W. Cruise power grows with mass
to the 1.21, not the 1.5 of hovering.

Residuals: cruise energy within 1.7% of the model in every run. The single hover power leaves
per-phase residuals as in Phase 5 (A63): takeoff +8%, approach +8%, hop -13%. Fitted takeoff
and transition times grow slightly with payload (p = 0.051 and 0.064), so they stay constants.

| Visit (fit lane) | Time error, in sample | Time error, payload left out | Energy error, in sample | Energy error, payload left out |
|---|---|---|---|---|
| 0 kg, seeds 1 and 2 | +1.0%, +0.9% | +1.5%, +1.3% | -1.9%, -1.8% | -2.3%, -2.2% |
| 2 kg | +0.3%, -0.7% | +0.2%, -0.8% | -0.5%, -3.6% | -0.5%, -3.6% |
| 5 kg | -0.4%, -1.1% | -0.8%, -1.5% | -3.9%, -3.9% | -4.8%, -4.8% |

Leaving a payload out costs at most 1.5% in time and 4.8% in energy. The fit was committed
before the held-out flights were flown.
