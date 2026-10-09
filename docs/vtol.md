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
