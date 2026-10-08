//! Pad controller (spec 3): pad mode (3.1), pods on spots (3.2), visit machine (3.3, 3.4) and
//! spot assignment (3.5). Transition IDs in comments and logs are the spec's row IDs.

use crate::event::{Ev, Msg, NO_STATE, tcode};
use crate::ids::{
    AlertCode, AlertSeverity, DroneId, EntityKind, FaultKind, HoldReason, MissionKind, NO_DRONE,
    NO_ORDER, NO_PAD, NO_POD, OrderKind, OutEmptyPolicy, PadId, PadMode, PodId, ServiceOutcome,
    Spot, TimeMs, TimerKind, VisitOutcome, VisitState, ms_from_s,
};
use crate::world::{OutSpot, Placement, Shipment, Visit, World};

impl World {
    fn pad(&self, p: PadId) -> &crate::world::PadSt {
        &self.pads[usize::from(p)]
    }

    fn pad_mut(&mut self, p: PadId) -> &mut crate::world::PadSt {
        &mut self.pads[usize::from(p)]
    }

    fn out(&self, p: PadId, k: usize) -> &OutSpot {
        &self.pads[usize::from(p)].outs[k]
    }

    fn out_mut(&mut self, p: PadId, k: usize) -> &mut OutSpot {
        &mut self.pads[usize::from(p)].outs[k]
    }

    fn n_out(&self, p: PadId) -> usize {
        self.pads[usize::from(p)].outs.len()
    }

    /// The OUT spot index of the drone's visit (its granted, landed or hop-target spot).
    fn visit_out(&self, p: PadId, d: DroneId) -> usize {
        self.visit(p, d).spot.out_index().unwrap_or(0)
    }

    fn visit(&self, p: PadId, d: DroneId) -> &Visit {
        &self.visits[self.visit_index(p, d)]
    }

    fn visit_mut(&mut self, p: PadId, d: DroneId) -> &mut Visit {
        let i = self.visit_index(p, d);
        &mut self.visits[i]
    }

    /// Move visit `d` to `to`, log transition `code`, bump generation, keep the movement count.
    fn visit_to(&mut self, p: PadId, d: DroneId, code: u16, to: VisitState) {
        let now = self.now;
        let v = self.visit_mut(p, d);
        let (pad, from) = (v.pad, v.state);
        v.state = to;
        v.since = now;
        v.generation = v.generation.wrapping_add(1);
        v.retries = 0;
        let moving = |s: VisitState| matches!(s, VisitState::Hopping | VisitState::Departing);
        let pad_st = &mut self.pads[usize::from(pad)];
        if moving(from) {
            pad_st.movements -= 1;
        }
        if moving(to) {
            pad_st.movements += 1;
        }
        // A visit only leaves CLOSED when it is created, which the spec writes as "(none)".
        let from_code = if from == VisitState::Closed {
            NO_STATE
        } else {
            from.code()
        };
        self.log_transition(code, pad, d, from_code, to.code());
    }

    fn visit_close(&mut self, p: PadId, d: DroneId, code: u16, outcome: VisitOutcome) {
        self.visit_to(p, d, code, VisitState::Closed);
        self.visit_mut(p, d).outcome = outcome;
    }

    fn pc_timer_arm(&mut self, p: PadId, d: DroneId, timer: TimerKind, delay_ms: TimeMs) {
        let generation = self.visit(p, d).generation;
        self.timer(delay_ms, timer, EntityKind::Pad, p, d, generation);
    }

    fn pad_status(&mut self, p: PadId) {
        let s = self.pad(p);
        let msg = Msg::PadStatus {
            pad: p,
            mode: s.mode,
            empty_pods: u16::try_from(s.empty_pods).unwrap_or(u16::MAX),
            backlog_len: u16::try_from(s.backlog.len()).unwrap_or(u16::MAX),
            in_drone: s.in_drone,
            out_drone: s.outs[0].drone,
            out_pod: s.outs[0].pod,
            marker_fault: s.marker_fault && self.cfg.pad_ops.marker_self_check,
        };
        self.send(msg);
    }

    fn set_in_drone(&mut self, p: PadId, d: DroneId) {
        self.pad_mut(p).in_drone = d;
        self.metrics
            .spot_occupied(usize::from(p), None, d != NO_DRONE, self.now);
    }

    fn set_out_drone(&mut self, p: PadId, k: usize, d: DroneId) {
        self.out_mut(p, k).drone = d;
        self.metrics
            .spot_occupied(usize::from(p), Some(k), d != NO_DRONE, self.now);
    }

    fn set_empty_pods(&mut self, p: PadId, n: u32) {
        self.pad_mut(p).empty_pods = n;
        self.metrics.pods_available(usize::from(p), n > 0, self.now);
    }

    fn in_free(&self, p: PadId) -> bool {
        let s = self.pad(p);
        s.in_drone == NO_DRONE && s.in_reserved == NO_DRONE && s.in_pod == NO_POD
    }

    fn out_drone_free(&self, p: PadId, k: usize) -> bool {
        let o = self.out(p, k);
        o.drone == NO_DRONE && o.reserved == NO_DRONE && !o.placing
    }

    /// The OUT spot an empty drone should use: the one holding its pickup pod, else one with
    /// no pod (lowest index), both drone-free; `sensor` adds the presence interlock.
    fn out_for(&self, p: PadId, d: DroneId, empty_ok: bool, sensor: bool) -> Option<usize> {
        let usable = |k: usize| {
            self.out_drone_free(p, k) && (!sensor || self.spot_sensor_clear(p, Spot::out(k)))
        };
        let ks = 0..self.n_out(p);
        let mine = ks.clone().find(|&k| {
            let o = self.out(p, k);
            o.pod != NO_POD && o.pickup_drone == d && usable(k)
        });
        mine.or_else(|| {
            ks.clone()
                .find(|&k| empty_ok && self.out(p, k).pod == NO_POD && usable(k))
        })
    }

    /// Spec 3.5, with the presence-sensor interlock: a spot whose sensor shows a drone is
    /// never granted, whatever the PC's records say.
    fn spot_for(&self, p: PadId, d: DroneId, carries_pod: bool) -> Option<Spot> {
        let in_ok = self.in_free(p) && self.spot_sensor_clear(p, Spot::In);
        if carries_pod {
            return in_ok.then_some(Spot::In);
        }
        if let Some(k) = self.out_for(p, d, true, true) {
            Some(Spot::out(k))
        } else if in_ok {
            Some(Spot::In)
        } else {
            None
        }
    }

    fn spot_sensor_clear(&self, p: PadId, spot: Spot) -> bool {
        !self
            .drones
            .iter()
            .any(|dr| dr.pad == p && dr.spot == Some(spot))
    }

    fn charge_needed(&self, p: PadId, soc: f32) -> bool {
        self.pad(p).has_charger && f64::from(soc) < self.cfg.drone.charge_target_soc
    }

    /// With parked charging and a free parking slot, a drone with nothing to latch charges in
    /// the parking area instead of holding OUT (A72).
    fn charges_in_parking(&self, p: PadId) -> bool {
        let s = self.pad(p);
        self.cfg.charging.charge_while_parked && s.parked < s.parking_capacity
    }

    /// The command the drone should currently hold, for V38 and M02 re-sends.
    fn current_command(&self, p: PadId, d: DroneId) -> Option<Msg> {
        let v = self.visit(p, d);
        let (pad, drone) = (v.pad, d);
        let s = self.pad(pad);
        Some(match v.state {
            VisitState::Held => {
                let pos = s.landing_queue.iter().position(|&q| q == d).unwrap_or(0);
                Msg::LandingHold {
                    pad,
                    drone,
                    reason: HoldReason::InOccupied,
                    queue_pos: u16::try_from(pos).unwrap_or(u16::MAX),
                }
            }
            VisitState::Approach => Msg::LandingGrant {
                pad,
                drone,
                spot: v.spot,
            },
            VisitState::Releasing => Msg::ReleaseCmd {
                pad,
                drone,
                attempt: v.attempt,
            },
            VisitState::Hopping => Msg::HopCmd {
                pad,
                drone,
                out_index: v.hop_out,
            },
            VisitState::Latching => Msg::LatchCmd {
                pad,
                drone,
                pod: s.outs[self.visit_out(p, d)].pod,
                attempt: v.attempt,
            },
            VisitState::Charging => {
                let remaining = v.charge_end.saturating_sub(self.now);
                #[allow(clippy::cast_possible_truncation)]
                let dwell_s = crate::ids::s_from_ms(remaining) as f32;
                Msg::ChargeStart {
                    pad,
                    drone,
                    dwell_s,
                }
            }
            VisitState::Ready if v.service_sent => self.service_complete_msg(p, d, v.carried),
            VisitState::Departing => Msg::DepartClear { pad, drone },
            _ => return None,
        })
    }

    fn service_complete_msg(&self, p: PadId, d: DroneId, outcome: ServiceOutcome) -> Msg {
        let v = self.visit(p, d);
        Msg::ServiceComplete {
            pad: v.pad,
            drone: d,
            outcome,
            spot: v.spot,
            pod: self.drones[usize::from(d)].pod,
            soc: self.soc_f32(d),
        }
    }

    fn send_service_complete(&mut self, p: PadId, d: DroneId, outcome: ServiceOutcome) {
        let msg = self.service_complete_msg(p, d, outcome);
        let v = self.visit_mut(p, d);
        v.carried = outcome;
        v.service_sent = true;
        self.send(msg);
    }

    pub(crate) fn pc_message(&mut self, p: PadId, msg: Msg) {
        match msg {
            Msg::LandingRequest {
                drone, pod, soc, ..
            } => self.pc_landing_request(p, drone, pod, soc),
            Msg::PickupAssigned {
                pod, order, drone, ..
            } => {
                // P07
                if let Some(k) = (0..self.n_out(p)).find(|&k| self.out(p, k).pod == pod) {
                    let o = self.out_mut(p, k);
                    o.pickup_drone = drone;
                    o.pickup_order = order;
                    self.log_transition(tcode::p(7), p, drone, NO_STATE, NO_STATE);
                }
            }
            Msg::PodReturnRequest { pod, dest, .. } => self.pc_return_request(p, pod, dest),
            _ => self.pc_visit_message(p, msg),
        }
    }

    fn pc_landing_request(&mut self, p: PadId, d: DroneId, pod: PodId, soc: f32) {
        if self.visit(p, d).is_open_at(p) {
            match self.visit(p, d).state {
                VisitState::Held => {
                    let cmd = self.current_command(p, d).expect("HELD has a reply");
                    self.log_transition(
                        tcode::v(3),
                        p,
                        d,
                        VisitState::Held.code(),
                        VisitState::Held.code(),
                    );
                    self.send(cmd);
                }
                VisitState::Approach => {
                    // V11: the drone approaches from this re-sent grant, so the landing
                    // timeout restarts with it (a fresh generation drops the old timer).
                    self.visit_to(p, d, tcode::v(11), VisitState::Approach);
                    let timeout = self.t_land_timeout_ms();
                    self.pc_timer_arm(p, d, TimerKind::PcLandingTimeout, timeout);
                    let spot = self.visit(p, d).spot;
                    self.send(Msg::LandingGrant {
                        pad: p,
                        drone: d,
                        spot,
                    });
                }
                _ => self.pc_duplicate(p, d),
            }
            return;
        }
        if self.pad(p).mode == PadMode::OutOfService {
            self.log_transition(tcode::v(0), p, d, NO_STATE, VisitState::Closed.code());
            self.send(Msg::LandingRefused {
                pad: p,
                drone: d,
                reason: HoldReason::OutOfService,
            });
            return;
        }
        let mut v = Visit::closed();
        v.pad = p;
        v.state = VisitState::Closed;
        v.arrived_with_pod = pod;
        v.soc = soc;
        v.generation = self.visit(p, d).generation.wrapping_add(1);
        v.since = self.now;
        {
            let i = self.visit_index(p, d);
            self.visits[i] = v;
        }
        if self.pad(p).landing_queue.is_empty()
            && let Some(spot) = self.spot_for(p, d, pod != NO_POD)
        {
            self.grant(p, d, spot, tcode::v(1));
            return;
        }
        // V02
        self.pad_mut(p).landing_queue.push_back(d);
        self.visit_to(p, d, tcode::v(2), VisitState::Held);
        self.visit_mut(p, d).held_since = self.now;
        let cmd = self.current_command(p, d).expect("HELD has a reply");
        self.send(cmd);
    }

    fn grant(&mut self, p: PadId, d: DroneId, spot: Spot, code: u16) {
        match spot.out_index() {
            None => self.pad_mut(p).in_reserved = d,
            Some(k) => self.out_mut(p, k).reserved = d,
        }
        self.visit_mut(p, d).spot = spot;
        self.visit_to(p, d, code, VisitState::Approach);
        let timeout = self.t_land_timeout_ms();
        self.pc_timer_arm(p, d, TimerKind::PcLandingTimeout, timeout);
        self.send(Msg::LandingGrant {
            pad: p,
            drone: d,
            spot,
        });
    }

    fn free_reservation(&mut self, p: PadId, d: DroneId) {
        let s = self.pad_mut(p);
        if s.in_reserved == d {
            s.in_reserved = NO_DRONE;
        }
        for o in &mut s.outs {
            if o.reserved == d {
                o.reserved = NO_DRONE;
            }
        }
    }

    /// V38: a drone repeated a message we already answered; re-send our current command.
    fn pc_duplicate(&mut self, p: PadId, d: DroneId) {
        if let Some(cmd) = self.current_command(p, d) {
            let s = self.visit(p, d).state.code();
            self.log_transition(tcode::v(38), p, d, s, s);
            self.send(cmd);
        }
    }

    fn pc_visit_message(&mut self, p: PadId, msg: Msg) {
        let (d, is_drone) = match msg.sender() {
            (EntityKind::Drone, d) => (d, true),
            _ => (NO_DRONE, false),
        };
        if !is_drone {
            return;
        }
        if !self.visit(p, d).is_open_at(p) {
            match msg {
                Msg::DepartRequest { mission, .. } => self.pc_depart_from_parking(p, d, mission),
                Msg::Landed { spot, pod, soc, .. } => {
                    self.pc_adopt_late_landing(p, d, spot, pod, soc)
                }
                // LANDING_CANCEL / APPROACH_ABORT / anything else for no open visit: ignored.
                _ => {}
            }
            return;
        }
        let state = self.visit(p, d).state;
        if state == VisitState::DroneSilent {
            // V37
            let back = self.visit(p, d).silent_from;
            self.visit_to(p, d, tcode::v(37), back);
            // The resumed state's reply timer was lost with the old generation; re-arm it.
            let ms = self.cfg.handshake.response_timeout_ms;
            self.pc_timer_arm(p, d, TimerKind::PcResponseTimeout, ms);
            self.pc_visit_message(p, msg);
            return;
        }
        match (state, msg) {
            (VisitState::Held | VisitState::Approach, Msg::LandingCancel { .. }) => {
                self.pad_mut(p).landing_queue.retain(|&q| q != d);
                self.free_reservation(p, d);
                self.visit_close(p, d, tcode::v(5), VisitOutcome::Cancelled);
            }
            (VisitState::Approach, Msg::ApproachAbort { .. }) => {
                self.free_reservation(p, d);
                self.visit_close(p, d, tcode::v(12), VisitOutcome::ApproachAborted);
            }
            (VisitState::Approach, Msg::Landed { spot, pod, soc, .. })
                if spot == self.visit(p, d).spot =>
            {
                self.pc_land(p, d, spot, pod, soc, None);
            }
            (VisitState::Hopping, Msg::Landed { spot, soc, .. }) if spot.is_out() => {
                self.pc_hop_landed(p, d, soc, None)
            }
            (VisitState::Releasing, Msg::ReleaseReport { latch_open, .. }) => {
                self.visit_mut(p, d).latch_open = latch_open;
                self.visit_to(p, d, tcode::v(15), VisitState::VerifyRelease);
                let ms = self.cfg.pad_ops.release_verify_ms;
                self.pc_timer_arm(p, d, TimerKind::PcVerifyReleaseDone, ms);
            }
            (VisitState::Latching, Msg::LatchReport { locked, .. }) => {
                self.visit_mut(p, d).locked = locked;
                self.visit_to(p, d, tcode::v(28), VisitState::VerifyLock);
                let ms = self.cfg.pad_ops.latch_verify_ms;
                self.pc_timer_arm(p, d, TimerKind::PcVerifyLockDone, ms);
            }
            (VisitState::Ready, Msg::DepartRequest { mission, dest, .. })
                if mission == MissionKind::Pickup && dest == p && self.hop_to_pickup(p, d) => {}
            (VisitState::Ready, Msg::DepartRequest { mission, dest, .. })
                if mission == MissionKind::Park
                    && dest == p
                    && self.visit(p, d).spot == Spot::In
                    && self.pad(p).parked >= self.pad(p).parking_capacity =>
            {
                self.hop_to_free_out(p, d);
            }
            (VisitState::Ready, Msg::DepartRequest { mission, .. }) => {
                self.visit_mut(p, d).depart_mission = mission;
                if !self.pad(p).depart_queue.contains(&d) {
                    self.pad_mut(p).depart_queue.push_back(d);
                }
            }
            (VisitState::Departing, Msg::Departed { .. }) => self.pc_departed(p, d, tcode::v(36)),
            _ => self.pc_duplicate(p, d),
        }
    }

    /// V42: LANDED from a drone whose visit was closed (its grant timed out just before it
    /// touched down). The sensor confirms it; the PC adopts it so it is never stranded.
    fn pc_adopt_late_landing(&mut self, p: PadId, d: DroneId, spot: Spot, pod: PodId, soc: f32) {
        let recorded_free = match (spot, spot.out_index()) {
            (Spot::In, _) => {
                self.pad(p).in_drone == NO_DRONE && self.pad(p).in_reserved == NO_DRONE
            }
            (_, Some(k)) if k < self.n_out(p) => {
                self.out(p, k).drone == NO_DRONE && self.out(p, k).reserved == NO_DRONE
            }
            _ => false,
        };
        if !(recorded_free && self.sensor_shows(p, d, spot)) {
            return;
        }
        let mut v = Visit::closed();
        v.pad = p;
        v.spot = spot;
        v.arrived_with_pod = pod;
        v.soc = soc;
        v.generation = self.visit(p, d).generation.wrapping_add(1);
        v.since = self.now;
        {
            let i = self.visit_index(p, d);
            self.visits[i] = v;
        }
        self.visit_to(p, d, tcode::v(42), VisitState::Approach);
        self.pc_land(p, d, spot, pod, soc, None);
    }

    /// V40: a drone in the parking area asks to leave.
    fn pc_depart_from_parking(&mut self, p: PadId, d: DroneId, mission: MissionKind) {
        let drone = &self.drones[usize::from(d)];
        if drone.pad != p || drone.spot != Some(Spot::Parking) {
            return;
        }
        let mut v = Visit::closed();
        v.pad = p;
        v.spot = Spot::Parking;
        v.generation = self.visit(p, d).generation.wrapping_add(1);
        v.depart_mission = mission;
        v.soc = self.soc_f32(d);
        {
            let i = self.visit_index(p, d);
            self.visits[i] = v;
        }
        self.visit_to(p, d, tcode::v(40), VisitState::Ready);
        self.pad_mut(p).depart_queue.push_back(d);
    }

    /// V06 to V10 (from APPROACH), or V13 when `fallback` (landing timeout, sensor shows drone).
    fn pc_land(
        &mut self,
        p: PadId,
        d: DroneId,
        spot: Spot,
        pod: PodId,
        soc: f32,
        fallback: Option<u16>,
    ) {
        self.free_reservation(p, d);
        self.visit_mut(p, d).soc = soc;
        let code = |row: u16| fallback.unwrap_or(tcode::v(row));
        if spot == Spot::In {
            self.set_in_drone(p, d);
            if pod != NO_POD {
                self.visit_mut(p, d).attempt = 1;
                self.visit_to(p, d, code(6), VisitState::Releasing);
                self.send_release_cmd(p, d);
            } else {
                self.visit_to(p, d, code(7), VisitState::WaitOut);
                self.visit_mut(p, d).out_wait_expired = false;
                let ms = self.cfg.pad_ops.out_empty_wait_ms;
                self.pc_timer_arm(p, d, TimerKind::PcOutWaitTimeout, ms);
            }
            return;
        }
        let k = spot.out_index().expect("a landing on IN returned above");
        self.set_out_drone(p, k, d);
        self.pc_on_out(p, d, k, soc, [code(8), code(9), code(10)]);
    }

    /// Common tail of V08-V10 and V24-V26: latch a pickup, else charge, else ready.
    fn pc_on_out(&mut self, p: PadId, d: DroneId, k: usize, soc: f32, codes: [u16; 3]) {
        let o = self.out(p, k);
        if o.pod != NO_POD && o.pickup_drone == d {
            let order = o.pickup_order;
            if order != NO_ORDER {
                self.fm.orders[order as usize].latch_start_t_ms = self.now;
            }
            self.visit_mut(p, d).attempt = 1;
            self.visit_to(p, d, codes[0], VisitState::Latching);
            self.send_latch_cmd(p, d);
        } else if self.charge_needed(p, soc) && !self.charges_in_parking(p) {
            self.start_charging(p, d, codes[1], ServiceOutcome::NoPod);
        } else {
            self.visit_to(p, d, codes[2], VisitState::Ready);
            self.send_service_complete(p, d, ServiceOutcome::NoPod);
        }
    }

    fn start_charging(&mut self, p: PadId, d: DroneId, code: u16, carried: ServiceOutcome) {
        let soc = f64::from(self.visit(p, d).soc);
        let c = &self.cfg;
        let dwell_s = (c.drone.charge_target_soc - soc).max(0.0) * c.drone.battery_capacity_j
            / (c.charging.power_w * c.charging.efficiency);
        let dwell_ms = ms_from_s(dwell_s);
        self.visit_to(p, d, code, VisitState::Charging);
        let now = self.now;
        let v = self.visit_mut(p, d);
        v.carried = carried;
        v.charge_end = now + dwell_ms;
        self.pc_timer_arm(p, d, TimerKind::PcChargeDone, dwell_ms);
        #[allow(clippy::cast_possible_truncation)]
        self.send(Msg::ChargeStart {
            pad: p,
            drone: d,
            dwell_s: dwell_s as f32,
        });
    }

    fn send_release_cmd(&mut self, p: PadId, d: DroneId) {
        let attempt = self.visit(p, d).attempt;
        let ms = self.service_ms(
            p,
            self.cfg.pad_ops.manual_release_ms,
            self.cfg.pad_ops.release_ms,
        ) + self.cfg.handshake.response_timeout_ms;
        self.pc_timer_arm(p, d, TimerKind::PcResponseTimeout, ms);
        self.send(Msg::ReleaseCmd {
            pad: p,
            drone: d,
            attempt,
        });
    }

    fn send_latch_cmd(&mut self, p: PadId, d: DroneId) {
        let attempt = self.visit(p, d).attempt;
        let pod = self.out(p, self.visit_out(p, d)).pod;
        let ms = self.service_ms(
            p,
            self.cfg.pad_ops.manual_latch_ms,
            self.cfg.pad_ops.latch_ms,
        ) + self.cfg.handshake.response_timeout_ms;
        self.pc_timer_arm(p, d, TimerKind::PcResponseTimeout, ms);
        self.send(Msg::LatchCmd {
            pad: p,
            drone: d,
            pod,
            attempt,
        });
    }

    /// V43: a drone on one OUT spot asks to leave for a pickup at this pad while its pod waits
    /// on another OUT spot. It hops there on the ground instead of taking off and queueing to
    /// land again (A72). Returns whether the hop was commanded.
    fn hop_to_pickup(&mut self, p: PadId, d: DroneId) -> bool {
        // Only an empty drone: one that latched a pod in this visit leaves as a normal departure.
        let v = self.visit(p, d);
        if !v.spot.is_out() || v.locked {
            return false;
        }
        let Some(k) = self.out_for(p, d, false, false) else {
            return false;
        };
        self.out_mut(p, k).reserved = d;
        self.visit_mut(p, d).hop_out = u8::try_from(k).expect("at most 3 OUT spots");
        self.visit_to(p, d, tcode::v(43), VisitState::Hopping);
        self.send_hop(p, d);
        true
    }

    /// V44 (A74): an idle drone on IN makes room by hopping to a free OUT spot, because the
    /// parking area has no room. With no free OUT spot the request waits: the PC answers as to a
    /// duplicate, and the drone's watchdog asks again.
    fn hop_to_free_out(&mut self, p: PadId, d: DroneId) {
        let Some(k) = self.out_for(p, d, true, true) else {
            self.pc_duplicate(p, d);
            return;
        };
        self.out_mut(p, k).reserved = d;
        self.visit_mut(p, d).hop_out = u8::try_from(k).expect("at most 3 OUT spots");
        self.visit_to(p, d, tcode::v(44), VisitState::Hopping);
        self.send_hop(p, d);
    }

    /// V24 to V26 (or V13-style fallback from the hop timeout): the drone has hopped from IN, or
    /// from another OUT spot (V43), to its OUT spot.
    fn pc_hop_landed(&mut self, p: PadId, d: DroneId, soc: f32, fallback: Option<u16>) {
        self.free_reservation(p, d);
        self.visit_mut(p, d).soc = soc;
        match self.visit(p, d).spot.out_index() {
            Some(from) => self.set_out_drone(p, from, NO_DRONE),
            None => {
                self.set_in_drone(p, NO_DRONE);
                self.drone_left_in(p);
            }
        }
        let k = usize::from(self.visit(p, d).hop_out);
        self.set_out_drone(p, k, d);
        self.visit_mut(p, d).spot = Spot::out(k);
        let code = |row: u16| fallback.unwrap_or(tcode::v(row));
        self.pc_on_out(p, d, k, soc, [code(24), code(25), code(26)]);
    }

    /// P04: the removal clock for a delivered pod starts when the drone leaves IN.
    fn drone_left_in(&mut self, p: PadId) {
        let s = self.pad_mut(p);
        if s.in_pod != NO_POD && !s.in_pod_removal_scheduled {
            s.in_pod_removal_scheduled = true;
            let pod = s.in_pod;
            let ms = self.cfg.pad_ops.pod_remove_ms;
            self.log_transition(tcode::p(4), p, NO_DRONE, NO_STATE, NO_STATE);
            self.after(ms, Ev::PodRemoveDone { pad: p, pod });
        }
    }

    fn pc_departed(&mut self, p: PadId, d: DroneId, code: u16) {
        let spot = self.visit(p, d).spot;
        match (spot, spot.out_index()) {
            (Spot::In, _) => {
                self.set_in_drone(p, NO_DRONE);
                self.drone_left_in(p);
            }
            (_, Some(k)) => self.set_out_drone(p, k, NO_DRONE),
            _ => self.pad_mut(p).parked -= 1,
        }
        if self.visit(p, d).depart_mission == MissionKind::Park {
            self.pad_mut(p).parked += 1;
        }
        self.visit_close(p, d, code, VisitOutcome::Completed);
    }

    fn sensor_shows(&self, p: PadId, d: DroneId, spot: Spot) -> bool {
        let drone = &self.drones[usize::from(d)];
        drone.pad == p && drone.spot == Some(spot)
    }

    pub(crate) fn pc_timer(&mut self, p: PadId, d: DroneId, timer: TimerKind) {
        let state = self.visit(p, d).state;
        let max_retries = self.cfg.handshake.max_retries;
        match (timer, state) {
            (TimerKind::PcResponseTimeout, VisitState::DroneSilent) => {
                let spot = self.visit(p, d).spot;
                if self.sensor_shows(p, d, spot) {
                    let ms = self.cfg.handshake.response_timeout_ms;
                    self.pc_timer_arm(p, d, TimerKind::PcResponseTimeout, ms);
                } else if self.visit(p, d).silent_from == VisitState::Departing {
                    // V41: the silent drone left; close the departure as V36 would.
                    self.pc_departed(p, d, tcode::v(41));
                } else {
                    match (spot, spot.out_index()) {
                        (Spot::In, _) => self.set_in_drone(p, NO_DRONE),
                        (_, Some(k)) => self.set_out_drone(p, k, NO_DRONE),
                        _ => {}
                    }
                    self.free_reservation(p, d);
                    self.visit_close(p, d, tcode::v(41), VisitOutcome::LostContact);
                }
            }
            (TimerKind::PcLandingTimeout, VisitState::Approach) => {
                let spot = self.visit(p, d).spot;
                if self.sensor_shows(p, d, spot) {
                    let (pod, soc) = (self.visit(p, d).arrived_with_pod, self.visit(p, d).soc);
                    self.pc_land(p, d, spot, pod, soc, Some(tcode::v(13)));
                } else {
                    self.free_reservation(p, d);
                    self.visit_close(p, d, tcode::v(14), VisitOutcome::LostContact);
                    self.alert(
                        AlertCode::DroneSilent,
                        AlertSeverity::Warning,
                        (EntityKind::Pad, p),
                        p,
                        d,
                        NO_ORDER,
                    );
                }
            }
            (TimerKind::PcResponseTimeout, VisitState::Hopping)
                if self.sensor_shows(p, d, Spot::out(usize::from(self.visit(p, d).hop_out))) =>
            {
                let soc = self.visit(p, d).soc;
                self.pc_hop_landed(p, d, soc, None);
            }
            (TimerKind::PcResponseTimeout, VisitState::Departing)
                if !self.sensor_shows(p, d, self.visit(p, d).spot) =>
            {
                self.pc_departed(p, d, tcode::v(36));
            }
            (
                TimerKind::PcResponseTimeout,
                VisitState::Releasing
                | VisitState::Hopping
                | VisitState::Latching
                | VisitState::Departing,
            ) => {
                if self.visit(p, d).retries < max_retries {
                    let code = match state {
                        VisitState::Releasing => tcode::v(16),
                        VisitState::Hopping => tcode::v(23),
                        VisitState::Latching => tcode::v(27),
                        _ => tcode::v(35),
                    };
                    self.pc_retry(p, d, code);
                } else {
                    self.visit_mut(p, d).silent_from = state;
                    self.visit_to(p, d, tcode::v(17), VisitState::DroneSilent);
                    // Keep watching the spot sensor while silent (V41).
                    let ms = self.cfg.handshake.response_timeout_ms;
                    self.pc_timer_arm(p, d, TimerKind::PcResponseTimeout, ms);
                    self.alert(
                        AlertCode::DroneSilent,
                        AlertSeverity::Warning,
                        (EntityKind::Pad, p),
                        p,
                        d,
                        NO_ORDER,
                    );
                }
            }
            (TimerKind::PcVerifyReleaseDone, VisitState::VerifyRelease) => {
                self.pc_verify_release(p, d)
            }
            (TimerKind::PcVerifyLockDone, VisitState::VerifyLock) => self.pc_verify_lock(p, d),
            (TimerKind::PcOutWaitTimeout, VisitState::WaitOut) => {
                self.visit_mut(p, d).out_wait_expired = true;
                if !self.try_hop(p, d) {
                    self.visit_to(p, d, tcode::v(22), VisitState::Ready);
                    self.send_service_complete(p, d, ServiceOutcome::NoPod);
                }
            }
            (TimerKind::PcChargeDone, VisitState::Charging) => {
                let c = &self.cfg;
                let added = s_dwell(self.visit(p, d).charge_end, self.visit(p, d).since)
                    * c.charging.power_w
                    * c.charging.efficiency;
                self.add_charge(d, added);
                let carried = self.visit(p, d).carried;
                self.visit_to(p, d, tcode::v(33), VisitState::Ready);
                self.send_service_complete(p, d, carried);
            }
            _ => {}
        }
    }

    /// Physical charging effect (A16): energy into the battery, capped at the target.
    pub(crate) fn add_charge(&mut self, d: DroneId, joules: f64) {
        let cap = self.cfg.drone.battery_capacity_j;
        let target = self.cfg.drone.charge_target_soc;
        let drone = &mut self.drones[usize::from(d)];
        let room = ((target - drone.soc64()) * cap).max(0.0);
        let added = joules.min(room);
        let soc = drone.soc64() + added / cap;
        drone.set_soc(soc);
        drone.energy_charged_j += added;
        self.metrics.energy_added(usize::from(d), self.now, added);
    }

    /// Retry rule of 3.4: same command, same attempt; the timer is re-armed with the same generation.
    fn pc_retry(&mut self, p: PadId, d: DroneId, code: u16) {
        let state = self.visit(p, d).state;
        self.log_transition(code, p, d, state.code(), state.code());
        let retries = self.visit(p, d).retries + 1;
        match state {
            VisitState::Releasing => self.send_release_cmd(p, d),
            VisitState::Latching => self.send_latch_cmd(p, d),
            VisitState::Hopping => self.send_hop(p, d),
            _ => self.send_depart_clear(p, d),
        }
        self.visit_mut(p, d).retries = retries;
    }

    fn send_hop(&mut self, p: PadId, d: DroneId) {
        let ms = ms_from_s(self.cfg.drone.hop_s) + self.cfg.handshake.response_timeout_ms;
        self.pc_timer_arm(p, d, TimerKind::PcResponseTimeout, ms);
        let out_index = self.visit(p, d).hop_out;
        self.send(Msg::HopCmd {
            pad: p,
            drone: d,
            out_index,
        });
    }

    fn send_depart_clear(&mut self, p: PadId, d: DroneId) {
        let move_s = if self.visit(p, d).depart_mission == MissionKind::Park {
            self.cfg.drone.hop_s
        } else {
            self.cfg.drone.takeoff_s
        };
        let ms = ms_from_s(move_s) + self.cfg.handshake.response_timeout_ms;
        self.pc_timer_arm(p, d, TimerKind::PcResponseTimeout, ms);
        self.send(Msg::DepartClear { pad: p, drone: d });
    }

    fn pc_verify_release(&mut self, p: PadId, d: DroneId) {
        let released = self.visit(p, d).latch_open && self.pad(p).in_pod != NO_POD;
        if released {
            // V18: delivery measurement point.
            let pod = self.pad(p).in_pod;
            self.visit_to(p, d, tcode::v(18), VisitState::WaitOut);
            self.visit_mut(p, d).out_wait_expired = false;
            self.send(Msg::PodDelivered {
                pad: p,
                pod,
                drone: d,
            });
            let ms = self.cfg.pad_ops.out_empty_wait_ms;
            self.pc_timer_arm(p, d, TimerKind::PcOutWaitTimeout, ms);
            self.try_hop(p, d);
        } else if self.visit(p, d).attempt <= self.cfg.pad_ops.release_max_retries {
            self.visit_mut(p, d).attempt += 1;
            self.visit_to(p, d, tcode::v(19), VisitState::Releasing);
            self.send_release_cmd(p, d);
        } else {
            // V20: fail-safe grounding; the drone keeps IN blocked, the pad goes out of service.
            let order = self.fm.pod_order(self.drones[usize::from(d)].pod);
            self.visit_close(p, d, tcode::v(20), VisitOutcome::DroneGrounded);
            self.alert(
                AlertCode::ReleaseFault,
                AlertSeverity::Critical,
                (EntityKind::Pad, p),
                p,
                d,
                order,
            );
            self.send_service_complete(p, d, ServiceOutcome::ReleaseFailed);
            self.pad_mut(p).permanent_oos = true;
            if self.pad(p).mode == PadMode::Online {
                self.go_out_of_service(p, true);
            }
            if let Some(r) = self.cfg.pad_ops.repair {
                // One REPAIR draw per grounding, uniform on mean +- spread (A69).
                let u = self.rng.repair[usize::from(p)].uniform();
                let repair_s = r.mean_s - r.spread_s + 2.0 * r.spread_s * u;
                self.after(ms_from_s(repair_s), Ev::RepairDone { pad: p, drone: d });
            }
        }
    }

    fn pc_verify_lock(&mut self, p: PadId, d: DroneId) {
        let locked = self.visit(p, d).locked && self.drones[usize::from(d)].pod != NO_POD;
        let soc = self.soc_f32(d);
        if locked {
            let pod = self.drones[usize::from(d)].pod;
            let k = self.visit_out(p, d);
            let o = self.out_mut(p, k);
            o.pickup_drone = NO_DRONE;
            o.pickup_order = NO_ORDER;
            self.send(Msg::PodLatched {
                pad: p,
                pod,
                drone: d,
            });
            if self.charge_needed(p, soc) {
                self.start_charging(p, d, tcode::v(31), ServiceOutcome::Normal);
            } else {
                self.visit_to(p, d, tcode::v(32), VisitState::Ready);
                self.send_service_complete(p, d, ServiceOutcome::Normal);
            }
        } else if self.visit(p, d).attempt <= self.cfg.pad_ops.latch_max_retries {
            self.visit_mut(p, d).attempt += 1;
            self.visit_to(p, d, tcode::v(29), VisitState::Latching);
            self.send_latch_cmd(p, d);
        } else {
            let k = self.visit_out(p, d);
            let order = self.out(p, k).pickup_order;
            let o = self.out_mut(p, k);
            o.pickup_drone = NO_DRONE;
            o.pickup_order = NO_ORDER;
            self.metrics.latch_abort(self.now);
            self.alert(
                AlertCode::LatchFault,
                AlertSeverity::Warning,
                (EntityKind::Pad, p),
                p,
                d,
                order,
            );
            if self.charge_needed(p, soc) {
                self.start_charging(p, d, tcode::v(30), ServiceOutcome::LatchFailed);
            } else {
                self.visit_to(p, d, tcode::v(30), VisitState::Ready);
                self.send_service_complete(p, d, ServiceOutcome::LatchFailed);
            }
        }
    }

    /// V21 guard and action: hop to the OUT spot with this drone's pickup pod, or, when the
    /// policy allows an empty hop, to a spot with no pod. Returns whether the hop was commanded.
    fn try_hop(&mut self, p: PadId, d: DroneId) -> bool {
        let empty_ok = self.cfg.pad_ops.out_empty_policy == OutEmptyPolicy::DepartEmpty
            || self.visit(p, d).out_wait_expired;
        let Some(k) = self.out_for(p, d, empty_ok, false) else {
            return false;
        };
        self.out_mut(p, k).reserved = d;
        self.visit_mut(p, d).hop_out = u8::try_from(k).expect("at most 3 OUT spots");
        self.visit_to(p, d, tcode::v(21), VisitState::Hopping);
        self.send_hop(p, d);
        true
    }

    /// Re-evaluate guards after any PC event: V21, held departures (V34), the landing queue
    /// (V04) and placement (P02), in that order.
    pub(crate) fn pad_tick(&mut self, p: PadId) {
        if self.pad(p).mode == PadMode::ControllerFault {
            return;
        }
        let waiting = self.pad(p).in_drone;
        if waiting != NO_DRONE
            && self.visit(p, waiting).is_open_at(p)
            && self.visit(p, waiting).state == VisitState::WaitOut
        {
            self.try_hop(p, waiting);
        }
        while self.pad(p).movements == 0 {
            let Some(d) = self.pad_mut(p).depart_queue.pop_front() else {
                break;
            };
            if self.visit(p, d).is_open_at(p) && self.visit(p, d).state == VisitState::Ready {
                self.visit_to(p, d, tcode::v(34), VisitState::Departing);
                self.send_depart_clear(p, d);
            }
        }
        self.run_landing_queue(p);
        self.try_placement(p);
    }

    fn run_landing_queue(&mut self, p: PadId) {
        while self.pad(p).mode == PadMode::Online {
            let Some(&d) = self.pad(p).landing_queue.front() else {
                break;
            };
            if !(self.visit(p, d).is_open_at(p) && self.visit(p, d).state == VisitState::Held) {
                self.pad_mut(p).landing_queue.pop_front();
                continue;
            }
            let carries = self.visit(p, d).arrived_with_pod != NO_POD;
            let Some(spot) = self.spot_for(p, d, carries) else {
                break;
            };
            self.pad_mut(p).landing_queue.pop_front();
            let held = self.now - self.visit(p, d).held_since;
            self.metrics.landing_hold(self.now, held);
            self.grant(p, d, spot, tcode::v(4));
        }
    }

    pub(crate) fn pc_shipment_request(&mut self, p: PadId, dest: PadId, deadline: TimeMs) {
        let stockout = self.pad(p).empty_pods == 0;
        self.metrics.shipment_request(self.now, stockout);
        let full = self
            .cfg
            .max_backlog_per_pad
            .is_some_and(|cap| self.pad(p).backlog.len() >= usize::from(cap));
        if full {
            self.metrics.request_rejected(self.now);
            self.log_transition(tcode::p(8), p, NO_DRONE, NO_STATE, NO_STATE);
            return;
        }
        let now = self.now;
        self.pad_mut(p).backlog.push_back(Shipment {
            requested: now,
            dest,
            deadline,
        });
        self.log_transition(tcode::p(1), p, NO_DRONE, NO_STATE, NO_STATE);
        // V39: only when no OUT spot could take the placement and a drone parked on one could
        // make room.
        let n = self.n_out(p);
        if (0..n).any(|k| self.out_drone_free(p, k) && self.out(p, k).pod == NO_POD) {
            return;
        }
        let parked = (0..n).find(|&k| {
            let od = self.out(p, k).drone;
            od != NO_DRONE
                && self.visit(p, od).is_open_at(p)
                && self.visit(p, od).state == VisitState::Ready
                && self.out(p, k).pickup_drone != od
        });
        if let Some(k) = parked {
            let out_drone = self.out(p, k).drone;
            let s = self.visit(p, out_drone).state.code();
            self.log_transition(tcode::v(39), p, out_drone, s, s);
            self.send(Msg::ShipmentPending { pad: p, out_drone });
        }
    }

    /// P02 on every OUT spot that can take a pod, lowest index first.
    fn try_placement(&mut self, p: PadId) {
        while self.pad(p).mode == PadMode::Online {
            let Some(k) = (0..self.n_out(p))
                .find(|&k| self.out_drone_free(p, k) && self.out(p, k).pod == NO_POD)
            else {
                return;
            };
            if !self.place_on(p, k) {
                return;
            }
        }
    }

    /// Start placing the next pod (backlog, else a return) on OUT spot `k`; false if none.
    fn place_on(&mut self, p: PadId, k: usize) -> bool {
        let s = self.pad(p);
        let placement = if !s.backlog.is_empty() && s.empty_pods > 0 {
            let ship = self
                .pad_mut(p)
                .backlog
                .pop_front()
                .expect("backlog not empty");
            let pod = self.lowest_unreserved_pod(p).expect("empty_pods > 0");
            let n = self.pad(p).empty_pods - 1;
            self.set_empty_pods(p, n);
            self.pod_mass_kg[pod as usize] = crate::world::narrow_f32(self.cfg.pod_gross_mass_kg);
            Placement {
                pod,
                kind: OrderKind::Shipment,
                dest: ship.dest,
                requested: ship.requested,
                deadline: ship.deadline,
            }
        } else if let Some((pod, dest, requested)) = self.pad_mut(p).returns.pop_front() {
            self.pc_return_dest[pod as usize] = NO_PAD;
            self.pod_mass_kg[pod as usize] = crate::world::narrow_f32(self.cfg.pod_empty_mass_kg);
            Placement {
                pod,
                kind: OrderKind::PodReturn,
                dest,
                requested,
                deadline: crate::ids::NO_TIME,
            }
        } else {
            return false;
        };
        self.pad_mut(p).inventory.retain(|&x| x != placement.pod);
        let o = self.out_mut(p, k);
        o.placing = true;
        o.placement = placement;
        o.pickup_drone = NO_DRONE;
        o.pickup_order = NO_ORDER;
        self.log_transition(tcode::p(2), p, NO_DRONE, NO_STATE, NO_STATE);
        let ms = self.cfg.pad_ops.pod_place_ms;
        self.after(
            ms,
            Ev::PodPlaceDone {
                pad: p,
                pod: placement.pod,
            },
        );
        self.pad_status(p);
        true
    }

    fn lowest_unreserved_pod(&self, p: PadId) -> Option<PodId> {
        self.pad(p)
            .inventory
            .iter()
            .copied()
            .filter(|&pod| self.pc_return_dest[pod as usize] == NO_PAD)
            .min()
    }

    pub(crate) fn pc_pod_place_done(&mut self, p: PadId, pod: PodId) {
        let k = (0..self.n_out(p))
            .find(|&k| self.out(p, k).placing && self.out(p, k).placement.pod == pod)
            .expect("a placement in progress for this pod");
        let o = self.out_mut(p, k);
        let pl = o.placement;
        o.placing = false;
        o.pod = pod;
        self.log_transition(tcode::p(3), p, NO_DRONE, NO_STATE, NO_STATE);
        #[allow(clippy::cast_possible_truncation)]
        let payload_kg = self.pod_mass_kg[pod as usize];
        self.send(Msg::PodPlaced {
            pad: p,
            pod,
            kind: pl.kind,
            dest: pl.dest,
            requested: pl.requested,
            deadline: pl.deadline,
            payload_kg,
            out_index: u8::try_from(k).expect("at most 3 OUT spots"),
        });
    }

    pub(crate) fn pc_pod_remove_done(&mut self, p: PadId, pod: PodId) {
        let s = self.pad_mut(p);
        s.in_pod = NO_POD;
        s.in_pod_removal_scheduled = false;
        s.inventory.push(pod);
        let n = s.empty_pods + 1;
        self.set_empty_pods(p, n);
        self.pod_mass_kg[pod as usize] = crate::world::narrow_f32(self.cfg.pod_empty_mass_kg);
        self.log_transition(tcode::p(5), p, NO_DRONE, NO_STATE, NO_STATE);
        self.send(Msg::PodRemoved { pad: p, pod });
    }

    fn pc_return_request(&mut self, p: PadId, pod: PodId, dest: PadId) {
        let in_inventory = self.pad(p).inventory.contains(&pod);
        if !in_inventory || self.pc_return_dest[pod as usize] != NO_PAD {
            return;
        }
        self.pc_return_dest[pod as usize] = dest;
        let n = self.pad(p).empty_pods - 1;
        self.set_empty_pods(p, n);
        let now = self.now;
        self.pad_mut(p).returns.push_back((pod, dest, now));
        self.log_transition(tcode::p(6), p, NO_DRONE, NO_STATE, NO_STATE);
        self.pad_status(p);
    }

    fn set_mode(&mut self, p: PadId, code: u16, mode: PadMode) {
        let from = self.pad(p).mode;
        let now = self.now;
        let s = self.pad_mut(p);
        s.mode = mode;
        s.mode_since = now;
        self.log_transition(code, p, NO_DRONE, from.code(), mode.code());
    }

    /// M03 (from a scheduled fault, or from V20 when `grounded`).
    fn go_out_of_service(&mut self, p: PadId, grounded: bool) {
        self.set_mode(p, tcode::m(3), PadMode::OutOfService);
        while let Some(d) = self.pad_mut(p).landing_queue.pop_front() {
            if self.visit(p, d).is_open_at(p) && self.visit(p, d).state == VisitState::Held {
                self.visit_close(p, d, tcode::v(0), VisitOutcome::Refused);
                self.send(Msg::LandingRefused {
                    pad: p,
                    drone: d,
                    reason: HoldReason::OutOfService,
                });
            }
        }
        self.pad_status(p);
        if grounded {
            self.alert(
                AlertCode::PadOutOfService,
                AlertSeverity::Critical,
                (EntityKind::Pad, p),
                p,
                NO_DRONE,
                NO_ORDER,
            );
        }
    }

    pub(crate) fn fault_start(&mut self, kind: FaultKind, p: PadId, random: bool) {
        let mode = self.pad(p).mode;
        match kind {
            FaultKind::PcFault => {
                if random {
                    self.schedule_next_random_fault(p);
                }
                if mode != PadMode::Online {
                    // M05: suppressed; its draw was still consumed.
                    self.log_transition(tcode::m(5), p, NO_DRONE, mode.code(), mode.code());
                    return;
                }
                let duration = if random {
                    self.cfg.pad_ops.pc_fault_duration_ms
                } else {
                    let now = self.now;
                    self.cfg
                        .faults
                        .iter()
                        .find(|f| f.pad == p && f.kind == kind && f.start_ms == now)
                        .map_or(0, |f| f.duration_ms)
                };
                self.pad_mut(p).fault_end = self.now + duration;
                if random {
                    self.after(duration, Ev::FaultEnd { kind, pad: p });
                }
                self.set_mode(p, tcode::m(1), PadMode::ControllerFault);
            }
            FaultKind::OutOfService => {
                self.pad_mut(p).oos_window = true;
                match mode {
                    PadMode::Online => self.go_out_of_service(p, false),
                    PadMode::ControllerFault => self.pad_mut(p).oos_pending = true,
                    PadMode::OutOfService => {}
                }
            }
            FaultKind::MarkerNotDetected => self.set_marker_fault(p, true),
            FaultKind::ReleaseFault => self.pad_mut(p).release_fault = true,
            FaultKind::LatchFault => self.pad_mut(p).latch_fault = true,
        }
    }

    pub(crate) fn fault_end(&mut self, kind: FaultKind, p: PadId) {
        match kind {
            FaultKind::PcFault => {
                if self.pad(p).mode == PadMode::ControllerFault && self.pad(p).fault_end == self.now
                {
                    self.pc_recover(p);
                }
            }
            FaultKind::OutOfService => {
                self.pad_mut(p).oos_pending = false;
                self.pad_mut(p).oos_window = false;
                if self.pad(p).mode == PadMode::OutOfService && !self.pad(p).permanent_oos {
                    self.set_mode(p, tcode::m(4), PadMode::Online);
                    self.pad_status(p);
                    self.pad_tick(p);
                }
            }
            FaultKind::MarkerNotDetected => self.set_marker_fault(p, false),
            FaultKind::ReleaseFault => self.pad_mut(p).release_fault = false,
            FaultKind::LatchFault => self.pad_mut(p).latch_fault = false,
        }
    }

    /// A pad that checks its own fiducial reports a marker fault as it starts and ends (A35). A
    /// PC in CONTROLLER_FAULT is silent; its recovery status (M02) carries the flag instead.
    fn set_marker_fault(&mut self, p: PadId, faulted: bool) {
        self.pad_mut(p).marker_fault = faulted;
        if self.cfg.pad_ops.marker_self_check && self.pad(p).mode != PadMode::ControllerFault {
            self.pad_status(p);
        }
    }

    /// REPAIR_DONE (A69): the technician takes the stuck pod off the grounded drone into the
    /// pad's inventory (P09), the pad comes back (M06) unless a scheduled out-of-service window is
    /// still open, and the drone, empty on IN, reports its landing again (D25), which the PC
    /// adopts as V42.
    pub(crate) fn repair_done(&mut self, p: PadId, d: DroneId) {
        let pod = self.drones[usize::from(d)].pod;
        if pod != NO_POD {
            let dr = &mut self.drones[usize::from(d)];
            dr.pod = NO_POD;
            dr.payload_kg = 0.0;
            self.pod_mass_kg[pod as usize] = crate::world::narrow_f32(self.cfg.pod_empty_mass_kg);
            self.pad_mut(p).inventory.push(pod);
            let n = self.pad(p).empty_pods + 1;
            self.set_empty_pods(p, n);
            self.log_transition(tcode::p(9), p, d, NO_STATE, NO_STATE);
            self.send(Msg::PodRemoved { pad: p, pod });
        }
        // Only the PC's record goes: the drone never left IN, so its occupancy metric stays set
        // and the presence sensor keeps IN from being granted until the drone is adopted.
        self.pad_mut(p).in_drone = NO_DRONE;
        self.pad_mut(p).permanent_oos = false;
        if self.pad(p).mode == PadMode::OutOfService && !self.pad(p).oos_window {
            self.set_mode(p, tcode::m(6), PadMode::Online);
            self.pad_status(p);
        }
        self.drone_repaired(d, p);
        self.pad_tick(p);
    }

    /// M02: held events in order, re-send each open visit's command, then status and guards.
    fn pc_recover(&mut self, p: PadId) {
        self.pad_mut(p).fault_end = crate::ids::NO_TIME;
        self.set_mode(p, tcode::m(2), PadMode::Online);
        self.release_deferred(p);
        for d in 0..self.drones.len() {
            let id = DroneId::try_from(d).expect("fleet size fits u16");
            if self.visit(p, id).is_open_at(p)
                && let Some(cmd) = self.current_command(p, id)
            {
                self.send(cmd);
            }
        }
        self.pad_status(p);
        if self.pad(p).oos_pending {
            self.pad_mut(p).oos_pending = false;
            self.go_out_of_service(p, false);
        }
        self.pad_tick(p);
    }
}

fn s_dwell(end: TimeMs, start: TimeMs) -> f64 {
    crate::ids::s_from_ms(end.saturating_sub(start))
}
