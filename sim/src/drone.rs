//! Drone agent (spec 4.4): flight, fiducial landing, own latch, battery, handshake retries and
//! the ground watchdog (4.4.1). Row IDs D01 to D24 are logged as transition records.

use crate::event::{Ev, Msg, tcode};
use crate::ids::{
    AlertCode, AlertSeverity, DroneId, DroneMode, EntityKind, FlightSource, HoldReason,
    MissionKind, NO_LANE, NO_ORDER, NO_PAD, NO_POD, OrderId, PadId, ServiceOutcome, Spot, TimeMs,
    TimerKind, ms_from_s, s_from_ms,
};
use crate::world::{Await, DroneSt, World};

impl World {
    fn dr(&self, d: DroneId) -> &DroneSt {
        &self.drones[usize::from(d)]
    }

    fn dr_mut(&mut self, d: DroneId) -> &mut DroneSt {
        &mut self.drones[usize::from(d)]
    }

    fn drone_to(&mut self, d: DroneId, code: u16, to: DroneMode) {
        let now = self.now;
        let dr = self.dr_mut(d);
        let (from, since, order) = (dr.mode, dr.mode_since, dr.order);
        dr.mode = to;
        dr.mode_since = now;
        let pad = dr.pad;
        self.log_transition(code, pad, d, from.code(), to.code());
        // Holding with a pod is the queue part of its order's door-to-door time. Measured on the
        // drone: the PC's HELD state can outlast the hold when a cancel message is lost.
        if from == DroneMode::Holding && order != NO_ORDER {
            self.fm.orders[order as usize].hold_ms += now - since;
        }
    }

    fn drain(&mut self, d: DroneId, joules: f64) {
        let cap = self.cfg.drone.battery_capacity_j;
        let dr = self.dr_mut(d);
        let soc = dr.soc64() - joules / cap;
        dr.set_soc(soc);
    }

    fn hover(&mut self, d: DroneId, seconds: f64) {
        let p = self.energy.model.hover_power_w(self.dr(d).payload64());
        self.drain(d, p * seconds);
    }

    /// Send to a PC and remember the message for D16 re-sends.
    fn drone_send_pc(&mut self, d: DroneId, msg: Msg) {
        let seq = self.send(msg);
        self.dr_mut(d).last_msg = Some((msg, seq));
    }

    fn status(&mut self, d: DroneId) {
        let dr = self.dr(d);
        #[allow(clippy::cast_possible_truncation)]
        let msg = Msg::DroneStatus {
            drone: d,
            mode: dr.mode,
            pad: dr.pad,
            dest: dr.dest,
            pod: dr.pod,
            soc: dr.soc,
        };
        self.send(msg);
    }

    fn arm_response(&mut self, d: DroneId, delay_ms: TimeMs, awaiting: Await) {
        let dr = self.dr_mut(d);
        dr.gen_response = dr.gen_response.wrapping_add(1);
        dr.awaiting = awaiting;
        let generation = dr.gen_response;
        self.timer(
            delay_ms,
            TimerKind::DResponseTimeout,
            EntityKind::Drone,
            d,
            d,
            generation,
        );
    }

    fn disarm_response(&mut self, d: DroneId) {
        let dr = self.dr_mut(d);
        dr.gen_response = dr.gen_response.wrapping_add(1);
        dr.awaiting = Await::None;
    }

    /// 4.4.1: after a ground exchange the PC may stay quiet for `quiet_ms`.
    fn ground_watchdog(&mut self, d: DroneId, quiet_ms: TimeMs) {
        let ms = quiet_ms + self.cfg.handshake.response_timeout_ms;
        let now = self.now;
        let dr = self.dr_mut(d);
        dr.retries = 0;
        if dr.silence_since == crate::ids::NO_TIME {
            dr.silence_since = now;
        }
        self.arm_response(d, ms, Await::Ground);
    }

    fn arm_hold_retry(&mut self, d: DroneId, delay_ms: TimeMs) {
        let dr = self.dr_mut(d);
        dr.gen_hold_retry = dr.gen_hold_retry.wrapping_add(1);
        let generation = dr.gen_hold_retry;
        self.timer(
            delay_ms,
            TimerKind::DHoldRetry,
            EntityKind::Drone,
            d,
            d,
            generation,
        );
    }

    fn enter_holding(&mut self, d: DroneId, code: u16, pad: PadId) {
        self.dr_mut(d).pad = pad;
        self.drone_to(d, code, DroneMode::Holding);
        let now = self.now;
        let p_hover = self.energy.model.hover_power_w(self.dr(d).payload64());
        let dr = self.dr_mut(d);
        dr.hold_since = now;
        dr.gen_reserve = dr.gen_reserve.wrapping_add(1);
        let generation = dr.gen_reserve;
        let usable_j = ((dr.soc64() - self.cfg.drone.reserve_soc)
            * self.cfg.drone.battery_capacity_j)
            .max(0.0);
        self.timer(
            ms_from_s(usable_j / p_hover),
            TimerKind::DEnergyReserve,
            EntityKind::Drone,
            d,
            d,
            generation,
        );
    }

    fn leave_holding(&mut self, d: DroneId) {
        let held_s = s_from_ms(self.now - self.dr(d).hold_since);
        self.hover(d, held_s);
        let dr = self.dr_mut(d);
        dr.gen_reserve = dr.gen_reserve.wrapping_add(1);
        dr.gen_hold_retry = dr.gen_hold_retry.wrapping_add(1);
        dr.last_hold_report = None;
    }

    fn landing_request(&mut self, d: DroneId) {
        let dr = self.dr(d);
        #[allow(clippy::cast_possible_truncation)]
        let msg = Msg::LandingRequest {
            drone: d,
            pad: dr.pad,
            pod: dr.pod,
            soc: dr.soc,
        };
        self.drone_send_pc(d, msg);
        self.dr_mut(d).retries = 0;
        let ms = self.cfg.handshake.response_timeout_ms;
        self.arm_response(d, ms, Await::LandingReply);
    }

    fn hold_report(&mut self, d: DroneId, reason: HoldReason) {
        let dr = self.dr_mut(d);
        dr.last_hold_report = Some(reason);
        #[allow(clippy::cast_possible_truncation)]
        let msg = Msg::HoldReport {
            drone: d,
            pad: dr.pad,
            reason,
            hold_cycles: dr.hold_cycles,
            soc: dr.soc,
        };
        self.send(msg);
    }

    fn start_lane(&mut self, d: DroneId, from: PadId) {
        let dest = self.dr(d).dest;
        let lane = self.net.next_lane(from, dest);
        let len = f64::from(self.net.lane_len_m[lane as usize]);
        let (time_s, _) = self.energy.model.segment(len, self.dr(d).payload64());
        let now = self.now;
        #[allow(clippy::cast_possible_truncation)]
        let soc = self.dr(d).soc;
        let dr = self.dr_mut(d);
        dr.lane = lane;
        dr.lane_start = now;
        dr.lane_soc_start = soc;
        dr.pad = NO_PAD;
        self.after(ms_from_s(time_s), Ev::LaneDone { drone: d, lane });
    }

    pub(crate) fn drone_message(&mut self, d: DroneId, msg: Msg) {
        if self.dr(d).mode == DroneMode::Grounded {
            return;
        }
        if msg.sender().0 == EntityKind::Pad {
            let dr = self.dr_mut(d);
            dr.silence_since = crate::ids::NO_TIME;
            dr.silence_alert_armed = false;
            dr.gen_silence = dr.gen_silence.wrapping_add(1);
        }
        let mode = self.dr(d).mode;
        match msg {
            Msg::LandingGrant { pad, spot, .. }
                if mode == DroneMode::Holding && self.dr(d).pad == pad =>
            {
                self.leave_holding(d);
                self.disarm_response(d);
                self.dr_mut(d).approach_spot = spot;
                self.drone_to(d, tcode::d(4), DroneMode::Approach);
                self.after(
                    ms_from_s(self.cfg.drone.approach_s),
                    Ev::ApproachDone {
                        drone: d,
                        pad,
                        spot,
                    },
                );
            }
            Msg::LandingHold { pad, .. } if mode == DroneMode::Holding && self.dr(d).pad == pad => {
                self.disarm_response(d);
                self.drone_to(d, tcode::d(7), DroneMode::Holding);
                if self.dr(d).last_hold_report != Some(HoldReason::InOccupied) {
                    self.hold_report(d, HoldReason::InOccupied);
                }
                let ms = self.cfg.handshake.hold_retry_ms;
                self.arm_hold_retry(d, ms);
            }
            Msg::LandingRefused { pad, .. }
                if mode == DroneMode::Holding && self.dr(d).pad == pad =>
            {
                self.disarm_response(d);
                self.drone_to(d, tcode::d(11), DroneMode::Holding);
                if self.dr(d).last_hold_report != Some(HoldReason::OutOfService) {
                    self.hold_report(d, HoldReason::OutOfService);
                }
            }
            Msg::ReleaseCmd { pad, attempt, .. }
                if mode == DroneMode::OnPad && self.dr(d).spot == Some(Spot::In) =>
            {
                self.disarm_response(d);
                if attempt <= self.dr(d).done_release_attempt {
                    self.resend_report(d);
                } else if !self.dr(d).busy && self.dr(d).pod != NO_POD {
                    self.drone_to(d, tcode::d(15), DroneMode::OnPad);
                    let dr = self.dr_mut(d);
                    dr.busy = true;
                    dr.cmd_attempt = attempt;
                    let ms = self.service_ms(
                        pad,
                        self.cfg.pad_ops.manual_release_ms,
                        self.cfg.pad_ops.release_ms,
                    );
                    self.after(ms, Ev::ReleaseDone { drone: d, pad });
                }
            }
            Msg::HopCmd { pad, .. } if mode == DroneMode::OnPad => {
                self.disarm_response(d);
                match self.dr(d).spot {
                    Some(Spot::In) => {
                        self.drone_to(d, tcode::d(15), DroneMode::Hopping);
                        self.after(
                            ms_from_s(self.cfg.drone.hop_s),
                            Ev::HopDone { drone: d, pad },
                        );
                    }
                    Some(Spot::Out) => self.resend_report(d),
                    _ => {}
                }
            }
            Msg::LatchCmd { pad, attempt, .. }
                if mode == DroneMode::OnPad && self.dr(d).spot == Some(Spot::Out) =>
            {
                self.disarm_response(d);
                if attempt <= self.dr(d).done_latch_attempt {
                    self.resend_report(d);
                } else if !self.dr(d).busy {
                    self.drone_to(d, tcode::d(15), DroneMode::OnPad);
                    let dr = self.dr_mut(d);
                    dr.busy = true;
                    dr.cmd_attempt = attempt;
                    let ms = self.service_ms(
                        pad,
                        self.cfg.pad_ops.manual_latch_ms,
                        self.cfg.pad_ops.latch_ms,
                    );
                    self.after(ms, Ev::LatchDone { drone: d, pad });
                }
            }
            Msg::ChargeStart { dwell_s, .. } if mode == DroneMode::OnPad => {
                self.drone_to(d, tcode::d(15), DroneMode::OnPad);
                self.ground_watchdog(d, ms_from_s(f64::from(dwell_s)));
            }
            Msg::ServiceComplete { outcome, .. }
                if mode == DroneMode::OnPad && !self.dr(d).service_done =>
            {
                self.disarm_response(d);
                self.dr_mut(d).service_done = true;
                if outcome == ServiceOutcome::ReleaseFailed {
                    self.drone_to(d, tcode::d(13), DroneMode::Grounded);
                    self.status(d);
                    return;
                }
                self.drone_to(d, tcode::d(17), DroneMode::OnPad);
                let dr = self.dr(d);
                #[allow(clippy::cast_possible_truncation)]
                let msg = Msg::MissionRequest {
                    drone: d,
                    pad: dr.pad,
                    spot: dr.spot.unwrap_or(Spot::In),
                    pod: dr.pod,
                    soc: dr.soc,
                    outcome,
                };
                self.send(msg);
            }
            Msg::DepartClear { pad, .. }
                if matches!(mode, DroneMode::OnPad | DroneMode::Parked)
                    && self.dr(d).pad == pad =>
            {
                self.disarm_response(d);
                self.apply_parked_charge(d);
                self.drone_to(d, tcode::d(1), DroneMode::Takeoff);
                let move_s = if self.dr(d).mission == MissionKind::Park {
                    self.cfg.drone.hop_s
                } else {
                    self.cfg.drone.takeoff_s
                };
                self.after(ms_from_s(move_s), Ev::TakeoffDone { drone: d, pad });
            }
            Msg::MissionAssign {
                mission,
                dest,
                order,
                pickup_order,
                ..
            } => self.drone_mission(d, mission, dest, order, pickup_order),
            Msg::HoldContinue { .. } if mode == DroneMode::Holding => {
                self.drone_to(d, tcode::d(21), DroneMode::Holding);
                let Msg::HoldContinue { retry_after_s, .. } = msg else {
                    unreachable!()
                };
                self.arm_hold_retry(d, ms_from_s(f64::from(retry_after_s)));
            }
            _ => {}
        }
    }

    /// Send a ground report and arm the 4.4.1 watchdog for its quiet time. Every report goes
    /// through here so a re-sent report can never leave the drone without a watchdog.
    fn send_report(&mut self, d: DroneId, msg: Msg, quiet_ms: TimeMs) {
        self.drone_send_pc(d, msg);
        let dr = self.dr_mut(d);
        dr.last_report = Some(msg);
        dr.last_report_quiet_ms = quiet_ms;
        self.ground_watchdog(d, quiet_ms);
    }

    fn resend_report(&mut self, d: DroneId) {
        if let Some(report) = self.dr(d).last_report {
            let quiet = self.dr(d).last_report_quiet_ms;
            self.send_report(d, report, quiet);
        }
    }

    /// Parked charging (A16), applied when the drone leaves its parking place.
    fn apply_parked_charge(&mut self, d: DroneId) {
        let dr = self.dr(d);
        if dr.mode != DroneMode::Parked
            || !self.cfg.charging.charge_while_parked
            || !self.pads[usize::from(dr.pad)].has_charger
        {
            return;
        }
        let joules = s_from_ms(self.now - dr.park_since)
            * self.cfg.charging.power_w
            * self.cfg.charging.efficiency;
        self.add_charge(d, joules);
    }

    fn depart_request(&mut self, d: DroneId) {
        let dr = self.dr(d);
        let msg = Msg::DepartRequest {
            drone: d,
            pad: dr.pad,
            dest: dr.dest,
            mission: dr.mission,
        };
        self.drone_send_pc(d, msg);
        self.drone_to(d, tcode::d(19), self.dr(d).mode);
        let ms = ms_from_s(self.cfg.drone.takeoff_s + self.cfg.drone.hop_s);
        self.ground_watchdog(d, ms);
    }

    fn drone_mission(
        &mut self,
        d: DroneId,
        mission: MissionKind,
        dest: PadId,
        order: OrderId,
        pickup: OrderId,
    ) {
        let dr = *self.dr(d);
        let set = |w: &mut World| {
            let x = w.dr_mut(d);
            x.mission = mission;
            x.dest = dest;
            x.order = if order == NO_ORDER { x.order } else { order };
            x.pickup_order = pickup;
        };
        match dr.mode {
            DroneMode::OnPad | DroneMode::Parked => {
                if dr.mode == DroneMode::OnPad && !dr.service_done {
                    return;
                }
                set(self);
                if mission != MissionKind::Park {
                    self.depart_request(d);
                    return;
                }
                if self.pads[usize::from(dr.pad)].parking_capacity == 0 {
                    if dr.spot == Some(Spot::Out) && dr.mode == DroneMode::OnPad {
                        self.dr_mut(d).park_since = self.now;
                        self.drone_to(d, tcode::d(18), DroneMode::Parked);
                        self.status(d);
                    }
                } else if dr.spot != Some(Spot::Parking) {
                    self.depart_request(d);
                }
            }
            DroneMode::EnRoute => {
                set(self);
                let code = if dest == dr.dest {
                    tcode::d(22)
                } else {
                    tcode::d(24)
                };
                self.drone_to(d, code, DroneMode::EnRoute);
            }
            DroneMode::Holding if dest == dr.pad => {
                set(self);
                self.drone_to(d, tcode::d(22), DroneMode::Holding);
            }
            DroneMode::Holding => {
                // D12
                let old = dr.pad;
                self.leave_holding(d);
                self.disarm_response(d);
                set(self);
                self.dr_mut(d).hold_cycles = 0;
                self.drone_send_pc(d, Msg::LandingCancel { drone: d, pad: old });
                self.drone_to(d, tcode::d(12), DroneMode::EnRoute);
                self.start_lane(d, old);
                self.status(d);
            }
            DroneMode::Takeoff => {
                self.dr_mut(d).pending = Some((mission, dest, order, pickup));
                self.drone_to(d, tcode::d(24), DroneMode::Takeoff);
            }
            DroneMode::Approach | DroneMode::GoAround | DroneMode::Hopping => {
                if dest == dr.pad {
                    set(self);
                } else {
                    self.drone_to(d, tcode::d(24), dr.mode);
                }
            }
            DroneMode::Grounded => {}
        }
    }

    pub(crate) fn drone_completion(&mut self, ev: Ev) {
        match ev {
            Ev::TakeoffDone { drone, pad } => self.takeoff_done(drone, pad),
            Ev::LaneDone { drone, lane } => self.lane_done(drone, lane),
            Ev::ApproachDone { drone, pad, spot } => self.approach_done(drone, pad, spot),
            Ev::GoAroundDone { drone, pad } => {
                self.hover(drone, self.cfg.drone.go_around_s);
                self.drone_to(drone, tcode::d(6), DroneMode::Approach);
                let spot = self.dr(drone).approach_spot;
                self.after(
                    ms_from_s(self.cfg.drone.approach_s),
                    Ev::ApproachDone { drone, pad, spot },
                );
            }
            Ev::HopDone { drone, pad } => {
                self.hover(drone, self.cfg.drone.hop_s);
                self.dr_mut(drone).spot = Some(Spot::Out);
                self.drone_to(drone, tcode::d(15), DroneMode::OnPad);
                self.landed(drone, pad, Spot::Out);
            }
            Ev::ReleaseDone { drone, pad } => self.release_done(drone, pad),
            Ev::LatchDone { drone, pad } => self.latch_done(drone, pad),
            _ => {}
        }
    }

    fn landed(&mut self, d: DroneId, pad: PadId, spot: Spot) {
        let dr = self.dr(d);
        #[allow(clippy::cast_possible_truncation)]
        let msg = Msg::Landed {
            drone: d,
            pad,
            spot,
            pod: dr.pod,
            soc: dr.soc,
        };
        let quiet = if spot == Spot::In && dr.pod == NO_POD {
            self.cfg.pad_ops.out_empty_wait_ms
        } else {
            0
        };
        self.send_report(d, msg, quiet);
    }

    fn takeoff_done(&mut self, d: DroneId, pad: PadId) {
        let parking = self.dr(d).mission == MissionKind::Park;
        self.hover(
            d,
            if parking {
                self.cfg.drone.hop_s
            } else {
                self.cfg.drone.takeoff_s
            },
        );
        let left = self.dr(d).spot.unwrap_or(Spot::In);
        self.drone_send_pc(
            d,
            Msg::Departed {
                drone: d,
                pad,
                spot: left,
            },
        );
        if parking {
            let now = self.now;
            let dr = self.dr_mut(d);
            dr.spot = Some(Spot::Parking);
            dr.park_since = now;
            self.drone_to(d, tcode::d(23), DroneMode::Parked);
            self.status(d);
            return;
        }
        let now = self.now;
        let dr = self.dr_mut(d);
        dr.spot = None;
        dr.left_pad = pad;
        dr.left_pad_at = now;
        dr.service_done = false;
        dr.hold_cycles = 0;
        if let Some((mission, dest, order, pickup)) = dr.pending.take() {
            dr.mission = mission;
            dr.dest = dest;
            dr.order = if order == NO_ORDER { dr.order } else { order };
            dr.pickup_order = pickup;
        }
        if self.dr(d).dest == pad {
            self.enter_holding(d, tcode::d(20), pad);
            self.landing_request(d);
        } else {
            self.drone_to(d, tcode::d(2), DroneMode::EnRoute);
            self.start_lane(d, pad);
            self.status(d);
        }
    }

    fn lane_done(&mut self, d: DroneId, lane: u32) {
        let len = f64::from(self.net.lane_len_m[lane as usize]);
        let payload = self.dr(d).payload64();
        let (_, energy_j) = self.energy.model.segment(len, payload);
        self.drain(d, energy_j);
        self.flight_record(d, lane, energy_j);
        let order = self.dr(d).order;
        if order != NO_ORDER {
            self.fm.orders[order as usize].flight_ms += self.now - self.dr(d).lane_start;
        }
        let end = self.net.lane_to[lane as usize];
        self.dr_mut(d).lane = NO_LANE;
        if end == self.dr(d).dest {
            self.enter_holding(d, tcode::d(3), end);
            self.landing_request(d);
        } else {
            self.drone_to(d, tcode::d(3), DroneMode::EnRoute);
            self.start_lane(d, end);
        }
    }

    fn flight_record(&mut self, d: DroneId, lane: u32, energy_j: f64) {
        let dr = self.dr(d);
        let mut rec = [0u8; 48];
        rec[0..8].copy_from_slice(&dr.lane_start.to_le_bytes());
        rec[8..16].copy_from_slice(&self.now.to_le_bytes());
        rec[16..20].copy_from_slice(&lane.to_le_bytes());
        rec[20..24].copy_from_slice(&dr.order.to_le_bytes());
        #[allow(clippy::cast_possible_truncation)]
        {
            rec[24..28].copy_from_slice(&dr.payload_kg.to_le_bytes());
            rec[28..32].copy_from_slice(&(energy_j as f32).to_le_bytes());
            rec[32..36].copy_from_slice(&dr.lane_soc_start.to_le_bytes());
            rec[36..40].copy_from_slice(&dr.soc.to_le_bytes());
        }
        rec[40..42].copy_from_slice(&d.to_le_bytes());
        rec[42..44].copy_from_slice(&self.net.lane_from[lane as usize].to_le_bytes());
        rec[44..46].copy_from_slice(&self.net.lane_to[lane as usize].to_le_bytes());
        rec[46] = FlightSource::Des.code();
        self.out.flight_record(&rec);
    }

    fn approach_done(&mut self, d: DroneId, pad: PadId, spot: Spot) {
        self.hover(d, self.cfg.drone.approach_s);
        let p = if self.pads[usize::from(pad)].marker_fault {
            0.0
        } else {
            self.cfg.pad_ops.p_marker_detect
        };
        let detected = self.rng.marker[usize::from(pad)].bernoulli(p);
        let go_around = !detected && self.dr(d).go_arounds < self.cfg.pad_ops.max_go_arounds;
        self.metrics.approach(self.now, go_around);
        if detected {
            let dr = self.dr_mut(d);
            dr.go_arounds = 0;
            dr.hold_cycles = 0;
            dr.spot = Some(spot);
            dr.busy = false;
            dr.done_release_attempt = 0;
            dr.done_latch_attempt = 0;
            dr.last_report = None;
            self.drone_to(d, tcode::d(5), DroneMode::OnPad);
            self.landed(d, pad, spot);
            self.status(d);
        } else if go_around {
            self.dr_mut(d).go_arounds += 1;
            self.drone_to(d, tcode::d(5), DroneMode::GoAround);
            self.after(
                ms_from_s(self.cfg.drone.go_around_s),
                Ev::GoAroundDone { drone: d, pad },
            );
        } else {
            let dr = self.dr_mut(d);
            dr.go_arounds = 0;
            dr.hold_cycles = dr.hold_cycles.saturating_add(1);
            self.enter_holding(d, tcode::d(5), pad);
            self.drone_send_pc(
                d,
                Msg::ApproachAbort {
                    drone: d,
                    pad,
                    reason: HoldReason::MarkerNotDetected,
                },
            );
            self.hold_report(d, HoldReason::MarkerNotDetected);
        }
    }

    fn release_done(&mut self, d: DroneId, pad: PadId) {
        let p_ok = if self.pads[usize::from(pad)].release_fault {
            0.0
        } else {
            1.0 - self.cfg.drone.p_release_fault
        };
        let open = self.rng.release[usize::from(d)].bernoulli(p_ok);
        let dr = self.dr_mut(d);
        dr.busy = false;
        dr.done_release_attempt = dr.cmd_attempt;
        if open && dr.pod != NO_POD {
            let pod = dr.pod;
            dr.pod = NO_POD;
            dr.payload_kg = 0.0;
            self.pads[usize::from(pad)].in_pod = pod;
        }
        let msg = Msg::ReleaseReport {
            drone: d,
            pad,
            latch_open: open,
        };
        self.drone_to(d, tcode::d(15), DroneMode::OnPad);
        let quiet = self.cfg.pad_ops.release_verify_ms + self.cfg.pad_ops.out_empty_wait_ms;
        self.send_report(d, msg, quiet);
    }

    fn latch_done(&mut self, d: DroneId, pad: PadId) {
        let p_ok = if self.pads[usize::from(pad)].latch_fault {
            0.0
        } else {
            1.0 - self.cfg.drone.p_latch_fault
        };
        let locked = self.rng.latch[usize::from(d)].bernoulli(p_ok);
        let out_pod = self.pads[usize::from(pad)].out_pod;
        let mass = if out_pod == NO_POD {
            0.0
        } else {
            self.pod_mass_kg[out_pod as usize]
        };
        let dr = self.dr_mut(d);
        dr.busy = false;
        dr.done_latch_attempt = dr.cmd_attempt;
        if locked && out_pod != NO_POD {
            dr.pod = out_pod;
            dr.payload_kg = mass;
            self.pads[usize::from(pad)].out_pod = NO_POD;
        }
        let msg = Msg::LatchReport {
            drone: d,
            pad,
            pod: out_pod,
            locked,
        };
        self.drone_to(d, tcode::d(15), DroneMode::OnPad);
        let quiet = self.cfg.pad_ops.latch_verify_ms;
        self.send_report(d, msg, quiet);
    }

    pub(crate) fn drone_timer(&mut self, d: DroneId, timer: TimerKind) {
        let mode = self.dr(d).mode;
        if mode == DroneMode::Grounded {
            return;
        }
        let max_retries = self.cfg.handshake.max_retries;
        match timer {
            TimerKind::DResponseTimeout => match self.dr(d).awaiting {
                crate::world::Await::LandingReply if mode == DroneMode::Holding => {
                    if self.dr(d).retries < max_retries {
                        self.dr_mut(d).retries += 1;
                        self.drone_to(d, tcode::d(9), DroneMode::Holding);
                        self.resend_last(d);
                        let ms = self.cfg.handshake.response_timeout_ms;
                        self.arm_response(d, ms, Await::LandingReply);
                    } else {
                        self.disarm_response(d);
                        self.drone_to(d, tcode::d(10), DroneMode::Holding);
                        if self.dr(d).last_hold_report != Some(HoldReason::PcSilent) {
                            self.hold_report(d, HoldReason::PcSilent);
                        }
                        let ms = self.cfg.handshake.hold_retry_ms;
                        self.arm_hold_retry(d, ms);
                    }
                }
                crate::world::Await::Ground => {
                    self.drone_to(d, tcode::d(16), mode);
                    if self.dr(d).silence_since == crate::ids::NO_TIME {
                        let now = self.now;
                        self.dr_mut(d).silence_since = now;
                    }
                    self.resend_last(d);
                    let retries = self.dr(d).retries.saturating_add(1);
                    self.dr_mut(d).retries = retries;
                    if retries > max_retries && !self.dr(d).silence_alert_armed {
                        let dr = self.dr_mut(d);
                        dr.silence_alert_armed = true;
                        dr.gen_silence = dr.gen_silence.wrapping_add(1);
                        let generation = dr.gen_silence;
                        let due = dr.silence_since + self.cfg.handshake.pc_silence_alert_ms;
                        self.timer(
                            due.saturating_sub(self.now),
                            TimerKind::DPcSilenceAlert,
                            EntityKind::Drone,
                            d,
                            d,
                            generation,
                        );
                    }
                    let ms = self.cfg.handshake.response_timeout_ms;
                    self.arm_response(d, ms, Await::Ground);
                }
                _ => {}
            },
            TimerKind::DHoldRetry if mode == DroneMode::Holding => {
                self.drone_to(d, tcode::d(8), DroneMode::Holding);
                self.landing_request(d);
            }
            TimerKind::DPcSilenceAlert => {
                let pad = self.dr(d).pad;
                self.alert(
                    AlertCode::PcSilent,
                    AlertSeverity::Warning,
                    (EntityKind::Drone, d),
                    pad,
                    d,
                    NO_ORDER,
                );
            }
            TimerKind::DEnergyReserve if mode == DroneMode::Holding => self.forced_landing(d),
            _ => {}
        }
    }

    fn resend_last(&mut self, d: DroneId) {
        if let Some((msg, seq)) = self.dr(d).last_msg {
            self.send_with_seq(msg, seq);
        }
    }

    /// D25: back in service after a repair (A69), empty and still on IN. The drone reports its
    /// landing again so the PC adopts it (V42) and the ground watchdog runs as after any landing.
    pub(crate) fn drone_repaired(&mut self, d: DroneId, pad: PadId) {
        let dr = self.dr_mut(d);
        dr.busy = false;
        dr.service_done = false;
        dr.done_release_attempt = 0;
        dr.done_latch_attempt = 0;
        dr.last_report = None;
        dr.order = NO_ORDER;
        dr.pickup_order = NO_ORDER;
        dr.mission = MissionKind::None;
        dr.go_arounds = 0;
        dr.hold_cycles = 0;
        self.drone_to(d, tcode::d(25), DroneMode::OnPad);
        self.landed(d, pad, Spot::In);
        self.status(d);
    }

    /// D14: reserve reached while holding; land off the network, pod lost.
    fn forced_landing(&mut self, d: DroneId) {
        let pad = self.dr(d).pad;
        self.leave_holding(d);
        self.disarm_response(d);
        self.drone_send_pc(d, Msg::LandingCancel { drone: d, pad });
        let order = self.dr(d).order;
        self.drone_to(d, tcode::d(14), DroneMode::Grounded);
        self.status(d);
        let dr = self.dr_mut(d);
        dr.pad = NO_PAD;
        dr.spot = None;
        self.alert(
            AlertCode::DroneGrounded,
            AlertSeverity::Critical,
            (EntityKind::Drone, d),
            pad,
            d,
            order,
        );
    }
}
