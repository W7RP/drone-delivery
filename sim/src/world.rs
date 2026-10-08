//! Engine core: all state as flat arrays indexed by ID, the event queue (spec 5), message
//! delivery (4.1), logging (9.4) and the main loop. Component logic lives in `pad.rs` (PC),
//! `drone.rs` (drone agent) and `fm.rs` (fleet manager), each as `impl World`.

use std::collections::{BinaryHeap, VecDeque};
use std::io;

use serde_json::Value;

use crate::config::Config;
use crate::event::{Args, Entry, Ev, LogStatus, Msg, NO_STATE, Receiver, TRANSITION_KIND};
use crate::flight::EnergyRules;
use crate::fm::Fm;
use crate::ids::{
    AlertCode, AlertSeverity, DroneId, DroneMode, EntityKind, HoldReason, LaneId, MissionKind,
    NO_DRONE, NO_LANE, NO_ORDER, NO_PAD, NO_POD, NO_TIME, OrderId, OrderKind, PadId, PadMode,
    PodId, ServiceMode, ServiceOutcome, Spot, TimeMs, TimerKind, VisitOutcome, VisitState,
    ms_from_s,
};
use crate::metrics::{CostInputs, Metrics};
use crate::network::Network;
use crate::output::Outputs;
use crate::rng::{Streams, master_seed};

/// An ID printed as JSON: the number, or `null` for its NONE value. Formats without allocating.
struct OptId(u64, u64);

impl std::fmt::Display for OptId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.0 == self.1 {
            f.write_str("null")
        } else {
            write!(f, "{}", self.0)
        }
    }
}

#[allow(clippy::cast_possible_truncation)]
pub fn narrow_f32(v: f64) -> f32 {
    v as f32
}

/// Queue capacity reserved up front: a few pending events per entity is the steady state.
const QUEUE_SLOTS_PER_ENTITY: usize = 16;
/// Initial backlog capacity per pad when `demand.max_backlog_per_pad` is absent; it grows as needed.
const DEFAULT_BACKLOG_CAPACITY: usize = 64;

#[derive(Clone, Copy, Debug)]
pub struct Shipment {
    pub requested: TimeMs,
    pub dest: PadId,
    pub deadline: TimeMs,
}

#[derive(Clone, Copy, Debug)]
pub struct Placement {
    pub pod: PodId,
    pub kind: OrderKind,
    pub dest: PadId,
    pub requested: TimeMs,
    pub deadline: TimeMs,
}

pub struct PadSt {
    pub mode: PadMode,
    pub mode_since: TimeMs,
    /// End time of the active PC fault; a FAULT_END at another time is ignored (A37).
    pub fault_end: TimeMs,
    pub oos_pending: bool,
    /// A scheduled OUT_OF_SERVICE window is open; a repair does not end it (A69).
    pub oos_window: bool,
    /// A drone grounded on IN keeps the pad out of service until it is repaired.
    pub permanent_oos: bool,
    pub has_charger: bool,
    pub service_mode: ServiceMode,
    pub parking_capacity: u16,
    pub in_drone: DroneId,
    pub in_reserved: DroneId,
    pub in_pod: PodId,
    pub in_pod_removal_scheduled: bool,
    pub out_drone: DroneId,
    pub out_reserved: DroneId,
    pub out_pod: PodId,
    pub out_placing: bool,
    pub placing: Placement,
    pub pickup_drone: DroneId,
    pub pickup_order: OrderId,
    /// Empty pods physically in inventory, reserved or not.
    pub inventory: Vec<PodId>,
    /// Unreserved count (`PadState.empty_pods`).
    pub empty_pods: u32,
    pub backlog: VecDeque<Shipment>,
    pub returns: VecDeque<(PodId, PadId, TimeMs)>,
    pub landing_queue: VecDeque<DroneId>,
    pub depart_queue: VecDeque<DroneId>,
    pub movements: u8,
    pub parked: u16,
    pub marker_fault: bool,
    pub release_fault: bool,
    pub latch_fault: bool,
    pub deferred: Vec<Entry>,
    pub msg_seq: u32,
}

/// The PC's record of one drone's pass through its pad, indexed by `visit_index(pad, drone)`.
#[derive(Clone, Copy, Debug)]
pub struct Visit {
    pub pad: PadId,
    pub state: VisitState,
    pub spot: Spot,
    pub attempt: u8,
    pub retries: u8,
    pub generation: u32,
    pub since: TimeMs,
    pub outcome: VisitOutcome,
    pub arrived_with_pod: PodId,
    pub soc: f32,
    pub silent_from: VisitState,
    pub out_wait_expired: bool,
    pub depart_mission: MissionKind,
    pub carried: ServiceOutcome,
    pub charge_end: TimeMs,
    pub latch_open: bool,
    pub locked: bool,
    pub service_sent: bool,
    pub held_since: TimeMs,
}

impl Visit {
    pub fn closed() -> Visit {
        Visit {
            pad: NO_PAD,
            state: VisitState::Closed,
            spot: Spot::In,
            attempt: 0,
            retries: 0,
            generation: 0,
            since: 0,
            outcome: VisitOutcome::Open,
            arrived_with_pod: NO_POD,
            soc: 0.0,
            silent_from: VisitState::Closed,
            out_wait_expired: false,
            depart_mission: MissionKind::None,
            carried: ServiceOutcome::NoPod,
            charge_end: 0,
            latch_open: false,
            locked: false,
            service_sent: false,
            held_since: 0,
        }
    }

    pub fn is_open_at(&self, pad: PadId) -> bool {
        self.pad == pad && self.state != VisitState::Closed
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Await {
    None,
    LandingReply,
    Ground,
}

#[derive(Clone, Copy, Debug)]
pub struct DroneSt {
    pub mode: DroneMode,
    pub mode_since: TimeMs,
    /// Pad the drone is on, over, or holding at; NO_PAD en route or off-network.
    pub pad: PadId,
    /// Physical spot when on a pad (sensor truth).
    pub spot: Option<Spot>,
    pub mission: MissionKind,
    pub dest: PadId,
    pub pending: Option<(MissionKind, PadId, OrderId, OrderId)>,
    pub lane: LaneId,
    pub lane_start: TimeMs,
    pub lane_soc_start: f32,
    pub pod: PodId,
    pub order: OrderId,
    pub pickup_order: OrderId,
    pub soc: f32,
    pub payload_kg: f32,
    pub go_arounds: u8,
    pub hold_cycles: u8,
    pub energy_charged_j: f64,
    pub hold_since: TimeMs,
    pub park_since: TimeMs,
    pub last_hold_report: Option<HoldReason>,
    pub msg_seq: u32,
    pub last_msg: Option<(Msg, u32, u32)>,
    pub awaiting: Await,
    pub retries: u8,
    pub silence_since: TimeMs,
    pub silence_alert_armed: bool,
    pub gen_response: u32,
    pub gen_hold_retry: u32,
    pub gen_silence: u32,
    pub gen_reserve: u32,
    pub busy: bool,
    pub done_release_attempt: u8,
    pub done_latch_attempt: u8,
    pub last_report: Option<Msg>,
    pub last_report_quiet_ms: TimeMs,
    pub service_done: bool,
    pub approach_spot: Spot,
    pub cmd_attempt: u8,
    /// Pad and time of the last physical departure, for the PC-occupancy invariant.
    pub left_pad: PadId,
    pub left_pad_at: TimeMs,
}

impl DroneSt {
    /// Stored as f32 (spec 0); arithmetic is done in f64 and narrowed once, here.
    pub fn soc64(&self) -> f64 {
        f64::from(self.soc)
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn set_soc(&mut self, soc: f64) {
        self.soc = soc as f32;
    }

    pub fn payload64(&self) -> f64 {
        f64::from(self.payload_kg)
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn set_payload(&mut self, kg: f64) {
        self.payload_kg = kg as f32;
    }
}

/// Spec 4.1 reordering on the D <-> PC radio link: per-link sequence numbers, the next one
/// each receiver expects, and the messages it holds while an earlier one is outstanding.
/// Link index: drone -> pad `d * n_pads + p`, pad -> drone `n_pads * n_drones + p * n_drones + d`.
pub struct Links {
    pub last_sent: Vec<u32>,
    pub expected: Vec<u32>,
    pub held: Vec<(usize, u32, Entry)>,
    /// Fault injection: (sender, message kind code, extra latency) for the next such message.
    pub injected: Vec<((EntityKind, u16), u16, TimeMs)>,
}

pub struct World {
    pub cfg: Config,
    pub net: Network,
    pub energy: EnergyRules,
    pub replication: u32,
    pub now: TimeMs,
    seq: u64,
    queue: BinaryHeap<Entry>,
    pub pads: Vec<PadSt>,
    pub visits: Vec<Visit>,
    pub drones: Vec<DroneSt>,
    pub fm: Fm,
    pub metrics: Metrics,
    pub rng: Streams,
    pub out: Outputs,
    /// Physical mass of each pod (gross while loaded, empty otherwise).
    pub pod_mass_kg: Vec<f32>,
    /// PC-side return reservation per pod (P06), NO_PAD when unreserved.
    pub pc_return_dest: Vec<PadId>,
    pub links: Links,
    pub ended: bool,
    #[cfg(debug_assertions)]
    scratch: Vec<u8>,
}

impl World {
    pub fn new(cfg: Config, net: Network, replication: u32, out: Outputs) -> World {
        let n_pads = cfg.pads.len();
        let n_drones = usize::from(cfg.drone.fleet_size);
        let master = master_seed(cfg.seed, replication);
        let rng = Streams::new(master, cfg.flows.len(), n_pads, n_drones);
        let energy = EnergyRules::from_config(&cfg);
        let metrics = Metrics::new(
            (cfg.warmup_ms, cfg.duration_ms),
            cfg.hist_bin_ms,
            cfg.hist_max_ms,
            n_pads,
            n_drones,
        );
        let total_pods = cfg.total_pods() as usize;
        let mut pads = Vec::with_capacity(n_pads);
        let mut pod_home = Vec::with_capacity(total_pods);
        for (i, p) in cfg.pads.iter().enumerate() {
            let pad = PadId::try_from(i).expect("validated pad count");
            let first = PodId::try_from(pod_home.len()).expect("pod count fits u32");
            let inventory: Vec<PodId> = (first..first + p.initial_empty_pods).collect();
            pod_home.extend(std::iter::repeat_n(pad, inventory.len()));
            let mut inv = Vec::with_capacity(total_pods);
            inv.extend_from_slice(&inventory);
            pads.push(PadSt {
                mode: PadMode::Online,
                mode_since: 0,
                fault_end: NO_TIME,
                oos_pending: false,
                oos_window: false,
                permanent_oos: false,
                has_charger: cfg.has_charger(pad),
                service_mode: p.service_mode,
                parking_capacity: p.parking_capacity,
                in_drone: NO_DRONE,
                in_reserved: NO_DRONE,
                in_pod: NO_POD,
                in_pod_removal_scheduled: false,
                out_drone: NO_DRONE,
                out_reserved: NO_DRONE,
                out_pod: NO_POD,
                out_placing: false,
                placing: Placement {
                    pod: NO_POD,
                    kind: OrderKind::Shipment,
                    dest: NO_PAD,
                    requested: 0,
                    deadline: 0,
                },
                pickup_drone: NO_DRONE,
                pickup_order: NO_ORDER,
                inventory: inv,
                empty_pods: p.initial_empty_pods,
                // A capped backlog never grows past its cap, so it never reallocates.
                backlog: VecDeque::with_capacity(
                    cfg.max_backlog_per_pad
                        .map_or(DEFAULT_BACKLOG_CAPACITY, usize::from),
                ),
                returns: VecDeque::with_capacity(16),
                landing_queue: VecDeque::with_capacity(n_drones),
                depart_queue: VecDeque::with_capacity(n_drones),
                movements: 0,
                parked: 0,
                marker_fault: false,
                release_fault: false,
                latch_fault: false,
                deferred: Vec::with_capacity(256),
                msg_seq: 0,
            });
        }
        let drones = vec![
            DroneSt {
                mode: DroneMode::Parked,
                mode_since: 0,
                pad: NO_PAD,
                spot: None,
                mission: MissionKind::Park,
                dest: NO_PAD,
                pending: None,
                lane: NO_LANE,
                lane_start: 0,
                lane_soc_start: 0.0,
                pod: NO_POD,
                order: NO_ORDER,
                pickup_order: NO_ORDER,
                soc: 0.0,
                payload_kg: 0.0,
                go_arounds: 0,
                hold_cycles: 0,
                energy_charged_j: 0.0,
                hold_since: 0,
                park_since: 0,
                last_hold_report: None,
                msg_seq: 0,
                last_msg: None,
                awaiting: Await::None,
                retries: 0,
                silence_since: NO_TIME,
                silence_alert_armed: false,
                gen_response: 0,
                gen_hold_retry: 0,
                gen_silence: 0,
                gen_reserve: 0,
                busy: false,
                done_release_attempt: 0,
                done_latch_attempt: 0,
                last_report: None,
                last_report_quiet_ms: 0,
                service_done: false,
                approach_spot: Spot::In,
                cmd_attempt: 0,
                left_pad: NO_PAD,
                left_pad_at: 0,
            };
            n_drones
        ];
        let fm = Fm::new(&cfg, &pod_home);
        let entities = n_pads + n_drones + cfg.flows.len();
        let mut world = World {
            pod_mass_kg: vec![narrow_f32(cfg.pod_empty_mass_kg); total_pods],
            pc_return_dest: vec![NO_PAD; total_pods],
            links: Links {
                last_sent: vec![0; 2 * n_pads * n_drones],
                expected: vec![1; 2 * n_pads * n_drones],
                held: Vec::new(),
                injected: Vec::new(),
            },
            energy,
            replication,
            now: 0,
            seq: 0,
            queue: BinaryHeap::with_capacity(entities * QUEUE_SLOTS_PER_ENTITY),
            pads,
            visits: vec![Visit::closed(); n_pads * n_drones],
            drones,
            fm,
            metrics,
            rng,
            out,
            ended: false,
            #[cfg(debug_assertions)]
            scratch: vec![0; total_pods.max(n_pads * 3)],
            net,
            cfg,
        };
        for d in &mut world.drones {
            d.set_soc(world.cfg.drone.initial_soc);
        }
        world.place_initial_drones();
        world.schedule_initial_events();
        world
    }

    fn place_initial_drones(&mut self) {
        for d in 0..self.drones.len() {
            let id = DroneId::try_from(d).expect("validated fleet size");
            let pad = self.cfg.drone.initial_pads[d];
            let p = usize::from(pad);
            let drone = &mut self.drones[d];
            drone.pad = pad;
            drone.dest = pad;
            if self.pads[p].parking_capacity > 0 {
                drone.spot = Some(Spot::Parking);
                self.pads[p].parked += 1;
            } else {
                drone.spot = Some(Spot::Out);
                self.pads[p].out_drone = id;
                self.metrics.spot_occupied(p, false, true, 0);
                let mut v = Visit::closed();
                v.pad = pad;
                v.state = VisitState::Ready;
                v.spot = Spot::Out;
                v.outcome = VisitOutcome::Open;
                v.soc = self.drones[d].soc;
                let i = self.visit_index(pad, id);
                self.visits[i] = v;
            }
            self.fm
                .init_drone(id, pad, self.drones[d].spot, self.drones[d].soc);
        }
        for p in 0..self.pads.len() {
            let available = self.pads[p].empty_pods > 0;
            self.metrics.pods_available(p, available, 0);
        }
    }

    fn schedule_initial_events(&mut self) {
        let faults = self.cfg.faults.clone();
        for f in &faults {
            self.schedule(
                f.start_ms,
                Ev::FaultStart {
                    kind: f.kind,
                    pad: f.pad,
                    random: false,
                },
            );
            self.schedule(
                f.start_ms + f.duration_ms,
                Ev::FaultEnd {
                    kind: f.kind,
                    pad: f.pad,
                },
            );
        }
        let end = self.cfg.duration_ms;
        self.schedule(
            end,
            Ev::Timer {
                timer: TimerKind::EndOfRun,
                owner_kind: EntityKind::FleetManager,
                owner_id: 0,
                subject: NO_DRONE,
                generation: 0,
            },
        );
        for flow in 0..self.cfg.flows.len() {
            self.schedule_next_shipment(u32::try_from(flow).expect("flow count fits u32"));
        }
        for p in 0..self.pads.len() {
            self.schedule_next_random_fault(PadId::try_from(p).expect("validated pad count"));
        }
    }

    pub fn schedule_next_shipment(&mut self, flow: u32) {
        let f = self.cfg.flows[flow as usize];
        if f.rate_per_s <= 0.0 {
            return;
        }
        let t = self.now + self.rng.demand[flow as usize].exponential_ms(f.rate_per_s);
        let deadline = t + self.cfg.deadline_offset_ms;
        self.schedule(
            t,
            Ev::ShipmentRequest {
                flow,
                pad: f.origin,
                dest: f.dest,
                deadline,
            },
        );
    }

    pub fn schedule_next_random_fault(&mut self, pad: PadId) {
        let rate = self.cfg.pad_ops.pc_fault_rate_per_s;
        if rate > 0.0 {
            let t = self.now + self.rng.pc_fault[usize::from(pad)].exponential_ms(rate);
            self.schedule(
                t,
                Ev::FaultStart {
                    kind: crate::ids::FaultKind::PcFault,
                    pad,
                    random: true,
                },
            );
        }
    }

    pub fn schedule(&mut self, t: TimeMs, ev: Ev) {
        let entry = Entry {
            t,
            class: ev.class().code(),
            seq: self.seq,
            ev,
        };
        self.seq += 1;
        self.queue.push(entry);
    }

    pub fn after(&mut self, delay_ms: TimeMs, ev: Ev) {
        self.schedule(self.now + delay_ms, ev);
    }

    pub fn timer(
        &mut self,
        delay_ms: TimeMs,
        timer: TimerKind,
        owner_kind: EntityKind,
        owner_id: u16,
        subject: u16,
        generation: u32,
    ) {
        self.after(
            delay_ms,
            Ev::Timer {
                timer,
                owner_kind,
                owner_id,
                subject,
                generation,
            },
        );
    }

    fn next_seq(&mut self, sender: (EntityKind, u16)) -> u32 {
        let slot = match sender {
            (EntityKind::Pad, p) => &mut self.pads[usize::from(p)].msg_seq,
            (EntityKind::Drone, d) => &mut self.drones[usize::from(d)].msg_seq,
            (EntityKind::FleetManager, _) => &mut self.fm.msg_seq,
        };
        *slot += 1;
        *slot
    }

    /// Send a new message (fresh `msg_seq`, and on the radio link a fresh `link_seq`); returns
    /// both so the sender can re-send it unchanged.
    pub fn send(&mut self, msg: Msg) -> (u32, u32) {
        let seq = self.next_seq(msg.sender());
        let link_seq = match self.link_of(&msg) {
            Some(link) => {
                self.links.last_sent[link] += 1;
                self.links.last_sent[link]
            }
            None => 0,
        };
        self.send_with_seq(msg, seq, link_seq);
        (seq, link_seq)
    }

    /// The radio link a message travels on (spec 4.1), None for FM links.
    fn link_of(&self, msg: &Msg) -> Option<usize> {
        let (n_pads, n_drones) = (self.pads.len(), self.drones.len());
        match (msg.sender(), msg.receiver()) {
            ((EntityKind::Drone, d), Receiver::Pad(p)) => {
                Some(usize::from(d) * n_pads + usize::from(p))
            }
            ((EntityKind::Pad, p), Receiver::Drone(d)) => {
                Some(n_pads * n_drones + usize::from(p) * n_drones + usize::from(d))
            }
            _ => None,
        }
    }

    /// Fault injection for tests: the next `kind` message from `sender` arrives `extra_ms` late,
    /// so a message sent after it on the same link can overtake it (spec 4.1 reordering).
    pub fn inject_message_delay(&mut self, sender: (EntityKind, u16), kind: u16, extra_ms: TimeMs) {
        self.links.injected.push((sender, kind, extra_ms));
    }

    fn injected_delay(&mut self, msg: &Msg) -> TimeMs {
        if self.links.injected.is_empty() {
            return 0;
        }
        let key = (msg.sender(), msg.kind_code());
        match self
            .links
            .injected
            .iter()
            .position(|&(s, k, _)| (s, k) == key)
        {
            Some(i) => self.links.injected.remove(i).2,
            None => 0,
        }
    }

    /// Spec 4.1: latency per link, loss only on the D <-> PC radio link.
    pub fn send_with_seq(&mut self, msg: Msg, seq: u32, link_seq: u32) {
        let latency = if msg.is_radio() {
            self.cfg.handshake.local_latency_ms
        } else {
            self.cfg.handshake.fm_latency_ms
        } + self.injected_delay(&msg);
        if msg.is_radio() {
            let (kind, id) = msg.sender();
            let stream = match kind {
                EntityKind::Pad => &mut self.rng.loss_pad[usize::from(id)],
                _ => &mut self.rng.loss_drone[usize::from(id)],
            };
            if stream.bernoulli(self.cfg.handshake.p_msg_loss) {
                let ev = Ev::Message {
                    seq,
                    link_seq,
                    sent: self.now,
                    msg,
                };
                self.log_event(self.now, &ev, LogStatus::Lost);
                return;
            }
        }
        self.after(
            latency,
            Ev::Message {
                seq,
                link_seq,
                sent: self.now,
                msg,
            },
        );
    }

    pub fn log_event(&mut self, t: TimeMs, ev: &Ev, status: LogStatus) {
        let mut args = Args::default();
        ev.encode(&mut args);
        let seq = if let Ev::Message { seq, .. } = ev {
            *seq
        } else {
            0
        };
        self.out
            .event_record(t, ev.kind_code(), ev.class().code(), status, seq, &args);
    }

    pub fn log_transition(&mut self, code: u16, pad: PadId, drone: DroneId, from: u8, to: u8) {
        let mut args = Args::default();
        args.u16(code).u16(pad).u16(drone).u8(from).u8(to);
        self.out.event_record(
            self.now,
            TRANSITION_KIND,
            NO_STATE,
            LogStatus::Processed,
            0,
            &args,
        );
    }

    #[allow(clippy::too_many_arguments)]
    pub fn alert(
        &mut self,
        code: AlertCode,
        severity: AlertSeverity,
        source: (EntityKind, u16),
        pad: PadId,
        drone: DroneId,
        order: OrderId,
    ) {
        self.metrics.alert(code, self.now);
        let (pad_s, drone_s, order_s) = (
            OptId(u64::from(pad), u64::from(NO_PAD)),
            OptId(u64::from(drone), u64::from(NO_DRONE)),
            OptId(u64::from(order), u64::from(NO_ORDER)),
        );
        self.out.alert_line(format_args!(
            "{{\"t_ms\":{},\"code\":\"{}\",\"severity\":\"{}\",\"source_kind\":\"{}\",\"source_id\":{},\"pad_id\":{},\"drone_id\":{},\"order_id\":{}}}\n",
            self.now,
            code.name(),
            severity.name(),
            source.0.name(),
            source.1,
            pad_s,
            drone_s,
            order_s
        ));
    }

    /// Run to END_OF_RUN, then flush outputs. Returns the metrics summary JSON.
    pub fn run(&mut self) -> io::Result<Value> {
        while let Some(entry) = self.queue.pop() {
            self.now = entry.t;
            self.dispatch(entry);
            self.metrics.events_processed += 1;
            #[cfg(debug_assertions)]
            self.check_invariants();
            if self.ended {
                break;
            }
        }
        self.finish_run();
        self.out.finish()?;
        let d = &self.cfg.drone;
        let cost = CostInputs {
            fleet_size: d.fleet_size,
            drone_per_s: self.cfg.costs.drone_per_s,
            energy_per_j: self.cfg.costs.energy_per_j,
            charging_efficiency: self.cfg.charging.efficiency,
            battery_capacity_j: d.battery_capacity_j,
        };
        let mut summary = self.metrics.summary(&cost);
        summary["scenario"] = Value::from(self.cfg.name.clone());
        summary["seed"] = Value::from(self.cfg.seed);
        summary["replication"] = Value::from(self.replication);
        summary["source_hash"] = Value::from(format!("{:016x}", self.cfg.source_hash));
        summary["fleet_size"] = Value::from(d.fleet_size);
        summary["pads"] = Value::from(self.pads.len());
        Ok(summary)
    }

    fn finish_run(&mut self) {
        self.fm_write_open_orders();
        self.metrics.finish();
        #[cfg(debug_assertions)]
        self.check_orders_accounted();
    }

    fn pad_is_faulted(&self, pad: PadId) -> bool {
        self.pads[usize::from(pad)].mode == PadMode::ControllerFault
    }

    /// Pad addressed by a non-message event, if any (for PC-fault deferral, A36).
    fn pad_target(ev: &Ev) -> Option<PadId> {
        match *ev {
            Ev::ShipmentRequest { pad, .. }
            | Ev::PodPlaceDone { pad, .. }
            | Ev::PodRemoveDone { pad, .. } => Some(pad),
            Ev::Timer {
                owner_kind: EntityKind::Pad,
                owner_id,
                ..
            } => Some(owner_id),
            _ => None,
        }
    }

    fn timer_is_current(&self, ev: &Ev) -> bool {
        let Ev::Timer {
            timer,
            owner_kind,
            owner_id,
            subject,
            generation,
        } = *ev
        else {
            return true;
        };
        if timer == TimerKind::ReorderRelease {
            let link = self.held_link(owner_kind, owner_id, subject);
            return self
                .links
                .held
                .iter()
                .any(|&(l, s, _)| l == link && s == generation);
        }
        match owner_kind {
            EntityKind::Pad => {
                let v = &self.visits[self.visit_index(owner_id, subject)];
                v.is_open_at(owner_id) && v.generation == generation
            }
            EntityKind::Drone => {
                let d = &self.drones[usize::from(owner_id)];
                generation
                    == match timer {
                        TimerKind::DResponseTimeout => d.gen_response,
                        TimerKind::DHoldRetry => d.gen_hold_retry,
                        TimerKind::DPcSilenceAlert => d.gen_silence,
                        _ => d.gen_reserve,
                    }
            }
            EntityKind::FleetManager => match timer {
                TimerKind::EndOfRun => true,
                TimerKind::FmPadRecheck => self.fm.pad_gen[usize::from(subject)] == generation,
                _ => self.fm.hold_gen[usize::from(subject)] == generation,
            },
        }
    }

    pub(crate) fn dispatch(&mut self, entry: Entry) {
        let ev = entry.ev;
        if let Ev::Message { msg, link_seq, .. } = ev {
            if let Receiver::Pad(p) = msg.receiver()
                && self.pad_is_faulted(p)
            {
                self.log_event(self.now, &ev, LogStatus::Dropped);
                return;
            }
            if self.cfg.handshake.reorder_window_ms > 0
                && let Some(link) = self.link_of(&msg)
            {
                self.receive_in_order(link, link_seq, entry);
                return;
            }
        } else if let Some(p) = Self::pad_target(&ev)
            && self.pad_is_faulted(p)
        {
            self.log_event(self.now, &ev, LogStatus::Deferred);
            self.pads[usize::from(p)].deferred.push(entry);
            return;
        }
        if !self.timer_is_current(&ev) {
            self.log_event(self.now, &ev, LogStatus::Stale);
            return;
        }
        self.process(ev);
    }

    /// Spec 4.1 reordering: a message ahead of a gap on its link is held for
    /// `handshake.reorder_window_s`; the one it overtook releases it, or the window ends and the
    /// gap is taken as a lost message. A link sequence below the expected one is a re-send or a
    /// late copy and goes straight through (receivers treat duplicates as such).
    fn receive_in_order(&mut self, link: usize, link_seq: u32, entry: Entry) {
        let expected = self.links.expected[link];
        if link_seq > expected {
            self.log_event(self.now, &entry.ev, LogStatus::Deferred);
            self.links.held.push((link, link_seq, entry));
            let Ev::Message { msg, .. } = entry.ev else {
                unreachable!("only messages are held")
            };
            let (receiver_kind, receiver_id) = match msg.receiver() {
                Receiver::Pad(p) => (EntityKind::Pad, p),
                Receiver::Drone(d) => (EntityKind::Drone, d),
                Receiver::FleetManager => unreachable!("FM links are not reordered"),
            };
            let window = self.cfg.handshake.reorder_window_ms;
            self.timer(
                window,
                TimerKind::ReorderRelease,
                receiver_kind,
                receiver_id,
                msg.sender().1,
                link_seq,
            );
            return;
        }
        if link_seq == expected {
            self.links.expected[link] = expected + 1;
        }
        self.process(entry.ev);
        self.release_in_order(link);
    }

    /// Process held messages on `link` that are now due: every one at or below the expected
    /// sequence, lowest first.
    fn release_in_order(&mut self, link: usize) {
        loop {
            let due = self
                .links
                .held
                .iter()
                .enumerate()
                .filter(|&(_, &(l, s, _))| l == link && s <= self.links.expected[link])
                .min_by_key(|&(_, &(_, s, _))| s)
                .map(|(i, _)| i);
            let Some(i) = due else { break };
            let (_, s, entry) = self.links.held.remove(i);
            if s == self.links.expected[link] {
                self.links.expected[link] = s + 1;
            }
            let receiver_faulted = matches!(
                entry.ev,
                Ev::Message { msg, .. }
                    if matches!(msg.receiver(), Receiver::Pad(p) if self.pad_is_faulted(p))
            );
            if receiver_faulted {
                self.log_event(self.now, &entry.ev, LogStatus::Dropped);
            } else {
                self.process(entry.ev);
            }
        }
    }

    fn held_link(&self, receiver_kind: EntityKind, receiver: u16, sender: u16) -> usize {
        let (n_pads, n_drones) = (self.pads.len(), self.drones.len());
        match receiver_kind {
            EntityKind::Pad => usize::from(sender) * n_pads + usize::from(receiver),
            _ => n_pads * n_drones + usize::from(sender) * n_drones + usize::from(receiver),
        }
    }

    /// The reorder window of a held message ran out: the gap before it is a lost message.
    fn reorder_window_ended(
        &mut self,
        receiver_kind: EntityKind,
        receiver: u16,
        sender: u16,
        seq: u32,
    ) {
        let link = self.held_link(receiver_kind, receiver, sender);
        self.links.expected[link] = self.links.expected[link].max(seq);
        self.release_in_order(link);
    }

    fn process(&mut self, ev: Ev) {
        self.log_event(self.now, &ev, LogStatus::Processed);
        match ev {
            Ev::Message { sent, msg, .. } => match msg.receiver() {
                Receiver::Pad(p) => {
                    self.pc_message(p, msg);
                    self.pad_tick(p);
                }
                Receiver::Drone(d) => self.drone_message(d, msg),
                Receiver::FleetManager => self.fm_message(sent, msg),
            },
            Ev::ShipmentRequest {
                flow,
                pad,
                dest,
                deadline,
            } => {
                self.pc_shipment_request(pad, dest, deadline);
                self.pad_tick(pad);
                self.schedule_next_shipment(flow);
            }
            Ev::PodPlaceDone { pad, pod } => {
                self.pc_pod_place_done(pad, pod);
                self.pad_tick(pad);
            }
            Ev::PodRemoveDone { pad, pod } => {
                self.pc_pod_remove_done(pad, pod);
                self.pad_tick(pad);
            }
            Ev::FaultStart { kind, pad, random } => self.fault_start(kind, pad, random),
            Ev::FaultEnd { kind, pad } => self.fault_end(kind, pad),
            Ev::RepairDone { pad, drone } => self.repair_done(pad, drone),
            Ev::Timer {
                timer: TimerKind::ReorderRelease,
                owner_kind,
                owner_id,
                subject,
                generation,
            } => self.reorder_window_ended(owner_kind, owner_id, subject, generation),
            Ev::Timer {
                timer,
                owner_kind,
                owner_id,
                subject,
                ..
            } => match owner_kind {
                EntityKind::Pad => {
                    self.pc_timer(owner_id, subject, timer);
                    self.pad_tick(owner_id);
                }
                EntityKind::Drone => self.drone_timer(owner_id, timer),
                EntityKind::FleetManager => match timer {
                    TimerKind::EndOfRun => self.ended = true,
                    TimerKind::FmPadRecheck => self.fm_pad_recheck(subject),
                    _ => self.fm_hold_limit(subject),
                },
            },
            _ => self.drone_completion(ev),
        }
    }

    /// Process events held during a PC fault, in their original order (M02, A36).
    pub(crate) fn release_deferred(&mut self, pad: PadId) {
        let held = std::mem::take(&mut self.pads[usize::from(pad)].deferred);
        let mut held = held;
        for entry in held.drain(..) {
            self.dispatch(entry);
        }
        self.pads[usize::from(pad)].deferred = held;
    }

    /// Visits are kept per (pad, drone) so a pad that is faulted or waiting on a silent drone
    /// never loses its record when the drone starts a visit elsewhere.
    pub fn visit_index(&self, pad: PadId, drone: DroneId) -> usize {
        usize::from(pad) * self.drones.len() + usize::from(drone)
    }

    pub fn soc_f32(&self, d: DroneId) -> f32 {
        #[allow(clippy::cast_possible_truncation)]
        let s = self.drones[usize::from(d)].soc;
        s
    }

    pub fn service_ms(&self, pad: PadId, manual_ms: TimeMs, auto_ms: TimeMs) -> TimeMs {
        match self.pads[usize::from(pad)].service_mode {
            ServiceMode::Manual => manual_ms,
            ServiceMode::Auto => auto_ms,
        }
    }

    pub fn t_land_timeout_ms(&self) -> TimeMs {
        ms_from_s(self.energy.t_cycle_s) + self.cfg.handshake.response_timeout_ms
    }

    /// The four required invariants, checked after every event in debug builds.
    #[cfg(debug_assertions)]
    fn check_invariants(&mut self) {
        let n_pods = self.pod_mass_kg.len();
        let marks = &mut self.scratch;
        marks[..n_pods].fill(0);
        let mut mark = |pod: PodId, what: &str| {
            if pod != NO_POD {
                let m = &mut marks[pod as usize];
                *m += 1;
                assert!(*m == 1, "pod {pod} counted twice ({what})");
            }
        };
        for pad in &self.pads {
            for &pod in &pad.inventory {
                mark(pod, "inventory");
            }
            mark(pad.in_pod, "IN");
            mark(pad.out_pod, "OUT");
            if pad.out_placing {
                mark(pad.placing.pod, "placing");
            }
        }
        for d in &self.drones {
            mark(d.pod, "drone");
        }
        assert!(
            marks[..n_pods].iter().all(|&m| m == 1),
            "pod conservation violated at t={}",
            self.now
        );
        if let Some(cap) = self.cfg.max_backlog_per_pad {
            assert!(
                self.pads
                    .iter()
                    .all(|p| p.backlog.len() <= usize::from(cap)),
                "a backlog exceeds demand.max_backlog_per_pad at t={}",
                self.now
            );
        }

        let n_pads = self.pads.len();
        let occ = &mut self.scratch;
        occ[..n_pads * 3].fill(0);
        for (i, d) in self.drones.iter().enumerate() {
            assert!(
                d.soc.is_finite() && d.soc >= 0.0,
                "drone {i} battery {} at t={}",
                d.soc,
                self.now
            );
            if let (Some(spot), true) = (d.spot, d.pad != NO_PAD) {
                let slot = usize::from(d.pad) * 3 + usize::from(spot.code());
                occ[slot] += 1;
            }
        }
        for p in 0..n_pads {
            assert!(
                occ[p * 3] <= 1 && occ[p * 3 + 1] <= 1,
                "two drones on one spot at pad {p}, t={}",
                self.now
            );
            let cap = self.pads[p].parking_capacity;
            assert!(
                u16::from(occ[p * 3 + 2]) <= cap,
                "parking over capacity at pad {p}, t={}",
                self.now
            );
        }
        // The PC's spot occupancy may lag the physical truth by at most one departure-timer
        // period (a lost DEPARTED is caught by the spot sensor at the next check), unless the
        // PC is faulted.
        let move_ms = ms_from_s(self.cfg.drone.takeoff_s.max(self.cfg.drone.hop_s));
        let lag_ms =
            move_ms + self.cfg.handshake.local_latency_ms + self.cfg.handshake.response_timeout_ms;
        for (p, pad) in self.pads.iter().enumerate() {
            if pad.mode == PadMode::ControllerFault {
                continue;
            }
            for d in [pad.in_drone, pad.out_drone] {
                if d == NO_DRONE {
                    continue;
                }
                let dr = &self.drones[usize::from(d)];
                let here = usize::from(dr.pad) == p && dr.spot.is_some();
                let just_left =
                    usize::from(dr.left_pad) == p && self.now - dr.left_pad_at <= lag_ms;
                assert!(
                    here || just_left,
                    "pad {p} believes drone {d} is on a spot it left, t={}",
                    self.now
                );
            }
        }
        for o in &self.fm.orders {
            assert!(
                o.status != crate::ids::OrderStatus::Failed || o.fail_reason.is_some(),
                "order {} failed without a reason",
                o.order_id
            );
        }
    }

    /// End of run: every order is closed (delivered, diverted, failed with a reason) or open
    /// with its pod physically present on the network.
    #[cfg(debug_assertions)]
    fn check_orders_accounted(&self) {
        use crate::ids::OrderStatus;
        for o in &self.fm.orders {
            match o.status {
                OrderStatus::Delivered | OrderStatus::Diverted => {
                    assert!(o.delivered_pad != NO_PAD)
                }
                OrderStatus::Failed => assert!(o.fail_reason.is_some()),
                _ => {
                    let pod = o.pod_id;
                    let on_drone = self.drones.iter().any(|d| d.pod == pod);
                    let on_pad = self
                        .pads
                        .iter()
                        .any(|p| p.out_pod == pod || p.in_pod == pod);
                    assert!(
                        on_drone || on_pad,
                        "open order {} lost its pod {pod}",
                        o.order_id
                    );
                }
            }
        }
    }
}
