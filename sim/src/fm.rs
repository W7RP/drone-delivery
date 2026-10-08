//! Fleet manager (spec 6): order and pod records, mirrors of drones and pads built only from
//! messages, the six policy interfaces with their default implementations, and the truck
//! baseline (spec 7). Policies are pure functions of a read-only `FleetView`; FM turns their
//! decisions into messages.

use crate::config::Config;
use crate::event::Msg;
use crate::flight::EnergyRules;
use crate::ids::{
    AlertCode, AlertSeverity, DroneId, DroneMode, EntityKind, HoldReason, InOccupiedAction,
    MissionKind, NO_DRONE, NO_ORDER, NO_PAD, NO_POD, NO_TIME, OrderId, OrderKind, OrderStatus,
    PadId, PadMode, PodId, PodLocation, ReturnPolicy, ServiceOutcome, Spot, TimeMs, TimerKind,
    ms_from_s, s_from_ms,
};
use crate::metrics::{ClosedOrder, DoorToDoorSplit};
use crate::network::Network;
use crate::world::World;

const SECONDS_PER_DAY: f64 = 86_400.0;
const SECONDS_PER_HOUR: f64 = 3600.0;

#[derive(Clone, Copy, Debug)]
pub struct FmDrone {
    pub mode: DroneMode,
    pub pad: PadId,
    pub last_pad: PadId,
    pub dest: PadId,
    pub mission: MissionKind,
    pub pod: PodId,
    pub order: OrderId,
    pub pickup_order: OrderId,
    pub soc: f32,
    pub depart_t: TimeMs,
    pub spot: Option<Spot>,
    pub hold_since: TimeMs,
    pub landed_since_hold: bool,
    /// Sent MISSION_REQUEST and has no mission yet: idle and assignable from any pad.
    pub awaiting_mission: bool,
    /// Told to make room (A74) and not heard from since.
    pub yielding: bool,
}

impl FmDrone {
    pub fn soc64(&self) -> f64 {
        f64::from(self.soc)
    }
}

#[derive(Clone, Debug)]
pub struct FmPad {
    pub mode: PadMode,
    pub empty_pods: u32,
    pub backlog_len: u16,
    /// Pod waiting on each OUT spot, as FM last heard (POD_PLACED, POD_LATCHED, PAD_STATUS).
    pub out_pods: Vec<PodId>,
    pub returns_inbound: u32,
    /// The pad reported an unreadable fiducial (A35): no drone is sent to land there.
    pub marker_fault: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Order {
    pub order_id: OrderId,
    pub kind: OrderKind,
    pub status: OrderStatus,
    pub pod_id: PodId,
    pub origin_pad: PadId,
    pub dest_pad: PadId,
    pub delivered_pad: PadId,
    pub drone_id: DroneId,
    pub payload_kg: f32,
    pub requested_t_ms: TimeMs,
    pub placed_t_ms: TimeMs,
    pub deadline_t_ms: TimeMs,
    pub assigned_t_ms: TimeMs,
    pub latched_t_ms: TimeMs,
    pub departed_t_ms: TimeMs,
    pub delivered_t_ms: TimeMs,
    pub truck_delivered_t_ms: TimeMs,
    pub truck_cost: f64,
    pub fail_reason: Option<AlertCode>,
    /// Measurement only (spec 7 M2 split), never read by FM decisions: start of the latch that
    /// picked this pod up, lane time flown with it latched, and time its drone held with it.
    pub latch_start_t_ms: TimeMs,
    pub flight_ms: TimeMs,
    pub hold_ms: TimeMs,
    pub excluded_drone: DroneId,
}

impl Order {
    /// Door-to-door time of a delivered or diverted order in four parts that sum to it exactly
    /// (spec 7, M2): pod wait (request to pod on OUT), queue (pod on OUT to latch start, plus
    /// landing holds), flight (lanes flown with the pod) and pad service (the rest: latch,
    /// charging before departure, takeoff, approaches, release). None while open or failed.
    pub fn split(&self) -> Option<DoorToDoorSplit> {
        let delivered = matches!(self.status, OrderStatus::Delivered | OrderStatus::Diverted);
        if !delivered || self.latch_start_t_ms == NO_TIME {
            return None;
        }
        let pod_wait_ms = self.placed_t_ms - self.requested_t_ms;
        let queue_ms = (self.latch_start_t_ms - self.placed_t_ms) + self.hold_ms;
        let total_ms = self.delivered_t_ms - self.requested_t_ms;
        Some(DoorToDoorSplit {
            pod_wait_ms,
            queue_ms,
            flight_ms: self.flight_ms,
            pad_service_ms: total_ms - pod_wait_ms - queue_ms - self.flight_ms,
        })
    }

    fn is_closed(&self) -> bool {
        matches!(
            self.status,
            OrderStatus::Delivered
                | OrderStatus::Diverted
                | OrderStatus::Failed
                | OrderStatus::Trucked
        )
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PodRec {
    pub home_pad: PadId,
    pub location: PodLocation,
    pub pad: PadId,
    pub drone: DroneId,
    pub order: OrderId,
    pub loaded: bool,
    pub return_dest: PadId,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mission {
    pub kind: MissionKind,
    pub dest: PadId,
    pub order: OrderId,
    pub pickup_order: OrderId,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HoldDecision {
    Continue {
        retry_after_ms: TimeMs,
        limit_t: Option<TimeMs>,
    },
    Divert,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum YieldCause {
    Placement,
    Inbound,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PodReturn {
    pub pod: PodId,
    pub from: PadId,
    pub dest: PadId,
}

/// Read-only state a policy may look at (spec 6 `FleetView`).
pub struct FleetView<'a> {
    pub fm: &'a Fm,
    pub cfg: &'a Config,
    pub net: &'a Network,
    pub energy: &'a EnergyRules,
    pub now: TimeMs,
    /// Weather blocks flights between pads this hour (A79): policies assign and move nothing.
    pub flights_blocked: bool,
}

pub trait AssignmentPolicy {
    fn assign(&self, v: &FleetView<'_>, order: OrderId) -> Option<DroneId>;
}
pub trait DispatchPolicy {
    fn mission_for(&self, v: &FleetView<'_>, drone: DroneId) -> Option<Mission>;
}
pub trait HoldingPolicy {
    fn on_hold(
        &self,
        v: &FleetView<'_>,
        drone: DroneId,
        pad: PadId,
        reason: HoldReason,
        hold_cycles: u8,
    ) -> HoldDecision;
}
pub trait DroneRebalancingPolicy {
    fn place_idle(&self, v: &FleetView<'_>, drone: DroneId) -> Option<Mission>;
    fn yield_for(&self, v: &FleetView<'_>, parked: DroneId, cause: YieldCause) -> Option<Mission>;
}
pub trait PodRebalancingPolicy {
    fn on_inventory_change(
        &self,
        v: &FleetView<'_>,
        pad: PadId,
        removed: Option<PodId>,
        out: &mut Vec<PodReturn>,
    );
}
pub trait DiversionPolicy {
    fn alternate(&self, v: &FleetView<'_>, drone: DroneId, failed: PadId) -> Option<PadId>;
}

pub struct Policies {
    pub assignment: Box<dyn AssignmentPolicy>,
    pub dispatch: Box<dyn DispatchPolicy>,
    pub holding: Box<dyn HoldingPolicy>,
    pub drone_rebalancing: Box<dyn DroneRebalancingPolicy>,
    pub pod_rebalancing: Box<dyn PodRebalancingPolicy>,
    pub diversion: Box<dyn DiversionPolicy>,
}

impl Policies {
    /// The phase-2 defaults; config names are validated to these at load.
    pub fn defaults() -> Policies {
        Policies {
            assignment: Box::new(MinEta),
            dispatch: Box::new(Immediate),
            holding: Box::new(BoundedHold),
            drone_rebalancing: Box::new(NearestUnclaimed),
            pod_rebalancing: Box::new(MinThreshold),
            diversion: Box::new(NearestFeasible),
        }
    }
}

pub struct Fm {
    pub drones: Vec<FmDrone>,
    pub pads: Vec<FmPad>,
    pub orders: Vec<Order>,
    pub pods: Vec<PodRec>,
    pub unassigned: Vec<OrderId>,
    pub msg_seq: u32,
    pub hold_gen: Vec<u32>,
    /// Generation of each pad's FM_PAD_RECHECK timer.
    pub pad_gen: Vec<u32>,
    /// ENERGY_STRANDED already raised for this drone's current stranding.
    pub stranded: Vec<bool>,
    retrying: bool,
    making_room: bool,
    /// Claims per pad (spec 6.4), refreshed before each rebalancing decision.
    pub claims: Vec<u16>,
    returns_buf: Vec<PodReturn>,
    pub policies: Policies,
}

impl Fm {
    pub fn new(cfg: &Config, pod_home: &[PadId]) -> Fm {
        let n_drones = usize::from(cfg.drone.fleet_size);
        Fm {
            drones: vec![
                FmDrone {
                    mode: DroneMode::Parked,
                    pad: NO_PAD,
                    last_pad: NO_PAD,
                    dest: NO_PAD,
                    mission: MissionKind::Park,
                    pod: NO_POD,
                    order: NO_ORDER,
                    pickup_order: NO_ORDER,
                    soc: crate::world::narrow_f32(cfg.drone.initial_soc),
                    depart_t: 0,
                    spot: None,
                    hold_since: 0,
                    landed_since_hold: false,
                    awaiting_mission: false,
                    yielding: false,
                };
                n_drones
            ],
            pads: cfg
                .pads
                .iter()
                .map(|p| FmPad {
                    mode: PadMode::Online,
                    empty_pods: p.initial_empty_pods,
                    backlog_len: 0,
                    out_pods: vec![NO_POD; usize::from(p.out_spots)],
                    returns_inbound: 0,
                    marker_fault: false,
                })
                .collect(),
            orders: Vec::with_capacity(1024),
            pods: pod_home
                .iter()
                .map(|&home| PodRec {
                    home_pad: home,
                    location: PodLocation::PadInventory,
                    pad: home,
                    drone: NO_DRONE,
                    order: NO_ORDER,
                    loaded: false,
                    return_dest: NO_PAD,
                })
                .collect(),
            unassigned: Vec::with_capacity(cfg.pads.len() * 2),
            msg_seq: 0,
            hold_gen: vec![0; n_drones],
            pad_gen: vec![0; cfg.pads.len()],
            stranded: vec![false; n_drones],
            retrying: false,
            making_room: false,
            claims: vec![0; cfg.pads.len()],
            returns_buf: Vec::with_capacity(cfg.pads.len()),
            policies: Policies::defaults(),
        }
    }

    pub fn init_drone(&mut self, d: DroneId, pad: PadId, spot: Option<Spot>, soc: f32) {
        let fd = &mut self.drones[usize::from(d)];
        fd.pad = pad;
        fd.last_pad = pad;
        fd.dest = pad;
        fd.spot = spot;
        fd.soc = soc;
    }

    pub fn pod_order(&self, pod: PodId) -> OrderId {
        if pod == NO_POD {
            NO_ORDER
        } else {
            self.pods[pod as usize].order
        }
    }

    /// The drone is on an OUT spot with no pod on it, so it may park there (6.4).
    fn parks_on_free_out(&self, d: DroneId) -> bool {
        let fd = &self.drones[usize::from(d)];
        fd.spot
            .and_then(Spot::out_index)
            .is_some_and(|k| self.pads[usize::from(fd.pad)].out_pods[k] == NO_POD)
    }

    fn claim_of(&self, d: usize) -> PadId {
        let fd = &self.drones[d];
        match fd.mode {
            DroneMode::Grounded => NO_PAD,
            DroneMode::OnPad | DroneMode::Parked
                if matches!(fd.mission, MissionKind::Park | MissionKind::None) =>
            {
                fd.pad
            }
            _ => fd.dest,
        }
    }

    fn refresh_claims(&mut self) {
        self.claims.fill(0);
        for d in 0..self.drones.len() {
            let p = self.claim_of(d);
            if p != NO_PAD {
                self.claims[usize::from(p)] += 1;
            }
        }
    }
}

impl FleetView<'_> {
    pub fn online(&self, p: PadId) -> bool {
        self.fm.pads[usize::from(p)].mode == PadMode::Online
    }

    pub fn charger(&self, p: PadId) -> bool {
        self.cfg.has_charger(p)
    }

    pub fn marker_fault(&self, p: PadId) -> bool {
        self.fm.pads[usize::from(p)].marker_fault
    }

    /// A pad a drone may be sent to land at: ONLINE and not reporting a marker fault (A35).
    pub fn landable(&self, p: PadId) -> bool {
        self.online(p) && !self.marker_fault(p)
    }

    fn drone(&self, d: DroneId) -> &FmDrone {
        &self.fm.drones[usize::from(d)]
    }

    /// Mass the drone carries, from FM's records.
    pub fn payload(&self, d: DroneId) -> f64 {
        let fd = self.drone(d);
        if fd.pod == NO_POD || fd.order == NO_ORDER {
            0.0
        } else {
            f64::from(self.fm.orders[fd.order as usize].payload_kg)
        }
    }

    pub fn dispatch_ok(&self, soc: f64, a: PadId, b: PadId, payload_kg: f64) -> bool {
        a == b
            || self.energy.usable_j(soc)
                >= self.energy.dispatch_need_j(
                    self.net,
                    a,
                    b,
                    payload_kg,
                    |q| self.landable(q),
                    |q| self.charger(q),
                )
    }

    /// Where an order to `dest` should be flown from `origin` (6.1 amendment): the destination
    /// when it is ONLINE; the nearest landable alternate (not the origin) when it is
    /// OUT_OF_SERVICE; None while it is only suspected faulted (wait, usually brief) or reports
    /// a marker fault (wait until it clears, A35).
    pub fn delivery_target(&self, origin: PadId, dest: PadId) -> Option<PadId> {
        if self.marker_fault(dest) {
            return None;
        }
        match self.fm.pads[usize::from(dest)].mode {
            PadMode::Online => Some(dest),
            PadMode::ControllerFault => None,
            PadMode::OutOfService => self
                .energy
                .nearest_other(self.net, dest, |q| q != origin && self.landable(q)),
        }
    }

    /// Spec 6.4: claims of the other drones are fewer than the pad's slots.
    pub fn unclaimed_for(&self, p: PadId, d: DroneId) -> bool {
        let own = u16::from(self.fm.claim_of(usize::from(d)) == p);
        self.online(p)
            && self.fm.claims[usize::from(p)] - own < self.cfg.pads[usize::from(p)].slots()
    }

    fn nearest_unclaimed(&self, d: DroneId, from: PadId, need_charger: bool) -> Option<PadId> {
        let soc = self.drone(d).soc64();
        self.energy.nearest_other(self.net, from, |q| {
            self.unclaimed_for(q, d)
                && !self.marker_fault(q)
                && (!need_charger || self.charger(q))
                && self.dispatch_ok(soc, from, q, 0.0)
        })
    }
}

/// Default MIN_ETA (6.1).
pub struct MinEta;

impl AssignmentPolicy for MinEta {
    fn assign(&self, v: &FleetView<'_>, o: OrderId) -> Option<DroneId> {
        if v.flights_blocked {
            return None;
        }
        let ord = &v.fm.orders[o as usize];
        let (p, q, m) = (ord.origin_pad, ord.dest_pad, f64::from(ord.payload_kg));
        if !v.online(p) {
            return None;
        }
        let q = v.delivery_target(p, q)?;
        let cap = v.energy.capacity_j;
        let hover0 = v.energy.model.hover_power_w(0.0);
        let mut best: Option<(TimeMs, DroneId)> = None;
        for (i, fd) in v.fm.drones.iter().enumerate() {
            let e = DroneId::try_from(i).expect("fleet size fits u16");
            if fd.mode == DroneMode::Grounded
                || fd.pickup_order != NO_ORDER
                || ord.excluded_drone == e
            {
                continue;
            }
            let at_origin = matches!(fd.mode, DroneMode::OnPad | DroneMode::Parked)
                && fd.pad == p
                && fd.pod == NO_POD;
            let inbound = matches!(
                fd.mode,
                DroneMode::Takeoff | DroneMode::EnRoute | DroneMode::Holding
            ) && fd.dest == p;
            let idle = fd.mode == DroneMode::Parked
                || (fd.mode == DroneMode::OnPad && fd.awaiting_mission);
            // A drone elsewhere would have to land at the origin: not while it reports a
            // marker fault (A35). Drones on it or already inbound keep their chance.
            let parked_elsewhere = idle && fd.pad != p && fd.pod == NO_POD && !v.marker_fault(p);
            let (eta_ms, mut soc) = if at_origin {
                (0, f64::from(fd.soc))
            } else if inbound {
                let route_s = v.energy.route_time_s(v.net, fd.last_pad, p) + v.cfg.drone.takeoff_s;
                let arrive = fd.depart_t + ms_from_s(route_s);
                let used = v.energy.route_energy(v.net, fd.last_pad, p, v.payload(e));
                (arrive.saturating_sub(v.now), f64::from(fd.soc) - used / cap)
            } else if parked_elsewhere {
                if !v.dispatch_ok(f64::from(fd.soc), fd.pad, p, 0.0) {
                    continue;
                }
                let route_s = v.cfg.drone.takeoff_s + v.energy.route_time_s(v.net, fd.pad, p);
                let used = hover0 * (v.cfg.drone.takeoff_s + v.cfg.drone.approach_s)
                    + v.energy.route_energy(v.net, fd.pad, p, 0.0);
                (ms_from_s(route_s), f64::from(fd.soc) - used / cap)
            } else {
                continue;
            };
            if v.charger(p) {
                soc = soc.max(v.energy.charge_target_soc);
            }
            if !v.dispatch_ok(soc, p, q, m) {
                continue;
            }
            if best.is_none_or(|(t, _)| eta_ms < t) {
                best = Some((eta_ms, e));
            }
        }
        best.map(|(_, e)| e)
    }
}

/// Default IMMEDIATE (6.2).
pub struct Immediate;

impl DispatchPolicy for Immediate {
    fn mission_for(&self, v: &FleetView<'_>, d: DroneId) -> Option<Mission> {
        if v.flights_blocked {
            return None;
        }
        let fd = v.drone(d);
        if fd.pod != NO_POD {
            let order = v.fm.pod_order(fd.pod);
            let ordered = v.fm.orders[order as usize].dest_pad;
            let dest = v.delivery_target(fd.pad, ordered).unwrap_or(ordered);
            if dest != ordered {
                return Some(Mission {
                    kind: MissionKind::Divert,
                    dest,
                    order,
                    pickup_order: NO_ORDER,
                });
            }
            let pickup = if fd.pickup_order != NO_ORDER
                && v.fm.orders[fd.pickup_order as usize].origin_pad == dest
            {
                fd.pickup_order
            } else {
                NO_ORDER
            };
            return Some(Mission {
                kind: MissionKind::Deliver,
                dest,
                order,
                pickup_order: pickup,
            });
        }
        if fd.pickup_order != NO_ORDER {
            let origin = v.fm.orders[fd.pickup_order as usize].origin_pad;
            return Some(Mission {
                kind: MissionKind::Pickup,
                dest: origin,
                order: NO_ORDER,
                pickup_order: fd.pickup_order,
            });
        }
        None
    }
}

/// Default BOUNDED_HOLD (6.3).
pub struct BoundedHold;

impl HoldingPolicy for BoundedHold {
    fn on_hold(
        &self,
        v: &FleetView<'_>,
        d: DroneId,
        pad: PadId,
        reason: HoldReason,
        hold_cycles: u8,
    ) -> HoldDecision {
        let fd = v.drone(d);
        let m = v.payload(d);
        let hover = v.energy.model.hover_power_w(m);
        let cap = v.energy.capacity_j;
        let alt = v.energy.nearest_other(v.net, pad, |q| v.landable(q));
        match reason {
            HoldReason::PcSilent | HoldReason::OutOfService => HoldDecision::Divert,
            HoldReason::MarkerNotDetected => {
                let more_s = v.cfg.policy.marker_hold_s + v.energy.t_cycle_s;
                let ok = hold_cycles <= v.cfg.policy.marker_max_hold_cycles
                    && alt.is_some_and(|q| {
                        v.energy.divert_ok(
                            v.net,
                            f64::from(fd.soc) - hover * more_s / cap,
                            pad,
                            q,
                            m,
                        )
                    });
                if ok {
                    HoldDecision::Continue {
                        retry_after_ms: ms_from_s(v.cfg.policy.marker_hold_s),
                        limit_t: None,
                    }
                } else {
                    HoldDecision::Divert
                }
            }
            HoldReason::InOccupied
                if v.cfg.policy.in_occupied_action == InOccupiedAction::Divert =>
            {
                HoldDecision::Divert
            }
            HoldReason::InOccupied | HoldReason::AwaitOut | HoldReason::Weather => {
                let Some(q) = alt else {
                    return HoldDecision::Continue {
                        retry_after_ms: v.cfg.handshake.hold_retry_ms,
                        limit_t: None,
                    };
                };
                let reserve_for_divert =
                    v.energy.route_energy(v.net, pad, q, m) + hover * v.energy.t_cycle_s;
                let spare_j = v.energy.usable_j(f64::from(fd.soc)) - reserve_for_divert;
                if spare_j <= 0.0 {
                    return HoldDecision::Divert;
                }
                let energy_limit = v.now + ms_from_s(spare_j / hover);
                let time_limit = fd.hold_since + ms_from_s(v.cfg.policy.max_hold_s);
                HoldDecision::Continue {
                    retry_after_ms: v.cfg.handshake.hold_retry_ms,
                    limit_t: Some(energy_limit.min(time_limit)),
                }
            }
        }
    }
}

/// Default NEAREST_UNCLAIMED (6.4).
pub struct NearestUnclaimed;

impl DroneRebalancingPolicy for NearestUnclaimed {
    fn place_idle(&self, v: &FleetView<'_>, d: DroneId) -> Option<Mission> {
        let fd = v.drone(d);
        let p = fd.pad;
        let reposition = |q: PadId| Mission {
            kind: MissionKind::Reposition,
            dest: q,
            order: NO_ORDER,
            pickup_order: NO_ORDER,
        };
        if !v.charger(p)
            && f64::from(fd.soc) < v.energy.charge_target_soc
            && let Some(q) = v.nearest_unclaimed(d, p, true)
        {
            return Some(reposition(q));
        }
        let can_park_here =
            v.cfg.pads[usize::from(p)].parking_capacity > 0 || v.fm.parks_on_free_out(d);
        if v.unclaimed_for(p, d) && can_park_here {
            return Some(Mission {
                kind: MissionKind::Park,
                dest: p,
                order: NO_ORDER,
                pickup_order: NO_ORDER,
            });
        }
        v.nearest_unclaimed(d, p, false).map(reposition)
    }

    fn yield_for(&self, v: &FleetView<'_>, parked: DroneId, cause: YieldCause) -> Option<Mission> {
        if v.flights_blocked {
            return None;
        }
        let fd = v.drone(parked);
        match cause {
            // Yielding locally means landing here again, which a marker fault rules out (A35).
            YieldCause::Placement if !v.marker_fault(fd.pad) => Some(Mission {
                kind: MissionKind::YieldLocal,
                dest: fd.pad,
                order: NO_ORDER,
                pickup_order: NO_ORDER,
            }),
            YieldCause::Placement | YieldCause::Inbound => {
                v.nearest_unclaimed(parked, fd.pad, false).map(|q| Mission {
                    kind: MissionKind::Reposition,
                    dest: q,
                    order: NO_ORDER,
                    pickup_order: NO_ORDER,
                })
            }
        }
    }
}

/// Default MIN_THRESHOLD (6.5).
pub struct MinThreshold;

impl PodRebalancingPolicy for MinThreshold {
    fn on_inventory_change(
        &self,
        v: &FleetView<'_>,
        pad: PadId,
        removed: Option<PodId>,
        out: &mut Vec<PodReturn>,
    ) {
        if let Some(pod) = removed {
            let rec = &v.fm.pods[pod as usize];
            if v.cfg.pads[usize::from(pad)].return_policy == ReturnPolicy::ReturnHome
                && rec.home_pad != pad
                && rec.return_dest == NO_PAD
            {
                out.push(PodReturn {
                    pod,
                    from: pad,
                    dest: rec.home_pad,
                });
            }
        }
        let min = v.cfg.policy.pod_min_per_pad;
        let taken_from = |q: usize, out: &Vec<PodReturn>| -> u32 {
            u32::try_from(out.iter().filter(|r| usize::from(r.from) == q).count())
                .unwrap_or(u32::MAX)
        };
        for p in 0..v.fm.pads.len() {
            let fp = &v.fm.pads[p];
            let incoming = fp.returns_inbound
                + u32::try_from(out.iter().filter(|r| usize::from(r.dest) == p).count())
                    .unwrap_or(0);
            if fp.empty_pods + incoming >= min {
                continue;
            }
            let donor = (0..v.fm.pads.len())
                .filter(|&q| q != p)
                .map(|q| {
                    (
                        q,
                        i64::from(v.fm.pads[q].empty_pods)
                            - i64::from(taken_from(q, out))
                            - i64::from(min),
                    )
                })
                .filter(|&(_, surplus)| surplus > 0)
                .max_by(|a, b| a.1.cmp(&b.1).then(b.0.cmp(&a.0)));
            let Some((q, _)) = donor else { continue };
            let q_id = PadId::try_from(q).expect("pad count fits u16");
            let pod =
                v.fm.pods
                    .iter()
                    .enumerate()
                    .filter(|(i, r)| {
                        r.location == PodLocation::PadInventory
                            && r.pad == q_id
                            && r.return_dest == NO_PAD
                            && !out.iter().any(|x| x.pod as usize == *i)
                    })
                    .map(|(i, _)| PodId::try_from(i).expect("pod count fits u32"))
                    .next();
            if let Some(pod) = pod {
                out.push(PodReturn {
                    pod,
                    from: q_id,
                    dest: PadId::try_from(p).expect("pad count fits u16"),
                });
            }
        }
    }
}

/// Default NEAREST_FEASIBLE (6.6).
pub struct NearestFeasible;

impl DiversionPolicy for NearestFeasible {
    /// Feasible ONLINE alternates ranked by congestion, then route length, then pad ID: the
    /// fewest other drones holding over the pad or diverting to it. Without the congestion
    /// term two busy neighbours bounce a drone between them until its reserve (6.6 amendment).
    fn alternate(&self, v: &FleetView<'_>, d: DroneId, failed: PadId) -> Option<PadId> {
        let soc = v.drone(d).soc64();
        let m = v.payload(d);
        let congestion = |q: PadId| {
            v.fm.drones
                .iter()
                .enumerate()
                .filter(|&(i, x)| {
                    i != usize::from(d)
                        && ((x.mode == DroneMode::Holding && x.pad == q)
                            || (x.mission == MissionKind::Divert && x.dest == q))
                })
                .count()
        };
        let mut best: Option<(usize, f64, PadId)> = None;
        for q in 0..v.net.n_pads {
            let q = PadId::try_from(q).expect("pad count fits u16");
            if q == failed || !v.landable(q) || !v.energy.divert_ok(v.net, soc, failed, q, m) {
                continue;
            }
            let key = (congestion(q), v.net.route_len(failed, q), q);
            if best.is_none_or(|b| key.0 < b.0 || (key.0 == b.0 && key.1 < b.1)) {
                best = Some(key);
            }
        }
        best.map(|(_, _, q)| q)
    }
}

/// Truck baseline (spec 7): completion time and cost for an order requested at `requested`.
/// With weather, the departure hour's weather slows the truck too (A82).
pub fn truck_baseline(
    cfg: &Config,
    net: &Network,
    origin: PadId,
    dest: PadId,
    requested: TimeMs,
) -> (TimeMs, f64) {
    let t = &cfg.truck;
    let t_dep = requested + ms_from_s(t.dispatch_delay_s + t.load_s);
    let tod_s = (cfg.start_time_of_day_s + s_from_ms(t_dep)).rem_euclid(SECONDS_PER_DAY);
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let hour = ((tod_s / SECONDS_PER_HOUR).floor() as usize).min(23);
    let weather = cfg.weather.as_ref().map_or(1.0, |w| {
        w.truck_factor(&w.hour(w.hour_index_clamped(t_dep)))
    });
    let speed_factor = t.tod_speed_factors[hour] * weather;
    let drive_ms = ms_from_s(net.truck_s(origin, dest) / speed_factor);
    let delivered = t_dep + drive_ms + ms_from_s(t.unload_s);
    let cost = cfg.costs.truck_per_m * net.road(origin, dest)
        + cfg.costs.truck_per_s * (t.load_s + s_from_ms(drive_ms) + t.unload_s);
    (delivered, cost)
}

impl World {
    fn view(&self) -> FleetView<'_> {
        FleetView {
            fm: &self.fm,
            cfg: &self.cfg,
            net: &self.net,
            energy: &self.energy,
            now: self.now,
            flights_blocked: self.flights_blocked(),
        }
    }

    fn fd_mut(&mut self, d: DroneId) -> &mut FmDrone {
        &mut self.fm.drones[usize::from(d)]
    }

    pub(crate) fn fm_message(&mut self, sent: TimeMs, msg: Msg) {
        if let (EntityKind::Pad, pad) = msg.sender() {
            self.fm_pad_alive(pad);
        }
        match msg {
            Msg::PodPlaced {
                pad,
                pod,
                kind,
                dest,
                requested,
                deadline,
                payload_kg,
                out_index,
            } => {
                self.fm_pod_placed(sent, pad, pod, kind, dest, requested, deadline, payload_kg);
                self.fm.pads[usize::from(pad)].out_pods[usize::from(out_index)] = pod;
            }
            Msg::PodLatched { pod, drone, .. } => self.fm_pod_latched(sent, pod, drone),
            Msg::PodDelivered { pad, pod, drone } => self.fm_pod_delivered(sent, pad, pod, drone),
            Msg::PodRemoved { pad, pod } => self.fm_pod_removed(pad, pod),
            Msg::PadStatus {
                pad,
                mode,
                empty_pods,
                backlog_len,
                out_pod,
                marker_fault,
                ..
            } => self.fm_pad_status(pad, mode, empty_pods, backlog_len, out_pod, marker_fault),
            Msg::ShipmentPending { pad, out_drone } => self.fm_shipment_pending(pad, out_drone),
            Msg::DroneStatus {
                drone,
                mode,
                pad,
                dest,
                pod,
                soc,
            } => self.fm_drone_status(sent, drone, mode, pad, dest, pod, soc),
            Msg::MissionRequest {
                drone,
                pad,
                spot,
                pod,
                soc,
                outcome,
            } => self.fm_mission_request(drone, pad, spot, pod, soc, outcome),
            Msg::HoldReport {
                drone,
                pad,
                reason,
                hold_cycles,
                soc,
            } => self.fm_hold_report(drone, pad, reason, hold_cycles, soc),
            _ => {}
        }
        if let (EntityKind::Drone, d) = msg.sender() {
            self.fd_mut(d).yielding = false;
        }
        self.fm_make_room();
    }

    /// A74 (`policy.yield_on_request`): an idle, empty drone on a spot makes room when a drone
    /// holds over its pad, or when a pod there waits for an OUT spot. It moves to a free parking
    /// slot, else (from IN) to a free OUT spot, else away by the rebalancing policy. One move
    /// per pad per pass; a drone is moved again only after it reports.
    pub(crate) fn fm_make_room(&mut self) {
        if !self.cfg.policy.yield_on_request || self.fm.making_room {
            return;
        }
        self.fm.making_room = true;
        for p in 0..self.fm.pads.len() {
            let pad = PadId::try_from(p).expect("pad count fits u16");
            let holders = self
                .fm
                .drones
                .iter()
                .filter(|fd| fd.mode == DroneMode::Holding && fd.pad == pad);
            let (mut held, mut loaded) = (false, false);
            for fd in holders {
                held = true;
                loaded |= fd.pod != NO_POD;
            }
            let free_out = self.fm_free_out(pad).is_some();
            let placing = !free_out && self.fm_placement_waits(pad);
            if !held && !placing {
                continue;
            }
            let idle_on = |spot: fn(Spot) -> bool, fm: &Fm| {
                (0..fm.drones.len()).find(|&i| {
                    let fd = &fm.drones[i];
                    fd.pad == pad
                        && fd.pod == NO_POD
                        && fd.pickup_order == NO_ORDER
                        && !fd.yielding
                        && fd.spot.is_some_and(spot)
                        && (fd.mode == DroneMode::Parked
                            || (fd.mode == DroneMode::OnPad && fd.awaiting_mission))
                })
            };
            let on_in = idle_on(|s| s == Spot::In, &self.fm);
            let on_out = idle_on(Spot::is_out, &self.fm);
            // A loaded drone needs IN; an empty one can use either spot.
            let pick = match (held, loaded, placing) {
                (true, true, _) => on_in,
                (true, false, _) if !free_out => on_in.or(on_out),
                (_, _, true) => on_out,
                _ => None,
            };
            let Some(i) = pick else { continue };
            let d = DroneId::try_from(i).expect("fleet size fits u16");
            // For a waiting pod the last resort is the existing local yield: lift off, let the
            // pod be placed, land again. For a drone waiting to land it would only go round.
            let mut mv = self.fm_room_move(d, pad);
            if mv.is_none() && placing && !held {
                let v = self.view();
                mv =
                    v.fm.policies
                        .drone_rebalancing
                        .yield_for(&v, d, YieldCause::Placement);
            }
            if let Some(m) = mv {
                self.fm_send_mission(d, m);
                self.fd_mut(d).yielding = true;
            }
        }
        self.fm.making_room = false;
    }

    /// Where a yielding drone goes (A74): parking with room, a free OUT spot (from IN; the pad
    /// turns the PARK into a ground hop, V44), or the rebalancing policy's nearest unclaimed pad.
    fn fm_room_move(&mut self, d: DroneId, pad: PadId) -> Option<Mission> {
        let stay = Mission {
            kind: MissionKind::Park,
            dest: pad,
            order: NO_ORDER,
            pickup_order: NO_ORDER,
        };
        let capacity = self.cfg.pads[usize::from(pad)].parking_capacity;
        let in_parking = self
            .fm
            .drones
            .iter()
            .filter(|fd| {
                fd.pad == pad
                    && (fd.spot == Some(Spot::Parking)
                        || (fd.mission == MissionKind::Park && fd.yielding))
            })
            .count();
        if usize::from(capacity) > in_parking {
            return Some(stay);
        }
        let fd = self.fm.drones[usize::from(d)];
        if fd.spot == Some(Spot::In) && self.fm_free_out(pad).is_some() {
            return Some(stay);
        }
        self.fm.refresh_claims();
        let v = self.view();
        v.fm.policies
            .drone_rebalancing
            .yield_for(&v, d, YieldCause::Inbound)
    }

    /// An OUT spot at `pad` with no pod and no drone, as FM last heard.
    fn fm_free_out(&self, pad: PadId) -> Option<usize> {
        let fp = &self.fm.pads[usize::from(pad)];
        (0..fp.out_pods.len()).find(|&k| {
            fp.out_pods[k] == NO_POD
                && !self
                    .fm
                    .drones
                    .iter()
                    .any(|fd| fd.pad == pad && fd.spot == Some(Spot::out(k)))
        })
    }

    /// A shipment or a pod return at `pad` is waiting to be placed on OUT.
    fn fm_placement_waits(&self, pad: PadId) -> bool {
        let fp = &self.fm.pads[usize::from(pad)];
        (fp.backlog_len > 0 && fp.empty_pods > 0)
            || self.fm.pods.iter().any(|r| {
                r.location == PodLocation::PadInventory && r.pad == pad && r.return_dest != NO_PAD
            })
    }

    #[allow(clippy::too_many_arguments)]
    fn fm_pod_placed(
        &mut self,
        sent: TimeMs,
        pad: PadId,
        pod: PodId,
        kind: OrderKind,
        dest: PadId,
        requested: TimeMs,
        deadline: TimeMs,
        payload_kg: f32,
    ) {
        let order_id = self.fm_new_order(pad, pod, kind, dest, requested, deadline, sent);
        self.fm.orders[order_id as usize].payload_kg = payload_kg;
        let rec = &mut self.fm.pods[pod as usize];
        rec.location = PodLocation::PadOut;
        rec.pad = pad;
        rec.order = order_id;
        rec.loaded = kind == OrderKind::Shipment;
        rec.return_dest = NO_PAD;
        self.fm.unassigned.push(order_id);
        self.fm_run_assignment();
        self.fm_rebalance_pods(pad, None);
        // A83: a shipment that was being placed when its check ran is checked again now.
        if kind == OrderKind::Shipment && self.flights_blocked() {
            self.schedule_truck_fallback(requested);
        }
    }

    fn fm_pod_latched(&mut self, sent: TimeMs, pod: PodId, drone: DroneId) {
        let order = self.fm.pod_order(pod);
        let rec = &mut self.fm.pods[pod as usize];
        let pad = rec.pad;
        rec.location = PodLocation::OnDrone;
        rec.pad = NO_PAD;
        rec.drone = drone;
        if pad != NO_PAD {
            for slot in &mut self.fm.pads[usize::from(pad)].out_pods {
                if *slot == pod {
                    *slot = NO_POD;
                }
            }
        }
        let fd = self.fd_mut(drone);
        fd.pod = pod;
        fd.order = order;
        if fd.pickup_order == order {
            fd.pickup_order = NO_ORDER;
        }
        if order != NO_ORDER {
            let o = &mut self.fm.orders[order as usize];
            o.status = OrderStatus::Latched;
            o.latched_t_ms = sent;
            o.drone_id = drone;
            // The pad may have let the drone latch after FM had unassigned or moved the pickup
            // (messages cross); the latch decides. Drop the order from the unassigned list and
            // free any other drone that was since sent for it.
            self.fm.unassigned.retain(|&x| x != order);
            for e in 0..self.fm.drones.len() {
                let other = self.fm.drones[e];
                if e != usize::from(drone) && other.pickup_order == order {
                    let id = DroneId::try_from(e).expect("fleet size fits u16");
                    self.fd_mut(id).pickup_order = NO_ORDER;
                    if matches!(
                        other.mode,
                        DroneMode::Takeoff | DroneMode::EnRoute | DroneMode::Holding
                    ) {
                        self.send(Msg::MissionAssign {
                            drone: id,
                            mission: MissionKind::Reposition,
                            dest: other.dest,
                            order: other.order,
                            pickup_order: NO_ORDER,
                        });
                        self.fd_mut(id).mission = MissionKind::Reposition;
                    }
                }
            }
        }
    }

    fn fm_pod_delivered(&mut self, sent: TimeMs, pad: PadId, pod: PodId, drone: DroneId) {
        let order = self.fm.pod_order(pod);
        let rec = &mut self.fm.pods[pod as usize];
        rec.location = PodLocation::PadIn;
        rec.pad = pad;
        rec.drone = NO_DRONE;
        let fd = self.fd_mut(drone);
        if fd.pod == pod {
            fd.pod = NO_POD;
            fd.order = NO_ORDER;
        }
        if order == NO_ORDER {
            return;
        }
        let o = self.fm.orders[order as usize];
        if o.kind == OrderKind::PodReturn {
            let inbound = &mut self.fm.pads[usize::from(o.dest_pad)].returns_inbound;
            *inbound = inbound.saturating_sub(1);
        }
        let status = if pad == o.dest_pad {
            OrderStatus::Delivered
        } else {
            OrderStatus::Diverted
        };
        self.fm_close_order(order, status, pad, sent, None);
        // The drone is empty now and may take a pod waiting at this pad.
        self.fm_run_assignment();
        self.fm_retry_waiting();
    }

    fn fm_pod_removed(&mut self, pad: PadId, pod: PodId) {
        let rec = &mut self.fm.pods[pod as usize];
        rec.location = PodLocation::PadInventory;
        rec.pad = pad;
        rec.order = NO_ORDER;
        rec.loaded = false;
        self.fm.pads[usize::from(pad)].empty_pods += 1;
        self.fm_rebalance_pods(pad, Some(pod));
    }

    fn fm_rebalance_pods(&mut self, pad: PadId, removed: Option<PodId>) {
        let mut buf = std::mem::take(&mut self.fm.returns_buf);
        buf.clear();
        {
            let v = self.view();
            v.fm.policies
                .pod_rebalancing
                .on_inventory_change(&v, pad, removed, &mut buf);
        }
        for r in &buf {
            let rec = &mut self.fm.pods[r.pod as usize];
            rec.return_dest = r.dest;
            self.fm.pads[usize::from(r.dest)].returns_inbound += 1;
            let from = &mut self.fm.pads[usize::from(r.from)];
            from.empty_pods = from.empty_pods.saturating_sub(1);
            self.send(Msg::PodReturnRequest {
                pad: r.from,
                pod: r.pod,
                dest: r.dest,
            });
        }
        self.fm.returns_buf = buf;
    }

    fn fm_pad_status(
        &mut self,
        pad: PadId,
        mode: PadMode,
        empty_pods: u16,
        backlog_len: u16,
        out_pod: PodId,
        marker_fault: bool,
    ) {
        let fp = &mut self.fm.pads[usize::from(pad)];
        let was = fp.mode;
        let marker_was = fp.marker_fault;
        fp.mode = mode;
        fp.empty_pods = u32::from(empty_pods);
        fp.backlog_len = backlog_len;
        fp.out_pods[0] = out_pod;
        fp.marker_fault = marker_fault;
        if marker_fault && !marker_was {
            self.fm_withdraw_pickups(pad);
        } else if marker_was && !marker_fault {
            self.fm_run_assignment();
        }
        if mode != was {
            if mode == PadMode::Online {
                self.fm_run_assignment();
            } else {
                self.fm_reroute_inbound(pad);
            }
        }
        self.fm_retry_waiting();
    }

    /// 6.6 reroute inbound: every drone whose destination stopped being usable.
    fn fm_reroute_inbound(&mut self, pad: PadId) {
        for d in 0..self.fm.drones.len() {
            let id = DroneId::try_from(d).expect("fleet size fits u16");
            let fd = self.fm.drones[d];
            if fd.dest == pad
                && matches!(
                    fd.mode,
                    DroneMode::Takeoff | DroneMode::EnRoute | DroneMode::Holding
                )
            {
                self.fm_divert(id, pad);
            }
        }
        for o in 0..self.fm.orders.len() {
            let ord = self.fm.orders[o];
            if ord.origin_pad == pad && ord.status == OrderStatus::Assigned {
                self.fm_unassign(OrderId::try_from(o).expect("order count fits u32"));
            }
        }
    }

    /// A35: a pad reports a marker fault. Pickups there assigned to drones that would still
    /// have to fly in and land are withdrawn; drones already in the air keep the holding policy.
    fn fm_withdraw_pickups(&mut self, pad: PadId) {
        for o in 0..self.fm.orders.len() {
            let ord = self.fm.orders[o];
            if ord.origin_pad != pad || ord.status != OrderStatus::Assigned {
                continue;
            }
            let fd = self.fm.drones[usize::from(ord.drone_id)];
            let on_pad = matches!(fd.mode, DroneMode::OnPad | DroneMode::Parked) && fd.pad == pad;
            let airborne = matches!(
                fd.mode,
                DroneMode::Takeoff | DroneMode::EnRoute | DroneMode::Holding
            );
            if !on_pad && !airborne {
                self.fm_unassign(OrderId::try_from(o).expect("order count fits u32"));
            }
        }
    }

    fn fm_shipment_pending(&mut self, pad: PadId, out_drone: DroneId) {
        if out_drone == NO_DRONE {
            return;
        }
        let fd = self.fm.drones[usize::from(out_drone)];
        if fd.mode == DroneMode::Parked && fd.pad == pad && fd.spot.is_some_and(Spot::is_out) {
            self.fm.refresh_claims();
            let mission = {
                let v = self.view();
                v.fm.policies
                    .drone_rebalancing
                    .yield_for(&v, out_drone, YieldCause::Placement)
            };
            if let Some(m) = mission {
                self.fm_send_mission(out_drone, m);
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn fm_drone_status(
        &mut self,
        sent: TimeMs,
        d: DroneId,
        mode: DroneMode,
        pad: PadId,
        dest: PadId,
        pod: PodId,
        soc: f32,
    ) {
        let prev = self.fm.drones[usize::from(d)];
        let has_parking = pad != NO_PAD && self.cfg.pads[usize::from(pad)].parking_capacity > 0;
        let fd = self.fd_mut(d);
        fd.soc = soc;
        fd.pod = pod;
        match mode {
            DroneMode::EnRoute => {
                if prev.pad != NO_PAD {
                    fd.last_pad = prev.pad;
                }
                fd.mode = mode;
                fd.pad = NO_PAD;
                fd.dest = dest;
                fd.depart_t = sent;
                let order = self.fm.pod_order(pod);
                if order != NO_ORDER
                    && self.fm.orders[order as usize].status == OrderStatus::Latched
                {
                    let o = &mut self.fm.orders[order as usize];
                    o.status = OrderStatus::InTransit;
                    o.departed_t_ms = sent;
                }
            }
            DroneMode::OnPad => {
                fd.mode = mode;
                fd.pad = pad;
                fd.last_pad = pad;
                fd.dest = pad;
                fd.mission = MissionKind::None;
                fd.landed_since_hold = true;
                fd.spot = None;
                // 6.1 amendment: a drone landing where a pod waits can take it at once.
                self.fm_run_assignment();
                self.fm_retry_waiting();
            }
            DroneMode::Parked => {
                fd.mode = mode;
                fd.pad = pad;
                fd.last_pad = pad;
                fd.dest = pad;
                fd.mission = MissionKind::Park;
                // Without parking the drone stays on the OUT spot it reported last.
                fd.spot = Some(if has_parking {
                    Spot::Parking
                } else {
                    prev.spot.filter(|s| s.is_out()).unwrap_or(Spot::Out)
                });
                // A pickup assigned while the drone was moving into parking is started now.
                let pickup = fd.pickup_order;
                if pickup != NO_ORDER {
                    let origin = self.fm.orders[pickup as usize].origin_pad;
                    if origin != pad && self.flights_blocked() {
                        // A79: no flight now; the order goes back to be assigned later.
                        self.fm_unassign(pickup);
                    } else {
                        let m = Mission {
                            kind: MissionKind::Pickup,
                            dest: origin,
                            order: NO_ORDER,
                            pickup_order: pickup,
                        };
                        self.fm_send_mission(d, m);
                    }
                }
                self.fm_run_assignment();
                self.fm_retry_waiting();
            }
            DroneMode::Grounded => {
                fd.mode = mode;
                // D13 grounds a drone on IN after a release fault; D14 is the forced landing.
                let reason = if prev.mode == DroneMode::OnPad {
                    AlertCode::ReleaseFault
                } else {
                    AlertCode::DroneGrounded
                };
                let pickup = prev.pickup_order;
                fd.pickup_order = NO_ORDER;
                if pickup != NO_ORDER {
                    self.fm_unassign(pickup);
                }
                if pod != NO_POD {
                    if reason == AlertCode::DroneGrounded {
                        self.fm.pods[pod as usize].location = PodLocation::Lost;
                    }
                    let order = self.fm.pod_order(pod);
                    if order != NO_ORDER {
                        self.fm_close_order(
                            order,
                            OrderStatus::Failed,
                            NO_PAD,
                            NO_TIME,
                            Some(reason),
                        );
                    }
                }
            }
            _ => fd.mode = mode,
        }
    }

    fn fm_mission_request(
        &mut self,
        d: DroneId,
        pad: PadId,
        spot: Spot,
        pod: PodId,
        soc: f32,
        outcome: ServiceOutcome,
    ) {
        let fd = self.fd_mut(d);
        fd.mode = DroneMode::OnPad;
        fd.pad = pad;
        fd.last_pad = pad;
        fd.dest = pad;
        fd.spot = Some(spot);
        fd.pod = pod;
        fd.soc = soc;
        fd.mission = MissionKind::None;
        fd.awaiting_mission = true;
        if outcome == ServiceOutcome::LatchFailed {
            let order = fd.pickup_order;
            fd.pickup_order = NO_ORDER;
            if order != NO_ORDER {
                self.fm.orders[order as usize].excluded_drone = d;
                self.fm_unassign(order);
            }
        }
        self.fm_steal_local_pickup(d, pad);
        self.fm_dispatch_waiting(d);
    }

    /// Decide a mission for a drone that asked for one (6.2, then 6.4). Called again by
    /// `fm_retry_waiting` while the drone is still waiting.
    fn fm_dispatch_waiting(&mut self, d: DroneId) {
        // A79: the drone keeps waiting; the next WEATHER_HOUR that allows flight retries it.
        if self.flights_blocked() {
            return;
        }
        let pad = self.fm.drones[usize::from(d)].pad;
        // A35: a loaded drone whose destination reports a marker fault waits where it is; FM
        // retries it on the PAD_STATUS that clears the fault.
        let pod = self.fm.drones[usize::from(d)].pod;
        let order = self.fm.pod_order(pod);
        if order != NO_ORDER {
            let dest = self.fm.orders[order as usize].dest_pad;
            if self.fm.pads[usize::from(dest)].marker_fault {
                return;
            }
        }
        let mut mission = {
            let v = self.view();
            v.fm.policies.dispatch.mission_for(&v, d)
        };
        if mission.is_none() {
            self.fm_run_assignment();
            let v = self.view();
            mission = v.fm.policies.dispatch.mission_for(&v, d);
        }
        if let Some(m) = mission {
            let ok = {
                let v = self.view();
                let payload = v.payload(d);
                !matches!(m.kind, MissionKind::Deliver | MissionKind::Divert)
                    || v.dispatch_ok(v.drone(d).soc64(), pad, m.dest, payload)
            };
            if ok {
                if m.kind == MissionKind::Divert {
                    // Planned diversion: the order's destination is out of service.
                    let dest = self.fm.orders[m.order as usize].dest_pad;
                    self.alert(
                        AlertCode::Diverted,
                        AlertSeverity::Warning,
                        (EntityKind::FleetManager, 0),
                        dest,
                        d,
                        m.order,
                    );
                }
                self.fm_send_mission(d, m);
                return;
            }
            self.fm_alert_stranded(d, pad, m.order);
        }
        self.fm_place_idle(d);
    }

    /// ENERGY_STRANDED once per stranding (A24); cleared when the drone gets a mission.
    fn fm_alert_stranded(&mut self, d: DroneId, pad: PadId, order: OrderId) {
        if !self.fm.stranded[usize::from(d)] {
            self.fm.stranded[usize::from(d)] = true;
            self.alert(
                AlertCode::EnergyStranded,
                AlertSeverity::Warning,
                (EntityKind::FleetManager, 0),
                pad,
                d,
                order,
            );
        }
    }

    /// A24 amendment: drones left waiting for a mission are retried whenever FM's view
    /// changes, so a blocked drone is never forgotten.
    pub(crate) fn fm_retry_waiting(&mut self) {
        if self.fm.retrying {
            return;
        }
        self.fm.retrying = true;
        for i in 0..self.fm.drones.len() {
            let fd = self.fm.drones[i];
            if fd.awaiting_mission && fd.mode == DroneMode::OnPad {
                self.fm_dispatch_waiting(DroneId::try_from(i).expect("fleet size fits u16"));
            }
        }
        self.fm.retrying = false;
    }

    fn fm_place_idle(&mut self, d: DroneId) {
        self.fm.refresh_claims();
        let idle = {
            let v = self.view();
            v.fm.policies.drone_rebalancing.place_idle(&v, d)
        };
        match idle {
            Some(m) => self.fm_send_mission(d, m),
            None => {
                let pad = self.fm.drones[usize::from(d)].pad;
                self.fm_alert_stranded(d, pad, NO_ORDER);
                let parkable = self.cfg.pads[usize::from(pad)].parking_capacity > 0
                    || self.fm.parks_on_free_out(d);
                if parkable {
                    self.fm_send_mission(
                        d,
                        Mission {
                            kind: MissionKind::Park,
                            dest: pad,
                            order: NO_ORDER,
                            pickup_order: NO_ORDER,
                        },
                    );
                }
            }
        }
    }

    /// A30: a drone free on pad p takes a pickup at p from a drone that is not there yet.
    fn fm_steal_local_pickup(&mut self, d: DroneId, pad: PadId) {
        let fd = self.fm.drones[usize::from(d)];
        if fd.pod != NO_POD || fd.pickup_order != NO_ORDER {
            return;
        }
        let candidate = self.fm.orders.iter().position(|o| {
            o.origin_pad == pad
                && o.status == OrderStatus::Assigned
                && o.drone_id != d
                && o.excluded_drone != d
                && {
                    let other = self.fm.drones[usize::from(o.drone_id)];
                    !(matches!(other.mode, DroneMode::OnPad | DroneMode::Parked)
                        && other.pad == pad)
                }
        });
        let Some(o) = candidate else { return };
        let order = OrderId::try_from(o).expect("order count fits u32");
        let other = self.fm.orders[o].drone_id;
        let fo = self.fm.drones[usize::from(other)];
        self.fd_mut(other).pickup_order = NO_ORDER;
        if fo.mode != DroneMode::Parked {
            let m = Mission {
                kind: MissionKind::Reposition,
                dest: fo.dest,
                order: fo.order,
                pickup_order: NO_ORDER,
            };
            self.send(Msg::MissionAssign {
                drone: other,
                mission: m.kind,
                dest: m.dest,
                order: m.order,
                pickup_order: NO_ORDER,
            });
            self.fd_mut(other).mission = MissionKind::Reposition;
        }
        self.fm_do_assign(order, d);
    }

    fn fm_unassign(&mut self, order: OrderId) {
        let o = &mut self.fm.orders[order as usize];
        if o.status != OrderStatus::Assigned {
            return;
        }
        let prev = o.drone_id;
        o.status = OrderStatus::Created;
        o.drone_id = NO_DRONE;
        if prev != NO_DRONE && self.fm.drones[usize::from(prev)].pickup_order == order {
            self.fd_mut(prev).pickup_order = NO_ORDER;
        }
        let pos = self.fm.unassigned.partition_point(|&x| x < order);
        if self.fm.unassigned.get(pos) != Some(&order) {
            self.fm.unassigned.insert(pos, order);
        }
    }

    pub(crate) fn fm_run_assignment(&mut self) {
        let mut i = 0;
        while i < self.fm.unassigned.len() {
            let order = self.fm.unassigned[i];
            if self.fm.orders[order as usize].status != OrderStatus::Created {
                self.fm.unassigned.remove(i);
                continue;
            }
            let pick = {
                let v = self.view();
                v.fm.policies.assignment.assign(&v, order)
            };
            match pick {
                Some(d) => {
                    self.fm.unassigned.remove(i);
                    self.fm_do_assign(order, d);
                }
                None => i += 1,
            }
        }
    }

    fn fm_do_assign(&mut self, order: OrderId, d: DroneId) {
        let now = self.now;
        let o = &mut self.fm.orders[order as usize];
        o.status = OrderStatus::Assigned;
        o.drone_id = d;
        if o.assigned_t_ms == NO_TIME {
            o.assigned_t_ms = now;
        }
        let (origin, pod) = (o.origin_pad, o.pod_id);
        self.fm.unassigned.retain(|&x| x != order);
        self.fd_mut(d).pickup_order = order;
        self.send(Msg::PickupAssigned {
            pad: origin,
            pod,
            order,
            drone: d,
        });
        let fd = self.fm.drones[usize::from(d)];
        match fd.mode {
            DroneMode::Takeoff | DroneMode::EnRoute | DroneMode::Holding => {
                self.send(Msg::MissionAssign {
                    drone: d,
                    mission: fd.mission,
                    dest: fd.dest,
                    order: fd.order,
                    pickup_order: order,
                });
            }
            DroneMode::Parked => {
                self.fm_send_mission(
                    d,
                    Mission {
                        kind: MissionKind::Pickup,
                        dest: origin,
                        order: NO_ORDER,
                        pickup_order: order,
                    },
                );
            }
            _ => {}
        }
    }

    /// Send a mission, update FM's mirror, and make a drone parked on the destination's OUT
    /// yield (6.4, cause inbound).
    fn fm_send_mission(&mut self, d: DroneId, m: Mission) {
        self.send(Msg::MissionAssign {
            drone: d,
            mission: m.kind,
            dest: m.dest,
            order: m.order,
            pickup_order: m.pickup_order,
        });
        let now = self.now;
        self.fm.stranded[usize::from(d)] = false;
        let fd = self.fd_mut(d);
        let from_pad = fd.pad;
        fd.awaiting_mission = false;
        fd.mission = m.kind;
        fd.dest = m.dest;
        fd.pickup_order = m.pickup_order;
        if m.kind != MissionKind::Park && matches!(fd.mode, DroneMode::OnPad | DroneMode::Parked) {
            fd.mode = DroneMode::Takeoff;
            fd.last_pad = from_pad;
            fd.depart_t = now;
        }
        if m.kind != MissionKind::Park && m.dest != from_pad {
            self.fm_yield_check(m.dest, d);
        }
    }

    fn fm_yield_check(&mut self, pad: PadId, incoming: DroneId) {
        if self.cfg.pads[usize::from(pad)].parking_capacity > 0 {
            return;
        }
        let parked = self.fm.drones.iter().enumerate().position(|(i, fd)| {
            i != usize::from(incoming)
                && fd.mode == DroneMode::Parked
                && fd.pad == pad
                && fd.spot.is_some_and(Spot::is_out)
        });
        let Some(e) = parked else { return };
        let e = DroneId::try_from(e).expect("fleet size fits u16");
        self.fm.refresh_claims();
        let mission = {
            let v = self.view();
            v.fm.policies
                .drone_rebalancing
                .yield_for(&v, e, YieldCause::Inbound)
        };
        if let Some(m) = mission {
            self.fm_send_mission(e, m);
        }
    }

    fn fm_hold_report(
        &mut self,
        d: DroneId,
        pad: PadId,
        reason: HoldReason,
        hold_cycles: u8,
        soc: f32,
    ) {
        let now = self.now;
        let fd = self.fd_mut(d);
        if !(fd.mode == DroneMode::Holding && fd.pad == pad) {
            fd.hold_since = now;
        }
        fd.mode = DroneMode::Holding;
        fd.pad = pad;
        fd.soc = soc;
        fd.landed_since_hold = false;
        if matches!(reason, HoldReason::PcSilent)
            && self.fm.pads[usize::from(pad)].mode == PadMode::Online
        {
            self.fm.pads[usize::from(pad)].mode = PadMode::ControllerFault;
            // 4.5 amendment: the suspicion lapses after one expected outage (FM_PAD_RECHECK).
            let generation = self.fm.pad_gen[usize::from(pad)].wrapping_add(1);
            self.fm.pad_gen[usize::from(pad)] = generation;
            let ms = self.cfg.pad_ops.pc_fault_duration_ms;
            self.timer(
                ms,
                TimerKind::FmPadRecheck,
                EntityKind::FleetManager,
                0,
                pad,
                generation,
            );
            // Reroute diverts this drone too, along with every other drone bound for the pad.
            self.fm_reroute_inbound(pad);
            return;
        }
        let decision = {
            let v = self.view();
            v.fm.policies
                .holding
                .on_hold(&v, d, pad, reason, hold_cycles)
        };
        match decision {
            HoldDecision::Divert => self.fm_divert(d, pad),
            HoldDecision::Continue {
                retry_after_ms,
                limit_t,
            } => {
                #[allow(clippy::cast_possible_truncation)]
                let retry_after_s = s_from_ms(retry_after_ms) as f32;
                self.send(Msg::HoldContinue {
                    drone: d,
                    pad,
                    retry_after_s,
                });
                if let Some(t) = limit_t {
                    let generation = self.fm.hold_gen[usize::from(d)].wrapping_add(1);
                    self.fm.hold_gen[usize::from(d)] = generation;
                    self.timer(
                        t.saturating_sub(self.now),
                        TimerKind::FmHoldLimit,
                        EntityKind::FleetManager,
                        0,
                        d,
                        generation,
                    );
                }
            }
        }
    }

    /// Any message from a PC proves it is up: clear a PC_SILENT suspicion (4.5 amendment).
    fn fm_pad_alive(&mut self, pad: PadId) {
        let fp = &mut self.fm.pads[usize::from(pad)];
        if fp.mode == PadMode::ControllerFault {
            fp.mode = PadMode::Online;
            self.fm.pad_gen[usize::from(pad)] = self.fm.pad_gen[usize::from(pad)].wrapping_add(1);
            self.fm_run_assignment();
        }
        self.fm_retry_waiting();
    }

    /// FM_PAD_RECHECK: the suspected pad gets another chance; a still-silent PC is caught again
    /// by the next drone that tries to land there.
    pub(crate) fn fm_pad_recheck(&mut self, pad: PadId) {
        self.fm_pad_alive(pad);
    }

    pub(crate) fn fm_hold_limit(&mut self, d: DroneId) {
        let fd = self.fm.drones[usize::from(d)];
        if fd.mode == DroneMode::Holding && !fd.landed_since_hold {
            self.fm_divert(d, fd.pad);
        }
    }

    fn fm_divert(&mut self, d: DroneId, failed: PadId) {
        let generation = self.fm.hold_gen[usize::from(d)].wrapping_add(1);
        self.fm.hold_gen[usize::from(d)] = generation;
        let alt = {
            let v = self.view();
            v.fm.policies.diversion.alternate(&v, d, failed)
        };
        let fd = self.fm.drones[usize::from(d)];
        let Some(q) = alt else {
            self.alert(
                AlertCode::NoAlternate,
                AlertSeverity::Critical,
                (EntityKind::FleetManager, 0),
                failed,
                d,
                fd.order,
            );
            if fd.mode == DroneMode::Holding {
                #[allow(clippy::cast_possible_truncation)]
                let retry_after_s = s_from_ms(self.cfg.handshake.hold_retry_ms) as f32;
                self.send(Msg::HoldContinue {
                    drone: d,
                    pad: failed,
                    retry_after_s,
                });
            }
            return;
        };
        if fd.pickup_order != NO_ORDER {
            self.fm_unassign(fd.pickup_order);
        }
        self.alert(
            AlertCode::Diverted,
            AlertSeverity::Warning,
            (EntityKind::FleetManager, 0),
            failed,
            d,
            fd.order,
        );
        self.fm_send_mission(
            d,
            Mission {
                kind: MissionKind::Divert,
                dest: q,
                order: fd.order,
                pickup_order: NO_ORDER,
            },
        );
    }

    /// A new CREATED order with its truck baseline; `placed` is NO_TIME for one that never
    /// had a pod placed (A83).
    #[allow(clippy::too_many_arguments)]
    fn fm_new_order(
        &mut self,
        pad: PadId,
        pod: PodId,
        kind: OrderKind,
        dest: PadId,
        requested: TimeMs,
        deadline: TimeMs,
        placed: TimeMs,
    ) -> OrderId {
        let order_id = OrderId::try_from(self.fm.orders.len()).expect("order count fits u32");
        let (truck_delivered, truck_cost) =
            truck_baseline(&self.cfg, &self.net, pad, dest, requested);
        self.fm.orders.push(Order {
            order_id,
            kind,
            status: OrderStatus::Created,
            pod_id: pod,
            origin_pad: pad,
            dest_pad: dest,
            delivered_pad: NO_PAD,
            drone_id: NO_DRONE,
            payload_kg: 0.0,
            requested_t_ms: requested,
            placed_t_ms: placed,
            deadline_t_ms: deadline,
            assigned_t_ms: NO_TIME,
            latched_t_ms: NO_TIME,
            departed_t_ms: NO_TIME,
            delivered_t_ms: NO_TIME,
            truck_delivered_t_ms: truck_delivered,
            truck_cost,
            fail_reason: None,
            latch_start_t_ms: NO_TIME,
            flight_ms: 0,
            hold_ms: 0,
            excluded_drone: NO_DRONE,
        });
        order_id
    }

    fn fm_close_order(
        &mut self,
        order: OrderId,
        status: OrderStatus,
        pad: PadId,
        t: TimeMs,
        reason: Option<AlertCode>,
    ) {
        let o = &mut self.fm.orders[order as usize];
        if o.is_closed() {
            return;
        }
        o.status = status;
        o.delivered_pad = pad;
        o.delivered_t_ms = t;
        o.fail_reason = reason;
        let o = *o;
        if status == OrderStatus::Trucked {
            self.metrics.shipment_trucked(o.requested_t_ms);
        } else if o.kind == OrderKind::Shipment {
            let closed = ClosedOrder {
                requested_ms: o.requested_t_ms,
                placed_ms: o.placed_t_ms,
                latched_ms: (o.latched_t_ms != NO_TIME).then_some(o.latched_t_ms),
                delivered_ms: (t != NO_TIME).then_some(t),
                truck_delivered_ms: o.truck_delivered_t_ms,
                truck_cost: o.truck_cost,
                split: o.split(),
            };
            self.metrics.shipment_closed(
                &closed,
                status == OrderStatus::Delivered,
                status == OrderStatus::Diverted,
            );
        }
        self.write_delivery_row(&o);
    }

    fn write_delivery_row(&mut self, o: &Order) {
        let id = |v: u64, none: u64| OptNum(v, none);
        let t = |v: TimeMs| OptNum(v, NO_TIME);
        let secs = |end: TimeMs| {
            OptSecs(if end == NO_TIME {
                None
            } else {
                Some(end - o.requested_t_ms)
            })
        };
        let reason = o.fail_reason.map_or("", |r| r.name());
        let split = o.split();
        let part = |f: fn(&DoorToDoorSplit) -> TimeMs| OptSecs(split.as_ref().map(f));
        self.out.delivery_line(format_args!(
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{:.4},{},{},{},{}\n",
            o.order_id,
            o.kind.name(),
            o.status.name(),
            reason,
            o.origin_pad,
            o.dest_pad,
            id(u64::from(o.delivered_pad), u64::from(NO_PAD)),
            id(u64::from(o.drone_id), u64::from(NO_DRONE)),
            o.requested_t_ms,
            t(o.placed_t_ms),
            t(o.latched_t_ms),
            t(o.delivered_t_ms),
            secs(o.delivered_t_ms),
            secs(o.truck_delivered_t_ms),
            o.truck_cost,
            part(|s| s.pod_wait_ms),
            part(|s| s.queue_ms),
            part(|s| s.pad_service_ms),
            part(|s| s.flight_ms),
        ));
    }

    /// TRUCK_FALLBACK (A83): with dispatch blocked, every shipment that has waited the
    /// timeout since it was requested or since the block began, whichever is later, goes by
    /// truck from now. A shipment still in a pad's backlog leaves it; one whose pod waits on
    /// a free OUT spot with no drone assigned is unpacked into the truck and the pod goes back
    /// to inventory empty. Operations do this on the ground, outside the radio protocol (like
    /// the demand generator), and update FM's records directly.
    pub(crate) fn weather_truck_fallback(&mut self) {
        let Some(since) = self.weather.and_then(|w| w.blocked_since) else {
            return;
        };
        let w = self.cfg.weather.as_ref().expect("weather configured");
        if w.hold_policy != crate::ids::WeatherHoldPolicy::TruckFallback
            || self.now < since + w.truck_fallback_timeout_ms
        {
            return;
        }
        let cutoff = self.now - w.truck_fallback_timeout_ms;
        for p in 0..self.pads.len() {
            let pad = PadId::try_from(p).expect("pad count fits u16");
            if self.pads[p].mode == PadMode::ControllerFault {
                continue;
            }
            for ship in self.pc_truck_backlog(pad, cutoff) {
                let order = self.fm_new_order(
                    pad,
                    NO_POD,
                    OrderKind::Shipment,
                    ship.dest,
                    ship.requested,
                    ship.deadline,
                    NO_TIME,
                );
                self.fm_close_trucked(order);
            }
            let waiting: Vec<OrderId> = self
                .fm
                .orders
                .iter()
                .filter(|o| {
                    o.status == OrderStatus::Created
                        && o.kind == OrderKind::Shipment
                        && o.origin_pad == pad
                        && o.requested_t_ms <= cutoff
                })
                .map(|o| o.order_id)
                .collect();
            for order in waiting {
                let pod = self.fm.orders[order as usize].pod_id;
                if self.pc_truck_out_pod(pad, pod) {
                    for slot in &mut self.fm.pads[p].out_pods {
                        if *slot == pod {
                            *slot = NO_POD;
                        }
                    }
                    self.fm_close_trucked(order);
                    self.fm_pod_removed(pad, pod);
                }
            }
            self.pad_status(pad);
            self.pad_tick(pad);
        }
    }

    /// A trucked order leaves now and arrives after the same dwell as the truck baseline.
    fn fm_close_trucked(&mut self, order: OrderId) {
        let o = self.fm.orders[order as usize];
        let (arrives, _) = truck_baseline(&self.cfg, &self.net, o.origin_pad, o.dest_pad, self.now);
        self.fm.unassigned.retain(|&x| x != order);
        self.fm_close_order(order, OrderStatus::Trucked, o.dest_pad, arrives, None);
    }

    pub(crate) fn fm_write_open_orders(&mut self) {
        for i in 0..self.fm.orders.len() {
            let o = self.fm.orders[i];
            if !o.is_closed() {
                self.metrics.open_at_end += 1;
                self.write_delivery_row(&o);
            }
        }
    }
}

/// CSV field: the number, or empty for its NONE value.
struct OptNum(u64, u64);

impl std::fmt::Display for OptNum {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.0 == self.1 {
            Ok(())
        } else {
            write!(f, "{}", self.0)
        }
    }
}

struct OptSecs(Option<TimeMs>);

impl std::fmt::Display for OptSecs {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.0 {
            Some(ms) => write!(f, "{:.3}", s_from_ms(ms)),
            None => Ok(()),
        }
    }
}
