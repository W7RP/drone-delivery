//! DES events and messages (spec 4.2, 5.1), their queue ordering (5.2) and their packed
//! encoding in `events.bin` (9.4). Every type here is `Copy`: the hot loop never allocates.

use std::cmp::Ordering;

use crate::ids::{
    DroneId, DroneMode, EntityKind, EventClass, FaultKind, HoldReason, LaneId, MissionKind,
    OrderId, OrderKind, PadId, PadMode, PodId, ServiceOutcome, Spot, TimeMs, TimerKind,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Msg {
    LandingRequest {
        drone: DroneId,
        pad: PadId,
        pod: PodId,
        soc: f32,
    },
    LandingCancel {
        drone: DroneId,
        pad: PadId,
    },
    ApproachAbort {
        drone: DroneId,
        pad: PadId,
        reason: HoldReason,
    },
    Landed {
        drone: DroneId,
        pad: PadId,
        spot: Spot,
        pod: PodId,
        soc: f32,
    },
    ReleaseReport {
        drone: DroneId,
        pad: PadId,
        latch_open: bool,
    },
    LatchReport {
        drone: DroneId,
        pad: PadId,
        pod: PodId,
        locked: bool,
    },
    DepartRequest {
        drone: DroneId,
        pad: PadId,
        dest: PadId,
        mission: MissionKind,
    },
    Departed {
        drone: DroneId,
        pad: PadId,
        spot: Spot,
    },
    LandingGrant {
        pad: PadId,
        drone: DroneId,
        spot: Spot,
    },
    LandingHold {
        pad: PadId,
        drone: DroneId,
        reason: HoldReason,
        queue_pos: u16,
    },
    LandingRefused {
        pad: PadId,
        drone: DroneId,
        reason: HoldReason,
    },
    ReleaseCmd {
        pad: PadId,
        drone: DroneId,
        attempt: u8,
    },
    HopCmd {
        pad: PadId,
        drone: DroneId,
    },
    LatchCmd {
        pad: PadId,
        drone: DroneId,
        pod: PodId,
        attempt: u8,
    },
    ChargeStart {
        pad: PadId,
        drone: DroneId,
        dwell_s: f32,
    },
    ServiceComplete {
        pad: PadId,
        drone: DroneId,
        outcome: ServiceOutcome,
        spot: Spot,
        pod: PodId,
        soc: f32,
    },
    DepartClear {
        pad: PadId,
        drone: DroneId,
    },
    DroneStatus {
        drone: DroneId,
        mode: DroneMode,
        pad: PadId,
        dest: PadId,
        pod: PodId,
        soc: f32,
    },
    MissionRequest {
        drone: DroneId,
        pad: PadId,
        spot: Spot,
        pod: PodId,
        soc: f32,
        outcome: ServiceOutcome,
    },
    HoldReport {
        drone: DroneId,
        pad: PadId,
        reason: HoldReason,
        hold_cycles: u8,
        soc: f32,
    },
    MissionAssign {
        drone: DroneId,
        mission: MissionKind,
        dest: PadId,
        order: OrderId,
        pickup_order: OrderId,
    },
    HoldContinue {
        drone: DroneId,
        pad: PadId,
        retry_after_s: f32,
    },
    PodPlaced {
        pad: PadId,
        pod: PodId,
        kind: OrderKind,
        dest: PadId,
        requested: TimeMs,
        deadline: TimeMs,
        payload_kg: f32,
    },
    PodLatched {
        pad: PadId,
        pod: PodId,
        drone: DroneId,
    },
    PodDelivered {
        pad: PadId,
        pod: PodId,
        drone: DroneId,
    },
    PodRemoved {
        pad: PadId,
        pod: PodId,
    },
    PadStatus {
        pad: PadId,
        mode: PadMode,
        empty_pods: u16,
        backlog_len: u16,
        in_drone: DroneId,
        out_drone: DroneId,
        out_pod: PodId,
    },
    ShipmentPending {
        pad: PadId,
        out_drone: DroneId,
    },
    PickupAssigned {
        pad: PadId,
        pod: PodId,
        order: OrderId,
        drone: DroneId,
    },
    PodReturnRequest {
        pad: PadId,
        pod: PodId,
        dest: PadId,
    },
}

/// Where a message is delivered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Receiver {
    Pad(PadId),
    Drone(DroneId),
    FleetManager,
}

impl Msg {
    /// Index in the `kind` enum of `events.schema.json`.
    pub fn kind_code(&self) -> u16 {
        match self {
            Msg::LandingRequest { .. } => 0,
            Msg::LandingCancel { .. } => 1,
            Msg::ApproachAbort { .. } => 2,
            Msg::Landed { .. } => 3,
            Msg::ReleaseReport { .. } => 4,
            Msg::LatchReport { .. } => 5,
            Msg::DepartRequest { .. } => 6,
            Msg::Departed { .. } => 7,
            Msg::LandingGrant { .. } => 8,
            Msg::LandingHold { .. } => 9,
            Msg::LandingRefused { .. } => 10,
            Msg::ReleaseCmd { .. } => 11,
            Msg::HopCmd { .. } => 12,
            Msg::LatchCmd { .. } => 13,
            Msg::ChargeStart { .. } => 14,
            Msg::ServiceComplete { .. } => 15,
            Msg::DepartClear { .. } => 16,
            Msg::DroneStatus { .. } => 17,
            Msg::MissionRequest { .. } => 18,
            Msg::HoldReport { .. } => 19,
            Msg::MissionAssign { .. } => 20,
            Msg::HoldContinue { .. } => 21,
            Msg::PodPlaced { .. } => 22,
            Msg::PodLatched { .. } => 23,
            Msg::PodDelivered { .. } => 24,
            Msg::PodRemoved { .. } => 25,
            Msg::PadStatus { .. } => 26,
            Msg::ShipmentPending { .. } => 27,
            Msg::PickupAssigned { .. } => 28,
            Msg::PodReturnRequest { .. } => 29,
        }
    }

    pub fn sender(&self) -> (EntityKind, u16) {
        match *self {
            Msg::LandingRequest { drone, .. }
            | Msg::LandingCancel { drone, .. }
            | Msg::ApproachAbort { drone, .. }
            | Msg::Landed { drone, .. }
            | Msg::ReleaseReport { drone, .. }
            | Msg::LatchReport { drone, .. }
            | Msg::DepartRequest { drone, .. }
            | Msg::Departed { drone, .. }
            | Msg::DroneStatus { drone, .. }
            | Msg::MissionRequest { drone, .. }
            | Msg::HoldReport { drone, .. } => (EntityKind::Drone, drone),
            Msg::LandingGrant { pad, .. }
            | Msg::LandingHold { pad, .. }
            | Msg::LandingRefused { pad, .. }
            | Msg::ReleaseCmd { pad, .. }
            | Msg::HopCmd { pad, .. }
            | Msg::LatchCmd { pad, .. }
            | Msg::ChargeStart { pad, .. }
            | Msg::ServiceComplete { pad, .. }
            | Msg::DepartClear { pad, .. }
            | Msg::PodPlaced { pad, .. }
            | Msg::PodLatched { pad, .. }
            | Msg::PodDelivered { pad, .. }
            | Msg::PodRemoved { pad, .. }
            | Msg::PadStatus { pad, .. }
            | Msg::ShipmentPending { pad, .. } => (EntityKind::Pad, pad),
            Msg::MissionAssign { .. }
            | Msg::HoldContinue { .. }
            | Msg::PickupAssigned { .. }
            | Msg::PodReturnRequest { .. } => (EntityKind::FleetManager, 0),
        }
    }

    pub fn receiver(&self) -> Receiver {
        match *self {
            Msg::LandingRequest { pad, .. }
            | Msg::LandingCancel { pad, .. }
            | Msg::ApproachAbort { pad, .. }
            | Msg::Landed { pad, .. }
            | Msg::ReleaseReport { pad, .. }
            | Msg::LatchReport { pad, .. }
            | Msg::DepartRequest { pad, .. }
            | Msg::Departed { pad, .. }
            | Msg::PickupAssigned { pad, .. }
            | Msg::PodReturnRequest { pad, .. } => Receiver::Pad(pad),
            Msg::LandingGrant { drone, .. }
            | Msg::LandingHold { drone, .. }
            | Msg::LandingRefused { drone, .. }
            | Msg::ReleaseCmd { drone, .. }
            | Msg::HopCmd { drone, .. }
            | Msg::LatchCmd { drone, .. }
            | Msg::ChargeStart { drone, .. }
            | Msg::ServiceComplete { drone, .. }
            | Msg::DepartClear { drone, .. }
            | Msg::MissionAssign { drone, .. }
            | Msg::HoldContinue { drone, .. } => Receiver::Drone(drone),
            _ => Receiver::FleetManager,
        }
    }

    /// The D <-> PC radio link is the only lossy one (spec 4.1).
    pub fn is_radio(&self) -> bool {
        matches!(
            (self.sender().0, self.receiver()),
            (EntityKind::Drone, Receiver::Pad(_)) | (EntityKind::Pad, Receiver::Drone(_))
        )
    }

    /// Payload fields in schema `required` order at their `x-type` widths (spec 9.4).
    pub fn encode(&self, a: &mut Args) {
        match *self {
            Msg::LandingRequest {
                drone,
                pad,
                pod,
                soc,
            } => a.u16(drone).u16(pad).u32(pod).f32(soc),
            Msg::LandingCancel { drone, pad } => a.u16(drone).u16(pad),
            Msg::ApproachAbort { drone, pad, reason } => a.u16(drone).u16(pad).u8(reason.code()),
            Msg::Landed {
                drone,
                pad,
                spot,
                pod,
                soc,
            } => a.u16(drone).u16(pad).u8(spot.code()).u32(pod).f32(soc),
            Msg::ReleaseReport {
                drone,
                pad,
                latch_open,
            } => a.u16(drone).u16(pad).flag(latch_open),
            Msg::LatchReport {
                drone,
                pad,
                pod,
                locked,
            } => a.u16(drone).u16(pad).u32(pod).flag(locked),
            Msg::DepartRequest {
                drone,
                pad,
                dest,
                mission,
            } => a.u16(drone).u16(pad).u16(dest).u8(mission.code()),
            Msg::Departed { drone, pad, spot } => a.u16(drone).u16(pad).u8(spot.code()),
            Msg::LandingGrant { pad, drone, spot } => a.u16(pad).u16(drone).u8(spot.code()),
            Msg::LandingHold {
                pad,
                drone,
                reason,
                queue_pos,
            } => a.u16(pad).u16(drone).u8(reason.code()).u16(queue_pos),
            Msg::LandingRefused { pad, drone, reason } => a.u16(pad).u16(drone).u8(reason.code()),
            Msg::ReleaseCmd {
                pad,
                drone,
                attempt,
            } => a.u16(pad).u16(drone).u8(attempt),
            Msg::HopCmd { pad, drone } | Msg::DepartClear { pad, drone } => a.u16(pad).u16(drone),
            Msg::LatchCmd {
                pad,
                drone,
                pod,
                attempt,
            } => a.u16(pad).u16(drone).u32(pod).u8(attempt),
            Msg::ChargeStart {
                pad,
                drone,
                dwell_s,
            } => a.u16(pad).u16(drone).f32(dwell_s),
            Msg::ServiceComplete {
                pad,
                drone,
                outcome,
                spot,
                pod,
                soc,
            } => a
                .u16(pad)
                .u16(drone)
                .u8(outcome.code())
                .u8(spot.code())
                .u32(pod)
                .f32(soc),
            Msg::DroneStatus {
                drone,
                mode,
                pad,
                dest,
                pod,
                soc,
            } => a
                .u16(drone)
                .u8(mode.code())
                .u16(pad)
                .u16(dest)
                .u32(pod)
                .f32(soc),
            Msg::MissionRequest {
                drone,
                pad,
                spot,
                pod,
                soc,
                outcome,
            } => a
                .u16(drone)
                .u16(pad)
                .u8(spot.code())
                .u32(pod)
                .f32(soc)
                .u8(outcome.code()),
            Msg::HoldReport {
                drone,
                pad,
                reason,
                hold_cycles,
                soc,
            } => a
                .u16(drone)
                .u16(pad)
                .u8(reason.code())
                .u8(hold_cycles)
                .f32(soc),
            Msg::MissionAssign {
                drone,
                mission,
                dest,
                order,
                pickup_order,
            } => a
                .u16(drone)
                .u8(mission.code())
                .u16(dest)
                .u32(order)
                .u32(pickup_order),
            Msg::HoldContinue {
                drone,
                pad,
                retry_after_s,
            } => a.u16(drone).u16(pad).f32(retry_after_s),
            Msg::PodPlaced {
                pad,
                pod,
                kind,
                dest,
                requested,
                deadline,
                payload_kg,
            } => a
                .u16(pad)
                .u32(pod)
                .u8(kind.code())
                .u16(dest)
                .u64(requested)
                .u64(deadline)
                .f32(payload_kg),
            Msg::PodLatched { pad, pod, drone } | Msg::PodDelivered { pad, pod, drone } => {
                a.u16(pad).u32(pod).u16(drone)
            }
            Msg::PodRemoved { pad, pod } => a.u16(pad).u32(pod),
            Msg::PadStatus {
                pad,
                mode,
                empty_pods,
                backlog_len,
                in_drone,
                out_drone,
                out_pod,
            } => a
                .u16(pad)
                .u8(mode.code())
                .u16(empty_pods)
                .u16(backlog_len)
                .u16(in_drone)
                .u16(out_drone)
                .u32(out_pod),
            Msg::ShipmentPending { pad, out_drone } => a.u16(pad).u16(out_drone),
            Msg::PickupAssigned {
                pad,
                pod,
                order,
                drone,
            } => a.u16(pad).u32(pod).u32(order).u16(drone),
            Msg::PodReturnRequest { pad, pod, dest } => a.u16(pad).u32(pod).u16(dest),
        };
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Ev {
    Message {
        seq: u32,
        sent: TimeMs,
        msg: Msg,
    },
    ShipmentRequest {
        flow: u32,
        pad: PadId,
        dest: PadId,
        deadline: TimeMs,
    },
    PodPlaceDone {
        pad: PadId,
        pod: PodId,
    },
    PodRemoveDone {
        pad: PadId,
        pod: PodId,
    },
    TakeoffDone {
        drone: DroneId,
        pad: PadId,
    },
    LaneDone {
        drone: DroneId,
        lane: LaneId,
    },
    ApproachDone {
        drone: DroneId,
        pad: PadId,
        spot: Spot,
    },
    GoAroundDone {
        drone: DroneId,
        pad: PadId,
    },
    HopDone {
        drone: DroneId,
        pad: PadId,
    },
    ReleaseDone {
        drone: DroneId,
        pad: PadId,
    },
    LatchDone {
        drone: DroneId,
        pad: PadId,
    },
    /// `random` marks faults drawn from `pad_ops.pc_fault_rate_per_h` (they schedule their end).
    FaultStart {
        kind: FaultKind,
        pad: PadId,
        random: bool,
    },
    FaultEnd {
        kind: FaultKind,
        pad: PadId,
    },
    /// A technician has repaired a pad grounded by a release fault, and its drone (A69).
    RepairDone {
        pad: PadId,
        drone: DroneId,
    },
    Timer {
        timer: TimerKind,
        owner_kind: EntityKind,
        owner_id: u16,
        subject: u16,
        generation: u32,
    },
}

impl Ev {
    pub fn class(&self) -> EventClass {
        match self {
            Ev::Message { .. } => EventClass::Message,
            Ev::ShipmentRequest { .. } => EventClass::Arrival,
            Ev::FaultStart { .. } | Ev::FaultEnd { .. } | Ev::RepairDone { .. } => {
                EventClass::StateChange
            }
            Ev::Timer { .. } => EventClass::Timer,
            _ => EventClass::Completion,
        }
    }

    pub fn kind_code(&self) -> u16 {
        match self {
            Ev::Message { msg, .. } => msg.kind_code(),
            Ev::ShipmentRequest { .. } => 30,
            Ev::PodPlaceDone { .. } => 31,
            Ev::PodRemoveDone { .. } => 32,
            Ev::TakeoffDone { .. } => 33,
            Ev::LaneDone { .. } => 34,
            Ev::ApproachDone { .. } => 35,
            Ev::GoAroundDone { .. } => 36,
            Ev::HopDone { .. } => 37,
            Ev::ReleaseDone { .. } => 38,
            Ev::LatchDone { .. } => 39,
            Ev::FaultStart { .. } => 40,
            Ev::FaultEnd { .. } => 41,
            Ev::Timer { .. } => 42,
            Ev::RepairDone { .. } => 43,
        }
    }

    pub fn encode(&self, a: &mut Args) {
        match *self {
            Ev::Message { msg, .. } => msg.encode(a),
            Ev::ShipmentRequest {
                pad,
                dest,
                deadline,
                ..
            } => {
                a.u16(pad).u16(dest).u64(deadline);
            }
            Ev::PodPlaceDone { pad, pod } | Ev::PodRemoveDone { pad, pod } => {
                a.u16(pad).u32(pod);
            }
            Ev::TakeoffDone { drone, pad }
            | Ev::GoAroundDone { drone, pad }
            | Ev::HopDone { drone, pad }
            | Ev::ReleaseDone { drone, pad }
            | Ev::LatchDone { drone, pad } => {
                a.u16(drone).u16(pad);
            }
            Ev::LaneDone { drone, lane } => {
                a.u16(drone).u32(lane);
            }
            Ev::ApproachDone { drone, pad, spot } => {
                a.u16(drone).u16(pad).u8(spot.code());
            }
            Ev::FaultStart { kind, pad, .. } | Ev::FaultEnd { kind, pad } => {
                a.u8(kind.code()).u16(pad);
            }
            Ev::RepairDone { pad, drone } => {
                a.u16(pad).u16(drone);
            }
            Ev::Timer {
                timer,
                owner_kind,
                owner_id,
                subject,
                generation,
            } => {
                a.u8(timer.code())
                    .u8(owner_kind.code())
                    .u16(owner_id)
                    .u16(subject)
                    .u32(generation);
            }
        }
    }
}

/// Packed little-endian argument area of an `events.bin` record (32 bytes, zero padded).
#[derive(Default)]
pub struct Args {
    pub buf: [u8; 32],
    len: usize,
}

impl Args {
    fn put(&mut self, bytes: &[u8]) -> &mut Self {
        let end = self.len + bytes.len();
        debug_assert!(end <= self.buf.len(), "event args exceed 32 bytes");
        self.buf[self.len..end].copy_from_slice(bytes);
        self.len = end;
        self
    }
    pub fn u8(&mut self, v: u8) -> &mut Self {
        self.put(&[v])
    }
    pub fn flag(&mut self, v: bool) -> &mut Self {
        self.put(&[u8::from(v)])
    }
    pub fn u16(&mut self, v: u16) -> &mut Self {
        self.put(&v.to_le_bytes())
    }
    pub fn u32(&mut self, v: u32) -> &mut Self {
        self.put(&v.to_le_bytes())
    }
    pub fn u64(&mut self, v: u64) -> &mut Self {
        self.put(&v.to_le_bytes())
    }
    pub fn f32(&mut self, v: f32) -> &mut Self {
        self.put(&v.to_le_bytes())
    }
}

/// A scheduled event; the queue pops the smallest `(t, class, seq)` first (spec 5.2).
#[derive(Clone, Copy, Debug)]
pub struct Entry {
    pub t: TimeMs,
    pub class: u8,
    pub seq: u64,
    pub ev: Ev,
}

impl PartialEq for Entry {
    fn eq(&self, other: &Self) -> bool {
        (self.t, self.class, self.seq) == (other.t, other.class, other.seq)
    }
}

impl Eq for Entry {}

impl Ord for Entry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reversed: BinaryHeap is a max-heap.
        (other.t, other.class, other.seq).cmp(&(self.t, self.class, self.seq))
    }
}

impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Transition codes of spec 9.4: letter (M 1, P 2, V 3, D 4) * 100 + row number.
pub mod tcode {
    pub const fn m(n: u16) -> u16 {
        100 + n
    }
    pub const fn p(n: u16) -> u16 {
        200 + n
    }
    pub const fn v(n: u16) -> u16 {
        300 + n
    }
    pub const fn d(n: u16) -> u16 {
        400 + n
    }

    /// "V06" style label, for tests and debugging.
    pub fn label(code: u16) -> String {
        let letter = match code / 100 {
            1 => 'M',
            2 => 'P',
            3 => 'V',
            4 => 'D',
            _ => '?',
        };
        format!("{letter}{:02}", code % 100)
    }
}

/// `kind` value of a transition record in `events.bin`.
pub const TRANSITION_KIND: u16 = 0x8000;
pub const NO_STATE: u8 = 255;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum LogStatus {
    Processed = 0,
    Stale = 1,
    Dropped = 2,
    Lost = 3,
    Deferred = 4,
}
