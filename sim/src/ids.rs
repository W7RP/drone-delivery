//! ID types, NONE sentinels and the enums of `schemas/common.schema.json`.
//!
//! Enum discriminants are the spec codes (zero-based position in the schema enum list), and
//! `name()` is the schema string. `tests/schema_consistency.rs` checks both against the schema.

pub type PadId = u16;
pub type DroneId = u16;
pub type PodId = u32;
pub type OrderId = u32;
pub type LaneId = u32;
/// Simulation time in integer milliseconds since run start (spec section 0).
pub type TimeMs = u64;

pub const NO_PAD: PadId = PadId::MAX;
pub const NO_DRONE: DroneId = DroneId::MAX;
pub const NO_POD: PodId = PodId::MAX;
pub const NO_ORDER: OrderId = OrderId::MAX;
pub const NO_LANE: LaneId = LaneId::MAX;
pub const NO_TIME: TimeMs = TimeMs::MAX;

macro_rules! code_enum {
    ($(#[$meta:meta])* $name:ident { $($variant:ident = $text:literal),+ $(,)? }) => {
        $(#[$meta])*
        #[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
        #[repr(u8)]
        pub enum $name { $($variant),+ }

        impl $name {
            pub const ALL: &'static [$name] = &[$($name::$variant),+];

            pub fn name(self) -> &'static str {
                match self { $($name::$variant => $text),+ }
            }

            pub fn code(self) -> u8 {
                self as u8
            }

            pub fn from_name(text: &str) -> Option<Self> {
                match text { $($text => Some($name::$variant),)+ _ => None }
            }
        }
    };
}

code_enum!(PadMode { Online = "ONLINE", ControllerFault = "CONTROLLER_FAULT", OutOfService = "OUT_OF_SERVICE" });
code_enum!(Spot { In = "IN", Out = "OUT", Parking = "PARKING" });
code_enum!(VisitState {
    Held = "HELD", Approach = "APPROACH", Releasing = "RELEASING", VerifyRelease = "VERIFY_RELEASE",
    WaitOut = "WAIT_OUT", Hopping = "HOPPING", Latching = "LATCHING", VerifyLock = "VERIFY_LOCK",
    Charging = "CHARGING", Ready = "READY", Departing = "DEPARTING", DroneSilent = "DRONE_SILENT",
    Closed = "CLOSED",
});
code_enum!(VisitOutcome {
    Open = "OPEN", Completed = "COMPLETED", ApproachAborted = "APPROACH_ABORTED",
    Cancelled = "CANCELLED", Refused = "REFUSED", LostContact = "LOST_CONTACT",
    DroneGrounded = "DRONE_GROUNDED",
});
code_enum!(DroneMode {
    Parked = "PARKED", OnPad = "ON_PAD", Takeoff = "TAKEOFF", EnRoute = "EN_ROUTE",
    Holding = "HOLDING", Approach = "APPROACH", GoAround = "GO_AROUND", Hopping = "HOPPING",
    Grounded = "GROUNDED",
});
code_enum!(MissionKind {
    None = "NONE", Deliver = "DELIVER", Pickup = "PICKUP", Reposition = "REPOSITION",
    YieldLocal = "YIELD_LOCAL", Divert = "DIVERT", Park = "PARK",
});
code_enum!(OrderKind { Shipment = "SHIPMENT", PodReturn = "POD_RETURN" });
code_enum!(OrderStatus {
    Created = "CREATED", Assigned = "ASSIGNED", Latched = "LATCHED", InTransit = "IN_TRANSIT",
    Delivered = "DELIVERED", Diverted = "DIVERTED", Failed = "FAILED",
});
code_enum!(PodLocation {
    PadInventory = "PAD_INVENTORY", PadOut = "PAD_OUT", PadIn = "PAD_IN", OnDrone = "ON_DRONE",
    Lost = "LOST",
});
code_enum!(HoldReason {
    InOccupied = "IN_OCCUPIED", AwaitOut = "AWAIT_OUT", MarkerNotDetected = "MARKER_NOT_DETECTED",
    PcSilent = "PC_SILENT", OutOfService = "OUT_OF_SERVICE",
});
code_enum!(ServiceOutcome {
    Normal = "NORMAL", NoPod = "NO_POD", ReleaseFailed = "RELEASE_FAILED", LatchFailed = "LATCH_FAILED",
});
code_enum!(AlertCode {
    ReleaseFault = "RELEASE_FAULT", LatchFault = "LATCH_FAULT", PcSilent = "PC_SILENT",
    DroneSilent = "DRONE_SILENT", Diverted = "DIVERTED", NoAlternate = "NO_ALTERNATE",
    DroneGrounded = "DRONE_GROUNDED", EnergyStranded = "ENERGY_STRANDED",
    PadOutOfService = "PAD_OUT_OF_SERVICE",
});
code_enum!(AlertSeverity { Info = "INFO", Warning = "WARNING", Critical = "CRITICAL" });
code_enum!(EntityKind { Pad = "PAD", Drone = "DRONE", FleetManager = "FLEET_MANAGER" });
code_enum!(EventClass {
    StateChange = "STATE_CHANGE", Completion = "COMPLETION", Message = "MESSAGE",
    Arrival = "ARRIVAL", Timer = "TIMER",
});
code_enum!(TimerKind {
    PcLandingTimeout = "PC_LANDING_TIMEOUT", PcResponseTimeout = "PC_RESPONSE_TIMEOUT",
    PcVerifyReleaseDone = "PC_VERIFY_RELEASE_DONE", PcVerifyLockDone = "PC_VERIFY_LOCK_DONE",
    PcOutWaitTimeout = "PC_OUT_WAIT_TIMEOUT", PcChargeDone = "PC_CHARGE_DONE",
    DResponseTimeout = "D_RESPONSE_TIMEOUT", DHoldRetry = "D_HOLD_RETRY",
    DPcSilenceAlert = "D_PC_SILENCE_ALERT", DEnergyReserve = "D_ENERGY_RESERVE",
    FmHoldLimit = "FM_HOLD_LIMIT", EndOfRun = "END_OF_RUN",
});
code_enum!(FlightSource { Des = "DES", Px4Sitl = "PX4_SITL" });
code_enum!(ServiceMode { Auto = "AUTO", Manual = "MANUAL" });
code_enum!(ReturnPolicy { KeepLocal = "KEEP_LOCAL", ReturnHome = "RETURN_HOME" });
code_enum!(ChargingMode { OutContacts = "OUT_CONTACTS", HubOnly = "HUB_ONLY" });
code_enum!(OutEmptyPolicy { Wait = "WAIT", DepartEmpty = "DEPART_EMPTY" });
code_enum!(InOccupiedAction { Hold = "HOLD", Divert = "DIVERT" });
code_enum!(FaultKind {
    MarkerNotDetected = "MARKER_NOT_DETECTED", PcFault = "PC_FAULT", OutOfService = "OUT_OF_SERVICE",
    ReleaseFault = "RELEASE_FAULT", LatchFault = "LATCH_FAULT",
});
code_enum!(Preset {
    Synthetic = "SYNTHETIC", PartsLogistics = "PARTS_LOGISTICS", HospitalLab = "HOSPITAL_LAB",
    RetailTransfer = "RETAIL_TRANSFER", RemoteTerrain = "REMOTE_TERRAIN",
});
code_enum!(ConfigStatus { Complete = "COMPLETE", Stub = "STUB" });
code_enum!(MatrixSource { Synthetic = "SYNTHETIC", File = "FILE" });
code_enum!(NetworkTopology { HubSpoke = "HUB_SPOKE", Mesh = "MESH" });
code_enum!(AssignmentPolicyName { MinEta = "MIN_ETA" });
code_enum!(DispatchPolicyName { Immediate = "IMMEDIATE" });
code_enum!(HoldingPolicyName { BoundedHold = "BOUNDED_HOLD" });
code_enum!(DroneRebalancingPolicyName { NearestUnclaimed = "NEAREST_UNCLAIMED" });
code_enum!(PodRebalancingPolicyName { MinThreshold = "MIN_THRESHOLD" });
code_enum!(DiversionPolicyName { NearestFeasible = "NEAREST_FEASIBLE" });

/// Convert seconds to integer milliseconds with the spec rule `floor(s * 1000 + 0.5)`.
///
/// Callers pass validated, finite, non-negative durations; anything else is a bug.
pub fn ms_from_s(seconds: f64) -> TimeMs {
    debug_assert!(
        seconds.is_finite() && seconds >= 0.0,
        "bad duration {seconds}"
    );
    let ms = (seconds * 1000.0 + 0.5).floor();
    // Saturating float-to-int cast; validated inputs stay far below u64::MAX.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let out = ms as TimeMs;
    out
}

pub fn s_from_ms(ms: TimeMs) -> f64 {
    #[allow(clippy::cast_precision_loss)]
    let out = ms as f64 / 1000.0;
    out
}
