//! Reader for `events.bin` (spec 9.4), used by tests and `sim dump`.

use crate::event::{TRANSITION_KIND, tcode};
use crate::output::EVENT_RECORD_BYTES;

/// Event kind names in `events.schema.json` order (index = kind code).
pub const KIND_NAMES: [&str; 43] = [
    "LANDING_REQUEST",
    "LANDING_CANCEL",
    "APPROACH_ABORT",
    "LANDED",
    "RELEASE_REPORT",
    "LATCH_REPORT",
    "DEPART_REQUEST",
    "DEPARTED",
    "LANDING_GRANT",
    "LANDING_HOLD",
    "LANDING_REFUSED",
    "RELEASE_CMD",
    "HOP_CMD",
    "LATCH_CMD",
    "CHARGE_START",
    "SERVICE_COMPLETE",
    "DEPART_CLEAR",
    "DRONE_STATUS",
    "MISSION_REQUEST",
    "HOLD_REPORT",
    "MISSION_ASSIGN",
    "HOLD_CONTINUE",
    "POD_PLACED",
    "POD_LATCHED",
    "POD_DELIVERED",
    "POD_REMOVED",
    "PAD_STATUS",
    "SHIPMENT_PENDING",
    "PICKUP_ASSIGNED",
    "POD_RETURN_REQUEST",
    "SHIPMENT_REQUEST",
    "POD_PLACE_DONE",
    "POD_REMOVE_DONE",
    "TAKEOFF_DONE",
    "LANE_DONE",
    "APPROACH_DONE",
    "GO_AROUND_DONE",
    "HOP_DONE",
    "RELEASE_DONE",
    "LATCH_DONE",
    "FAULT_START",
    "FAULT_END",
    "TIMER",
];

const STATUS_NAMES: [&str; 5] = ["PROCESSED", "STALE", "DROPPED", "LOST", "DEFERRED"];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Record {
    pub t_ms: u64,
    pub kind: u16,
    pub class: u8,
    pub status: u8,
    pub msg_seq: u32,
    pub args: [u8; 32],
}

/// A state-machine transition record, decoded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Transition {
    pub t_ms: u64,
    pub code: u16,
    pub pad: u16,
    pub drone: u16,
    pub from: u8,
    pub to: u8,
}

impl Record {
    pub fn transition(&self) -> Option<Transition> {
        (self.kind == TRANSITION_KIND).then(|| {
            let u16_at = |i: usize| u16::from_le_bytes([self.args[i], self.args[i + 1]]);
            Transition {
                t_ms: self.t_ms,
                code: u16_at(0),
                pad: u16_at(2),
                drone: u16_at(4),
                from: self.args[6],
                to: self.args[7],
            }
        })
    }

    pub fn kind_name(&self) -> String {
        match self.transition() {
            Some(t) => tcode::label(t.code),
            None => KIND_NAMES
                .get(usize::from(self.kind))
                .map_or_else(|| format!("kind{}", self.kind), |s| (*s).to_owned()),
        }
    }
}

/// Decode a whole `events.bin`, checking the header.
pub fn decode(bytes: &[u8]) -> Result<Vec<Record>, String> {
    if bytes.len() < 32 || &bytes[0..8] != b"DDEVLOG\0" {
        return Err("not an events.bin file".into());
    }
    let header = u32::from_le_bytes(bytes[12..16].try_into().expect("4 bytes")) as usize;
    let rec = EVENT_RECORD_BYTES as usize;
    let body = bytes.get(header..).ok_or("truncated header")?;
    if body.len() % rec != 0 {
        return Err("record area is not a multiple of the record size".into());
    }
    Ok(body
        .chunks_exact(rec)
        .map(|c| Record {
            t_ms: u64::from_le_bytes(c[0..8].try_into().expect("8 bytes")),
            kind: u16::from_le_bytes([c[8], c[9]]),
            class: c[10],
            status: c[11],
            msg_seq: u32::from_le_bytes(c[12..16].try_into().expect("4 bytes")),
            args: c[16..48].try_into().expect("32 bytes"),
        })
        .collect())
}

pub fn format_record(r: &Record) -> String {
    let status = STATUS_NAMES
        .get(usize::from(r.status))
        .copied()
        .unwrap_or("?");
    match r.transition() {
        Some(t) => format!(
            "{:>12.3} {:<6} pad={} drone={} {}->{}",
            r.t_ms as f64 / 1000.0,
            tcode::label(t.code),
            t.pad,
            t.drone,
            t.from,
            t.to
        ),
        None => format!(
            "{:>12.3} {:<20} {:<9} seq={} args={:02x?}",
            r.t_ms as f64 / 1000.0,
            r.kind_name(),
            status,
            r.msg_seq,
            &r.args[..16]
        ),
    }
}
