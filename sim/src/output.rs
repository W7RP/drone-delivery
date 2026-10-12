//! Run outputs (spec 8.4): `events.bin` (9.4), `flight_log.bin` (9.3), `alerts.jsonl` and
//! `deliveries.csv`, all streamed. The first I/O error is kept and reported when the run ends;
//! writes after it are skipped so a full disk cannot corrupt the event order silently.

use std::fmt;
use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;

use crate::event::{Args, LogStatus};
use crate::ids::TimeMs;

pub const EVENT_RECORD_BYTES: u32 = 48;
pub const FLIGHT_RECORD_BYTES: u32 = 48;
const HEADER_BYTES: u32 = 32;
const VERSION_MAJOR: u16 = 1;
const VERSION_MINOR: u16 = 0;
/// Buffer per output stream; large enough that the hot loop rarely reaches the OS.
const BUFFER_BYTES: usize = 1 << 20;

pub const DELIVERIES_HEADER: &str = "order_id,kind,status,fail_reason,origin_pad,dest_pad,delivered_pad,drone_id,requested_t_ms,placed_t_ms,latched_t_ms,delivered_t_ms,drone_door_to_door_s,truck_door_to_door_s,truck_cost_cu,pod_wait_s,queue_s,pad_service_s,flight_s\n";

pub enum Sink {
    File(BufWriter<File>),
    Memory(Vec<u8>),
    Null,
}

impl Sink {
    pub fn file(path: &Path) -> io::Result<Sink> {
        Ok(Sink::File(BufWriter::with_capacity(
            BUFFER_BYTES,
            File::create(path)?,
        )))
    }

    pub fn bytes(&self) -> &[u8] {
        match self {
            Sink::Memory(v) => v,
            _ => &[],
        }
    }
}

impl Write for Sink {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        match self {
            Sink::File(w) => w.write(buf),
            Sink::Memory(v) => v.write(buf),
            Sink::Null => Ok(buf.len()),
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        match self {
            Sink::File(w) => w.flush(),
            _ => Ok(()),
        }
    }
}

pub struct Outputs {
    pub events: Sink,
    pub flights: Sink,
    pub alerts: Sink,
    pub deliveries: Sink,
    error: Option<io::Error>,
    /// Bytes of `events.bin` written so far, and the run's cap on them (`sim.max_event_log_mib`).
    events_bytes: u64,
    events_cap_bytes: u64,
    events_capped: bool,
}

fn header(magic: &[u8; 8], count_a: u32, source_hash: u64) -> [u8; 32] {
    let mut h = [0u8; 32];
    h[0..8].copy_from_slice(magic);
    h[8..10].copy_from_slice(&VERSION_MAJOR.to_le_bytes());
    h[10..12].copy_from_slice(&VERSION_MINOR.to_le_bytes());
    h[12..16].copy_from_slice(&HEADER_BYTES.to_le_bytes());
    h[16..20].copy_from_slice(&count_a.to_le_bytes());
    h[24..32].copy_from_slice(&source_hash.to_le_bytes());
    h
}

impl Outputs {
    pub fn new(
        events: Sink,
        flights: Sink,
        alerts: Sink,
        deliveries: Sink,
        source_hash: u64,
    ) -> Outputs {
        let mut out = Outputs {
            events,
            flights,
            alerts,
            deliveries,
            error: None,
            events_bytes: 0,
            events_cap_bytes: u64::MAX,
            events_capped: false,
        };
        let ev = header(b"DDEVLOG\0", EVENT_RECORD_BYTES, source_hash);
        let fl = header(b"DDFLOG\0\0", FLIGHT_RECORD_BYTES, source_hash);
        out.write_events(&ev);
        out.write_flights(&fl);
        let r = out.deliveries.write_all(DELIVERIES_HEADER.as_bytes());
        out.keep(r);
        out
    }

    pub fn in_memory(source_hash: u64) -> Outputs {
        Outputs::new(
            Sink::Memory(Vec::new()),
            Sink::Memory(Vec::new()),
            Sink::Memory(Vec::new()),
            Sink::Memory(Vec::new()),
            source_hash,
        )
    }

    pub fn to_dir(dir: &Path, source_hash: u64) -> io::Result<Outputs> {
        Outputs::to_dir_with(dir, source_hash, true)
    }

    /// As `to_dir`; without `event_log` no `events.bin` is written (sweeps, which only read the
    /// metrics and deliveries).
    pub fn to_dir_with(dir: &Path, source_hash: u64, event_log: bool) -> io::Result<Outputs> {
        std::fs::create_dir_all(dir)?;
        let events = if event_log {
            Sink::file(&dir.join("events.bin"))?
        } else {
            Sink::Null
        };
        Ok(Outputs::new(
            events,
            Sink::file(&dir.join("flight_log.bin"))?,
            Sink::file(&dir.join("alerts.jsonl"))?,
            Sink::file(&dir.join("deliveries.csv"))?,
            source_hash,
        ))
    }

    fn keep(&mut self, r: io::Result<()>) {
        if let Err(e) = r {
            self.error.get_or_insert(e);
        }
    }

    /// Cap `events.bin` at `bytes`; past it nothing more is written and `events_capped` is set.
    pub fn set_event_cap(&mut self, bytes: u64) {
        self.events_cap_bytes = bytes;
    }

    pub fn events_capped(&self) -> bool {
        self.events_capped
    }

    fn write_events(&mut self, bytes: &[u8]) {
        let len = bytes.len() as u64;
        if self.events_bytes + len > self.events_cap_bytes {
            self.events_capped = true;
            return;
        }
        self.events_bytes += len;
        if self.error.is_none() {
            let r = self.events.write_all(bytes);
            self.keep(r);
        }
    }

    fn write_flights(&mut self, bytes: &[u8]) {
        if self.error.is_none() {
            let r = self.flights.write_all(bytes);
            self.keep(r);
        }
    }

    pub fn event_record(
        &mut self,
        t: TimeMs,
        kind: u16,
        class: u8,
        status: LogStatus,
        msg_seq: u32,
        args: &Args,
    ) {
        let mut rec = [0u8; 48];
        rec[0..8].copy_from_slice(&t.to_le_bytes());
        rec[8..10].copy_from_slice(&kind.to_le_bytes());
        rec[10] = class;
        rec[11] = status as u8;
        rec[12..16].copy_from_slice(&msg_seq.to_le_bytes());
        rec[16..48].copy_from_slice(&args.buf);
        self.write_events(&rec);
    }

    pub fn flight_record(&mut self, rec: &[u8; 48]) {
        self.write_flights(rec);
    }

    /// Formats straight into the buffered sink; no intermediate String is allocated.
    pub fn alert_line(&mut self, line: fmt::Arguments<'_>) {
        if self.error.is_none() {
            let r = self.alerts.write_fmt(line);
            self.keep(r);
        }
    }

    pub fn delivery_line(&mut self, line: fmt::Arguments<'_>) {
        if self.error.is_none() {
            let r = self.deliveries.write_fmt(line);
            self.keep(r);
        }
    }

    /// Flush everything and return the first error seen during the run, if any.
    pub fn finish(&mut self) -> io::Result<()> {
        for sink in [
            &mut self.events,
            &mut self.flights,
            &mut self.alerts,
            &mut self.deliveries,
        ] {
            if let Err(e) = sink.flush() {
                self.error.get_or_insert(e);
            }
        }
        match self.error.take() {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }
}
