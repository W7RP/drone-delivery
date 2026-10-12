//! Memory rule: no per-event heap allocation in the hot loop. A counting allocator measures
//! allocations made during `World::run` on one simulated day of the 200-drone case, with
//! null output sinks so buffered file I/O is not counted. The few that remain are amortised
//! growth of order and queue storage plus the end-of-run summary.

// A counting `GlobalAlloc` needs `unsafe`; this test file is the only place it is allowed.
#![allow(unsafe_code)]

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicU64, Ordering};

use drone_sim::output::{Outputs, Sink};
use drone_sim::scenario::Scenario;

struct Counting;

static ALLOCS: AtomicU64 = AtomicU64::new(0);

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

#[test]
fn run_loop_does_not_allocate_per_event() {
    let path = format!(
        "{}/../configs/scale_200/scenario.toml",
        env!("CARGO_MANIFEST_DIR")
    );
    let mut t: toml::Table = std::fs::read_to_string(path).unwrap().parse().unwrap();
    t["sim"]["duration_s"]["value"] = toml::Value::Float(86_400.0);
    let scenario = Scenario::from_toml(&toml::to_string(&t).unwrap()).unwrap();
    let out = Outputs::new(Sink::Null, Sink::Null, Sink::Null, Sink::Null, 0);
    let mut world = scenario.into_world(0, out);
    let before = ALLOCS.load(Ordering::Relaxed);
    let summary = world.run().unwrap();
    let allocs = ALLOCS.load(Ordering::Relaxed) - before;
    let events = summary["events_processed"].as_u64().unwrap();
    println!(
        "{allocs} allocations over {events} events ({:.5} per event)",
        allocs as f64 / events as f64
    );
    assert!(events > 100_000);
    assert!(
        allocs * 1000 < events,
        "{allocs} allocations for {events} events"
    );
}
