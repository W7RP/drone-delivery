//! M11 order outcomes (spec 7, `metrics.order_outcomes`): what became of every shipment
//! requested in the window, counted at the horizon, and the drone/truck door-to-door ratio in
//! two forms. Measurement only; nothing in the run reads it.

use serde_json::{Value, json};

use crate::fm::truck_baseline;
use crate::ids::{DroneMode, OrderKind, OrderStatus, PadId, TimeMs, s_from_ms};
use crate::world::World;

/// Counts per outcome class and the time sums behind the two ratios.
#[derive(Default)]
struct Tally {
    delivered_drone: u64,
    delivered_truck: u64,
    diverted_or_failed: u64,
    in_flight: u64,
    blocked_weather: u64,
    blocked_energy: u64,
    waiting: u64,
    done_network_ms: u64,
    done_truck_ms: u64,
    open_elapsed_ms: u64,
    open_truck_ms: u64,
}

impl World {
    /// A drone charged to `drone.charge_target_soc` could not fly a loaded pod from `origin`
    /// to `dest` under this hour's energy rules (`dispatch_ok`, spec 6).
    pub(crate) fn route_energy_blocked(&self, origin: PadId, dest: PadId) -> bool {
        let soc = self.cfg.drone.charge_target_soc;
        let payload = self.cfg.pod_gross_mass_kg;
        !self.view().dispatch_ok(soc, origin, dest, payload)
    }

    /// Why a shipment not yet on a flying drone is still open at the horizon.
    fn open_class(&self, origin: PadId, dest: PadId, t: &mut Tally) {
        if self.flights_blocked() {
            t.blocked_weather += 1;
        } else if self.route_energy_blocked(origin, dest) {
            t.blocked_energy += 1;
        } else {
            t.waiting += 1;
        }
    }

    fn open_times(&self, t: &mut Tally, requested: TimeMs, truck_delivered: TimeMs) {
        t.open_elapsed_ms += self.now - requested;
        t.open_truck_ms += truck_delivered - requested;
    }

    /// M11 at the horizon: every SHIPMENT_REQUEST in W lands in exactly one class.
    pub(crate) fn order_outcomes(&self) -> Value {
        let mut t = Tally::default();
        for o in &self.fm.orders {
            if o.kind != OrderKind::Shipment || !self.metrics.in_window(o.requested_t_ms) {
                continue;
            }
            match o.status {
                OrderStatus::Delivered | OrderStatus::Trucked => {
                    if o.status == OrderStatus::Delivered {
                        t.delivered_drone += 1;
                    } else {
                        t.delivered_truck += 1;
                    }
                    t.done_network_ms += o.delivered_t_ms - o.requested_t_ms;
                    t.done_truck_ms += o.truck_delivered_t_ms - o.requested_t_ms;
                }
                OrderStatus::Diverted | OrderStatus::Failed => t.diverted_or_failed += 1,
                OrderStatus::Latched | OrderStatus::InTransit
                    if !matches!(
                        self.drones[usize::from(o.drone_id)].mode,
                        DroneMode::OnPad | DroneMode::Parked | DroneMode::Grounded
                    ) =>
                {
                    t.in_flight += 1;
                    self.open_times(&mut t, o.requested_t_ms, o.truck_delivered_t_ms);
                }
                _ => {
                    self.open_class(o.origin_pad, o.dest_pad, &mut t);
                    self.open_times(&mut t, o.requested_t_ms, o.truck_delivered_t_ms);
                }
            }
        }
        // Shipments never placed: still in a backlog, or being placed on OUT.
        for (p, pad) in self.pads.iter().enumerate() {
            let origin = PadId::try_from(p).expect("pad count fits u16");
            let placing = pad
                .outs
                .iter()
                .filter(|o| o.placing && o.placement.kind == OrderKind::Shipment)
                .map(|o| (o.placement.requested, o.placement.dest));
            let queued = pad.backlog.iter().map(|s| (s.requested, s.dest));
            for (requested, dest) in queued.chain(placing) {
                if !self.metrics.in_window(requested) {
                    continue;
                }
                self.open_class(origin, dest, &mut t);
                let (truck, _) =
                    truck_baseline(&self.cfg, &self.net, self.master, origin, dest, requested);
                self.open_times(&mut t, requested, truck);
            }
        }
        let (requests, rejected) = self.metrics.requests_in_window();
        let done = t.delivered_drone + t.delivered_truck;
        let counted = done
            + rejected
            + t.diverted_or_failed
            + t.in_flight
            + t.blocked_weather
            + t.blocked_energy
            + t.waiting;
        debug_assert_eq!(counted, requests, "every request in W has one outcome");
        #[allow(clippy::cast_precision_loss)]
        let ratio = |net: u64, truck: u64| (truck > 0).then(|| net as f64 / truck as f64);
        #[allow(clippy::cast_precision_loss)]
        let rate = (requests > 0).then(|| done as f64 / requests as f64);
        json!({
            "requests": requests,
            "delivered_drone": t.delivered_drone,
            "delivered_truck": t.delivered_truck,
            "rejected_cap": rejected,
            "blocked_weather": t.blocked_weather,
            "blocked_energy": t.blocked_energy,
            "in_flight": t.in_flight,
            "waiting": t.waiting,
            "diverted_or_failed": t.diverted_or_failed,
            "completion_rate": rate,
            "ratio_delivered": ratio(t.done_network_ms, t.done_truck_ms),
            "ratio_lower_bound": ratio(
                t.done_network_ms + t.open_elapsed_ms,
                t.done_truck_ms + t.open_truck_ms
            ),
            "sums_s": {
                "delivered_network": s_from_ms(t.done_network_ms),
                "delivered_truck": s_from_ms(t.done_truck_ms),
                "unfinished_elapsed": s_from_ms(t.open_elapsed_ms),
                "unfinished_truck": s_from_ms(t.open_truck_ms),
            },
        })
    }
}
