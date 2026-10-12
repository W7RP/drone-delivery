//! Online metric aggregation (spec section 7): counters, time integrals updated on state
//! changes, and fixed-bin histograms. Memory is bounded by entity counts, not run length.

use serde_json::{Value, json};

use crate::ids::{AlertCode, TimeMs, s_from_ms};

const SECONDS_PER_HOUR: f64 = 3600.0;

/// Fixed-bin histogram; the last bin is open-ended at `max_ms` (spec 7).
#[derive(Debug, Clone)]
pub struct Histogram {
    bin_ms: TimeMs,
    max_ms: TimeMs,
    counts: Vec<u64>,
    n: u64,
    sum_s: f64,
}

impl Histogram {
    pub fn new(bin_ms: TimeMs, max_ms: TimeMs) -> Self {
        let bins = usize::try_from(max_ms.div_ceil(bin_ms)).expect("bin count fits usize");
        Self {
            bin_ms,
            max_ms,
            counts: vec![0; bins + 1],
            n: 0,
            sum_s: 0.0,
        }
    }

    pub fn add(&mut self, value_ms: TimeMs) {
        let last = self.counts.len() - 1;
        let idx = if value_ms >= self.max_ms {
            last
        } else {
            usize::try_from(value_ms / self.bin_ms).unwrap_or(last)
        };
        self.counts[idx.min(last)] += 1;
        self.n += 1;
        self.sum_s += s_from_ms(value_ms);
    }

    pub fn mean_s(&self) -> Option<f64> {
        #[allow(clippy::cast_precision_loss)]
        (self.n > 0).then(|| self.sum_s / self.n as f64)
    }

    /// Upper edge of the bin holding rank ceil(q n). In the open last bin the value is
    /// reported as `max_ms`, a lower bound.
    pub fn percentile_s(&self, q: f64) -> Option<f64> {
        if self.n == 0 {
            return None;
        }
        #[allow(
            clippy::cast_precision_loss,
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss
        )]
        let rank = ((q * self.n as f64).ceil() as u64).max(1);
        let mut seen = 0;
        for (i, &c) in self.counts.iter().enumerate() {
            seen += c;
            if seen >= rank {
                let last = self.counts.len() - 1;
                let edge = if i == last {
                    self.max_ms
                } else {
                    (u64::try_from(i).unwrap_or(0) + 1) * self.bin_ms
                };
                return Some(s_from_ms(edge.min(self.max_ms)));
            }
        }
        Some(s_from_ms(self.max_ms))
    }

    pub fn count(&self) -> u64 {
        self.n
    }

    fn summary(&self) -> Value {
        json!({ "n": self.n, "mean": self.mean_s(), "p50": self.percentile_s(0.5), "p95": self.percentile_s(0.95) })
    }
}

/// Time a boolean condition holds, clipped to the metrics window.
#[derive(Debug, Clone, Copy, Default)]
pub struct Interval {
    since: Option<TimeMs>,
    total_ms: TimeMs,
}

impl Interval {
    pub fn set(&mut self, on: bool, t: TimeMs, window: (TimeMs, TimeMs)) {
        match (on, self.since) {
            (true, None) => self.since = Some(t),
            (false, Some(start)) => {
                self.total_ms += overlap(start, t, window);
                self.since = None;
            }
            _ => {}
        }
    }

    fn close(&mut self, window: (TimeMs, TimeMs)) {
        if let Some(start) = self.since.take() {
            self.total_ms += overlap(start, window.1, window);
        }
    }

    fn fraction(&self, window: (TimeMs, TimeMs)) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        let f = self.total_ms as f64 / (window.1 - window.0) as f64;
        f
    }
}

fn overlap(a: TimeMs, b: TimeMs, window: (TimeMs, TimeMs)) -> TimeMs {
    b.min(window.1).saturating_sub(a.max(window.0))
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ClosedOrder {
    pub requested_ms: TimeMs,
    pub placed_ms: TimeMs,
    pub latched_ms: Option<TimeMs>,
    pub delivered_ms: Option<TimeMs>,
    pub truck_delivered_ms: TimeMs,
    pub truck_cost: f64,
    pub split: Option<DoorToDoorSplit>,
}

/// Parts of one order's drone door-to-door time (spec 7, M2); they sum to it exactly.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DoorToDoorSplit {
    pub pod_wait_ms: TimeMs,
    pub queue_ms: TimeMs,
    pub pad_service_ms: TimeMs,
    pub flight_ms: TimeMs,
}

#[derive(Debug, Clone)]
pub struct Metrics {
    pub window: (TimeMs, TimeMs),
    deliveries_in_window: u64,
    split_sum: DoorToDoorSplit,
    split_n: u64,
    closed_delivered: u64,
    closed_diverted: u64,
    closed_failed: u64,
    door_drone: Histogram,
    door_truck: Histogram,
    saving_sum_s: f64,
    saving_n: u64,
    truck_cost_sum: f64,
    truck_cost_n: u64,
    spot_in: Vec<Interval>,
    /// Per pad, one interval per OUT spot (A72).
    spot_out: Vec<Vec<Interval>>,
    placement_wait: Histogram,
    pickup_wait: Histogram,
    landing_hold: Histogram,
    energy_added_j: Vec<f64>,
    pod_available: Vec<Interval>,
    requests: u64,
    stockouts: u64,
    rejected: u64,
    approaches: u64,
    go_arounds: u64,
    latch_aborts: u64,
    diverts: u64,
    alerts: [u64; 9],
    pub open_at_end: u64,
    pub events_processed: u64,
}

pub struct CostInputs {
    pub fleet_size: u16,
    pub drone_per_s: f64,
    pub energy_per_j: f64,
    pub charging_efficiency: f64,
    pub battery_capacity_j: f64,
}

impl Metrics {
    pub fn new(
        window: (TimeMs, TimeMs),
        bin_ms: TimeMs,
        max_ms: TimeMs,
        n_pads: usize,
        n_drones: usize,
        out_spots: usize,
    ) -> Self {
        let h = || Histogram::new(bin_ms, max_ms);
        Self {
            window,
            deliveries_in_window: 0,
            split_sum: DoorToDoorSplit::default(),
            split_n: 0,
            closed_delivered: 0,
            closed_diverted: 0,
            closed_failed: 0,
            door_drone: h(),
            door_truck: h(),
            saving_sum_s: 0.0,
            saving_n: 0,
            truck_cost_sum: 0.0,
            truck_cost_n: 0,
            spot_in: vec![Interval::default(); n_pads],
            spot_out: vec![vec![Interval::default(); out_spots]; n_pads],
            placement_wait: h(),
            pickup_wait: h(),
            landing_hold: h(),
            energy_added_j: vec![0.0; n_drones],
            pod_available: vec![Interval::default(); n_pads],
            requests: 0,
            stockouts: 0,
            rejected: 0,
            approaches: 0,
            go_arounds: 0,
            latch_aborts: 0,
            diverts: 0,
            alerts: [0; 9],
            open_at_end: 0,
            events_processed: 0,
        }
    }

    pub fn in_window(&self, t: TimeMs) -> bool {
        t >= self.window.0 && t <= self.window.1
    }

    /// IN when `out` is None, else the OUT spot with that index.
    pub fn spot_occupied(&mut self, pad: usize, out: Option<usize>, occupied: bool, t: TimeMs) {
        let w = self.window;
        match out {
            None => self.spot_in[pad].set(occupied, t, w),
            Some(k) => self.spot_out[pad][k].set(occupied, t, w),
        }
    }

    pub fn pods_available(&mut self, pad: usize, available: bool, t: TimeMs) {
        let w = self.window;
        self.pod_available[pad].set(available, t, w);
    }

    pub fn shipment_request(&mut self, t: TimeMs, stockout: bool) {
        if self.in_window(t) {
            self.requests += 1;
            self.stockouts += u64::from(stockout);
        }
    }

    /// M9: a request turned away at a full backlog (P08). It also counted in `shipment_request`.
    pub fn request_rejected(&mut self, t: TimeMs) {
        if self.in_window(t) {
            self.rejected += 1;
        }
    }

    pub fn approach(&mut self, t: TimeMs, went_around: bool) {
        if self.in_window(t) {
            self.approaches += 1;
            self.go_arounds += u64::from(went_around);
        }
    }

    pub fn latch_abort(&mut self, t: TimeMs) {
        self.latch_aborts += u64::from(self.in_window(t));
    }

    pub fn landing_hold(&mut self, t: TimeMs, held_ms: TimeMs) {
        if self.in_window(t) {
            self.landing_hold.add(held_ms);
        }
    }

    pub fn energy_added(&mut self, drone: usize, t: TimeMs, joules: f64) {
        if self.in_window(t) {
            self.energy_added_j[drone] += joules;
        }
    }

    pub fn alert(&mut self, code: AlertCode, t: TimeMs) {
        self.alerts[usize::from(code.code())] += 1;
        if code == AlertCode::Diverted && self.in_window(t) {
            self.diverts += 1;
        }
    }

    /// M1, M2, M4, M7 contributions of a closed SHIPMENT order.
    /// Mean of each door-to-door part over the orders in `door_to_door_s` (M2).
    fn split_summary(&self) -> Value {
        #[allow(clippy::cast_precision_loss)]
        let n = self.split_n as f64;
        let mean = |ms: TimeMs| (n > 0.0).then(|| s_from_ms(ms) / n);
        let s = self.split_sum;
        json!({
            "n": self.split_n,
            "pod_wait": mean(s.pod_wait_ms),
            "queue": mean(s.queue_ms),
            "pad_service": mean(s.pad_service_ms),
            "flight": mean(s.flight_ms),
        })
    }

    pub fn shipment_closed(&mut self, o: &ClosedOrder, delivered: bool, diverted: bool) {
        if delivered && o.delivered_ms.is_some_and(|t| self.in_window(t)) {
            self.deliveries_in_window += 1;
        }
        if !self.in_window(o.requested_ms) {
            return;
        }
        match (delivered, diverted) {
            (true, _) => self.closed_delivered += 1,
            (false, true) => self.closed_diverted += 1,
            _ => self.closed_failed += 1,
        }
        self.placement_wait
            .add(o.placed_ms.saturating_sub(o.requested_ms));
        if let Some(l) = o.latched_ms {
            self.pickup_wait.add(l.saturating_sub(o.placed_ms));
        }
        self.truck_cost_sum += o.truck_cost;
        self.truck_cost_n += 1;
        let truck_ms = o.truck_delivered_ms - o.requested_ms;
        self.door_truck.add(truck_ms);
        if let (true, Some(d)) = (delivered, o.delivered_ms) {
            let drone_ms = d - o.requested_ms;
            self.door_drone.add(drone_ms);
            if let Some(s) = o.split {
                self.split_sum.pod_wait_ms += s.pod_wait_ms;
                self.split_sum.queue_ms += s.queue_ms;
                self.split_sum.pad_service_ms += s.pad_service_ms;
                self.split_sum.flight_ms += s.flight_ms;
                self.split_n += 1;
            }
            #[allow(clippy::cast_precision_loss)]
            {
                self.saving_sum_s += (truck_ms as f64 - drone_ms as f64) / 1000.0;
            }
            self.saving_n += 1;
        }
    }

    pub fn finish(&mut self) {
        let w = self.window;
        for i in self
            .spot_in
            .iter_mut()
            .chain(self.spot_out.iter_mut().flatten())
            .chain(self.pod_available.iter_mut())
        {
            i.close(w);
        }
    }

    #[allow(clippy::cast_precision_loss)]
    pub fn summary(&self, cost: &CostInputs) -> Value {
        let w = self.window;
        let window_h = s_from_ms(w.1 - w.0) / SECONDS_PER_HOUR;
        let deliveries = self.deliveries_in_window as f64;
        let closed = self.closed_delivered + self.closed_diverted + self.closed_failed;
        let ratio = |a: f64, b: f64| (b > 0.0).then(|| a / b);
        let mean_truck = self.door_truck.mean_s();
        let mean_drone = self.door_drone.mean_s();
        let truck_cost = ratio(self.truck_cost_sum, self.truck_cost_n as f64);
        let energy_j: f64 = self.energy_added_j.iter().sum();
        let grid_j = energy_j / cost.charging_efficiency;
        let fleet = f64::from(cost.fleet_size);
        let drone_total =
            cost.drone_per_s * fleet * s_from_ms(w.1 - w.0) + cost.energy_per_j * grid_j;
        let drone_cost = ratio(drone_total, deliveries);
        let energy_cost_per_delivery = ratio(cost.energy_per_j * grid_j, deliveries);
        let breakeven = match (truck_cost, energy_cost_per_delivery) {
            (Some(t), Some(e)) if t - e > 0.0 => {
                Some(cost.drone_per_s * SECONDS_PER_HOUR / (t - e))
            }
            _ => None,
        };
        let cycles: Vec<f64> = self
            .energy_added_j
            .iter()
            .map(|e| e / cost.battery_capacity_j)
            .collect();
        let per_spot = |v: &[Interval]| -> Vec<f64> { v.iter().map(|i| i.fraction(w)).collect() };
        // A pad's OUT utilisation is the mean over its OUT spots (one spot: that spot's).
        let per_pad_out: Vec<f64> = self
            .spot_out
            .iter()
            .map(|outs| outs.iter().map(|i| i.fraction(w)).sum::<f64>() / outs.len() as f64)
            .collect();
        json!({
            "window_s": [s_from_ms(w.0), s_from_ms(w.1)],
            "deliveries_per_h": deliveries / window_h,
            "deliveries_in_window": self.deliveries_in_window,
            "door_to_door_s": self.door_drone.summary(),
            "truck_door_to_door_s": self.door_truck.summary(),
            "door_to_door_split_s": self.split_summary(),
            "speedup": match (mean_truck, mean_drone) { (Some(t), Some(d)) => ratio(t, d), _ => None },
            "saving_s": ratio(self.saving_sum_s, self.saving_n as f64),
            "spot_utilization": { "in": per_spot(&self.spot_in), "out": per_pad_out },
            "queue_time_s": {
                "placement_wait": self.placement_wait.summary(),
                "pickup_wait": self.pickup_wait.summary(),
                "landing_hold": self.landing_hold.summary(),
            },
            "battery_cycles": {
                "per_drone": cycles,
                "fleet_mean": cycles.iter().sum::<f64>() / fleet,
                "per_delivery": ratio(cycles.iter().sum::<f64>(), deliveries),
            },
            "pod_availability": per_spot(&self.pod_available),
            "stockout_fraction": ratio(self.stockouts as f64, self.requests as f64),
            "shipment_requests_in_window": self.requests,
            "requests_rejected": self.rejected,
            "rejected_fraction": ratio(self.rejected as f64, self.requests as f64),
            "abort_rate": ratio((self.closed_diverted + self.closed_failed) as f64, closed as f64),
            "orders_closed": { "delivered": self.closed_delivered, "diverted": self.closed_diverted, "failed": self.closed_failed },
            "go_around_rate": ratio(self.go_arounds as f64, self.approaches as f64),
            "latch_abort_count": self.latch_aborts,
            "divert_count": self.diverts,
            "cost": {
                "truck_cost_per_delivery": truck_cost,
                "drone_cost_per_delivery": drone_cost,
                "cost_ratio": match (drone_cost, truck_cost) { (Some(d), Some(t)) => ratio(d, t), _ => None },
                "breakeven_deliveries_per_drone_h": breakeven,
                "actual_deliveries_per_drone_h": deliveries / fleet / window_h,
            },
            "alerts": AlertCode::ALL.iter().map(|c| (c.name().to_owned(), json!(self.alerts[usize::from(c.code())]))).collect::<serde_json::Map<_, _>>(),
            "open_at_end": self.open_at_end,
            "events_processed": self.events_processed,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn histogram_percentiles_use_upper_bin_edges() {
        let mut h = Histogram::new(10_000, 100_000);
        for v in [1_000, 12_000, 15_000, 35_000] {
            h.add(v);
        }
        assert_eq!(h.percentile_s(0.5), Some(20.0));
        assert_eq!(h.percentile_s(0.95), Some(40.0));
        assert!((h.mean_s().unwrap() - 15.75).abs() < 1e-12);
        h.add(500_000);
        assert_eq!(h.percentile_s(1.0), Some(100.0));
    }

    #[test]
    fn interval_is_clipped_to_window() {
        let mut i = Interval::default();
        let w = (1_000, 5_000);
        i.set(true, 0, w);
        i.set(false, 2_000, w);
        i.set(true, 4_500, w);
        i.close(w);
        assert_eq!(i.total_ms, 1_500);
        assert!((i.fraction(w) - 0.375).abs() < 1e-12);
    }
}
