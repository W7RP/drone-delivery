//! Pads, lanes and routes: the synthetic generator (spec 8.6), the in-memory CSR lane graph
//! (spec 9.1 layout), all-pairs shortest routes (spec 6) and the truck matrix (spec 9.2). A
//! `network_files` scenario takes its lanes and, with `truck.matrix_source` = FILE, its truck
//! matrix from the pipeline's files instead of computing them from pad positions.

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::f64::consts::PI;

use crate::config::{Config, PadCfg, SyntheticNetworkCfg, SyntheticTruckCfg};
use crate::ids::{LaneId, NO_LANE, NetworkTopology, PadId};
use crate::netfile::LaneFile;
use crate::rng::{StreamKind, master_seed, stream};

pub const EARTH_RADIUS_M: f64 = 6_371_000.0;
/// Route lengths within this many metres count as equal for the lower-lane-id tie rule.
const ROUTE_TIE_TOLERANCE_M: f64 = 1e-6;

/// Local ENU metres (x east, y north) from geodetic degrees, equirectangular (spec section 0).
pub fn enu_from_geo(lat_deg: f64, lon_deg: f64, lat0_deg: f64, lon0_deg: f64) -> (f64, f64) {
    let x = EARTH_RADIUS_M * lat0_deg.to_radians().cos() * (lon_deg - lon0_deg).to_radians();
    let y = EARTH_RADIUS_M * (lat_deg - lat0_deg).to_radians();
    (x, y)
}

pub fn geo_from_enu(x_m: f64, y_m: f64, lat0_deg: f64, lon0_deg: f64) -> (f64, f64) {
    let lat = lat0_deg + (y_m / EARTH_RADIUS_M).to_degrees();
    let lon = lon0_deg + (x_m / (EARTH_RADIUS_M * lat0_deg.to_radians().cos())).to_degrees();
    (lat, lon)
}

fn pad_id(i: usize) -> PadId {
    PadId::try_from(i).expect("pad count is validated below 65535")
}

fn lane_id(i: usize) -> LaneId {
    LaneId::try_from(i).expect("lane count fits u32")
}

/// Spec 8.6. The mesh draws from the NETWORK stream of replication 0 of the config seed, so the
/// network does not change with the CLI seed override or the replication index (A41).
pub fn generate(
    cfg: &SyntheticNetworkCfg,
    lat0_deg: f64,
    lon0_deg: f64,
    config_seed: u64,
) -> (Vec<PadCfg>, Vec<(PadId, PadId)>) {
    let n = usize::from(cfg.n_pads);
    let mut xy: Vec<(f64, f64)> = Vec::with_capacity(n);
    let mut lanes: Vec<(PadId, PadId)> = Vec::new();
    match cfg.topology {
        NetworkTopology::HubSpoke => {
            xy.push((0.0, 0.0));
            #[allow(clippy::cast_precision_loss)]
            let spokes = (n - 1) as f64;
            for i in 1..n {
                #[allow(clippy::cast_precision_loss)]
                let a = 2.0 * PI * (i - 1) as f64 / spokes;
                xy.push((cfg.radius_m * a.cos(), cfg.radius_m * a.sin()));
                lanes.push((0, pad_id(i)));
            }
        }
        NetworkTopology::Mesh => {
            let mut rng = stream(master_seed(config_seed, 0), StreamKind::Network, 0);
            for _ in 0..n {
                let (u1, u2) = (rng.uniform(), rng.uniform());
                let (rho, a) = (cfg.radius_m * u1.sqrt(), 2.0 * PI * u2);
                xy.push((rho * a.cos(), rho * a.sin()));
            }
            mesh_lanes(&xy, usize::from(cfg.mesh_neighbors), &mut lanes);
        }
    }
    let pads = xy
        .iter()
        .enumerate()
        .map(|(i, &(x, y))| {
            let (lat_deg, lon_deg) = geo_from_enu(x, y, lat0_deg, lon0_deg);
            let is_hub = cfg.topology == NetworkTopology::HubSpoke && i == 0;
            PadCfg {
                name: format!("P{i}"),
                lat_deg,
                lon_deg,
                is_hub,
                service_mode: cfg.service_mode,
                return_policy: cfg.return_policy,
                initial_empty_pods: cfg.initial_empty_pods,
                parking_capacity: if is_hub {
                    cfg.hub_parking_capacity
                } else {
                    cfg.parking_capacity
                },
            }
        })
        .collect();
    (pads, lanes)
}

fn dist(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

/// k-nearest-neighbour lanes plus the minimum spanning tree (Prim from P0), deduplicated.
fn mesh_lanes(xy: &[(f64, f64)], k: usize, lanes: &mut Vec<(PadId, PadId)>) {
    let n = xy.len();
    let add = |a: usize, b: usize, lanes: &mut Vec<(PadId, PadId)>| {
        let key = (pad_id(a.min(b)), pad_id(a.max(b)));
        if !lanes.contains(&key) {
            lanes.push(key);
        }
    };
    for i in 0..n {
        let mut others: Vec<usize> = (0..n).filter(|&j| j != i).collect();
        others.sort_by(|&a, &b| {
            dist(xy[i], xy[a])
                .total_cmp(&dist(xy[i], xy[b]))
                .then(a.cmp(&b))
        });
        for &j in others.iter().take(k) {
            add(i, j, lanes);
        }
    }
    let mut in_tree = vec![false; n];
    let mut best = vec![f64::INFINITY; n];
    let mut parent = vec![0usize; n];
    best[0] = 0.0;
    for _ in 0..n {
        let next = (0..n)
            .filter(|&j| !in_tree[j])
            .min_by(|&a, &b| best[a].total_cmp(&best[b]).then(a.cmp(&b)))
            .expect("a vertex remains");
        in_tree[next] = true;
        if next != 0 {
            add(parent[next], next, lanes);
        }
        for j in 0..n {
            let d = dist(xy[next], xy[j]);
            if !in_tree[j] && d < best[j] {
                best[j] = d;
                parent[j] = next;
            }
        }
    }
    lanes.sort_unstable();
}

/// CSR lane graph of undirected config lanes, lengths from ENU pad positions (spec 2.6).
fn lanes_from_positions(lanes: &[(PadId, PadId)], pos_m: &[(f64, f64)]) -> LaneFile {
    let mut directed: Vec<(PadId, PadId)> =
        lanes.iter().flat_map(|&(a, b)| [(a, b), (b, a)]).collect();
    directed.sort_unstable();
    let mut row_offsets = vec![0u32; pos_m.len() + 1];
    for &(a, _) in &directed {
        row_offsets[usize::from(a) + 1] += 1;
    }
    for i in 0..pos_m.len() {
        row_offsets[i + 1] += row_offsets[i];
    }
    #[allow(clippy::cast_possible_truncation)]
    let length_m = directed
        .iter()
        .map(|&(a, b)| dist(pos_m[usize::from(a)], pos_m[usize::from(b)]) as f32)
        .collect();
    let reverse_lane = directed
        .iter()
        .map(|&(a, b)| {
            lane_id(
                directed
                    .binary_search(&(b, a))
                    .expect("every lane has a reverse"),
            )
        })
        .collect();
    LaneFile {
        row_offsets,
        to_pad: directed.iter().map(|l| l.1).collect(),
        length_m,
        reverse_lane,
    }
}

/// Static network data, flat arrays indexed by pad and lane ID.
pub struct Network {
    pub n_pads: usize,
    pub pos_m: Vec<(f64, f64)>,
    /// CSR: lanes leaving pad p are `row_offsets[p]..row_offsets[p + 1]`.
    pub row_offsets: Vec<u32>,
    pub lane_from: Vec<PadId>,
    pub lane_to: Vec<PadId>,
    pub lane_len_m: Vec<f32>,
    pub reverse_lane: Vec<LaneId>,
    /// `[a * n + b]`: shortest route length a -> b in m (0 on the diagonal).
    pub route_len_m: Vec<f64>,
    /// `[a * n + b]`: first lane of the shortest route a -> b, NO_LANE on the diagonal.
    pub next_lane: Vec<LaneId>,
    pub truck_free_flow_s: Vec<f32>,
    pub road_m: Vec<f32>,
}

#[derive(Copy, Clone, PartialEq)]
struct HeapItem {
    dist: f64,
    pad: usize,
}

impl Eq for HeapItem {}

impl Ord for HeapItem {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .dist
            .total_cmp(&self.dist)
            .then(other.pad.cmp(&self.pad))
    }
}

impl PartialOrd for HeapItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Network {
    pub fn build(cfg: &Config) -> Network {
        let n = cfg.pads.len();
        let pos_m: Vec<(f64, f64)> = cfg
            .pads
            .iter()
            .map(|p| enu_from_geo(p.lat_deg, p.lon_deg, cfg.origin_lat_deg, cfg.origin_lon_deg))
            .collect();
        let lanes = match &cfg.network_files {
            Some(files) => files.lanes.clone(),
            None => lanes_from_positions(&cfg.lanes, &pos_m),
        };
        let lane_from: Vec<PadId> = (0..n)
            .flat_map(|p| {
                let count = lanes.row_offsets[p + 1] - lanes.row_offsets[p];
                std::iter::repeat_n(pad_id(p), count as usize)
            })
            .collect();
        let LaneFile {
            row_offsets,
            to_pad: lane_to,
            length_m: lane_len_m,
            reverse_lane,
        } = lanes;
        let mut net = Network {
            n_pads: n,
            pos_m,
            row_offsets,
            lane_from,
            lane_to,
            lane_len_m,
            reverse_lane,
            route_len_m: vec![0.0; n * n],
            next_lane: vec![NO_LANE; n * n],
            truck_free_flow_s: vec![0.0; n * n],
            road_m: vec![0.0; n * n],
        };
        net.compute_routes();
        match (
            cfg.truck.synthetic,
            cfg.network_files.as_ref().and_then(|f| f.truck.as_ref()),
        ) {
            (Some(synthetic), _) => net.compute_truck(synthetic),
            (None, Some(file)) => {
                net.truck_free_flow_s.copy_from_slice(&file.free_flow_s);
                net.road_m.copy_from_slice(&file.road_m);
            }
            (None, None) => unreachable!("the config loader requires a matrix source (R17)"),
        }
        net
    }

    pub fn lanes_from(&self, pad: PadId) -> std::ops::Range<usize> {
        let p = usize::from(pad);
        self.row_offsets[p] as usize..self.row_offsets[p + 1] as usize
    }

    fn compute_routes(&mut self) {
        let n = self.n_pads;
        for src in 0..n {
            let mut d = vec![f64::INFINITY; n];
            d[src] = 0.0;
            let mut heap = BinaryHeap::new();
            heap.push(HeapItem {
                dist: 0.0,
                pad: src,
            });
            while let Some(HeapItem { dist, pad }) = heap.pop() {
                if dist > d[pad] {
                    continue;
                }
                for l in self.lanes_from(pad_id(pad)) {
                    let v = usize::from(self.lane_to[l]);
                    let nd = dist + f64::from(self.lane_len_m[l]);
                    if nd < d[v] {
                        d[v] = nd;
                        heap.push(HeapItem { dist: nd, pad: v });
                    }
                }
            }
            self.route_len_m[src * n..(src + 1) * n].copy_from_slice(&d);
        }
        for a in 0..n {
            for b in 0..n {
                if a == b {
                    continue;
                }
                let target = self.route_len_m[a * n + b];
                let first = self
                    .lanes_from(pad_id(a))
                    .find(|&l| {
                        let via = f64::from(self.lane_len_m[l])
                            + self.route_len_m[usize::from(self.lane_to[l]) * n + b];
                        via <= target + ROUTE_TIE_TOLERANCE_M
                    })
                    .expect("connected graph has a first lane");
                self.next_lane[a * n + b] = lane_id(first);
            }
        }
    }

    fn compute_truck(&mut self, synthetic: SyntheticTruckCfg) {
        let n = self.n_pads;
        for a in 0..n {
            for b in 0..n {
                let road = dist(self.pos_m[a], self.pos_m[b]) * synthetic.detour_factor;
                #[allow(clippy::cast_possible_truncation)]
                {
                    self.road_m[a * n + b] = road as f32;
                    self.truck_free_flow_s[a * n + b] =
                        (road / synthetic.free_flow_speed_mps) as f32;
                }
            }
        }
    }

    pub fn route_len(&self, a: PadId, b: PadId) -> f64 {
        self.route_len_m[usize::from(a) * self.n_pads + usize::from(b)]
    }

    pub fn next_lane(&self, a: PadId, b: PadId) -> LaneId {
        self.next_lane[usize::from(a) * self.n_pads + usize::from(b)]
    }

    pub fn truck_s(&self, a: PadId, b: PadId) -> f64 {
        f64::from(self.truck_free_flow_s[usize::from(a) * self.n_pads + usize::from(b)])
    }

    pub fn road(&self, a: PadId, b: PadId) -> f64 {
        f64::from(self.road_m[usize::from(a) * self.n_pads + usize::from(b)])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ids::{ReturnPolicy, ServiceMode};

    fn synth(topology: NetworkTopology, n: u16, k: u16) -> SyntheticNetworkCfg {
        SyntheticNetworkCfg {
            topology,
            n_pads: n,
            radius_m: 3000.0,
            mesh_neighbors: k,
            service_mode: ServiceMode::Auto,
            return_policy: ReturnPolicy::KeepLocal,
            initial_empty_pods: 2,
            parking_capacity: 3,
            hub_parking_capacity: 5,
        }
    }

    #[test]
    fn enu_round_trip() {
        let (x, y) = enu_from_geo(40.45, -79.9, 40.44, -79.99);
        let (lat, lon) = geo_from_enu(x, y, 40.44, -79.99);
        assert!((lat - 40.45).abs() < 1e-9 && (lon + 79.9).abs() < 1e-9);
    }

    #[test]
    fn hub_spoke_geometry() {
        let (pads, lanes) = generate(&synth(NetworkTopology::HubSpoke, 6, 1), 40.0, -80.0, 1);
        assert_eq!(pads.len(), 6);
        assert!(pads[0].is_hub && pads[0].parking_capacity == 5);
        assert!(
            pads[1..]
                .iter()
                .all(|p| !p.is_hub && p.parking_capacity == 3)
        );
        assert_eq!(lanes, vec![(0, 1), (0, 2), (0, 3), (0, 4), (0, 5)]);
        for p in &pads[1..] {
            let (x, y) = enu_from_geo(p.lat_deg, p.lon_deg, 40.0, -80.0);
            assert!((x.hypot(y) - 3000.0).abs() < 1e-6);
        }
    }

    #[test]
    fn mesh_is_connected_deterministic_and_seed_dependent() {
        let a = generate(&synth(NetworkTopology::Mesh, 12, 2), 40.0, -80.0, 9);
        let b = generate(&synth(NetworkTopology::Mesh, 12, 2), 40.0, -80.0, 9);
        let c = generate(&synth(NetworkTopology::Mesh, 12, 2), 40.0, -80.0, 10);
        assert_eq!(a.1, b.1);
        assert!((a.0[3].lat_deg - b.0[3].lat_deg).abs() < f64::EPSILON);
        assert!((a.0[3].lat_deg - c.0[3].lat_deg).abs() > 0.0);
        // Spanning tree edges guarantee n - 1 <= lanes; every pad has at least k neighbours.
        assert!(a.1.len() >= 11);
        for i in 0..12u16 {
            assert!(a.1.iter().filter(|&&(x, y)| x == i || y == i).count() >= 2);
        }
    }
}
