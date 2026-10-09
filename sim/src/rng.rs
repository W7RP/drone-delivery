//! Seeded random streams (spec 5.4): xoshiro256** seeded by SplitMix64, one stream per
//! `(kind, index)` so a change in one decision never shifts the draws of another.

use crate::ids::ms_from_s;

pub struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    pub fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
}

#[derive(Clone)]
pub struct Xoshiro256 {
    s: [u64; 4],
}

impl Xoshiro256 {
    pub fn from_state(s: [u64; 4]) -> Self {
        Self { s }
    }

    pub fn from_seed(seed: u64) -> Self {
        let mut sm = SplitMix64::new(seed);
        Self {
            s: [sm.next_u64(), sm.next_u64(), sm.next_u64(), sm.next_u64()],
        }
    }

    pub fn next_u64(&mut self) -> u64 {
        let result = self.s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        result
    }

    /// Uniform in [0, 1): `(next >> 11) * 2^-53`.
    pub fn uniform(&mut self) -> f64 {
        #[allow(clippy::cast_precision_loss)]
        let u = (self.next_u64() >> 11) as f64;
        u * (1.0 / 9_007_199_254_740_992.0)
    }

    pub fn bernoulli(&mut self, p: f64) -> bool {
        self.uniform() < p
    }

    /// Exponential inter-arrival in ms for `rate_per_s` > 0.
    /// Standard normal by Box-Muller, from two uniforms.
    pub fn normal(&mut self) -> f64 {
        let (u1, u2) = (self.uniform(), self.uniform());
        (-2.0 * (1.0 - u1).ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }

    pub fn exponential_ms(&mut self, rate_per_s: f64) -> u64 {
        let u = self.uniform();
        ms_from_s(-(1.0 - u).ln() / rate_per_s)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum StreamKind {
    Demand = 1,
    Marker = 2,
    Release = 3,
    Latch = 4,
    PcFault = 5,
    MsgLoss = 6,
    Network = 7,
    Repair = 8,
    Traffic = 9,
}

/// `master_r` of spec 5.4: first SplitMix64 output of `seed + replication`.
pub fn master_seed(seed: u64, replication: u32) -> u64 {
    SplitMix64::new(seed.wrapping_add(u64::from(replication))).next_u64()
}

pub fn stream(master: u64, kind: StreamKind, index: u32) -> Xoshiro256 {
    Xoshiro256::from_seed(master ^ ((kind as u64) << 56) ^ u64::from(index))
}

/// All streams of one run, preallocated per entity so no draw allocates.
pub struct Streams {
    pub demand: Vec<Xoshiro256>,
    pub marker: Vec<Xoshiro256>,
    pub release: Vec<Xoshiro256>,
    pub latch: Vec<Xoshiro256>,
    pub pc_fault: Vec<Xoshiro256>,
    pub loss_pad: Vec<Xoshiro256>,
    pub loss_drone: Vec<Xoshiro256>,
    pub repair: Vec<Xoshiro256>,
}

impl Streams {
    pub fn new(master: u64, n_flows: usize, n_pads: usize, n_drones: usize) -> Self {
        let make = |kind, n: usize, base: u32| -> Vec<Xoshiro256> {
            (0..n)
                .map(|i| stream(master, kind, base | index_u32(i)))
                .collect()
        };
        // MSG_LOSS index = (sender EntityKind code << 16) | sender id; PAD = 0, DRONE = 1.
        Self {
            demand: make(StreamKind::Demand, n_flows, 0),
            marker: make(StreamKind::Marker, n_pads, 0),
            release: make(StreamKind::Release, n_drones, 0),
            latch: make(StreamKind::Latch, n_drones, 0),
            pc_fault: make(StreamKind::PcFault, n_pads, 0),
            loss_pad: make(StreamKind::MsgLoss, n_pads, 0),
            loss_drone: make(StreamKind::MsgLoss, n_drones, 1 << 16),
            repair: make(StreamKind::Repair, n_pads, 0),
        }
    }
}

fn index_u32(i: usize) -> u32 {
    u32::try_from(i).expect("stream index fits u32 (entity counts are u16)")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splitmix_reference_value() {
        // Reference output of SplitMix64 seeded with 0 (Vigna's splitmix64.c).
        assert_eq!(SplitMix64::new(0).next_u64(), 0xE220_A839_7B1D_CDAF);
    }

    #[test]
    fn xoshiro_reference_value() {
        // With state [1, 2, 3, 4] the first output is rotl(2 * 5, 7) * 9 = 11520.
        let mut x = Xoshiro256::from_state([1, 2, 3, 4]);
        assert_eq!(x.next_u64(), 11520);
    }

    #[test]
    fn streams_are_independent_and_reproducible() {
        let m = master_seed(42, 0);
        let mut a = stream(m, StreamKind::Marker, 0);
        let mut b = stream(m, StreamKind::Marker, 0);
        let mut c = stream(m, StreamKind::Marker, 1);
        let (xa, xb, xc) = (a.next_u64(), b.next_u64(), c.next_u64());
        assert_eq!(xa, xb);
        assert_ne!(xa, xc);
        assert_ne!(master_seed(42, 0), master_seed(42, 1));
    }

    #[test]
    fn uniform_stays_in_unit_interval() {
        let mut x = Xoshiro256::from_seed(7);
        for _ in 0..10_000 {
            let u = x.uniform();
            assert!((0.0..1.0).contains(&u));
        }
    }
}
