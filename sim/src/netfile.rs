//! Readers for the offline pipeline's network files (spec 9.0 to 9.2): `lanes.csr.bin` and
//! `truck_matrix.bin`. Every reader rule of 9.0 is enforced, plus the two that tie a file to the
//! pad catalog it was built for: `count_a` must equal the catalog's pad count and `source_hash`
//! the catalog's FNV-1a hash. Any violation is an error naming the rule; nothing is repaired.

use crate::ids::{LaneId, PadId};

const HEADER_BYTES: usize = 32;
const ALIGN: usize = 8;
const VERSION_MAJOR: u16 = 1;
const LANES_MAGIC: &[u8; 8] = b"DDLANES\0";
/// Minor 1 added `max_terrain_m` and `ground_risk_per_km2`, which this reader validates.
const LANES_MINOR: u16 = 1;
const TRUCK_MAGIC: &[u8; 8] = b"DDTRUCK\0";
/// Minor 1 added `road_m`, which the cost baseline needs.
const TRUCK_MINOR: u16 = 1;

/// Directed lanes in CSR order (spec 9.1). Terrain and ground risk are validated, not kept: the
/// core does not use them (spec 2.6).
#[derive(Debug, Clone)]
pub struct LaneFile {
    pub row_offsets: Vec<u32>,
    pub to_pad: Vec<PadId>,
    pub length_m: Vec<f32>,
    pub reverse_lane: Vec<LaneId>,
}

impl LaneFile {
    /// Undirected lanes as `(a, b)` with `a < b`, in CSR order.
    pub fn undirected(&self) -> Vec<(PadId, PadId)> {
        let mut out = Vec::with_capacity(self.to_pad.len() / 2);
        for from in 0..self.row_offsets.len() - 1 {
            let a = PadId::try_from(from).expect("pad count validated against the catalog");
            for l in self.row_offsets[from] as usize..self.row_offsets[from + 1] as usize {
                if a < self.to_pad[l] {
                    out.push((a, self.to_pad[l]));
                }
            }
        }
        out
    }
}

/// Row-major `[from * n + to]` matrices of spec 9.2.
#[derive(Debug, Clone)]
pub struct TruckFile {
    pub free_flow_s: Vec<f32>,
    pub road_m: Vec<f32>,
}

struct Header {
    minor: u16,
    data_start: usize,
    count_b: u32,
}

fn u16_at(b: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([b[at], b[at + 1]])
}

fn u32_at(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(b[at..at + 4].try_into().expect("4 bytes"))
}

fn u64_at(b: &[u8], at: usize) -> u64 {
    u64::from_le_bytes(b[at..at + 8].try_into().expect("8 bytes"))
}

fn header(
    b: &[u8],
    magic: &[u8; 8],
    min_minor: u16,
    n_pads: usize,
    source_hash: u64,
) -> Result<Header, String> {
    if b.len() < HEADER_BYTES {
        return Err(format!("{} bytes, shorter than the header", b.len()));
    }
    if &b[0..8] != magic {
        return Err(format!("magic {:?}, expected {:?}", &b[0..8], magic));
    }
    let (major, minor) = (u16_at(b, 8), u16_at(b, 10));
    if major != VERSION_MAJOR || minor < min_minor {
        return Err(format!(
            "version {major}.{minor}, this core reads {VERSION_MAJOR}.{min_minor} or a later minor"
        ));
    }
    let data_start = u32_at(b, 12) as usize;
    if data_start < HEADER_BYTES {
        return Err(format!("header_bytes {data_start} < {HEADER_BYTES}"));
    }
    let count_a = u32_at(b, 16);
    if count_a as usize != n_pads {
        return Err(format!(
            "count_a {count_a} pads, the catalog has {n_pads} (size mismatch)"
        ));
    }
    let hash = u64_at(b, 24);
    if hash != source_hash {
        return Err(format!(
            "source_hash {hash:016x} does not match pads.toml ({source_hash:016x}); rebuild the network files"
        ));
    }
    Ok(Header {
        minor,
        data_start,
        count_b: u32_at(b, 20),
    })
}

/// Sequential reader over the 8-byte-aligned arrays that follow the header.
struct Arrays<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl<'a> Arrays<'a> {
    fn take(&mut self, width: usize, n: usize, name: &str) -> Result<&'a [u8], String> {
        self.at = self.at.next_multiple_of(ALIGN);
        let end = n
            .checked_mul(width)
            .and_then(|len| self.at.checked_add(len))
            .ok_or_else(|| format!("{name}: length overflows"))?;
        if end > self.bytes.len() {
            return Err(format!(
                "{name}: file shorter than its layout (size mismatch)"
            ));
        }
        let out = &self.bytes[self.at..end];
        self.at = end;
        Ok(out)
    }

    fn u16s(&mut self, n: usize, name: &str) -> Result<Vec<u16>, String> {
        Ok(self
            .take(2, n, name)?
            .as_chunks::<2>()
            .0
            .iter()
            .map(|c| u16::from_le_bytes(*c))
            .collect())
    }

    fn u32s(&mut self, n: usize, name: &str) -> Result<Vec<u32>, String> {
        Ok(self
            .take(4, n, name)?
            .as_chunks::<4>()
            .0
            .iter()
            .map(|c| u32::from_le_bytes(*c))
            .collect())
    }

    fn f32s(&mut self, n: usize, name: &str) -> Result<Vec<f32>, String> {
        let v: Vec<f32> = self
            .take(4, n, name)?
            .as_chunks::<4>()
            .0
            .iter()
            .map(|c| f32::from_le_bytes(*c))
            .collect();
        if v.iter().any(|x| !x.is_finite()) {
            return Err(format!("{name}: NaN or infinity"));
        }
        Ok(v)
    }

    /// At the reader's own minor version nothing may follow the layout; a later minor may
    /// append arrays this reader does not know.
    fn finish(&self, minor: u16, own_minor: u16) -> Result<(), String> {
        if minor == own_minor && self.at != self.bytes.len() {
            return Err(format!(
                "{} bytes beyond the version {VERSION_MAJOR}.{minor} layout (size mismatch)",
                self.bytes.len() - self.at
            ));
        }
        Ok(())
    }
}

pub fn read_lanes(bytes: &[u8], n_pads: usize, source_hash: u64) -> Result<LaneFile, String> {
    let h = header(bytes, LANES_MAGIC, LANES_MINOR, n_pads, source_hash)?;
    let n_lanes = h.count_b as usize;
    let mut a = Arrays {
        bytes,
        at: h.data_start,
    };
    let row_offsets = a.u32s(n_pads + 1, "row_offsets")?;
    let to_pad = a.u16s(n_lanes, "to_pad")?;
    let length_m = a.f32s(n_lanes, "length_m")?;
    let reverse_lane = a.u32s(n_lanes, "reverse_lane")?;
    a.f32s(n_lanes, "max_terrain_m")?;
    let risk = a.f32s(n_lanes, "ground_risk_per_km2")?;
    a.finish(h.minor, LANES_MINOR)?;

    if row_offsets[0] != 0
        || row_offsets[n_pads] != h.count_b
        || row_offsets.windows(2).any(|w| w[0] > w[1])
    {
        return Err("row_offsets: must start at 0, not decrease and end at n_lanes".into());
    }
    let mut from = vec![0usize; n_lanes];
    for p in 0..n_pads {
        from[row_offsets[p] as usize..row_offsets[p + 1] as usize].fill(p);
    }
    for l in 0..n_lanes {
        let to = usize::from(to_pad[l]);
        if to >= n_pads || to == from[l] {
            return Err(format!("to_pad[{l}] = {to}: out of range or a self lane"));
        }
        if l > 0 && from[l] == from[l - 1] && to_pad[l] <= to_pad[l - 1] {
            return Err(format!(
                "lane {l}: not sorted by (from_pad, to_pad) or a duplicate"
            ));
        }
        if length_m[l] <= 0.0 || risk[l] < 0.0 {
            return Err(format!(
                "lane {l}: length_m must be > 0 and ground_risk_per_km2 >= 0"
            ));
        }
        let r = reverse_lane[l] as usize;
        let reverse_ok = r < n_lanes
            && reverse_lane[r] as usize == l
            && from[r] == to
            && usize::from(to_pad[r]) == from[l]
            && length_m[r].to_bits() == length_m[l].to_bits();
        if !reverse_ok {
            return Err(format!(
                "reverse_lane[{l}] = {r}: not the same lane with ends swapped and equal length"
            ));
        }
    }
    Ok(LaneFile {
        row_offsets,
        to_pad,
        length_m,
        reverse_lane,
    })
}

pub fn read_truck(bytes: &[u8], n_pads: usize, source_hash: u64) -> Result<TruckFile, String> {
    let h = header(bytes, TRUCK_MAGIC, TRUCK_MINOR, n_pads, source_hash)?;
    let n2 = n_pads * n_pads;
    let mut a = Arrays {
        bytes,
        at: h.data_start,
    };
    let free_flow_s = a.f32s(n2, "free_flow_s")?;
    let road_m = a.f32s(n2, "road_m")?;
    a.finish(h.minor, TRUCK_MINOR)?;
    for (name, m) in [("free_flow_s", &free_flow_s), ("road_m", &road_m)] {
        for i in 0..n_pads {
            for j in 0..n_pads {
                let v = m[i * n_pads + j];
                let ok = if i == j { v == 0.0 } else { v > 0.0 };
                if !ok {
                    return Err(format!(
                        "{name}[{i}][{j}] = {v}: diagonal must be 0, every other pair > 0"
                    ));
                }
            }
        }
    }
    Ok(TruckFile {
        free_flow_s,
        road_m,
    })
}
