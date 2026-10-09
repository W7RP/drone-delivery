//! Readers for the offline pipeline's binary files (spec 9.0 to 9.2, 9.7): `lanes.csr.bin`,
//! `truck_matrix.bin` and `weather.bin`. Every reader rule of 9.0 is enforced, plus, for the
//! network files, the two that tie a file to the pad catalog it was built for: `count_a` must
//! equal the catalog's pad count and `source_hash` the catalog's FNV-1a hash. Any violation is
//! an error naming the rule; nothing is repaired.

use crate::ids::{LaneId, PadId};

const HEADER_BYTES: usize = 32;
const ALIGN: usize = 8;
const VERSION_MAJOR: u16 = 1;
const LANES_MAGIC: &[u8; 8] = b"DDLANES\0";
/// Minor 1 added `max_terrain_m` and `ground_risk_per_km2`, which this reader validates.
const LANES_MINOR: u16 = 1;
const TRUCK_MAGIC: &[u8; 8] = b"DDTRUCK\0";
/// Minor 1 added `road_m`, which the cost baseline needs; minor 2 the traffic fractions,
/// which only traffic needs, so minor-1 files still load.
const TRUCK_MIN_MINOR: u16 = 1;
const TRUCK_MINOR: u16 = 2;
/// Road-class fractions of one route sum to 1 up to f32 rounding.
const FRACTION_SUM_TOLERANCE: f64 = 1e-5;
const WEATHER_MAGIC: &[u8; 8] = b"DDWTHR\0\0";
const WEATHER_MINOR: u16 = 0;
/// `wx_flags` bits of spec 9.7.
pub const WX_RAIN: u8 = 1;
pub const WX_SNOW: u8 = 2;
const WX_ALL: u8 = 0x0f;
/// Air temperature bounds of spec 9.7, deg C.
const TEMP_MIN_C: f32 = -90.0;
const TEMP_MAX_C: f32 = 60.0;
const DEG_PER_TURN: f32 = 360.0;

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
    /// version_minor 2 only.
    pub fractions: Option<RouteFractions>,
}

/// Share of each route's length per road class (motorway and trunk, primary and secondary,
/// other) and on a bridge or in a tunnel (spec 9.2 version_minor 2).
#[derive(Debug, Clone)]
pub struct RouteFractions {
    pub class: [Vec<f32>; 3],
    pub structure: Vec<f32>,
}

/// Hourly surface weather at one station (spec 9.7). Hour `i` starts `start_unix_h + i` hours
/// after 1970-01-01T00:00Z. Wind and gust are at 10 m.
#[derive(Debug, Clone)]
pub struct WeatherFile {
    pub start_unix_h: u32,
    pub wind_mps: Vec<f32>,
    pub wind_from_deg: Vec<f32>,
    pub gust_mps: Vec<f32>,
    pub temp_c: Vec<f32>,
    pub visibility_m: Vec<f32>,
    pub precip_mm_h: Vec<f32>,
    pub wx_flags: Vec<u8>,
    pub missing: Vec<u8>,
}

struct Header {
    minor: u16,
    data_start: usize,
    count_a: u32,
    count_b: u32,
    source_hash: u64,
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

fn header(b: &[u8], magic: &[u8; 8], min_minor: u16) -> Result<Header, String> {
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
    Ok(Header {
        minor,
        data_start,
        count_a: u32_at(b, 16),
        count_b: u32_at(b, 20),
        source_hash: u64_at(b, 24),
    })
}

/// A network file's header, which must match the pad catalog it is read with.
fn network_header(
    b: &[u8],
    magic: &[u8; 8],
    min_minor: u16,
    n_pads: usize,
    source_hash: u64,
) -> Result<Header, String> {
    let h = header(b, magic, min_minor)?;
    if h.count_a as usize != n_pads {
        return Err(format!(
            "count_a {} pads, the catalog has {n_pads} (size mismatch)",
            h.count_a
        ));
    }
    if h.source_hash != source_hash {
        return Err(format!(
            "source_hash {:016x} does not match pads.toml ({source_hash:016x}); rebuild the network files",
            h.source_hash
        ));
    }
    Ok(h)
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

    fn u8s(&mut self, n: usize, name: &str) -> Result<Vec<u8>, String> {
        Ok(self.take(1, n, name)?.to_vec())
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
    let h = network_header(bytes, LANES_MAGIC, LANES_MINOR, n_pads, source_hash)?;
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
    let h = network_header(bytes, TRUCK_MAGIC, TRUCK_MIN_MINOR, n_pads, source_hash)?;
    let n2 = n_pads * n_pads;
    let mut a = Arrays {
        bytes,
        at: h.data_start,
    };
    let free_flow_s = a.f32s(n2, "free_flow_s")?;
    let road_m = a.f32s(n2, "road_m")?;
    let fractions = if h.minor >= TRUCK_MINOR {
        Some(RouteFractions {
            class: [
                a.f32s(n2, "frac_motorway")?,
                a.f32s(n2, "frac_primary")?,
                a.f32s(n2, "frac_other")?,
            ],
            structure: a.f32s(n2, "frac_structure")?,
        })
    } else {
        None
    };
    a.finish(h.minor, h.minor.min(TRUCK_MINOR))?;
    if let Some(f) = &fractions {
        check_fractions(f, n_pads)?;
    }
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
        fractions,
    })
}

fn check_fractions(f: &RouteFractions, n_pads: usize) -> Result<(), String> {
    for i in 0..n_pads {
        for j in 0..n_pads {
            let k = i * n_pads + j;
            let values = [f.class[0][k], f.class[1][k], f.class[2][k], f.structure[k]];
            let in_range = if i == j {
                values.iter().all(|&v| v == 0.0)
            } else {
                values.iter().all(|v| (0.0..=1.0).contains(v))
            };
            let sum: f64 = values[..3].iter().map(|&v| f64::from(v)).sum();
            if !in_range || (i != j && (sum - 1.0).abs() > FRACTION_SUM_TOLERANCE) {
                return Err(format!(
                    "route fractions [{i}][{j}]: diagonal 0, each in [0, 1], classes summing to 1"
                ));
            }
        }
    }
    Ok(())
}

pub fn read_weather(bytes: &[u8]) -> Result<WeatherFile, String> {
    let h = header(bytes, WEATHER_MAGIC, WEATHER_MINOR)?;
    let n = h.count_a as usize;
    if n == 0 {
        return Err("count_a: no hours".into());
    }
    let mut a = Arrays {
        bytes,
        at: h.data_start,
    };
    let w = WeatherFile {
        start_unix_h: h.count_b,
        wind_mps: a.f32s(n, "wind_mps")?,
        wind_from_deg: a.f32s(n, "wind_from_deg")?,
        gust_mps: a.f32s(n, "gust_mps")?,
        temp_c: a.f32s(n, "temp_c")?,
        visibility_m: a.f32s(n, "visibility_m")?,
        precip_mm_h: a.f32s(n, "precip_mm_h")?,
        wx_flags: a.u8s(n, "wx_flags")?,
        missing: a.u8s(n, "missing")?,
    };
    a.finish(h.minor, WEATHER_MINOR)?;
    for i in 0..n {
        let ok = w.wind_mps[i] >= 0.0
            && w.gust_mps[i] >= w.wind_mps[i]
            && (0.0..DEG_PER_TURN).contains(&w.wind_from_deg[i])
            && (TEMP_MIN_C..=TEMP_MAX_C).contains(&w.temp_c[i])
            && w.visibility_m[i] >= 0.0
            && w.precip_mm_h[i] >= 0.0
            && w.wx_flags[i] & !WX_ALL == 0
            && w.missing[i] <= 1;
        if !ok {
            return Err(format!("hour {i}: a value outside the ranges of spec 9.7"));
        }
    }
    Ok(w)
}
