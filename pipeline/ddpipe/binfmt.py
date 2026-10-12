"""Writers and readers for the binary network files of spec section 9 (lanes.csr.bin 9.1,
truck_matrix.bin 9.2). The readers re-check every rule a reader must reject, so the sanity step
does not trust the writers."""

import struct
from dataclasses import dataclass

import numpy as np

HEADER = struct.Struct("<8sHHIIIQ")
HEADER_BYTES = 32
ALIGN = 8
VERSION_MAJOR = 1
LANES_MAGIC = b"DDLANES\0"
LANES_MINOR = 1
TRUCK_MAGIC = b"DDTRUCK\0"
TRUCK_MINOR = 1


def fnv1a64(data: bytes) -> int:
    h = 0xCBF29CE484222325
    for b in data:
        h = ((h ^ b) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def f32_at_least(x: np.ndarray) -> np.ndarray:
    """Nearest f32 that is not below x, so a stored length never undercuts the true one."""
    f = np.asarray(x, dtype=np.float32)
    low = f.astype(np.float64) < np.asarray(x, dtype=np.float64)
    return np.where(low, np.nextafter(f, np.float32(np.inf)), f).astype(np.float32)


def _pack(magic: bytes, minor: int, count_a: int, count_b: int, source_hash: int,
          arrays: list[np.ndarray]) -> bytes:
    out = bytearray(HEADER.pack(magic, VERSION_MAJOR, minor, HEADER_BYTES, count_a, count_b,
                                source_hash))
    for a in arrays:
        out += bytes(-len(out) % ALIGN)
        out += np.ascontiguousarray(a).astype(a.dtype.newbyteorder("<")).tobytes()
    return bytes(out)


@dataclass
class LaneGraph:
    """Directed lanes in CSR order (spec 9.1)."""
    n_pads: int
    row_offsets: np.ndarray
    to_pad: np.ndarray
    length_m: np.ndarray
    reverse_lane: np.ndarray
    max_terrain_m: np.ndarray
    ground_risk_per_km2: np.ndarray

    @property
    def from_pad(self) -> np.ndarray:
        return np.repeat(np.arange(self.n_pads), np.diff(self.row_offsets))


def lane_graph(n_pads: int, lanes) -> LaneGraph:
    """From undirected lanes with fields a, b, length_m, max_terrain_m, ground_risk_per_km2."""
    directed = sorted([(l.a, l.b, l) for l in lanes] + [(l.b, l.a, l) for l in lanes],
                      key=lambda d: (d[0], d[1]))
    index = {(a, b): i for i, (a, b, _) in enumerate(directed)}
    counts = np.bincount([a for a, _, _ in directed], minlength=n_pads)
    return LaneGraph(
        n_pads=n_pads,
        row_offsets=np.concatenate([[0], np.cumsum(counts)]).astype(np.uint32),
        to_pad=np.array([b for _, b, _ in directed], dtype=np.uint16),
        length_m=f32_at_least([l.length_m for _, _, l in directed]),
        reverse_lane=np.array([index[(b, a)] for a, b, _ in directed], dtype=np.uint32),
        max_terrain_m=np.array([l.max_terrain_m for _, _, l in directed], dtype=np.float32),
        ground_risk_per_km2=np.array([l.ground_risk_per_km2 for _, _, l in directed],
                                     dtype=np.float32),
    )


def write_lanes(g: LaneGraph, source_hash: int) -> bytes:
    return _pack(LANES_MAGIC, LANES_MINOR, g.n_pads, len(g.to_pad), source_hash,
                 [g.row_offsets, g.to_pad, g.length_m, g.reverse_lane, g.max_terrain_m,
                  g.ground_risk_per_km2])


def write_truck(free_flow_s: np.ndarray, road_m: np.ndarray, source_hash: int) -> bytes:
    n = free_flow_s.shape[0]
    return _pack(TRUCK_MAGIC, TRUCK_MINOR, n, 0, source_hash,
                 [free_flow_s.astype(np.float32).ravel(), road_m.astype(np.float32).ravel()])


class FormatError(ValueError):
    pass


def _header(data: bytes, magic: bytes, min_minor: int):
    if len(data) < HEADER_BYTES:
        raise FormatError("file shorter than the header")
    m, major, minor, header_bytes, count_a, count_b, source_hash = HEADER.unpack_from(data)
    if m != magic:
        raise FormatError(f"magic {m!r}, expected {magic!r}")
    if major != VERSION_MAJOR or minor < min_minor:
        raise FormatError(f"version {major}.{minor}, need {VERSION_MAJOR}.{min_minor} or later")
    if header_bytes < HEADER_BYTES:
        raise FormatError(f"header_bytes {header_bytes} < 32")
    return minor, header_bytes, count_a, count_b, source_hash


def _arrays(data: bytes, offset: int, specs, exact: bool):
    out = []
    for dtype, n in specs:
        offset += -offset % ALIGN
        size = np.dtype(dtype).itemsize * n
        if offset + size > len(data):
            raise FormatError("file shorter than its layout")
        out.append(np.frombuffer(data, dtype=np.dtype(dtype).newbyteorder("<"), count=n,
                                 offset=offset))
        offset += size
    if exact and offset != len(data):
        raise FormatError("trailing bytes after the layout of this version")
    return out


def _finite(name: str, a: np.ndarray) -> None:
    if not np.all(np.isfinite(a)):
        raise FormatError(f"{name}: NaN or infinity")


def read_lanes(data: bytes) -> tuple[LaneGraph, int]:
    minor, start, n, n_lanes, source_hash = _header(data, LANES_MAGIC, LANES_MINOR)
    ro, to, length, rev, terrain, risk = _arrays(
        data, start, [("u4", n + 1), ("u2", n_lanes), ("f4", n_lanes), ("u4", n_lanes),
                      ("f4", n_lanes), ("f4", n_lanes)], exact=minor == LANES_MINOR)
    g = LaneGraph(n, ro, to, length, rev, terrain, risk)
    if ro[0] != 0 or ro[-1] != n_lanes or np.any(np.diff(ro.astype(np.int64)) < 0):
        raise FormatError("row_offsets not 0..n_lanes and non-decreasing")
    frm = g.from_pad
    if np.any(to >= n) or np.any(to == frm):
        raise FormatError("to_pad out of range or a self lane")
    key = frm.astype(np.int64) * n + to
    if np.any(np.diff(key) <= 0):
        raise FormatError("lanes not strictly sorted by (from_pad, to_pad)")
    for name, a in (("length_m", length), ("max_terrain_m", terrain), ("ground_risk", risk)):
        _finite(name, a)
    if np.any(length <= 0) or np.any(risk < 0):
        raise FormatError("length_m must be > 0 and ground_risk >= 0")
    if np.any(rev >= n_lanes):
        raise FormatError("reverse_lane out of range")
    if (np.any(rev[rev] != np.arange(n_lanes)) or np.any(frm[rev] != to)
            or np.any(to[rev] != frm) or np.any(length[rev] != length)):
        raise FormatError("reverse_lane is not an involution with swapped ends and equal length")
    return g, source_hash


def read_truck(data: bytes) -> tuple[np.ndarray, np.ndarray, int]:
    minor, start, n, _, source_hash = _header(data, TRUCK_MAGIC, TRUCK_MINOR)
    ff, road = _arrays(data, start, [("f4", n * n), ("f4", n * n)], exact=minor == TRUCK_MINOR)
    ff, road = ff.reshape(n, n), road.reshape(n, n)
    for name, a in (("free_flow_s", ff), ("road_m", road)):
        _finite(name, a)
        off = ~np.eye(n, dtype=bool)
        if np.any(np.diag(a) != 0) or np.any(a[off] <= 0):
            raise FormatError(f"{name}: diagonal must be 0 and off-diagonal > 0")
    return ff, road, source_hash
