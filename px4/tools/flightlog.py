"""Reader and validator for the logs of one SITL run (docs/spec.md 9.3, 9.5, 9.6):
flight_log.bin, flight_samples.bin, events.jsonl and visits.json.

Usage: flightlog.py validate <run_dir> [<run_dir> ...]   exit 1 if any run fails a check
       flightlog.py summary <run_dir>                     per-phase times and energy as JSON

Checks: spec 9.0 headers and record sizes; monotonic sample and event timestamps; the drone and
pad state sequences equal the spec's normal visit (tools/expected.py); handshake messages
appear once each, in the spec's order (send order: send time, then per-sender sequence
number), with gap-free per-sender sequence numbers; every state has a duration; the run ended COMPLETE; the lane record matches the
transitions that bound it."""

import json
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import expected  # noqa: E402

HEADER = struct.Struct("<8sHHIIIQ")
LANE = struct.Struct("<QQIIffffHHHBx")
SAMPLE_DTYPE = np.dtype([
    ("t_us", "<u8"), ("pos_ned_m", "<f4", 3), ("vel_ned_mps", "<f4", 3), ("q_ned_frd", "<f4", 4),
    ("motor_radps", "<f4", 4), ("battery_v", "<f4"), ("battery_a", "<f4"),
    ("battery_remaining", "<f4"), ("truth_enu_m", "<f4", 3), ("drone_mode", "u1"),
    ("nav_state", "u1"), ("arming_state", "u1"), ("pods_latched", "u1"), ("pusher_radps", "<f4"),
])
assert SAMPLE_DTYPE.itemsize == 96 and LANE.size == 48
PX4_SITL = 1
DRONE_MODES = ["PARKED", "ON_PAD", "TAKEOFF", "EN_ROUTE", "HOLDING", "APPROACH", "GO_AROUND",
               "HOPPING", "GROUNDED"]


class LogError(Exception):
    pass


def _header(data: bytes, magic: bytes, record_bytes: int):
    if len(data) < HEADER.size:
        raise LogError("shorter than the 32-byte header")
    m, major, minor, header_bytes, count_a, count_b, source_hash = HEADER.unpack_from(data)
    if m != magic or major != 1 or header_bytes < 32 or count_a != record_bytes:
        raise LogError(f"bad header: magic {m!r}, version {major}.{minor}, record {count_a} B")
    if source_hash != 0:
        raise LogError("PX4 calibration logs carry source_hash 0 (spec 9.0)")
    if (len(data) - header_bytes) % record_bytes:
        raise LogError("length is not a whole number of records")
    return header_bytes


@dataclass
class Lane:
    t_start_ms: int
    t_end_ms: int
    lane_id: int
    order_id: int
    payload_kg: float
    energy_j: float
    soc_start: float
    soc_end: float
    drone_id: int
    from_pad: int
    to_pad: int
    source: int


def read_lanes(path: Path) -> list[Lane]:
    data = path.read_bytes()
    start = _header(data, b"DDFLOG\0\0", LANE.size)
    lanes = [Lane(*LANE.unpack_from(data, at)) for at in range(start, len(data), LANE.size)]
    for l in lanes:
        if l.t_end_ms < l.t_start_ms or l.payload_kg < 0 or l.energy_j < 0 or l.source != PX4_SITL:
            raise LogError(f"lane record out of range: {l}")
        if not (0 <= l.soc_start <= 1 and 0 <= l.soc_end <= 1):
            raise LogError(f"soc outside 0..1: {l}")
    return lanes


def read_samples(path: Path) -> np.ndarray:
    data = path.read_bytes()
    start = _header(data, b"DDSAMPL\0", SAMPLE_DTYPE.itemsize)
    return np.frombuffer(data, dtype=SAMPLE_DTYPE, offset=start)


def read_events(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def pad_roles(events: list[dict]) -> dict[str, int]:
    """A = the pad the drone departs, B = the pad that grants it a landing."""
    msgs = [e for e in events if e["type"] == "msg"]
    a = next(e["pad_id"] for e in msgs if e["name"] == "DEPART_REQUEST")
    b = next(e["pad_id"] for e in msgs if e["name"] == "LANDING_GRANT")
    return {"A": a, "B": b}


def sender_of(m: dict):
    """Spec 4.1 numbers messages per sender; each pad controller is its own sender."""
    return (m["sender"], m["pad_id"]) if m["sender"] == "PAD" else (m["sender"], None)


def send_order(msgs: list[dict]) -> list[dict]:
    """Messages in the order they were sent: by send time, then, for one sender, by sequence
    number. The logger's receive order is not that order: DDS delivers different topics
    independently, so two messages a pad sends in one sim tick can arrive swapped. Messages of
    different senders sent in the same tick keep their receive order, which follows causality
    (a reply is received after its request)."""
    indexed = list(enumerate(msgs))
    indexed.sort(key=lambda im: (im[1]["t_us"], im[0]))
    out = [m for _, m in indexed]
    i = 0
    while i < len(out):
        j = i
        while j < len(out) and out[j]["t_us"] == out[i]["t_us"]:
            j += 1
        group = out[i:j]
        for key in {sender_of(m) for m in group}:
            slots = [k for k, m in enumerate(group) if sender_of(m) == key]
            ordered = sorted((group[k] for k in slots), key=lambda m: m["seq"])
            for k, m in zip(slots, ordered):
                group[k] = m
        out[i:j] = group
        i = j
    return out


def check(run: Path) -> list[str]:
    problems: list[str] = []

    def need(ok: bool, what: str):
        if not ok:
            problems.append(what)

    try:
        events = read_events(run / "events.jsonl")
        samples = read_samples(run / "flight_samples.bin")
        lanes = read_lanes(run / "flight_log.bin")
        visits = json.loads((run / "visits.json").read_text())
    except (OSError, LogError, json.JSONDecodeError, KeyError) as e:
        return [f"unreadable: {e}"]

    end = [e for e in events if e["type"] == "run_status"]
    need(len(end) == 1 and end[0]["status"] == "COMPLETE", f"run status {end}")
    t = [e["t_us"] for e in events]
    need(all(a <= b for a, b in zip(t, t[1:])), "event timestamps not monotonic")
    need(len(samples) > 0 and bool(np.all(np.diff(samples["t_us"].astype(np.int64)) > 0)),
         "sample timestamps not strictly increasing")

    roles = pad_roles(events)
    trans = [e for e in events if e["type"] == "transition"]
    def seq(machine, pad=None):
        return [(e["row"], e["from"], e["to"]) for e in trans
                if e["machine"] == machine and (pad is None or e["pad_id"] == pad)]
    need(seq("VISIT", roles["A"]) == expected.VISIT_A, f"pad A visit {seq('VISIT', roles['A'])}")
    need(seq("VISIT", roles["B"]) == expected.VISIT_B, f"pad B visit {seq('VISIT', roles['B'])}")
    need(seq("DRONE") == expected.DRONE, f"drone sequence {seq('DRONE')}")

    msgs = send_order([e for e in events if e["type"] == "msg"])
    got = [(m["sender"], m["name"]) for m in msgs]
    want = [(s, n) for s, n, _, _ in expected.MESSAGES]
    need(got == want, f"messages {got}")
    if got == want:
        for m, (_, name, role, code) in zip(msgs, expected.MESSAGES):
            if role is not None:
                need(m["pad_id"] == roles[role], f"{name} names pad {m['pad_id']}, expected {role}")
            if code is not None:
                need(m["code"] == code, f"{name} code {m['code']!r}, expected {code}")
        sent = [m["t_us"] for m in msgs]
        need(all(a <= b for a, b in zip(sent, sent[1:])), "message send times out of spec order")
    for sender in {sender_of(m) for m in msgs}:
        s = [m["seq"] for m in msgs if sender_of(m) == sender]
        need(s == list(range(1, len(s) + 1)), f"{sender} sequence numbers {s}")

    durations = visits["state_durations"]
    need(len(durations) == len(trans) - sum(1 for e in trans if e["machine"] == "PAD"),
         "a state without a duration")
    need(all(d["duration_s"] is not None and d["duration_s"] >= 0 for d in durations),
         "negative or missing state duration")
    landings = visits["landings"]
    need([l["spot"] for l in landings] == ["IN", "OUT"] and
         all(l["error_cm"] is not None for l in landings), f"landings {landings}")

    need(len(lanes) == 1, f"{len(lanes)} lane records, expected 1")
    if lanes:
        d02 = next(e["t_us"] for e in trans if e["row"] == "D02")
        d03 = next(e["t_us"] for e in trans if e["row"] == "D03")
        l = lanes[0]
        need((l.t_start_ms, l.t_end_ms) == (d02 // 1000, d03 // 1000), "lane times != D02/D03")
        need((l.from_pad, l.to_pad) == (roles["A"], roles["B"]), "lane pads != A, B")
    return problems


def phase_times(events: list[dict]) -> dict[str, float]:
    """Seconds per flight phase from the drone transitions: takeoff (D01 to D02), cruise (lane,
    D02 to D03), hold at the fix (D03 to D04), approach and landing at IN (D04 to D05), hop
    (D15 into HOPPING to D15 out of it); total = takeoff to the OUT touchdown."""
    d = [e for e in events if e["type"] == "transition" and e["machine"] == "DRONE"]
    def at(row, to=None, frm=None):
        return next(e["t_us"] for e in d if e["row"] == row and (to is None or e["to"] == to)
                    and (frm is None or e["from"] == frm)) * 1e-6
    t = {"takeoff_start": at("D01"), "top_of_climb": at("D02"), "fix": at("D03"),
         "grant": at("D04"), "landed_in": at("D05"), "hop_start": at("D15", to="HOPPING"),
         "landed_out": at("D15", frm="HOPPING")}
    return {
        "takeoff_s": t["top_of_climb"] - t["takeoff_start"],
        "cruise_s": t["fix"] - t["top_of_climb"],
        "hold_s": t["grant"] - t["fix"],
        "approach_landing_s": t["landed_in"] - t["grant"],
        "hop_s": t["landed_out"] - t["hop_start"],
        "airborne_total_s": (t["landed_in"] - t["takeoff_start"]) + (t["landed_out"] - t["hop_start"]),
        "_times": t,
    }


def energy_between(samples: np.ndarray, power_w: np.ndarray, t0: float, t1: float) -> float:
    t = samples["t_us"] * 1e-6
    m = (t >= t0) & (t < t1)
    if m.sum() < 2:
        return 0.0
    return float(np.trapezoid(power_w[m], t[m]))


def summary(run: Path) -> dict:
    events = read_events(run / "events.jsonl")
    samples = read_samples(run / "flight_samples.bin")
    lanes = read_lanes(run / "flight_log.bin")
    visits = json.loads((run / "visits.json").read_text())
    params = (run / "ros_params.yaml").read_text()
    import yaml
    logger = yaml.safe_load(params)["flight_logger"]["ros__parameters"]
    k, r, rho = logger["motor_constant"], logger["rotor_radius_m"], logger["air_density_kgpm3"]
    omega = samples["motor_radps"].astype(np.float64)
    power = (k * omega**2) ** 1.5 / np.sqrt(2 * rho * np.pi * r * r)
    power = power.sum(axis=1)
    p = phase_times(events)
    t = p.pop("_times")
    energy = {
        "takeoff_j": energy_between(samples, power, t["takeoff_start"], t["top_of_climb"]),
        "cruise_j": energy_between(samples, power, t["top_of_climb"], t["fix"]),
        "hold_approach_landing_j": energy_between(samples, power, t["fix"], t["landed_in"]),
        "hop_j": energy_between(samples, power, t["hop_start"], t["landed_out"]),
    }
    energy["total_j"] = sum(energy.values())
    lane = lanes[0] if lanes else None
    return {
        "run": run.name,
        "payload_kg": logger["payload_kg"],
        "phases_s": {k_: round(v, 2) for k_, v in p.items()},
        "energy_proxy_j": {k_: round(v) for k_, v in energy.items()},
        "lane_record": None if lane is None else {
            "lane_id": lane.lane_id, "duration_s": (lane.t_end_ms - lane.t_start_ms) / 1000,
            "energy_j": round(lane.energy_j), "soc_start": round(lane.soc_start, 4),
            "soc_end": round(lane.soc_end, 4)},
        "landing_error_cm": {l["spot"]: round(l["error_cm"], 2) for l in visits["landings"]},
    }


def main() -> int:
    if len(sys.argv) < 3 or sys.argv[1] not in ("validate", "summary"):
        print(__doc__)
        return 2
    if sys.argv[1] == "summary":
        print(json.dumps(summary(Path(sys.argv[2])), indent=1))
        return 0
    failed = 0
    for arg in sys.argv[2:]:
        problems = check(Path(arg))
        print(f"{'PASS' if not problems else 'FAIL'} {arg}")
        for p in problems:
            print(f"  - {p}")
        failed += bool(problems)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
