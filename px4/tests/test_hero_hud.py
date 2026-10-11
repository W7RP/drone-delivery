"""Phase 8 acceptance: the clip's HUD agrees with the logs (docs/hero-clip.md 6).

For the camera run that tools/hero_edit.py cut (<run>/edit/hud_timeline.json):
- every change of a HUD state (drone, pad A, pad B, VTOL phase) between two consecutive frames
  of a segment is caused by a log event of that value in between: the HUD changes within one
  frame of the log timestamp, never early;
- every state event inside a shown segment is on screen by the first frame at or after it,
  unless a later event replaced it within the same frame;
- every handshake message shown carries its log send time exactly, first appears on the first
  frame at or after that time, and messages appear in send order;
- the battery and height figures are the samples' values at the frame's sim time;
- the drone-vs-truck panel equals a fresh run of the demo scenario's deliveries.csv (seed 42):
  same order count, median drone door-to-door, the median order's split, median truck time.

Usage: px4/py.sh tests/test_hero_hud.py <run dir>   (exit 1 on any failure)"""

import csv
import json
import statistics
import subprocess
import sys
import tempfile
import tomllib
from bisect import bisect_right
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import flightlog  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
SIM = REPO / "sim" / "target" / "release" / "sim"
DEMO = REPO / "configs" / "parts_logistics_pittsburgh_vtol_demo" / "scenario.toml"
SPLIT = ("pod_wait_s", "queue_s", "pad_service_s", "flight_s")
STATE_KEYS = ("drone", "pad_A", "pad_B", "vtol")


def state_events(events: list[dict], roles: dict) -> list[tuple[float, str, str]]:
    pad = {roles["A"]: "pad_A", roles["B"]: "pad_B"}
    out = []
    for e in events:
        t = e["t_us"] * 1e-6
        if e["type"] == "transition" and e["machine"] == "DRONE" and e["from"] != e["to"]:
            out.append((t, "drone", e["to"]))
        elif e["type"] == "transition" and e["machine"] == "VISIT" and e["pad_id"] in pad:
            out.append((t, pad[e["pad_id"]], e["to"]))
        elif e["type"] == "vtol":
            out.append((t, "vtol", e["state"]))
    return out


def check_states(frames, segments, events, problems):
    by_seg = {}
    for f in frames:
        if "segment" in f:
            by_seg.setdefault(f["segment"], []).append(f)
    for seg in segments:
        fs = by_seg.get(seg["index"], [])
        for prev, cur in zip(fs, fs[1:]):
            for key in STATE_KEYS:
                if cur[key] != prev[key]:
                    causes = [e for e in events if e[1] == key and e[2] == cur[key]
                              and prev["sim_t_s"] < e[0] <= cur["sim_t_s"] + 1e-6]
                    if not causes:
                        problems.append(f"frame {cur['frame']}: {key} -> {cur[key]} with no log event in "
                                        f"({prev['sim_t_s']}, {cur['sim_t_s']}]")
        if not fs:
            continue
        times = [f["sim_t_s"] for f in fs]
        for t, key, value in events:
            if not fs[0]["sim_t_s"] < t <= fs[-1]["sim_t_s"]:
                continue
            k = next(i for i, ft in enumerate(times) if ft >= t - 1e-6)
            later = [e for e in events if e[1] == key and t < e[0] <= times[k] + 1e-6]
            if fs[k][key] != value and not later:
                problems.append(f"{key} {value} at {t:.3f} s not on frame {fs[k]['frame']} ({times[k]:.3f} s)")


def check_messages(frames, msgs, problems):
    sent = {(m["sender"], m["seq"], m["name"]): m["t_us"] * 1e-6 for m in msgs}
    order = {(m["sender"], m["seq"], m["name"]): i for i, m in enumerate(msgs)}
    seen = {}
    prev_t = None
    for f in frames:
        if "messages" not in f:
            prev_t = None
            continue
        keys = [(m["sender"], m["seq"], m["name"]) for m in f["messages"]]
        if [order[k] for k in keys] != sorted(order[k] for k in keys):
            problems.append(f"frame {f['frame']}: messages out of send order")
        for m, k in zip(f["messages"], keys):
            if abs(m["t_s"] - sent[k]) > 5e-4:
                problems.append(f"frame {f['frame']}: {m['name']} shows {m['t_s']} s, logged {sent[k]:.3f} s")
            if k not in seen:
                seen[k] = f["frame"]
                if f["sim_t_s"] < sent[k] - 1e-6 or (prev_t is not None and prev_t >= sent[k] - 1e-6):
                    problems.append(f"frame {f['frame']} ({f['sim_t_s']} s): {m['name']} sent {sent[k]:.3f} s, "
                                    f"previous frame {prev_t}")
        prev_t = f["sim_t_s"]
    return len(seen)


def check_samples(frames, samples, problems):
    t = samples["t_us"] * 1e-6
    for f in frames:
        if "sim_t_s" not in f:
            continue
        i = max(0, bisect_right(t, f["sim_t_s"] + 1e-9) - 1)
        battery = round(float(samples["battery_remaining"][i]) * 100, 1)
        if f["battery_pct"] != battery:
            problems.append(f"frame {f['frame']}: battery {f['battery_pct']} %, samples {battery} %")


def demo_panel(lane_id: int) -> dict:
    cfg = tomllib.loads(DEMO.read_text())
    derived = (DEMO.parent / cfg["network_files"]["dir"])
    data = (derived / "lanes.csr.bin").read_bytes()
    # The lane's ends from lanes.csr.bin (spec 9.1): row offsets, then to_pad.
    import struct
    _, _, _, header, n_pads, n_lanes, _ = struct.unpack_from("<8sHHIIIQ", data)
    rows = struct.unpack_from(f"<{n_pads + 1}I", data, header)
    at = header + 4 * (n_pads + 1)
    at += -at % 8
    to = struct.unpack_from(f"<{n_lanes}H", data, at)
    a = next(p for p in range(n_pads) if rows[p] <= lane_id < rows[p + 1])
    b = to[lane_id]
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([str(SIM), "run", str(DEMO), "--seed", "42", "--out", tmp, "--no-event-log"],
                       check=True, capture_output=True, timeout=120)
        with open(Path(tmp) / "deliveries.csv") as f:
            orders = [r for r in csv.DictReader(f) if r["kind"] == "SHIPMENT" and r["status"] == "DELIVERED"
                      and int(r["origin_pad"]) == a and int(r["dest_pad"]) == b and r["delivered_pad"] == r["dest_pad"]]
    orders.sort(key=lambda r: (float(r["drone_door_to_door_s"]), int(r["order_id"])))
    med = orders[(len(orders) - 1) // 2]
    return {"orders": len(orders), "drone_median_s": float(med["drone_door_to_door_s"]),
            "median_order_split_s": {k: float(med[k]) for k in SPLIT},
            "truck_median_s": statistics.median_low(float(r["truck_door_to_door_s"]) for r in orders)}


def main() -> int:
    run = Path(sys.argv[1])
    doc = json.loads((run / "edit" / "hud_timeline.json").read_text())
    frames, segments = doc["frames"], doc["segments"]
    events = flightlog.read_events(run / "events.jsonl")
    msgs = flightlog.send_order([e for e in events if e["type"] == "msg"])
    states = state_events(events, flightlog.pad_roles(events))
    problems: list[str] = []
    check_states(frames, segments, states, problems)
    shown = check_messages(frames, msgs, problems)
    check_samples(frames, flightlog.read_samples(run / "flight_samples.bin"), problems)
    want = demo_panel(doc["panel"]["lane_id"])
    for k, v in want.items():
        if doc["panel"][k] != v:
            problems.append(f"panel {k}: HUD {doc['panel'][k]}, deliveries.csv {v}")
    shown_frames = sum("sim_t_s" in f for f in frames)
    for p in problems[:30]:
        print("FAIL", p)
    print(f"{shown_frames} HUD frames, {len(states)} state events, {shown} messages shown, panel "
          f"{want['orders']} orders: {'PASS' if not problems else f'{len(problems)} problems'}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
