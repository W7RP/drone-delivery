"""Phase 8 step 6: cut the clip from one camera flight (docs/hero-clip.md 6).

Rule-based, from the run's logs and pipeline/hero/hero.yaml (`edit`): the visit is split into
segments at anchors found in events.jsonl (a message, a state transition, a VTOL phase) or in
the samples (the drone down to its approach height over pad B). Each segment names a camera and
plays at 1x, or is compressed to a set length and labelled "x N". Output frames sample an exact
30 Hz grid of sim time (scaled by N in compressed segments); each takes the camera frame with the
latest stamp at or before its sim time (tools/frame_recorder.py stamps).

The HUD comes from the logs only: drone, pad A and pad B states, the VTOL phase, the last three
handshake messages, the sim timer, height above the terrain mesh, PX4's battery %, and the
drone-vs-truck panel of pipeline/hero/out/panel.json (the demo scenario's own deliveries.csv
numbers). Every frame's HUD content goes to <run>/edit/hud_timeline.json, which
px4/tests/test_hero_hud.py checks against the logs.

Usage: hero_edit.py --run DIR --out FILE.mp4"""

import argparse
import json
import math
import subprocess
import sys
from bisect import bisect_right
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parent))
import flightlog  # noqa: E402

PX4 = Path(__file__).resolve().parents[1]
REPO = PX4.parent
HERO_CFG = REPO / "pipeline" / "hero" / "hero.yaml"
PANEL = REPO / "pipeline" / "hero" / "out" / "panel.json"
FPS = 30
W, H = 1280, 720
PAD_LETTER = {}


def ffmpeg_exe() -> str:
    import imageio_ffmpeg
    return imageio_ffmpeg.get_ffmpeg_exe()


class Logs:
    """Events and samples of one run, with times in sim seconds."""

    def __init__(self, run: Path):
        self.events = flightlog.read_events(run / "events.jsonl")
        for e in self.events:
            e["t"] = e["t_us"] * 1e-6
        self.msgs = flightlog.send_order([e for e in self.events if e["type"] == "msg"])
        self.samples = flightlog.read_samples(run / "flight_samples.bin")
        self.t = self.samples["t_us"] * 1e-6
        self.roles = flightlog.pad_roles(self.events)

    def anchor(self, spec: str, scene: dict, cfg: dict) -> float:
        """'msg:NAME[#n]', 'drone:ROW[#n]', 'vtol:STATE[#n]', 'run_status' or 'approach_height'."""
        name, _, nth = spec.partition("#")
        n = int(nth) if nth else 1
        kind, _, key = name.partition(":")
        if kind == "msg":
            hits = [e["t"] for e in self.msgs if e["name"] == key]
        elif kind == "drone":
            hits = [e["t"] for e in self.events if e["type"] == "transition" and e["machine"] == "DRONE" and e["row"] == key]
        elif kind == "vtol":
            hits = [e["t"] for e in self.events if e["type"] == "vtol" and e["state"] == key]
        elif kind == "run_status":
            hits = [e["t"] for e in self.events if e["type"] == "run_status"]
        elif kind == "approach_height":
            # First sample after the landing grant at most this high above pad B's ground.
            zb = scene.get("pad_ground_z_m", {}).get("B", 0.0)
            grant = self.anchor("msg:LANDING_GRANT", scene, cfg)
            z = self.samples["truth_enu_m"][:, 2]
            ok = (self.t > grant) & (z <= zb + cfg["approach_height_m"])
            hits = [float(self.t[np.argmax(ok)])] if ok.any() else []
        else:
            raise ValueError(f"unknown anchor {spec}")
        if len(hits) < n:
            raise ValueError(f"anchor {spec}: {len(hits)} found")
        return hits[n - 1]


class Hud:
    """HUD content at a sim time, from the logs only."""

    def __init__(self, logs: Logs, scene: dict, terrain: tuple, panel: dict, origin_t: float):
        self.logs = logs
        self.origin_t = origin_t
        ev = logs.events
        pad_name = {logs.roles["A"]: "A", logs.roles["B"]: "B"}
        self.changes = []  # (t, key, value)
        for e in ev:
            if e["type"] == "transition" and e["machine"] == "DRONE" and e["from"] != e["to"]:
                self.changes.append((e["t"], "drone", e["to"]))
            elif e["type"] == "transition" and e["machine"] == "VISIT" and e["pad_id"] in pad_name:
                self.changes.append((e["t"], f"pad_{pad_name[e['pad_id']]}", e["to"]))
            elif e["type"] == "vtol":
                self.changes.append((e["t"], "vtol", e["state"]))
        self.change_t = [c[0] for c in self.changes]
        self.msg_t = [m["t"] for m in logs.msgs]
        self.pad_name = pad_name
        self.grid, self.x0, self.y0, self.g = terrain
        self.panel = panel

    def label(self, m: dict) -> str:
        pad = self.pad_name.get(m["pad_id"])
        if m["sender"] == "PAD":
            who = f"PAD {pad}"
        elif m["sender"] == "DRONE":
            who = "DRONE"
        else:
            who = "FLEET MGR"
        extra = {0: " IN", 1: " OUT"}.get(m["spot"], "") if m["name"] in ("LANDING_GRANT", "LANDED") else ""
        return f"{who}: {m['name']}{extra}"

    def at(self, t: float) -> dict:
        state = {"drone": "PARKED", "pad_A": "-", "pad_B": "-", "vtol": "MC"}
        for _, key, value in self.changes[:bisect_right(self.change_t, t)]:
            state[key] = value
        k = bisect_right(self.msg_t, t)
        last = [{"t_s": round(m["t"], 3), "timer_s": round(m["t"] - self.origin_t, 1), "seq": m["seq"],
                 "sender": m["sender"], "name": m["name"], "text": self.label(m)}
                for m in self.logs.msgs[max(0, k - 3):k]]
        i = min(max(0, bisect_right(self.logs.t, t) - 1), len(self.logs.t) - 1)
        x, y, z = (float(v) for v in self.logs.samples["truth_enu_m"][i])
        ground = terrain_z(self.grid, self.x0, self.y0, self.g, x, y)
        return {"sim_t_s": round(t, 4), "timer_s": round(t - self.origin_t, 1), **state, "messages": last,
                "agl_m": round(z - ground, 1),
                "battery_pct": round(float(self.logs.samples["battery_remaining"][i]) * 100, 1),
                "truth_enu_m": [x, y, z]}


def terrain_z(grid, x0, y0, g, x, y) -> float:
    ny, nx = grid.shape
    u = min(max((x - x0) / g, 0), nx - 1 - 1e-9)
    v = min(max((y - y0) / g, 0), ny - 1 - 1e-9)
    i, j = int(u), int(v)
    u, v = u - i, v - j
    z00, z10, z01, z11 = grid[j, i], grid[j, i + 1], grid[j + 1, i], grid[j + 1, i + 1]
    return float(z00 + u * (z10 - z00) + v * (z11 - z10) if u >= v else z00 + v * (z01 - z00) + u * (z11 - z01))


def segments(logs: Logs, scene: dict, cfg: dict) -> list[dict]:
    """Contiguous segments: each runs from its anchor to the next one's (the last to `end`)."""
    rules = cfg["segments"]
    starts = [logs.anchor(r["from"], scene, cfg) + r.get("offset_s", 0.0) for r in rules]
    end = logs.anchor(cfg["end"]["at"], scene, cfg) + cfg["end"].get("offset_s", 0.0)
    out = []
    for k, r in enumerate(rules):
        t0, t1 = starts[k], starts[k + 1] if k + 1 < len(rules) else end
        if t1 <= t0:
            raise ValueError(f"segment {k} ({r['from']}) is empty: {t0:.2f} to {t1:.2f}")
        if "duration_s" in r:
            speed = (t1 - t0) / r["duration_s"]
            n = int(round(r["duration_s"] * FPS))
        else:
            speed = 1.0
            n = int(round((t1 - t0) * FPS))
        out.append({"index": k, "from": r["from"], "camera": r["camera"], "t0_s": t0, "t1_s": t1,
                    "speed": speed, "frames": n, "label": r.get("label", "")})
    return out


class Reader:
    """Sequential decoder of one camera's master: frame k on demand, forward only."""

    def __init__(self, video: Path, stamps: Path):
        self.stamps = [int(l.split(",")[1]) * 1e-6 for l in stamps.read_text().split()[1:]]
        self.proc = subprocess.Popen([ffmpeg_exe(), "-loglevel", "error", "-i", str(video), "-f", "rawvideo",
                                      "-pix_fmt", "rgb24", "-"], stdout=subprocess.PIPE)
        self.k = -1
        self.frame = None

    def at(self, t: float) -> tuple[np.ndarray, int]:
        want = max(0, bisect_right(self.stamps, t) - 1)
        while self.k < want:
            buf = self.proc.stdout.read(W * H * 3)
            if len(buf) < W * H * 3:
                raise RuntimeError("camera master ended before its stamps")
            self.frame = buf
            self.k += 1
        return np.frombuffer(self.frame, np.uint8).reshape(H, W, 3), self.k

    def close(self) -> None:
        # Stopped before its end on purpose: kill first, so it never writes to a closed pipe.
        self.proc.kill()
        self.proc.wait()
        self.proc.stdout.close()


def font(size: int):
    return ImageFont.load_default(size=size)


def clock(seconds: float) -> str:
    """mm:ss.s of a time from the timer origin; times before it show a minus sign."""
    tenths = round(abs(seconds) * 10)
    m, rest = divmod(tenths, 600)
    return f"{'-' if seconds < 0 and tenths else ''}{m:02d}:{rest // 10:02d}.{rest % 10}"


def fmt_min(s: float) -> str:
    return f"{s / 60:.1f} min"


def project(cam: dict, p) -> tuple[float, float] | None:
    """Pixel of world point p in a fixed camera (looking along +X, pitch then yaw)."""
    x, y, z = (p[i] - cam["xyz_m"][i] for i in range(3))
    cy, sy = math.cos(cam["yaw_rad"]), math.sin(cam["yaw_rad"])
    xr, yr = cy * x + sy * y, -sy * x + cy * y
    cp, sp = math.cos(cam["pitch_rad"]), math.sin(cam["pitch_rad"])
    xc, zc = cp * xr - sp * z, sp * xr + cp * z
    if xc <= 1.0:
        return None
    f = (W / 2) / math.tan(cam["hfov_rad"] / 2)
    return W / 2 - f * yr / xc, H / 2 - f * zc / xc


def draw_hud(img: Image.Image, hud: dict, seg: dict, panel: dict, cfg: dict, locator) -> None:
    d = ImageDraw.Draw(img, "RGBA")
    f_big, f, f_small = font(26), font(19), font(15)
    d.rectangle([12, 12, 470, 196], fill=(0, 0, 0, 150))
    d.text((24, 20), f"SIM  T+{clock(hud['timer_s'])}", font=f_big, fill="white")
    rate = "real time" if seg["speed"] == 1.0 else f"x {seg['speed']:.0f}"
    d.text((300, 24), rate, font=f, fill=(255, 210, 80) if seg["speed"] != 1.0 else (180, 220, 180))
    d.text((24, 58), f"Drone   {hud['drone']}  ({hud['vtol']})", font=f, fill="white")
    d.text((24, 84), f"Pad A   {hud['pad_A']}", font=f, fill="white")
    d.text((24, 110), f"Pad B   {hud['pad_B']}", font=f, fill="white")
    d.text((24, 140), f"Height above ground  {hud['agl_m']:6.1f} m", font=f, fill="white")
    d.text((24, 166), f"Battery {hud['battery_pct']:5.1f} % (PX4 SITL, drains with time)", font=f, fill="white")
    # Handshake messages, newest last.
    d.rectangle([W - 472, 12, W - 12, 128], fill=(0, 0, 0, 150))
    d.text((W - 460, 18), "Handshake", font=f_small, fill=(200, 200, 200))
    for i, msg in enumerate(hud["messages"]):
        d.text((W - 460, 40 + 28 * i), f"{clock(msg['timer_s'])}  {msg['text']}", font=f, fill="white")
    # Drone vs truck on this lane, from the demo scenario.
    p = panel
    x0, y0 = W - 472, H - 168
    d.rectangle([x0, y0, W - 12, H - 12], fill=(0, 0, 0, 160))
    d.text((x0 + 12, y0 + 6), f"Lane {p['lane_id']}, demo week, seed {p['seed']}: {p['orders']} drone orders",
           font=f_small, fill=(200, 200, 200))
    d.text((x0 + 12, y0 + 30), f"Drone door-to-door (median)  {fmt_min(p['drone_median_s'])}", font=f, fill="white")
    sp = p["median_order_split_s"]
    d.text((x0 + 12, y0 + 56), f"pod wait {fmt_min(sp['pod_wait_s'])}, queue {fmt_min(sp['queue_s'])}, "
           f"pad {fmt_min(sp['pad_service_s'])}, flight {fmt_min(sp['flight_s'])}", font=f_small, fill="white")
    d.text((x0 + 12, y0 + 80), f"Truck, same orders (median)   {fmt_min(p['truck_median_s'])}", font=f, fill="white")
    d.text((x0 + 12, y0 + 116), "simulation, placeholder inputs", font=f_small, fill=(255, 210, 80))
    if locator is not None:
        u, v = locator
        r = cfg["locator_radius_px"]
        d.ellipse([u - r, v - r, u + r, v + r], outline=(255, 210, 80), width=3)


def card(lines: list[tuple[str, int]], foot: str) -> Image.Image:
    img = Image.new("RGB", (W, H), (18, 22, 28))
    d = ImageDraw.Draw(img)
    y = 220
    for text, size in lines:
        fnt = font(size)
        w = d.textlength(text, font=fnt)
        d.text(((W - w) / 2, y), text, font=fnt, fill="white")
        y += size + 22
    fnt = font(20)
    d.text(((W - d.textlength(foot, font=fnt)) / 2, H - 80), foot, font=fnt, fill=(255, 210, 80))
    return img


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    full = yaml.safe_load(HERO_CFG.read_text())
    cfg = full["edit"]
    run = a.run
    status = (run / "run_status").read_text().strip()
    if status != "COMPLETE":
        raise SystemExit(f"{run}: {status}")
    scene = json.loads((run / "scene.json").read_text())
    lane = json.loads((run / "lane.json").read_text())
    cams = json.loads((run / "cameras.json").read_text())["cameras"]
    panel = json.loads(PANEL.read_text())
    if panel["lane_id"] != lane["lane_id"]:
        raise SystemExit(f"panel is for lane {panel['lane_id']}, the run flew lane {lane['lane_id']}")
    world = Path(json.loads((run / "run.json").read_text())["world_dir"])
    obst = json.loads((world / "obstacles" / "obstacles.json").read_text())["terrain"]
    terrain = (np.load(world / "obstacles" / obst["file"]).astype(np.float64), obst["x0_m"], obst["y0_m"], obst["spacing_m"])
    logs = Logs(run)
    segs = segments(logs, scene, cfg)
    origin = logs.anchor(cfg["timer_origin"], scene, cfg)
    hud = Hud(logs, scene, terrain, panel, origin)
    readers = {name: Reader(run / "video" / f"{name}.mp4", run / "video" / f"{name}.stamps.csv")
               for name in {s["camera"] for s in segs}}
    cam_pose = {c["name"]: {**c, "hfov_rad": full["cameras"]["hfov_rad"]} for c in cams.values()}
    a.out.parent.mkdir(parents=True, exist_ok=True)
    enc = subprocess.Popen([ffmpeg_exe(), "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                            "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264", "-preset", "slow",
                            "-crf", str(cfg["crf"]), "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(a.out)],
                           stdin=subprocess.PIPE)
    timeline = []

    def emit(img: Image.Image, entry: dict) -> None:
        enc.stdin.write(img.tobytes())
        entry["frame"] = len(timeline)
        timeline.append(entry)

    a_name, b_name = lane["from_pad"]["name"].replace("_", " "), lane["to_pad"]["name"].replace("_", " ")
    title = card([(f"Lane {lane['lane_id']}: {a_name} to {b_name}", 34),
                  (f"{lane['length_m'] / 1000:.1f} km over Pittsburgh terrain and buildings", 26),
                  ("Scaled PX4 standard VTOL, 5 kg pod", 26)], "simulated: PX4 SITL and Gazebo, placeholder inputs")
    for _ in range(int(cfg["title_s"] * FPS)):
        emit(title, {"card": "title"})
    for s in segs:
        for k in range(s["frames"]):
            t = s["t0_s"] + k * s["speed"] / FPS
            frame, src = readers[s["camera"]].at(t)
            h = hud.at(t)
            img = Image.fromarray(frame.copy())
            loc = project(cam_pose[s["camera"]], h["truth_enu_m"]) if s["camera"] in cfg["locator_cameras"] else None
            draw_hud(img, h, s, panel, cfg, loc)
            emit(img, {"segment": s["index"], "camera": s["camera"], "source_frame": src,
                       "speed": round(s["speed"], 3), **{k2: v for k2, v in h.items() if k2 != "truth_enu_m"}})
    visit = logs.anchor(cfg["end"]["at"], scene, cfg) - origin
    closing = card([(f"This visit: {fmt_min(visit)} from mission to parked, sim time", 30),
                    (f"Lane {panel['lane_id']} in the demo week: drone door-to-door median {fmt_min(panel['drone_median_s'])}", 26),
                    (f"Truck on the same {panel['orders']} orders: median {fmt_min(panel['truck_median_s'])}", 26)],
                   "simulation, placeholder inputs")
    for _ in range(int(cfg["closing_s"] * FPS)):
        emit(closing, {"card": "closing"})
    enc.stdin.close()
    if enc.wait() != 0:
        raise SystemExit("encoder failed")
    for r in readers.values():
        r.close()
    edit = run / "edit"
    edit.mkdir(exist_ok=True)
    (edit / "hud_timeline.json").write_text(json.dumps({
        "fps": FPS, "panel": panel, "segments": segs, "frames": timeline}, separators=(",", ":")) + "\n")
    print(json.dumps({"frames": len(timeline), "seconds": len(timeline) / FPS,
                      "segments": [(s["camera"], round(s["t1_s"] - s["t0_s"], 1), round(s["speed"], 1)) for s in segs]}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
