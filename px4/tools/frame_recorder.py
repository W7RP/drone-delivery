"""Phase 8: stream Gazebo camera topics into one H.264 file per camera, without storing frames.

Each frame received is written to its camera's ffmpeg stdin as raw RGB, and ffmpeg encodes it at
once, so nothing uncompressed reaches the disk. Frames are timed by their sim-time stamp, not by
arrival, and each camera's stamps go to <out>/<camera>.stamps.csv (frame index, sim time in us).
The video's nominal rate is the cameras' 30 Hz; tools/hero_edit.py maps frames to sim time by
the stamps.

Runs under the system Python 3.10, which has Gazebo Harmonic's gz-transport bindings; the ffmpeg
binary comes from the px4 venv's imageio-ffmpeg wheel (`--ffmpeg`).

Usage: frame_recorder.py --out DIR --ffmpeg PATH --camera NAME=TOPIC [...] [--seconds S]
Stops on SIGINT/SIGTERM, or after S wall seconds. Prints a JSON summary per camera."""

import argparse
import json
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

FPS = 30
# x264 settings for the per-camera masters: near-lossless enough to re-encode in the edit.
X264_CRF = 16
X264_PRESET = "veryfast"


class Camera:
    def __init__(self, name: str, out: Path, ffmpeg: str):
        self.name = name
        self.out = out
        self.ffmpeg = ffmpeg
        self.proc: subprocess.Popen | None = None
        self.size: tuple[int, int] | None = None
        self.stamps: list[int] = []
        self.arrivals: list[float] = []
        self.lock = threading.Lock()
        self.errors = 0
        self.closed = False

    def start(self, width: int, height: int) -> None:
        self.size = (width, height)
        self.proc = subprocess.Popen(
            [self.ffmpeg, "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
             "-s", f"{width}x{height}", "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264",
             "-preset", X264_PRESET, "-crf", str(X264_CRF), "-pix_fmt", "yuv420p",
             str(self.out / f"{self.name}.mp4")],
            stdin=subprocess.PIPE)

    def on_image(self, msg: Image) -> None:
        with self.lock:
            if self.closed:
                return
            if self.proc is None:
                self.start(msg.width, msg.height)
            if (msg.width, msg.height) != self.size:
                self.errors += 1
                return
            stamp_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nsec // 1000
            # Gazebo can publish a stamp twice when a subscriber joins; keep the first.
            if self.stamps and stamp_us <= self.stamps[-1]:
                self.errors += 1
                return
            self.proc.stdin.write(msg.data)
            self.stamps.append(stamp_us)
            self.arrivals.append(time.monotonic())

    def close(self) -> dict:
        with self.lock:
            self.closed = True
            if self.proc is not None:
                self.proc.stdin.close()
                self.proc.wait()
            (self.out / f"{self.name}.stamps.csv").write_text(
                "frame,sim_us\n" + "".join(f"{i},{s}\n" for i, s in enumerate(self.stamps)))
            n = len(self.stamps)
            summary = {"camera": self.name, "frames": n, "size": self.size, "rejected": self.errors,
                       "ffmpeg_rc": None if self.proc is None else self.proc.returncode}
            if n >= 2:
                span_s = (self.stamps[-1] - self.stamps[0]) * 1e-6
                gaps = [b - a for a, b in zip(self.stamps, self.stamps[1:])]
                period_us = sorted(gaps)[len(gaps) // 2]
                summary.update({
                    "sim_span_s": round(span_s, 3),
                    "sim_rate_hz": round((n - 1) / span_s, 3),
                    "max_gap_ms": round(max(gaps) / 1000, 1),
                    "missing_frames": sum(max(0, round(g / period_us) - 1) for g in gaps),
                    "wall_fps": round((n - 1) / (self.arrivals[-1] - self.arrivals[0]), 2),
                    "real_time_factor": round(span_s / (self.arrivals[-1] - self.arrivals[0]), 3)})
            return summary


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--ffmpeg", required=True)
    ap.add_argument("--camera", action="append", required=True, help="NAME=TOPIC")
    ap.add_argument("--seconds", type=float)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    node = Node()
    cams = []
    for spec in a.camera:
        name, topic = spec.split("=", 1)
        cam = Camera(name, a.out, a.ffmpeg)
        if not node.subscribe(Image, topic, cam.on_image):
            print(f"cannot subscribe to {topic}", file=sys.stderr)
            return 1
        cams.append(cam)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    stop.wait(a.seconds)
    summaries = [c.close() for c in cams]
    (a.out / "recorder.json").write_text(json.dumps(summaries, indent=1) + "\n")
    print(json.dumps(summaries))
    return 0 if all(s["ffmpeg_rc"] == 0 for s in summaries) else 1


if __name__ == "__main__":
    sys.exit(main())
