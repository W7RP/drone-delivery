"""Phase 4 step 1 check: subscribe to the smoke camera for a few seconds and report frame rate,
size and whether the scene rendered (the black and white tiles must both show). Uses the
gz-transport Python bindings that ship with Gazebo Harmonic, so it needs no extra packages."""

import sys
import time

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

TOPIC = "/smoke/down_cam"
LISTEN_S = 5.0
MIN_RATE_HZ = 8.0
# A rendered frame has the black tile (dark pixels) and the white tile (bright pixels).
DARK, BRIGHT = 40, 215

frames = []


def on_image(msg: Image) -> None:
    frames.append((time.monotonic(), msg.width, msg.height, msg.data))


def main() -> int:
    node = Node()
    if not node.subscribe(Image, TOPIC, on_image):
        print(f"cannot subscribe to {TOPIC}")
        return 1
    time.sleep(LISTEN_S)
    if len(frames) < 2:
        print(f"FAIL: {len(frames)} frames in {LISTEN_S} s")
        return 1
    rate = (len(frames) - 1) / (frames[-1][0] - frames[0][0])
    _, w, h, data = frames[-1]
    dark = sum(1 for b in data[::3] if b < DARK)
    bright = sum(1 for b in data[::3] if b > BRIGHT)
    print(f"{len(frames)} frames, {rate:.1f} Hz wall, {w}x{h}, dark px {dark}, bright px {bright}")
    ok = rate >= MIN_RATE_HZ and dark > 0 and bright > 0 and (w, h) == (320, 240)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
