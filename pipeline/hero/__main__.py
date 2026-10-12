"""`pipeline/run.sh hero <step>`: lane (step 2), world (step 3), panel (step 6), terrain-check
(step 7)."""

import argparse
import sys

from . import panel, select_lane, terrain_check, world

STEPS = {"lane": select_lane.main, "world": world.main, "panel": panel.main,
         "terrain-check": terrain_check.main}


def main() -> int:
    ap = argparse.ArgumentParser(prog="hero")
    ap.add_argument("step", choices=sorted(STEPS))
    a = ap.parse_args()
    STEPS[a.step]()
    return 0


if __name__ == "__main__":
    sys.exit(main())
