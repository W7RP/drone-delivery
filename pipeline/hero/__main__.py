"""`pipeline/run.sh hero <step>`: lane (step 2), world (step 3), panel (step 6), terrain-check
(step 7), and `map <run dir>` (the README's track map of a flight)."""

import argparse
import sys
from pathlib import Path

from . import panel, select_lane, terrain_check, track_map, world

STEPS = {"lane": select_lane.main, "world": world.main, "panel": panel.main,
         "terrain-check": terrain_check.main}


def main() -> int:
    ap = argparse.ArgumentParser(prog="hero")
    ap.add_argument("step", choices=sorted([*STEPS, "map"]))
    ap.add_argument("run", nargs="?", type=Path, help="map: the flight's run directory")
    a = ap.parse_args()
    if a.step == "map":
        if a.run is None:
            ap.error("map needs a run directory")
        track_map.main(a.run.resolve())
    else:
        STEPS[a.step]()
    return 0


if __name__ == "__main__":
    sys.exit(main())
