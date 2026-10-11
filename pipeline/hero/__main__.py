"""`pipeline/run.sh hero <step>`: lane (step 2)."""

import argparse
import sys

from . import select_lane

STEPS = {"lane": select_lane.main}


def main() -> int:
    ap = argparse.ArgumentParser(prog="hero")
    ap.add_argument("step", choices=sorted(STEPS))
    a = ap.parse_args()
    STEPS[a.step]()
    return 0


if __name__ == "__main__":
    sys.exit(main())
