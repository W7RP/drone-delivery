"""One command for the Phase 5 calibration: `pipeline/run.sh calib [step ...]`.
Steps run in order: audit, phases, residuals, fit, profile, consequences, heldout, scenarios, plots (default: all of them; a
step whose script does not exist is skipped). Each writes into
pipeline/calib/out/."""

import runpy
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STEPS = ["audit", "phases", "residuals", "fit", "profile", "consequences", "heldout", "scenarios", "plots"]


def main() -> int:
    sys.path.insert(0, str(HERE))
    steps = sys.argv[1:] or STEPS
    for step in steps:
        if step not in STEPS:
            print(f"unknown step {step}; choose from {STEPS}")
            return 2
        if (HERE / f"{step}.py").exists():
            print(f"== {step}", flush=True)
            runpy.run_path(str(HERE / f"{step}.py"), run_name="__main__")
    return 0


if __name__ == "__main__":
    sys.exit(main())
