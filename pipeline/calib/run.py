"""One command for the Phase 5 calibration: `pipeline/run.sh calib [step ...]`; `pipeline/run.sh
calib vtol` runs the Phase 7 VTOL steps (VTOL_STEPS) instead.
Steps run in order: audit, phases, residuals, fit, profile, consequences, heldout, scenarios, plots (default: all of them; a
step whose script does not exist is skipped). Each writes into
pipeline/calib/out/."""

import runpy
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
STEPS = ["audit", "phases", "residuals", "fit", "profile", "consequences", "heldout", "scenarios", "plots"]
VTOL_STEPS = ["vtol_phases", "vtol_fit", "vtol_profile", "vtol_consequences", "vtol_heldout", "vtol_plots"]


def main() -> int:
    sys.path.insert(0, str(HERE))
    steps = sys.argv[1:] or STEPS
    if steps == ["vtol"]:
        steps = VTOL_STEPS
    for step in steps:
        if step not in STEPS + VTOL_STEPS:
            print(f"unknown step {step}; choose from {STEPS + VTOL_STEPS}")
            return 2
        if (HERE / f"{step}.py").exists():
            print(f"== {step}", flush=True)
            runpy.run_path(str(HERE / f"{step}.py"), run_name="__main__")
    return 0


if __name__ == "__main__":
    sys.exit(main())
