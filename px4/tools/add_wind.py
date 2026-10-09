"""Phase 7 step 9: add Gazebo's wind to a run's world (spec A94). A constant wind blowing from
`--from-deg` (clockwise from north) at `--mps`, the same at every height, plus Gazebo's
WindEffects system with Gaussian noise on the magnitude, standard deviation `--gust-frac` of
it, as the gust. Links react only where the vehicle model sets enable_wind (make_vehicle.py
--wind). Usage: add_wind.py <world.sdf> --mps M --from-deg D --gust-frac G"""

import argparse
import math
import sys
from pathlib import Path

# Gazebo's WindEffects force scale (its default); how hard the wind pushes on a link.
FORCE_SCALE = 1.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("world", type=Path)
    ap.add_argument("--mps", type=float, required=True)
    ap.add_argument("--from-deg", type=float, required=True)
    ap.add_argument("--gust-frac", type=float, required=True)
    a = ap.parse_args()
    toward = math.radians(a.from_deg + 180.0)
    vx, vy = a.mps * math.sin(toward), a.mps * math.cos(toward)  # ENU east, north
    block = f"""
    <wind><linear_velocity>{vx:.4f} {vy:.4f} 0</linear_velocity></wind>
    <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <force_approximation_scaling_factor>{FORCE_SCALE}</force_approximation_scaling_factor>
      <horizontal>
        <magnitude>
          <time_for_rise>1</time_for_rise>
          <noise type="gaussian"><mean>0</mean><stddev>{a.gust_frac}</stddev></noise>
        </magnitude>
      </horizontal>
    </plugin>
"""
    text = a.world.read_text()
    if text.count("</world>") != 1:
        raise SystemExit(f"{a.world}: expected one </world>")
    a.world.write_text(text.replace("</world>", block + "  </world>"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
