"""Phase 7 step 9: add Gazebo's wind to a run (spec A94). A constant wind blowing from
`--from-deg` (clockwise from north) at `--mps`, the same at every height, goes into the world;
Gazebo's WindEffects system, with Gaussian noise of standard deviation `--gust-frac` times
`--mps` (m/s) on the magnitude as the gust, goes into a copy of the server config.

The system cannot go in the world: a world that names any system gets only those, not the
server config's defaults (physics, scene broadcaster, sensors), and never comes up. Links react
only where the vehicle model sets enable_wind (make_vehicle.py --wind).

Usage: add_wind.py <world.sdf> <server.config in> <server.config out> --mps M --from-deg D
--gust-frac G"""

import argparse
import math
import sys
from pathlib import Path

# Gazebo's WindEffects force scale (its default); how hard the wind pushes on a link.
FORCE_SCALE = 1.0
# WindEffects' first-order rise time for magnitude changes, s.
RISE_TIME_S = 1.0


def replace_once(text: str, marker: str, new: str, where: Path) -> str:
    if text.count(marker) != 1:
        raise SystemExit(f"{where}: expected one {marker}")
    return text.replace(marker, new)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("world", type=Path)
    ap.add_argument("config_in", type=Path)
    ap.add_argument("config_out", type=Path)
    ap.add_argument("--mps", type=float, required=True)
    ap.add_argument("--from-deg", type=float, required=True)
    ap.add_argument("--gust-frac", type=float, required=True)
    a = ap.parse_args()
    toward = math.radians(a.from_deg + 180.0)
    vx, vy = a.mps * math.sin(toward), a.mps * math.cos(toward)  # ENU east, north
    wind = f"    <wind><linear_velocity>{vx:.4f} {vy:.4f} 0</linear_velocity></wind>\n  </world>"
    a.world.write_text(replace_once(a.world.read_text(), "</world>", wind, a.world))
    system = f"""    <plugin entity_name="*" entity_type="world" filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <force_approximation_scaling_factor>{FORCE_SCALE}</force_approximation_scaling_factor>
      <horizontal>
        <magnitude>
          <time_for_rise>{RISE_TIME_S}</time_for_rise>
          <noise type="gaussian"><mean>0</mean><stddev>{a.gust_frac * a.mps:.4f}</stddev></noise>
        </magnitude>
      </horizontal>
    </plugin>
  </plugins>"""
    a.config_out.write_text(replace_once(a.config_in.read_text(), "</plugins>", system, a.config_in))
    return 0


if __name__ == "__main__":
    sys.exit(main())
