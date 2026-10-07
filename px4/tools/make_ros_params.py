"""Per-run configuration of the ROS side: <run>/ros_params.yaml (one section per node) and
<run>/bridge.yaml (ros_gz_bridge topics), from config/lane.json, config/scene.json,
config/flight.yaml and the run's vehicle.json. The seed sets the drone's start offset from A's
OUT spot (written back into vehicle.json's spawn poses, pod 1 moves with the drone) and the
detector's corner noise. Nothing here seeds Gazebo or PX4.

Usage: make_ros_params.py --run <dir> --seed N"""

import argparse
import json
import math
import random
import sys
from pathlib import Path

import yaml

PX4 = Path(__file__).resolve().parents[1]
MAVLINK_HOST = "127.0.0.1"
# PX4 SITL's onboard MAVLink instance (udp port 14580, remote 14540).
MAVLINK_PORT = 14580
DRONE_ID = 0
# Order of the pod the drone carries from A; the pickup at B is the next order.
ORDER_ID = 0
# A pod counts as resting on a slab when its centre is at most this much above its resting
# height (carried in the latch it hangs carry_clearance_m higher).
POD_REST_MARGIN_M = 0.01
SPOT_CODE = {"IN": 0, "OUT": 1}
# Energy proxy constants: PX4 x500 motor constant (scaled per vehicle, make_vehicle.py), the
# x500 propeller radius (half its 0.2792 m rotor collision box) and ISA sea-level air density.
X500_MOTOR_CONSTANT = 8.54858e-06
X500_ROTOR_RADIUS_M = 0.1396
AIR_DENSITY_KGPM3 = 1.225


# WGS84 (the ellipsoid of Gazebo's spherical_coordinates EARTH_WGS84).
WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


def enu_to_latlon(e: float, n: float, lat0: float, lon0: float, h0: float = 0.0) -> list[float]:
    """World ENU metres to latitude/longitude the way Gazebo's NavSat reports them: ENU on the
    tangent plane at the world origin, through ECEF, back to WGS84 geodetic. The drone gets pads
    as latitude/longitude, like a real vehicle given surveyed pad positions (spec A61)."""
    p0, l0 = math.radians(lat0), math.radians(lon0)
    def ecef(p, l, h):
        nn = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(p) ** 2)
        return ((nn + h) * math.cos(p) * math.cos(l), (nn + h) * math.cos(p) * math.sin(l),
                (nn * (1 - WGS84_E2) + h) * math.sin(p))
    x0, y0, z0 = ecef(p0, l0, h0)
    x = x0 - math.sin(l0) * e - math.sin(p0) * math.cos(l0) * n
    y = y0 + math.cos(l0) * e - math.sin(p0) * math.sin(l0) * n
    z = z0 + math.cos(p0) * n
    lon = math.atan2(y, x)
    r = math.hypot(x, y)
    lat = math.atan2(z, r * (1 - WGS84_E2))
    for _ in range(10):
        nn = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
        h = r / math.cos(lat) - nn
        lat = math.atan2(z, r * (1 - WGS84_E2 * nn / (nn + h)))
    return [math.degrees(lat), math.degrees(lon)]


class NoAliasDumper(yaml.SafeDumper):
    """rcl's parameter-file parser rejects YAML aliases, which PyYAML emits for shared lists."""

    def ignore_aliases(self, data):
        return True


def bridge(world: str) -> list[dict]:
    def entry(ros, gz, ros_type, gz_type, direction):
        return {"ros_topic_name": ros, "gz_topic_name": gz, "ros_type_name": ros_type,
                "gz_type_name": gz_type, "direction": direction}

    out = [
        entry("/clock", f"/world/{world}/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS"),
        entry("/dd/down_cam", "/dd/down_cam", "sensor_msgs/msg/Image", "gz.msgs.Image", "GZ_TO_ROS"),
        entry("/dd/motor_speed", "/dd_quad/command/motor_speed", "actuator_msgs/msg/Actuators",
              "gz.msgs.Actuators", "GZ_TO_ROS"),
    ]
    for model in ("dd_quad", "dd_pod_1", "dd_pod_2"):
        out.append(entry(f"/dd/truth/{model}", f"/model/{model}/pose", "geometry_msgs/msg/Pose",
                         "gz.msgs.Pose", "GZ_TO_ROS"))
    for pod in ("dd_pod_1", "dd_pod_2"):
        for action in ("attach", "detach"):
            out.append(entry(f"/dd/{pod}/{action}", f"/dd/{pod}/{action}", "std_msgs/msg/Empty",
                             "gz.msgs.Empty", "ROS_TO_GZ"))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, required=True)
    ap.add_argument("--seed", type=int, required=True)
    args = ap.parse_args()
    lane = json.loads((PX4 / "config" / "lane.json").read_text())
    scene = json.loads((PX4 / "config" / "scene.json").read_text())
    cfg = yaml.safe_load((PX4 / "config" / "flight.yaml").read_text())
    vehicle_path = args.run / "vehicle.json"
    vehicle = json.loads(vehicle_path.read_text())

    rng = random.Random(args.seed)
    sigma = cfg["seeded"]["start_offset_sigma_m"]
    offset = [rng.gauss(0.0, sigma), rng.gauss(0.0, sigma)]
    for model in ("dd_quad", "dd_pod_1"):
        pose = vehicle["spawn"][model]
        pose[0] += offset[0]
        pose[1] += offset[1]
    vehicle["seed"] = args.seed
    vehicle["start_offset_enu_m"] = offset
    vehicle_path.write_text(json.dumps(vehicle, indent=1) + "\n")

    f, d, cam = cfg["flight"], cfg["detector"], cfg["vehicle"]["camera"]
    spots = scene["spots"]
    lat0, lon0 = lane["from_pad"]["lat_deg"], lane["from_pad"]["lon_deg"]
    def latlon(enu):
        return enu_to_latlon(enu[0], enu[1], lat0, lon0, cfg["world"]["elevation_m"])
    common = {"use_sim_time": True}
    legs = {"accel_mps2": f["accel_mps2"], "arrive_radius_m": f["arrive_radius_m"],
            "settle_speed_mps": f["settle_speed_mps"], "engage_timeout_s": f["engage_timeout_s"]}
    params = {
        "marker_detector": {"ros__parameters": {
            **common,
            "marker_side_m": scene["marker_side_m"],
            "marker_offset_enu_m": scene["marker_offset_m"],
            "camera_mount_flu_m": cam["mount_m"],
            "camera_hfov_rad": cam["horizontal_fov_rad"],
            "corner_noise_px": d["corner_noise_px"],
            "window": d["window"],
            "min_detections": d["min_detections"],
            "seed": args.seed,
            "mavlink_host": MAVLINK_HOST,
            "mavlink_port": MAVLINK_PORT,
        }},
        "precland_check": {"ros__parameters": {
            **common, **legs,
            "spot_enu_m": spots["A_IN"]["enu_m"],
            "spot_latlon_deg": latlon(spots["A_IN"]["enu_m"]),
            "marker_id": spots["A_IN"]["marker_id"],
            "approach_alt_m": f["approach_alt_m"],
            "min_detections": d["min_detections"],
        }},
    }
    pads = [("A", lane["from_pad"]["id"]), ("B", lane["to_pad"]["id"])]
    pad_center = [c for name, _ in pads
                  for c in ((spots[f"{name}_IN"]["enu_m"][0] + spots[f"{name}_OUT"]["enu_m"][0]) / 2,
                            (spots[f"{name}_IN"]["enu_m"][1] + spots[f"{name}_OUT"]["enu_m"][1]) / 2)]
    pod_rest_max_z = scene["slab_top_m"] + cfg["vehicle"]["pod"]["size_m"][2] / 2 + POD_REST_MARGIN_M
    params["drone_agent"] = {"ros__parameters": {
        **common, **legs,
        "drone_id": DRONE_ID,
        "pad_ids": [pid for _, pid in pads],
        "pad_center_latlon_deg": [c for i in range(len(pads)) for c in latlon(pad_center[2 * i:2 * i + 2])],
        "pad_in_latlon_deg": [c for name, _ in pads for c in latlon(spots[f"{name}_IN"]["enu_m"])],
        "pad_out_latlon_deg": [c for name, _ in pads for c in latlon(spots[f"{name}_OUT"]["enu_m"])],
        "pad_in_marker": [spots[f"{name}_IN"]["marker_id"] for name, _ in pads],
        "pad_out_marker": [spots[f"{name}_OUT"]["marker_id"] for name, _ in pads],
        "start_pad": pads[0][1],
        "start_pod": 1,
        "start_order": ORDER_ID,
        "cruise_speed_mps": lane["drone"]["cruise_speed_mps"],
        "cruise_alt_m": lane["cruise_alt_m"],
        "approach_alt_m": f["approach_alt_m"],
        "hop_alt_m": f["hop_alt_m"],
        "hop_speed_mps": f["hop_speed_mps"],
        "release_s": lane["pad_ops"]["release_s"],
        "latch_s": lane["pad_ops"]["latch_s"],
        "latch_capture_radius_m": cfg["pads"]["latch_capture_radius_m"],
        "min_detections": d["min_detections"],
        "target_timeout_s": f["target_timeout_s"],
    }}
    for name, pid in pads:
        params[f"pad_{name.lower()}"] = {"ros__parameters": {
            **common,
            "pad_id": pid,
            "in_enu_m": spots[f"{name}_IN"]["enu_m"],
            "out_enu_m": spots[f"{name}_OUT"]["enu_m"],
            "spot_sensor_radius_m": cfg["pads"]["slab_side_m"] / 2,
            "in_pod_sensor_radius_m": cfg["pads"]["in_pod_sensor_radius_m"],
            "pod_rest_max_z_m": pod_rest_max_z,
            "latch_capture_radius_m": cfg["pads"]["latch_capture_radius_m"],
            "release_verify_s": lane["pad_ops"]["release_verify_s"],
            "latch_verify_s": lane["pad_ops"]["latch_verify_s"],
            # Pod 2 waits on B's OUT; the drone starts parked on A's OUT.
            "out_pod": 2 if name == "B" else -1,
            "parked_drone_on_out": DRONE_ID if name == "A" else -1,
        }}
    params["fleet_manager"] = {"ros__parameters": {
        **common,
        "drone_id": DRONE_ID,
        "dest_pad": pads[1][1],
        "order_id": ORDER_ID,
        "pickup_pod": 2,
        "pickup_order_id": ORDER_ID + 1,
    }}
    spot_names = [f"{name}_{kind}" for name, _ in pads for kind in ("IN", "OUT")]
    params["flight_logger"] = {"ros__parameters": {
        **common,
        "run_dir": str(args.run.resolve()),
        "lane_id": lane["lane_id"],
        "order_id": ORDER_ID,
        "payload_kg": vehicle["payload_kg"],
        "drone_id": DRONE_ID,
        "from_pad": pads[0][1],
        "to_pad": pads[1][1],
        "motor_constant": X500_MOTOR_CONSTANT * vehicle["thrust_scale"],
        "rotor_radius_m": X500_ROTOR_RADIUS_M,
        "air_density_kgpm3": AIR_DENSITY_KGPM3,
        "sample_period_s": cfg["logging"]["sample_period_s"],
        "spot_pad_ids": [pid for name, pid in pads for _ in ("IN", "OUT")],
        "spot_codes": [SPOT_CODE[n.split("_")[1]] for n in spot_names],
        "spot_enu_m": [c for n in spot_names for c in spots[n]["enu_m"]],
    }}
    (args.run / "ros_params.yaml").write_text(yaml.dump(params, Dumper=NoAliasDumper, sort_keys=False))
    (args.run / "bridge.yaml").write_text(yaml.dump(bridge(scene["world"]), Dumper=NoAliasDumper,
                                                    sort_keys=False))
    print(f"seed {args.seed}: start offset {offset[0]:+.3f}, {offset[1]:+.3f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
