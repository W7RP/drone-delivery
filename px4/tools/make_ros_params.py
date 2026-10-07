"""Per-run configuration of the ROS side: <run>/ros_params.yaml (one section per node) and
<run>/bridge.yaml (ros_gz_bridge topics), from config/lane.json, config/scene.json,
config/flight.yaml and the run's vehicle.json. The seed sets the drone's start offset from A's
OUT spot (written back into vehicle.json's spawn poses, pod 1 moves with the drone) and the
detector's corner noise. Nothing here seeds Gazebo or PX4.

Usage: make_ros_params.py --run <dir> --seed N"""

import argparse
import json
import random
import sys
from pathlib import Path

import yaml

PX4 = Path(__file__).resolve().parents[1]
MAVLINK_HOST = "127.0.0.1"
# PX4 SITL's onboard MAVLink instance (udp port 14580, remote 14540).
MAVLINK_PORT = 14580
SPOT_CODE = {"IN": 0, "OUT": 1}


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
    start_enu = vehicle["spawn"]["dd_quad"][:2]

    f, d, cam = cfg["flight"], cfg["detector"], cfg["vehicle"]["camera"]
    spots = scene["spots"]
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
            "start_enu_m": start_enu,
            "spot_enu_m": spots["A_IN"]["enu_m"],
            "marker_id": spots["A_IN"]["marker_id"],
            "approach_alt_m": f["approach_alt_m"],
            "min_detections": d["min_detections"],
        }},
    }
    (args.run / "ros_params.yaml").write_text(yaml.safe_dump(params, sort_keys=False))
    (args.run / "bridge.yaml").write_text(yaml.safe_dump(bridge(scene["world"]), sort_keys=False))
    print(f"seed {args.seed}: start offset {offset[0]:+.3f}, {offset[1]:+.3f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
