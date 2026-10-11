"""Write the per-run vehicle model: PX4's x500 airframe scaled to the scenario's drone (spec 2.7
masses) with a downward camera, plus the two pods (step 4). Output: <out>/models/dd_quad/ and
<out>/models/dd_pod_<n>/, which tools/flight.sh puts first on GZ_SIM_RESOURCE_PATH.

Scaling (spec A55): the body is PX4's x500_base (read from the PX4 tree, not changed there)
with its base_link mass raised so the airframe weighs drone.empty_mass_kg, plus a latch link of
drone.latch_mass_kg. The motor constant is raised so the four motors give
thrust_to_weight_at_mtow times the weight at drone.mtow_kg, and the body inertia rises by the
same factor. Control authority per unit command is then the x500's, so PX4's stock x500
airframe (4001) and its gains fly it unchanged.

Collision masks (A55): the drone's collisions use bit 0 and the pods' bit 1, so drone and pods
never touch (a latch funnel guides the pod), while ground and slabs (default mask) collide with
both. The x500's collisions are tagged here, which is why the body is copied rather than
included."""

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

PX4 = Path(__file__).resolve().parents[1]
GRAVITY = 9.80665
# PX4 Tools/simulation/gz/models/x500 and x500_base (v1.17.0).
X500_BODY_KG = 2.0
X500_ROTOR_KG = 0.016076923076923075
X500_I = (0.02166666666666667, 0.02166666666666667, 0.04000000000000001)
X500_MOTOR_CONSTANT = 8.54858e-06
X500_MAX_ROT_VELOCITY = 1000.0
X500_ROTOR_DRAG = 8.06428e-05
X500_ROLLING_MOMENT = 1e-06
PX4_MODELS = Path(os.environ.get("PX4_DIR", Path.home() / "PX4-Autopilot")) / \
    "Tools/simulation/gz/models"
X500_BASE_SDF = PX4_MODELS / "x500_base/model.sdf"
# PX4 Tools/simulation/gz/models/standard_vtol (v1.17.0), airframe 4004 (Phase 7, spec A91).
VTOL_SDF = PX4_MODELS / "standard_vtol/model.sdf"
VTOL_STOCK_KG = 5.0 + 5 * 0.005
VTOL_ROTOR_KG = 0.005
VTOL_ROTORS = 5
# Skids under the motor booms give the pod clearance under the wing (the stock model rests on
# its wing box): bottom this far below base_link, which then stands this high.
VTOL_SKID_BOTTOM_Z = -0.30
VTOL_SKID_TOP_Z = -0.08
VTOL_BOOM_Y = 0.35
VTOL_SKID_SIZE_XY = (0.74, 0.03)
# Camera below the wing and forward of the carried pod.
VTOL_CAMERA_MOUNT = (0.25, 0.0, -0.15)
VTOL_LATCH_Z = -0.11
# Rotor radii of the stock model's collision discs (hover rotors, pusher); the energy proxy
# scales their disc areas by the same factor as thrust, keeping the stock disc loading (A91).
VTOL_HOVER_RADIUS_M = 0.1
VTOL_PUSHER_RADIUS_M = 0.06
VTOL_HOVER_MOTOR_CONSTANT = 2e-05
VTOL_PUSHER_MOTOR_CONSTANT = 8.54858e-06
# The quad's proxy (Phase 5): x500 rotors, motor constant scaled, disc not (A57).
X500_ROTOR_RADIUS_M = 0.1396
# base_link height above the model origin when standing (x500_base model pose), and the skid
# bottom below base_link.
BASE_HEIGHT_M = 0.24
SKID_BOTTOM_Z = -0.2270
CAMERA_KG = 0.01
LATCH_Z = -0.08
MOTORS = (("rotor_0_joint", "rotor_0", "ccw"), ("rotor_1_joint", "rotor_1", "ccw"),
          ("rotor_2_joint", "rotor_2", "cw"), ("rotor_3_joint", "rotor_3", "cw"))
# Collision bitmasks (decimal; SDF reads them as unsigned integers).
DRONE_MASK = 1
POD_MASK = 2


def thrust_scale(lane: dict, cfg: dict) -> float:
    max_thrust_n = cfg["vehicle"]["thrust_to_weight_at_mtow"] * lane["drone"]["mtow_kg"] * GRAVITY
    return max_thrust_n / (4 * X500_MOTOR_CONSTANT * X500_MAX_ROT_VELOCITY**2)


def motor_plugin(n: int, joint: str, link: str, turning: str, s: float) -> str:
    return f"""
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>{joint}</jointName>
      <linkName>{link}</linkName>
      <turningDirection>{turning}</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>{X500_MAX_ROT_VELOCITY}</maxRotVelocity>
      <motorConstant>{X500_MOTOR_CONSTANT * s:.6e}</motorConstant>
      <momentConstant>0.016</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>{n}</motorNumber>
      <rotorDragCoefficient>{X500_ROTOR_DRAG * s:.6e}</rotorDragCoefficient>
      <rollingMomentCoefficient>{X500_ROLLING_MOMENT * s:.6e}</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>"""


def x500_body(empty_kg: float, s: float) -> str:
    """PX4's x500_base links, joints and sensors as SDF text, with base_link carrying the whole
    airframe mass and the scaled inertia, every collision tagged with DRONE_MASK, and the model
    pose folded into base_link so the model origin is on the ground."""
    root = ET.parse(X500_BASE_SDF).getroot()
    model = root.find("model")
    pose = model.find("pose")
    model.remove(pose)
    for link in model.findall("link"):
        lp = link.find("pose")
        if lp is None:
            lp = ET.SubElement(link, "pose")
            lp.text = "0 0 0 0 0 0"
        x, y, z, *rest = (float(v) for v in lp.text.split())
        lp.text = " ".join(str(v) for v in (x, y, z + BASE_HEIGHT_M, *rest))
        for col in link.iter("collision"):
            surface = col.find("surface")
            if surface is None:
                surface = ET.SubElement(col, "surface")
            contact = surface.find("contact")
            if contact is None:
                contact = ET.SubElement(surface, "contact")
            ET.SubElement(contact, "collide_bitmask").text = str(DRONE_MASK)
    base = next(l for l in model.findall("link") if l.get("name") == "base_link")
    inertial = base.find("inertial")
    inertial.find("mass").text = f"{empty_kg - 4 * X500_ROTOR_KG - CAMERA_KG:.4f}"
    inertia = inertial.find("inertia")
    for tag, value in zip(("ixx", "iyy", "izz"), X500_I):
        inertia.find(tag).text = f"{value * s:.6f}"
    return "".join(ET.tostring(child, encoding="unicode") for child in model
                   if child.tag in ("link", "joint"))


def vtol_scale(lane: dict) -> float:
    """One factor on mass, inertia, every motor and drag constant and the wing area, so that at
    drone.mtow_kg the vehicle is PX4's standard VTOL at 5.025 kg scaled up: same
    accelerations, stall speed and thrust-to-weight, and 4004's gains fly it (spec A91)."""
    return lane["drone"]["mtow_kg"] / VTOL_STOCK_KG


def vtol_body(empty_kg: float, s: float) -> str:
    """standard_vtol's links, joints, sensors and plugins, scaled by s; base_link carries the
    airframe mass; skids added; every collision tagged DRONE_MASK; links raised so the model
    origin is on the ground."""
    root = ET.parse(VTOL_SDF).getroot()
    model = root.find("model")
    model.remove(model.find("pose"))
    stand = -VTOL_SKID_BOTTOM_Z
    for link in model.findall("link"):
        lp = link.find("pose")
        if lp is None:
            lp = ET.SubElement(link, "pose")
            lp.text = "0 0 0 0 0 0"
        x, y, z, *rest = (float(v) for v in lp.text.split())
        lp.text = " ".join(str(v) for v in (x, y, z + stand, *rest))
    base = next(l for l in model.findall("link") if l.get("name") == "base_link")
    inertial = base.find("inertial")
    inertial.find("mass").text = f"{empty_kg - VTOL_ROTORS * VTOL_ROTOR_KG - CAMERA_KG:.4f}"
    for tag in ("ixx", "iyy", "izz"):
        el = inertial.find("inertia").find(tag)
        el.text = f"{float(el.text) * s:.6f}"
    sx, sy = VTOL_SKID_SIZE_XY
    height = VTOL_SKID_TOP_Z - VTOL_SKID_BOTTOM_Z
    for side in (1, -1):
        col = ET.SubElement(base, "collision", name=f"skid_{'left' if side > 0 else 'right'}")
        ET.SubElement(col, "pose").text = \
            f"0 {side * VTOL_BOOM_Y} {VTOL_SKID_BOTTOM_Z + height / 2} 0 0 0"
        ET.SubElement(ET.SubElement(ET.SubElement(col, "geometry"), "box"), "size").text = \
            f"{sx} {sy} {height}"
    for col in model.iter("collision"):
        surface = col.find("surface")
        if surface is None:
            surface = ET.SubElement(col, "surface")
        contact = surface.find("contact")
        if contact is None:
            contact = ET.SubElement(surface, "contact")
        ET.SubElement(contact, "collide_bitmask").text = str(DRONE_MASK)
    for plugin in model.findall("plugin"):
        for tag in ("motorConstant", "rotorDragCoefficient", "rollingMomentCoefficient", "area"):
            el = plugin.find(tag)
            if el is not None:
                el.text = f"{float(el.text) * s:.6e}"
    return "".join(ET.tostring(child, encoding="unicode") for child in model
                   if child.tag in ("link", "joint", "plugin"))


def vtol_sdf(lane: dict, cfg: dict, latch_kg: float, pods: list[str]) -> str:
    s = vtol_scale(lane)
    empty = lane["drone"]["empty_mass_kg"]
    cam = cfg["vehicle"]["camera"]
    cx, cy, cz = VTOL_CAMERA_MOUNT
    return f"""<?xml version="1.0"?>
<!-- Generated by px4/tools/make_vehicle.py from PX4's standard_vtol; do not edit. Airframe
     {empty} kg, forces, inertia and wing area x{s:.3f} the stock model's, latch {latch_kg} kg. -->
<sdf version="1.10">
  <model name="dd_vtol">
    <self_collide>false</self_collide>
    {vtol_body(empty, s)}
    {latch_camera_sdf(latch_kg, cam, (cx, cy, cz), VTOL_LATCH_Z)}
    {pose_publisher_and_joints(pods)}
  </model>
</sdf>
"""


def latch_camera_sdf(latch_kg: float, cam: dict, mount, latch_z: float) -> str:
    cx, cy, cz = mount
    return f"""<link name="latch_link">
      <pose relative_to="base_link">0 0 {latch_z} 0 0 0</pose>
      <inertial>
        <mass>{max(latch_kg, 1e-3)}</mass>
        <inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="latch"><geometry><box><size>0.08 0.08 0.03</size></box></geometry>
        <material><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
    </link>
    <joint name="latch_joint" type="fixed"><parent>base_link</parent><child>latch_link</child></joint>
    <link name="camera_link">
      <!-- Pitched +90 deg: the camera's +x looks down, image up = body forward. -->
      <pose relative_to="base_link">{cx} {cy} {cz} 0 1.5707963 0</pose>
      <inertial>
        <mass>{CAMERA_KG}</mass>
        <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <sensor name="down_cam" type="camera">
        <topic>dd/down_cam</topic>
        <update_rate>{cam['rate_hz']}</update_rate>
        <always_on>1</always_on>
        <camera>
          <horizontal_fov>{cam['horizontal_fov_rad']}</horizontal_fov>
          <image><width>{cam['width']}</width><height>{cam['height']}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>200</far></clip>
        </camera>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed"><parent>base_link</parent><child>camera_link</child></joint>"""


def pose_publisher_and_joints(pods: list[str]) -> str:
    joints = "".join(f"""
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>latch_link</parent_link>
      <child_model>{pod}</child_model>
      <child_link>link</child_link>
      <attach_topic>/dd/{pod}/attach</attach_topic>
      <detach_topic>/dd/{pod}/detach</detach_topic>
      <output_topic>/dd/{pod}/state</output_topic>
    </plugin>""" for pod in pods)
    return f"""<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_model_pose>true</publish_model_pose>
      <publish_nested_model_pose>false</publish_nested_model_pose>
      <use_pose_vector_msg>false</use_pose_vector_msg>
      <update_frequency>50</update_frequency>
    </plugin>{joints}"""


def drone_sdf(lane: dict, cfg: dict, latch_kg: float, pods: list[str]) -> str:
    s = thrust_scale(lane, cfg)
    empty = lane["drone"]["empty_mass_kg"]
    cam = cfg["vehicle"]["camera"]
    cx, cy, cz = cam["mount_m"]
    motors = "".join(motor_plugin(n, *m, s) for n, m in enumerate(MOTORS))
    joints = "".join(f"""
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>latch_link</parent_link>
      <child_model>{pod}</child_model>
      <child_link>link</child_link>
      <attach_topic>/dd/{pod}/attach</attach_topic>
      <detach_topic>/dd/{pod}/detach</detach_topic>
      <output_topic>/dd/{pod}/state</output_topic>
    </plugin>""" for pod in pods)
    return f"""<?xml version="1.0"?>
<!-- Generated by px4/tools/make_vehicle.py from PX4's x500_base; do not edit. Airframe
     {empty} kg, thrust and inertia x{s:.3f} the x500's, latch {latch_kg} kg. -->
<sdf version="1.9">
  <model name="dd_quad">
    <self_collide>false</self_collide>
    {x500_body(empty, s)}
    <link name="latch_link">
      <pose relative_to="base_link">0 0 {LATCH_Z} 0 0 0</pose>
      <inertial>
        <mass>{max(latch_kg, 1e-3)}</mass>
        <inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="latch"><geometry><box><size>0.08 0.08 0.03</size></box></geometry>
        <material><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
    </link>
    <joint name="latch_joint" type="fixed"><parent>base_link</parent><child>latch_link</child></joint>
    <link name="camera_link">
      <!-- Pitched +90 deg: the camera's +x looks down, image up = body forward. -->
      <pose relative_to="base_link">{cx} {cy} {cz} 0 1.5707963 0</pose>
      <inertial>
        <mass>{CAMERA_KG}</mass>
        <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <sensor name="down_cam" type="camera">
        <topic>dd/down_cam</topic>
        <update_rate>{cam['rate_hz']}</update_rate>
        <always_on>1</always_on>
        <camera>
          <horizontal_fov>{cam['horizontal_fov_rad']}</horizontal_fov>
          <image><width>{cam['width']}</width><height>{cam['height']}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>200</far></clip>
        </camera>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed"><parent>base_link</parent><child>camera_link</child></joint>
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_model_pose>true</publish_model_pose>
      <publish_nested_model_pose>false</publish_nested_model_pose>
      <use_pose_vector_msg>false</use_pose_vector_msg>
      <update_frequency>50</update_frequency>
    </plugin>{motors}{joints}
  </model>
</sdf>
"""


def pod_sdf(name: str, mass_kg: float, cfg: dict) -> str:
    sx, sy, sz = cfg["vehicle"]["pod"]["size_m"]
    k = mass_kg / 12
    return f"""<?xml version="1.0"?>
<!-- Generated by px4/tools/make_vehicle.py; do not edit. -->
<sdf version="1.9">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>{mass_kg}</mass>
        <inertia><ixx>{k * (sy**2 + sz**2):.6e}</ixx><iyy>{k * (sx**2 + sz**2):.6e}</iyy><izz>{k * (sx**2 + sy**2):.6e}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <collision name="c">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <surface><contact><collide_bitmask>{POD_MASK}</collide_bitmask></contact></surface>
      </collision>
      <visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><diffuse>0.9 0.5 0.1 1</diffuse><ambient>0.9 0.5 0.1 1</ambient></material></visual>
    </link>
    <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
      <publish_link_pose>false</publish_link_pose>
      <publish_model_pose>true</publish_model_pose>
      <use_pose_vector_msg>false</use_pose_vector_msg>
      <update_frequency>20</update_frequency>
    </plugin>
  </model>
</sdf>
"""


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    (path.parent / "model.config").write_text(f"""<?xml version="1.0"?>
<model><name>{path.parent.name}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf></model>
""")


def run_file(run: Path, name: str) -> Path:
    """A run flown on its own lane keeps lane.json and scene.json in its directory; otherwise
    the default lane's files in px4/config apply."""
    own = run / name
    return own if own.exists() else PX4 / "config" / name


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--payload-kg", type=float, required=True)
    ap.add_argument("--latch-kg", type=float, default=None)
    ap.add_argument("--pods", type=int, default=2, help="0 for a vehicle without pods")
    ap.add_argument("--airframe", choices=["quad", "vtol"], default="quad")
    ap.add_argument("--wind", action="store_true", help="let Gazebo's wind act on the drone's links")
    args = ap.parse_args()
    lane = json.loads(run_file(args.out, "lane.json").read_text())
    cfg = yaml.safe_load((PX4 / "config" / "flight.yaml").read_text())
    latch = lane["drone"]["latch_mass_kg"] if args.latch_kg is None else args.latch_kg
    pod_kg = max(args.payload_kg, cfg["vehicle"]["min_pod_mass_kg"])
    pods = [f"dd_pod_{n}" for n in range(1, args.pods + 1)]
    vtol = args.airframe == "vtol"
    name = "dd_vtol" if vtol else "dd_quad"
    sdf = vtol_sdf(lane, cfg, latch, pods) if vtol else drone_sdf(lane, cfg, latch, pods)
    if args.wind:
        # Gazebo's WindEffects pushes only on links that ask for it (A94).
        sdf = sdf.replace("<inertial>", "<enable_wind>true</enable_wind><inertial>")
    scene = json.loads(run_file(args.out, "scene.json").read_text())
    if scene.get("elevation_m", 0.0) != 0.0:
        # A terrain world (Phase 8) sits at its real elevation, which Gazebo's NavSat adds to
        # the GPS altitude but its air pressure sensor does not: without the same reference the
        # EKF's height jumped by the elevation at arming and the vehicle never took off.
        sdf = sdf.replace("<air_pressure>", f"<air_pressure>\n          <reference_altitude>{scene['elevation_m']}</reference_altitude>", 1)
    write(args.out / "models" / name / "model.sdf", sdf)
    for pod in pods:
        write(args.out / "models" / pod / "model.sdf", pod_sdf(pod, pod_kg, cfg))
    # A terrain world (Phase 8) gives each pad's ground height; the flat world is at 0 (A54).
    ground = scene.get("pad_ground_z_m", {"A": 0.0, "B": 0.0})
    slab_a = ground["A"] + scene["slab_top_m"]
    slab_b = ground["B"] + scene["slab_top_m"]
    a_out = scene["spots"]["A_OUT"]["enu_m"]
    b_out = scene["spots"]["B_OUT"]["enu_m"]
    sz = cfg["vehicle"]["pod"]["size_m"][2]
    base_height, skid_bottom = (-VTOL_SKID_BOTTOM_Z, VTOL_SKID_BOTTOM_Z) if vtol else (BASE_HEIGHT_M, SKID_BOTTOM_Z)
    carry_z = slab_a + base_height + skid_bottom + cfg["vehicle"]["pod"]["carry_clearance_m"] + sz / 2
    # Spawn poses (x, y, z, yaw) for lib/sitl.sh: the drone on A's OUT facing east (along the
    # spots), pod 1 in the latch, pod 2 waiting on B's OUT.
    s = vtol_scale(lane) if vtol else thrust_scale(lane, cfg)
    if vtol:
        proxy = {"hover_motor_constant": VTOL_HOVER_MOTOR_CONSTANT * s,
                 "hover_rotor_radius_m": VTOL_HOVER_RADIUS_M * math.sqrt(s),
                 "pusher_motor_constant": VTOL_PUSHER_MOTOR_CONSTANT * s,
                 "pusher_radius_m": VTOL_PUSHER_RADIUS_M * math.sqrt(s)}
        mount = list(VTOL_CAMERA_MOUNT)
    else:
        proxy = {"hover_motor_constant": X500_MOTOR_CONSTANT * s,
                 "hover_rotor_radius_m": X500_ROTOR_RADIUS_M,
                 "pusher_motor_constant": 0.0, "pusher_radius_m": 0.0}
        mount = cfg["vehicle"]["camera"]["mount_m"]
    info = {"airframe": args.airframe, "model": name, "proxy": proxy, "camera_mount_m": mount,
            "thrust_scale": vtol_scale(lane) if vtol else thrust_scale(lane, cfg),
            "latch_kg": latch, "pod_kg": pod_kg,
            "payload_kg": args.payload_kg,
            "pods": pods, "skid_bottom_z": skid_bottom, "base_height_m": base_height,
            "spawn": {name: [*a_out, slab_a, 0.0],
                      "dd_pod_1": [*a_out, carry_z, 0.0],
                      "dd_pod_2": [*b_out, slab_b + sz / 2, 0.0]}}
    (args.out / "vehicle.json").write_text(json.dumps(info, indent=1) + "\n")
    print(json.dumps(info))
    return 0


if __name__ == "__main__":
    sys.exit(main())
