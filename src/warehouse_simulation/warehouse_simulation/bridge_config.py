"""ros_gz_bridge configuration generators (YAML written to a temp file at launch time)."""
from __future__ import annotations

from typing import Dict, List

import yaml

from warehouse_common import layout as L


def _entry(ros: str, gz: str, ros_type: str, gz_type: str, direction: str) -> Dict[str, str]:
    return {"ros_topic_name": ros, "gz_topic_name": gz, "ros_type_name": ros_type,
            "gz_type_name": gz_type, "direction": direction}


def robot_bridge_entries(robot: str, use_camera: bool = True) -> List[Dict[str, str]]:
    m = f"/model/{robot}"
    e: List[Dict[str, str]] = [
        # raw velocity command from the cmd_vel gate (Nav2 -> gate -> here)
        _entry(f"/{robot}/cmd_vel", f"{m}/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist", "ROS_TO_GZ"),
        _entry(f"/{robot}/odom", f"{m}/odometry", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
        _entry(f"/{robot}/tf", f"{m}/tf", "tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V", "GZ_TO_ROS"),
        _entry(f"/{robot}/scan", f"{m}/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan", "GZ_TO_ROS"),
        _entry(f"/{robot}/scan_rear", f"{m}/scan_rear", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan", "GZ_TO_ROS"),
        _entry(f"/{robot}/joint_states_raw", f"{m}/joint_states", "sensor_msgs/msg/JointState",
               "gz.msgs.Model", "GZ_TO_ROS"),
        _entry(f"/{robot}/gt_pose", f"{m}/pose", "tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V", "GZ_TO_ROS"),
        _entry(f"/{robot}/contact",
               f"/world/{L.WORLD_NAME}/model/{robot}/link/base_link/sensor/chassis_contact/contact",
               "ros_gz_interfaces/msg/Contacts", "gz.msgs.Contacts", "GZ_TO_ROS"),
    ]
    for j in range(1, 5):
        e.append(_entry(f"/{robot}/arm/joint{j}/cmd_pos", f"{m}/arm/joint{j}/cmd_pos",
                        "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"))
    for side in ("left", "right"):
        e.append(_entry(f"/{robot}/gripper/{side}/cmd_pos", f"{m}/gripper/{side}/cmd_pos",
                        "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"))
    if use_camera:
        e.append(_entry(f"/{robot}/camera/image_raw", f"{m}/camera/image", "sensor_msgs/msg/Image",
                        "gz.msgs.Image", "GZ_TO_ROS"))
        e.append(_entry(f"/{robot}/camera/camera_info", f"{m}/camera/camera_info",
                        "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "GZ_TO_ROS"))
    return e


def world_bridge_entries() -> List[Dict[str, str]]:
    w = f"/world/{L.WORLD_NAME}"
    # raw 250 Hz streams are rate-limited by sim_relay (-> /clock, /<robot>/joint_states)
    entries = [_entry("/sim/clock_raw", f"{w}/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS")]
    # ground-truth item poses (PosePublisher plugin in every item model)
    for iid in L.ITEMS:
        entries.append(_entry(f"/items/{iid}/gt_pose", f"/model/{iid}/pose", "tf2_msgs/msg/TFMessage",
                              "gz.msgs.Pose_V", "GZ_TO_ROS"))
    return entries


def write_yaml(entries: List[Dict[str, str]], path: str) -> str:
    with open(path, "w") as f:
        yaml.safe_dump(entries, f, sort_keys=False)
    return path


# services bridged with command-line style arguments ("<gz service>@<ros srv type>")
def world_service_args() -> List[str]:
    w = f"/world/{L.WORLD_NAME}"
    return [f"{w}/control@ros_gz_interfaces/srv/ControlWorld",
            f"{w}/set_pose@ros_gz_interfaces/srv/SetEntityPose"]
