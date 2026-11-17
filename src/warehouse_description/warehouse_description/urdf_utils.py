"""Helpers to expand the robot xacro (used by launch files and tests)."""
import os

import xacro
from ament_index_python.packages import get_package_share_directory

ROBOT_RGBA = {
    "robot1": "0.224 0.529 0.898 1",   # blue   #3987e5
    "robot2": "0.851 0.349 0.149 1",   # orange #d95926
    "robot3": "0.098 0.620 0.439 1",   # aqua   #199e70
}


def xacro_path() -> str:
    return os.path.join(get_package_share_directory("warehouse_description"),
                        "urdf", "mobile_manipulator.urdf.xacro")


def build_urdf(robot_name: str, use_camera: bool = True, path: str = None) -> str:
    path = path or xacro_path()
    doc = xacro.process_file(path, mappings={
        "robot_name": robot_name,
        "robot_color": ROBOT_RGBA.get(robot_name, "0.7 0.7 0.7 1"),
        "use_camera": "true" if use_camera else "false",
    })
    return doc.toxml()


def write_urdf(robot_name: str, out_dir: str = "/tmp", use_camera: bool = True) -> str:
    out = os.path.join(out_dir, f"warehouse_{robot_name}.urdf")
    with open(out, "w") as f:
        f.write(build_urdf(robot_name, use_camera))
    return out
