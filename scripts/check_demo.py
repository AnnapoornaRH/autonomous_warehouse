#!/usr/bin/env python3
"""Offline demo release verification: no ROS runtime required."""
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parent.parent
required = ["demo.sh", "run_warehouse.sh", "README.md",
            "src/warehouse_description/urdf/mobile_manipulator.urdf.xacro",
            "src/warehouse_simulation/worlds/warehouse.sdf",
            "src/warehouse_navigation/maps/warehouse.yaml",
            "src/warehouse_bringup/launch/warehouse.launch.py",
            "dashboard/dist/index.html"]
for filename in required:
    assert (root / filename).is_file(), f"missing {filename}"
robot = (root / required[3]).read_text()
for i in range(1, 5):
    assert re.search(fr'name="arm_joint{i}" type="revolute"', robot), f"missing arm_joint{i}"
assert "finger_left_joint" in robot and "finger_right_joint" in robot, "missing gripper fingers"
ET.parse(root / required[4])
manifests = sorted((root / "src").glob("*/package.xml"))
assert len(manifests) >= 10
for m in manifests:
    ET.parse(m)
print(f"OK: model 4-DOF, gripper, world, dashboard, launch, {len(manifests)} ROS packages")
print("Runtime with ROS 2 Jazzy + Gazebo remains to be checked on target machine")
