#!/usr/bin/env bash
# Static / offline tests (no simulation needed):
#   1. python unit tests of the core logic (layout, IK, reservations, scheduler, orders)
#   2. package.xml validity
#   3. robot xacro -> URDF -> SDF conversion, contact-sensor collision name, controller topics
#   4. generated world / map are in sync with warehouse_common.layout and the world SDF is valid
# Run after building and sourcing ROS:   source /opt/ros/jazzy/setup.bash && source install/setup.bash && scripts/run_tests.sh
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
fail=0
step() { echo; echo "== $*"; }
bad() { echo "FAIL: $*"; fail=1; }

step "1/4 unit tests"
python3 -m pytest src/warehouse_common/test -q -p no:cacheprovider || bad "unit tests"

step "2/4 package manifests"
python3 - <<'PY' || bad "package.xml"
import glob, xml.dom.minidom
fs = glob.glob("src/*/package.xml")
for f in fs:
    xml.dom.minidom.parse(f)
print(f"{len(fs)} package.xml files are well-formed")
PY

step "3/4 robot model"
TMP="$(mktemp -d)"
if command -v xacro >/dev/null && command -v gz >/dev/null; then
  xacro src/warehouse_description/urdf/mobile_manipulator.urdf.xacro robot_name:=robot1 > "$TMP/r.urdf" 2>/dev/null || bad "xacro"
  command -v check_urdf >/dev/null && { check_urdf "$TMP/r.urdf" >/dev/null || bad "check_urdf"; }
  gz sdf -p "$TMP/r.urdf" > "$TMP/r.sdf" 2>/dev/null || bad "urdf -> sdf"
  python3 - "$TMP/r.sdf" <<'PY' || bad "robot sdf content"
import re, sys
s = open(sys.argv[1]).read()
need = ["base_link_fixed_joint_lump__chassis_collision_collision", "gz::sim::systems::DiffDrive",
        "gz::sim::systems::JointStatePublisher", "gz::sim::systems::PosePublisher", "gpu_lidar",
        "/model/robot1/cmd_vel", "/model/robot1/arm/joint4/cmd_pos", "/model/robot1/gripper/right/cmd_pos"]
miss = [n for n in need if n not in s]
joints = re.findall(r"<joint name='([^']+)'", s)
print("joints:", joints)
assert not miss, f"missing in SDF: {miss}"
assert sum(j.startswith("arm_joint") for j in joints) == 4 and sum(j.startswith("finger_") for j in joints) == 2
PY
else
  echo "xacro / gz not available - skipped"
fi

step "4/4 world and map"
python3 - <<'PY' || bad "world / map generation"
import filecmp, os, sys, tempfile
sys.path.insert(0, "src/warehouse_common"); sys.path.insert(0, "src/warehouse_simulation")
from warehouse_simulation import world_builder as W
d = tempfile.mkdtemp()
W.write_assets(os.path.join(d, "w.sdf"), os.path.join(d, "maps"))
same_world = filecmp.cmp(os.path.join(d, "w.sdf"), "src/warehouse_simulation/worlds/warehouse.sdf", shallow=False)
same_map = filecmp.cmp(os.path.join(d, "maps", "warehouse.pgm"), "src/warehouse_navigation/maps/warehouse.pgm", shallow=False)
print("world in sync:", same_world, "| map in sync:", same_map)
assert same_world and same_map, "run scripts/generate_assets.py"
PY
command -v gz >/dev/null && { gz sdf -k src/warehouse_simulation/worlds/warehouse.sdf || bad "world SDF invalid"; }
rm -rf "$TMP"

echo
if [[ $fail -eq 0 ]]; then echo "ALL OFFLINE TESTS PASSED"; else echo "SOME TESTS FAILED"; fi
exit $fail
