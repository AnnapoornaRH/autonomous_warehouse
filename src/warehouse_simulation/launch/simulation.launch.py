"""Gazebo warehouse + three robots (spawn, bridges, robot_state_publisher) + simulation services.

Arguments
  gui:=true|false          open the Gazebo GUI (false = headless server with software rendering)
  robots:=robot1,robot2,robot3
  use_camera:=true|false   per-robot camera (costs CPU in VMs without a GPU)
  world:=<path>            override the world file
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from warehouse_common import layout as L
from warehouse_description.urdf_utils import write_urdf
from warehouse_simulation import bridge_config as B


def _b(v: str) -> bool:
    return str(v).lower() in ("1", "true", "yes", "on")


def _setup(context):
    gui = _b(LaunchConfiguration("gui").perform(context))
    use_camera = _b(LaunchConfiguration("use_camera").perform(context))
    robots = [r for r in LaunchConfiguration("robots").perform(context).split(",") if r]
    world = LaunchConfiguration("world").perform(context)
    share = get_package_share_directory("warehouse_simulation")
    if not world:
        world = os.path.join(share, "worlds", "warehouse.sdf")
    tmp = os.path.join(tempfile.gettempdir(), "warehouse_launch")
    os.makedirs(tmp, exist_ok=True)

    actions = []
    # ------------------------------------------------------------------ Gazebo
    if gui:
        gz_cmd = ["gz", "sim", "-r", "--gui-config", os.path.join(share, "config", "gui.config"), world]
    else:
        gz_cmd = ["gz", "sim", "-r", "-s", "--headless-rendering", world]
    actions.append(ExecuteProcess(cmd=gz_cmd, name="gazebo", output="screen"))

    # ------------------------------------------------------------------ bridges
    world_cfg = B.write_yaml(B.world_bridge_entries(), os.path.join(tmp, "bridge_world.yaml"))
    actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="world_bridge",
                        parameters=[{"config_file": world_cfg}], output="screen"))
    actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="world_service_bridge",
                        arguments=B.world_service_args(), output="screen"))

    actions.append(Node(package="warehouse_simulation", executable="sim_relay", name="sim_relay", output="screen",
                        parameters=[{"robots": ",".join(robots), "use_sim_time": False}]))

    wait_world = (f"until gz service -l 2>/dev/null | grep -q /world/{L.WORLD_NAME}/create; do sleep 1; done; ")
    for i, robot in enumerate(robots):
        x, y, yaw = L.bay_pose(robot)
        urdf_path = write_urdf(robot, tmp, use_camera)
        with open(urdf_path) as f:
            urdf = f.read()
        actions.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
                            namespace=robot, name="robot_state_publisher",
                            parameters=[{"robot_description": urdf, "use_sim_time": True}],
                            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")], output="screen"))
        cfg = B.write_yaml(B.robot_bridge_entries(robot, use_camera), os.path.join(tmp, f"bridge_{robot}.yaml"))
        actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name=f"{robot}_bridge",
                            parameters=[{"config_file": cfg}], output="screen"))
        spawn = (wait_world + f"sleep {1.5 * i}; ros2 run ros_gz_sim create -world {L.WORLD_NAME} "
                 f"-file {urdf_path} -name {robot} -x {x} -y {y} -z 0.02 -Y {yaw}")
        actions.append(ExecuteProcess(cmd=["bash", "-c", spawn], name=f"spawn_{robot}", output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("robots", default_value=",".join(L.ROBOT_IDS)),
        DeclareLaunchArgument("world", default_value=""),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                               os.environ.get("GZ_SIM_RESOURCE_PATH", "")),
        OpaqueFunction(function=_setup),
    ])
