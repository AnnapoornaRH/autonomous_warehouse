"""Complete autonomous warehouse: simulation + navigation + manipulation + fleet management + dashboard.

Arguments
  gui:=true|false            Gazebo GUI (false = headless server)
  robots:=robot1,robot2,robot3
  use_camera:=true|false
  scheduler:=deadline_aware|nearest_idle
  orders_per_minute:=1.0
  dashboard:=true|false      start the web dashboard backend (http://localhost:3000)
  dashboard_port:=3000
  metrics_dir:=<path>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from warehouse_common import layout as L


def _setup(context):
    robots = [r for r in LaunchConfiguration("robots").perform(context).split(",") if r]
    sim_share = get_package_share_directory("warehouse_simulation")
    nav_share = get_package_share_directory("warehouse_navigation")
    sim_time = {"use_sim_time": True}
    wall = {"use_sim_time": False}
    robots_csv = ",".join(robots)
    tf_remap = [("/tf", "tf"), ("/tf_static", "tf_static")]
    acts = []

    # 1. Gazebo + robots + bridges ------------------------------------------------------------------
    acts.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, "launch", "simulation.launch.py")),
        launch_arguments={"gui": LaunchConfiguration("gui"), "robots": robots_csv,
                          "use_camera": LaunchConfiguration("use_camera")}.items()))
    # 2. simulation services (wall clock so it survives a paused simulation)
    acts.append(Node(package="warehouse_simulation", executable="sim_manager", name="sim_manager", output="screen",
                     parameters=[wall, {"robots": robots_csv, "auto_start": False}]))
    # 3. fleet management ------------------------------------------------------------------------------
    acts.append(Node(package="warehouse_traffic_manager", executable="traffic_manager", name="traffic_manager",
                     output="screen", parameters=[sim_time]))
    acts.append(Node(package="warehouse_order_manager", executable="order_manager", name="order_manager",
                     output="screen",
                     parameters=[sim_time, {"orders_per_minute": LaunchConfiguration("orders_per_minute")}]))
    acts.append(Node(package="warehouse_task_allocator", executable="task_allocator", name="task_allocator",
                     output="screen", parameters=[sim_time, {"scheduler": LaunchConfiguration("scheduler")}]))
    acts.append(Node(package="warehouse_metrics", executable="metrics_node", name="metrics_node", output="screen",
                     parameters=[sim_time, {"output_dir": LaunchConfiguration("metrics_dir")}]))
    # 4. Nav2 (after Gazebo had time to spawn the robots) ----------------------------------------------
    acts.append(TimerAction(period=12.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "navigation.launch.py")),
        launch_arguments={"robots": robots_csv}.items())]))
    # 5. per robot manipulation + state machine ----------------------------------------------------------
    for r in robots:
        acts.append(TimerAction(period=8.0, actions=[
            Node(package="warehouse_manipulation", executable="arm_controller", name="arm_controller",
                 namespace=r, output="screen", parameters=[sim_time], remappings=tf_remap),
            Node(package="warehouse_robot_agent", executable="robot_agent", name="robot_agent", namespace=r,
                 output="screen", parameters=[sim_time], remappings=tf_remap)]))
    # 6. start-up sequencer + dashboard backend -----------------------------------------------------------
    acts.append(Node(package="warehouse_bringup", executable="system_monitor", name="system_monitor",
                     output="screen", parameters=[wall, {"robots": robots_csv}]))
    acts.append(Node(package="warehouse_dashboard_bridge", executable="dashboard_bridge", name="dashboard_bridge",
                     output="screen", condition=IfCondition(LaunchConfiguration("dashboard")),
                     parameters=[wall, {"port": LaunchConfiguration("dashboard_port"),
                                        "robots": robots_csv}]))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("robots", default_value=",".join(L.ROBOT_IDS)),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("scheduler", default_value="deadline_aware"),
        DeclareLaunchArgument("orders_per_minute", default_value="1.0"),
        DeclareLaunchArgument("dashboard", default_value="true"),
        DeclareLaunchArgument("dashboard_port", default_value="3000"),
        DeclareLaunchArgument("metrics_dir", default_value=os.path.join(os.path.expanduser("~"), "warehouse_metrics")),
        OpaqueFunction(function=_setup),
    ])
