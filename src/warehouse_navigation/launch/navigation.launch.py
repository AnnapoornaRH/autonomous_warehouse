"""Multi-robot Nav2: one complete, independent navigation stack per robot.

  /map                      shared static map (one map_server)
  /<robot>/amcl             localisation (own TF tree /<robot>/tf)
  /<robot>/planner_server   global planner + global costmap
  /<robot>/controller_server  Regulated Pure Pursuit + local costmap (fed by /<robot>/scan)
  /<robot>/behavior_server  spin / wait recoveries
  /<robot>/bt_navigator     NavigateToPose action server (/<robot>/navigate_to_pose)
  /<robot>/cmd_vel_gate     Nav2 cmd_vel_nav -> gated cmd_vel

Nothing here is shared between robots except the map, so the Nav2 stacks cannot interfere with
each other: topics, services, actions, parameters, costmaps and TF trees are all namespaced.
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from warehouse_common import layout as L


def _setup(context):
    robots = [r for r in LaunchConfiguration("robots").perform(context).split(",") if r]
    nav_share = get_package_share_directory("warehouse_navigation")
    params = os.path.join(nav_share, "config", "nav2_params.yaml")
    map_yaml = os.path.join(nav_share, "maps", "warehouse.yaml")
    remaps_tf = [("/tf", "tf"), ("/tf_static", "tf_static")]

    actions = []
    # shared map server -------------------------------------------------------------------
    actions.append(Node(package="nav2_map_server", executable="map_server", name="map_server",
                        output="screen",
                        parameters=[{"use_sim_time": True, "yaml_filename": map_yaml,
                                     "topic_name": "map", "frame_id": "map"}]))
    actions.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                        name="lifecycle_manager_map", output="screen",
                        parameters=[{"use_sim_time": True, "autostart": True,
                                     "node_names": ["map_server"], "bond_timeout": 0.0}]))

    for index, robot in enumerate(robots):
        x, y, yaw = L.bay_pose(robot)
        cfg = RewrittenYaml(source_file=params, root_key=robot, param_rewrites={}, convert_types=True)
        init = {"initial_pose.x": float(x), "initial_pose.y": float(y), "initial_pose.yaw": float(yaw)}

        def nav2(pkg, exe, name, extra_remaps=(), extra_params=None):
            return Node(package=pkg, executable=exe, name=name, namespace=robot, output="screen",
                        parameters=[cfg] + ([extra_params] if extra_params else []),
                        remappings=remaps_tf + list(extra_remaps))

        group = [
            nav2("nav2_amcl", "amcl", "amcl", extra_params=init),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name="lifecycle_manager_localization", namespace=robot, output="screen",
                 parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["amcl"],
                              "bond_timeout": 0.0}]),
            nav2("nav2_controller", "controller_server", "controller_server",
                 extra_remaps=[("cmd_vel", "cmd_vel_nav")]),
            nav2("nav2_planner", "planner_server", "planner_server"),
            nav2("nav2_behaviors", "behavior_server", "behavior_server",
                 extra_remaps=[("cmd_vel", "cmd_vel_nav")]),
            nav2("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
            Node(package="warehouse_navigation", executable="cmd_vel_gate", name="cmd_vel_gate",
                 namespace=robot, output="screen", parameters=[{"use_sim_time": True}]),
        ]
        nav_lm = Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                      name="lifecycle_manager_navigation", namespace=robot, output="screen",
                      parameters=[{"use_sim_time": True, "autostart": True, "bond_timeout": 0.0,
                                   "node_names": ["controller_server", "planner_server",
                                                  "behavior_server", "bt_navigator"]}])
        # stagger the robots: starting 3 x 9 Nav2 nodes at the same instant overloads small machines
        stagger = 20.0 * index
        actions.append(TimerAction(period=stagger, actions=group))
        # bring navigation up after localisation had a moment to publish map->odom
        actions.append(TimerAction(period=stagger + 10.0, actions=[nav_lm]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value=",".join(L.ROBOT_IDS)),
        OpaqueFunction(function=_setup),
    ])
