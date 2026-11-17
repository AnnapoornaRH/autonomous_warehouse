#!/usr/bin/env python3
"""Velocity gate between Nav2 and the Gazebo diff-drive plugin (one instance per robot).

    Nav2 (cmd_vel_nav) --> [gate] --> cmd_vel --> ros_gz_bridge --> gz DiffDrive

The gate guarantees that

  * the base NEVER moves while the arm is manipulating   (topic ``manipulating``: Bool)
  * the base NEVER moves during an emergency stop         (global topic ``/warehouse/estop``: Bool, latched)
  * a stale command (Nav2 stopped publishing) turns into zero velocity within ``cmd_timeout``
  * the robot stops when something is closer than ``stop_distance`` in front of the LiDAR
    (last-line-of-defence safety stop, independent of Nav2's costmaps)

It publishes zero velocity continuously while blocked so that the diff-drive plugin actively
holds the wheels (without a command the wheels would roll when the arm pushes on the chassis).
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class CmdVelGate(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_gate")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("stop_distance", 0.15)
        self.declare_parameter("stop_cone_deg", 35.0)
        self.declare_parameter("safety_stop_enabled", True)
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        self.stop_distance = self.get_parameter("stop_distance").value
        self.stop_cone = math.radians(self.get_parameter("stop_cone_deg").value)
        self.safety_enabled = self.get_parameter("safety_stop_enabled").value

        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.manipulating = False
        self.estop = False
        self.front_min = float("inf")
        self.rear_min = float("inf")
        self.safety_active = False

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "base_guard_status", 10)
        self.create_subscription(Twist, "cmd_vel_nav", self.on_cmd, 10)
        self.create_subscription(Bool, "manipulating", self.on_manip, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, "/warehouse/estop", self.on_estop, latched)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan_rear", self.on_scan_rear, qos_profile_sensor_data)
        self.create_timer(1.0 / self.get_parameter("rate").value, self.tick)
        self.create_timer(1.0, self.publish_status)

    def on_cmd(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def on_manip(self, msg: Bool) -> None:
        self.manipulating = msg.data

    def on_estop(self, msg: Bool) -> None:
        self.estop = msg.data

    def on_scan(self, msg: LaserScan) -> None:
        best = float("inf")
        a = msg.angle_min
        for r in msg.ranges:
            if abs(a) <= self.stop_cone and msg.range_min <= r <= msg.range_max and r < best:
                best = r
            a += msg.angle_increment
        self.front_min = best

    def on_scan_rear(self, msg: LaserScan) -> None:
        best = float("inf")
        a = msg.angle_min
        for r in msg.ranges:
            if abs(a) <= self.stop_cone and msg.range_min <= r <= msg.range_max and r < best:
                best = r
            a += msg.angle_increment
        self.rear_min = best

    def tick(self) -> None:
        out = Twist()
        blocked = self.manipulating or self.estop
        fresh = (self.last_cmd_time is not None and
                 (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 < self.cmd_timeout)
        if not blocked and fresh:
            out = self.last_cmd
            if self.safety_enabled and out.linear.x > 0.0 and self.front_min < self.stop_distance:
                out = Twist()
                out.angular.z = self.last_cmd.angular.z * 0.5
                self.safety_active = True
            elif self.safety_enabled and out.linear.x < 0.0 and self.rear_min < self.stop_distance:
                out = Twist()
                out.angular.z = self.last_cmd.angular.z * 0.5
                self.safety_active = True
            else:
                self.safety_active = False
        self.pub.publish(out)

    def publish_status(self) -> None:
        s = "estop" if self.estop else "manipulating" if self.manipulating else \
            "safety_stop" if self.safety_active else "ok"
        self.status_pub.publish(String(data=s))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
