#!/usr/bin/env python3
"""Rate limiter for the two highest-rate Gazebo streams.

Gazebo publishes /clock and the joint states of every robot once per physics step (250 Hz of simulation time).
Forwarding that to ~50 ROS nodes (every node on simulation time subscribes to /clock) and into robot_state_publisher
(which turns every joint-state message into TF) wastes most of the CPU of a modest machine.

  /sim/clock_raw             (ros_gz_bridge)  ->  /clock                    50 Hz of simulation time
  /<robot>/joint_states_raw  (ros_gz_bridge)  ->  /<robot>/joint_states     20 Hz of simulation time

The throttling is done on *simulation* time, so behaviour is identical at any real-time factor.
This node runs on wall time (use_sim_time=false) because it generates /clock.
"""
import rclpy
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from warehouse_common import layout as L


class SimRelay(Node):
    def __init__(self) -> None:
        super().__init__("sim_relay")
        self.declare_parameter("robots", ",".join(L.ROBOT_IDS))
        self.declare_parameter("clock_hz", 50.0)
        self.declare_parameter("joint_hz", 20.0)
        robots = [r for r in self.get_parameter("robots").value.split(",") if r]
        self.clock_dt = 1.0 / float(self.get_parameter("clock_hz").value)
        self.joint_dt = 1.0 / float(self.get_parameter("joint_hz").value)
        self.last_clock = -1e9
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.create_subscription(Clock, "/sim/clock_raw", self.on_clock, qos_profile_sensor_data)
        self.js_pub = {}
        self.js_last = {}
        for r in robots:
            self.js_pub[r] = self.create_publisher(JointState, f"/{r}/joint_states", 10)
            self.js_last[r] = -1e9
            self.create_subscription(JointState, f"/{r}/joint_states_raw",
                                     lambda m, rr=r: self.on_js(rr, m), qos_profile_sensor_data)
        self.get_logger().info(f"sim_relay: /clock {1/self.clock_dt:.0f} Hz, joint states {1/self.joint_dt:.0f} Hz (sim time)")

    def on_clock(self, msg: Clock) -> None:
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if t < self.last_clock:                      # simulation restarted: always forward
            self.last_clock = t
            self.clock_pub.publish(msg)
        elif t - self.last_clock >= self.clock_dt - 1e-9:
            self.last_clock = t
            self.clock_pub.publish(msg)

    def on_js(self, robot: str, msg: JointState) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - self.js_last[robot] >= self.joint_dt - 1e-9 or t < self.js_last[robot]:
            self.js_last[robot] = t
            self.js_pub[robot].publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimRelay()
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
