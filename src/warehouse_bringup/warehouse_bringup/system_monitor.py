#!/usr/bin/env python3
"""System monitor / start-up sequencer.

Waits until every subsystem is really up (instead of launching everything and hoping):
  per robot : Nav2 NavigateToPose server, pick_place server, assign_order service, localisation (map->odom TF),
              ground-truth pose
  global    : traffic manager services, order manager services, sim manager services, Gazebo services
When all checks pass it calls the sim manager's ``start`` command (=> orders start to arrive, allocation begins)
and publishes the result on /warehouse/startup_status (JSON, latched) which the dashboard shows.
Runs on wall time.
"""
import json
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from warehouse_common import layout as L
from warehouse_robot_interfaces.action import PickPlace
from warehouse_robot_interfaces.msg import EntityPoses
from warehouse_robot_interfaces.srv import AssignOrder, GenerateOrders, ReleaseResources, RequestResources, SimCommand

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class SystemMonitor(Node):
    def __init__(self) -> None:
        super().__init__("system_monitor")
        self.declare_parameter("robots", ",".join(L.ROBOT_IDS))
        self.declare_parameter("auto_start", True)
        self.robots = [r for r in self.get_parameter("robots").value.split(",") if r]
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.t0 = time.time()
        self.started = False
        self.have_map_odom = {r: False for r in self.robots}
        self.have_gt = set()
        self.nav = {r: ActionClient(self, NavigateToPose, f"/{r}/navigate_to_pose") for r in self.robots}
        self.pp = {r: ActionClient(self, PickPlace, f"/{r}/pick_place") for r in self.robots}
        self.assign = {r: self.create_client(AssignOrder, f"/{r}/assign_order") for r in self.robots}
        # supervision of the two Nav2 lifecycle managers of every robot (localisation, navigation)
        self.lc = {}
        for r in self.robots:
            for mgr in ("lifecycle_manager_localization", "lifecycle_manager_navigation"):
                self.lc[(r, mgr)] = {
                    "active": self.create_client(Trigger, f"/{r}/{mgr}/is_active"),
                    "manage": self.create_client(ManageLifecycleNodes, f"/{r}/{mgr}/manage_nodes"),
                    "ok": False, "since": None, "next_retry": 0.0, "busy": False}
        self.traffic = self.create_client(RequestResources, "/traffic/request")
        self.traffic_rel = self.create_client(ReleaseResources, "/traffic/release")
        self.orders = self.create_client(GenerateOrders, "/warehouse/generate_orders")
        self.sim = self.create_client(SimCommand, "/warehouse/sim_command")
        for r in self.robots:
            self.create_subscription(TFMessage, f"/{r}/tf", lambda m, rr=r: self.on_tf(rr, m), 10)
        self.create_subscription(EntityPoses, "/warehouse/ground_truth", self.on_gt, 10)
        self.pub = self.create_publisher(String, "/warehouse/startup_status", LATCHED)
        self.ready_pub = self.create_publisher(Bool, "/warehouse/system_ready", LATCHED)
        self.ready_pub.publish(Bool(data=False))
        self.create_timer(1.0, self.tick)

    def on_tf(self, robot: str, msg: TFMessage) -> None:
        for t in msg.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                self.have_map_odom[robot] = True

    def on_gt(self, msg: EntityPoses) -> None:
        self.have_gt.update(msg.names)

    def lifecycle_active(self, r: str, mgr: str) -> bool:
        """Is every node of this robot's Nav2 lifecycle manager active?  Under heavy start-up load a manager
        can hang or give up ('Failed to bring up all requested nodes'); then reset and start it again."""
        d = self.lc[(r, mgr)]
        if d["ok"]:
            return True
        now = time.time()
        if d["since"] is None:
            d["since"] = now
        if d["active"].service_is_ready():
            fut = d["active"].call_async(Trigger.Request())
            fut.add_done_callback(lambda f, dd=d: dd.__setitem__("ok", bool(f.result() and f.result().success)))
        if now - d["since"] > 60.0 and now >= d["next_retry"] and not d["busy"] and d["manage"].service_is_ready():
            d["busy"] = True
            d["next_retry"] = now + 75.0
            self.get_logger().warn(f"{r}: {mgr} not active after {now - d['since']:.0f}s -> restarting its bring-up")

            def startup(_f, dd=d):
                req2 = ManageLifecycleNodes.Request()
                req2.command = ManageLifecycleNodes.Request.STARTUP
                dd["manage"].call_async(req2).add_done_callback(lambda _g: dd.__setitem__("busy", False))

            req = ManageLifecycleNodes.Request()
            req.command = ManageLifecycleNodes.Request.RESET
            d["manage"].call_async(req).add_done_callback(startup)
        return False

    def checks(self) -> dict:
        c = {}
        c["gazebo_ground_truth"] = all(r in self.have_gt for r in self.robots) and len(self.have_gt) >= len(L.ITEMS)
        for r in self.robots:
            c[f"{r}_localisation"] = self.have_map_odom[r] and self.lifecycle_active(r, "lifecycle_manager_localization")
            c[f"{r}_nav2"] = self.nav[r].server_is_ready() and self.lifecycle_active(r, "lifecycle_manager_navigation")
            c[f"{r}_arm"] = self.pp[r].server_is_ready()
            c[f"{r}_agent"] = self.assign[r].service_is_ready()
        nodes = {n for n in self.get_node_names()}
        for n in ("task_allocator", "metrics_node", "order_manager", "traffic_manager"):
            c[f"node_{n}"] = n in nodes
        c["traffic_manager"] = self.traffic.service_is_ready() and self.traffic_rel.service_is_ready()
        c["order_manager"] = self.orders.service_is_ready()
        c["sim_manager"] = self.sim.service_is_ready()
        return c

    def tick(self) -> None:
        c = self.checks()
        ready = all(c.values())
        status = {"ready": ready, "started": self.started, "elapsed": round(time.time() - self.t0, 1),
                  "checks": c, "missing": [k for k, v in c.items() if not v]}
        self.pub.publish(String(data=json.dumps(status)))
        if ready and not self.started:
            self.get_logger().info("all subsystems are up -> starting the warehouse")
            self.started = True
            self.ready_pub.publish(Bool(data=True))
            if self.auto_start:
                req = SimCommand.Request()
                req.command = "start"
                self.sim.call_async(req)
        elif not self.started and int(time.time() - self.t0) % 10 == 0:
            self.get_logger().info(f"waiting for: {status['missing']}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemMonitor()
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
