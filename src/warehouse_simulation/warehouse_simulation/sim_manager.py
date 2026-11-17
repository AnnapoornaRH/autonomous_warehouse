#!/usr/bin/env python3
"""Simulation manager: the only node that talks to Gazebo's world services.

Responsibilities
  * ground truth        merges the per-model pose streams into /warehouse/ground_truth (EntityPoses)
  * collision monitor   chassis contact sensors -> WarehouseEvent(category=collision), debounced per
                        contact episode; near misses (robot-robot centre distance) are reported separately
  * sim control         service /warehouse/sim_command: pause | resume | start | reset | estop | release_estop
  * item watchdog       items that were knocked off their shelf (and are not part of an active order)
                        are put back; delivered items are restocked after a short packing delay
  * state               /warehouse/sim_state (JSON string, latched) for the dashboard

This node runs on WALL time (use_sim_time=false) so it keeps answering while the simulation is paused.
"""
import json
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts, Entity
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

from warehouse_common import layout as L
from warehouse_robot_interfaces.msg import EntityPoses, OrderList, WarehouseEvent
from warehouse_robot_interfaces.srv import SimCommand

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def classify(name: str) -> str:
    n = name.split("::")[0]
    if n.startswith("wall"):
        return "wall"
    if n.startswith("shelf"):
        return "shelf"
    if n.startswith("station"):
        return "station"
    if n.startswith("robot"):
        return "robot"
    if n.startswith(("crate", "barrel", "pallet")):
        return "obstacle"
    if n.startswith("item"):
        return "item"
    if n == "ground" or n == "warehouse_floor":
        return "ground"
    return "other"


class SimManager(Node):
    def __init__(self) -> None:
        super().__init__("sim_manager")
        self.declare_parameter("robots", ",".join(L.ROBOT_IDS))
        self.declare_parameter("auto_start", True)
        self.declare_parameter("restock_delay", 6.0)
        self.declare_parameter("near_miss_distance", 0.70)
        self.declare_parameter("collision_debounce", 2.0)
        self.robots: List[str] = [r for r in self.get_parameter("robots").value.split(",") if r]
        self.restock_delay = float(self.get_parameter("restock_delay").value)
        self.near_miss_d = float(self.get_parameter("near_miss_distance").value)
        self.coll_debounce = float(self.get_parameter("collision_debounce").value)

        self.cb = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.sim_time = 0.0
        self.paused = False
        self.estop = False
        self.state = "RUNNING" if self.get_parameter("auto_start").value else "STOPPED"
        self.resetting = False
        self.poses: Dict[str, Tuple[float, float, float, float, float, float, float]] = {}  # x y z qx qy qz qw
        self.active_items: Dict[str, str] = {}            # item_id -> order status (ASSIGNED/IN_PROGRESS)
        self.restock_queue: Dict[str, float] = {}         # item_id -> wall time to restock
        self.last_contact: Dict[Tuple[str, str], float] = {}
        self.collision_count = 0
        self.near_pairs: Dict[Tuple[str, str], bool] = {}
        self._rtf_ref = (time.time(), 0.0)
        self.rtf = 0.0

        # ---------------------------------------------------------------- publishers
        self.gt_pub = self.create_publisher(EntityPoses, "/warehouse/ground_truth", 10)
        self.event_pub = self.create_publisher(WarehouseEvent, "/warehouse/events", 50)
        self.estop_pub = self.create_publisher(Bool, "/warehouse/estop", LATCHED)
        self.reset_pub = self.create_publisher(String, "/warehouse/reset", 10)
        self.state_pub = self.create_publisher(String, "/warehouse/sim_state", LATCHED)
        self.run_pub = self.create_publisher(Bool, "/warehouse/running", LATCHED)
        self.initpose_pubs = {r: self.create_publisher(PoseWithCovarianceStamped, f"/{r}/initialpose", 10)
                              for r in self.robots}
        self.estop_pub.publish(Bool(data=False))

        # ---------------------------------------------------------------- subscriptions
        self.create_subscription(Clock, "/clock", self.on_clock, 10, callback_group=self.cb)
        for r in self.robots:
            self.create_subscription(TFMessage, f"/{r}/gt_pose", self.on_pose, 10, callback_group=self.cb)
            self.create_subscription(Contacts, f"/{r}/contact", lambda m, rr=r: self.on_contact(rr, m),
                                     10, callback_group=self.cb)
        for iid in L.ITEMS:
            self.create_subscription(TFMessage, f"/items/{iid}/gt_pose", self.on_pose, 10,
                                     callback_group=self.cb)
        self.create_subscription(OrderList, "/warehouse/orders", self.on_orders, 10, callback_group=self.cb)
        self.create_subscription(String, "/warehouse/restock", self.on_restock, 10, callback_group=self.cb)

        # ---------------------------------------------------------------- gazebo clients / services
        self.control_cli = self.create_client(ControlWorld, f"/world/{L.WORLD_NAME}/control",
                                              callback_group=self.cb)
        self.setpose_cli = self.create_client(SetEntityPose, f"/world/{L.WORLD_NAME}/set_pose",
                                              callback_group=self.cb)
        self.create_service(SimCommand, "/warehouse/sim_command", self.on_command, callback_group=self.cb)

        self.create_timer(0.1, self.publish_gt, callback_group=self.cb)
        self.create_timer(0.5, self.publish_state, callback_group=self.cb)
        self.create_timer(0.5, self.check_near_miss, callback_group=self.cb)
        self.create_timer(1.0, self.watchdog, callback_group=self.cb)
        self.run_pub.publish(Bool(data=self.state == "RUNNING"))
        self.get_logger().info("sim_manager ready (robots: %s, state %s)" % (self.robots, self.state))

    # ------------------------------------------------------------------ data in
    def on_clock(self, msg: Clock) -> None:
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        now = time.time()
        if now - self._rtf_ref[0] >= 2.0:
            dt = now - self._rtf_ref[0]
            self.rtf = max(0.0, (self.sim_time - self._rtf_ref[1]) / dt)
            self._rtf_ref = (now, self.sim_time)

    def on_pose(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            t, q = tf.transform.translation, tf.transform.rotation
            with self.lock:
                self.poses[tf.child_frame_id] = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)

    def on_orders(self, msg: OrderList) -> None:
        active = {o.item_id: o.status for o in msg.orders if o.status in ("ASSIGNED", "IN_PROGRESS")}
        with self.lock:
            self.active_items = active

    def on_restock(self, msg: String) -> None:
        iid = msg.data
        if iid in L.ITEMS:
            with self.lock:
                self.restock_queue[iid] = self.sim_time + self.restock_delay       # simulation seconds

    # ------------------------------------------------------------------ data out
    def publish_gt(self) -> None:
        msg = EntityPoses()
        msg.stamp = self._stamp()
        with self.lock:
            for name, p in self.poses.items():
                msg.names.append(name)
                from geometry_msgs.msg import Pose
                ps = Pose()
                ps.position.x, ps.position.y, ps.position.z = p[0], p[1], p[2]
                ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = p[3:7]
                msg.poses.append(ps)
        self.gt_pub.publish(msg)

    def _stamp(self) -> TimeMsg:
        t = TimeMsg()
        t.sec = int(self.sim_time)
        t.nanosec = int((self.sim_time - int(self.sim_time)) * 1e9)
        return t

    def publish_state(self) -> None:
        self.state_pub.publish(String(data=json.dumps({
            "state": self.state, "paused": self.paused, "estop": self.estop,
            "sim_time": round(self.sim_time, 2), "rtf": round(self.rtf, 2),
            "running": self.state == "RUNNING" and not self.estop}))
        )

    def emit(self, category: str, severity: str, robot: str, detail: str, value: float = 0.0,
             order_id: str = "") -> None:
        e = WarehouseEvent()
        e.stamp = self._stamp()
        e.category, e.severity, e.robot_id, e.order_id, e.detail, e.value = category, severity, robot, \
            order_id, detail, float(value)
        self.event_pub.publish(e)

    # ------------------------------------------------------------------ collisions / near misses
    def on_contact(self, robot: str, msg: Contacts) -> None:
        now = time.time()
        for c in msg.contacts:
            a, b = c.collision1.name, c.collision2.name
            other = b if a.startswith(robot + "::") else a
            kind = classify(other)
            if kind in ("ground", "item"):
                continue
            other_name = other.split("::")[0]
            # both robots report a robot-robot contact: count that collision only once
            key = tuple(sorted((robot, other_name))) if kind == "robot" else (robot, other_name)
            last = self.last_contact.get(key, 0.0)
            self.last_contact[key] = now
            if now - last > self.coll_debounce:           # new contact episode
                self.collision_count += 1
                self.emit("collision", "error", robot, f"{robot} collided with {kind} "
                          f"'{other.split('::')[0]}'", 1.0)
                self.get_logger().warn(f"COLLISION {robot} <-> {other}")

    def robot_xy(self, r: str) -> Optional[Tuple[float, float]]:
        with self.lock:
            p = self.poses.get(r)
        return (p[0], p[1]) if p else None

    def check_near_miss(self) -> None:
        if self.paused:
            return
        for i, a in enumerate(self.robots):
            for b in self.robots[i + 1:]:
                pa, pb = self.robot_xy(a), self.robot_xy(b)
                if not pa or not pb:
                    continue
                d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
                key = (a, b)
                if d < self.near_miss_d and not self.near_pairs.get(key):
                    self.near_pairs[key] = True
                    self.emit("near_miss", "warn", a, f"{a} and {b} came within {d:.2f} m", d)
                elif d > self.near_miss_d + 0.35:
                    self.near_pairs[key] = False

    # ------------------------------------------------------------------ items
    def set_entity_pose(self, name: str, x: float, y: float, z: float, yaw: float, timeout: float = 5.0) -> bool:
        if not self.setpose_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("set_pose service not available")
            return False
        req = SetEntityPose.Request()
        req.entity.name = name
        req.entity.type = Entity.MODEL
        req.pose.position.x, req.pose.position.y, req.pose.position.z = float(x), float(y), float(z)
        req.pose.orientation.z = math.sin(yaw / 2.0)
        req.pose.orientation.w = math.cos(yaw / 2.0)
        fut = self.setpose_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.01)
        return bool(fut.done() and fut.result() and fut.result().success)

    def restock_item(self, iid: str) -> None:
        it = L.ITEMS[iid]
        if self.set_entity_pose(iid, it.x, it.y, it.z + 0.01, 0.0):
            self.emit("restock", "info", "", f"{iid} restocked on shelf {it.shelf_id}")

    def watchdog(self) -> None:
        if self.resetting or self.paused:
            return
        with self.lock:
            due = [i for i, t in self.restock_queue.items() if t <= self.sim_time]
            for i in due:
                del self.restock_queue[i]
            poses = dict(self.poses)
            active = dict(self.active_items)
        for iid in due:                            # the agent publishes /warehouse/restock only when it is done
            self.restock_item(iid)                 # with the item (delivered or failed), so it is always safe
        for iid, it in L.ITEMS.items():
            p = poses.get(iid)
            if not p or iid in active or iid in self.restock_queue:
                continue
            if math.hypot(p[0] - it.x, p[1] - it.y) > 0.12 or p[2] < it.z - 0.05 or p[2] > it.z + 0.08:
                # displaced and not in use: only fix it when no robot is close (avoid fighting an arm)
                near = any(self.robot_xy(r) and math.hypot(self.robot_xy(r)[0] - p[0],
                                                           self.robot_xy(r)[1] - p[1]) < 0.9
                           for r in self.robots)
                on_floor_or_deck = p[2] < 0.35          # not on a shelf / table / in a raised gripper
                if not near or on_floor_or_deck:
                    self.get_logger().info(f"watchdog: putting {iid} back on its shelf")
                    self.restock_item(iid)

    # ------------------------------------------------------------------ commands
    def world_control(self, **kwargs) -> bool:
        if not self.control_cli.wait_for_service(timeout_sec=2.0):
            return False
        req = ControlWorld.Request()
        for k, v in kwargs.items():
            setattr(req.world_control, k, v)
        fut = self.control_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            time.sleep(0.01)
        return bool(fut.done() and fut.result() and fut.result().success)

    def on_command(self, req: SimCommand.Request, res: SimCommand.Response) -> SimCommand.Response:
        cmd = req.command.strip().lower()
        self.get_logger().info(f"sim command: {cmd}")
        ok, msg = True, "ok"
        if cmd in ("start", "resume"):
            if self.paused:
                ok = self.world_control(pause=False)
                self.paused = not ok
            self.state = "RUNNING"
            self.run_pub.publish(Bool(data=True))
            self.emit("sim", "info", "", f"simulation {cmd}")
        elif cmd == "pause":
            ok = self.world_control(pause=True)
            self.paused = ok
            if ok:
                self.state = "PAUSED"
                self.run_pub.publish(Bool(data=False))
                self.emit("sim", "info", "", "simulation paused")
        elif cmd == "estop":
            self.estop = True
            self.estop_pub.publish(Bool(data=True))
            self.emit("sim", "error", "", "EMERGENCY STOP engaged")
        elif cmd == "release_estop":
            self.estop = False
            self.estop_pub.publish(Bool(data=False))
            self.emit("sim", "info", "", "emergency stop released")
        elif cmd == "reset":
            threading.Thread(target=self.do_reset, daemon=True).start()
            msg = "reset started"
        else:
            ok, msg = False, f"unknown command '{req.command}'"
        res.success, res.message = ok, msg
        return res

    def do_reset(self) -> None:
        if self.resetting:
            return
        self.resetting = True
        self.state = "RESETTING"
        self.get_logger().info("RESET: begin")
        was_paused = self.paused
        if self.paused:
            self.world_control(pause=False)
            self.paused = False
        self.estop_pub.publish(Bool(data=True))                 # hold everything while we reset
        self.reset_pub.publish(String(data="begin"))            # nodes flush their state, arms stow
        time.sleep(6.0)                                         # arms stow (wall time; sim keeps running)
        for r in self.robots:
            x, y, yaw = L.bay_pose(r)
            self.set_entity_pose(r, x, y, 0.02, yaw)
        for iid, it in L.ITEMS.items():
            self.set_entity_pose(iid, it.x, it.y, it.z + 0.01, 0.0)
        time.sleep(1.5)
        for r in self.robots:                                    # re-localise AMCL at the bay
            x, y, yaw = L.bay_pose(r)
            m = PoseWithCovarianceStamped()
            m.header.frame_id = "map"
            m.pose.pose.position.x, m.pose.pose.position.y = x, y
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.pose.covariance[0] = m.pose.covariance[7] = 0.02
            m.pose.covariance[35] = 0.02
            for _ in range(3):
                self.initpose_pubs[r].publish(m)
                time.sleep(0.2)
        time.sleep(2.0)
        self.estop = False
        self.estop_pub.publish(Bool(data=False))
        self.reset_pub.publish(String(data="done"))
        self.state = "RUNNING"
        self.run_pub.publish(Bool(data=True))
        self.resetting = False
        self.emit("sim", "info", "", "simulation reset complete")
        self.get_logger().info("RESET: done")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimManager()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
