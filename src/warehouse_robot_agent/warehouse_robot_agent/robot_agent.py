#!/usr/bin/env python3
"""Robot agent: the state machine of one mobile manipulator (runs in the robot's namespace).

For every order the agent
   1. requests the aisle zones it needs from the traffic manager (atomic, queued, waits at its bay)
   2. drives to the shelf pick pose with Nav2 (with navigation recovery)
   3. aligns, locks the base and picks the item with the arm (physics grasp)
   4. leaves the aisle (zones are released progressively as it clears them)
   5. reserves a packing station, drives to it, places the item, verifies the delivery
   6. only then reports the order COMPLETED and returns to its bay

States: IDLE ASSIGNED NAVIGATING_TO_SHELF WAITING_FOR_AISLE POSITIONING_FOR_PICKUP PICKING
        NAVIGATING_TO_PACKING WAITING_FOR_PACKING DROPPING COMPLETED RECOVERING FAULTED
Transitions are validated against ALLOWED; an illegal transition is a bug and puts the robot in FAULTED.
"""
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros
from rclpy.time import Time
from std_msgs.msg import Bool, String

from ament_index_python.packages import get_package_share_directory

from warehouse_common import layout as L
from warehouse_robot_interfaces.action import PickPlace
from warehouse_robot_interfaces.msg import (EntityPoses, Order, OrderUpdate, RobotStatus, TrafficCommand,
                                            TrafficState, WarehouseEvent)
from warehouse_robot_interfaces.srv import AssignOrder, ReleaseResources, RequestResources

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

IDLE, ASSIGNED = "IDLE", "ASSIGNED"
NAV_SHELF, WAIT_AISLE = "NAVIGATING_TO_SHELF", "WAITING_FOR_AISLE"
POSITIONING, PICKING = "POSITIONING_FOR_PICKUP", "PICKING"
NAV_PACK, WAIT_PACK, DROPPING = "NAVIGATING_TO_PACKING", "WAITING_FOR_PACKING", "DROPPING"
COMPLETED, RECOVERING, FAULTED = "COMPLETED", "RECOVERING", "FAULTED"

ALLOWED = {
    IDLE: {ASSIGNED, FAULTED},
    ASSIGNED: {WAIT_AISLE, NAV_SHELF, IDLE, RECOVERING, FAULTED},
    WAIT_AISLE: {NAV_SHELF, ASSIGNED, RECOVERING, IDLE, FAULTED},
    NAV_SHELF: {POSITIONING, WAIT_AISLE, RECOVERING, IDLE, FAULTED},
    POSITIONING: {PICKING, NAV_SHELF, RECOVERING, IDLE, FAULTED},
    PICKING: {NAV_PACK, RECOVERING, IDLE, FAULTED},
    NAV_PACK: {WAIT_PACK, DROPPING, RECOVERING, IDLE, FAULTED},
    WAIT_PACK: {NAV_PACK, DROPPING, RECOVERING, IDLE, FAULTED},
    DROPPING: {COMPLETED, RECOVERING, IDLE, FAULTED},
    COMPLETED: {IDLE, ASSIGNED, FAULTED},
    RECOVERING: {IDLE, ASSIGNED, NAV_SHELF, NAV_PACK, WAIT_AISLE, WAIT_PACK, POSITIONING, PICKING, DROPPING,
                 FAULTED},
    FAULTED: {IDLE},
}


class OrderAbort(Exception):
    """The current order must be dropped (reset, revoked reservation ...)."""


class TaskFailed(Exception):
    def __init__(self, kind: str, detail: str) -> None:
        super().__init__(detail)
        self.kind, self.detail = kind, detail


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RobotAgent(Node):
    def __init__(self) -> None:
        super().__init__("robot_agent")
        self.robot = self.get_namespace().strip("/") or "robot1"
        for name, default in (("max_queue", 2), ("nav_attempts", 4), ("stall_timeout", 30.0),
                              ("nav_timeout", 150.0), ("aisle_swap_after", 45.0),
                              ("station_switch_after", 30.0), ("fault_cooldown", 40.0),
                              ("max_order_attempts", 3)):
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value          # noqa: E731
        self.max_queue, self.nav_attempts = int(g("max_queue")), int(g("nav_attempts"))
        self.stall_timeout, self.nav_timeout = float(g("stall_timeout")), float(g("nav_timeout"))
        self.aisle_swap_after, self.station_switch_after = float(g("aisle_swap_after")), float(g("station_switch_after"))
        self.fault_cooldown = float(g("fault_cooldown"))
        self.bt_xml = ""
        try:
            self.bt_xml = get_package_share_directory("warehouse_navigation") + "/behavior_trees/warehouse_nav_to_pose.xml"
        except Exception:                                    # noqa: BLE001
            pass

        self.cb = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.state = IDLE
        self.queue: List[Order] = []
        self.order: Optional[Order] = None
        self.estop = False
        self.reset_flag = False
        self.revoked: Optional[List[str]] = None
        self.force_recover = False
        self.pose: Optional[Tuple[float, float, float]] = None
        self.last_pose_xy: Optional[Tuple[float, float]] = None
        self.speed = 0.0
        self.prev_xy_time: Tuple[Tuple[float, float], float] = ((0.0, 0.0), 0.0)
        self.distance = 0.0
        self.battery = 100.0
        self.tasks_completed = 0
        self.tasks_failed = 0
        self.healthy = True
        self.nav_status = "IDLE"
        self.arm_status = "UNKNOWN"
        self.gripper_status = "UNKNOWN"
        self.goal: Optional[Tuple[float, float, float]] = None
        self.destination = ""
        self.waiting_for = ""
        self.path_xy: List[float] = []
        self.exiting = False
        self.held: List[str] = []
        self.station_to_release: Optional[Tuple[str, Tuple[float, float]]] = None
        self.traffic: Optional[TrafficState] = None
        self.current_nav_handle = None
        self.current_pp_handle = None
        self.cur_station = ""

        # ------------------------------------------------------------ interfaces
        self.status_pub = self.create_publisher(RobotStatus, "status", 10)
        self.update_pub = self.create_publisher(OrderUpdate, "/warehouse/order_update", 20)
        self.event_pub = self.create_publisher(WarehouseEvent, "/warehouse/events", 50)
        self.restock_pub = self.create_publisher(String, "/warehouse/restock", 10)
        self.align_pub = self.create_publisher(Twist, "cmd_vel_nav", 10)      # goes through the cmd_vel gate
        # localisation supervisor: AMCL on a very symmetric map can get lost (e.g. after a bump); the robot compares
        # its AMCL pose with the reference pose of the simulator (in a real warehouse: floor markers / UWB anchors)
        # and re-seeds AMCL when they disagree.
        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.loc_bad = 0
        self.loc_last_reseed = -1e9
        self.loc_reseeds = 0
        self.create_service(AssignOrder, "assign_order", self.on_assign, callback_group=self.cb)
        self.req_cli = self.create_client(RequestResources, "/traffic/request", callback_group=self.cb)
        self.rel_cli = self.create_client(ReleaseResources, "/traffic/release", callback_group=self.cb)
        self.nav_cli = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=self.cb)
        self.pp_cli = ActionClient(self, PickPlace, "pick_place", callback_group=self.cb)
        self.clear_local = self.create_client(ClearEntireCostmap, "local_costmap/clear_entirely_local_costmap",
                                              callback_group=self.cb)
        self.clear_global = self.create_client(ClearEntireCostmap, "global_costmap/clear_entirely_global_costmap",
                                               callback_group=self.cb)
        self.create_subscription(EntityPoses, "/warehouse/ground_truth", self.on_gt, 10, callback_group=self.cb)
        self.create_subscription(Bool, "/warehouse/estop", self.on_estop, LATCHED, callback_group=self.cb)
        self.create_subscription(String, "/warehouse/reset", self.on_reset, 10, callback_group=self.cb)
        self.create_subscription(TrafficCommand, "/traffic/commands", self.on_traffic_cmd, 10, callback_group=self.cb)
        self.create_subscription(TrafficState, "/traffic/state", lambda m: setattr(self, "traffic", m), 10,
                                 callback_group=self.cb)
        self.create_subscription(String, "arm_status", lambda m: setattr(self, "arm_status", m.data), LATCHED,
                                 callback_group=self.cb)
        self.create_subscription(String, "gripper_status", lambda m: setattr(self, "gripper_status", m.data),
                                 LATCHED, callback_group=self.cb)
        self.create_subscription(Path, "plan", self.on_plan, 5, callback_group=self.cb)
        self.create_timer(0.5, self.publish_status, callback_group=self.cb)
        self.create_timer(0.5, self.monitor_resources, callback_group=self.cb)
        self.create_timer(0.5, self.check_localisation, callback_group=self.cb)
        self.worker = threading.Thread(target=self.work_loop, daemon=True)
        self.worker.start()
        self.get_logger().info(f"robot_agent {self.robot} ready")

    # ================================================================== utilities
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep(self, secs: float) -> None:
        """Sleep in simulation time; raises OrderAbort on reset."""
        end = self.now() + secs
        last = self.now()
        while True:
            time.sleep(0.05)
            if self.reset_flag:
                raise OrderAbort("reset")
            t = self.now()
            if self.estop:
                end += t - last
            last = t
            if t >= end:
                return

    def set_state(self, new: str) -> None:
        with self.lock:
            if new == self.state:
                return
            if new not in ALLOWED.get(self.state, set()):
                msg = f"illegal state transition {self.state} -> {new}"
                self.get_logger().error(msg)
                raise RuntimeError(msg)
            self.get_logger().info(f"[{self.robot}] {self.state} -> {new}")
            self.state = new

    def event(self, category: str, severity: str, detail: str, value: float = 0.0, order_id: str = "") -> None:
        e = WarehouseEvent()
        e.stamp = self.get_clock().now().to_msg()
        e.category, e.severity, e.robot_id, e.detail, e.value = category, severity, self.robot, detail, float(value)
        e.order_id = order_id or (self.order.order_id if self.order else "")
        self.event_pub.publish(e)

    def update_order(self, status: str, reason: str = "", order_id: str = "") -> None:
        u = OrderUpdate()
        u.order_id = order_id or (self.order.order_id if self.order else "")
        u.status, u.robot_id, u.reason = status, self.robot, reason
        self.update_pub.publish(u)

    # ================================================================== subscriptions
    def on_gt(self, msg: EntityPoses) -> None:
        for n, p in zip(msg.names, msg.poses):
            if n == self.robot:
                xy = (p.position.x, p.position.y)
                t = self.now()
                if self.last_pose_xy is not None:
                    d = math.dist(xy, self.last_pose_xy)
                    if d < 1.0:
                        self.distance += d
                        self.battery = max(0.0, self.battery - 0.04 * d)
                    dt = t - self.prev_xy_time[1]
                    if dt > 0.2:
                        self.speed = math.dist(xy, self.prev_xy_time[0]) / dt
                        self.prev_xy_time = (xy, t)
                else:
                    self.prev_xy_time = (xy, t)
                self.last_pose_xy = xy
                self.pose = (xy[0], xy[1], yaw_of(p.orientation))

    def on_estop(self, msg: Bool) -> None:
        self.estop = msg.data

    def on_reset(self, msg: String) -> None:
        if msg.data == "begin":
            self.get_logger().warn(f"[{self.robot}] RESET requested")
            self.reset_flag = True
            with self.lock:
                self.queue.clear()
            for h in (self.current_nav_handle, self.current_pp_handle):
                if h is not None:
                    try:
                        h.cancel_goal_async()
                    except Exception:                       # noqa: BLE001
                        pass
        elif msg.data == "done":
            threading.Thread(target=self.finish_reset, daemon=True).start()

    def finish_reset(self) -> None:
        time.sleep(0.5)
        with self.lock:
            self.queue.clear()
            self.order = None
            self.state = IDLE
            self.nav_status = "IDLE"
            self.goal = None
            self.destination = ""
            self.waiting_for = ""
            self.held = []
            self.exiting = False
            self.station_to_release = None
            self.revoked = None
            self.healthy = True
            self.distance = 0.0
            self.battery = 100.0
            self.tasks_completed = 0
            self.tasks_failed = 0
        self.reset_flag = False

    def on_traffic_cmd(self, msg: TrafficCommand) -> None:
        if msg.robot_id != self.robot:
            return
        if msg.command == "REVOKE":
            self.revoked = list(msg.resources)
            self.get_logger().warn(f"[{self.robot}] reservation revoked: {msg.resources} ({msg.reason})")
        elif msg.command == "RECOVER":
            self.force_recover = True

    def on_plan(self, msg: Path) -> None:
        pts = msg.poses[::max(1, len(msg.poses) // 25)]
        self.path_xy = [v for p in pts for v in (p.pose.position.x, p.pose.position.y)]

    # ================================================================== assignment
    def on_assign(self, req: AssignOrder.Request, res: AssignOrder.Response) -> AssignOrder.Response:
        o = req.order
        with self.lock:
            active = 1 if self.order is not None else 0
            if not self.healthy or self.state == FAULTED:
                res.accepted, res.reason = False, "robot faulted"
            elif self.reset_flag:
                res.accepted, res.reason = False, "resetting"
            elif any(q.order_id == o.order_id for q in self.queue) or (self.order and self.order.order_id == o.order_id):
                res.accepted, res.reason = False, "duplicate"
            elif len(self.queue) + active >= self.max_queue:
                res.accepted, res.reason = False, "queue full"
            else:
                self.queue.append(o)
                res.accepted, res.reason = True, "ok"
        if res.accepted:
            self.event("order_assigned", "info", f"{o.order_id} -> {self.robot} ({o.item_name}, shelf {o.shelf_id})",
                       order_id=o.order_id)
        return res

    # ================================================================== status
    def publish_status(self) -> None:
        m = RobotStatus()
        m.stamp = self.get_clock().now().to_msg()
        m.robot_id, m.state = self.robot, self.state
        m.order_id = self.order.order_id if self.order else ""
        m.item_id = self.order.item_id if self.order else ""
        m.nav_status, m.arm_status, m.gripper_status = self.nav_status, self.arm_status, self.gripper_status
        m.destination = self.destination
        m.has_goal = self.goal is not None
        if self.goal:
            m.goal_x, m.goal_y, m.goal_yaw = self.goal
        if self.pose:
            m.x, m.y, m.yaw = self.pose
        m.speed = float(self.speed)
        # battery: drains slowly with work, recharges while parked at the bay
        if self.state == IDLE and self.pose and math.dist(self.pose[:2], L.bay_pose(self.robot)[:2]) < 0.6:
            self.battery = min(100.0, self.battery + 0.25)
        elif self.state != IDLE:
            self.battery = max(0.0, self.battery - 0.005)
        m.battery = float(self.battery)
        m.distance_travelled = float(self.distance)
        m.tasks_completed, m.tasks_failed = self.tasks_completed, self.tasks_failed
        with self.lock:
            m.queue_length = len(self.queue) + (1 if self.order else 0)
            m.queued_orders = [q.order_id for q in self.queue]
        m.held_resources = list(self.held)
        m.waiting_for = self.waiting_for
        m.healthy = self.healthy
        m.path_xy = [float(v) for v in self.path_xy] if self.nav_status == "NAVIGATING" else []
        self.status_pub.publish(m)

    # ================================================================== resource helpers
    def call(self, client, request, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=3.0):
            return None
        fut = client.call_async(request)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.01)
        return fut.result() if fut.done() else None

    def request_resources(self, resources: List[str], priority: float) -> Tuple[str, int]:
        r = RequestResources.Request()
        r.robot_id, r.resources, r.priority = self.robot, resources, float(priority)
        r.order_id = self.order.order_id if self.order else ""
        res = self.call(self.req_cli, r)
        if res is None:
            return "ERROR", 0
        return res.status, int(res.queue_position)

    def release_resources(self, resources: Optional[List[str]]) -> None:
        r = ReleaseResources.Request()
        r.robot_id, r.resources = self.robot, resources or []
        self.call(self.rel_cli, r)
        with self.lock:
            if not resources:
                self.held = []
            else:
                self.held = [h for h in self.held if h not in resources]

    def acquire(self, resources: List[str], priority: float, waiting_state: str, what: str,
                alternative=None) -> List[str]:
        """Block until all resources are granted.  Waits in ``waiting_state``."""
        t_start = self.now()
        waiting = False
        while True:
            if self.reset_flag:
                raise OrderAbort("reset")
            status, pos = self.request_resources(resources, priority)
            if status == "GRANTED":
                with self.lock:
                    self.held = sorted(set(self.held) | set(resources))
                    self.waiting_for = ""
                if waiting:
                    self.event("reservation", "info", f"{self.robot} granted {','.join(resources)} after "
                               f"{self.now() - t_start:.0f}s wait", self.now() - t_start)
                return resources
            if status == "DENIED":
                raise TaskFailed("reservation", f"reservation denied for {resources}")
            if not waiting:
                waiting = True
                self.set_state(waiting_state)
                self.nav_status = "WAITING"
            self.waiting_for = f"{what} {','.join(resources)} (#{pos})"
            if alternative is not None:
                alt = alternative(self.now() - t_start)
                if alt:
                    return alt
            if self.estop:
                pass
            self.sleep(1.0)

    # ================================================================== navigation
    def navigate(self, x: float, y: float, yaw: float, label: str, interruptible=None) -> str:
        """Send a Nav2 goal and block.  Returns SUCCEEDED | FAILED | STALLED | TIMEOUT | CANCELED."""
        self.goal, self.destination = (x, y, yaw), label
        if not self.nav_cli.wait_for_server(timeout_sec=20.0):
            return "FAILED"
        t_start = self.now()
        best_remaining, t_progress = float("inf"), self.now()
        while True:
            if self.reset_flag:
                raise OrderAbort("reset")
            while self.estop:                                # hold during emergency stop
                self.nav_status = "HELD"
                self.sleep(0.5)
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            goal.behavior_tree = self.bt_xml
            fut = self.nav_cli.send_goal_async(goal)
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 10:
                time.sleep(0.02)
            gh = fut.result() if fut.done() else None
            if gh is None or not gh.accepted:
                self.nav_status = "FAILED"
                return "FAILED"
            self.current_nav_handle = gh
            self.nav_status = "NAVIGATING"
            res_fut = gh.get_result_async()
            interrupted = False
            while not res_fut.done():
                time.sleep(0.1)
                if self.reset_flag:
                    gh.cancel_goal_async()
                    raise OrderAbort("reset")
                if self.estop:
                    gh.cancel_goal_async()
                    interrupted = True
                    break
                if interruptible and interruptible():
                    gh.cancel_goal_async()
                    self.current_nav_handle = None
                    self.nav_status = "IDLE"
                    return "CANCELED"
                if self.pose is not None:
                    remaining = math.dist(self.pose[:2], (x, y))
                    if remaining < best_remaining - 0.15:
                        best_remaining, t_progress = remaining, self.now()
                    if self.now() - t_progress > self.stall_timeout and self.speed < 0.05:
                        gh.cancel_goal_async()
                        self.nav_status = "STALLED"
                        self.current_nav_handle = None
                        return "STALLED"
                if self.now() - t_start > self.nav_timeout:
                    gh.cancel_goal_async()
                    self.nav_status = "TIMEOUT"
                    self.current_nav_handle = None
                    return "TIMEOUT"
            self.current_nav_handle = None
            if interrupted:
                continue                                      # re-send the goal once the e-stop is released
            status = res_fut.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.nav_status = "SUCCEEDED"
                return "SUCCEEDED"
            self.nav_status = "FAILED"
            return "FAILED" if status != GoalStatus.STATUS_CANCELED else "CANCELED"

    def clear_costmaps(self) -> None:
        for cli in (self.clear_local, self.clear_global):
            self.call(cli, ClearEntireCostmap.Request(), timeout=3.0)

    def navigate_with_recovery(self, x: float, y: float, yaw: float, label: str, resume_state: str,
                               pos_tol: float = 0.55, interruptible=None) -> None:
        """Drive to a pose, running the agent level recovery ladder if Nav2 gives up."""
        last = "FAILED"
        for attempt in range(1, self.nav_attempts + 1):
            if self.revoked is not None and resume_state in (NAV_SHELF, POSITIONING):
                raise TaskFailed("revoked", "aisle reservation revoked while navigating")
            result = self.navigate(x, y, yaw, label, interruptible)
            if result == "CANCELED" and interruptible:
                return
            if result == "SUCCEEDED" and (self.pose is None or math.dist(self.pose[:2], (x, y)) < pos_tol):
                return
            last = result if result != "SUCCEEDED" else "OFF_TARGET"
            self.event("nav_failure", "warn", f"{label}: navigation {last} (attempt {attempt}/{self.nav_attempts})",
                       attempt)
            self.set_state(RECOVERING)
            self.nav_status = "RECOVERING"
            self.get_logger().warn(f"[{self.robot}] navigation to {label} {last}: recovery {attempt}")
            self.event("recovery", "warn", f"{self.robot} navigation recovery #{attempt} ({last})", attempt)
            self.clear_costmaps()
            self.sleep(2.0 + 3.0 * attempt)                    # give a blocking robot time to move on
            self.set_state(resume_state)
        raise TaskFailed("navigation", f"navigation to {label} failed ({last})")

    # ================================================================== fine alignment (closed loop on ground truth)
    def fine_align(self, px: float, py: float, pyaw: float, pos_tol: float = 0.015, yaw_tol: float = 0.025,
                   timeout: float = 45.0, max_reverse: float = 0.30) -> bool:
        """Creep the base onto the exact pick/drop pose (Nav2's goal tolerance is 10 cm / 6 deg).
        Rotate-drive-rotate with a P controller on the simulator pose; speeds are kept small."""
        t_start = self.now()
        ok = False
        stable = 0
        while self.now() - t_start < timeout:
            if self.reset_flag:
                raise OrderAbort("reset")
            if self.estop or self.pose is None:
                self.sleep(0.2)
                continue
            x, y, yaw = self.pose
            ex, ey = px - x, py - y
            dist = math.hypot(ex, ey)
            cmd = Twist()
            if dist > pos_tol:
                hd = math.atan2(ey, ex)
                herr = L.wrap_angle(hd - yaw)
                direction = 1.0
                if abs(herr) > math.pi / 2 and dist < max_reverse:
                    herr = L.wrap_angle(herr - math.pi)
                    direction = -1.0
                if abs(herr) > 0.10:
                    cmd.angular.z = max(-0.6, min(0.6, 1.6 * herr))
                    if abs(cmd.angular.z) < 0.12:
                        cmd.angular.z = math.copysign(0.12, herr)
                else:
                    cmd.linear.x = direction * max(0.04, min(0.12, 0.8 * dist))
                    cmd.angular.z = max(-0.4, min(0.4, 1.5 * herr))
                stable = 0
            else:
                yerr = L.wrap_angle(pyaw - yaw)
                if abs(yerr) > yaw_tol:
                    cmd.angular.z = max(-0.5, min(0.5, 1.6 * yerr))
                    if abs(cmd.angular.z) < 0.10:
                        cmd.angular.z = math.copysign(0.10, yerr)
                    stable = 0
                else:
                    stable += 1
                    if stable >= 3:
                        ok = True
                        break
            self.align_pub.publish(cmd)
            time.sleep(0.1)
        self.align_pub.publish(Twist())
        self.sleep(0.3)
        if self.pose:
            self.get_logger().info(f"[{self.robot}] fine alignment {'ok' if ok else 'TIMEOUT'}: error "
                                   f"{math.dist(self.pose[:2], (px, py)):.3f} m, "
                                   f"{abs(L.wrap_angle(self.pose[2] - pyaw)):.3f} rad")
        return ok

    # ================================================================== manipulation
    def run_pick_place(self, mode: str, item_id: str, station_id: str = "") -> Tuple[bool, str]:
        if not self.pp_cli.wait_for_server(timeout_sec=20.0):
            return False, "pick_place server unavailable"
        goal = PickPlace.Goal()
        goal.mode, goal.item_id, goal.station_id = mode, item_id, station_id
        fut = self.pp_cli.send_goal_async(goal)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 10:
            time.sleep(0.02)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted:
            return False, "manipulation goal rejected"
        self.current_pp_handle = gh
        res_fut = gh.get_result_async()
        while not res_fut.done():
            time.sleep(0.1)
            if self.reset_flag:
                gh.cancel_goal_async()
                raise OrderAbort("reset")
        self.current_pp_handle = None
        r = res_fut.result().result
        return bool(r.success), r.message

    def zone_cleared(self, zid: str, margin: float = 0.42) -> bool:
        """Has the robot completely LEFT this zone?  Every exit leads south, so a zone is cleared once the robot is
        south of its southern boundary (plus a margin for the robot body).  Zones the robot has not reached yet
        (further inside the aisle, i.e. north of the robot) are NOT cleared - they must stay reserved because the
        robot still has to drive through them on its way out."""
        z = L.ZONES[zid]
        return self.pose is not None and self.pose[1] < z.y0 - margin

    def zones_occupied(self, margin: float = 0.42) -> List[str]:
        """Held zones the robot is still in, or still has to pass through."""
        return [h for h in self.held if h in L.ZONES and not self.zone_cleared(h, margin)]

    def release_clear_zones(self) -> None:
        """Release reservations the robot has physically left (never the ones it is still inside)."""
        rel = [h for h in self.held if h in L.ZONES and self.zone_cleared(h)]
        if self.station_to_release is None:
            rel += [h for h in self.held if h in L.STATIONS]
        if rel:
            self.release_resources(rel)

    def check_localisation(self) -> None:
        if self.pose is None or self.reset_flag:
            return
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except Exception:                                    # noqa: BLE001
            return
        ax, ay = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        ayaw = yaw_of(q)
        err = math.hypot(ax - self.pose[0], ay - self.pose[1])
        yerr = abs(L.wrap_angle(ayaw - self.pose[2]))
        if err > 0.35 or yerr > 0.6:
            self.loc_bad += 1
        else:
            self.loc_bad = 0
        if self.loc_bad >= 4 and self.now() - self.loc_last_reseed > 8.0:     # 2 s of disagreement
            self.loc_bad = 0
            self.loc_last_reseed = self.now()
            self.loc_reseeds += 1
            m = PoseWithCovarianceStamped()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.pose.position.x, m.pose.pose.position.y = float(self.pose[0]), float(self.pose[1])
            m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(self.pose[2] / 2), math.cos(self.pose[2] / 2)
            m.pose.covariance[0] = m.pose.covariance[7] = 0.03
            m.pose.covariance[35] = 0.03
            self.initpose_pub.publish(m)
            self.get_logger().warn(f"[{self.robot}] localisation error {err:.2f} m / {yerr:.2f} rad -> AMCL re-seeded")
            self.event("recovery", "warn", f"{self.robot} localisation lost ({err:.2f} m off) - AMCL re-seeded "
                       f"from the reference pose", err)

    # ================================================================== resource monitor (progressive release)
    def monitor_resources(self) -> None:
        if self.pose is None:
            return
        px, py = self.pose[0], self.pose[1]
        # aisle zones: release the ones the robot has completely left (passed to the south) while exiting
        if self.exiting and self.held:
            rel = [rid for rid in list(self.held) if rid in L.ZONES and self.zone_cleared(rid)]
            if rel:
                threading.Thread(target=self.release_resources, args=(rel,), daemon=True).start()
        # packing station: release once the robot drove away from the drop pose
        if self.station_to_release is not None:
            sid, xy = self.station_to_release
            if math.dist((px, py), xy) > 0.95:
                self.station_to_release = None
                threading.Thread(target=self.release_resources, args=([sid],), daemon=True).start()

    # ================================================================== main loop
    def work_loop(self) -> None:
        while rclpy.ok():
            try:
                time.sleep(0.2)
                if self.state == FAULTED:
                    self.fault_recovery()
                    continue
                if self.reset_flag or self.pose is None:
                    continue
                with self.lock:
                    nxt = self.queue.pop(0) if self.queue else None
                if nxt is None:
                    continue
                self.run_order(nxt)
            except OrderAbort:
                self.get_logger().warn(f"[{self.robot}] order aborted")
            except Exception as e:                           # noqa: BLE001
                self.get_logger().error(f"[{self.robot}] unexpected error: {e!r}")
                self.fail_hard(f"exception {e!r}")

    def fault_recovery(self) -> None:
        """Cool-down, then drive back to the bay (leaving any aisle the robot is stuck in) and return to service."""
        self.nav_status = "FAULT"
        t = self.now()
        while self.now() - t < self.fault_cooldown and not self.reset_flag:
            time.sleep(0.5)
        if self.reset_flag:
            return
        self.get_logger().info(f"[{self.robot}] fault cool-down over, driving back to the bay")
        try:
            bx, by, byaw = L.bay_pose(self.robot)
            self.exiting = True
            self.clear_costmaps()
            self.navigate(bx, by, byaw, "bay (fault recovery)")
            self.release_clear_zones()
            if not self.zones_occupied():
                self.release_resources(None)
            self.exiting = False
            self.state = IDLE
            self.healthy = True
            self.nav_status = "IDLE"
            self.event("recovery", "info", f"{self.robot} returned from FAULTED to IDLE")
        except OrderAbort:
            return
        except Exception as e:                              # noqa: BLE001
            self.get_logger().error(f"fault recovery failed: {e!r}")
            self.state = FAULTED

    def fail_hard(self, why: str) -> None:
        o = self.order
        if o is not None:
            self.update_order("PENDING", f"robot_failed:{why}")
        self.release_resources(None)
        self.order = None
        self.exiting = False
        self.healthy = False
        self.tasks_failed += 1
        self.state = FAULTED
        self.event("recovery", "error", f"{self.robot} FAULTED: {why}")

    # ------------------------------------------------------------------ one order
    def run_order(self, order: Order) -> None:
        self.order = order
        self.revoked = None
        self.exiting = False
        self.cur_station = order.station_id
        self.set_state(ASSIGNED)
        self.update_order("IN_PROGRESS")
        self.event("order_started", "info", f"{self.robot} started {order.order_id}: {order.item_name} from shelf "
                   f"{order.shelf_id} to {order.station_id}")
        held_item = False
        try:
            zones = L.zones_for_item(order.item_id)
            self.pick_phase(order, zones)
            held_item = True
            self.deliver_phase(order)
            # ---- delivered & verified
            self.set_state(COMPLETED)
            self.tasks_completed += 1
            self.update_order("COMPLETED")
            self.restock_pub.publish(String(data=order.item_id))
            self.event("order_completed", "info", f"{self.robot} delivered {order.order_id}")
            self.order = None
            self.go_home_or_next()
        except OrderAbort:
            self.order = None
            self.release_resources(None)
            self.set_state_force(IDLE)
            raise
        except TaskFailed as e:
            self.handle_failure(order, e, held_item)

    def set_state_force(self, s: str) -> None:
        with self.lock:
            self.state = s

    def pick_phase(self, order: Order, zones: List[str]) -> None:
        priority = order.deadline
        # 1. aisle access (waits at the bay / wherever the robot is)
        self.set_state(WAIT_AISLE)
        self.acquire(zones, priority, WAIT_AISLE, "aisle", alternative=self.aisle_alternative(order))
        # 2. navigate to the shelf
        self.set_state(NAV_SHELF)
        px, py, pyaw = L.item_pick_pose(order.item_id)
        self.navigate_with_recovery(px, py, pyaw, f"shelf {order.shelf_id} ({order.item_name})", NAV_SHELF)
        # 3. align + settle (closed loop on the simulator pose)
        self.set_state(POSITIONING)
        self.fine_align(px, py, pyaw)
        t0 = self.now()
        while self.speed > 0.03 and self.now() - t0 < 6.0:
            self.sleep(0.3)
        self.sleep(0.5)
        # 4. pick
        self.set_state(PICKING)
        self.nav_status = "IDLE"
        self.goal = None
        for realign in range(3):
            ok, msg = self.run_pick_place("pick", order.item_id)
            if ok or not msg.startswith("MISALIGNED"):
                break
            self.get_logger().warn(f"[{self.robot}] {msg} -> re-aligning (try {realign + 1})")
            self.fine_align(px, py, pyaw, pos_tol=0.02)
            self.sleep(0.5)
        if not ok:
            self.event("pickup_failure", "warn", f"pick failed: {msg}")
            raise TaskFailed("pickup", msg)
        # the arm reaction can creep the base towards the shelf: restore the stand-off before Nav2 takes over
        self.fine_align(px, py, pyaw, pos_tol=0.04, yaw_tol=0.06, timeout=25.0)

    def aisle_alternative(self, order: Order):
        """After a long wait try another queued order whose resources are free (avoid idling and blocking)."""
        def alt(waited: float):
            if waited < self.aisle_swap_after:
                return None
            with self.lock:
                others = [q for q in self.queue if q.order_id != order.order_id]
            if not others or self.traffic is None:
                return None
            free = {z.zone_id for z in self.traffic.zones if z.owner == "" and not z.queue}
            for q in others:
                if set(L.zones_for_item(q.item_id)) <= free:
                    with self.lock:
                        self.queue.remove(q)
                        self.queue.insert(0, q)          # the free order runs next ...
                        self.queue.insert(1, order)      # ... and this one waits behind it
                    self.release_resources(None)
                    self.event("recovery", "warn", f"{self.robot} waited {waited:.0f}s for aisle; swapped "
                               f"{order.order_id} with {q.order_id}", waited)
                    raise TaskFailed("swap", f"swapped with {q.order_id}")
            return None
        return alt

    def deliver_phase(self, order: Order) -> None:
        self.exiting = True
        station = order.station_id
        self.set_state(NAV_PACK)
        # leave the aisle first (zones are released progressively while we drive out); a robot never asks
        # for a packing station while it still owns aisle zones - that would be hold-and-wait
        bx, by, byaw = L.bay_pose(self.robot)
        priority = order.deadline
        self.navigate_with_recovery(bx, by, byaw, "bay (leaving aisle)", NAV_PACK, pos_tol=0.45)
        zones_left = [h for h in self.held if h in L.ZONES]
        if zones_left:
            self.release_resources(zones_left)
        self.exiting = False
        status, pos = self.request_resources([station], priority)
        if status != "GRANTED":
            self.set_state(WAIT_PACK)
            self.acquire([station], priority, WAIT_PACK, "station", alternative=self.station_alternative(order))
            station = self.cur_station
            self.set_state(NAV_PACK)
        else:
            with self.lock:
                self.held = sorted(set(self.held) | {station})
        self.waiting_for = ""
        self.cur_station = station
        dx, dy, dyaw = L.station_drop_pose(station)
        self.navigate_with_recovery(dx, dy, dyaw, f"packing station {station}", NAV_PACK)
        self.fine_align(dx, dy, dyaw)
        t0 = self.now()
        while self.speed > 0.03 and self.now() - t0 < 6.0:
            self.sleep(0.3)
        self.set_state(DROPPING)
        self.nav_status = "IDLE"
        self.goal = None
        ok, msg = self.run_pick_place("place", order.item_id, station)
        self.station_to_release = (station, (dx, dy))
        if ok:
            self.fine_align(dx, dy, dyaw, pos_tol=0.04, yaw_tol=0.06, timeout=25.0)
        if not ok:
            self.event("delivery_failure", "warn", f"drop failed: {msg}")
            raise TaskFailed("delivery", msg)

    def station_alternative(self, order: Order):
        def alt(waited: float):
            if waited < self.station_switch_after or self.traffic is None:
                return None
            for st in self.traffic.stations:
                if st.owner == "" and not st.queue and st.zone_id != self.cur_station:
                    self.release_resources([self.cur_station])
                    self.event("recovery", "warn", f"{self.robot} waited {waited:.0f}s for {self.cur_station}; "
                               f"switching to free station {st.zone_id}", waited)
                    self.cur_station = st.zone_id
                    r = self.request_resources([st.zone_id], order.deadline)
                    if r[0] == "GRANTED":
                        with self.lock:
                            self.held = sorted(set(self.held) | {st.zone_id})
                        return [st.zone_id]
            return None
        return alt

    def go_home_or_next(self) -> None:
        """After a delivery: leave the station and go to the bay unless the next order is already queued."""
        bx, by, byaw = L.bay_pose(self.robot)
        self.set_state_force(COMPLETED)
        def new_work() -> bool:
            with self.lock:
                return bool(self.queue)
        # always clear the station first (go to the bay), then the next order starts from there
        try:
            self.navigate_with_recovery(bx, by, byaw, "bay", COMPLETED, pos_tol=0.45)
        except TaskFailed:
            self.get_logger().warn(f"[{self.robot}] could not return to bay")
        self.station_to_release = self.station_to_release            # monitor releases it when we left
        self.set_state_force(IDLE)
        self.nav_status = "IDLE"
        self.goal, self.destination = None, ""

    # ------------------------------------------------------------------ failure handling
    def handle_failure(self, order: Order, e: TaskFailed, held_item: bool) -> None:
        self.get_logger().error(f"[{self.robot}] order {order.order_id} failed: {e.kind}: {e.detail}")
        if e.kind == "swap":
            # the swapped-in order is already first in the queue (acquire released the reservation)
            self.order = None
            self.set_state_force(IDLE)
            return
        self.tasks_failed += 1
        if e.kind == "navigation":
            self.event("nav_failure", "error", f"{self.robot} gave up on {order.order_id}: {e.detail}")
        self.set_state_force(RECOVERING)
        self.nav_status = "RECOVERING"
        self.exiting = True
        # leave the aisle / station area so nobody is blocked; reservations are only released once the robot
        # has PHYSICALLY left the zone (a robot that is stuck inside keeps the aisle reserved)
        bx, by, byaw = L.bay_pose(self.robot)
        cleared = False
        try:
            for attempt in range(3):
                if e.kind != "revoked" or attempt > 0:
                    self.navigate(bx, by, byaw, "bay (after failure)")
                self.release_clear_zones()
                if not self.zones_occupied():
                    cleared = True
                    break
                self.clear_costmaps()
                self.sleep(3.0)
        except OrderAbort:
            raise
        except Exception:                                    # noqa: BLE001
            pass
        if cleared:
            self.release_resources(None)
        self.exiting = False
        self.restock_pub.publish(String(data=order.item_id))      # put a dropped / displaced item back on its shelf
        reason = f"robot_failed:{self.robot}:{e.kind}:{e.detail}"
        # the item was put back on its shelf above, so a lost / dropped item is retried like any other failure
        if order.attempts + 1 >= int(self.get_parameter("max_order_attempts").value):
            self.update_order("FAILED", reason)
            self.event("order_failed", "error", f"{order.order_id} failed after retries: {e.detail}",
                       order_id=order.order_id)
        else:
            self.update_order("PENDING", reason)
        self.order = None
        self.set_state_force(IDLE)
        self.nav_status = "IDLE"
        if e.kind == "navigation" or not cleared:
            # a robot that repeatedly cannot navigate is taken out of service for a while
            self.healthy = False
            self.set_state_force(FAULTED)

    def arm_to_safe(self) -> None:
        pass                                                  # the arm node always stows itself after an action


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotAgent()
    ex = MultiThreadedExecutor(num_threads=6)
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
