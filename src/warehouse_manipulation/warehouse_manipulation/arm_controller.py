#!/usr/bin/env python3
"""Arm controller / pick-and-place action server (one instance per robot, in the robot namespace).

Action  ``pick_place`` (warehouse_robot_interfaces/PickPlace)
  pick : stow -> raised "safe" pose -> pre-grasp -> straight-line approach -> close gripper ->
         verify grasp (finger gap) -> lift -> straight-line retract -> stow -> verify carry
         (finger gap + item lifted off the shelf).   Up to ``max_attempts`` tries.
  place: stow -> safe pose -> pre-place -> straight-line approach -> lower -> open gripper ->
         retract -> stow -> verify (item resting on the packing table, gripper open)

Grasping is real contact physics in Gazebo: two friction fingers close on the 6 cm item, nothing is
attached or teleported.  Inverse kinematics is analytic (warehouse_common.kinematics).

The node publishes ``manipulating`` (Bool) while the arm is working; the cmd_vel gate uses it to keep the
mobile base locked.  Fine alignment uses the simulator's ground-truth poses of the robot and of the
target item (equivalent to a perfect object-pose estimate from the camera), navigation itself uses AMCL.
"""
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String

from warehouse_common import kinematics as K
from warehouse_common import layout as L
from warehouse_robot_interfaces.action import PickPlace
from warehouse_robot_interfaces.msg import EntityPoses, WarehouseEvent

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
GRIP_OPEN = 0.065
GRIP_CLOSED = 0.0
ITEM_WIDTH = L.ITEM_W


class Abort(Exception):
    pass


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ArmController(Node):
    def __init__(self) -> None:
        super().__init__("arm_controller")
        self.robot = self.get_namespace().strip("/") or "robot1"
        self.declare_parameter("max_attempts", 3)
        self.declare_parameter("speed_scale", 1.0)       # >1 = slower
        self.max_attempts = int(self.get_parameter("max_attempts").value)
        self.speed = float(self.get_parameter("speed_scale").value)
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.RLock()

        self.joint_pos: Dict[str, float] = {}
        self.gt: Dict[str, Tuple[float, float, float, float]] = {}      # name -> x y z yaw
        self.estop = False
        self.reset_requested = False
        self.busy = False
        self.cmd_q: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0)
        self.grip_cmd = GRIP_OPEN
        self.holding_item = ""
        self.status = "UNKNOWN"
        self.initialised = False

        self.jpubs = [self.create_publisher(Float64, f"arm/joint{i}/cmd_pos", 10) for i in range(1, 5)]
        self.gl_pub = self.create_publisher(Float64, "gripper/left/cmd_pos", 10)
        self.gr_pub = self.create_publisher(Float64, "gripper/right/cmd_pos", 10)
        self.manip_pub = self.create_publisher(Bool, "manipulating", LATCHED)
        self.status_pub = self.create_publisher(String, "arm_status", LATCHED)
        self.grip_status_pub = self.create_publisher(String, "gripper_status", LATCHED)
        self.event_pub = self.create_publisher(WarehouseEvent, "/warehouse/events", 20)

        self.create_subscription(JointState, "joint_states", self.on_joints, 10, callback_group=self.cb)
        self.create_subscription(EntityPoses, "/warehouse/ground_truth", self.on_gt, 10, callback_group=self.cb)
        self.create_subscription(Bool, "/warehouse/estop", self.on_estop, LATCHED, callback_group=self.cb)
        self.create_subscription(String, "/warehouse/reset", self.on_reset, 10, callback_group=self.cb)

        self.server = ActionServer(self, PickPlace, "pick_place", execute_callback=self.execute,
                                   goal_callback=self.on_goal, cancel_callback=lambda g: CancelResponse.ACCEPT,
                                   callback_group=self.cb)
        self.create_timer(0.5, self.housekeeping, callback_group=self.cb)
        self.manip_pub.publish(Bool(data=False))
        self.get_logger().info(f"arm_controller for {self.robot} ready")

    # ------------------------------------------------------------------ subscriptions
    def on_joints(self, msg: JointState) -> None:
        with self.lock:
            for n, p in zip(msg.name, msg.position):
                self.joint_pos[n] = p

    def on_gt(self, msg: EntityPoses) -> None:
        with self.lock:
            for n, p in zip(msg.names, msg.poses):
                self.gt[n] = (p.position.x, p.position.y, p.position.z, yaw_of(p.orientation))

    def on_estop(self, msg: Bool) -> None:
        self.estop = msg.data

    def on_reset(self, msg: String) -> None:
        if msg.data == "begin":
            self.reset_requested = True
            self.holding_item = ""
            if not self.busy:
                threading.Thread(target=self.stow_now, daemon=True).start()
        elif msg.data == "done":
            self.reset_requested = False

    def on_goal(self, goal) -> GoalResponse:
        if self.busy or self.estop:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ------------------------------------------------------------------ helpers
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep_sim(self, secs: float) -> None:
        """Sleep in simulation time (freezes while estopped or the simulation is paused)."""
        end = self.now() + secs / max(1e-3, 1.0)
        last = self.now()
        while True:
            time.sleep(0.02)
            t = self.now()
            if self.estop:
                end += t - last                     # do not count estop time
            last = t
            if t >= end:
                return
            if self.reset_requested and self.busy:
                raise Abort("reset")

    def set_status(self, s: str) -> None:
        if s != self.status:
            self.status = s
            self.status_pub.publish(String(data=s))

    def gripper_width(self) -> float:
        with self.lock:
            return self.joint_pos.get("finger_left_joint", 0.0) + self.joint_pos.get("finger_right_joint", 0.0)

    def publish_grip_status(self) -> None:
        w = self.gripper_width()
        if w > 0.10:
            s = "OPEN"
        elif ITEM_WIDTH - 0.02 <= w <= ITEM_WIDTH + 0.02:
            s = "HOLDING"
        elif w < 0.02:
            s = "EMPTY_CLOSED"
        else:
            s = "MOVING"
        self.grip_status_pub.publish(String(data=s))

    def housekeeping(self) -> None:
        self.publish_grip_status()
        if not self.initialised and len(self.joint_pos) >= 6 and self.now() > 1.0:
            self.initialised = True
            threading.Thread(target=self.stow_now, daemon=True).start()
        # keep the commands alive (position controllers hold the last command, this is a safety refresh)
        if self.initialised and not self.busy:
            self.send_q(self.cmd_q)
            self.send_grip(self.grip_cmd)

    def current_q(self) -> Tuple[float, ...]:
        with self.lock:
            return tuple(self.joint_pos.get(f"arm_joint{i}", 0.0) for i in range(1, 5))

    def send_q(self, q) -> None:
        self.cmd_q = tuple(q)
        for pub, v in zip(self.jpubs, q):
            pub.publish(Float64(data=float(v)))

    def send_grip(self, v: float) -> None:
        self.grip_cmd = v
        self.gl_pub.publish(Float64(data=float(v)))
        self.gr_pub.publish(Float64(data=float(v)))

    def move_joints(self, q_to, duration: float) -> None:
        q_from = self.cmd_q if self.initialised else self.current_q()
        duration *= self.speed
        t0, elapsed, last = self.now(), 0.0, self.now()
        while elapsed < duration:
            time.sleep(0.04)
            t = self.now()
            if not self.estop:
                # time dilation: if this thread was starved of CPU (heavily loaded machine) the trajectory must NOT
                # jump ahead - a jump would make the arm sweep through the item - it just continues more slowly
                elapsed += min(t - last, 0.06)
            last = t
            f = min(1.0, elapsed / duration)
            f = f * f * (3 - 2 * f)                         # smoothstep
            self.send_q(tuple(a + (b - a) * f for a, b in zip(q_from, q_to)))
            if self.reset_requested and self.busy:
                raise Abort("reset")
        self.send_q(q_to)

    def move_cartesian(self, p_from, p_to, duration: float, pitch: float = math.pi / 2) -> None:
        n = max(2, int(duration * 5))
        qs = []
        for i in range(1, n + 1):
            f = i / n
            p = tuple(a + (b - a) * f for a, b in zip(p_from, p_to))
            q = K.inverse(p, pitch)
            if q is None:
                raise RuntimeError(f"IK failed at {p}")
            qs.append(q)
        for q in qs:
            self.move_joints(q, duration / n)

    def stow_now(self) -> None:
        try:
            self.set_status("MOVING")
            self.move_joints(K.STOW, 4.0)
            if self.holding_item == "":
                self.send_grip(GRIP_OPEN)
            self.sleep_sim(1.0)
            self.set_status("HOLDING" if self.holding_item else "STOWED")
        except Abort:
            pass

    def feedback(self, goal_handle, phase: str, progress: float) -> None:
        fb = PickPlace.Feedback()
        fb.phase, fb.progress = phase, float(progress)
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f"[{self.robot}] {phase}")

    # ------------------------------------------------------------------ geometry from ground truth
    def robot_pose(self) -> Optional[Tuple[float, float, float]]:
        with self.lock:
            p = self.gt.get(self.robot)
        return (p[0], p[1], p[3]) if p else None

    def world_to_base(self, pw: Tuple[float, float, float], rp) -> Tuple[float, float, float]:
        c, s = math.cos(-rp[2]), math.sin(-rp[2])
        rx, ry = pw[0] - rp[0], pw[1] - rp[1]
        return (c * rx - s * ry, s * rx + c * ry, pw[2])

    def item_world(self, item_id: str) -> Optional[Tuple[float, float, float]]:
        with self.lock:
            p = self.gt.get(item_id)
        if p:
            return (p[0], p[1], p[2])
        it = L.ITEMS[item_id]
        return (it.x, it.y, it.z)

    # ------------------------------------------------------------------ action
    def execute(self, goal_handle):
        goal = goal_handle.request
        result = PickPlace.Result()
        self.busy = True
        self.manip_pub.publish(Bool(data=True))
        try:
            self.sleep_sim(0.8)                     # let the base come to a halt
            if goal.mode == "pick":
                ok, msg = self.do_pick(goal_handle, goal.item_id)
            elif goal.mode == "place":
                ok, msg = self.do_place(goal_handle, goal.item_id, goal.station_id)
            else:
                ok, msg = False, f"unknown mode {goal.mode}"
        except Abort as e:
            ok, msg = False, f"aborted ({e})"
        except Exception as e:                       # noqa: BLE001
            self.get_logger().error(f"manipulation error: {e!r}")
            ok, msg = False, f"error: {e!r}"
        if goal_handle.is_cancel_requested:
            ok, msg = False, "cancelled"
        try:
            self.move_joints(K.STOW, 3.0)
            self.sleep_sim(0.8)
        except Abort:
            pass
        self.set_status("HOLDING" if self.holding_item else ("STOWED" if ok else "FAULT"))
        result.success, result.message, result.gripper_width = ok, msg, float(self.gripper_width())
        self.busy = False
        self.manip_pub.publish(Bool(data=False))
        (goal_handle.succeed if ok else goal_handle.abort)()
        return result

    def check_cancel(self, gh) -> None:
        if gh.is_cancel_requested:
            raise Abort("cancelled")

    # ------------------------------------------------------------------ pick
    def do_pick(self, gh, item_id: str) -> Tuple[bool, str]:
        self.set_status("PICKING")
        for attempt in range(1, self.max_attempts + 1):
            self.check_cancel(gh)
            rp = self.robot_pose()
            if rp is None:
                return False, "no robot ground-truth pose"
            iw = self.item_world(item_id)
            home = L.ITEMS[item_id]
            if abs(iw[2] - home.z) > 0.05 or math.hypot(iw[0] - home.x, iw[1] - home.y) > 0.20:
                return False, (f"ITEM_DISPLACED: {item_id} is at ({iw[0]:.2f},{iw[1]:.2f},{iw[2]:.2f}), "
                               "not on its shelf slot")
            tgt = self.world_to_base(iw, rp)
            self.feedback(gh, f"attempt {attempt}: target in base frame ({tgt[0]:.2f},{tgt[1]:.2f},{tgt[2]:.2f})", 0.05)
            if not (0.42 <= tgt[0] <= 0.60) or abs(tgt[1]) > 0.07:
                return False, (f"MISALIGNED: item at ({tgt[0]:.2f},{tgt[1]:.2f}) in the base frame, "
                               f"expected (0.51, 0.00)")
            if K.inverse(tgt) is None:
                return False, f"item unreachable from here (base frame {tgt[0]:.2f},{tgt[1]:.2f},{tgt[2]:.2f})"
            safe = (tgt[0] - 0.20, tgt[1] * 0.5, tgt[2] + 0.16)
            pre = (tgt[0] - 0.10, tgt[1], tgt[2])
            q_safe = K.inverse(safe)
            if q_safe is None:
                return False, "safe pose unreachable"
            self.send_grip(GRIP_OPEN)
            self.feedback(gh, "raising arm", 0.15)
            self.move_joints(q_safe, 3.0)
            self.feedback(gh, "pre-grasp", 0.3)
            self.move_cartesian(safe, pre, 1.6)
            self.feedback(gh, "approach", 0.45)
            self.move_cartesian(pre, tgt, 1.8)
            self.sleep_sim(0.5)
            self.feedback(gh, "closing gripper", 0.55)
            self.send_grip(GRIP_CLOSED)
            self.sleep_sim(1.5)
            w_prev = -1.0
            for _ in range(8):                       # wait until the finger gap stops changing
                w = self.gripper_width()
                if abs(w - w_prev) < 0.0004:
                    break
                w_prev = w
                self.sleep_sim(0.4)
            w = self.gripper_width()
            if not (ITEM_WIDTH - 0.02 <= w <= ITEM_WIDTH + 0.02):
                self.get_logger().warn(f"[{self.robot}] grasp miss: gripper width {w:.3f}")
                self.send_grip(GRIP_OPEN)
                self.sleep_sim(1.0)
                self.move_cartesian(tgt, pre, 1.2)
                self.move_joints(q_safe, 2.0)
                self.event("pickup_failure", "warn", f"grasp miss attempt {attempt} (width {w:.3f})", item_id)
                continue
            self.feedback(gh, "lifting", 0.7)
            lift = (tgt[0], tgt[1], tgt[2] + 0.06)
            back = (tgt[0] - 0.14, tgt[1], tgt[2] + 0.06)
            self.move_cartesian(tgt, lift, 1.2)
            self.feedback(gh, "retracting", 0.8)
            self.move_cartesian(lift, back, 2.4)
            w = self.gripper_width()
            if not (ITEM_WIDTH - 0.02 <= w <= ITEM_WIDTH + 0.02):
                self.event("pickup_failure", "warn", f"item slipped while retracting (width {w:.3f})", item_id)
                self.send_grip(GRIP_OPEN)
                self.sleep_sim(1.0)
                self.move_joints(q_safe, 2.5)
                continue
            self.holding_item = item_id
            self.feedback(gh, "stowing with item", 0.9)
            self.move_joints(K.STOW, 4.5)
            self.sleep_sim(1.0)
            ok, why = self.verify_carry(item_id)
            if ok:
                self.set_status("HOLDING")
                return True, "item picked"
            self.get_logger().warn(f"[{self.robot}] pick verification failed: {why}")
            self.event("pickup_failure", "warn", f"verification failed attempt {attempt}: {why}", item_id)
            self.holding_item = ""
            self.send_grip(GRIP_OPEN)
            self.sleep_sim(1.0)
        return False, f"pick failed after {self.max_attempts} attempts"

    def verify_carry(self, item_id: str) -> Tuple[bool, str]:
        w = self.gripper_width()
        if not (ITEM_WIDTH - 0.02 <= w <= ITEM_WIDTH + 0.02):
            return False, f"gripper width {w:.3f} does not match item"
        rp = self.robot_pose()
        iw = self.item_world(item_id)
        if rp is None or iw is None:
            return False, "no ground truth"
        b = self.world_to_base(iw, rp)
        grip = K.forward(self.current_q())
        d = math.dist(b, grip)
        # the hand may sag a couple of cm in the fingers; below ~0.33 m the item is on the deck/floor
        if iw[2] < L.SHELF_TOP_Z - 0.07 or d > 0.10:
            return False, f"item not in hand (z={iw[2]:.2f}, {d:.2f} m from gripper)"
        return True, "ok"

    # ------------------------------------------------------------------ place
    def do_place(self, gh, item_id: str, station_id: str) -> Tuple[bool, str]:
        self.set_status("PLACING")
        rp = self.robot_pose()
        if rp is None:
            return False, "no robot ground-truth pose"
        ok, why = self.verify_carry(item_id)
        if not ok:
            self.event("delivery_failure", "warn", f"item lost before placing: {why}", item_id)
            return False, f"ITEM_LOST: {why}"
        px, py, pz = L.station_place_point(station_id)
        # where is the item relative to the gripper centre?  shift the target so the *item* lands on the spot
        iw = self.item_world(item_id)
        grip_now = K.forward(self.current_q())
        off = (0.0, 0.0, 0.0)
        if iw is not None:
            ib = self.world_to_base(iw, rp)
            off = (ib[0] - grip_now[0], ib[1] - grip_now[1], ib[2] - grip_now[2])
            off = tuple(max(-0.05, min(0.05, v)) for v in off)
        tgt_item = self.world_to_base((px, py, pz + 0.015), rp)
        tgt = (tgt_item[0] - off[0], tgt_item[1] - off[1], tgt_item[2] - off[2])
        safe = (tgt[0] - 0.20, tgt[1] * 0.5, tgt[2] + 0.18)
        pre = (tgt[0] - 0.10, tgt[1], tgt[2] + 0.04)
        q_safe, q_tgt = K.inverse(safe), K.inverse(tgt)
        if q_safe is None or q_tgt is None or K.inverse(pre) is None:
            return False, f"station unreachable (target {tgt[0]:.2f},{tgt[1]:.2f},{tgt[2]:.2f})"
        self.feedback(gh, "raising arm", 0.1)
        self.move_joints(q_safe, 3.0)
        w = self.gripper_width()
        if w < ITEM_WIDTH - 0.025:
            return False, f"ITEM_LOST: gripper closed to {w:.3f} while raising the arm"
        self.feedback(gh, "pre-place", 0.3)
        self.move_cartesian(safe, pre, 1.6)
        self.feedback(gh, "approach", 0.5)
        self.move_cartesian(pre, tgt, 1.8)
        self.sleep_sim(0.6)
        self.feedback(gh, "releasing", 0.7)
        self.send_grip(GRIP_OPEN)
        self.sleep_sim(2.0)
        self.holding_item = ""
        self.feedback(gh, "retracting", 0.8)
        back = (tgt[0] - 0.14, tgt[1], tgt[2] + 0.05)
        self.move_cartesian(tgt, back, 1.6)
        self.move_joints(K.STOW, 3.0)
        self.sleep_sim(1.2)
        ok, why = self.verify_placed(item_id, station_id)
        if ok:
            return True, "item delivered"
        self.event("delivery_failure", "warn", f"delivery verification failed: {why}", item_id)
        return False, why

    def verify_placed(self, item_id: str, station_id: str) -> Tuple[bool, str]:
        st = L.STATIONS[station_id]
        iw = self.item_world(item_id)
        if iw is None:
            return False, "no ground truth for item"
        on_table = (abs(iw[0] - st.x) <= L.TABLE_WIDTH / 2 + 0.02 and st.table_y0 <= iw[1] <= st.table_y1 + 0.02
                    and L.SHELF_TOP_Z - 0.01 <= iw[2] <= L.SHELF_TOP_Z + 0.10)
        if not on_table:
            return False, f"item not on table {station_id} (at {iw[0]:.2f},{iw[1]:.2f},{iw[2]:.2f})"
        if self.gripper_width() < 0.10:
            return False, "gripper did not open"
        return True, "ok"

    def event(self, category: str, severity: str, detail: str, item_id: str = "") -> None:
        e = WarehouseEvent()
        e.stamp = self.get_clock().now().to_msg()
        e.category, e.severity, e.robot_id, e.detail = category, severity, self.robot, detail
        self.event_pub.publish(e)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmController()
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
