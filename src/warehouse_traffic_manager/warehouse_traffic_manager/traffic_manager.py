#!/usr/bin/env python3
"""Traffic manager: aisle reservation, queueing, deadlock prevention and recovery.

State machine of every managed resource (aisle zone A#-S / A#-N, packing station PK#):

      FREE --request(all resources free, nobody ahead)--> OWNED(robot)
      FREE/OWNED --request--> QUEUED(robot at position n)   (atomic, strict priority order, no overtaking)
      OWNED --release / hold-timeout / robot fault--> FREE  (next queued request is granted immediately)

Services   /traffic/request   (atomic reservation, calling again refreshes a queued request)
           /traffic/release   (empty list = everything the robot holds)
Topics     /traffic/state     TrafficState (owners + queues, 2 Hz)
           /traffic/commands  TrafficCommand (REVOKE / RECOVER) to robot agents
           /warehouse/events  reservation | wait | release | recovery | deadlock_prevented

Recovery
  * request not refreshed for ``request_timeout`` s   -> dropped
  * resource held longer than ``hold_timeout`` s       -> revoked, owner told to recover
  * owner robot stalled inside its zone (not moving, not manipulating) for ``stall_timeout`` s
                                                         -> RECOVER command, then forced release
  * owner robot FAULTED / not reporting                -> reservations freed immediately
Every decision is logged as a WarehouseEvent so the dashboard shows the full history.
"""
import math
from typing import Dict

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from warehouse_common import layout as L
from warehouse_common import reservation as R
from warehouse_robot_interfaces.msg import (RobotStatus, TrafficCommand, TrafficState, WarehouseEvent, ZoneState)
from warehouse_robot_interfaces.srv import ReleaseResources, RequestResources


class TrafficManager(Node):
    def __init__(self) -> None:
        super().__init__("traffic_manager")
        for n, d in (("hold_timeout", 420.0), ("request_timeout", 8.0), ("stall_timeout", 60.0),
                     ("aging_rate", 0.5), ("status_timeout", 15.0)):
            self.declare_parameter(n, d)
        g = lambda n: float(self.get_parameter(n).value)            # noqa: E731
        self.rm = R.ReservationManager(list(L.ZONES) + list(L.STATIONS), hold_timeout=g("hold_timeout"),
                                       request_timeout=g("request_timeout"), aging_rate=g("aging_rate"))
        self.stall_timeout, self.status_timeout = g("stall_timeout"), g("status_timeout")
        self.status: Dict[str, RobotStatus] = {}
        self.status_time: Dict[str, float] = {}
        self.stalled_since: Dict[str, float] = {}
        self.recover_sent: Dict[str, float] = {}
        self.cb = ReentrantCallbackGroup()

        self.state_pub = self.create_publisher(TrafficState, "/traffic/state", 10)
        self.cmd_pub = self.create_publisher(TrafficCommand, "/traffic/commands", 10)
        self.event_pub = self.create_publisher(WarehouseEvent, "/warehouse/events", 100)
        self.create_service(RequestResources, "/traffic/request", self.on_request, callback_group=self.cb)
        self.create_service(ReleaseResources, "/traffic/release", self.on_release, callback_group=self.cb)
        for r in L.ROBOT_IDS:
            self.create_subscription(RobotStatus, f"/{r}/status", self.on_status, 10, callback_group=self.cb)
        self.create_subscription(String, "/warehouse/reset", self.on_reset, 10, callback_group=self.cb)
        self.create_timer(0.5, self.tick, callback_group=self.cb)
        self.get_logger().info("traffic_manager ready: %d zones, %d stations" % (len(L.ZONES), len(L.STATIONS)))

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ events
    def flush_events(self) -> None:
        for ev in self.rm.drain_events():
            e = WarehouseEvent()
            e.stamp = self.get_clock().now().to_msg()
            e.category = ev.category
            e.severity = "warn" if ev.category in ("wait", "recovery", "deadlock_prevented") else "info"
            e.robot_id, e.detail, e.value = ev.robot, ev.detail, float(ev.value)
            self.event_pub.publish(e)
            self.get_logger().info(f"[{ev.category}] {ev.robot}: {ev.detail}")

    # ------------------------------------------------------------------ services
    def on_request(self, req: RequestResources.Request, res: RequestResources.Response):
        status, pos, why = self.rm.request(req.robot_id, list(req.resources), req.priority, req.order_id, self.now())
        res.status, res.queue_position, res.reason = status, pos, why
        self.flush_events()
        return res

    def on_release(self, req: ReleaseResources.Request, res: ReleaseResources.Response):
        released = self.rm.release(req.robot_id, list(req.resources) or None, self.now())
        res.success, res.message = True, f"released {released}"
        self.flush_events()
        return res

    def on_reset(self, msg: String) -> None:
        if msg.data == "begin":
            self.rm = R.ReservationManager(list(L.ZONES) + list(L.STATIONS), hold_timeout=self.rm.hold_timeout,
                                           request_timeout=self.rm.request_timeout, aging_rate=self.rm.aging_rate)
            self.stalled_since.clear()

    def on_status(self, msg: RobotStatus) -> None:
        self.status[msg.robot_id] = msg
        self.status_time[msg.robot_id] = self.now()

    # ------------------------------------------------------------------ periodic work
    def command(self, robot: str, cmd: str, resources, reason: str) -> None:
        m = TrafficCommand()
        m.robot_id, m.command, m.resources, m.reason = robot, cmd, list(resources), reason
        self.cmd_pub.publish(m)

    def tick(self) -> None:
        now = self.now()
        for robot, kind, res in self.rm.tick(now):
            if kind == "revoked":
                self.command(robot, "REVOKE", res, "hold timeout")
        # robots that are faulted or silent lose their reservations - except the zones they are still physically
        # standing in (an aisle with a broken robot inside must stay closed until it is out)
        for robot in list(self.rm.held):
            st = self.status.get(robot)
            silent = now - self.status_time.get(robot, now) > self.status_timeout
            faulted = st is not None and (not st.healthy or st.state == "FAULTED")
            if not (faulted or silent):
                continue
            keep = []
            if st is not None:
                keep = [r for r in self.rm.held_by(robot)
                        if r in L.ZONES and st.y >= L.ZONES[r].y0 - 0.45]       # not yet south of the zone
            free = [r for r in self.rm.held_by(robot) if r not in keep]
            if free:
                self.rm.release(robot, free, now, reason="robot faulted" if faulted else "robot not reporting")
                self.rm.recoveries += 1
                self.command(robot, "REVOKE", free, "robot faulted" if faulted else "silent")
        # stall detection for robots that own resources
        for robot in list(self.rm.held):
            st = self.status.get(robot)
            if st is None:
                continue
            moving_state = st.state in ("NAVIGATING_TO_SHELF", "NAVIGATING_TO_PACKING", "RECOVERING")
            if moving_state and st.speed < 0.03:
                t0 = self.stalled_since.setdefault(robot, now)
                if now - t0 > self.stall_timeout:
                    if now - self.recover_sent.get(robot, -1e9) > self.stall_timeout:
                        self.recover_sent[robot] = now
                        self.rm.recoveries += 1
                        self.rm.events.append(R.Event("recovery", robot,
                                                      f"{robot} stalled {now - t0:.0f}s inside {','.join(self.rm.held_by(robot))}"
                                                      ", RECOVER commanded", tuple(self.rm.held_by(robot)), now - t0))
                        self.command(robot, "RECOVER", self.rm.held_by(robot), "stalled")
                    if now - t0 > 2.5 * self.stall_timeout:
                        released = self.rm.force_release(robot, now, "stall timeout")
                        self.command(robot, "REVOKE", released, "stalled too long")
                        self.stalled_since.pop(robot, None)
            else:
                self.stalled_since.pop(robot, None)
        self.flush_events()
        self.publish_state(now)

    def publish_state(self, now: float) -> None:
        snap = self.rm.snapshot(now)
        msg = TrafficState()
        msg.stamp = self.get_clock().now().to_msg()
        for rid, s in snap.items():
            z = ZoneState()
            z.zone_id, z.owner, z.queue, z.held_since = rid, s["owner"], list(s["queue"]), float(s["held_since"])
            (msg.stations if rid in L.STATIONS else msg.zones).append(z)
        msg.deadlock_preventions = self.rm.deadlock_preventions
        msg.recoveries = self.rm.recoveries
        self.state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrafficManager()
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
