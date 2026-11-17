#!/usr/bin/env python3
"""Task allocator: continuously assigns pending orders to robots.

Inputs   /warehouse/orders (order book), /<robot>/status (position, state, queue), /traffic/state (congestion)
Output   AssignOrder service call to the chosen robot agent (the agent may refuse: queue full, faulted ...),
         then OrderUpdate(ASSIGNED) to the order manager.

The algorithm lives in warehouse_common.scheduler (parameter ``scheduler``: deadline_aware | nearest_idle).
An order is only marked ASSIGNED after the robot accepted it, and an order is never offered to two robots:
orders with a call in flight are skipped, and the order manager ignores a second ASSIGNED for the same order.
"""
import re
import json
from typing import Dict, List, Set

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from warehouse_common import layout as L
from warehouse_common import scheduler as S
from warehouse_robot_interfaces.msg import (Order, OrderList, OrderUpdate, RobotStatus, TrafficState,
                                            WarehouseEvent)
from warehouse_robot_interfaces.srv import AssignOrder

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

PROGRESS_LEFT = {"IDLE": 0.0, "ASSIGNED": 1.0, "WAITING_FOR_AISLE": 1.0, "NAVIGATING_TO_SHELF": 0.95,
                 "POSITIONING_FOR_PICKUP": 0.6, "PICKING": 0.55, "NAVIGATING_TO_PACKING": 0.4,
                 "WAITING_FOR_PACKING": 0.35, "DROPPING": 0.1, "COMPLETED": 0.0, "RECOVERING": 0.8,
                 "FAULTED": 0.0}


class TaskAllocator(Node):
    def __init__(self) -> None:
        super().__init__("task_allocator")
        self.declare_parameter("scheduler", "deadline_aware")
        self.declare_parameter("cycle", 1.0)
        self.declare_parameter("max_queue", 2)
        self.declare_parameter("status_timeout", 6.0)
        self.scheduler = S.make_scheduler(self.get_parameter("scheduler").value)
        self.max_queue = int(self.get_parameter("max_queue").value)
        self.status_timeout = float(self.get_parameter("status_timeout").value)
        self.cb = ReentrantCallbackGroup()
        self.orders: Dict[str, Order] = {}
        self.status: Dict[str, RobotStatus] = {}
        self.status_time: Dict[str, float] = {}
        self.traffic: TrafficState = None
        self.excluded: Dict[str, Set[str]] = {}
        self.inflight: Set[str] = set()
        self.estop = False
        self.running = False
        self.fleet_selection = "all"
        self.assign_clients = {r: self.create_client(AssignOrder, f"/{r}/assign_order", callback_group=self.cb)
                        for r in L.ROBOT_IDS}
        self.update_pub = self.create_publisher(OrderUpdate, "/warehouse/order_update", 20)
        self.event_pub = self.create_publisher(WarehouseEvent, "/warehouse/events", 20)
        self.create_subscription(OrderList, "/warehouse/orders", self.on_orders, LATCHED, callback_group=self.cb)
        self.create_subscription(OrderUpdate, "/warehouse/order_update", self.on_update, 50, callback_group=self.cb)
        for r in L.ROBOT_IDS:
            self.create_subscription(RobotStatus, f"/{r}/status", self.on_status, 10, callback_group=self.cb)
        self.create_subscription(TrafficState, "/traffic/state", lambda m: setattr(self, "traffic", m), 10,
                                 callback_group=self.cb)
        self.create_subscription(Bool, "/warehouse/estop", lambda m: setattr(self, "estop", m.data), LATCHED,
                                 callback_group=self.cb)
        self.create_subscription(String, "/warehouse/fleet_selection", self.on_fleet_selection, LATCHED,
                                 callback_group=self.cb)
        self.create_subscription(Bool, "/warehouse/running", lambda m: setattr(self, "running", m.data), LATCHED,
                                 callback_group=self.cb)
        self.create_timer(float(self.get_parameter("cycle").value), self.cycle, callback_group=self.cb)
        self.get_logger().info(f"task_allocator ready (scheduler: {self.scheduler.name})")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_orders(self, msg: OrderList) -> None:
        self.orders = {o.order_id: o for o in msg.orders}

    def on_status(self, msg: RobotStatus) -> None:
        self.status[msg.robot_id] = msg
        self.status_time[msg.robot_id] = self.now()

    def on_update(self, u: OrderUpdate) -> None:
        # remember which robots failed an order so it is offered to somebody else
        if u.status == "PENDING" and u.reason.startswith("robot_failed"):
            self.excluded.setdefault(u.order_id, set()).add(u.robot_id)

    def on_fleet_selection(self, msg: String) -> None:
        # Validated at the dashboard API, also verify here for non-dashboard publishers.
        value = msg.data
        if value == "all" or value in L.ROBOT_IDS:
            self.fleet_selection = value
            self.get_logger().info(f"Robot dispatch selection: {value}")

    # ------------------------------------------------------------------ planning
    def robot_views(self, now: float) -> List[S.RobotView]:
        views = []
        for rid in L.ROBOT_IDS:
            st = self.status.get(rid)
            fresh = st is not None and now - self.status_time.get(rid, -1e9) < self.status_timeout
            if not fresh:
                views.append(S.RobotView(rid, *L.bay_pose(rid)[:2], healthy=False, accepting=False))
                continue
            mine = [o for o in self.orders.values()
                    if o.assigned_robot == rid and o.status in ("ASSIGNED", "IN_PROGRESS")]
            mine.sort(key=lambda o: (0 if o.status == "IN_PROGRESS" else 1, o.arrival_time))
            busy, pos = now, (st.x, st.y)
            for k, o in enumerate(mine):
                svc = S.service_time(pos, o.item_id, o.station_id)
                busy += svc * (PROGRESS_LEFT.get(st.state, 1.0) if k == 0 and o.status == "IN_PROGRESS" else 1.0)
                pos = L.station_drop_pose(o.station_id)[:2]
            view = S.RobotView(
                rid, st.x, st.y, healthy=st.healthy and st.state != "FAULTED",
                accepting=not self.estop and (self.fleet_selection == "all" or rid == self.fleet_selection), queue=[o.order_id for o in mine], busy_until=busy,
                end_xy=pos if mine else None, max_queue=self.max_queue,
                recent_failures=min(3, st.tasks_failed))
            views.append(view)
        return views

    def congestion(self) -> Dict[str, float]:
        out: Dict[str, float] = {}
        if self.traffic is None:
            return out
        for z in self.traffic.zones:
            aisle = z.zone_id.split("-")[0]
            out[aisle] = out.get(aisle, 0.0) + (25.0 if z.owner else 0.0) + 20.0 * len(z.queue)
        return out

    def cycle(self) -> None:
        if self.estop or not self.running:
            return
        now = self.now()
        pending = [o for o in self.orders.values() if o.status == "PENDING" and o.order_id not in self.inflight]
        if not pending:
            return
        views = [S.OrderView(o.order_id, o.item_id, o.station_id, o.arrival_time, o.deadline, o.attempts,
                             self.excluded.get(o.order_id, set())) for o in pending]
        plan = self.scheduler.plan(views, self.robot_views(now), now, self.congestion())
        for a in plan:
            if a.order_id in self.inflight:
                continue
            self.inflight.add(a.order_id)
            self.send(a)

    def send(self, a: S.Assignment) -> None:
        o = self.orders[a.order_id]
        cli = self.assign_clients[a.robot_id]
        if not cli.service_is_ready():
            self.inflight.discard(a.order_id)
            return
        req = AssignOrder.Request()
        req.order = o
        fut = cli.call_async(req)

        def done(f, oid=a.order_id, rid=a.robot_id, est=a.est_finish):
            self.inflight.discard(oid)
            try:
                res = f.result()
            except Exception as e:                                   # noqa: BLE001
                self.get_logger().warn(f"assign {oid}->{rid} failed: {e!r}")
                return
            if res.accepted:
                u = OrderUpdate()
                u.order_id, u.status, u.robot_id, u.reason = oid, "ASSIGNED", rid, f"est finish t={est:.0f}"
                self.update_pub.publish(u)
                self.get_logger().info(f"assigned {oid} -> {rid} (est. finish t={est:.0f}s)")
            else:
                self.get_logger().info(f"{rid} refused {oid}: {res.reason}")

        fut.add_done_callback(done)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskAllocator()
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
