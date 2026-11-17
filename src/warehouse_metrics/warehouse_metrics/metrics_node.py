#!/usr/bin/env python3
"""Metrics node: warehouse KPIs computed from real simulation events, plus CSV export.

Official KPIs
  1. orders completed per minute      (completed orders in the last ``window`` sim-minutes, or since start)
  2. number of late orders            (completed after the deadline, or still open past the deadline)
  3. number of collisions             (chassis contact sensor episodes with walls / shelves / obstacles / robots)
Additional
  average order completion time, per-robot utilisation / idle time / distance / tasks, navigation,
  pickup and delivery failures, deadlock-prevention events, recoveries.  Near misses (two robots closer than
  0.7 m) and waits are tracked separately from collisions.

Files written to ``output_dir`` (default ~/warehouse_metrics/run_<timestamp>/):
  metrics_timeseries.csv   one row every ``sample_period`` sim seconds
  orders.csv               every order seen, rewritten every 10 s
  events.csv               every WarehouseEvent (append)
  robots.csv               per robot summary, rewritten every 10 s
Service /warehouse/export_metrics (std_srvs/Trigger) forces a flush and returns the directory.
"""
import csv
import os
import time
from collections import deque
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from warehouse_common import layout as L
from warehouse_robot_interfaces.msg import MetricsSnapshot, OrderList, RobotStatus, TrafficState, WarehouseEvent

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class MetricsNode(Node):
    def __init__(self) -> None:
        super().__init__("metrics_node")
        self.declare_parameter("output_dir", os.path.join(os.path.expanduser("~"), "warehouse_metrics"))
        self.declare_parameter("window_minutes", 5.0)
        self.declare_parameter("sample_period", 5.0)
        self.window = float(self.get_parameter("window_minutes").value) * 60.0
        self.sample_period = float(self.get_parameter("sample_period").value)
        self.base_dir = self.get_parameter("output_dir").value
        self.counts: Dict[str, int] = {}
        self.orders: Dict[str, dict] = {}
        self.completed_times = deque()
        self.status: Dict[str, RobotStatus] = {}
        self.robot_busy: Dict[str, float] = {r: 0.0 for r in L.ROBOT_IDS}
        self.robot_idle: Dict[str, float] = {r: 0.0 for r in L.ROBOT_IDS}
        self.robot_last_t: Dict[str, float] = {}
        self.task_durations: Dict[str, list] = {r: [] for r in L.ROBOT_IDS}
        self.traffic = None
        self.running = False
        self.estop = False
        self.t_start = None
        self.last_sample = -1e9
        self.last_flush = 0.0
        self.open_dir()

        self.pub = self.create_publisher(MetricsSnapshot, "/warehouse/metrics", 10)
        self.create_subscription(OrderList, "/warehouse/orders", self.on_orders, LATCHED)
        self.create_subscription(WarehouseEvent, "/warehouse/events", self.on_event, 200)
        for r in L.ROBOT_IDS:
            self.create_subscription(RobotStatus, f"/{r}/status", self.on_status, 10)
        self.create_subscription(TrafficState, "/traffic/state", lambda m: setattr(self, "traffic", m), 10)
        self.create_subscription(Bool, "/warehouse/running", lambda m: setattr(self, "running", m.data), LATCHED)
        self.create_subscription(Bool, "/warehouse/estop", lambda m: setattr(self, "estop", m.data), LATCHED)
        self.create_subscription(String, "/warehouse/reset", self.on_reset, 10)
        self.create_service(Trigger, "/warehouse/export_metrics", self.on_export)
        self.create_timer(1.0, self.tick)
        self.get_logger().info(f"metrics_node ready, writing CSV to {self.run_dir}")

    # ------------------------------------------------------------------ files
    def open_dir(self) -> None:
        self.run_dir = os.path.join(self.base_dir, "run_" + time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self.run_dir, exist_ok=True)
        with open(os.path.join(self.run_dir, "events.csv"), "w", newline="") as f:
            csv.writer(f).writerow(["sim_time", "category", "severity", "robot", "order", "detail", "value"])
        with open(os.path.join(self.run_dir, "metrics_timeseries.csv"), "w", newline="") as f:
            csv.writer(f).writerow(self.TS_HEADER)

    TS_HEADER = ["sim_time", "orders_total", "completed", "pending", "processing", "late", "failed",
                 "orders_per_minute", "avg_completion_time_s", "collisions", "near_misses", "nav_failures",
                 "pickup_failures", "delivery_failures", "deadlock_preventions", "recoveries",
                 "util_robot1", "util_robot2", "util_robot3"]

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ inputs
    def on_orders(self, msg: OrderList) -> None:
        for o in msg.orders:
            self.orders[o.order_id] = dict(
                order_id=o.order_id, item=o.item_name, item_id=o.item_id, shelf=o.shelf_id, station=o.station_id,
                arrival=o.arrival_time, deadline=o.deadline, status=o.status, robot=o.assigned_robot,
                started=o.started_time, completed=o.completed_time, attempts=o.attempts)

    def on_event(self, e: WarehouseEvent) -> None:
        t = e.stamp.sec + e.stamp.nanosec * 1e-9
        self.counts[e.category] = self.counts.get(e.category, 0) + 1
        if e.category == "order_completed":
            self.completed_times.append(t)
        with open(os.path.join(self.run_dir, "events.csv"), "a", newline="") as f:
            csv.writer(f).writerow([f"{t:.1f}", e.category, e.severity, e.robot_id, e.order_id, e.detail,
                                    f"{e.value:.2f}"])

    def on_status(self, s: RobotStatus) -> None:
        t = self.now()
        last = self.robot_last_t.get(s.robot_id)
        if last is not None and self.running and not self.estop:
            dt = min(2.0, max(0.0, t - last))
            if self.status.get(s.robot_id) is not None and self.status[s.robot_id].state == "IDLE":
                self.robot_idle[s.robot_id] += dt
            else:
                self.robot_busy[s.robot_id] += dt
        self.robot_last_t[s.robot_id] = t
        prev = self.status.get(s.robot_id)
        if prev is not None and s.tasks_completed > prev.tasks_completed:
            pass
        self.status[s.robot_id] = s

    def on_reset(self, msg: String) -> None:
        if msg.data == "begin":
            self.flush(final=True)
            self.counts.clear()
            self.orders.clear()
            self.completed_times.clear()
            for r in L.ROBOT_IDS:
                self.robot_busy[r] = self.robot_idle[r] = 0.0
            self.t_start = None
            self.open_dir()

    def on_export(self, req, res):
        self.flush(final=True)
        res.success, res.message = True, self.run_dir
        return res

    # ------------------------------------------------------------------ computation
    def snapshot(self) -> MetricsSnapshot:
        now = self.now()
        if self.t_start is None:
            self.t_start = now
        m = MetricsSnapshot()
        m.stamp = self.get_clock().now().to_msg()
        m.sim_time = now
        m.running, m.estop = self.running, self.estop
        statuses = [o["status"] for o in self.orders.values()]
        m.orders_total = self.counts.get("order_created", 0)
        m.orders_completed = self.counts.get("order_completed", 0)
        m.orders_failed = self.counts.get("order_failed", 0)
        m.orders_pending = sum(1 for s in statuses if s == "PENDING")
        m.orders_processing = sum(1 for s in statuses if s in ("ASSIGNED", "IN_PROGRESS"))
        late = 0
        times = []
        for o in self.orders.values():
            if o["status"] == "COMPLETED":
                times.append(o["completed"] - o["arrival"])
                if o["completed"] > o["deadline"]:
                    late += 1
            elif o["status"] in ("PENDING", "ASSIGNED", "IN_PROGRESS", "FAILED") and now > o["deadline"]:
                late += 1
        m.orders_late = late
        m.avg_completion_time = sum(times) / len(times) if times else 0.0
        while self.completed_times and now - self.completed_times[0] > self.window:
            self.completed_times.popleft()
        span = min(self.window, max(60.0, now - self.t_start))
        m.orders_per_minute = len(self.completed_times) / (span / 60.0)
        c = self.counts
        m.collisions, m.near_misses = c.get("collision", 0), c.get("near_miss", 0)
        m.nav_failures, m.pickup_failures = c.get("nav_failure", 0), c.get("pickup_failure", 0)
        m.delivery_failures = c.get("delivery_failure", 0)
        m.deadlock_preventions = self.traffic.deadlock_preventions if self.traffic else 0
        m.recoveries = (self.traffic.recoveries if self.traffic else 0) + c.get("recovery", 0)
        for r in L.ROBOT_IDS:
            st = self.status.get(r)
            busy, idle = self.robot_busy[r], self.robot_idle[r]
            m.robot_ids.append(r)
            m.utilization.append(busy / (busy + idle) if busy + idle > 1e-3 else 0.0)
            m.idle_time.append(idle)
            m.tasks_completed.append(st.tasks_completed if st else 0)
            m.distance_travelled.append(st.distance_travelled if st else 0.0)
            done = [o for o in self.orders.values() if o["robot"] == r and o["status"] == "COMPLETED"
                    and o["started"] > 0]
            m.avg_task_time.append(sum(o["completed"] - o["started"] for o in done) / len(done) if done else 0.0)
        return m

    def tick(self) -> None:
        m = self.snapshot()
        self.pub.publish(m)
        if m.sim_time - self.last_sample >= self.sample_period and self.running:
            self.last_sample = m.sim_time
            with open(os.path.join(self.run_dir, "metrics_timeseries.csv"), "a", newline="") as f:
                csv.writer(f).writerow([
                    f"{m.sim_time:.1f}", m.orders_total, m.orders_completed, m.orders_pending, m.orders_processing,
                    m.orders_late, m.orders_failed, f"{m.orders_per_minute:.3f}", f"{m.avg_completion_time:.1f}",
                    m.collisions, m.near_misses, m.nav_failures, m.pickup_failures, m.delivery_failures,
                    m.deadlock_preventions, m.recoveries] + [f"{u:.3f}" for u in m.utilization])
        if time.time() - self.last_flush > 10.0:
            self.flush()

    def flush(self, final: bool = False) -> None:
        self.last_flush = time.time()
        m = self.snapshot()
        with open(os.path.join(self.run_dir, "orders.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["order_id", "item", "shelf", "station", "robot", "status", "arrival_s", "deadline_s",
                        "started_s", "completed_s", "completion_time_s", "late", "attempts"])
            for o in sorted(self.orders.values(), key=lambda x: x["arrival"]):
                done = o["status"] == "COMPLETED"
                w.writerow([o["order_id"], o["item"], o["shelf"], o["station"], o["robot"], o["status"],
                            f"{o['arrival']:.1f}", f"{o['deadline']:.1f}", f"{o['started']:.1f}",
                            f"{o['completed']:.1f}", f"{o['completed'] - o['arrival']:.1f}" if done else "",
                            int(done and o["completed"] > o["deadline"]), o["attempts"]])
        with open(os.path.join(self.run_dir, "robots.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["robot", "tasks_completed", "utilization", "idle_time_s", "distance_m", "avg_task_time_s"])
            for i, r in enumerate(m.robot_ids):
                w.writerow([r, m.tasks_completed[i], f"{m.utilization[i]:.3f}", f"{m.idle_time[i]:.1f}",
                            f"{m.distance_travelled[i]:.1f}", f"{m.avg_task_time[i]:.1f}"])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.flush(final=True)
        except Exception:                                    # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
