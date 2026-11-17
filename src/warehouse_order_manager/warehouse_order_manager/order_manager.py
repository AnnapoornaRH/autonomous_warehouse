#!/usr/bin/env python3
"""Order manager: autonomous order generator + authoritative order book.

  * generates random orders continuously (exponential inter-arrival times, adjustable rate)
  * publishes the order book on /warehouse/orders (active orders + the most recent finished ones)
  * applies OrderUpdate messages from the allocator and the robot agents - it is the ONLY writer of order
    status, so an order can never be completed by two robots or marked completed twice
  * services: /warehouse/generate_orders (burst of test orders), /warehouse/set_order_rate

Everything runs on simulation time: pausing the simulation pauses order arrival.
"""
import random
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from warehouse_common import orders as O
from warehouse_common import layout as L
from warehouse_robot_interfaces.msg import Order, OrderList, OrderUpdate, WarehouseEvent
from warehouse_robot_interfaces.srv import GenerateOrders, SetRate, SubmitOrderBatch

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class OrderManager(Node):
    def __init__(self) -> None:
        super().__init__("order_manager")
        self.declare_parameter("orders_per_minute", 1.0)
        self.declare_parameter("max_active_orders", 14)
        self.declare_parameter("seed", 7)
        self.declare_parameter("initial_orders", 3)
        self.declare_parameter("deadline_slack_min", 170.0)
        self.declare_parameter("deadline_slack_max", 280.0)
        self.declare_parameter("history_size", 80)
        self.rate = float(self.get_parameter("orders_per_minute").value)
        self.max_active = int(self.get_parameter("max_active_orders").value)
        self.history_size = int(self.get_parameter("history_size").value)
        self.seed = int(self.get_parameter("seed").value)
        self.factory = O.OrderFactory(self.seed, float(self.get_parameter("deadline_slack_min").value),
                                      float(self.get_parameter("deadline_slack_max").value))
        self.rng = random.Random(self.seed + 1)
        self.book: Dict[str, O.OrderRecord] = {}
        self.finished_order: List[str] = []
        self.late_reported: set = set()
        self.item_cooldown: Dict[str, float] = {}       # item -> sim time until it is back on its shelf
        self.running = False            # becomes True when the system monitor reports READY
        self.next_arrival = None
        self.initial_left = int(self.get_parameter("initial_orders").value)

        self.orders_pub = self.create_publisher(OrderList, "/warehouse/orders", LATCHED)
        self.event_pub = self.create_publisher(WarehouseEvent, "/warehouse/events", 50)
        self.create_subscription(OrderUpdate, "/warehouse/order_update", self.on_update, 50)
        self.create_subscription(Bool, "/warehouse/running", self.on_running, LATCHED)
        self.create_subscription(String, "/warehouse/reset", self.on_reset, 10)
        self.create_service(GenerateOrders, "/warehouse/generate_orders", self.on_generate)
        self.create_service(SetRate, "/warehouse/set_order_rate", self.on_rate)
        self.create_service(SubmitOrderBatch, "/warehouse/submit_order_batch", self.on_submit_batch)
        self.create_timer(1.0, self.tick)
        self.create_timer(0.5, self.publish_book)
        self.get_logger().info(f"order_manager ready: {self.rate:.1f} orders/min")

    # ------------------------------------------------------------------ helpers
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, category: str, severity: str, detail: str, order_id: str = "", robot: str = "",
              value: float = 0.0) -> None:
        e = WarehouseEvent()
        e.stamp = self.get_clock().now().to_msg()
        e.category, e.severity, e.detail, e.order_id, e.robot_id, e.value = category, severity, detail, \
            order_id, robot, float(value)
        self.event_pub.publish(e)

    def active_count(self) -> int:
        return sum(1 for o in self.book.values() if o.status in O.ACTIVE_STATES)

    def unavailable_items(self) -> set:
        now = self.now()
        busy = {o.item_id for o in self.book.values() if o.status in O.ACTIVE_STATES}
        busy |= {i for i, t in self.item_cooldown.items() if t > now}
        return busy

    def create_order(self):
        o = self.factory.new_order(self.now(), self.unavailable_items())
        if o is None:
            return None
        self.book[o.order_id] = o
        self.event("order_created", "info", f"{o.order_id}: {o.item_name} ({o.item_id}) from shelf {o.shelf_id} "
                   f"to {o.station_id}, due in {o.deadline - o.arrival_time:.0f}s", o.order_id)
        return o

    def schedule_next(self) -> None:
        mean = 60.0 / max(self.rate, 1e-6)
        self.next_arrival = self.now() + self.rng.expovariate(1.0 / mean)

    # ------------------------------------------------------------------ callbacks
    def on_running(self, msg: Bool) -> None:
        self.running = msg.data

    def on_reset(self, msg: String) -> None:
        if msg.data == "begin":
            self.book.clear()
            self.finished_order.clear()
            self.late_reported.clear()
            self.item_cooldown.clear()
            self.factory.reset()
            self.initial_left = int(self.get_parameter("initial_orders").value)
            self.next_arrival = None
            self.publish_book()

    def on_generate(self, req: GenerateOrders.Request, res: GenerateOrders.Response) -> GenerateOrders.Response:
        n = 0
        for _ in range(int(req.count)):
            if self.active_count() >= self.max_active * 2:
                break
            if self.create_order() is None:
                break                                   # every item is currently in use
            n += 1
        res.created, res.message = n, f"created {n} orders"
        self.publish_book()
        return res

    def on_submit_batch(self, req: SubmitOrderBatch.Request, res: SubmitOrderBatch.Response) -> SubmitOrderBatch.Response:
        """Atomically enqueue requested physical items, each exactly once per active batch."""
        ids = list(req.item_ids)
        station = req.station_id
        if not self.running:
            res.success, res.message = False, "Warehouse is not running. Press Start first."
            return res
        if not ids or len(ids) > 12 or len(set(ids)) != len(ids):
            res.success, res.message = False, "Select 1-12 distinct physical items."
            return res
        if station not in L.STATIONS:
            res.success, res.message = False, "Unknown packing station."
            return res
        if any(i not in L.ITEMS for i in ids):
            res.success, res.message = False, "Unknown item ID."
            return res
        busy = self.unavailable_items()
        unavailable = [i for i in ids if i in busy]
        if unavailable:
            res.success, res.message = False, "Items unavailable or restocking: " + ", ".join(unavailable)
            return res
        if self.active_count() + len(ids) > self.max_active * 2:
            res.success, res.message = False, "Order queue full."
            return res
        now = self.now()
        # Preserve the same item_id used by real-world Gazebo pick and place actions.
        for iid in ids:
            self.factory.counter += 1
            item = L.ITEMS[iid]
            order = O.OrderRecord(
                order_id=f"ORD-{self.factory.counter:04d}", item_id=iid,
                item_name=item.name, shelf_id=item.shelf_id, aisle_id=item.aisle,
                station_id=station, arrival_time=now,
                deadline=now + self.factory.rng.uniform(self.factory.slack_min, self.factory.slack_max))
            self.book[order.order_id] = order
            res.order_ids.append(order.order_id)
            self.event("order_created", "info",
                       f"Dashboard: {order.order_id}: {item.name} from {item.shelf_id} to {station}",
                       order.order_id)
        res.success = True
        res.message = f"Queued {len(ids)} item deliveries to {station}"
        self.publish_book()
        return res

    def on_rate(self, req: SetRate.Request, res: SetRate.Response) -> SetRate.Response:
        if not (0.0 <= req.orders_per_minute <= 30.0):
            res.success, res.message = False, "rate must be within 0..30 orders/min"
            return res
        self.rate = float(req.orders_per_minute)
        self.next_arrival = None
        res.success, res.message = True, f"order rate set to {self.rate:.2f}/min"
        self.event("sim", "info", f"order arrival rate set to {self.rate:.2f}/min")
        return res

    def on_update(self, u: OrderUpdate) -> None:
        o = self.book.get(u.order_id)
        if o is None:
            return
        now = self.now()
        if o.status in (O.COMPLETED, O.FAILED):
            return                                         # terminal: ignore late/duplicate updates
        if u.status == O.ASSIGNED:
            if o.status != O.PENDING:
                return                                     # never assign twice
            o.status, o.assigned_robot = O.ASSIGNED, u.robot_id
        elif u.status == O.IN_PROGRESS:
            if o.assigned_robot not in ("", u.robot_id):
                return
            o.status, o.assigned_robot = O.IN_PROGRESS, u.robot_id
            if o.started_time == 0.0:
                o.started_time = now
        elif u.status == O.COMPLETED:
            if o.assigned_robot != u.robot_id:
                return
            o.status, o.completed_time = O.COMPLETED, now
            self.item_cooldown[o.item_id] = now + 15.0           # packing + restocking time
            self.finished_order.append(o.order_id)
            late = o.completed_time > o.deadline
            self.event("order_completed", "warn" if late else "info",
                       f"{o.order_id} delivered by {u.robot_id} in {now - o.arrival_time:.0f}s"
                       f"{' (LATE)' if late else ''}", o.order_id, u.robot_id, now - o.arrival_time)
        elif u.status == O.FAILED:
            o.status, o.completed_time = O.FAILED, now
            self.item_cooldown[o.item_id] = now + 15.0
            self.finished_order.append(o.order_id)
            self.event("order_failed", "error", f"{o.order_id} failed: {u.reason}", o.order_id, u.robot_id)
        elif u.status == O.PENDING:                        # re-queue after a failure
            o.status, o.assigned_robot = O.PENDING, ""
            o.attempts += 1
            self.event("order_requeued", "warn", f"{o.order_id} re-queued ({u.reason})", o.order_id, u.robot_id)
        self.publish_book()

    # ------------------------------------------------------------------ periodic
    def tick(self) -> None:
        now = self.now()
        if self.running:
            if self.initial_left > 0:
                self.initial_left -= 1
                self.create_order()
            if self.rate > 0.0:
                if self.next_arrival is None:
                    self.schedule_next()
                elif now >= self.next_arrival:
                    if self.active_count() < self.max_active:
                        self.create_order()
                    self.schedule_next()
        for o in self.book.values():
            if (o.status in O.ACTIVE_STATES and now > o.deadline and o.order_id not in self.late_reported):
                self.late_reported.add(o.order_id)
                self.event("order_late", "warn", f"{o.order_id} missed its deadline ({o.status})",
                           o.order_id, o.assigned_robot)

    def publish_book(self) -> None:
        keep = [o for o in self.book.values() if o.status in O.ACTIVE_STATES]
        for oid in self.finished_order[-self.history_size:]:
            if oid in self.book:
                keep.append(self.book[oid])
        keep.sort(key=lambda o: o.arrival_time)
        msg = OrderList()
        msg.stamp = self.get_clock().now().to_msg()
        for o in keep:
            m = Order()
            (m.order_id, m.item_id, m.item_name, m.shelf_id, m.aisle_id, m.station_id, m.arrival_time, m.deadline,
             m.status, m.assigned_robot, m.started_time, m.completed_time, m.attempts) = (
                o.order_id, o.item_id, o.item_name, o.shelf_id, o.aisle_id, o.station_id, o.arrival_time,
                o.deadline, o.status, o.assigned_robot, o.started_time, o.completed_time, min(255, o.attempts))
            msg.orders.append(m)
        self.orders_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrderManager()
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
