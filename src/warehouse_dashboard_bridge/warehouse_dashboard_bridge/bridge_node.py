#!/usr/bin/env python3
"""Dashboard bridge: ROS 2 to browser.

A ROS 2 node subscribes to the real system topics, keeps a JSON-serialisable state and serves it to the web
dashboard through FastAPI:

    GET  /api/layout                 static warehouse geometry (shelves, aisles, zones, stations ...)
    GET  /api/state                  latest full state
    WS   /ws                         state pushed at ~5 Hz
    POST /api/control                {"command": start|pause|resume|reset|estop|release_estop}  -> /warehouse/sim_command
    POST /api/orders/generate        {"count": n}                                                  -> /warehouse/generate_orders
    POST /api/orders/rate            {"orders_per_minute": x}                                      -> /warehouse/set_order_rate
    POST /api/export                 flush the CSV files                                           -> /warehouse/export_metrics
    GET  /api/metrics.csv            time series CSV of the current run
    GET  /                           the built React dashboard (dashboard/dist)

Nothing in the state is fabricated: every number comes from a ROS topic published by the simulation or by the
fleet-management nodes.  Runs on wall time.
"""
import asyncio
import json
import math
import os
import threading
import time
from collections import deque
from typing import Any, Dict, List

import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from warehouse_common import layout as L
from warehouse_robot_interfaces.msg import (EntityPoses, MetricsSnapshot, OrderList, RobotStatus, TrafficState,
                                            WarehouseEvent)
from warehouse_robot_interfaces.srv import GenerateOrders, SetRate, SimCommand, SubmitOrderBatch

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def t_of(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class Bridge(Node):
    def __init__(self) -> None:
        super().__init__("dashboard_bridge")
        self.declare_parameter("port", 3000)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("robots", ",".join(L.ROBOT_IDS))
        self.declare_parameter("static_dir", "")
        self.robots = [r for r in self.get_parameter("robots").value.split(",") if r]
        self.lock = threading.RLock()
        self.robots_state: Dict[str, dict] = {}
        self.orders: List[dict] = []
        self.metrics: Dict[str, Any] = {}
        self.traffic: Dict[str, Any] = {"zones": [], "stations": [], "deadlock_preventions": 0, "recoveries": 0}
        self.sim: Dict[str, Any] = {"state": "STARTING", "paused": False, "estop": False, "sim_time": 0.0, "rtf": 0.0}
        self.startup: Dict[str, Any] = {"ready": False, "missing": ["starting"]}
        self.items: Dict[str, List[float]] = {}
        self.events: deque = deque(maxlen=300)
        self.event_seq = 0
        self.history: deque = deque(maxlen=1200)
        self.last_hist = -1e9
        self.csv_dir = ""
        self.last_msg_time = 0.0
        self.fleet_selection = "all"

        for r in self.robots:
            self.create_subscription(RobotStatus, f"/{r}/status", self.on_status, 10)
        self.create_subscription(OrderList, "/warehouse/orders", self.on_orders, LATCHED)
        self.create_subscription(MetricsSnapshot, "/warehouse/metrics", self.on_metrics, 10)
        self.create_subscription(TrafficState, "/traffic/state", self.on_traffic, 10)
        self.create_subscription(WarehouseEvent, "/warehouse/events", self.on_event, 200)
        self.create_subscription(String, "/warehouse/sim_state", self.on_sim, LATCHED)
        self.create_subscription(String, "/warehouse/startup_status", self.on_startup, LATCHED)
        self.create_subscription(EntityPoses, "/warehouse/ground_truth", self.on_gt, 5)
        self.cli_sim = self.create_client(SimCommand, "/warehouse/sim_command")
        self.cli_gen = self.create_client(GenerateOrders, "/warehouse/generate_orders")
        self.cli_batch = self.create_client(SubmitOrderBatch, "/warehouse/submit_order_batch")
        self.fleet_pub = self.create_publisher(String, "/warehouse/fleet_selection", LATCHED)
        self.create_timer(3.0, self.publish_fleet_selection)
        self.cli_rate = self.create_client(SetRate, "/warehouse/set_order_rate")
        self.cli_export = self.create_client(Trigger, "/warehouse/export_metrics")

    def publish_fleet_selection(self):
        m = String()
        m.data = self.fleet_selection
        self.fleet_pub.publish(m)

    # ------------------------------------------------------------------ ROS -> state
    def on_status(self, m: RobotStatus) -> None:
        d = {"id": m.robot_id, "state": m.state, "order_id": m.order_id, "item_id": m.item_id,
             "nav_status": m.nav_status, "arm_status": m.arm_status, "gripper_status": m.gripper_status,
             "destination": m.destination, "goal": [m.goal_x, m.goal_y, m.goal_yaw] if m.has_goal else None,
             "x": m.x, "y": m.y, "yaw": m.yaw, "speed": m.speed, "battery": m.battery,
             "distance": m.distance_travelled, "tasks_completed": m.tasks_completed, "tasks_failed": m.tasks_failed,
             "queue_length": m.queue_length, "queued_orders": list(m.queued_orders),
             "held_resources": list(m.held_resources), "waiting_for": m.waiting_for, "healthy": m.healthy,
             "path": [round(v, 2) for v in m.path_xy], "stamp": t_of(m.stamp), "wall": time.time()}
        with self.lock:
            self.robots_state[m.robot_id] = d

    def on_orders(self, m: OrderList) -> None:
        with self.lock:
            self.orders = [{"id": o.order_id, "item": o.item_name, "item_id": o.item_id, "shelf": o.shelf_id,
                            "aisle": o.aisle_id, "station": o.station_id, "arrival": o.arrival_time,
                            "deadline": o.deadline, "status": o.status, "robot": o.assigned_robot,
                            "started": o.started_time, "completed": o.completed_time, "attempts": o.attempts}
                           for o in m.orders]

    def on_metrics(self, m: MetricsSnapshot) -> None:
        d = {"sim_time": m.sim_time, "running": m.running, "estop": m.estop, "orders_total": m.orders_total,
             "completed": m.orders_completed, "pending": m.orders_pending, "processing": m.orders_processing,
             "late": m.orders_late, "failed": m.orders_failed, "orders_per_minute": m.orders_per_minute,
             "avg_completion_time": m.avg_completion_time, "collisions": m.collisions, "near_misses": m.near_misses,
             "nav_failures": m.nav_failures, "pickup_failures": m.pickup_failures,
             "delivery_failures": m.delivery_failures, "deadlock_preventions": m.deadlock_preventions,
             "recoveries": m.recoveries, "robot_ids": list(m.robot_ids), "utilization": list(m.utilization),
             "idle_time": list(m.idle_time), "tasks_completed": list(m.tasks_completed),
             "distance": list(m.distance_travelled), "avg_task_time": list(m.avg_task_time)}
        with self.lock:
            self.metrics = d
            if m.sim_time - self.last_hist >= 5.0:
                self.last_hist = m.sim_time
                self.history.append({k: d[k] for k in ("sim_time", "completed", "late", "collisions", "near_misses",
                                                       "orders_per_minute", "avg_completion_time", "pending",
                                                       "processing", "utilization", "tasks_completed",
                                                       "deadlock_preventions", "recoveries", "nav_failures")})

    def on_traffic(self, m: TrafficState) -> None:
        z = lambda s: {"id": s.zone_id, "owner": s.owner, "queue": list(s.queue), "since": s.held_since}  # noqa: E731
        with self.lock:
            self.traffic = {"zones": [z(s) for s in m.zones], "stations": [z(s) for s in m.stations],
                            "deadlock_preventions": m.deadlock_preventions, "recoveries": m.recoveries}

    def on_event(self, e: WarehouseEvent) -> None:
        with self.lock:
            self.event_seq += 1
            self.events.append({"seq": self.event_seq, "t": t_of(e.stamp), "category": e.category,
                                "severity": e.severity, "robot": e.robot_id, "order": e.order_id,
                                "detail": e.detail})

    def on_sim(self, m: String) -> None:
        try:
            with self.lock:
                self.sim = json.loads(m.data)
        except ValueError:
            pass

    def on_startup(self, m: String) -> None:
        try:
            with self.lock:
                self.startup = json.loads(m.data)
        except ValueError:
            pass

    def on_gt(self, m: EntityPoses) -> None:
        with self.lock:
            for n, p in zip(m.names, m.poses):
                if n.startswith("item_"):
                    self.items[n] = [round(p.position.x, 3), round(p.position.y, 3), round(p.position.z, 3)]
        self.last_msg_time = time.time()

    # ------------------------------------------------------------------ state out
    def snapshot(self) -> dict:
        now = time.time()
        with self.lock:
            robots = []
            for r in self.robots:
                d = dict(self.robots_state.get(r) or {"id": r, "state": "OFFLINE", "x": L.bay_pose(r)[0],
                                                      "y": L.bay_pose(r)[1], "yaw": L.bay_pose(r)[2],
                                                      "healthy": False, "held_resources": [], "path": [],
                                                      "battery": 0, "distance": 0, "tasks_completed": 0,
                                                      "tasks_failed": 0, "queue_length": 0, "speed": 0,
                                                      "nav_status": "-", "arm_status": "-", "gripper_status": "-",
                                                      "destination": "", "goal": None, "order_id": "",
                                                      "waiting_for": "", "queued_orders": []})
                d["online"] = (now - d.get("wall", 0)) < 5.0
                robots.append(d)
            return {"type": "state", "wall": now, "sim": self.sim, "startup": self.startup, "robots": robots,
                    "orders": self.orders, "metrics": self.metrics, "traffic": self.traffic,
                    "fleet_selection": self.fleet_selection,
                    "items": self.items, "events": list(self.events)[-80:], "event_seq": self.event_seq}

    def history_json(self) -> list:
        with self.lock:
            return list(self.history)

    # ------------------------------------------------------------------ service helpers
    def call(self, client, req, timeout: float = 8.0):
        if not client.wait_for_service(timeout_sec=2.0):
            raise HTTPException(status_code=503, detail="ROS service not available (system still starting?)")
        fut = client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.01)
        if not fut.done():
            raise HTTPException(status_code=504, detail="ROS service timed out")
        return fut.result()


def build_app(node: Bridge, static_dir: str) -> FastAPI:
    app = FastAPI(title="Autonomous Warehouse Dashboard Backend")
    app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

    @app.get("/api/layout")
    def layout():
        return L.layout_dict()

    @app.get("/api/state")
    def state():
        return node.snapshot()

    @app.get("/api/history")
    def history():
        return node.history_json()

    @app.get("/api/health")
    def health():
        return {"ok": True, "ros": rclpy.ok(), "last_ground_truth_age": round(time.time() - node.last_msg_time, 1)}

    @app.post("/api/control")
    def control(body: dict):
        cmd = str(body.get("command", "")).lower()
        if cmd not in ("start", "pause", "resume", "reset", "estop", "release_estop"):
            raise HTTPException(400, f"unknown command {cmd}")
        req = SimCommand.Request()
        req.command = cmd
        res = node.call(node.cli_sim, req, timeout=15.0)
        return {"success": res.success, "message": res.message}

    @app.post("/api/orders/generate")
    def generate(body: dict):
        req = GenerateOrders.Request()
        req.count = max(1, min(20, int(body.get("count", 3))))
        res = node.call(node.cli_gen, req)
        return {"created": res.created, "message": res.message}

    @app.get("/api/catalog")
    def catalog():
        occupied = {o["item_id"] for o in node.orders if o["status"] in ("PENDING", "ASSIGNED", "IN_PROGRESS")}
        return {"items": [{"id": i.item_id, "name": i.name, "shelf": i.shelf_id,
                            "aisle": i.aisle, "available": i.item_id not in occupied,
                            "slot": i.slot} for i in L.ITEMS.values()],
                "stations": sorted(L.STATIONS), "robot_ids": node.robots,
                "fleet_selection": node.fleet_selection}

    @app.post("/api/orders/submit")
    def submit(body: dict):
        ids = body.get("item_ids", [])
        station = body.get("station_id", "")
        if not isinstance(ids, list) or not (1 <= len(ids) <= 12) or any(not isinstance(i, str) for i in ids):
            raise HTTPException(400, "item_ids must be an array of 1-12 item IDs")
        if len(ids) != len(set(ids)) or any(i not in L.ITEMS for i in ids):
            raise HTTPException(400, "Unknown or duplicate physical items")
        if station not in L.STATIONS:
            raise HTTPException(400, "Unknown packing station")
        req = SubmitOrderBatch.Request()
        req.item_ids, req.station_id = ids, station
        res = node.call(node.cli_batch, req)
        if not res.success:
            raise HTTPException(409, res.message)
        return {"success": True, "message": res.message, "order_ids": list(res.order_ids)}

    @app.post("/api/fleet/selection")
    def fleet_selection(body: dict):
        selection = body.get("robot_id", "all")
        if selection not in (["all"] + node.robots):
            raise HTTPException(400, "Select all robots or one of the active robots")
        node.fleet_selection = selection
        node.publish_fleet_selection()
        return {"success": True, "message": "Dispatch selection: " + selection, "robot_id": selection}

    @app.post("/api/orders/rate")
    def rate(body: dict):
        req = SetRate.Request()
        req.orders_per_minute = float(body.get("orders_per_minute", 2.0))
        res = node.call(node.cli_rate, req)
        if not res.success:
            raise HTTPException(400, res.message)
        return {"success": True, "message": res.message}

    @app.post("/api/export")
    def export():
        res = node.call(node.cli_export, Trigger.Request())
        node.csv_dir = res.message
        return {"success": res.success, "directory": res.message}

    @app.get("/api/metrics.csv")
    def metrics_csv():
        if not node.csv_dir:
            res = node.call(node.cli_export, Trigger.Request())
            node.csv_dir = res.message
        path = os.path.join(node.csv_dir, "metrics_timeseries.csv")
        if not os.path.exists(path):
            raise HTTPException(404, "no metrics yet")
        return FileResponse(path, media_type="text/csv", filename="metrics_timeseries.csv")

    @app.get("/api/orders.csv")
    def orders_csv():
        res = node.call(node.cli_export, Trigger.Request())
        node.csv_dir = res.message
        path = os.path.join(node.csv_dir, "orders.csv")
        if not os.path.exists(path):
            raise HTTPException(404, "no orders yet")
        return FileResponse(path, media_type="text/csv", filename="orders.csv")

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        try:
            await sock.send_text(json.dumps({"type": "history", "history": node.history_json()}))
            last_hist = time.time()
            while True:
                await sock.send_text(json.dumps(node.snapshot()))
                if time.time() - last_hist > 5.0:
                    last_hist = time.time()
                    await sock.send_text(json.dumps({"type": "history", "history": node.history_json()}))
                await asyncio.sleep(0.2)
        except (WebSocketDisconnect, RuntimeError):
            return
        except Exception:                                    # noqa: BLE001
            return

    if static_dir and os.path.isdir(static_dir):
        app.mount("/", StaticFiles(directory=static_dir, html=True), name="dashboard")
    else:
        @app.get("/")
        def index():
            return PlainTextResponse("Dashboard build not found. Run: cd dashboard && npm install && npm run build\n"
                                     "API is available under /api/*", status_code=200)
    return app


def find_dist() -> str:
    """Locate <project>/dashboard/dist by walking up from this file (install/<pkg>/lib/pythonX/site-packages/...)."""
    d = os.path.abspath(os.path.dirname(__file__))
    for _ in range(9):
        cand = os.path.join(d, "dashboard", "dist")
        if os.path.isfile(os.path.join(cand, "index.html")):
            return cand
        d = os.path.dirname(d)
    return ""


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Bridge()
    static_dir = node.get_parameter("static_dir").value or os.environ.get("WAREHOUSE_DASHBOARD_DIST", "") \
        or find_dist()
    app = build_app(node, static_dir)
    port = int(node.get_parameter("port").value)
    host = node.get_parameter("host").value
    config = uvicorn.Config(app, host=host, port=port, log_level="warning", ws_ping_interval=20)
    server = uvicorn.Server(config)
    t = threading.Thread(target=server.run, daemon=True)
    t.start()
    node.get_logger().info(f"dashboard backend on http://localhost:{port}  (static: {static_dir or 'none'})")
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.should_exit = True
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
