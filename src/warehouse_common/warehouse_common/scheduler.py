"""Task allocation algorithms (pure python, no ROS).

The allocator node only talks to the ``Scheduler`` interface, so a different algorithm can be dropped
in by subclassing ``Scheduler`` and registering it in ``SCHEDULERS``.

DeadlineAwareScheduler (default)
  * orders are considered in order of increasing *slack*  (deadline - now - best possible service time)
  * for each order every eligible robot is scored with the estimated completion time of the order if
    it were appended to that robot's queue:
        est_start   = max(now, time the robot finishes its current queue)
        est_finish  = est_start + travel(robot end position -> shelf) + pick + travel(shelf -> station)
                      + place + congestion penalty for the aisle
    cost = est_finish + late_weight * max(0, est_finish - deadline) + load_weight * queued_orders
  * the lowest cost robot gets the order; its projected state is updated so that the following orders
    of the same planning round see the new workload (no robot is flooded, no order is assigned twice)
  * robots that are faulted, e-stopped, unhealthy or whose queue is full are not eligible; if nobody
    is eligible the order stays queued.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

from . import layout as L

NOMINAL_SPEED = 0.33          # m/s average including turns and slow-downs
PICK_TIME = 22.0              # s: stop, arm sequence, confirm
PLACE_TIME = 16.0
AISLE_OVERHEAD = 6.0          # s per order: entering/leaving an aisle, reservation latency


@dataclass
class RobotView:
    robot_id: str
    x: float
    y: float
    healthy: bool = True
    accepting: bool = True            # not e-stopped / not faulted
    queue: List[str] = field(default_factory=list)       # order ids (active first)
    busy_until: float = 0.0           # projected sim time when the current queue is finished
    end_xy: Optional[Tuple[float, float]] = None          # projected position after the queue
    max_queue: int = 2
    recent_failures: int = 0


@dataclass
class OrderView:
    order_id: str
    item_id: str
    station_id: str
    arrival: float
    deadline: float
    attempts: int = 0
    excluded: Set[str] = field(default_factory=set)       # robots that already failed this order


@dataclass
class Assignment:
    order_id: str
    robot_id: str
    est_finish: float
    cost: float


def service_time(start_xy: Tuple[float, float], item_id: str, station_id: str) -> float:
    d1 = L.travel_distance_to_item(start_xy, item_id)
    d2 = L.travel_distance_item_to_station(item_id, station_id)
    return (d1 + d2) / NOMINAL_SPEED + PICK_TIME + PLACE_TIME + AISLE_OVERHEAD


class Scheduler:
    name = "base"

    def plan(self, orders: List[OrderView], robots: List[RobotView], now: float,
             congestion: Optional[Dict[str, float]] = None) -> List[Assignment]:
        raise NotImplementedError


class NearestIdleScheduler(Scheduler):
    """Baseline: FIFO orders, nearest robot that has an empty queue."""
    name = "nearest_idle"

    def plan(self, orders, robots, now, congestion=None):
        out: List[Assignment] = []
        taken: Set[str] = set()
        for o in sorted(orders, key=lambda x: x.arrival):
            best = None
            for r in robots:
                if r.robot_id in taken or not r.healthy or not r.accepting or r.queue or r.robot_id in o.excluded:
                    continue
                d = L.travel_distance_to_item((r.x, r.y), o.item_id)
                if best is None or d < best[0]:
                    best = (d, r)
            if best:
                taken.add(best[1].robot_id)
                out.append(Assignment(o.order_id, best[1].robot_id, now + best[0] / NOMINAL_SPEED, best[0]))
        return out


class DeadlineAwareScheduler(Scheduler):
    name = "deadline_aware"

    def __init__(self, late_weight: float = 3.0, load_weight: float = 8.0,
                 congestion_weight: float = 1.0, failure_weight: float = 15.0) -> None:
        self.late_weight = late_weight
        self.load_weight = load_weight
        self.congestion_weight = congestion_weight
        self.failure_weight = failure_weight

    def plan(self, orders, robots, now, congestion=None):
        congestion = congestion or {}
        # work on private projections so the caller's views stay untouched
        proj: Dict[str, dict] = {}
        for r in robots:
            proj[r.robot_id] = {"view": r, "busy_until": max(now, r.busy_until),
                                "end_xy": r.end_xy or (r.x, r.y), "queue": len(r.queue)}

        def slack(o: OrderView) -> float:
            best = min((service_time(p["end_xy"], o.item_id, o.station_id) for p in proj.values()),
                       default=0.0)
            return o.deadline - now - best

        assignments: List[Assignment] = []
        for o in sorted(orders, key=lambda x: (slack(x), x.arrival)):
            best: Optional[Tuple[float, float, str]] = None
            for rid, p in proj.items():
                v: RobotView = p["view"]
                if (not v.healthy or not v.accepting or p["queue"] >= v.max_queue or rid in o.excluded):
                    continue
                svc = service_time(p["end_xy"], o.item_id, o.station_id)
                aisle = L.ITEMS[o.item_id].aisle
                cong = self.congestion_weight * congestion.get(aisle, 0.0)
                est_finish = p["busy_until"] + svc + cong
                late = max(0.0, est_finish - o.deadline)
                cost = (est_finish + self.late_weight * late + self.load_weight * p["queue"] +
                        self.failure_weight * v.recent_failures)
                if best is None or cost < best[0]:
                    best = (cost, est_finish, rid)
            if best is None:
                continue                         # nobody suitable: keep the order queued
            cost, est_finish, rid = best
            p = proj[rid]
            p["busy_until"] = est_finish
            p["end_xy"] = L.station_drop_pose(o.station_id)[:2]
            p["queue"] += 1
            assignments.append(Assignment(o.order_id, rid, est_finish, cost))
        return assignments


SCHEDULERS = {s.name: s for s in (DeadlineAwareScheduler, NearestIdleScheduler)}


def make_scheduler(name: str) -> Scheduler:
    cls = SCHEDULERS.get(name, DeadlineAwareScheduler)
    return cls()
