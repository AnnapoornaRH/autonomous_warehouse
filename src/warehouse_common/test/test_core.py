"""Unit tests for the ROS-independent core of the warehouse (layout, kinematics, reservations,
scheduler, orders).  Run with:  python3 -m pytest src/warehouse_common/test -v
"""
import math
import random

import pytest

from warehouse_common import kinematics as K
from warehouse_common import layout as L
from warehouse_common import orders as O
from warehouse_common import reservation as R
from warehouse_common import scheduler as S


# ------------------------------------------------------------------------------ layout
def test_zones_are_disjoint_and_cover_aisles():
    zs = list(L.ZONES.values())
    for i, a in enumerate(zs):
        for b in zs[i + 1:]:
            overlap = not (a.x1 <= b.x0 or b.x1 <= a.x0 or a.y1 <= b.y0 or b.y1 <= a.y0)
            assert not overlap, (a.zone_id, b.zone_id)
    assert len(zs) == 8


def test_items_are_inside_shelf_blocks():
    assert len(L.ITEMS) == 12
    for it in L.ITEMS.values():
        sh = L.SHELVES[it.shelf_id]
        bx = L.BLOCK_X[sh.block]
        assert abs(it.x - bx) < L.BLOCK_D / 2
        assert L.SHELF_Y0 < it.y < L.SHELF_Y1
        assert it.z > L.SHELF_TOP_Z


def _blocked(x, y, rects=None, r=0.0):
    rects = rects or L.zone_footprint_rects()
    for (x0, y0, x1, y1) in rects:
        dx = max(x0 - x, 0, x - x1)
        dy = max(y0 - y, 0, y - y1)
        if math.hypot(dx, dy) <= r:
            return True
    return False


def test_pick_and_drop_poses_are_navigable_for_the_robot_radius():
    rad = 0.34
    for iid in L.ITEMS:
        x, y, _ = L.item_pick_pose(iid)
        assert not _blocked(x, y, r=rad), f"pick pose of {iid} collides"
        assert L.zone_at(x, y) == L.ITEMS[iid].zone_id
    for sid in L.STATIONS:
        x, y, _ = L.station_drop_pose(sid)
        assert not _blocked(x, y, r=rad), f"drop pose of {sid} collides"
    for rid in L.ROBOT_IDS:
        x, y, _ = L.bay_pose(rid)
        assert not _blocked(x, y, r=rad + 0.1)


def test_zones_for_item():
    for iid, it in L.ITEMS.items():
        zs = L.zones_for_item(iid)
        assert zs[0].endswith("-S")
        assert (len(zs) == 1) == (it.slot == 0)
        assert it.zone_id in zs


# ------------------------------------------------------------------------------ kinematics
def _to_base(item_xy_z, pose):
    px, py, yaw = pose
    c, s = math.cos(-yaw), math.sin(-yaw)
    rx, ry = item_xy_z[0] - px, item_xy_z[1] - py
    return (c * rx - s * ry, s * rx + c * ry, item_xy_z[2])


@pytest.mark.parametrize("iid", sorted(L.ITEMS))
def test_ik_reaches_every_item_with_margin(iid):
    it = L.ITEMS[iid]
    tgt = _to_base((it.x, it.y, it.z), L.item_pick_pose(iid))
    for dx, dy in ((0, 0), (-0.10, 0), (0.0, 0.12), (0.0, -0.12), (-0.03, 0.08)):
        t = (tgt[0] + dx, tgt[1] + dy, tgt[2])
        q = K.inverse(t)
        assert q is not None, (iid, dx, dy)
        fk = K.forward(q)
        assert max(abs(a - b) for a, b in zip(fk, t)) < 1e-6
    assert K.inverse((tgt[0], tgt[1], tgt[2] + 0.06)) is not None      # lift
    assert K.inverse((tgt[0] - 0.14, tgt[1], tgt[2] + 0.06)) is not None  # retract


@pytest.mark.parametrize("sid", sorted(L.STATIONS))
def test_ik_reaches_every_station(sid):
    px, py, pz = L.station_place_point(sid)
    tgt = _to_base((px, py, pz + 0.012), L.station_drop_pose(sid))
    assert K.inverse(tgt) is not None
    assert K.inverse((tgt[0] - 0.10, tgt[1], tgt[2] + 0.04)) is not None


def test_stow_is_compact_and_above_the_deck():
    pts = K.link_points(K.STOW)
    for (x, y, z) in pts:
        assert -0.30 < x < 0.30
        assert z > 0.25            # above the chassis deck (0.21)
    gx, gy, gz = pts[-1]
    assert gx + 0.04 < 0.25        # a carried 6 cm item stays inside the chassis footprint (front at x=0.25)


def test_ik_unreachable_returns_none():
    assert K.inverse((1.2, 0.0, 0.4)) is None
    assert K.inverse((0.0, 0.0, 1.5)) is None


# ------------------------------------------------------------------------------ reservations
def make_rm(**kw):
    return R.ReservationManager(list(L.ZONES) + list(L.STATIONS), **kw)


def test_opposing_robots_cannot_share_an_aisle():
    rm = make_rm()
    assert rm.request("robot1", ["A1-S", "A1-N"], 10, "o1", 0)[0] == R.GRANTED
    status, pos, _ = rm.request("robot2", ["A1-S"], 5, "o2", 1)       # even a more urgent one waits
    assert status == R.QUEUED and pos == 1
    assert rm.owner["A1-S"] == "robot1"
    rm.release("robot1", ["A1-N"], 2)
    assert rm.request("robot2", ["A1-S"], 5, "o2", 3)[0] == R.QUEUED  # S still owned
    rm.release("robot1", None, 4)
    assert rm.request("robot2", ["A1-S"], 5, "o2", 5)[0] == R.GRANTED
    assert rm.owner["A1-S"] == "robot2"


def test_requests_are_atomic_no_partial_grant():
    rm = make_rm()
    assert rm.request("robot1", ["A2-S"], 1, "a", 0)[0] == R.GRANTED
    st, _, _ = rm.request("robot2", ["A2-S", "A2-N"], 1, "b", 1)
    assert st == R.QUEUED
    assert rm.owner["A2-N"] is None, "a queued robot must not hold part of its request"


def test_no_overtaking_on_conflict_but_independent_requests_pass():
    rm = make_rm()
    rm.request("robot1", ["A1-S"], 1, "a", 0)                       # holder
    assert rm.request("robot2", ["A1-S", "A1-N"], 5, "b", 1)[0] == R.QUEUED      # big request waits
    # robot3 wants only A1-N: free, but robot2 (higher rank) needs it -> must not overtake
    assert rm.request("robot3", ["A1-N"], 50, "c", 2)[0] == R.QUEUED
    # a request in another aisle is unaffected
    assert rm.request("robot3", ["A3-S"], 50, "c", 3)[0] in (R.GRANTED, R.QUEUED)


def test_hold_and_wait_is_denied():
    rm = make_rm()
    assert rm.request("robot1", ["A0-S"], 1, "a", 0)[0] == R.GRANTED
    st, _, why = rm.request("robot1", ["A1-S"], 1, "a", 1)
    assert st == R.DENIED and why == "hold_and_wait"
    assert rm.deadlock_preventions >= 1


def test_expired_requests_and_hold_timeouts():
    rm = make_rm(hold_timeout=50, request_timeout=5)
    rm.request("robot1", ["A0-S"], 1, "a", 0)
    rm.request("robot2", ["A0-S"], 1, "b", 1)
    out = rm.tick(10)           # robot2 never refreshed -> expired
    assert ("robot2", "expired", ["A0-S"]) in out
    rm.request("robot3", ["A0-S"], 1, "c", 20)
    rm.request("robot3", ["A0-S"], 1, "c", 58)          # robot3 keeps refreshing its request
    out = rm.tick(60)           # robot1 holds longer than hold_timeout -> revoked, robot3 gets it
    assert any(o[0] == "robot1" and o[1] == "revoked" for o in out)
    assert rm.owner["A0-S"] == "robot3"
    assert rm.recoveries >= 1


def test_priority_order_and_aging():
    rm = make_rm(aging_rate=1.0)
    rm.request("robot1", ["A0-S"], 0, "a", 0)
    rm.request("robot2", ["A0-S"], 100, "b", 1)        # low urgency, early
    rm.request("robot3", ["A0-S"], 50, "c", 2)         # more urgent, later
    rm.release("robot1", None, 3)
    assert rm.owner["A0-S"] == "robot3"
    rm.release("robot3", None, 4)
    assert rm.owner["A0-S"] == "robot2"


def test_events_are_logged():
    rm = make_rm()
    rm.request("robot1", ["A0-S"], 1, "a", 0)
    rm.request("robot2", ["A0-S"], 1, "b", 1)
    rm.release("robot1", None, 2)
    cats = [e.category for e in rm.drain_events()]
    for c in ("reservation", "wait", "deadlock_prevented", "release"):
        assert c in cats


def test_random_stress_never_double_books_and_never_cycles():
    rng = random.Random(42)
    rm = make_rm(hold_timeout=1e9, request_timeout=1e9)
    robots = ["r1", "r2", "r3", "r4", "r5"]
    holding = {}
    served = {r: 0 for r in robots}
    zones = list(L.ZONES)
    for step in range(4000):
        now = step * 0.5
        r = rng.choice(robots)
        if r in holding and rng.random() < 0.15:
            rm.release(r, None, now)
            del holding[r]
        elif r not in holding:
            aisle = rng.choice(list(L.AISLE_X))
            res = [f"{aisle}-S"] + ([f"{aisle}-N"] if rng.random() < 0.5 else [])
            st, _, _ = rm.request(r, res, rng.uniform(0, 300), "o", now)
            if st == R.GRANTED:
                holding[r] = res
                served[r] += 1
        # grants via queue processing
        for rr in robots:
            if rr not in holding and rm.held_by(rr):
                holding[rr] = rm.held_by(rr)
                served[rr] += 1
        owners = [o for o in rm.owner.values() if o]
        for res, o in rm.owner.items():
            if o:
                assert res in rm.held[o]
        assert rm.find_cycle() == []
        rm.tick(now)
    assert all(v > 20 for v in served.values()), served      # nobody starves


# ------------------------------------------------------------------------------ scheduler
def _robots(n=3, **kw):
    return [S.RobotView(r, *L.bay_pose(r)[:2], **kw) for r in L.ROBOT_IDS[:n]]


def _orders(n, now=0.0, slack=200.0):
    f = O.OrderFactory(seed=1, slack_min=slack, slack_max=slack)
    return [S.OrderView(o.order_id, o.item_id, o.station_id, o.arrival_time, o.deadline)
            for o in (f.new_order(now) for _ in range(n))]


@pytest.mark.parametrize("name", list(S.SCHEDULERS))
def test_no_order_assigned_twice_and_queue_limit(name):
    sch = S.make_scheduler(name)
    robots = _robots()
    res = sch.plan(_orders(10), robots, 0.0)
    ids = [a.order_id for a in res]
    assert len(ids) == len(set(ids))
    per_robot = {}
    for a in res:
        per_robot[a.robot_id] = per_robot.get(a.robot_id, 0) + 1
    assert all(v <= 2 for v in per_robot.values())
    assert len(res) <= 6


def test_deadline_aware_balances_workload():
    sch = S.DeadlineAwareScheduler()
    res = sch.plan(_orders(6), _robots(), 0.0)
    counts = {}
    for a in res:
        counts[a.robot_id] = counts.get(a.robot_id, 0) + 1
    assert len(res) == 6 and set(counts.values()) == {2}


def test_unhealthy_and_stopped_robots_get_nothing():
    sch = S.DeadlineAwareScheduler()
    robots = _robots()
    robots[0].healthy = False
    robots[1].accepting = False
    res = sch.plan(_orders(4), robots, 0.0)
    assert {a.robot_id for a in res} == {"robot3"} and len(res) == 2


def test_orders_stay_queued_if_nobody_is_suitable():
    sch = S.DeadlineAwareScheduler()
    robots = _robots(accepting=False)
    assert sch.plan(_orders(3), robots, 0.0) == []


def test_most_urgent_order_is_planned_first_and_gets_best_robot():
    sch = S.DeadlineAwareScheduler()
    robots = _robots(n=1)
    robots[0].max_queue = 1
    orders = _orders(2)
    orders[0].deadline += 500          # relaxed
    orders[1].deadline = 120           # urgent
    res = sch.plan(orders, robots, 0.0)
    assert len(res) == 1 and res[0].order_id == orders[1].order_id


def test_excluded_robot_is_skipped():
    sch = S.DeadlineAwareScheduler()
    o = _orders(1)[0]
    o.excluded = {"robot1", "robot2"}
    res = sch.plan([o], _robots(), 0.0)
    assert res[0].robot_id == "robot3"


# ------------------------------------------------------------------------------ orders
def test_order_factory_generates_valid_orders():
    f = O.OrderFactory(seed=3)
    seen = set()
    for i in range(60):
        o = f.new_order(float(i))
        assert o.order_id not in seen
        seen.add(o.order_id)
        assert o.item_id in L.ITEMS and o.station_id in L.STATIONS
        assert L.ITEMS[o.item_id].shelf_id == o.shelf_id
        assert o.deadline > o.arrival_time
        assert o.status == O.PENDING
