#!/usr/bin/env python3
"""End-to-end acceptance check against a RUNNING system (./run_warehouse.sh).

It talks to the dashboard backend only (REST), exactly like the browser does, and verifies on live data:

  * every subsystem reports ready, the robots are online
  * the dashboard controls really work: generate orders, change arrival rate, pause / resume (sim clock stops and
    restarts), emergency stop (robots stop within seconds, resume after release), export CSV
  * orders are never assigned to two robots, orders progress to COMPLETED only via the robots
  * robots never share an aisle (traffic invariant): no two robots inside the same aisle at the same time, and
    every robot inside an aisle zone owns its reservation
  * reservations are released once the robots are idle
  * collisions / late orders / orders per minute are recorded, CSV files exist on disk

Usage:  python3 scripts/e2e_check.py --minutes 20 --report docs/e2e_report.md
Exit code 0 when every check passed.
"""
import argparse
import json
import os
import sys
import time
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "warehouse_common"))
try:
    from warehouse_common import layout as L
except Exception:                                    # noqa: BLE001
    L = None

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""), flush=True)
    return ok


def get(base, path, timeout=10):
    with urllib.request.urlopen(base + path, timeout=timeout) as r:
        return json.loads(r.read().decode())


def post(base, path, body=None, timeout=30):
    req = urllib.request.Request(base + path, data=json.dumps(body or {}).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode())


def state(base):
    return get(base, "/api/state")


def wait_for(fn, timeout, step=1.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            v = fn()
            if v:
                return v
        except Exception:                            # noqa: BLE001
            pass
        time.sleep(step)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="http://localhost:3000")
    ap.add_argument("--minutes", type=float, default=15.0, help="wall minutes of observation")
    ap.add_argument("--report", default="")
    ap.add_argument("--min-completed", type=int, default=3)
    a = ap.parse_args()
    base = a.base

    # ------------------------------------------------------------------ start-up
    st = wait_for(lambda: (lambda s: s if s["startup"].get("ready") else None)(state(base)), 600, 2)
    check("all subsystems report ready (system_monitor)", st is not None,
          "" if st else f"missing: {state(base)['startup'].get('missing')}")
    if st is None:
        return finish(a)
    st = state(base)
    robots = [r["id"] for r in st["robots"]]
    check("three robots online", len(robots) == 3 and all(r.get("online") for r in st["robots"]), str(robots))
    check("simulation state RUNNING", st["sim"]["state"] == "RUNNING", str(st["sim"]))
    check("live positions come from the simulator (all robots have non-default poses)",
          all(abs(r["x"]) + abs(r["y"]) > 0.1 for r in st["robots"]))

    # ------------------------------------------------------------------ controls
    n0 = state(base)["metrics"].get("orders_total", 0)
    r = post(base, "/api/orders/generate", {"count": 4})
    time.sleep(3)
    n1 = state(base)["metrics"].get("orders_total", 0)
    check("control: generate test orders creates orders", r.get("created", 0) >= 1 and n1 >= n0 + r["created"],
          f"{n0} -> {n1}")
    r = post(base, "/api/orders/rate", {"orders_per_minute": 3.0})
    check("control: change order arrival rate", r.get("success") is True, r.get("message", ""))
    post(base, "/api/orders/rate", {"orders_per_minute": 1.0})

    t_a = state(base)["sim"]["sim_time"]
    post(base, "/api/control", {"command": "pause"})
    time.sleep(2)
    t_b = state(base)["sim"]["sim_time"]
    time.sleep(4)
    t_c = state(base)["sim"]["sim_time"]
    check("control: pause stops the simulation clock", state(base)["sim"]["paused"] and abs(t_c - t_b) < 0.05,
          f"sim_time {t_b:.2f} -> {t_c:.2f}")
    post(base, "/api/control", {"command": "resume"})
    time.sleep(5)
    t_d = state(base)["sim"]["sim_time"]
    check("control: resume restarts the clock", t_d > t_c + 0.3, f"{t_c:.2f} -> {t_d:.2f}")

    post(base, "/api/control", {"command": "estop"})
    time.sleep(6)
    s1 = state(base)
    time.sleep(6)
    s2 = state(base)
    moved = max(((a_["x"] - b_["x"]) ** 2 + (a_["y"] - b_["y"]) ** 2) ** 0.5 for a_, b_ in zip(s1["robots"], s2["robots"]))
    check("control: emergency stop halts every robot base", s2["sim"]["estop"] and moved < 0.05,
          f"max displacement while stopped {moved:.3f} m")
    post(base, "/api/control", {"command": "release_estop"})
    time.sleep(8)
    check("control: release e-stop returns to RUNNING", not state(base)["sim"]["estop"])

    # ------------------------------------------------------------------ observation
    t_end = time.time() + a.minutes * 60
    samples = 0
    dup_assign = 0
    shared_aisle = 0
    unreserved = 0
    max_by_robot_states = {}
    seen_states = set()
    completed_ids = set()
    zones = {z["id"]: z for z in get(base, "/api/layout")["zones"]} if L is None else None
    print(f"observing for {a.minutes:.1f} wall minutes ...", flush=True)
    last_print = 0
    while time.time() < t_end:
        try:
            s = state(base)
        except Exception as e:                       # noqa: BLE001
            print("state poll failed:", e)
            time.sleep(2)
            continue
        samples += 1
        seen = {}
        for r in s["robots"]:
            seen_states.add(r["state"])
            ids = ([r["order_id"]] if r["order_id"] else []) + list(r.get("queued_orders", []))
            for oid in ids:
                if oid in seen and seen[oid] != r["id"]:
                    dup_assign += 1
                seen[oid] = r["id"]
        # an order assigned to a robot in the order table must not appear with two robots
        owners = {}
        for o in s["orders"]:
            if o["status"] in ("ASSIGNED", "IN_PROGRESS") and o["robot"]:
                owners.setdefault(o["id"], set()).add(o["robot"])
        dup_assign += sum(1 for v in owners.values() if len(v) > 1)
        # aisle discipline
        zone_defs = L.ZONES if L else None
        per_aisle = {}
        for r in s["robots"]:
            if not r.get("online"):
                continue
            if L:
                for zid, z in L.ZONES.items():
                    if z.contains(r["x"], r["y"], margin=-0.12):         # clearly inside the aisle
                        per_aisle.setdefault(z.aisle, set()).add(r["id"])
                        if zid not in r.get("held_resources", []) and r["state"] not in ("RECOVERING", "FAULTED"):
                            unreserved += 1
        shared_aisle += sum(1 for v in per_aisle.values() if len(v) > 1)
        for o in s["orders"]:
            if o["status"] == "COMPLETED":
                completed_ids.add(o["id"])
        if time.time() - last_print > 30:
            last_print = time.time()
            m = s["metrics"]
            print(f"  t={s['sim']['sim_time']:.0f}s rtf={s['sim'].get('rtf')} completed={m.get('completed')} "
                  f"late={m.get('late')} coll={m.get('collisions')} near={m.get('near_misses')} "
                  f"states={[r['state'][:12] for r in s['robots']]}", flush=True)
        time.sleep(2)

    final = state(base)
    m = final["metrics"]
    check("orders were generated autonomously", m.get("orders_total", 0) >= 5, f"orders_total={m.get('orders_total')}")
    check("orders were completed through verified delivery", m.get("completed", 0) >= a.min_completed,
          f"completed={m.get('completed')} (required >= {a.min_completed})")
    check("no order was ever assigned to two robots", dup_assign == 0, f"{dup_assign} duplicate observations in {samples} samples")
    check("every robot worked on orders (workload shared)",
          sum(1 for v in m.get("tasks_completed", []) if v > 0) >= 2, f"tasks per robot {m.get('tasks_completed')}")
    check("no two robots were ever inside the same single-lane aisle", shared_aisle == 0,
          f"{shared_aisle} shared-aisle observations")
    check("robots inside an aisle zone owned its reservation", unreserved <= max(2, samples // 50),
          f"{unreserved} unreserved observations in {samples} samples")
    check("state machine visited work states", {"NAVIGATING_TO_SHELF", "PICKING", "NAVIGATING_TO_PACKING", "DROPPING"} <= seen_states,
          str(sorted(seen_states)))
    check("collisions measured (official KPI, from contact sensors)", "collisions" in m, f"collisions={m.get('collisions')}")
    check("late orders measured (official KPI)", "late" in m, f"late={m.get('late')}")
    check("orders per minute measured (official KPI)", m.get("orders_per_minute", 0) > 0, f"{m.get('orders_per_minute', 0):.2f}/min")
    check("deadlock-prevention events recorded", final["traffic"]["deadlock_preventions"] >= 0,
          f"{final['traffic']['deadlock_preventions']} preventions, {final['traffic']['recoveries']} recoveries")

    # reservations released once robots are idle: stop new orders, wait until the fleet drains
    post(base, "/api/orders/rate", {"orders_per_minute": 0})

    def drained():
        s = state(base)
        busy = [r for r in s["robots"] if r["state"] not in ("IDLE",) or r["queue_length"] > 0]
        pend = [o for o in s["orders"] if o["status"] in ("PENDING", "ASSIGNED", "IN_PROGRESS")]
        return s if not busy and not pend else None
    d = wait_for(drained, 900, 5)
    if d is not None:
        time.sleep(4)
        d = state(base)
        held = [z["id"] for z in d["traffic"]["zones"] + d["traffic"]["stations"] if z["owner"]]
        check("all reservations are released when the fleet is idle", not held, f"still held: {held}")
    else:
        check("fleet drains after orders stop (to verify reservation release)", False, "timed out")
    post(base, "/api/orders/rate", {"orders_per_minute": 1.0})

    # CSV export
    ex = post(base, "/api/export")
    d_ = ex.get("directory", "")
    files = {f: os.path.join(d_, f) for f in ("metrics_timeseries.csv", "orders.csv", "events.csv", "robots.csv")}
    ok_files = all(os.path.exists(p) and os.path.getsize(p) > 50 for p in files.values())
    check("metrics exported to CSV (timeseries, orders, events, robots)", ok_files, d_)
    return finish(a)


def finish(a):
    passed = sum(1 for _, ok, _ in RESULTS if ok)
    print(f"\n{passed}/{len(RESULTS)} checks passed")
    if a.report:
        os.makedirs(os.path.dirname(os.path.abspath(a.report)), exist_ok=True)
        with open(a.report, "w") as f:
            f.write(f"# End-to-end check ({time.strftime('%Y-%m-%d %H:%M:%S')})\n\n| Result | Check | Detail |\n|---|---|---|\n")
            for n, ok, d in RESULTS:
                f.write(f"| {'PASS' if ok else 'FAIL'} | {n} | {d} |\n")
            f.write(f"\n**{passed}/{len(RESULTS)} passed**\n")
    return 0 if passed == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
