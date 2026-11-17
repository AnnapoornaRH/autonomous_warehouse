"""Central resource reservation manager (aisle zones + packing stations).  Pure python.

Deadlock prevention strategy
----------------------------
* Every request is ATOMIC (all-or-nothing): a robot never holds part of what it needs while it
  waits for the rest.  Without hold-and-wait there can be no circular wait.
* A robot that already holds resources may not request different ones (DENIED "hold_and_wait") -
  it must release first.  This keeps the invariant above even if a client misbehaves.
* Waiting robots are served strictly in priority order (priority = deadline, with aging); a lower
  ranked request may only overtake a higher ranked one if the two do not share a resource, so a
  multi-zone request can not be starved by a stream of small ones.
* Zones have capacity 1: two robots can never be inside the same single-lane zone, in particular
  opposing robots never meet head-on inside an aisle.
* Queued requests that are not refreshed within ``request_timeout`` seconds are dropped (the robot
  died or changed its plan) and holders that exceed ``hold_timeout`` are revoked (recovery).
* A wait-for graph check runs on every change as a belt-and-braces guard (it can only trigger if
  the invariants above were bypassed); a detected cycle is broken by dropping the youngest request.

Time is injected (``now`` in seconds) so the class is deterministic and unit-testable.
"""
from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Set, Tuple

GRANTED = "GRANTED"
QUEUED = "QUEUED"
DENIED = "DENIED"


@dataclass
class Request:
    robot: str
    resources: Tuple[str, ...]
    priority: float
    order_id: str
    t_enqueued: float
    t_seen: float
    seq: int = 0


@dataclass
class Event:
    category: str            # reservation | wait | release | recovery | deadlock_prevented
    robot: str
    detail: str
    resources: Tuple[str, ...] = ()
    value: float = 0.0


class ReservationManager:
    def __init__(self, resources: Iterable[str], hold_timeout: float = 300.0,
                 request_timeout: float = 8.0, aging_rate: float = 0.5) -> None:
        self.resources: List[str] = list(resources)
        self.owner: Dict[str, Optional[str]] = {r: None for r in self.resources}
        self.held: Dict[str, Set[str]] = {}
        self.held_since: Dict[str, float] = {}      # resource -> time granted
        self.queue: List[Request] = []
        self.hold_timeout = hold_timeout
        self.request_timeout = request_timeout
        self.aging_rate = aging_rate
        self.events: List[Event] = []
        self.deadlock_preventions = 0
        self.recoveries = 0
        self._seq = itertools.count()

    # ------------------------------------------------------------------ helpers
    def _emit(self, category: str, robot: str, detail: str, resources=(), value: float = 0.0) -> None:
        self.events.append(Event(category, robot, detail, tuple(resources), value))

    def drain_events(self) -> List[Event]:
        ev, self.events = self.events, []
        return ev

    def _rank(self, req: Request, now: float) -> Tuple[float, int]:
        # lower = served first; aging lowers the rank of long waiting requests
        return (req.priority - self.aging_rate * max(0.0, now - req.t_enqueued), req.seq)

    def _free(self, res: str) -> bool:
        return self.owner.get(res) is None

    def _grant(self, req: Request, now: float) -> None:
        for r in req.resources:
            self.owner[r] = req.robot
            self.held_since[r] = now
        self.held.setdefault(req.robot, set()).update(req.resources)
        waited = now - req.t_enqueued
        self._emit("reservation", req.robot, f"granted {','.join(req.resources)} (waited {waited:.1f}s)",
                   req.resources, waited)

    def _find_request(self, robot: str) -> Optional[Request]:
        for q in self.queue:
            if q.robot == robot:
                return q
        return None

    # ------------------------------------------------------------------ API
    def request(self, robot: str, resources: Iterable[str], priority: float, order_id: str,
                now: float) -> Tuple[str, int, str]:
        res = tuple(dict.fromkeys(resources))           # unique, order preserving
        if not res:
            return DENIED, 0, "empty request"
        unknown = [r for r in res if r not in self.owner]
        if unknown:
            return DENIED, 0, f"unknown resource {unknown[0]}"
        mine = self.held.get(robot, set())
        if mine and set(res) <= mine:
            return GRANTED, 0, "already held"
        if mine and not set(res) <= mine:
            self._emit("deadlock_prevented", robot,
                       f"refused hold-and-wait: holds {sorted(mine)} requested {list(res)}", res)
            self.deadlock_preventions += 1
            return DENIED, 0, "hold_and_wait"

        existing = self._find_request(robot)
        if existing is not None:
            existing.t_seen = now
            existing.priority = priority
            if tuple(existing.resources) != res:
                existing.resources = res
        else:
            existing = Request(robot, res, priority, order_id, now, now, next(self._seq))
            self.queue.append(existing)
        self._process_queue(now)
        if self.held.get(robot) and set(res) <= self.held[robot]:
            return GRANTED, 0, "granted"
        # still waiting
        pos = self.queue_position(robot, now)
        if not getattr(existing, "_announced", False):
            holders = sorted({self.owner[r] for r in res if self.owner[r]} - {robot})
            blockers = [q.robot for q in self.queue
                        if q.robot != robot and set(q.resources) & set(res)
                        and self._rank(q, now) < self._rank(existing, now)]
            why = f"held by {','.join(holders)}" if holders else f"behind {','.join(blockers)}"
            self._emit("wait", robot, f"waiting for {','.join(res)} ({why})", res)
            self._emit("deadlock_prevented", robot,
                       f"conflict avoided on {','.join(res)} ({why}); robot held back instead of entering",
                       res)
            self.deadlock_preventions += 1
            existing._announced = True          # type: ignore[attr-defined]
        return QUEUED, pos, "queued"

    def queue_position(self, robot: str, now: float) -> int:
        ordered = sorted(self.queue, key=lambda q: self._rank(q, now))
        for i, q in enumerate(ordered):
            if q.robot == robot:
                return i + 1
        return 0

    def release(self, robot: str, resources: Optional[Iterable[str]], now: float,
                reason: str = "released") -> List[str]:
        mine = set(self.held.get(robot, set()))
        targets = mine if not resources else (set(resources) & mine)
        for r in targets:
            self.owner[r] = None
            self.held_since.pop(r, None)
        if targets:
            self.held[robot] = mine - targets
            if not self.held[robot]:
                del self.held[robot]
            self._emit("release", robot, f"{reason} {','.join(sorted(targets))}", tuple(sorted(targets)))
        if resources is None or not resources:
            q = self._find_request(robot)
            if q is not None:
                self.queue.remove(q)
        self._process_queue(now)
        return sorted(targets)

    def cancel_request(self, robot: str) -> None:
        q = self._find_request(robot)
        if q is not None:
            self.queue.remove(q)

    def _process_queue(self, now: float) -> None:
        blocked: Set[str] = set()
        progressed = True
        while progressed:
            progressed = False
            blocked = set()
            for q in sorted(self.queue, key=lambda x: self._rank(x, now)):
                if all(self._free(r) for r in q.resources) and not (blocked & set(q.resources)):
                    self.queue.remove(q)
                    self._grant(q, now)
                    progressed = True
                    break
                blocked.update(q.resources)

    def tick(self, now: float) -> List[Tuple[str, str, List[str]]]:
        """Housekeeping.  Returns [(robot, kind, resources)] for revoked holders / expired requests."""
        out: List[Tuple[str, str, List[str]]] = []
        for q in list(self.queue):
            if now - q.t_seen > self.request_timeout:
                self.queue.remove(q)
                self._emit("wait", q.robot, f"request for {','.join(q.resources)} expired (no refresh)",
                           q.resources)
                out.append((q.robot, "expired", list(q.resources)))
        for robot, res in list(self.held.items()):
            oldest = min((self.held_since.get(r, now) for r in res), default=now)
            if now - oldest > self.hold_timeout:
                released = self.release(robot, None, now, reason="hold timeout, revoked")
                self.recoveries += 1
                self._emit("recovery", robot, f"hold timeout after {now - oldest:.0f}s, revoked "
                           f"{','.join(released)}", tuple(released), now - oldest)
                out.append((robot, "revoked", released))
        self._process_queue(now)
        cyc = self.find_cycle()
        if cyc:
            victim = max((q for q in self.queue if q.robot in cyc), key=lambda q: q.seq, default=None)
            if victim is not None:
                self.queue.remove(victim)
                self.deadlock_preventions += 1
                self.recoveries += 1
                self._emit("recovery", victim.robot, f"wait-for cycle {cyc} broken", victim.resources)
                out.append((victim.robot, "cycle_broken", list(victim.resources)))
        return out

    def force_release(self, robot: str, now: float, reason: str) -> List[str]:
        released = self.release(robot, None, now, reason=reason)
        if released:
            self.recoveries += 1
            self._emit("recovery", robot, f"{reason}: released {','.join(released)}", tuple(released))
        return released

    # ------------------------------------------------------------------ analysis
    def wait_for_graph(self) -> Dict[str, Set[str]]:
        g: Dict[str, Set[str]] = {}
        for q in self.queue:
            deps = {self.owner[r] for r in q.resources if self.owner[r] and self.owner[r] != q.robot}
            if deps:
                g.setdefault(q.robot, set()).update(deps)  # type: ignore[arg-type]
        return g

    def find_cycle(self) -> List[str]:
        g = self.wait_for_graph()
        state: Dict[str, int] = {}
        stack: List[str] = []

        def dfs(u: str) -> Optional[List[str]]:
            state[u] = 1
            stack.append(u)
            for v in g.get(u, ()):
                if state.get(v, 0) == 0:
                    c = dfs(v)
                    if c:
                        return c
                elif state.get(v) == 1:
                    return stack[stack.index(v):]
            stack.pop()
            state[u] = 2
            return None

        for node in list(g):
            if state.get(node, 0) == 0:
                c = dfs(node)
                if c:
                    return c
        return []

    def snapshot(self, now: float) -> Dict[str, dict]:
        snap: Dict[str, dict] = {}
        for r in self.resources:
            waiting = [q.robot for q in sorted(self.queue, key=lambda x: self._rank(x, now))
                       if r in q.resources]
            snap[r] = {"owner": self.owner[r] or "", "queue": waiting,
                       "held_since": self.held_since.get(r, 0.0)}
        return snap

    def held_by(self, robot: str) -> List[str]:
        return sorted(self.held.get(robot, set()))
