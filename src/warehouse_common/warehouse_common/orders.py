"""Order generation (pure python)."""
from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Optional

from . import layout as L

PENDING = "PENDING"
ASSIGNED = "ASSIGNED"
IN_PROGRESS = "IN_PROGRESS"
COMPLETED = "COMPLETED"
FAILED = "FAILED"
ACTIVE_STATES = (PENDING, ASSIGNED, IN_PROGRESS)


@dataclass
class OrderRecord:
    order_id: str
    item_id: str
    item_name: str
    shelf_id: str
    aisle_id: str
    station_id: str
    arrival_time: float
    deadline: float
    status: str = PENDING
    assigned_robot: str = ""
    started_time: float = 0.0
    completed_time: float = 0.0
    attempts: int = 0

    @property
    def late(self) -> bool:
        return self.completed_time > self.deadline > 0 if self.completed_time else False


class OrderFactory:
    """Creates random orders: random item (=> shelf), random packing station, deadline slack."""

    def __init__(self, seed: Optional[int] = None, slack_min: float = 150.0, slack_max: float = 260.0) -> None:
        self.rng = random.Random(seed)
        self.counter = 0
        self.slack_min = slack_min
        self.slack_max = slack_max
        self._items = sorted(L.ITEMS)
        self._stations = sorted(L.STATIONS)

    def new_order(self, now: float, unavailable=frozenset()) -> Optional[OrderRecord]:
        """Random order for an item that is on its shelf (not in another active order / being restocked)."""
        choices = [i for i in self._items if i not in unavailable]
        if not choices:
            return None
        self.counter += 1
        item = L.ITEMS[self.rng.choice(choices)]
        return OrderRecord(
            order_id=f"ORD-{self.counter:04d}",
            item_id=item.item_id, item_name=item.name, shelf_id=item.shelf_id, aisle_id=item.aisle,
            station_id=self.rng.choice(self._stations),
            arrival_time=now, deadline=now + self.rng.uniform(self.slack_min, self.slack_max))

    def reset(self) -> None:
        self.counter = 0
