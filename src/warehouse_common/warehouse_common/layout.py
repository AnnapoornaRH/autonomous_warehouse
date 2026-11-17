"""Single source of truth for the warehouse geometry.

Everything that needs to know where a shelf, aisle, packing station, bay or item
is (world generator, map generator, task allocator, traffic manager, robot agent,
dashboard) imports this module.  All units are metres / radians, world frame
(x east, y north, z up, yaw counter-clockwise from +x).

Floor plan (top view, not to scale)::

    y=+5.5  +----------------------------------------------+
            |             north cross-aisle                |
    y=+3.5  +------+-----+------+-----+------+-----+------+
            | A0   | B1  |  A1  | B2  |  A2  | B3  |  A3  |   4 single-lane aisles
            | -N   |     | -N   |     | -N   |     | -N   |   (each split into a
    y= 0.0  |------|     |------|     |------|     |------|    north and a south
            | A0   |     |  A1  |     |  A2  |     |  A3  |    managed traffic zone)
            | -S   |     | -S   |     | -S   |     | -S   |
    y=-3.5  +------+-----+------+-----+------+-----+------+
            |             south cross-aisle (bays)         |
            |   [PK1]          [PK2]          [PK3]        |   packing stations
    y=-7.5  +----------------------------------------------+
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Tuple

# --------------------------------------------------------------------------
# Global constants
# --------------------------------------------------------------------------
AISLE_W = 1.6          # clear width of a narrow aisle
BLOCK_D = 0.9          # depth (x) of a shelf block
X_MIN, X_MAX = -4.55, 4.55
Y_MIN, Y_MAX = -7.5, 5.5
SHELF_Y0, SHELF_Y1 = -3.5, 3.5      # shelf blocks / aisles extend along y
SHELF_TOP_Z = 0.40                  # top surface of every shelf / table body
ITEM_H = 0.08                       # all items are 8 cm tall
ITEM_W = 0.06                       # ... and 6 cm wide (grip width)
ITEM_Z = SHELF_TOP_Z + ITEM_H / 2.0 + 0.002
ITEM_INSET = 0.06                   # item centre is this far inside the shelf edge
STAND_OFF = 0.45                    # base_link centre -> shelf/table front edge
ITEM_SLOT_Y = (-1.8, 1.8)           # y of the two item slots on every shelf face
TABLE_DEPTH = 0.65
TABLE_WIDTH = 1.0
MAP_RESOLUTION = 0.05
WORLD_NAME = "warehouse"

# --------------------------------------------------------------------------
# Aisles, blocks, zones
# --------------------------------------------------------------------------
AISLE_X: Dict[str, float] = {"A0": -3.75, "A1": -1.25, "A2": 1.25, "A3": 3.75}
BLOCK_X: Dict[str, float] = {"B1": -2.5, "B2": 0.0, "B3": 2.5}


@dataclass(frozen=True)
class Zone:
    zone_id: str
    aisle: str
    half: str                   # "S" or "N"
    x0: float
    x1: float
    y0: float
    y1: float

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (self.x0 - margin <= x <= self.x1 + margin and
                self.y0 - margin <= y <= self.y1 + margin)

    @property
    def center(self) -> Tuple[float, float]:
        return ((self.x0 + self.x1) / 2.0, (self.y0 + self.y1) / 2.0)


def _build_zones() -> Dict[str, Zone]:
    zones: Dict[str, Zone] = {}
    for aisle, ax in AISLE_X.items():
        hw = AISLE_W / 2.0
        zones[f"{aisle}-S"] = Zone(f"{aisle}-S", aisle, "S", ax - hw, ax + hw, SHELF_Y0, 0.0)
        zones[f"{aisle}-N"] = Zone(f"{aisle}-N", aisle, "N", ax - hw, ax + hw, 0.0, SHELF_Y1)
    return zones


ZONES: Dict[str, Zone] = _build_zones()

# --------------------------------------------------------------------------
# Shelves and items
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Shelf:
    shelf_id: str
    block: str
    side: str                   # "W" (face looks west) or "E"
    aisle: str
    face_x: float               # x of the front edge of the shelf
    pick_yaw: float             # yaw the robot needs to face the shelf


def _build_shelves() -> Dict[str, Shelf]:
    spec = [("S1", "B1", "W", "A0"), ("S2", "B1", "E", "A1"), ("S3", "B2", "W", "A1"),
            ("S4", "B2", "E", "A2"), ("S5", "B3", "W", "A2"), ("S6", "B3", "E", "A3")]
    out: Dict[str, Shelf] = {}
    for sid, block, side, aisle in spec:
        cx = BLOCK_X[block]
        if side == "W":
            out[sid] = Shelf(sid, block, side, aisle, cx - BLOCK_D / 2.0, 0.0)
        else:
            out[sid] = Shelf(sid, block, side, aisle, cx + BLOCK_D / 2.0, math.pi)
    return out


SHELVES: Dict[str, Shelf] = _build_shelves()

# (name, shape, rgba) -- 12 items, two per shelf face
_ITEM_STYLES = [
    ("Red parcel", "box", (0.85, 0.15, 0.15, 1)),
    ("Blue parcel", "box", (0.15, 0.35, 0.90, 1)),
    ("Yellow crate", "box", (0.95, 0.80, 0.10, 1)),
    ("Green canister", "cylinder", (0.15, 0.70, 0.30, 1)),
    ("Orange tube", "cylinder", (0.95, 0.50, 0.10, 1)),
    ("Purple box", "box", (0.55, 0.25, 0.75, 1)),
]


@dataclass(frozen=True)
class Item:
    item_id: str
    name: str
    shape: str
    rgba: Tuple[float, float, float, float]
    shelf_id: str
    slot: int                   # 0 -> south half of the aisle, 1 -> north half
    x: float
    y: float
    z: float

    @property
    def aisle(self) -> str:
        return SHELVES[self.shelf_id].aisle

    @property
    def zone_id(self) -> str:
        return f"{self.aisle}-{'S' if self.slot == 0 else 'N'}"


def _build_items() -> Dict[str, Item]:
    items: Dict[str, Item] = {}
    n = 0
    for sid, shelf in SHELVES.items():
        for slot, y in enumerate(ITEM_SLOT_Y):
            style = _ITEM_STYLES[n % len(_ITEM_STYLES)]
            n += 1
            x = shelf.face_x + (ITEM_INSET if shelf.side == "W" else -ITEM_INSET)
            iid = f"item_{n:02d}"
            items[iid] = Item(iid, style[0], style[1], style[2], sid, slot, x, y, ITEM_Z)
    return items


ITEMS: Dict[str, Item] = _build_items()

# --------------------------------------------------------------------------
# Packing stations, bays, robots, obstacles
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Station:
    station_id: str
    x: float                    # table centre x
    table_y0: float
    table_y1: float
    drop_x: float               # robot base_link pose while dropping
    drop_y: float
    drop_yaw: float
    place_x: float              # where the object is put down (table top)
    place_y: float
    place_z: float


def _build_stations() -> Dict[str, Station]:
    out: Dict[str, Station] = {}
    front_y = Y_MIN + TABLE_DEPTH
    for i, x in enumerate((-2.5, 0.0, 2.5), start=1):
        out[f"PK{i}"] = Station(
            f"PK{i}", x, Y_MIN, front_y,
            x, front_y + STAND_OFF, -math.pi / 2.0,
            x, front_y - ITEM_INSET - 0.04, ITEM_Z)
    return out


STATIONS: Dict[str, Station] = _build_stations()

ROBOT_IDS: List[str] = ["robot1", "robot2", "robot3"]
BAYS: Dict[str, Tuple[float, float, float]] = {
    "robot1": (-3.75, -5.0, math.pi / 2.0),
    "robot2": (-1.25, -5.0, math.pi / 2.0),
    "robot3": (3.75, -5.0, math.pi / 2.0),
}
# validated categorical slots 1-3 (dark surface): blue, orange, aqua  -> same colours in Gazebo and dashboard
ROBOT_COLORS: Dict[str, str] = {"robot1": "#3987e5", "robot2": "#d95926", "robot3": "#199e70"}


@dataclass(frozen=True)
class Obstacle:
    name: str
    shape: str                  # "box" | "cylinder"
    x: float
    y: float
    sx: float                   # box: size x ; cylinder: radius
    sy: float                   # box: size y ; cylinder: unused
    h: float
    rgba: Tuple[float, float, float, float]


OBSTACLES: List[Obstacle] = [
    Obstacle("crate_stack_1", "box", -2.5, -4.3, 0.70, 0.70, 0.55, (0.55, 0.38, 0.20, 1)),
    Obstacle("crate_stack_2", "box", 2.5, -4.3, 0.70, 0.70, 0.55, (0.55, 0.38, 0.20, 1)),
    Obstacle("barrel_1", "cylinder", 0.0, 4.5, 0.30, 0.0, 0.70, (0.80, 0.20, 0.10, 1)),
    Obstacle("barrel_2", "cylinder", 3.2, 4.6, 0.28, 0.0, 0.70, (0.80, 0.20, 0.10, 1)),
    Obstacle("pallet_1", "box", -2.5, 4.4, 0.80, 0.80, 0.45, (0.60, 0.45, 0.25, 1)),
]

# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def item_pick_pose(item_id: str) -> Tuple[float, float, float]:
    """base_link pose (x, y, yaw) from which the arm can reach the item."""
    it = ITEMS[item_id]
    sh = SHELVES[it.shelf_id]
    if sh.side == "W":
        return (sh.face_x - STAND_OFF, it.y, sh.pick_yaw)
    return (sh.face_x + STAND_OFF, it.y, sh.pick_yaw)


def station_drop_pose(station_id: str) -> Tuple[float, float, float]:
    st = STATIONS[station_id]
    return (st.drop_x, st.drop_y, st.drop_yaw)


def station_place_point(station_id: str) -> Tuple[float, float, float]:
    st = STATIONS[station_id]
    return (st.place_x, st.place_y, st.place_z)


def bay_pose(robot_id: str) -> Tuple[float, float, float]:
    return BAYS[robot_id]


def zones_for_item(item_id: str) -> List[str]:
    """Zones a robot must own to fetch the item (entering and leaving from the south).

    Items in the south half need only the south zone, items in the north half need
    the south zone (to drive through) and the north zone.
    """
    it = ITEMS[item_id]
    if it.slot == 0:
        return [f"{it.aisle}-S"]
    return [f"{it.aisle}-S", f"{it.aisle}-N"]


def zone_at(x: float, y: float, margin: float = 0.0) -> Optional[str]:
    for zid, z in ZONES.items():
        if z.contains(x, y, margin):
            return zid
    return None


def aisle_mouth(aisle: str, side: str = "S") -> Tuple[float, float]:
    return (AISLE_X[aisle], SHELF_Y0 if side == "S" else SHELF_Y1)


def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def travel_distance_to_item(from_xy: Tuple[float, float], item_id: str) -> float:
    """Approximate drive distance from a point in the south cross-aisle to the pick pose."""
    px, py, _ = item_pick_pose(item_id)
    mouth = aisle_mouth(ITEMS[item_id].aisle, "S")
    entry = (mouth[0], mouth[1] - 0.6)
    if zone_at(from_xy[0], from_xy[1], 0.1) is not None:   # already inside some aisle
        return _dist(from_xy, (px, py)) + 0.5
    return _dist(from_xy, entry) + _dist(entry, (px, py))


def travel_distance_item_to_station(item_id: str, station_id: str) -> float:
    px, py, _ = item_pick_pose(item_id)
    mouth = aisle_mouth(ITEMS[item_id].aisle, "S")
    exit_pt = (mouth[0], mouth[1] - 0.6)
    dx, dy, _ = station_drop_pose(station_id)
    return _dist((px, py), exit_pt) + _dist(exit_pt, (dx, dy))


def zone_footprint_rects() -> List[Tuple[float, float, float, float]]:
    """Solid rectangles (x0, y0, x1, y1) that make up the static map."""
    rects: List[Tuple[float, float, float, float]] = []
    t = 0.2
    rects.append((X_MIN - t, Y_MIN - t, X_MAX + t, Y_MIN))   # south wall
    rects.append((X_MIN - t, Y_MAX, X_MAX + t, Y_MAX + t))   # north wall
    rects.append((X_MIN - t, Y_MIN - t, X_MIN, Y_MAX + t))   # west wall
    rects.append((X_MAX, Y_MIN - t, X_MAX + t, Y_MAX + t))   # east wall
    for bx in BLOCK_X.values():
        rects.append((bx - BLOCK_D / 2, SHELF_Y0, bx + BLOCK_D / 2, SHELF_Y1))
    for st in STATIONS.values():
        rects.append((st.x - TABLE_WIDTH / 2, st.table_y0, st.x + TABLE_WIDTH / 2, st.table_y1))
    for ob in OBSTACLES:
        if ob.shape == "box":
            rects.append((ob.x - ob.sx / 2, ob.y - ob.sy / 2, ob.x + ob.sx / 2, ob.y + ob.sy / 2))
        else:
            r = ob.sx
            rects.append((ob.x - r, ob.y - r, ob.x + r, ob.y + r))
    return rects


def layout_dict() -> dict:
    """JSON-serialisable description used by the dashboard map."""
    return {
        "bounds": {"x_min": X_MIN, "x_max": X_MAX, "y_min": Y_MIN, "y_max": Y_MAX},
        "aisle_width": AISLE_W,
        "blocks": [{"id": b, "x0": x - BLOCK_D / 2, "x1": x + BLOCK_D / 2,
                    "y0": SHELF_Y0, "y1": SHELF_Y1} for b, x in BLOCK_X.items()],
        "aisles": [{"id": a, "x": x} for a, x in AISLE_X.items()],
        "zones": [{"id": z.zone_id, "aisle": z.aisle, "half": z.half,
                   "x0": z.x0, "x1": z.x1, "y0": z.y0, "y1": z.y1} for z in ZONES.values()],
        "shelves": [{"id": s.shelf_id, "block": s.block, "side": s.side, "aisle": s.aisle,
                     "face_x": s.face_x} for s in SHELVES.values()],
        "items": [{"id": i.item_id, "name": i.name, "shape": i.shape, "shelf": i.shelf_id,
                   "slot": i.slot, "x": i.x, "y": i.y,
                   "rgba": list(i.rgba)} for i in ITEMS.values()],
        "stations": [{"id": s.station_id, "x": s.x, "y0": s.table_y0, "y1": s.table_y1,
                      "width": TABLE_WIDTH, "drop": [s.drop_x, s.drop_y, s.drop_yaw]}
                     for s in STATIONS.values()],
        "bays": [{"robot": r, "x": p[0], "y": p[1], "yaw": p[2]} for r, p in BAYS.items()],
        "obstacles": [asdict(o) for o in OBSTACLES],
        "robots": [{"id": r, "color": ROBOT_COLORS[r]} for r in ROBOT_IDS],
    }
