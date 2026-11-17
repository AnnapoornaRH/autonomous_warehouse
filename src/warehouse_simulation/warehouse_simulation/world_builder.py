"""Generates the Gazebo (SDF) world and the Nav2 occupancy map from warehouse_common.layout.

Usage:  python3 -m warehouse_simulation.world_builder <world.sdf> <maps_dir>
(also called by scripts/generate_assets.py)
"""
from __future__ import annotations

import math
import os
import sys
from typing import List, Tuple

from warehouse_common import layout as L

WALL_H = 1.2
WALL_T = 0.2


def _mat(rgba, emissive=False) -> str:
    r, g, b, a = rgba
    em = f"<emissive>{r*0.35:.3f} {g*0.35:.3f} {b*0.35:.3f} 1</emissive>" if emissive else ""
    return (f"<material><ambient>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</ambient>"
            f"<diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular>{em}</material>")


def _box_geom(sx, sy, sz) -> str:
    return f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>"


def _static_box(name, cx, cy, cz, sx, sy, sz, rgba, collide=True) -> str:
    col = (f'<collision name="{name}_col">{_box_geom(sx, sy, sz)}</collision>' if collide else "")
    return f"""
    <model name="{name}"><static>true</static><pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>
      <link name="link">
        {col}
        <visual name="{name}_vis">{_box_geom(sx, sy, sz)}{_mat(rgba)}</visual>
      </link></model>"""


def _visual_box(name, cx, cy, cz, sx, sy, sz, rgba, emissive=False) -> str:
    return (f'<visual name="{name}"><pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>'
            f'{_box_geom(sx, sy, sz)}{_mat(rgba, emissive)}</visual>')


def _floor_markings() -> str:
    vis: List[str] = []
    # aisle floors: alternate tint per managed zone
    for zid, z in L.ZONES.items():
        cx, cy = z.center
        tint = (0.30, 0.42, 0.55, 1) if z.half == "S" else (0.34, 0.46, 0.60, 1)
        vis.append(_visual_box(f"floor_{zid}", cx, cy, 0.003, z.x1 - z.x0 - 0.02,
                               z.y1 - z.y0 - 0.02, 0.004, tint))
        # yellow centre line of the aisle
        vis.append(_visual_box(f"line_{zid}", cx, cy, 0.006, 0.04, z.y1 - z.y0 - 0.2, 0.002,
                               (0.95, 0.80, 0.10, 1)))
    # intersections / cross aisle tint
    vis.append(_visual_box("cross_south", 0.0, (L.Y_MIN + L.SHELF_Y0) / 2, 0.002,
                           L.X_MAX - L.X_MIN - 0.02, L.SHELF_Y0 - L.Y_MIN - 0.02, 0.003,
                           (0.46, 0.48, 0.52, 1)))
    vis.append(_visual_box("cross_north", 0.0, (L.Y_MAX + L.SHELF_Y1) / 2, 0.002,
                           L.X_MAX - L.X_MIN - 0.02, L.Y_MAX - L.SHELF_Y1 - 0.02, 0.003,
                           (0.46, 0.48, 0.52, 1)))
    # packing area (green)
    vis.append(_visual_box("pack_area", 0.0, L.Y_MIN + 0.95, 0.004, L.X_MAX - L.X_MIN - 0.6, 1.5,
                           0.002, (0.20, 0.55, 0.35, 1)))
    # robot bays (yellow squares)
    for rid, (bx, by, _) in L.BAYS.items():
        vis.append(_visual_box(f"bay_{rid}", bx, by, 0.006, 0.62, 0.62, 0.002, (0.95, 0.80, 0.10, 1)))
        vis.append(_visual_box(f"bay_in_{rid}", bx, by, 0.007, 0.54, 0.54, 0.002, (0.30, 0.32, 0.36, 1)))
    # aisle mouth stop bars
    for a, ax in L.AISLE_X.items():
        for y in (L.SHELF_Y0 - 0.12, L.SHELF_Y1 + 0.12):
            vis.append(_visual_box(f"stop_{a}_{'s' if y < 0 else 'n'}", ax, y, 0.006,
                                   L.AISLE_W - 0.1, 0.06, 0.002, (0.90, 0.20, 0.15, 1)))
    return f"""
    <model name="floor_markings"><static>true</static><pose>0 0 0 0 0 0</pose>
      <link name="link">{''.join(vis)}</link></model>"""


def _shelf_block(bid: str, cx: float) -> str:
    y0, y1 = L.SHELF_Y0, L.SHELF_Y1
    ly = (y1 - y0)
    cy = (y0 + y1) / 2
    d = L.BLOCK_D
    body_h = L.SHELF_TOP_Z
    vis = [_visual_box(f"{bid}_body", 0, 0, body_h / 2, d, ly, body_h, (0.16, 0.30, 0.55, 1)),
           _visual_box(f"{bid}_top", 0, 0, body_h + 0.01, d, ly, 0.02, (0.70, 0.72, 0.76, 1))]
    # decorative upper tiers (visual only): posts + two decks
    for k, zdeck in enumerate((0.85, 1.30)):
        vis.append(_visual_box(f"{bid}_deck{k}", 0, 0, zdeck, d, ly, 0.03, (0.78, 0.80, 0.84, 1)))
    for px in (-d / 2 + 0.03, d / 2 - 0.03):
        for j in range(8):
            py = -ly / 2 + 0.03 + j * (ly - 0.06) / 7
            vis.append(_visual_box(f"{bid}_post_{px:+.2f}_{j}", px, py, 0.87, 0.05, 0.05, 0.95,
                                   (0.95, 0.55, 0.10, 1)))
    # lower-front orange stripe so shelves read as shelves
    for sgn in (-1, 1):
        vis.append(_visual_box(f"{bid}_stripe_{sgn}", sgn * (d / 2 + 0.002), 0, 0.30, 0.004, ly, 0.05,
                               (0.95, 0.55, 0.10, 1)))
    return f"""
    <model name="shelf_{bid}"><static>true</static><pose>{cx:.4f} {cy:.4f} 0 0 0 0</pose>
      <link name="link">
        <collision name="{bid}_col"><pose>0 0 {body_h/2:.4f} 0 0 0</pose>{_box_geom(d, ly, body_h)}</collision>
        {''.join(vis)}
      </link></model>"""


def _station(st: L.Station) -> str:
    h = L.SHELF_TOP_Z
    w = L.TABLE_WIDTH
    d = st.table_y1 - st.table_y0
    cy = (st.table_y0 + st.table_y1) / 2
    vis = [_visual_box("body", 0, 0, h / 2, w, d, h, (0.12, 0.45, 0.30, 1)),
           _visual_box("top", 0, 0, h + 0.01, w, d, 0.02, (0.85, 0.88, 0.90, 1)),
           _visual_box("target", 0, -d / 2 + L.ITEM_INSET + 0.04, h + 0.022, 0.14, 0.14, 0.004,
                       (0.15, 0.85, 0.45, 1), emissive=True),
           _visual_box("sign", 0, d / 2 - 0.02, 1.05, w, 0.04, 0.30, (0.10, 0.60, 0.40, 1), emissive=True),
           _visual_box("post_l", -w / 2 + 0.03, d / 2 - 0.02, 0.6, 0.05, 0.05, 1.2, (0.95, 0.55, 0.10, 1)),
           _visual_box("post_r", w / 2 - 0.03, d / 2 - 0.02, 0.6, 0.05, 0.05, 1.2, (0.95, 0.55, 0.10, 1))]
    return f"""
    <model name="station_{st.station_id}"><static>true</static><pose>{st.x:.4f} {cy:.4f} 0 0 0 0</pose>
      <link name="link">
        <collision name="col"><pose>0 0 {h/2:.4f} 0 0 0</pose>{_box_geom(w, d, h)}</collision>
        {''.join(vis)}
      </link></model>"""


def _obstacle(ob: L.Obstacle) -> str:
    if ob.shape == "box":
        geom = _box_geom(ob.sx, ob.sy, ob.h)
    else:
        geom = f"<geometry><cylinder><radius>{ob.sx:.3f}</radius><length>{ob.h:.3f}</length></cylinder></geometry>"
    return f"""
    <model name="{ob.name}"><static>true</static><pose>{ob.x:.4f} {ob.y:.4f} {ob.h/2:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="col">{geom}</collision>
        <visual name="vis">{geom}{_mat(ob.rgba)}</visual>
      </link></model>"""


def _item(it: L.Item) -> str:
    m = 0.08
    if it.shape == "box":
        geom = _box_geom(L.ITEM_W, L.ITEM_W, L.ITEM_H)
        ixx = iyy = m / 12 * (L.ITEM_W ** 2 + L.ITEM_H ** 2)
        izz = m / 12 * (2 * L.ITEM_W ** 2)
        iyy = ixx
    else:
        r = L.ITEM_W / 2
        geom = f"<geometry><cylinder><radius>{r:.4f}</radius><length>{L.ITEM_H:.4f}</length></cylinder></geometry>"
        ixx = iyy = m / 12 * (3 * r ** 2 + L.ITEM_H ** 2)
        izz = 0.5 * m * r ** 2
    return f"""
    <model name="{it.item_id}"><pose>{it.x:.4f} {it.y:.4f} {it.z:.4f} 0 0 0</pose>
      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose><publish_model_pose>true</publish_model_pose>
        <publish_nested_model_pose>false</publish_nested_model_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg><static_publisher>false</static_publisher>
        <update_frequency>4</update_frequency>
      </plugin>
      <link name="link">
        <inertial><mass>{m}</mass><inertia><ixx>{ixx:.7f}</ixx><iyy>{iyy:.7f}</iyy><izz>{izz:.7f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="col">{geom}
          <surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction></surface></collision>
        <visual name="vis">{geom}{_mat(it.rgba)}</visual>
      </link></model>"""


def build_world_sdf() -> str:
    parts: List[str] = []
    parts.append(f"""<?xml version="1.0" ?>
<!-- GENERATED by warehouse_simulation.world_builder from warehouse_common.layout. Do not edit by hand. -->
<sdf version="1.9">
  <world name="{L.WORLD_NAME}">
    <physics name="dart_physics" type="dart">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">
      <dynamic_pose_hertz>20</dynamic_pose_hertz>
    </plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <scene>
      <ambient>0.55 0.55 0.58 1</ambient>
      <background>0.10 0.12 0.16 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -1</direction>
    </light>
    <light type="point" name="overhead_1">
      <cast_shadows>false</cast_shadows><pose>0 -1 5 0 0 0</pose>
      <diffuse>0.5 0.5 0.55 1</diffuse><specular>0.05 0.05 0.05 1</specular>
      <attenuation><range>25</range><constant>0.6</constant><linear>0.02</linear><quadratic>0.0</quadratic></attenuation>
    </light>

    <model name="ground"><static>true</static><pose>0 0 -0.05 0 0 0</pose>
      <link name="link">
        <collision name="col">{_box_geom(40, 40, 0.1)}
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="vis">{_box_geom(40, 40, 0.1)}{_mat((0.22, 0.24, 0.27, 1))}</visual>
      </link></model>
""")
    # floor of the warehouse itself
    cx = (L.X_MIN + L.X_MAX) / 2
    cy = (L.Y_MIN + L.Y_MAX) / 2
    parts.append(_static_box("warehouse_floor", cx, cy, 0.001, L.X_MAX - L.X_MIN, L.Y_MAX - L.Y_MIN,
                             0.002, (0.52, 0.54, 0.58, 1), collide=False))
    parts.append(_floor_markings())
    # perimeter walls
    sx = L.X_MAX - L.X_MIN + 2 * WALL_T
    sy = L.Y_MAX - L.Y_MIN + 2 * WALL_T
    wall_rgba = (0.78, 0.80, 0.84, 1)
    parts.append(_static_box("wall_south", cx, L.Y_MIN - WALL_T / 2, WALL_H / 2, sx, WALL_T, WALL_H, wall_rgba))
    parts.append(_static_box("wall_north", cx, L.Y_MAX + WALL_T / 2, WALL_H / 2, sx, WALL_T, WALL_H, wall_rgba))
    parts.append(_static_box("wall_west", L.X_MIN - WALL_T / 2, cy, WALL_H / 2, WALL_T, sy, WALL_H, wall_rgba))
    parts.append(_static_box("wall_east", L.X_MAX + WALL_T / 2, cy, WALL_H / 2, WALL_T, sy, WALL_H, wall_rgba))
    for bid, bx in L.BLOCK_X.items():
        parts.append(_shelf_block(bid, bx))
    for st in L.STATIONS.values():
        parts.append(_station(st))
    for ob in L.OBSTACLES:
        parts.append(_obstacle(ob))
    for it in L.ITEMS.values():
        parts.append(_item(it))
    parts.append("\n  </world>\n</sdf>\n")
    return "".join(parts)


# --------------------------------------------------------------------------
# Occupancy map for Nav2
# --------------------------------------------------------------------------

def build_map(res: float = L.MAP_RESOLUTION, pad: float = 0.5) -> Tuple[bytes, int, int, float, float]:
    ox = L.X_MIN - 0.2 - pad
    oy = L.Y_MIN - 0.2 - pad
    w = int(math.ceil((L.X_MAX - L.X_MIN + 0.4 + 2 * pad) / res))
    h = int(math.ceil((L.Y_MAX - L.Y_MIN + 0.4 + 2 * pad) / res))
    # 205 = unknown (outside), 254 = free, 0 = occupied
    img = [[205] * w for _ in range(h)]
    # free inside the walls
    for j in range(h):
        y = oy + (j + 0.5) * res
        for i in range(w):
            x = ox + (i + 0.5) * res
            if L.X_MIN <= x <= L.X_MAX and L.Y_MIN <= y <= L.Y_MAX:
                img[j][i] = 254
    for (x0, y0, x1, y1) in L.zone_footprint_rects():
        i0 = max(0, int(math.floor((x0 - ox) / res)))
        i1 = min(w - 1, int(math.ceil((x1 - ox) / res)) - 1)
        j0 = max(0, int(math.floor((y0 - oy) / res)))
        j1 = min(h - 1, int(math.ceil((y1 - oy) / res)) - 1)
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                img[j][i] = 0
    # PGM rows go top (max y) to bottom (min y)
    rows = bytearray()
    for j in range(h - 1, -1, -1):
        rows.extend(bytes(img[j]))
    return bytes(rows), w, h, ox, oy


def write_assets(world_path: str, maps_dir: str) -> None:
    os.makedirs(os.path.dirname(world_path), exist_ok=True)
    with open(world_path, "w") as f:
        f.write(build_world_sdf())
    os.makedirs(maps_dir, exist_ok=True)
    data, w, h, ox, oy = build_map()
    with open(os.path.join(maps_dir, "warehouse.pgm"), "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode())
        f.write(data)
    with open(os.path.join(maps_dir, "warehouse.yaml"), "w") as f:
        f.write("image: warehouse.pgm\nmode: trinary\n"
                f"resolution: {L.MAP_RESOLUTION}\norigin: [{ox:.3f}, {oy:.3f}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


def main(argv=None) -> int:
    argv = argv or sys.argv[1:]
    if len(argv) != 2:
        print(__doc__)
        return 2
    write_assets(argv[0], argv[1])
    print(f"wrote {argv[0]} and map in {argv[1]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
