#!/usr/bin/env python3
"""Regenerates the Gazebo world (SDF) and the Nav2 occupancy map from src/warehouse_common/warehouse_common/layout.py.

Edit the layout (aisle positions, shelves, items, stations, obstacles), run this script, rebuild:
    python3 scripts/generate_assets.py && ./run_warehouse.sh --rebuild
"""
import os
import sys

root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(root, "src", "warehouse_common"))
sys.path.insert(0, os.path.join(root, "src", "warehouse_simulation"))
from warehouse_simulation import world_builder  # noqa: E402

world_builder.write_assets(os.path.join(root, "src", "warehouse_simulation", "worlds", "warehouse.sdf"),
                           os.path.join(root, "src", "warehouse_navigation", "maps"))
print("world + map regenerated")
