#!/usr/bin/env bash
# Stops every process of the warehouse system (safe to run at any time).
#   scripts/stop_warehouse.sh          verbose
#   scripts/stop_warehouse.sh quiet
QUIET="${1:-}"
say() { [[ "$QUIET" == "quiet" ]] || echo "[stop] $*"; }

say "stopping ros2 launch"
pkill -INT -f "bin/ros2 launch warehouse_bringup" 2>/dev/null
sleep 2
for pat in "lib/warehouse_" "lib/nav2_" "lib/ros_gz_bridge" "lib/ros_gz_sim" "lib/robot_state_publisher" \
           "warehouse_bringup/launch" "gz sim"; do
  pkill -f "$pat" 2>/dev/null
done
pkill -x ruby 2>/dev/null          # the gz sim server/gui run as ruby processes
sleep 1
for pat in "lib/warehouse_" "lib/nav2_" "lib/ros_gz_bridge" "gz sim"; do pkill -9 -f "$pat" 2>/dev/null; done
pkill -9 -x ruby 2>/dev/null
say "done"
exit 0
