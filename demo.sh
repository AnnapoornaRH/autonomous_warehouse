#!/usr/bin/env bash
# Presentation-friendly launcher for the ROS 2 Jazzy + Gazebo Harmonic warehouse.
# Default: one robot for a smoother, easy-to-explain pick-and-place demonstration.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
MODE=solo
HEADLESS=false
RATE=0.25
PORT=3000
while (($#)); do
  case "$1" in
    --solo) MODE=solo ;;
    --fleet) MODE=fleet; RATE=0.5 ;;
    --headless) HEADLESS=true ;;
    --rate) shift; [[ $# -gt 0 ]] || { echo "Missing value for --rate" >&2; exit 2; }; RATE="$1" ;;
    --port) shift; [[ $# -gt 0 ]] || { echo "Missing value for --port" >&2; exit 2; }; PORT="$1" ;;
    -h|--help)
      cat <<'USAGE'
ROS 2 Jazzy Warehouse Demo
  ./demo.sh                       One mobile manipulator, Gazebo window + live dashboard
  ./demo.sh --fleet               Three robots + zone coordination and task scheduling
  ./demo.sh --headless            Headless Gazebo, dashboard remains available
  ./demo.sh --fleet --headless    Full fleet on a lower-power machine
  ./demo.sh --rate 0.5           Override orders per minute
  ./demo.sh --port 3001         Change dashboard port

Requirements: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Nav2.
First-time dependencies: ./scripts/install_deps.sh --yes
Stop: Ctrl+C. Dashboard: http://localhost:PORT
USAGE
      exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
if ! [[ "$RATE" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "Rate must be non-negative number" >&2; exit 2
fi
if ! [[ "$PORT" =~ ^[0-9]+$ ]] || (( PORT < 1024 || PORT > 65535 )); then
  echo "Port must be an integer from 1024 to 65535" >&2; exit 2
fi
if [[ "$MODE" == solo ]]; then N=1; else N=3; fi
printf '\n=== WAREHOUSE ROS 2 JAZZY DEMO ===\nMode: %s | Robots: %d | Arm: 4 DOF + 2-finger gripper | Orders/min: %s\n' "$MODE" "$N" "$RATE"
printf 'Gazebo Harmonic physics, Nav2 navigation, ROS 2 action interfaces, live dashboard\n'
printf 'Dashboard: http://localhost:%s\n\n' "$PORT"
args=(--robots "$N" --rate "$RATE" --port "$PORT")
$HEADLESS && args+=(--headless)
exec "$ROOT/run_warehouse.sh" "${args[@]}"
