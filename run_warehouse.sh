#!/usr/bin/env bash
# =====================================================================================================
#  Autonomous warehouse - ONE COMMAND launcher
#
#      ./run_warehouse.sh                 full system, Gazebo window + dashboard on http://localhost:3000
#      ./run_warehouse.sh --headless      no Gazebo window (best for slow VMs); the dashboard shows the live map
#
#  What it does (in order, waiting for each step):
#    1. checks Ubuntu / ROS 2 Jazzy / Gazebo Harmonic / Nav2 / python / node dependencies
#    2. sources ROS 2, builds the workspace with colcon if the sources changed
#    3. builds the web dashboard if dashboard/dist is missing (needs node + npm)
#    4. launches Gazebo, 3 robots, Nav2 x3, arm controllers, robot agents, order generator, task allocator,
#       traffic manager, metrics and the dashboard backend (one ros2 launch, staggered start)
#    5. waits until every subsystem reports ready (system_monitor), prints the dashboard URL
#    6. Ctrl+C shuts everything down cleanly
# =====================================================================================================
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

GUI=true; CAMERA=true; ROBOTS="robot1,robot2,robot3"; SCHED="deadline_aware"; RATE="1.0"; PORT=3000
REBUILD=false; OPEN_BROWSER=true; DASHBOARD=true; SOFTWARE_GL=false; INSTALL=false; READY_TIMEOUT=300
METRICS_DIR="${WAREHOUSE_METRICS_DIR:-$HOME/warehouse_metrics}"

usage() {
cat <<EOF
Usage: ./run_warehouse.sh [options]
  --headless            run Gazebo without its GUI window
  --no-camera           disable the robot cameras (saves CPU)
  --robots N            start only the first N robots (1-3, default 3)
  --scheduler NAME      deadline_aware (default) | nearest_idle
  --rate X              order arrival rate in orders/minute (default 1.0)
  --port P              dashboard port (default 3000)
  --rebuild             force a clean colcon build
  --software-gl         force software OpenGL (LIBGL_ALWAYS_SOFTWARE=1) - use inside VMs without 3D acceleration
  --no-browser          do not open the dashboard in a browser
  --no-dashboard        do not start the dashboard backend
  --install-deps        run scripts/install_deps.sh first (needs sudo)
  -h, --help            this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) GUI=false ;;
    --no-camera) CAMERA=false ;;
    --robots) shift; n="${1:-3}"; ROBOTS=$(printf "robot%s," $(seq 1 "$n")); ROBOTS="${ROBOTS%,}" ;;
    --scheduler) shift; SCHED="${1:-deadline_aware}" ;;
    --rate) shift; RATE="${1:-2.0}" ;;
    --port) shift; PORT="${1:-3000}" ;;
    --rebuild) REBUILD=true ;;
    --software-gl) SOFTWARE_GL=true ;;
    --no-browser) OPEN_BROWSER=false ;;
    --no-dashboard) DASHBOARD=false ;;
    --install-deps) INSTALL=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1"; usage; exit 2 ;;
  esac
  shift
done

c_ok="\033[1;32m"; c_warn="\033[1;33m"; c_err="\033[1;31m"; c_dim="\033[2m"; c_off="\033[0m"
say()  { echo -e "${c_dim}[warehouse]${c_off} $*"; }
ok()   { echo -e "${c_ok}[ ok ]${c_off} $*"; }
warn() { echo -e "${c_warn}[warn]${c_off} $*"; }
die()  { echo -e "${c_err}[fail]${c_off} $*" >&2; exit 1; }

# ----------------------------------------------------------------------------------------------- 0 deps
if $INSTALL; then bash "$ROOT/scripts/install_deps.sh" || die "dependency installation failed"; fi

say "checking environment"
. /etc/os-release 2>/dev/null || true
say "OS: ${PRETTY_NAME:-unknown}"

ROS_SETUP=""
for d in jazzy; do [[ -f /opt/ros/$d/setup.bash ]] && ROS_SETUP="/opt/ros/$d/setup.bash"; done
if [[ -z "$ROS_SETUP" && -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
fi
if [[ -z "$ROS_SETUP" && -n "${AMENT_PREFIX_PATH:-}" ]] && command -v ros2 >/dev/null; then
  ROS_SETUP=""      # ROS already sourced (e.g. conda/robostack) - nothing to source
elif [[ -z "$ROS_SETUP" ]]; then
  die "ROS 2 Jazzy not found in /opt/ros.  Run:  ./scripts/install_deps.sh   (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic)"
fi
# shellcheck disable=SC1090
# ROS / colcon setup scripts reference unset variables, so they must be sourced with nounset disabled
set +u
[[ -n "$ROS_SETUP" ]] && source "$ROS_SETUP"
set -u
DISTRO="${ROS_DISTRO:-unknown}"
[[ "$DISTRO" == "jazzy" ]] || warn "ROS_DISTRO is '$DISTRO'; this project targets ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04)."
ok "ROS 2 ${DISTRO}"

missing=()
for cmd in ros2 colcon gz python3; do command -v "$cmd" >/dev/null || missing+=("command:$cmd"); done
for pkg in nav2_bringup nav2_amcl nav2_map_server nav2_controller nav2_planner nav2_behaviors nav2_bt_navigator \
           nav2_lifecycle_manager nav2_regulated_pure_pursuit_controller nav2_navfn_planner nav2_common nav2_msgs \
           ros_gz_sim ros_gz_bridge ros_gz_interfaces robot_state_publisher xacro tf2_ros; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || missing+=("ros-pkg:$pkg")
done
python3 - <<'PY' 2>/dev/null || missing+=("python:fastapi/uvicorn/yaml/websockets")
import fastapi, uvicorn, yaml  # noqa
try:
    import websockets  # noqa
except ImportError:
    import wsproto  # noqa
PY
if [[ ${#missing[@]} -gt 0 ]]; then
  echo -e "${c_err}Missing dependencies:${c_off} ${missing[*]}"
  echo "  -> run   ./scripts/install_deps.sh     (or:  ./run_warehouse.sh --install-deps)"
  exit 1
fi
GZV="$(gz sim --versions 2>/dev/null | head -1)"
ok "Gazebo ${GZV:-?}, Nav2, ros_gz and python dependencies present"
if [[ -n "${GZV:-}" && "${GZV%%.*}" != "8" ]]; then warn "Gazebo $GZV found; Gazebo Harmonic (8.x) is what this project was built and tested with."; fi

# ----------------------------------------------------------------------------------------------- 1 build
need_build=$REBUILD
if [[ ! -f install/setup.bash ]]; then need_build=true; fi
if ! $need_build; then
  if [[ -n "$(find src -type f \( -name '*.py' -o -name '*.xacro' -o -name '*.yaml' -o -name '*.sdf' -o -name '*.xml' -o -name '*.msg' -o -name '*.srv' -o -name '*.action' -o -name '*.pgm' -o -name 'CMakeLists.txt' \) -newer install/.warehouse_build_stamp 2>/dev/null | head -1)" ]]; then
    need_build=true
  fi
fi
if $need_build; then
  say "building the workspace (first build takes a few minutes: message generation)"
  $REBUILD && rm -rf build install log
  # a clean, copy-install build (no --symlink-install: works with every setuptools version)
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion- 2>&1 | tail -n 25
  [[ -f install/setup.bash ]] || die "colcon build failed - see log/latest_build/"
  if grep -q "Failed" log/latest_build/events.log 2>/dev/null; then die "colcon build reported failures - see log/latest_build/"; fi
  touch install/.warehouse_build_stamp
  ok "workspace built"
else
  ok "workspace is up to date (use --rebuild to force)"
fi
# shellcheck disable=SC1091
set +u
source install/setup.bash
set -u

# ----------------------------------------------------------------------------------------------- 2 dashboard
if $DASHBOARD; then
  dashboard_needs_build=false
  [[ -f dashboard/dist/index.html ]] || dashboard_needs_build=true
  if [[ -f dashboard/dist/index.html ]] && find dashboard/src -type f -newer dashboard/dist/index.html | grep -q .; then
    dashboard_needs_build=true
  fi
  if $dashboard_needs_build; then
    if command -v npm >/dev/null; then
      say "building updated dashboard (npm install && npm run build)"
      (cd dashboard && npm install --no-audit --no-fund && npm run build) || warn "dashboard rebuild failed; run cd dashboard && npm install && npm run build"
    else
      warn "Dashboard sources changed; install Node.js/npm and run cd dashboard && npm install && npm run build"
    fi
  else
    ok "dashboard build found and up to date (dashboard/dist)"
  fi
  export WAREHOUSE_DASHBOARD_DIST="$ROOT/dashboard/dist"
fi

# ----------------------------------------------------------------------------------------------- 3 environment
# NOTE: ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST is deliberately NOT set: with ~40 ROS participants on one machine
# FastDDS in localhost-only mode did not discover every node (Nav2 of robots 2/3 never became ready in testing).
if [[ -n "${ROS_LOCALHOST_ONLY:-}${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" ]]; then
  warn "unsetting ROS_LOCALHOST_ONLY / ROS_AUTOMATIC_DISCOVERY_RANGE (localhost-only discovery breaks this many nodes)"
  unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE
fi
export GZ_IP="${GZ_IP:-127.0.0.1}"                    # gz-transport inside VMs with several NICs
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
export WAREHOUSE_METRICS_DIR="$METRICS_DIR"
if $SOFTWARE_GL; then export LIBGL_ALWAYS_SOFTWARE=1; warn "software OpenGL enabled (slow but works without a GPU)"; fi
if $GUI && [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
  warn "no display detected -> running headless"; GUI=false
fi

# stray processes from a previous run would fight over topics and the dashboard port
if pgrep -x ruby >/dev/null 2>&1 || pgrep -f "ros_gz_bridge" >/dev/null 2>&1; then
  warn "found processes of a previous run - cleaning up first"
  bash "$ROOT/scripts/stop_warehouse.sh" quiet || true
fi
if python3 - "$PORT" <<'PY'
import socket, sys
s = socket.socket(); s.settimeout(0.5)
sys.exit(0 if s.connect_ex(("127.0.0.1", int(sys.argv[1]))) == 0 else 1)
PY
then die "port $PORT is already in use (another dashboard?). Use --port or stop the other program."; fi

# ----------------------------------------------------------------------------------------------- 4 launch
LOG_DIR="$ROOT/log/run_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOG_DIR"; ln -sfn "$LOG_DIR" "$ROOT/log/latest_run"
say "starting the system (log: $LOG_DIR/launch.log)"
LAUNCH_PID=""
cleanup() {
  trap - INT TERM EXIT
  echo
  say "shutting down ..."
  if [[ -n "$LAUNCH_PID" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 1 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 1; done
    kill -TERM -- "-$LAUNCH_PID" 2>/dev/null
  fi
  bash "$ROOT/scripts/stop_warehouse.sh" quiet || true
  [[ -d "$METRICS_DIR" ]] && say "metrics CSV files are in $METRICS_DIR"
  ok "stopped"
  exit 0
}
trap cleanup INT TERM EXIT

setsid ros2 launch warehouse_bringup warehouse.launch.py \
    gui:=$GUI use_camera:=$CAMERA robots:=$ROBOTS scheduler:=$SCHED orders_per_minute:=$RATE \
    dashboard:=$DASHBOARD dashboard_port:=$PORT metrics_dir:="$METRICS_DIR" \
    > "$LOG_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!

# ----------------------------------------------------------------------------------------------- 5 wait until ready
URL="http://localhost:$PORT"
if $DASHBOARD; then
  say "waiting for all subsystems (Gazebo, Nav2, arms, agents, managers) ..."
  t0=$(date +%s); last=""; IS_READY=false
  while true; do
    kill -0 "$LAUNCH_PID" 2>/dev/null || { tail -n 30 "$LOG_DIR/launch.log"; die "launch process died - see $LOG_DIR/launch.log"; }
    resp="$(curl -fs --max-time 2 "$URL/api/state" 2>/dev/null || true)"
    if [[ -n "$resp" ]]; then
      info="$(python3 -c '
import json,sys
d=json.loads(sys.stdin.read()); s=d.get("startup",{})
print(("READY" if s.get("ready") else "WAIT:" + ",".join(s.get("missing",[])[:4])))' <<<"$resp" 2>/dev/null || echo WAIT)"
      if [[ "$info" == "READY" ]]; then IS_READY=true; break; fi
      [[ "$info" != "$last" ]] && { say "$info"; last="$info"; }
    fi
    (( $(date +%s) - t0 > READY_TIMEOUT )) && { warn "not ready after ${READY_TIMEOUT}s - continuing anyway; see $LOG_DIR/launch.log"; break; }
    sleep 2
  done
  echo
  if $IS_READY; then
    ok "system is up - robots are now generating and fulfilling orders on their own"
  else
    warn "some subsystems are still starting (slow machine?). The dashboard shows what is missing; robots start working as soon as everything is ready."
  fi
  echo -e "      ${c_ok}Dashboard:${c_off}  $URL"
  echo -e "      ${c_dim}logs: $LOG_DIR   |   Ctrl+C to stop${c_off}"
  if $OPEN_BROWSER && command -v xdg-open >/dev/null && [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    (xdg-open "$URL" >/dev/null 2>&1 &) || true
  fi
else
  ok "launched without dashboard; logs: $LOG_DIR/launch.log   (Ctrl+C to stop)"
fi

wait "$LAUNCH_PID"
