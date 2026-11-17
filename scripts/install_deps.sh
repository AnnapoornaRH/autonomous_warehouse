#!/usr/bin/env bash
# =====================================================================================================
#  Installs everything the autonomous warehouse needs on Ubuntu 24.04 (Noble):
#     ROS 2 Jazzy (desktop), Gazebo Harmonic (through ros-jazzy-ros-gz), Nav2, colcon, rosdep,
#     python web dependencies (fastapi, uvicorn, websockets) and Node.js (only needed to rebuild the dashboard).
#
#     ./scripts/install_deps.sh            interactive (asks for sudo)
#     ./scripts/install_deps.sh --yes      non-interactive
#     ./scripts/install_deps.sh --no-node  skip Node.js
# =====================================================================================================
set -euo pipefail
ASSUME_YES=""; WITH_NODE=true
for a in "$@"; do
  case "$a" in
    --yes|-y) ASSUME_YES="-y" ;;
    --no-node) WITH_NODE=false ;;
    -h|--help) sed -n 2,12p "$0"; exit 0 ;;
  esac
done

. /etc/os-release
if [[ "${VERSION_ID:-}" != "24.04" ]]; then
  echo "WARNING: this project targets Ubuntu 24.04 (found ${PRETTY_NAME:-unknown})."
  echo "         ROS 2 Jazzy binaries are only published for 24.04. On other releases use a 24.04 VM or container."
  read -r -p "Continue anyway? [y/N] " ans; [[ "$ans" =~ ^[Yy]$ ]] || exit 1
fi

SUDO=""; [[ $EUID -ne 0 ]] && SUDO="sudo"

echo "==> base tools"
$SUDO apt-get update
$SUDO apt-get install $ASSUME_YES software-properties-common curl gnupg lsb-release ca-certificates git

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "==> adding the ROS 2 apt repository"
  $SUDO add-apt-repository -y universe
  $SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
  $SUDO apt-get update
  echo "==> installing ROS 2 Jazzy desktop (large download)"
  $SUDO apt-get install $ASSUME_YES ros-jazzy-desktop
else
  echo "==> ROS 2 Jazzy already installed"
fi

echo "==> installing Gazebo Harmonic bridge, Nav2 and tools"
$SUDO apt-get install $ASSUME_YES \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  ros-jazzy-tf2-tools ros-jazzy-tf2-ros-py \
  python3-colcon-common-extensions python3-rosdep python3-pip python3-yaml python3-numpy \
  python3-fastapi python3-uvicorn python3-websockets \
  curl xdg-utils mesa-utils

echo "==> rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then $SUDO rosdep init || true; fi
rosdep update || true

# fastapi/uvicorn from apt are fine on 24.04; fall back to pip if the import still fails
python3 - <<'PY' || { echo "==> installing python web packages with pip"; python3 -m pip install --user --break-system-packages fastapi "uvicorn[standard]" websockets; }
import fastapi, uvicorn  # noqa
PY

if $WITH_NODE; then
  if ! command -v npm >/dev/null; then
    echo "==> installing Node.js / npm (only needed to rebuild the dashboard; a prebuilt one is included)"
    $SUDO apt-get install $ASSUME_YES nodejs npm || echo "   (could not install node - the prebuilt dashboard/dist will be used)"
  fi
fi

grep -q "source /opt/ros/jazzy/setup.bash" "$HOME/.bashrc" || echo "source /opt/ros/jazzy/setup.bash" >> "$HOME/.bashrc"
echo
echo "Done.  Next:  cd $(cd "$(dirname "$0")/.." && pwd) && ./run_warehouse.sh"
