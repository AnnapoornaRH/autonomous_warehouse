# Warehouse robot — ROS 2 Jazzy simulation (demonstration guide)

## What has been delivered

This is a **Gazebo Harmonic physics simulation**, not a video or a decorative mockup. The included warehouse world contains shelves, stock items, stations and obstacles. The 4-DOF arm + two-finger gripper is mounted on each differential-drive mobile base. The ROS 2 workspace contains separate robot-agent, navigation, manipulation, order, allocation, traffic and metrics packages, plus a browser dashboard.

**Honest verification statement:** The bundled source previously received integration testing documented in `docs/TEST_REPORT.md`, which identifies real defects with object handling and multi-robot collisions. In the environment used to prepare *this* package, ROS 2 and Gazebo were unavailable. Offline unit and static checks run here, but full runtime execution on your Ubuntu machine remains to be verified. Do not claim zero collisions or 100% grasp reliability.

## Your quickest demo (recommended)

On **Ubuntu 24.04** with **ROS 2 Jazzy** and **Gazebo Harmonic** installed:

```bash
unzip warehouse_jazzy_demo.zip
cd warehouse_project
./scripts/install_deps.sh --yes  # only if the required ROS/Gazebo dependencies are not already installed
./demo.sh
```

1. Gazebo opens the warehouse and one blue robot.
2. The robot should localize with Nav2, receive an order, drive toward a shelf, manipulate its arm, then head toward a packing station.
3. The live dashboard is served at `http://localhost:3000`. Use **Generate test orders** if you want a task immediately.
4. Open another terminal, source ROS and the workspace, then observe ROS topics or run the checks below.

For the **three-robot fleet** demonstration:

```bash
./demo.sh --fleet
```

For a VM / laptop without reliable 3D acceleration:

```bash
./demo.sh --headless        # one robot
./demo.sh --fleet --headless # three robots
```

All modes use the same Gazebo SDF world, ROS 2 nodes, and robot description. **Solo mode has lower computational cost and avoids inter-robot collisions**, but is not a proof of reliable grasping.

## Verify actual ROS 2 operation

Once `./demo.sh` is running, in a **second terminal**:

```bash
source /opt/ros/jazzy/setup.bash
cd warehouse_project
source install/setup.bash
ros2 node list
ros2 topic list | grep robot1
ros2 action list
ros2 topic echo /clock --once
python3 scripts/e2e_check.py --minutes 10 --min-completed 1 --report docs/live_demo_check.md
```

**Important:** the existing end-to-end test contains a fleet-specific assertion for three robots and is intended for `./demo.sh --fleet`, not solo mode. Don't use it as a pass/fail checklist for solo mode.

For offline checks:

```bash
bash scripts/run_tests.sh
```

## Demo story for a judge (90 seconds)

* **0–15 s:** Show the warehouse model in Gazebo and explain why mobile manipulators can automate repetitive material handling.
* **15–35 s:** Show the robot arm, mobile base, shelves, and target packing stations.
* **35–55 s:** Open the dashboard: order queue, robot status, physical positions and metrics.
* **55–75 s:** Generate an order, show navigation and an attempted pick-and-place task. If the grasp fails, describe the failure and recovery honestly.
* **75–90 s:** In fleet mode, show the traffic-zone reservations and highlight the remaining collision risk outside restricted aisles.

## Architecture

```text
                         React dashboard / REST API
                                     |
ROS 2 order_manager -> task_allocator -> robot_agent(s) -> Nav2 -> /cmd_vel
             |              |                |
             |       traffic_manager         +--> PickPlace action / arm_controller
             |              |                              |
             +--------------+------------------------------+
                                     |
                     ros_gz_bridge / Gazebo Harmonic
                       SDF warehouse + URDF manipulators
                                     |
                     metrics, status, positions, orders
```

## Scope and limitations

- The robot has **four actuated arm joints**, not six. The gripper has two opposed fingers. The source model is in `src/warehouse_description/urdf/mobile_manipulator.urdf.xacro`.
- Grasp uses **physical friction**, not a reliable magnetic or fixed attachment. The previous integration run recorded object slips and failed deliveries.
- Three-robot traffic control manages single-lane reservations, but earlier tests still observed collisions in shared open spaces.
- The included world is a warehouse, not a medical setting. It can be repurposed later, but it is **not already a sustainable-healthcare simulation**.
- A recorded Gazebo video **is not included**; record an actual run on your machine after installation.

## When the first run fails

1. Check your OS and binaries: `lsb_release -a; echo "$ROS_DISTRO"; gz sim --versions`.
2. Run `./scripts/install_deps.sh --yes`, then restart a terminal.
3. Try `./demo.sh --headless` if Gazebo GUI fails.
4. Inspect the newest `log/run_*/launch.log`, then execute `bash scripts/run_tests.sh`.
5. If Gazebo runs slowly, use solo mode, disable the camera via `./run_warehouse.sh --robots 1 --no-camera --headless --rate 0.25`.
