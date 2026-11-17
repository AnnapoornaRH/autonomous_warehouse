# Autonomous Warehouse – 3 mobile manipulators, ROS 2 Jazzy + Gazebo Harmonic + Nav2

Three independent differential-drive robots, each with a 4-DOF arm and a parallel gripper, fulfil randomly
generated warehouse orders inside a simulated warehouse (shelves, narrow single-lane aisles, intersections,
obstacles, packing stations) while a central **order manager**, **task allocator** and **traffic manager**
coordinate them. A live web **dashboard** shows everything that happens, from real ROS 2 data.

```
./run_warehouse.sh            # the one command  ->  Gazebo + 3 robots + Nav2 + managers + dashboard on http://localhost:3000
```

* Target platform: **Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8)** (your earlier delivery-robot project also used Jazzy).
* Hardware: 4+ CPU cores and 8 GB RAM recommended (see *Performance* below).  Use `--headless` in a VM without 3D acceleration.

---------------------------------------------------------------------------------------------------------------

## 1. Install and run (inside your Ubuntu VM)

```bash
# 1. copy the zip into the VM (shared folder, scp, drag & drop ...) and unpack it
unzip warehouse_project.zip
cd warehouse_project

# 2. only if ROS 2 Jazzy / Gazebo Harmonic / Nav2 are not installed yet (needs sudo, ~3 GB download)
./scripts/install_deps.sh --yes

# 3. run everything
./run_warehouse.sh                    # Gazebo window + dashboard
./run_warehouse.sh --headless         # no Gazebo window (recommended for slow VMs - the dashboard map shows the robots)
```

The first start builds the workspace (about 3–6 minutes, mostly message generation). The script then prints
`Dashboard: http://localhost:3000` and the robots start working by themselves. **Ctrl+C** shuts everything down
cleanly. `scripts/stop_warehouse.sh` kills leftovers if a terminal was closed abruptly.

Options: `--headless  --no-camera  --robots N  --scheduler deadline_aware|nearest_idle  --rate X  --port P
--rebuild  --software-gl  --no-browser  --no-dashboard  --install-deps`  (`./run_warehouse.sh --help`).

What `run_warehouse.sh` does: checks Ubuntu/ROS/Gazebo/Nav2/python dependencies and tells you what is missing →
sources ROS 2 → `colcon build` if sources changed → builds the dashboard if `dashboard/dist` is missing (a prebuilt
one is included) → one `ros2 launch` that starts Gazebo, spawns the robots, bridges, Nav2 ×3, arm controllers, robot
agents, order manager, allocator, traffic manager, metrics and the dashboard backend → waits until the
`system_monitor` reports that **every** subsystem is really up (Nav2 lifecycle active, action servers, services,
localisation, ground truth) → only then starts order generation → prints the URL.

---------------------------------------------------------------------------------------------------------------

## 2. What you see

**Gazebo**: a 9.1 m × 13 m warehouse – three double-sided shelf blocks, four 1.6 m wide single-lane aisles (each split
into a south and a north managed traffic zone), a south cross-aisle with three robot bays, three packing stations,
crates and barrels as obstacles, 12 pickable items (boxes and cylinders, 6 cm wide, 120 g). Robot bodies are blue / orange /
aqua – the same colours as in the dashboard.

**Dashboard (http://localhost:3000)** – dark control-centre UI:

| Area | Content (all live from ROS 2) |
|---|---|
| Header / controls | status, sim time, real-time factor · **Start, Pause, Resume, Reset, Emergency stop / release, Generate test orders, order arrival rate, CSV export** – every button calls a backend endpoint that calls a ROS 2 service |
| KPI row | total, completed, pending, processing, late orders · orders/min · active robots · collisions · system status |
| Live map | shelves, aisles, packing stations, obstacles, items (they move with the gripper), all robots with heading, Nav2 paths, goals, **aisle reservations (zone coloured by owner)**, waiting queues, waiting / faulted robots |
| Robot cards | state, task, destination, position, navigation / arm / gripper status, distance, queue, completed / failed, held reservations, battery estimate |
| Orders | id, object, shelf, station, robot, deadline countdown, status, completion time |
| Traffic monitoring | owner and queue of every zone / station, waiting robots, reservation / wait / release / deadlock-prevention / recovery events |
| Analytics | orders completed over time, robot utilisation, average completion time, late orders, workload per robot, collisions vs. near-misses (each chart has a table view) |

Metrics are also written to **`~/warehouse_metrics/run_<timestamp>/`**: `metrics_timeseries.csv`, `orders.csv`,
`events.csv`, `robots.csv` (flushed every 10 s, on *Export CSV*, on reset and on shutdown).

---------------------------------------------------------------------------------------------------------------

## 3. Architecture

```
                        +---------------- web dashboard (React) ----------------+
                        |   WebSocket /ws   REST /api/*   (served on :3000)     |
                        +----------------------------^--------------------------+
                                                     |
 warehouse_dashboard_bridge (FastAPI + rclpy) -------+---- services: sim_command, generate_orders, set_order_rate
        |  subscribes: robot status, orders, metrics, traffic, events, ground truth
        v
 +------------+  orders   +----------------+ assign_order +--------------------------+   +----------------------+
 | order_mgr  |---------->| task_allocator |------------->| robot_agent (per robot)  |-->| /traffic/request     |
 | (generator)|<----------|  deadline-aware|              |  state machine           |   | traffic_manager      |
 +------------+ updates   +----------------+              |  Nav2 client, arm client |   | (zones + stations)   |
       ^                          ^                       +-----+---------------+----+   +----------------------+
       | events                   | status                      |               |
 +------------+            +------------+          NavigateToPose|        pick_place|
 | metrics    |<-----------| all robots |                         v               v
 +------------+            +------------+                 Nav2 stack (x3)    arm_controller (x3) -- IK + joint commands
                                                          amcl/planner/       manipulating flag --+
                                                          controller/BT            |              v
                                                               | cmd_vel_nav   cmd_vel_gate (locks base while the arm works,
                                                               +-------------->   e-stop, stale-command and lidar safety stop)
                                                                                    | cmd_vel
 sim_manager (ground truth, collisions, near-misses, pause/resume/reset/estop, items)  v
        ^                                                                    ros_gz_bridge  <->  Gazebo Harmonic
        +------------------------------ gt poses, contacts, /clock ---------------^
```

### Packages (`src/`)

| Package | Content |
|---|---|
| `warehouse_robot_interfaces` | msgs (Order, RobotStatus, TrafficState, WarehouseEvent, MetricsSnapshot …), srvs (AssignOrder, RequestResources, ReleaseResources, SimCommand …), action PickPlace |
| `warehouse_common` | pure-python core: **layout** (single source of truth for geometry), arm **kinematics**, **reservation manager**, **scheduler**, order factory + 37 unit tests |
| `warehouse_description` | `mobile_manipulator.urdf.xacro` (base, wheels, casters, LiDAR, camera, arm, gripper, inertias, collisions, Gazebo plugins) |
| `warehouse_simulation` | world generator + generated `warehouse.sdf`, ros_gz bridge configs, `simulation.launch.py`, `sim_manager` |
| `warehouse_navigation` | multi-robot Nav2 parameters + launch, behaviour tree, `cmd_vel_gate`, occupancy map |
| `warehouse_manipulation` | `arm_controller` – pick / place action server |
| `warehouse_robot_agent` | per-robot state machine |
| `warehouse_order_manager` | autonomous order generation + order book |
| `warehouse_task_allocator` | workload / deadline aware allocation |
| `warehouse_traffic_manager` | aisle reservation, queues, deadlock prevention, recovery |
| `warehouse_metrics` | KPIs + CSV export |
| `warehouse_dashboard_bridge` | FastAPI/WebSocket bridge |
| `warehouse_bringup` | `warehouse.launch.py`, `system_monitor` (start-up sequencer) |
| `dashboard/` | React + Vite + Recharts source and prebuilt `dist/` |

### Robot model

Mobile base 0.50 × 0.44 m, two driven wheels (Gazebo **DiffDrive** plugin, real wheel physics) and two frictionless
casters · 2D **LiDAR** (177 samples, 176°, 6 m, 10 Hz) at the nose and a second one at the rear (together 352°), both at chassis height so that they see shelves, walls
*and the other robots* · front camera (320×240, 5 Hz, optional) · **4-DOF arm** (turret yaw, shoulder, elbow, wrist,
joint position controllers) · **2-finger gripper** (prismatic fingers, 6 cm item, friction grasp) · joint-state
publisher, robot_state_publisher, per-robot TF tree (`/robotN/tf`), chassis **contact sensor**.

> **Controller choice.** The base and arm are driven by Gazebo's own `DiffDrive` and `JointPositionController` plugins
> through `ros_gz_bridge` instead of `gz_ros2_control`. Namespaced multi-robot `ros2_control` +
> `controller_manager` setups are fragile, whereas the native plugins are deterministic and were testable here.
> Everything is still driven through ROS 2 topics (`/robotN/cmd_vel`, `/robotN/arm/jointK/cmd_pos`, …) and real
> physics – there is no animation-only movement.

### Navigation (Nav2, multi-robot)

One complete Nav2 stack per robot, namespaced (`/robot1/amcl`, `/robot1/controller_server`, …), separate TF trees,
costmaps fed by the robot's own scan, **AMCL** localisation initialised at the bay, Navfn A* planner, Regulated Pure
Pursuit controller, custom behaviour tree (clear costmaps → spin → wait, no blind back-up). A shared map server
publishes the occupancy map generated from the same layout as the Gazebo world. Robots see each other through the
LiDAR (the chassis is at LiDAR height). A velocity gate sits between Nav2 and the wheels.

### Pick and place

1. Nav2 drives to the pick pose (45 cm in front of the shelf edge) – coarse tolerance.
2. **Fine alignment** (closed loop on the simulator pose, ≤ 2 cm / 2°) – the arm reach is only ~10 cm tolerant.
3. `cmd_vel_gate` locks the base (`manipulating` flag): *the base cannot move while the arm works*.
4. Arm: raise → pre-grasp → straight-line approach (analytic IK, Cartesian interpolation) → close → **verify grasp
   from the finger gap** → lift → retract → stow (hand tilted up so the item stays seated) → **verify carry**
   (gap + item lifted off the shelf). 3 attempts, then the order is re-queued.
5. At the packing station: raise → pre-place → approach → release → retract → **verify delivery** (item rests on the table,
   gripper open). Only then is the order marked **COMPLETED**; the item is restocked on its shelf after a short delay.

Grasping is real contact physics (friction fingers on a 6 cm box / cylinder). Nothing is teleported or attached.

### Robot state machine

`IDLE → ASSIGNED → WAITING_FOR_AISLE → NAVIGATING_TO_SHELF → POSITIONING_FOR_PICKUP → PICKING → NAVIGATING_TO_PACKING →
WAITING_FOR_PACKING → DROPPING → COMPLETED → IDLE`, plus `RECOVERING` and `FAULTED`. Transitions are validated against a
table; an illegal transition is logged and puts the robot into FAULTED. A robot never starts a second order while one
is active; orders arrive through a queue (depth 2) that is only consumed by one worker. Handled failures: navigation
(agent-level recovery ladder: clear costmaps, wait, retry; then give the order back and go FAULTED for a cool-down),
pick failure (3 attempts, re-queue, item restocked), delivery failure, revoked reservation, e-stop, reset.

### Task allocation (`warehouse_common/scheduler.py`)

`DeadlineAwareScheduler`: orders are planned in order of *slack* (deadline − now − best service time). For every eligible
robot the estimated completion time is computed from its **projected end position and busy-until time** (current
task progress + queued orders), path length to shelf and station, pick / place time and a **congestion penalty**
from the traffic manager; cost = finish time + 3 × lateness + load + failure penalties. The best robot gets the order,
its projection is updated for the following orders of the same round, robots that are faulted / e-stopped / full are
skipped, otherwise the order stays queued. Orders are offered to one robot at a time and an order that a robot refused
or failed is not offered to the same robot again. The algorithm sits behind a small `Scheduler` interface
(`nearest_idle` is included as a baseline; add your own to `SCHEDULERS`).

### Deadlock prevention (`reservation.py` + `traffic_manager`)

Aisles are divided into **zones** (`A0-S … A3-N`), each with capacity 1; packing stations are resources too.

* requests are **atomic** (all zones needed for the visit, or none) → no hold-and-wait → no circular wait
* a robot that holds resources may not request different ones (`DENIED hold_and_wait`)
* waiting robots queue in strict priority order (deadline with aging) and **may not overtake a conflicting request**
  (no starvation)
* opposing robots can never be inside the same aisle: the second one waits at its bay (it never blocks an aisle mouth)
* zones are released **progressively** while the robot drives out; stations are released when the robot leaves
* timeouts: unrefreshed requests are dropped, over-long holds are revoked (`REVOKE` → robot recovers), stalled robots get a `RECOVER`
  command and are finally forced to release; robots that fault or stop reporting lose their reservations
* alternatives: a robot waiting too long for a station switches to a free one; a robot waiting for an aisle swaps to its next
  queued order if that order's zones are free
* a wait-for-graph cycle check runs on every tick as a belt-and-braces guard
* **every** reservation, wait, release, prevention and recovery is a `WarehouseEvent` (log, CSV, dashboard)

### ROS 2 interface cheat-sheet

| Name | Type | Purpose |
|---|---|---|
| `/warehouse/orders` | `OrderList` | order book |
| `/warehouse/order_update` | `OrderUpdate` | allocator / agents → order manager (single writer of order state) |
| `/<robot>/assign_order` | `AssignOrder` srv | allocator → agent (accept / refuse) |
| `/<robot>/status` | `RobotStatus` | state, pose, task, arm, gripper, reservations, path |
| `/<robot>/navigate_to_pose` | Nav2 action | navigation |
| `/<robot>/pick_place` | `PickPlace` action | manipulation |
| `/traffic/request`, `/traffic/release` | srv | reservations |
| `/traffic/state`, `/traffic/commands` | msgs | owners / queues, REVOKE / RECOVER |
| `/warehouse/events` | `WarehouseEvent` | everything notable |
| `/warehouse/metrics` | `MetricsSnapshot` | KPIs |
| `/warehouse/sim_command` | `SimCommand` srv | start / pause / resume / reset / estop / release_estop |
| `/warehouse/estop` | `Bool` (latched) | emergency stop (gates every base) |
| `/warehouse/ground_truth` | `EntityPoses` | simulator poses of robots and items |

---------------------------------------------------------------------------------------------------------------

## 4. Metrics (official KPIs and more)

1. **orders completed per minute** (sliding 5-minute window)  2. **late orders** (finished after the deadline or still open past it)
3. **collisions** – real contact episodes of the chassis contact sensor with walls, shelves, obstacles or another robot
   (debounced per contact episode).  *Near misses* (two robots closer than 0.7 m) and *waits* are separate counters.
Also: average order completion time, per-robot utilisation / idle time / distance / tasks / average task time, navigation,
pickup and delivery failures, deadlock-prevention events, recoveries. Everything comes from simulation events.

---------------------------------------------------------------------------------------------------------------

## 5. Testing

* `python3 -m pytest src/warehouse_common/test -v` – 37 unit tests (layout feasibility of every pick / drop pose, IK for
  every item and station, reservation manager incl. randomised stress test, scheduler, order factory).
* `python3 scripts/e2e_check.py --minutes 20 --report docs/e2e_report.md` – acceptance check against the running system
  (controls, no double assignment, aisle discipline, reservation release, KPIs, CSV).
* **`docs/TEST_REPORT.md` states exactly what was run, what passed, what failed and what could not be tested.**

---------------------------------------------------------------------------------------------------------------

## 6. Performance and tuning

Gazebo physics + LiDAR rendering + three Nav2 stacks + ~12 Python nodes is demanding. If the real-time factor shown in the
dashboard header is low:

* `./run_warehouse.sh --headless --no-camera` (no GUI, no camera rendering)
* give the VM more cores (4+) and enable 3D acceleration in the hypervisor; `--software-gl` forces Mesa llvmpipe
* `--robots 2` while experimenting
* most timing is **simulation time**, so a slow machine mainly makes everything slower – but pick-and-place is
  timing sensitive, and in the 0.15-0.4x real-time sandbox tests grasps failed far more often than in isolated tests.
  Aim for a real-time factor of at least ~0.7 (header of the dashboard) before judging the manipulation.

---------------------------------------------------------------------------------------------------------------

## 7. Troubleshooting

| Symptom | Fix |
|---|---|
| `Missing dependencies: …` | `./scripts/install_deps.sh` |
| Gazebo window black / crashes in a VM | `./run_warehouse.sh --headless` (dashboard still shows the live map) or `--software-gl` |
| `port 3000 is already in use` | `./scripts/stop_warehouse.sh` or `--port 3100` |
| Robots do not move for a long time after start | wait for the *System status* tile to change from STARTING to RUNNING; check `log/latest_run/launch.log` |
| Everything very slow | see *Performance* |
| Build error | `./run_warehouse.sh --rebuild`; logs in `log/latest_build/` |
| Dashboard says *disconnected* | the backend is part of the launch; check `log/latest_run/launch.log` for `dashboard_bridge` |
| Stale processes after closing the terminal | `./scripts/stop_warehouse.sh` |
| Gazebo topics not visible between processes | `export GZ_IP=127.0.0.1` (the script sets it) |
| Nav2 of some robots never becomes ready (`WAIT:robot2_nav2 ...` stays for minutes) | do **not** export `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` / `ROS_LOCALHOST_ONLY=1` in your shell (too many ROS nodes for localhost-only discovery); after a crash also run `./scripts/stop_warehouse.sh` and, if needed, `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` |

**Reset** (dashboard button or `POST /api/control {"command":"reset"}`): saves the CSV of the run, clears orders / metrics /
reservations, stows the arms, teleports robots to their bays and items to their shelves, re-initialises AMCL and restarts
the run. It is a *soft* reset (no world reload) so the clock keeps running and Nav2 is not disturbed.

---------------------------------------------------------------------------------------------------------------

## 8. Known limitations (honest list)

* **No MoveIt 2.** The arm is a 4-DOF planar arm with analytic IK and Cartesian interpolation – deterministic and
  testable. It has no collision-aware motion planning; waypoints were chosen so that every item and station is reachable.
* **No `ros2_control`** (native Gazebo controllers, see above).
* **Camera not used for perception.** It publishes `/<robot>/camera/image_raw`, but object poses for the arm and the final
  base alignment come from the simulator's ground truth (equivalent to a perfect pose estimate). Navigation uses AMCL.
* Two 176° LiDARs (nose and rear, mounted at chassis height so that robots see each other); there is no 3D sensing and the
  rear LiDAR is not used for localisation. Nav2's back-up recovery is deliberately not used (reversing is limited to a few
  centimetres during fine alignment and protected by the rear LiDAR).
* **AMCL on this very symmetric map can lose track** (identical aisles). Every robot agent therefore compares its AMCL pose with
  the simulator's reference pose and re-seeds AMCL when they disagree by > 0.35 m (event "localisation lost"). In a real
  warehouse floor markers / UWB anchors would play that role.
* One order = one item; two orders for the same item are never active at the same time.
* Reset is a soft reset; a robot that physically tips over is not recovered.
* **Pick-and-place is the weakest part.** Grasping is pure friction physics (no attachment tricks). A held item slowly
  slips along the fingers (~0.2-0.8 mm/s) and is lost after roughly 100-200 simulated seconds, and in the 3-robot tests
  the first grasp attempt often missed. Failed picks / deliveries are retried (item restocked, order re-queued, up to
  3 attempts), so orders are delayed or fail rather than being faked as delivered. A deck tray (put the item down, ride,
  pick it up again at the station) would fix the carry problem; it is not implemented.
* **Robots still collide** now and then (cross-aisle in front of the packing stations). The reservation system keeps
  two robots out of the same aisle, but the open cross-aisle and the station approaches are only protected by Nav2's
  costmaps and the LiDAR safety stop. Collisions are counted and shown in the dashboard / CSV.
* **Throughput was low in testing**: the test machine ran the simulation at 0.15-0.4x real time, and manipulation is
  timing sensitive. How it behaves on your VM is unknown until you run it; `docs/TEST_REPORT.md` lists exactly what was
  and was not tested (it was tested headless in a software-rendered 2-core sandbox).


## Dashboard product ordering upgrade (November 2026)

Launch `./demo.sh --fleet --rate 0` then open http://localhost:3000.
The existing dashboard now includes a **Product ordering & dispatch** section above the map.
Each of the twelve catalog entries represents one unique Gazebo item (`item_01` ... `item_12`),
with a named shelf (`S1` ... `S6`). Choose one or several physical items, choose `PK1`,
`PK2` or `PK3`, and press **Place order**. The new `/api/orders/submit` API atomically
submits them to the ROS 2 order manager; they use the **same** robot pick-and-place pipeline
as automatically generated orders. Ordered items are temporarily unavailable while
assigned/in progress, and rejected during the manager's 15-simulation-second restock cooldown.
If an item is unavailable, the entire batch is rejected rather than partially submitted.

The **Dispatch mode** dropdown lets you use all active robots or only one robot for **new
assignments**. Switching to one robot does not cancel tasks already assigned to the others.
Use `./demo.sh --fleet` to have three available robots; `./demo.sh --solo` spawns only robot1.
The existing deadline-aware task allocator considers estimated travel time, deadline slack,
congestion, and robot queue lengths. This is a heuristic assignment optimizer, **not**
a claim of globally optimal path planning; Nav2 still plans obstacle-aware paths per robot.

Endpoints:

* `GET /api/catalog` — physical items, shelf IDs, and packing stations.
* `POST /api/orders/submit` — `{ "item_ids": ["item_01", "item_07"], "station_id": "PK1" }`
* `POST /api/fleet/selection` — `{ "robot_id": "all" }` or `{ "robot_id": "robot2" }`

For the cleanest human-controlled demo, set order arrival rate to **0** and reset the
simulation if you don't want the initial demo orders. The launch configuration may create
initial orders on startup. The dashboard rebuilds automatically when its source is newer
than the compiled dashboard. If Node/npm is not installed, run `cd dashboard && npm install
&& npm run build` once after installing Node.js.

### Verification status

The source changes have passed Python syntax compilation and the original
`warehouse_common` test suite in the development environment. **A real ROS 2 Jazzy
colcon build, Gazebo run, browser production build, and end-to-end pick-up were not executed
here**; test these on your Ubuntu VM before claiming that the new order controls are
fully validated. The current project remains a **4-DOF arm** simulation.
