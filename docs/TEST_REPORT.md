# Test report

Date of testing: 2026-11-01. This report says what was actually run, what it showed, and what was **not** run.
Nothing below is claimed beyond the evidence listed with it.

## 1. Test environment (important: it is not your VM)

| Item | Value |
|---|---|
| OS | Ubuntu 24.04.5 container (cloud sandbox), **2 CPU cores, no GPU**, headless, software rendering |
| ROS 2 / Gazebo | ROS 2 Jazzy and Gazebo Harmonic (gz-sim 8.10), Nav2 1.3.13 – installed from conda (RoboStack), **not** from the apt packages that `scripts/install_deps.sh` installs |
| Real-time factor | 0.15 – 0.4 with 3 robots (the simulation ran 3–6x slower than real time) |
| Not available there | Gazebo GUI window, a browser pointed at the dashboard, apt access to `packages.ros.org` |

Because the machine was so slow, timing-sensitive behaviour (arm trajectories, Nav2 under load, controllers) is
pessimistic in these tests. A faster VM may behave better, but that has **not been verified**.

## 2. Summary

| Area | Result |
|---|---|
| Unit tests (37, pure Python: geometry, IK, reservations, scheduler, orders) | **PASS** 37/37 |
| `colcon build` of all packages, dashboard `npm run build` | **PASS** |
| World generation, 3 robots spawn with sensors / odometry / TF / joint states in namespaces | **PASS** |
| Nav2 multi-robot navigation (per-robot AMCL, planner, controller) | **PASS with caveats** – works; AMCL occasionally loses track and is re-seeded by the agent |
| Dashboard backend + controls (generate orders, rate, pause, resume, e-stop, release) | **PASS** (live check, section 5; browser rendering was only checked with screenshots of an earlier run) |
| Order generator, deadline-aware allocator, order re-queue | **PASS** (seen in live runs) |
| Traffic manager: aisle reservations, waiting, deadlock prevention, timeouts, station switching | **PASS** (seen in live runs; invariants unit-tested incl. randomised stress) |
| Pick and place with physics (friction grasp) in isolation | **PARTIAL** – about 60–90 % depending on the batch (section 4) |
| Pick and place inside the full 3-robot system | **NOT RELIABLE** – first grasp attempts often miss; carried items slowly slip out of the fingers |
| Collision-free operation | **FAIL** – robots still collide (section 6) |
| Throughput / on-time delivery | **FAIL** at the tested speed – only a handful of deliveries per run |
| `./run_warehouse.sh` one-command start/stop | **PASS** in the sandbox with the conda ROS install (section 7) – **not tested** on apt ROS |
| Gazebo GUI, apt-based `install_deps.sh`, your VM's performance, camera images | **NOT TESTED** |

## 3. Unit and offline tests

`scripts/run_tests.sh` -> **all 4 offline stages passed**: (1) 37 unit tests, (2) package manifests valid, (3) robot xacro -> URDF -> SDF conversion with the contact-sensor and controller topics, (4) generated world/map in sync with the layout and the world SDF is valid. Unit tests: They cover: layout feasibility (every item and station reachable, aisles wide
enough), analytic IK for every item/station and stow compactness, the reservation manager (atomic grant, priority
with aging, no hold-and-wait, timeouts, wait-for-graph cycle guard, randomised stress with invariants), the scheduler
and the order factory.

## 4. Pick and place (physics, no attachment tricks)

The arm is a 4-DOF arm with a two-finger gripper; the item is held by friction only. Verification uses the finger gap
and the item's ground-truth pose.

Isolated test rig (one robot, robot teleported to the pick pose with ±1.5 cm / ±1.5° error, the real `arm_controller`
action server):

* earlier configuration: 10 of 11 picks succeeded
* final configuration, batch of 8: **6 of 8 picks**, of the 6 places only 3 delivered (the rig teleports the robot
  *and* the carried item, which itself knocks items out of the fingers, so the place numbers understate the real
  behaviour but are not a clean measurement either way)

**Known defect – carry creep.** A held item slides slowly (~0.2–0.8 mm/s) along the fingers and is lost after roughly
100–200 simulated seconds. Driving and rotating for 20 s does *not* drop it. Things tried without a real fix: removing
the integrator, fingertip lips, lower finger force (slowed the creep ~1.5x but made grasps worse, so it was reverted).
Consequence: if a robot has to wait a long time with an item (e.g. for a packing station), the item is lost and the
order is retried. A tray on the robot deck (set the item down, ride, pick again) would avoid this; it is **not
implemented**.

## 5. Live system checks (3 robots, headless, `scripts/e2e_check.py`)

Full table: `docs/e2e_report.md`. Result: **18 checks passed, 3 failed, 2 not run.**

* PASS – all subsystems ready, three robots online, simulation RUNNING, live poses from the simulator
* PASS – controls: generate orders (4 -> 8), change arrival rate, pause freezes the clock, resume restarts it,
  emergency stop halts the bases (max drift 0.9 cm), release returns to RUNNING
* PASS – orders generated autonomously, never assigned to two robots, no two robots ever in the same aisle, every robot
  inside an aisle owned its reservation, deadlock preventions recorded, collisions/late orders measured
* **FAIL** – "orders completed through verified delivery" (0 in the 7-minute window; 2 shortly after), "workload shared",
  "orders per minute > 0" – all consequences of the pick-and-place problems in section 4
* not run – "reservations released when the fleet is idle" (the fleet never drained), scripted CSV check (CSV export
  was checked by hand and works)

KPI snapshot of that run when it was stopped (sim time 380 s, ~23 wall minutes): 12 orders, **2 delivered**, 6 late,
9 collisions, 4 near misses, 4 pickup failures, 8 delivery failures, 5 navigation failures, 20 recoveries.
Earlier full runs (before the last fixes) delivered 2 orders in ~340 simulated seconds with up to 23 collisions.

## 6. What did NOT work (honest list)

* **Collisions.** In live runs the robots collided with each other 1–4 times in the first 150–240 simulated seconds and 9 times in 380 s in the last run (one episode of repeated contacts while robots were recovering)
  (mostly in the cross-aisle in front of the packing stations). The reservation system prevents two robots from
  *entering the same aisle*, but robots still touch in the open cross-aisle and near stations. The metric counts them
  (debounced per robot pair).
* **Pickup failures** in full runs: typically the first grasp attempt closes on nothing ("gripper width 0.003 does not
  match item") and the item is then pushed off its slot, so the retry is refused (`ITEM_DISPLACED`), the item is
  restocked and the order is re-queued for another robot.
* **Delivery failures** from carry creep (above). Failed deliveries are retried (up to 3 attempts per order), then the
  order is counted as FAILED.
* **Localisation** on this symmetric map loses track now and then; the agent notices (compares AMCL with the simulator
  reference pose) and re-seeds AMCL. Each such event is logged on the dashboard.
* **Throughput**: 2 deliveries out of 12 orders in the last 23-wall-minute run at real-time factor ~0.25. No claim is made about how the
  system performs at real-time factor 1.

## 7. Launcher test (`./run_warehouse.sh`)

* `bash -n` on all scripts – OK; `--help` – OK; run without ROS installed -> clear error message and exit – OK.
* Full launch in the sandbox (`--headless --no-camera --no-browser`, conda ROS): dependency check, up-to-date build
  detection, dashboard check, launch, readiness wait, dashboard reachable, `Ctrl+C` -> clean shutdown with no processes
  left – OK.
* Bugs found by this test and fixed: (1) sourcing `install/setup.bash` under `set -u` crashed the launcher;
  (2) `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` made Nav2 of robots 2 and 3 never become ready (too many DDS participants
  for FastDDS' localhost-only discovery) – removed.
* Not tested: `./scripts/install_deps.sh` (needs the apt ROS repository), the Gazebo GUI path (`gui:=true`), opening a
  browser automatically.

## 8. Not tested at all

Gazebo GUI rendering, GPU/VM graphics, apt installation, camera-based perception (the camera publishes images but
nothing consumes them), behaviour at real-time factor 1, runs longer than ~15 wall minutes, hardware.

## 9. How to reproduce

```bash
./scripts/run_tests.sh                       # unit tests
./run_warehouse.sh --headless --no-camera    # start (dashboard http://localhost:3000)
python3 scripts/e2e_check.py --minutes 10 --report docs/e2e_report.md   # while it runs
```
Metrics CSV files: `~/warehouse_metrics/run_<timestamp>/`.
