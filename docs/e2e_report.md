# End-to-end check (2026-11-01, `scripts/e2e_check.py --minutes 7`, 3 robots, headless, sandbox at ~0.25x real time)

The script was **stopped by hand** during its last stage (waiting for the fleet to drain so that it can verify that all
reservations are released – it waits up to 15 minutes and the fleet was still working off its backlog). The checks below
are exactly what it printed; the drain/reservation-release check and the scripted CSV check did not run.
(CSV export was checked separately: `POST /api/export` returned success and `events.csv`, `metrics_timeseries.csv`,
`orders.csv`, `robots.csv` exist and are non-empty.)

| Result | Check | Detail |
|---|---|---|
| PASS | all subsystems report ready (system_monitor) | |
| PASS | three robots online | robot1, robot2, robot3 |
| PASS | simulation state RUNNING | rtf 0.18 |
| PASS | live positions come from the simulator | |
| PASS | control: generate test orders creates orders | 4 -> 8 |
| PASS | control: change order arrival rate | 3.00/min |
| PASS | control: pause stops the simulation clock | 155.59 -> 155.59 |
| PASS | control: resume restarts the clock | 155.59 -> 156.61 |
| PASS | control: emergency stop halts every robot base | max displacement while stopped 0.009 m |
| PASS | control: release e-stop returns to RUNNING | |
| PASS | orders were generated autonomously | orders_total=12 |
| **FAIL** | orders were completed through verified delivery | completed=0 at the end of the 7-minute window (required >= 3); 2 deliveries were completed shortly afterwards |
| PASS | no order was ever assigned to two robots | 0 duplicates in 208 samples |
| **FAIL** | every robot worked on orders (workload shared) | script counted [0, 0, 0] completed tasks per robot at that moment |
| PASS | no two robots were ever inside the same single-lane aisle | 0 observations |
| PASS | robots inside an aisle zone owned its reservation | 0 unreserved observations in 208 samples |
| PASS | state machine visited work states | NAVIGATING_TO_SHELF, POSITIONING_FOR_PICKUP, PICKING, NAVIGATING_TO_PACKING, WAITING_FOR_AISLE, WAITING_FOR_PACKING, DROPPING, RECOVERING |
| PASS | collisions measured (KPI, from contact sensors) | collisions=1 |
| PASS | late orders measured (KPI) | late=3 |
| **FAIL** | orders per minute measured (KPI) | 0.00/min (no delivery yet in the window) |
| PASS | deadlock-prevention events recorded | 4 preventions |
| not run | all reservations released when the fleet is idle | stopped before the fleet drained |
| not run | scripted CSV export check | checked manually, see above |

Final KPI snapshot of this run when it was stopped (simulation time 380 s, about 23 minutes of wall time):
12 orders created, **2 delivered**, 6 late, 0 permanently failed (5 pending re-tries), 9 collisions, 4 near misses,
5 navigation failures, 4 pickup failures, 8 delivery failures, 5 deadlock preventions, 20 recoveries.
