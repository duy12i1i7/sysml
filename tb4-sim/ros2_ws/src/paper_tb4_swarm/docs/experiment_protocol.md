# Paper Experiment Protocol

This protocol matches the current paper scope: MBSE traceability, executable
two-TurtleBot4 simulation, and a conservative physical deployment boundary.

## Experiment 1: Stable Simulation Trials

Purpose: provide the main quantitative result for the paper.

Run:

```bash
cd /home/avis/Downloads/HA_paper/tb4-sim
./scripts/tb4 build
./scripts/tb4 paper-sim-trials --repeats 3
```

This runs 15 trials by default: five target locations repeated three times.
The aggregate CSV is written to `metrics/paper_sim_trials/aggregate_*.csv`.

Use these paper metrics:

| Metric | Source | Paper use |
| --- | --- | --- |
| `success` | `metrics_logger` summary | target-acquisition completion |
| `duration_s` | `metrics_logger` summary | task completion time |
| `robot1_path_m` | `metrics_logger` summary | leader travel distance |
| `robot2_path_m` | `metrics_logger` summary | follower travel distance |
| `min_separation_m` | `metrics_logger` summary | two-robot spacing/safety observation |
| `role_switches` | `metrics_logger` summary | runtime role stability |
| `requirement_refs` | `metrics_logger` summary | MBSE requirement traceability |

Acceptance criterion for the paper: most or all simulation trials should
complete and produce the required metrics. Failures should be reported as
implementation limits, not hidden.

## Experiment 2: MBSE Traceability

Purpose: show that the paper model is connected to executable ROS 2 artifacts.

Use this matrix in the paper:

| Requirement | Runtime artifact | Evidence |
| --- | --- | --- |
| REQ-001 two robots cooperate on one target task | `coordinator.py`, `/target_pose`, `/robot*/goal_pose` | task-state JSON and timeline CSV |
| REQ-002 robot1 leader | `leader_namespace: robot1`, `/robot1/assigned_role` | role messages and summary CSV |
| REQ-003 robot2 follower | follower goal generation in `coordinator.py` | `/robot2/goal_pose`, path metric |
| REQ-004 observable roles/state/events | `/swarm/task_state`, `/swarm/events` | timeline CSV |
| REQ-005 simulation before hardware | `stable_two_tb4.launch.py`, `metrics_logger.py` | aggregate simulation CSV |
| REQ-006 explicit physical motion boundary | `real_two_tb4.launch.py`, `scripts/tb4 real`, `scripts/tb4 drive-real` | real deployment report |

Do not claim a general swarm benchmark from this matrix. It is evidence that
the MBSE requirements remain traceable into the ROS 2 implementation.

## Experiment 3: Physical Deployment Validation

Purpose: support the sim-to-real boundary without overclaiming physical swarm
performance.

Run when both physical robots are powered on and namespaced:

```bash
cd /home/avis/Downloads/HA_paper/tb4-sim
./scripts/tb4 paper-real-check
```

The command saves:

```text
metrics/paper_real_deployment/check_<timestamp>.txt
```

Acceptance criterion:

- `/robot1/odom` publishes `Odometry`.
- `/robot2/odom` publishes `Odometry`.
- `/robot1/scan` publishes `LaserScan`.
- `/robot2/scan` publishes `LaserScan`.
- command velocity topics are visible.

Optional physical Nav2 demonstration:

```bash
./scripts/tb4 nav2-real maps/lab.yaml
./scripts/tb4 rviz swarm
```

Only choose goals inside the green safe-goal overlay. This should be written as
a physical feasibility demonstration, not as the main quantitative result.

## Paper Claim Boundary

Use strong claims for:

- model-to-code traceability;
- repeatable two-robot simulation validation;
- physical ROS 2 deployment readiness.

Avoid strong claims for:

- arbitrary real-world goal selection;
- large swarm scalability;
- full physical two-robot navigation performance.
