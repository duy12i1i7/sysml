# Paper and Implementation Alignment

The executable scope is exactly two canonical TurtleBot4 robot identities:

- `robot1`
- `robot2`

The paper and implementation use the same two-robot target-acquisition boundary.
No larger team size is claimed as part of the implemented validation case.

For physical TurtleBot4 Lite robots, these names are not just cosmetic. The
single-robot lab setup in <https://github.com/duy12i1i7/tb4-lite> usually runs
FastDDS with `ROS_DOMAIN_ID=0`, Simple Discovery, an empty user namespace, and
root topics such as `/odom` and `/cmd_vel_unstamped`. For this repo, the
official multiple-robots setup is used next: set `ROBOT_NAMESPACE=robot1` on
the first robot and `ROBOT_NAMESPACE=robot2` on the second robot while keeping
the same DDS domain.

## Implemented Requirements

| Requirement | Paper meaning | Code location |
| --- | --- | --- |
| REQ-001 | Two robots cooperate on one target-acquisition task | `mbse_model.py`, `coordinator.py` |
| REQ-002 | Robot1 is the leader and receives the operator target | `coordinator.py` |
| REQ-003 | Robot2 is the follower and tracks a dynamic pose behind robot1 | `coordinator.py` |
| REQ-004 | Runtime publishes observable roles, goals, state, and events | `coordinator.py` |
| REQ-005 | Simulation runs before hardware and writes metrics | `stable_two_tb4.launch.py`, `metrics_logger.py` |
| REQ-006 | Real deployment records DDS setup and requires explicit motion enablement | `real_two_tb4.launch.py`, `ROS_DOMAIN_ID`, `/cmd_vel_unstamped` |

## Paper-Aligned Experiments

The validation package is intentionally limited to three evidence types:

| Experiment | Command | Paper role |
| --- | --- | --- |
| Stable simulation trials | `./scripts/tb4 paper-sim-trials --repeats 3` | main quantitative result |
| MBSE traceability matrix | `./scripts/tb4 paper-plan` | requirement-to-runtime evidence |
| Physical deployment check | `./scripts/tb4 paper-real-check` | sim-to-real feasibility boundary |

The detailed protocol is in `experiment_protocol.md`.

## Reality Boundary

Stable simulation uses lightweight differential-drive physics with TurtleBot4
visual meshes. This is the practical mode on the current machine because the
official full TurtleBot4 Gazebo stack previously crashed in the renderer.

Official TurtleBot4 simulation is still available through the official spawn
launch files, but it is server-only by default.

Real-robot launch starts only the paper-level coordination, target, and metrics
nodes. It assumes the physical robots have already been namespaced as
`/robot1/...` and `/robot2/...`. It does not send direct velocity commands in
observation mode. Drive mode is explicit through `./scripts/tb4 drive-real` or
`use_cmd_vel_follower:=true`. The real follower publishes
`/robot*/cmd_vel_unstamped` as `Twist` and `/robot*/cmd_vel` as `TwistStamped`.

For mapped operation, the repo delegates low-level navigation to TurtleBot4
Nav2. RViz clicks are converted into one shared `/target_pose`, the coordinator
assigns the target to robot1 and a following pose to robot2, and
`nav2_goal_dispatcher` sends those goals to `/robot1/navigate_to_pose` and
`/robot2/navigate_to_pose`. In this mode the coordinator uses
`/robot*/amcl_pose` so the following behavior is based on map-frame poses rather
than raw odometry.
