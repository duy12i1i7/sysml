# Two TurtleBot4 Sim-to-Real Swarm

This repo runs the same two-robot target-acquisition experiment in simulation
and on two physical TurtleBot4 Lite robots.

The shared interface is:

- `/robot1/odom`, `/robot1/assigned_role`, `/robot1/goal_pose`
- `/robot2/odom`, `/robot2/assigned_role`, `/robot2/goal_pose`
- `/target_pose`, `/swarm/task_state`, `/swarm/events`

Simulation creates those interfaces. Real mode expects the two TurtleBot4 robots
to be configured with namespaces `robot1` and `robot2` using the official
multiple-robots workflow.

## Build

```bash
cd /home/avis/Downloads/HA_paper/tb4-sim
./scripts/tb4 build
```

## Run Simulation

Default simulation is server-only and avoids the Gazebo GUI/render path that has
crashed on this machine.

```bash
./scripts/tb4 sim
```

With Gazebo GUI:

```bash
./scripts/tb4 sim --gui
```

Official TurtleBot4 Gazebo stack, server-only:

```bash
./scripts/tb4 sim-official
```

RViz target mode for the lightweight simulation:

```bash
./scripts/tb4 sim-rviz
```

Then open raw RViz in another terminal:

```bash
./scripts/tb4 rviz swarm
```

In raw RViz, publish a `2D Goal Pose` on `/goal_pose`, or publish a point on
`/clicked_point`. The bridge republishes that operator target to `/target_pose`.
The coordinator sends that target to robot1 and continuously sends robot2 a
following pose behind robot1.

If Gazebo crashes or retries:

```bash
./scripts/tb4 clean
```

## Paper Experiments

Use these commands for the final paper-aligned validation set.

Main quantitative result, 15 stable simulation trials:

```bash
./scripts/tb4 paper-sim-trials --repeats 3
```

The aggregate CSV is written under `metrics/paper_sim_trials/`.

Print the experiment protocol and traceability matrix:

```bash
./scripts/tb4 paper-plan
```

Physical deployment validation, with a saved report:

```bash
./scripts/tb4 paper-real-check
```

This checks the real `/robot1` and `/robot2` graph and writes a report under
`metrics/paper_real_deployment/`. Treat this as deployment evidence; the main
paper result remains the repeatable simulation trials plus traceability.

## Prepare Real Robots

Start from your reset/default TurtleBot4 Lite setup:

https://github.com/duy12i1i7/tb4-lite

That default state is a one-robot setup with root topics like `/odom` and
`/cmd_vel_unstamped`. For this two-robot repo, configure namespaces according to
the official TurtleBot4 multiple-robots tutorial:

https://turtlebot.github.io/turtlebot4-user-manual/tutorials/multiple_robots.html

Robot 1:

```text
turtlebot4-setup
ROS_DOMAIN_ID=0
RMW=FastDDS / rmw_fastrtps_cpp
Discovery=Simple Discovery
Discovery Server=Disabled
ROBOT_NAMESPACE=robot1
```

Robot 2:

```text
turtlebot4-setup
ROS_DOMAIN_ID=0
RMW=FastDDS / rmw_fastrtps_cpp
Discovery=Simple Discovery
Discovery Server=Disabled
ROBOT_NAMESPACE=robot2
```

More detail is in [REAL_ROBOTS.md](REAL_ROBOTS.md).

You can also print the same guide from the terminal:

```bash
./scripts/tb4 setup-real
```

## Check Real Robots

Check one reset/default robot before namespacing, if needed:

```bash
./scripts/tb4 check-default
```

After both robots are namespaced and online:

```bash
./scripts/tb4 check-real --motion --messages
```

Expected:

```text
OK: /robot1/odom
OK: /robot2/odom
OK: /robot1/scan
OK: /robot2/scan
OK: /robot1/odom is publishing Odometry
OK: /robot1/scan is publishing LaserScan
OK: /robot2/odom is publishing Odometry
OK: /robot2/scan is publishing LaserScan
Canonical two-TurtleBot4 graph check passed.
```

For a lighter topic-name check:

```bash
./scripts/tb4 check-real --motion
```

## Run Real Robots

Observation mode: coordinator, target publisher, and metrics only. No direct
velocity commands are sent, so the robots will not move.

```bash
./scripts/tb4 real use_target_publisher:=true
```

Drive mode: coordinator plus direct velocity following. This is the command to
make the two physical robots move.

```bash
./scripts/tb4 drive-real
```

Equivalent explicit launch:

```bash
./scripts/tb4 real use_target_publisher:=true use_cmd_vel_follower:=true
```

Emergency stop / zero command:

```bash
./scripts/tb4 stop-real
```

The real follower publishes `/robot*/cmd_vel_unstamped` as `Twist` and
`/robot*/cmd_vel` as `TwistStamped`, matching the TurtleBot4 Lite reset guide
and the Jazzy driving preference. Direct motion is intentionally off unless
`drive-real` or `use_cmd_vel_follower:=true` is used.

## RViz, SLAM, And Nav2 Workflow

For mapped real navigation, use the official TurtleBot4 SLAM/Nav2 stack and let
this repo provide the two-robot swarm layer above it.

Create a map with one robot, usually `robot1`. During this mapping stage,
`robot2` can stay powered off; it is only needed after the map is saved and you
run the two-robot mission.

Recommended mapping is manual: the repo starts TurtleBot4 SLAM and RViz, then
you drive `robot1` from the keyboard while the map updates live in RViz. This
keeps `robot2` powered off during mapping and avoids autonomous bumping in a
small room.

```bash
./scripts/tb4 check-map-real robot1
./scripts/tb4 manual-map-real robot1
```

Open a second terminal for keyboard driving:

```bash
cd /home/avis/Downloads/HA_paper/tb4-sim
./scripts/tb4 teleop-real robot1
```

When the RViz map looks good, stop keyboard driving with `Ctrl-C`, save the map,
then stop the SLAM/RViz terminal:

```bash
./scripts/tb4 save-map robot1 maps/lab
```

That creates `maps/lab.yaml` and `maps/lab.pgm`.

Autonomous mapping is still available as an experiment, but it is no longer the
default workflow:

```bash
./scripts/tb4 clear-map-real robot1
./scripts/tb4 explore-real robot1
```

Run both robots on the saved map with Nav2:

```bash
./scripts/tb4 nav2-real maps/lab.yaml
```

`nav2-real` runs the strict message check before launching. If either robot has
laser data but no `/robot*/odom` message, fix that physical robot's base
namespace/service or reboot it before trying Nav2 again.

Open RViz for each robot when you need to set AMCL initial pose:

```bash
./scripts/tb4 rviz robot1
./scripts/tb4 rviz robot2
```

After both robots are localized, set the shared swarm target from RViz. In a
namespaced TurtleBot4 RViz window, use **Publish Point**; this publishes
`/robot1/clicked_point` or `/robot2/clicked_point`, which the repo converts to
`/target_pose`. Robot1 receives the operator target, and robot2 receives a
dynamic following pose behind robot1. In raw RViz, a `2D Goal Pose` on
`/goal_pose` also works.

There is also a direct RViz drive mode:

```bash
./scripts/tb4 drive-real-rviz
```

Use that only in a clear lab area. It bypasses Nav2 and publishes velocity
commands directly, so it does not use the saved map for path planning or
obstacle avoidance.

## Inspect Runtime

```bash
./scripts/tb4 topics
ros2 topic echo /robot1/assigned_role --once
ros2 topic echo /robot2/assigned_role --once
ros2 topic echo /swarm/task_state --once
```

Metrics are written under:

```text
metrics/stable_sim/
metrics/official_tb4_sim/
metrics/real_two_tb4/
metrics/real_nav2_two_tb4/
```

## Paper Mapping

| Paper concept | Repo artifact |
| --- | --- |
| Two-robot requirement boundary | `robot1`, `robot2` parameters |
| Robot1 leader / robot2 follower behavior | `paper_tb4_swarm/coordinator.py` |
| Simulation validation | `stable_two_tb4.launch.py`, metrics CSV |
| Real TurtleBot4 deployment | `real_two_tb4.launch.py`, `REAL_ROBOTS.md` |
| Observable validation evidence | `/swarm/task_state`, `/swarm/events`, metrics CSV |
