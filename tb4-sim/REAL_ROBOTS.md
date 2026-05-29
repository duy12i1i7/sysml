# Real TurtleBot4 Setup

This repo assumes two physical TurtleBot4 Lite robots have been reset and
verified with the single-robot workflow from:

https://github.com/duy12i1i7/tb4-lite

After that reset, each robot starts as a single robot on root topics such as
`/odom` and `/cmd_vel_unstamped`. That is correct for one robot, but a two-robot
mission needs unique namespaces. Configure the robots with the official
TurtleBot4 multiple-robots workflow:

https://turtlebot.github.io/turtlebot4-user-manual/tutorials/multiple_robots.html

## Configure Robot Namespaces

On robot 1:

```bash
ssh ubuntu@<robot1-ip>
turtlebot4-setup
```

Set:

```text
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DOMAIN_ID=0
Discovery=Simple Discovery
Discovery Server=Disabled
ROBOT_NAMESPACE=robot1
```

Apply settings and reboot if the setup tool asks.

On robot 2:

```bash
ssh ubuntu@<robot2-ip>
turtlebot4-setup
```

Set:

```text
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DOMAIN_ID=0
Discovery=Simple Discovery
Discovery Server=Disabled
ROBOT_NAMESPACE=robot2
```

Apply settings and reboot if the setup tool asks.

## Verify From Laptop

```bash
cd /home/avis/Downloads/HA_paper/tb4-sim
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

The repo can command either `/robot*/cmd_vel_unstamped` or `/robot*/cmd_vel`,
but direct motion is off by default.

## Run Real Mode

Observation mode, no velocity commands. The robots will not move:

```bash
./scripts/tb4 real use_target_publisher:=true
```

Drive mode with direct velocity following:

```bash
./scripts/tb4 drive-real
```

Equivalent explicit launch:

```bash
./scripts/tb4 real use_target_publisher:=true use_cmd_vel_follower:=true
```

Stop command:

```bash
./scripts/tb4 stop-real
```

Keep both robots on the floor with space around them before enabling direct
motion.

## Run With RViz, SLAM, And Nav2

Map the lab once with one robot. For mapping, only `robot1` needs to be online;
`robot2` can stay powered off until the saved map is ready.

```bash
./scripts/tb4 check-map-real robot1
./scripts/tb4 manual-map-real robot1
```

Open a second terminal and drive `robot1` with the keyboard:

```bash
cd /home/avis/Downloads/HA_paper/tb4-sim
./scripts/tb4 teleop-real robot1
```

`manual-map-real` starts TurtleBot4 SLAM and RViz only; it does not publish
velocity commands. `teleop-real` publishes low-speed commands to
`/robot1/cmd_vel_unstamped`, so the robot moves only while you are actively
driving from the keyboard.

When RViz coverage is good, press `Ctrl-C` in the teleop terminal, save the map,
then press `Ctrl-C` in the SLAM/RViz terminal:

```bash
./scripts/tb4 save-map robot1 maps/lab
```

Autonomous mapping remains available for experiments, but manual mapping is the
recommended real-room workflow:

```bash
./scripts/tb4 clear-map-real robot1
./scripts/tb4 explore-real robot1
```

Run the two-robot mapped mission:

```bash
./scripts/tb4 nav2-real maps/lab.yaml
```

`nav2-real` refuses to start unless both robots publish odometry and lidar
messages. If one robot exposes `/robot*/scan` but not `/robot*/odom`, its lidar
computer is visible but the Create3/base odometry side is not ready.

Use RViz to set AMCL initial pose for each robot:

```bash
./scripts/tb4 rviz robot1
./scripts/tb4 rviz robot2
```

Then set the shared swarm target. In the namespaced RViz window, use **Publish
Point**. The repo listens to `/robot1/clicked_point` and `/robot2/clicked_point`
and republishes the selected point as `/target_pose`. The coordinator sends the
selected target to robot1, computes a following pose behind robot1 for robot2,
and sends both per-robot goals to Nav2 through `/robot1/navigate_to_pose` and
`/robot2/navigate_to_pose`.

Do not use direct-drive mode for mapped navigation. Direct-drive mode is only a
controlled lab test without Nav2 obstacle avoidance.
