# Image Placeholders To Replace

The paper now contains four image placeholders in `access.tex`.
Replace each placeholder box with the image file listed below.

## `figs/experiment_setup.png`

Use: Gazebo/RViz screenshot or lab photo.

Must show:
- `/robot1`
- `/robot2`
- shared target location
- two-robot validation setup

Caption already in paper:

> Experimental setup for the two-TurtleBot4 target-acquisition validation case.

## `figs/rviz_safe_goal_map.png`

Use: RViz screenshot of `maps/lab.yaml` with the safe-goal overlay.

Must show:
- saved map
- safe/free target cells
- unsafe or obstacle-adjacent cells
- why goals cannot be selected everywhere

Caption already in paper:

> RViz map view used to distinguish safe reachable targets from unsafe goal locations in the optional physical Nav2 demonstration.

## `figs/sim_trial_metrics.png`

Use: plot generated from:

```text
tb4-sim/metrics/paper_sim_trials/aggregate_20260528_173405.csv
```

Recommended panels:
- completion time per trial
- robot1 path length
- robot2 path length
- minimum separation

Caption already in paper:

> Simulation metrics over 15 repeated target-acquisition trials.

## `figs/real_deployment_check.png`

Use: terminal screenshot of:

```bash
./scripts/tb4 paper-real-check
```

or an `rqt_graph` screenshot.

Must show:
- canonical `/robot1` and `/robot2` interfaces
- odometry and command topics
- scan topic state
- physical navigation is not claimed as a result

Caption already in paper:

> Physical deployment check for the canonical two-TurtleBot4 ROS 2 interfaces.
