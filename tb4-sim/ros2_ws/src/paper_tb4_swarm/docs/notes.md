# Design Notes

The paper's full swarm concept is projected onto two TurtleBot4 robots because
the available hardware is two robots. The roles are:

- leader: robot1 drives to the operator-selected target;
- follower: robot2 tracks a dynamic pose behind robot1.

The default stable launch intentionally avoids render sensors. The previous
project crashed in Gazebo's render sensor thread with EGL/OGRE errors, so the
stable launch is the practical path for repeatable experiments on this machine.

The optional official launch uses TurtleBot4's installed simulator launch files:

- `turtlebot4_gz_bringup/launch/turtlebot4_spawn.launch.py`
- `ros_gz_sim/launch/gz_sim.launch.py`

It is server-only by default.
