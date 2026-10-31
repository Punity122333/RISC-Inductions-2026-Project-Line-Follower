# Line follower robot assets-only workspace

This workspace contains the robot URDF/meshes, Gazebo track world, CAD sources, sensor/track simulation nodes, and a camera-based PD controller. The Gym environment, training scripts, and policy checkpoints are not included.

Build and launch (from this workspace root):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select line_follower_py
source install/setup.bash
ros2 launch line_follower_py assets_only.launch.py
```

This launch starts Gazebo, the robot, track publisher, camera-based line detector, and PD controller. The line detector publishes `/line_error`, which the PD controller uses to publish `/cmd_vel`. Keep `track_geometry.py`, `gazebo_line_sim.py`, and `track_world.sdf` aligned if you further change the track dimensions.
