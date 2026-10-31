# Line follower robot assets-only workspace

This workspace contains the robot URDF/meshes, Gazebo track world, CAD sources, and sensor/track simulation nodes. PID/RL controllers, the Gym environment, training scripts, and policy checkpoints were intentionally excluded.

Build and launch (from this workspace root):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select line_follower_py
source install/setup.bash
ros2 launch line_follower_py assets_only.launch.py
```

This launch starts Gazebo, the robot, track publisher, and line-sensor pipeline, but deliberately starts no controller. Add your new controller as a separate node publishing `/cmd_vel`. Keep `track_geometry.py`, `gazebo_line_sim.py`, and `track_world.sdf` aligned if you further change the track dimensions.
