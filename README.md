# DracoViLoc

ROS 2 Humble workspace for a single FAIRINO arm with a wrist-mounted D435i
camera and a simple kinematic drone in Gazebo Sim.

## Build and launch

```bash
cd ~/dracoviloc
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch dracoviloc_bringup drone_tracking_demo.launch.py
```

Run drone keyboard control in another terminal:

```bash
cd ~/dracoviloc
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dracoviloc_drone drone_keyboard_teleop
```

Controls: `w/s` forward/back, `a/d` left/right, `r/f` up/down, `q/e` yaw,
Space stops, `x` resets, and Esc exits. The reset pose is `(0, 0, 1.2)`;
the configured bounds are `x [-2, 3]`, `y [-2, 2]`, and `z [0.4, 2.5]`
metres.

## Tracking

The tracker reads the ground-truth `world -> drone_base_link` transform and
points the `+Z` axis of `d435i_link` toward the drone. MoveIt uses planning
group `arm` and controller `arm_controller`. The desired viewing ray is
published on `/tracking/desired_ray`.

Tracking currently uses current-state-seeded IK followed by short OMPL
trajectories; it is responsive but not continuous MoveIt Servo control. Drone
yaw alone does not request a new arm motion.

Gazebo owns the single global `/clock`. Do not run `drone.launch.py` alongside
`drone_tracking_demo.launch.py`, because each standalone launch owns its own
Gazebo instance and clock bridge.
