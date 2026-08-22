# dracoviloc_moveit_config

MoveIt 2 semantic model, kinematics, joint limits, OMPL configuration,
controller mapping and RViz configuration for planning group `arm`.

```bash
ros2 launch dracoviloc_bringup demo.launch.py \
  sim:=true use_moveit:=true use_rviz:=true
```

The continuous audio tracker does not use MoveIt; it commands only joints 1 and
4 directly. This package remains available for interactive planning and future
collision-aware motions.
