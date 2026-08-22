# dracoviloc_description

URDF/Xacro, meshes, initial joint positions and ros2_control declarations for
the single FAIRINO arm and its attached sensor frames.

This package is normally consumed through `dracoviloc_bringup`:

```bash
ros2 launch dracoviloc_bringup demo.launch.py sim:=true
```

Edit fixed sensor mounting transforms in `urdf/dracoviloc.urdf.xacro`. Rebuild
this package and restart the launch after changing Xacro or configuration:

```bash
colcon build --symlink-install --packages-select dracoviloc_description
```
