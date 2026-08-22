# audio_utils_msgs

ROS interfaces required by the bundled `odas_ros` package. This is a vendored
dependency and is built automatically with DracoViLoc:

```bash
colcon build --symlink-install --packages-up-to dracoviloc_odas
```

Applications should normally use the higher-level topics documented in
`src/dracoviloc_odas/README.md`.
