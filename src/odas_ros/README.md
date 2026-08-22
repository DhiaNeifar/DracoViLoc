# odas_ros

Bundled ROS 2 wrapper and C source for ODAS. Its original configuration files
are retained under `config/`, including `configuration.cfg`; DracoViLoc does
not require a separate ODAS workspace.

Normally launch it through the integration package:

```bash
ros2 launch dracoviloc_odas audio_bringup.launch.py use_sim_time:=false
```

The wrapper starts `odas_core_node` and the visualization publishers. See
`src/dracoviloc_odas/README.md` for the UMA16v2 feeder, device ownership,
frames, complete arm demo, and topics.
