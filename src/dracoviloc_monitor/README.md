# dracoviloc_monitor

Native (C++) monitoring nodes for the control center. They do the heavy work
so the Python GUI only handles small messages.

| Node | Input | Output |
|---|---|---|
| `preview_node` | `/image/compressed` (JPEG from the camera node), `/detections_output`, `/yolov8_encoder/resize/camera_info` | `/dracoviloc/preview/raw/compressed` (camera JPEG, forwarded unchanged) and `/dracoviloc/preview/overlay/compressed` (the same frame with YOLO boxes, matched by header stamp) |
| `telemetry_node` | camera info, detections, direction topics, `/ekf/diagnostics`, `/mobilenetv2/diagnostics`, `/joint_states`, `/ee/*`, `/sst` | `/dracoviloc/telemetry` (`std_msgs/String`, JSON, 10 Hz) and optionally `telemetry.jsonl` (2 Hz) |

Raw 640x480 frames (~1 MB) are never moved through the monitoring path: the
DDS transport here is UDP only and drops whole frames. The camera node
publishes a JPEG copy (`/image/compressed`, same stamps) only while someone
subscribes. Both previews are encoded only while subscribed; the raw
recording (`dracoviloc_recording`) still records the full `/image`.

`monitor.launch.py` starts both nodes and, with `log_dir:=<folder>`, writes
`telemetry.jsonl` and records a ROS bag (`bag/`) of every diagnostics and
state topic (no images) for offline analysis (`ros2 bag info`, `ros2 bag play`).
`arm_demo.launch.py` includes it (`monitor_enabled:=true`, `run_log_dir:=`).

Telemetry snapshot (schema 1): `camera` (fps, size, latency), `yolo` (inference
and detection rates, best score, latency), `directions` (yolo / mobilenetv2 /
ekf: rate, age, vector, azimuth/elevation in degrees), `mobilenetv2` (window
rate, inference time, per-channel probability and votes), `ekf` (per source:
rate, decision counts, last NIS and innovation), `joints` (per joint position,
velocity, acceleration, jerk), `ee` (end-effector speed, acceleration, jerk),
`audio` (ODAS tracked sources).
