# Isaac ROS deployment

DracoViLoc's complete Isaac ROS overlay instructions live in
[`../isaac_ros/README.md`](../isaac_ros/README.md). That guide covers target
deployment, container builds, TensorRT engine generation, the bundled video
test, live acoustic-camera inference, model export, topics and troubleshooting.

The short workflow is:

```bash
cd ~/DracoViLoc
./scripts/deploy_isaac_ros.sh ~/workspaces/isaac_ros-dev

cd ~/workspaces/isaac_ros-dev
./src/isaac_ros_common/scripts/run_dev.sh
```

Then, inside the container:

```bash
/workspaces/isaac_ros-dev/scripts/build_dracoviloc_yolo.sh
source /workspaces/isaac_ros-dev/install/setup.bash
/workspaces/isaac_ros-dev/scripts/generate_tensorrt_engine.sh
ros2 launch isaac_ros_yolo_direction yolo_video.launch.py
```
