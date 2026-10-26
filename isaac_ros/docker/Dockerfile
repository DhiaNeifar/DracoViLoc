# DracoViLoc project image: the Isaac ROS development image plus everything
# the DracoViLoc YOLO packages need at build and run time, so that building
# inside the container never installs anything.
#
# Built by the control center (Setup & builds) or by hand:
#   docker build --build-arg BASE_IMAGE=isaac_ros_dev-aarch64:latest \
#     -t dracoviloc/isaac_ros:aarch64-latest isaac_ros/docker
# BASE_IMAGE is the Isaac ROS dev image for this CPU architecture
# (isaac_ros_dev-aarch64 on Jetson, isaac_ros_dev-x86_64 on x86).
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG DRACOVILOC_IMAGE_VERSION=dev
LABEL org.dracoviloc.image-version="${DRACOVILOC_IMAGE_VERSION}"

# Camera configuration (v4l2-ctl) and ROS image tools used by yolo_camera.launch.py.
RUN apt-get update \
    && apt-get install -y --no-install-recommends v4l-utils ros-humble-image-tools \
    && rm -rf /var/lib/apt/lists/*

# The Isaac ROS Python stack is built against NumPy 1.x.
RUN python3 -m pip install --no-cache-dir "numpy>=1.24.4,<2"
