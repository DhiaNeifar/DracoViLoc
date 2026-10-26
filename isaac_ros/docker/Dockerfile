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

# YOLO -> ONNX export environment, baked into the image so it is ready every
# time the container starts. It is a separate virtual environment: the CPU
# build of PyTorch it uses never touches the image's GPU PyTorch, and export
# runs on the CPU (exactly like isaac_ros/tools/export_yolo.py), so the same
# environment works on Jetson (aarch64) and x86. Versions are pinned to the
# Ultralytics release the drone models were trained with (8.4.129).
ARG YOLO_EXPORT_VENV=/opt/dracoviloc/yolo_export
COPY yolo_export_requirements.txt /opt/dracoviloc/yolo_export_requirements.txt
RUN python3 -m venv ${YOLO_EXPORT_VENV} \
    && ${YOLO_EXPORT_VENV}/bin/pip install --no-cache-dir "pip==25.3" \
    && ${YOLO_EXPORT_VENV}/bin/pip install --no-cache-dir \
         --index-url https://download.pytorch.org/whl/cpu "torch==2.5.1" "torchvision==0.20.1" \
    && ${YOLO_EXPORT_VENV}/bin/pip install --no-cache-dir --no-deps \
         -r /opt/dracoviloc/yolo_export_requirements.txt \
    && ${YOLO_EXPORT_VENV}/bin/pip check \
    && ${YOLO_EXPORT_VENV}/bin/python -c "import ultralytics, onnx, onnxslim, torch, torchvision"

# Never let Ultralytics install packages or download assets at export time.
ENV YOLO_EXPORT_PYTHON=${YOLO_EXPORT_VENV}/bin/python \
    YOLO_AUTOINSTALL=false \
    YOLO_OFFLINE=1 \
    YOLO_CONFIG_DIR=/tmp/Ultralytics
