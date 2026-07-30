# ROS 2 Humble and Gazebo Fortress Installation

This project targets Ubuntu 22.04 (Jammy), ROS 2 Humble, Gazebo Fortress,
MoveIt 2, `ros_gz`, and `gz_ros2_control`. Gazebo Classic is not used.

Official references:

- [ROS 2 Humble Debian installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Gazebo Fortress with ROS](https://gazebosim.org/docs/fortress/ros_installation/)
- [Gazebo Fortress Ubuntu binaries](https://gazebosim.org/docs/fortress/install_ubuntu/)

## 1. Configure locale

```bash
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

## 2. Add the ROS 2 repository

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

export ROS_APT_SOURCE_VERSION=$(
  curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
  grep -F '"tag_name"' | awk -F\" '{print $4}'
)

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

## 3. Install ROS 2 and project dependencies

`ros-humble-desktop` already includes the ROS base installation, RViz, and
common development tools; installing `ros-humble-ros-base` separately is not
necessary.

```bash
sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-moveit \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-planners-ompl \
  ros-humble-moveit-ros-visualization \
  ros-humble-moveit-simple-controller-manager \
  ros-humble-tf2-ros \
  ros-humble-tf2-eigen \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-gz-ros2-control
```

Optional packages:

```bash
sudo apt install -y \
  ros-humble-pick-ik \
  ros-humble-geometric-shapes
```

The project currently uses KDL and OMPL, so PickIK is not required for the
default demonstration.

## 4. Install Gazebo Fortress

Installing `ros-humble-ros-gz` selects the Gazebo version paired with ROS 2
Humble. If the standalone Fortress tools, libraries, and `ign gazebo` command
are not already installed, add the OSRF repository:

```bash
sudo apt install -y lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

sudo apt update
sudo apt install -y ignition-fortress
```

This workspace uses Gazebo Fortress's `ign gazebo` command and the ROS package
`gz_ros2_control`. Do not install or use Gazebo Classic packages such as
`gazebo_ros` or `gazebo_ros2_control`.

## 5. Configure the shell

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

## 6. Install workspace dependencies and build

```bash
cd ~/dracoviloc
source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

The real FAIRINO hardware package may additionally require the vendor SDK and
libraries. Those are not required for `sim:=true`.

## 7. Verify the installation

```bash
ros2 run demo_nodes_cpp talker
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

Test Gazebo:

```bash
ign gazebo shapes.sdf
```

Launch the DracoViLoc simulation:

```bash
cd ~/dracoviloc
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dracoviloc_bringup drone_tracking_demo.launch.py
```
