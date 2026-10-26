"""RViz on its own, with the same configuration as demo.launch.py.

Lets the control center close or restart RViz without touching the running
robot stack (RViz is not required by any other node)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _rviz(context):
    dual_arm = LaunchConfiguration("dual_arm").perform(context).lower() in ("1", "true", "yes", "on")
    description_share = get_package_share_directory("dracoviloc_description")
    moveit_share = get_package_share_directory("dracoviloc_moveit_config")
    xacro_file = os.path.join(description_share, "urdf", "dracoviloc.urdf.xacro")
    srdf = "config/dracoviloc_dual.srdf" if dual_arm else "config/dracoviloc_single.srdf"
    # Only the kinematic model matters to RViz; hardware arguments keep their defaults.
    moveit_config = (MoveItConfigsBuilder("dracoviloc", package_name="dracoviloc_moveit_config")
                     .robot_description(file_path=xacro_file,
                                        mappings={"dual_arm": "true" if dual_arm else "false"})
                     .robot_description_semantic(file_path=srdf)
                     .robot_description_kinematics(file_path="config/kinematics.yaml")
                     .joint_limits(file_path="config/joint_limits.yaml")
                     .planning_pipelines(pipelines=["ompl"]).to_moveit_configs())
    return [Node(package="rviz2", executable="rviz2", output="screen",
                 arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
                 parameters=[moveit_config.to_dict(), {"use_sim_time": False}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("dual_arm", default_value="false"),
        OpaqueFunction(function=_rviz),
    ])
