import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    use_rviz = LaunchConfiguration("use_rviz")
    use_moveit = LaunchConfiguration("use_moveit")
    use_gui = LaunchConfiguration("use_gui")
    world = LaunchConfiguration("world")
    robot_ip = LaunchConfiguration("robot_ip")

    description_share = get_package_share_directory("dracoviloc_description")
    bringup_share = get_package_share_directory("dracoviloc_bringup")
    moveit_share = get_package_share_directory("dracoviloc_moveit_config")
    xacro_file = os.path.join(description_share, "urdf", "dracoviloc.urdf.xacro")
    controllers_file = os.path.join(bringup_share, "config", "ros2_controllers.yaml")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ", xacro_file,
                " sim:=", sim,
                " robot_ip:=", robot_ip,
                " ros2_controllers_file:=", controllers_file,
            ]),
            value_type=str,
        )
    }

    moveit_config = (
        MoveItConfigsBuilder("dracoviloc", package_name="dracoviloc_moveit_config")
        .robot_description(file_path=xacro_file, mappings={
            "sim": sim,
            "robot_ip": robot_ip,
            "ros2_controllers_file": controllers_file,
        })
        .robot_description_semantic(file_path="config/dracoviloc.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r ", world],
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(use_gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -s ", world],
            "on_exit_shutdown": "true",
        }.items(),
        condition=UnlessCondition(use_gui),
    )
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "dracoviloc", "-topic", "robot_description", "-z", "0.02"],
        output="screen",
        condition=IfCondition(sim),
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )
    hardware_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_file, {"use_sim_time": True}],
        condition=UnlessCondition(sim),
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "120"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "120"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        condition=IfCondition(use_moveit),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="true",
                              description="Spawn the FAIRINO in Gazebo instead of using real hardware. Gazebo remains the global clock in both modes."),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_moveit", default_value="true"),
        DeclareLaunchArgument("use_gui", default_value="true",
                              description="Reserved for Gazebo GUI selection."),
        DeclareLaunchArgument(
            "world", default_value=os.path.join(bringup_share, "worlds", "empty.sdf")
        ),
        DeclareLaunchArgument("robot_ip", default_value="192.168.58.2"),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            os.path.dirname(description_share),
        ),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            os.path.dirname(description_share),
        ),
        gazebo,
        gazebo_headless,
        clock_bridge,
        robot_state_publisher,
        hardware_control,
        spawn_robot,
        joint_state_spawner,
        arm_spawner,
        move_group,
        rviz,
    ])
