import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("dracoviloc_audio_fusion")

    tracking_frame = LaunchConfiguration("tracking_frame")
    min_confidence = LaunchConfiguration("min_confidence")
    gre_trust = LaunchConfiguration("gre_trust")
    use_tf_for_visual = LaunchConfiguration("use_tf_for_visual")
    min_activity = LaunchConfiguration("min_activity")
    channels = LaunchConfiguration("channels")

    ast_enabled = LaunchConfiguration("ast_enabled")
    threshold = LaunchConfiguration("threshold")
    consecutive = LaunchConfiguration("consecutive")
    always_classify = LaunchConfiguration("always_classify")
    ast_venv_python = LaunchConfiguration("ast_venv_python")
    ast_engine_path = LaunchConfiguration("ast_engine_path")
    ast_project_dir = LaunchConfiguration("ast_project_dir")

    gre_enabled = LaunchConfiguration("gre_enabled")
    gre_reset_cooldown = LaunchConfiguration("gre_reset_cooldown")
    gre_verbose = LaunchConfiguration("gre_verbose")
    gre_venv_python = LaunchConfiguration("gre_venv_python")
    gre_engine_path = LaunchConfiguration("gre_engine_path")
    gre_repo_dir = LaunchConfiguration("gre_repo_dir")

    audio_enabled = LaunchConfiguration("audio_enabled")
    visual_enabled = LaunchConfiguration("visual_enabled")
    ekf_enabled = LaunchConfiguration("ekf_enabled")

    odas_ekf_adapter = Node(
        package="dracoviloc_audio_fusion",
        executable="odas_ekf_adapter",
        parameters=[{
            "use_sim_time": True,
            "min_activity": ParameterValue(min_activity, value_type=float),
        }],
        output="screen",
        condition=IfCondition(ekf_enabled))

    ekf_fusion = Node(
        package="dracoviloc_audio_fusion",
        executable="ekf_fusion_node",
        parameters=[{
            "use_sim_time": True,
            "tracking_frame": ParameterValue(tracking_frame, value_type=str),
            "min_confidence": ParameterValue(min_confidence, value_type=float),
            "gre_trust": ParameterValue(gre_trust, value_type=float),
            "use_tf_for_visual": ParameterValue(use_tf_for_visual, value_type=bool),
            "audio_enabled": ParameterValue(audio_enabled, value_type=bool),
            "visual_enabled": ParameterValue(visual_enabled, value_type=bool),
        }],
        output="screen",
        condition=IfCondition(ekf_enabled))

    # AST and GRE each need TensorRT/pycuda and other packages that live only
    # in their own venv (trt_env / gre_env), not the system ROS Python, so
    # neither is a ros2-run entry point - both run as raw processes against
    # their venv's own interpreter via ExecuteProcess. That interpreter still
    # sees rclpy and the workspace message packages because ExecuteProcess
    # children inherit the PYTHONPATH of the shell that ran `ros2 launch`
    # (i.e. wherever ROS and DracoViLoc's install/setup.bash were already
    # sourced) - no `source .../activate` step is needed here.
    #
    # Independent by design: ast_enabled and gre_enabled each gate their own
    # process. Either can run alone, both can run together (ekf_fusion_node
    # already supports two classifiers via gre_trust weighting - see its
    # "TWO CLASSIFIERS, ONE GATE" docstring), or neither, leaving the adapter
    # and EKF idle with nothing to fuse.
    ast_classifier = ExecuteProcess(
        cmd=[
            ast_venv_python,
            PathJoinSubstitution([ast_project_dir, "ast_classifier_node.py"]),
            "--project-dir", ast_project_dir,
            "--engine", ast_engine_path,
            "--channels", channels,
            "--threshold", threshold,
            "--consecutive", consecutive,
            "--min-activity", min_activity,
            "--always-classify", always_classify,
        ],
        name="ast_classifier_node",
        output="screen",
        emulate_tty=True,
        additional_env={"PYTHONUNBUFFERED": "1"},
        condition=IfCondition(ast_enabled))

    # PLACEHOLDER - see yolo_ekf_adapter.py's module docstring for what this
    # does and does not do (nearest-to-boresight target selection, no
    # confidence, no real data association). Plain rclpy/geometry_msgs, so
    # unlike AST/GRE it runs as a normal ros2-run entry point, no venv needed.
    yolo_adapter = Node(
        package="dracoviloc_audio_fusion",
        executable="yolo_ekf_adapter",
        output="screen",
        condition=IfCondition(visual_enabled))

    gre_classifier = ExecuteProcess(
        cmd=[
            gre_venv_python,
            PathJoinSubstitution([package_share, "gre", "gre_classifier_node.py"]),
            "--repo", gre_repo_dir,
            "--engine", gre_engine_path,
            "--channels", channels,
            "--min-activity", min_activity,
            "--reset-cooldown", gre_reset_cooldown,
            "--verbose", gre_verbose,
        ],
        name="gre_classifier_node",
        output="screen",
        condition=IfCondition(gre_enabled))

    return LaunchDescription([
        DeclareLaunchArgument(
            "tracking_frame", default_value="table_mic_link",
            description="TF frame the fused bearing is stamped in; must match "
                        "the frame_id ODAS was launched with (dracoviloc_odas "
                        "uses table_mic_link)."),
        DeclareLaunchArgument(
            "min_confidence", default_value="0.20",
            description="Minimum classifier confidence for the EKF to accept "
                        "an acoustic bearing."),
        DeclareLaunchArgument(
            "gre_trust", default_value="0.0",
            description="Weight on GRE's confidence relative to AST's, when "
                        "both are enabled. 0.0 means AST effectively decides "
                        "alone even if GRE is also running."),
        DeclareLaunchArgument("use_tf_for_visual", default_value="false"),
        DeclareLaunchArgument(
            "min_activity", default_value="0.1",
            description="Shared ODAS-track activity floor for the adapter "
                        "and both classifiers."),
        DeclareLaunchArgument(
            "channels", default_value="4",
            description="Shared: must equal len(sst.N_inactive) in "
                        "configuration.cfg. Used by both classifiers."),

        DeclareLaunchArgument(
            "ast_enabled", default_value="true",
            description="Run the AST classifier."),
        DeclareLaunchArgument("threshold", default_value="0.2"),
        DeclareLaunchArgument("consecutive", default_value="3"),
        DeclareLaunchArgument(
            "always_classify", default_value="false",
            description="true: AST classifies every /sss channel "
                        "continuously, ignoring whether ODAS has formed a "
                        "track. For validating the model/engine independent "
                        "of ODAS SSL/SST tuning; these classifications will "
                        "not gate the EKF (synthetic negative track ids)."),
        DeclareLaunchArgument(
            "ast_venv_python",
            default_value=PathJoinSubstitution(
                [EnvironmentVariable("HOME"), "DracoViLoc", "trt_env", "bin", "python3"]),
            description="Interpreter with TensorRT/pycuda/transformers "
                        "installed (--system-site-packages venv)."),
        DeclareLaunchArgument(
            "ast_engine_path",
            default_value=os.path.join(package_share, "models", "drone_ast.engine"),
            description="Machine-specific TensorRT engine; not tracked in "
                        "git, copy it here once per Jetson."),
        DeclareLaunchArgument(
            "ast_project_dir",
            default_value=os.path.join(package_share, "ast")),

        DeclareLaunchArgument(
            "gre_enabled", default_value="false",
            description="Run the GRE classifier."),
        DeclareLaunchArgument(
            "gre_reset_cooldown", default_value="2.0",
            description="Minimum seconds between GRE detector "
                        "reconstructions on track loss/change."),
        DeclareLaunchArgument("gre_verbose", default_value="false"),
        DeclareLaunchArgument(
            "gre_venv_python",
            default_value=PathJoinSubstitution(
                [EnvironmentVariable("HOME"), "DracoViLoc", "gre_env", "bin", "python3"]),
            description="Interpreter with TensorRT/pycuda/soundfile/pandas "
                        "installed (--system-site-packages venv)."),
        DeclareLaunchArgument(
            "gre_engine_path",
            default_value=os.path.join(package_share, "models", "model_logmel.engine"),
            description="Machine-specific TensorRT engine; not tracked in "
                        "git, copy it here once per Jetson."),
        DeclareLaunchArgument(
            "gre_repo_dir",
            default_value=os.path.join(package_share, "gre", "repo")),

        DeclareLaunchArgument(
            "audio_enabled", default_value="true",
            description="EKF-level gate: ignore /odas/sst even if ODAS and "
                        "the classifiers are running. Independent of "
                        "ast_enabled/gre_enabled, which instead control "
                        "whether those nodes run at all. Normally forwarded "
                        "from the bringup launch's own audio_enabled."),
        DeclareLaunchArgument(
            "ekf_enabled", default_value="false",
            description="Start odas_ekf_adapter and ekf_fusion_node. Defaults "
                        "off so AST/GRE can be tested without fusion."),
        DeclareLaunchArgument(
            "visual_enabled", default_value="false",
            description="Runs yolo_ekf_adapter (PLACEHOLDER - see its "
                        "docstring) and enables the EKF's visual measurement "
                        "path. Independent of audio_enabled - either, both, "
                        "or neither. Defaults off: unlike AST, this bridge "
                        "has not been validated against real detections."),

        odas_ekf_adapter,
        ast_classifier,
        gre_classifier,
        yolo_adapter,
        ekf_fusion,
    ])
