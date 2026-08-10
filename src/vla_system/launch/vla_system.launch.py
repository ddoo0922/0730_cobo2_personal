"""Bring up perception and agent. Robot execution is off by default -- cobot2_ws's
pick_fsm owns the M0609/RG2 hardware; vla_robot and gripper.py must not run
alongside it (shared DRFL connection / Modbus register, see md/plans/
2026-08-08-vla-integration.md #5-3 in cobot2_ws). Set enable_robot:=true only
for this node's own standalone DRY-RUN testing with no cobot2_ws FSM running."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    enable_realsense = LaunchConfiguration("enable_realsense")
    enable_agent = LaunchConfiguration("enable_agent")
    motion_enabled = LaunchConfiguration("motion_enabled")
    enable_wrist_grasp = LaunchConfiguration("enable_wrist_grasp")
    enable_robot = LaunchConfiguration("enable_robot")

    realsense = GroupAction(
        condition=IfCondition(enable_realsense),
        scoped=True,
        forwarding=False,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                    )
                ),
                # The RealSense is on the wrist now and nothing in the pipeline
                # reads it -- perception runs off the fixed webcam, and the GUI
                # only displays this colour stream. Depth stays on for the
                # wrist-guided grasping still to come, but the pointcloud is
                # dropped: nobody subscribed to it, and it shares a USB
                # controller with the webcam.
                launch_arguments={
                    "enable_color": "true",
                    "enable_depth": "true",
                    "rgb_camera.color_profile": "640x480x30",
                    "depth_module.depth_profile": "640x480x30",
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                    "pointcloud.enable": "false",
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("vla_system"), "config", "system.yaml"]
                ),
            ),
            DeclareLaunchArgument("enable_realsense", default_value="true"),
            DeclareLaunchArgument("enable_agent", default_value="true"),
            DeclareLaunchArgument("motion_enabled", default_value="false"),
            DeclareLaunchArgument("enable_wrist_grasp", default_value="false"),
            # Off by default: cobot2_ws's pick_fsm owns the robot/gripper now.
            # See the launch-time docstring above before flipping this on.
            DeclareLaunchArgument("enable_robot", default_value="false"),
            realsense,
            Node(
                package="vla_system",
                executable="perception_node",
                name="vla_perception",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="vla_system",
                executable="agent_node",
                name="vla_agent",
                output="screen",
                condition=IfCondition(enable_agent),
                parameters=[params_file],
            ),
            Node(
                package="vla_system",
                executable="robot_node",
                name="vla_robot",
                output="screen",
                condition=IfCondition(enable_robot),
                parameters=[params_file, {"motion_enabled": motion_enabled}],
            ),
            # Wrist-guided grasping. Off by default at launch level because it
            # loads a second YOLO plus GraspGenX (~1.2 GB VRAM) and is only
            # useful with the RealSense actually mounted on the arm.
            Node(
                package="vla_system",
                executable="wrist_grasp_node",
                name="vla_wrist",
                output="screen",
                condition=IfCondition(enable_wrist_grasp),
                parameters=[params_file],
            ),
        ]
    )
