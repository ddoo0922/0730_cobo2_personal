"""Bring up perception, agent, and the cobot2_ws bridge.

This ws no longer owns any robot/gripper execution -- robot_node.py (its own
Doosan/gripper control) and wrist_grasp_node.py (GraspGenX precision grasp,
whose only consumer was robot_node) were removed once cobot2_ws's pick_fsm
became the sole executor (CLAUDE.md #3). vla_pick_bridge_node is now the only
thing that turns an agent decision into a motion, by forwarding to pick_fsm
over /vla/pick_command -- it never moves an arm itself.

enable_pick_bridge defaults off here so a bare launch (e.g. running tests, or
perception-only debugging) never fires a command at cobot2_ws by accident;
turn it on deliberately when cobot2_ws's pick_fsm is actually up. This alone
does not start pick_fsm's cycle either: it sits in IDLE until /pick/start is
called (a human button, or cobot2_ws's own vla_command_node with
auto_start:=true -- a different launch file in a different clone, not started
from here). See README.md #3 "enable_pick_bridge:=true만으로는...".
"""

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
    enable_pick_bridge = LaunchConfiguration("enable_pick_bridge")

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
                # cobot2_ws's pick_fsm shares this same physical D435i when
                # enable_pick_bridge is on -- see start_pipeline() in vla_gui.py,
                # which pairs enable_pick_bridge:=true with
                # enable_realsense:=false so this ws never opens a second V4L2
                # handle onto a camera cobot2_ws's own launch already owns.
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
            # Off by default: the cobot2_ws side of this integration (§9's
            # checklist, docs/state.md "cobot2_ws 통합") still has open
            # questions -- same-PC/domain unconfirmed, approval UX not
            # built, class allow-lists not reconciled. Turn on deliberately,
            # never as the default path, until those are answered.
            DeclareLaunchArgument("enable_pick_bridge", default_value="false"),
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
            # The only executor: forwards RobotAction to cobot2_ws's pick_fsm
            # instead of moving an arm here.
            Node(
                package="vla_system",
                executable="vla_pick_bridge_node",
                name="vla_pick_bridge",
                output="screen",
                condition=IfCondition(enable_pick_bridge),
                parameters=[params_file],
            ),
        ]
    )
