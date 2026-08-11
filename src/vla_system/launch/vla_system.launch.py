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
    enable_perception = LaunchConfiguration("enable_perception")
    enable_agent = LaunchConfiguration("enable_agent")
    enable_pick_bridge = LaunchConfiguration("enable_pick_bridge")
    skill_tier_enabled = LaunchConfiguration("skill_tier_enabled")
    rule_store_path = LaunchConfiguration("rule_store_path")

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
            # 카메라 인식 노드. 끄면 /vla/scene을 아무도 안 내보내므로 판단
            # 계층은 빈 테이블을 본다 -- eval/dryrun_stage.py가 그 자리를 대신
            # 채우는 용도다(GPU 없는 개발 머신, 또는 팔·카메라 없이 대화 흐름만
            # 볼 때). 둘을 동시에 띄우면 같은 토픽에 둘이 발행해 서로 덮는다.
            DeclareLaunchArgument("enable_perception", default_value="true"),
            DeclareLaunchArgument("enable_agent", default_value="true"),
            # Off by default: the cobot2_ws side of this integration (§9's
            # checklist, docs/state.md "cobot2_ws 통합") still has open
            # questions -- same-PC/domain unconfirmed, approval UX not
            # built, class allow-lists not reconciled. Turn on deliberately,
            # never as the default path, until those are answered.
            DeclareLaunchArgument("enable_pick_bridge", default_value="false"),
            # On by default -- see agent_node.py's declare_parameter for why.
            # NOTE: this argument only has effect because it is explicitly
            # forwarded into agent_node's parameters below. A launch argument
            # that is declared but never passed into a Node's `parameters=[]`
            # does nothing -- `ros2 launch ... skill_tier_enabled:=true` would
            # silently no-op without that forwarding, and it did exactly that
            # before this fix (2026-08-11): the flag was readable by `ros2 run`
            # with `--ros-args -p`, which bypasses this file entirely, but not
            # by `ros2 launch`, which is what the GUI actually uses.
            DeclareLaunchArgument("skill_tier_enabled", default_value="true"),
            DeclareLaunchArgument("rule_store_path",
                                  default_value="~/.ros/vla_rules.json"),
            realsense,
            Node(
                package="vla_system",
                executable="perception_node",
                name="vla_perception",
                output="screen",
                condition=IfCondition(enable_perception),
                parameters=[params_file],
            ),
            Node(
                package="vla_system",
                executable="agent_node",
                name="vla_agent",
                output="screen",
                condition=IfCondition(enable_agent),
                parameters=[params_file, {
                    "skill_tier_enabled": skill_tier_enabled,
                    "rule_store_path": rule_store_path,
                }],
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
