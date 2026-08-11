"""Bring up perception and agent. Robot execution is off by default -- cobot2_ws's
pick_fsm owns the M0609/RG2 hardware; vla_robot and gripper.py must not run
alongside it (shared DRFL connection / Modbus register, see md/plans/
2026-08-08-vla-integration.md #5-3 in cobot2_ws). Set enable_robot:=true only
for this node's own standalone DRY-RUN testing with no cobot2_ws FSM running.

enable_pick_bridge and enable_robot are mutually exclusive: both subscribe
/vla/robot/action and publish /vla/robot/state, so running both means two
processes racing to answer the agent. enable_pick_bridge is the integration
path (forwards to cobot2_ws's pick_fsm over /vla/pick_command); enable_robot
is this node's own standalone arm control, off by default for the reason
above."""

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
            # Off by default: the cobot2_ws side of this integration (§9's
            # checklist, docs/state.md "cobot2_ws 통합") still has open
            # questions -- same-PC/domain unconfirmed, approval UX not
            # built, class allow-lists not reconciled. Turn on deliberately,
            # never as the default path, until those are answered.
            DeclareLaunchArgument("enable_pick_bridge", default_value="false"),
            # Off by default -- see agent_node.py's declare_parameter for why.
            # NOTE: this argument only has effect because it is explicitly
            # forwarded into agent_node's parameters below. A launch argument
            # that is declared but never passed into a Node's `parameters=[]`
            # does nothing -- `ros2 launch ... skill_tier_enabled:=true` would
            # silently no-op without that forwarding, and it did exactly that
            # before this fix (2026-08-11): the flag was readable by `ros2 run`
            # with `--ros-args -p`, which bypasses this file entirely, but not
            # by `ros2 launch`, which is what the GUI actually uses.
            DeclareLaunchArgument("skill_tier_enabled", default_value="false"),
            DeclareLaunchArgument("rule_store_path",
                                  default_value="~/.ros/vla_rules.json"),
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
                parameters=[params_file, {
                    "skill_tier_enabled": skill_tier_enabled,
                    "rule_store_path": rule_store_path,
                }],
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
            # cobot2_ws integration: forwards RobotAction to pick_fsm instead
            # of moving an arm here. See the module docstring above --
            # mutually exclusive with enable_robot.
            #
            # This alone does not start cobot2_ws's FSM cycle: pick_fsm sits
            # in IDLE until /pick/start is called (a human button, or
            # cobot2_ws's own vla_command_node with auto_start:=true -- a
            # different launch file in a different clone, not started from
            # here). See README.md #3 "enable_pick_bridge:=true만으로는...".
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
