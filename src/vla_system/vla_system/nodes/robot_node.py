#!/usr/bin/env python3
"""Executes the actions the agent decided on, and owns the stop path.

Two things live here that deliberately do not live in the agent:

1. **Ground truth.** RobotState is published from what actually happened to the
   arm, never from what the LLM believes. It is injected into every LLM call so
   a drifting conversational memory can not produce a "release" while the
   gripper is empty.

2. **The stop path.** ``/vla/robot/stop`` bypasses the LLM entirely. The GUI
   matches the keyword locally and publishes here; this node sets the cancel
   event and fires Doosan's ``motion/move_stop`` service. No API round-trip
   stands between the user saying "정지" and the arm braking.

Only one action is ever in flight. A second action arriving while the arm is
busy is rejected rather than queued -- the old FIFO queue was how a stale
target could still reach the arm three decisions after the user changed
their mind.
"""

from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from vla_interfaces.msg import (
    GraspPlan,
    GraspRequest,
    RobotAction,
    RobotState,
    SceneSnapshot,
    TcpPose,
)

from vla_system.robot.moves import (
    DoosanArm,
    DryRunArm,
    MotionCancelled,
    MotionError,
    MotionProfile,
    WorkspaceBounds,
)

PICK_ACTIONS = ("pick_and_place", "pick_and_hold")
KNOWN_ACTIONS = PICK_ACTIONS + ("release", "go_home")

# From dsr_msgs2/srv/MoveStop.srv.
DR_QSTOP_STO = 0
DR_QSTOP = 1
DR_SSTOP = 2
DR_HOLD = 3


@dataclass
class Holding:
    object_id: str
    class_name: str


class RobotNode(Node):
    def __init__(self):
        super().__init__("vla_robot")

        self.declare_parameter("action_topic", "/vla/robot/action")
        self.declare_parameter("stop_topic", "/vla/robot/stop")
        self.declare_parameter("estop_topic", "/vla/estop")
        self.declare_parameter("state_topic", "/vla/robot/state")
        self.declare_parameter("scene_topic", "/vla/scene")

        self.declare_parameter("motion_enabled", False)
        self.declare_parameter("robot_id", "dsr01")
        self.declare_parameter("robot_model", "m0609")
        self.declare_parameter("stop_mode", DR_SSTOP)
        self.declare_parameter("max_scene_age_s", 2.0)
        self.declare_parameter("dry_run_motion_s", 1.2)

        self.declare_parameter("velocity", 30.0)
        self.declare_parameter("acceleration", 30.0)
        self.declare_parameter("approach_height_m", 0.05)
        self.declare_parameter("lift_height_m", 0.05)
        self.declare_parameter("place_joints", [0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
        self.declare_parameter("home_joints", [0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
        self.declare_parameter("poll_interval_s", 0.02)
        self.declare_parameter("motion_start_grace_s", 0.3)
        self.declare_parameter("motion_timeout_s", 30.0)
        self.declare_parameter("motion_position_tolerance_m", 0.01)
        self.declare_parameter("motion_orientation_tolerance_deg", 3.0)
        self.declare_parameter("motion_joint_tolerance_deg", 1.0)

        self.declare_parameter("workspace_min_x_m", 0.2)
        self.declare_parameter("workspace_max_x_m", 0.9)
        self.declare_parameter("workspace_min_y_m", -0.6)
        self.declare_parameter("workspace_max_y_m", 0.6)
        self.declare_parameter("workspace_min_z_m", 0.02)
        self.declare_parameter("workspace_max_z_m", 0.8)

        # Named controller presets. tcp_name must match the TCP the wrist
        # hand-eye calibration was recorded with, or every wrist-planned grasp
        # lands in the wrong frame.
        self.declare_parameter("tool_name", "Tool Weight")
        self.declare_parameter("tcp_name", "GripperDA_v1")
        self.declare_parameter("observe_height_m", 0.25)
        self.declare_parameter("observe_ik_position_tolerance_m", 0.005)
        self.declare_parameter("observe_ik_orientation_tolerance_deg", 2.0)
        self.declare_parameter("observe_max_camera_tilt_deg", 10.0)
        self.declare_parameter("observe_transition_z_m", 0.28)
        self.declare_parameter("observe_camera_inset_max_m", 0.14)
        self.declare_parameter("observe_camera_inset_step_m", 0.02)
        self.declare_parameter("handeye_calibration", "")

        # Wrist-guided grasping. When false the arm uses the webcam position
        # directly, which is the behaviour that existed before GraspGenX.
        self.declare_parameter("use_wrist_grasp", True)
        self.declare_parameter("wrist_grasp_timeout_s", 8.0)
        self.declare_parameter("wrist_grasp_fallback", True)
        self.declare_parameter("grasp_request_topic", "/vla/grasp/request")
        self.declare_parameter("grasp_plan_topic", "/vla/grasp/plan")
        self.declare_parameter("tcp_pose_topic", "/vla/robot/tcp_pose")
        self.declare_parameter("tcp_pose_sample_period_s", 0.1)
        self.declare_parameter("tcp_pose_service_timeout_s", 0.25)

        self.declare_parameter("gripper_model", "rg2")
        self.declare_parameter("gripper_ip", "192.168.1.1")
        self.declare_parameter("gripper_port", 502)
        self.declare_parameter("gripper_force_n", 40.0)
        self.declare_parameter("grip_settle_s", 0.5)

        self.motion_enabled = bool(self.get_parameter("motion_enabled").value)
        self.robot_id = str(self.get_parameter("robot_id").value)
        self.stop_mode = int(self.get_parameter("stop_mode").value)
        self.max_scene_age_s = float(self.get_parameter("max_scene_age_s").value)

        bounds = WorkspaceBounds(
            min_x_m=float(self.get_parameter("workspace_min_x_m").value),
            max_x_m=float(self.get_parameter("workspace_max_x_m").value),
            min_y_m=float(self.get_parameter("workspace_min_y_m").value),
            max_y_m=float(self.get_parameter("workspace_max_y_m").value),
            min_z_m=float(self.get_parameter("workspace_min_z_m").value),
            max_z_m=float(self.get_parameter("workspace_max_z_m").value),
        )
        profile = MotionProfile(
            velocity=float(self.get_parameter("velocity").value),
            acceleration=float(self.get_parameter("acceleration").value),
            approach_height_m=float(self.get_parameter("approach_height_m").value),
            lift_height_m=float(self.get_parameter("lift_height_m").value),
            place_joints=tuple(self.get_parameter("place_joints").value),
            home_joints=tuple(self.get_parameter("home_joints").value),
            gripper_force_n=float(self.get_parameter("gripper_force_n").value),
            grip_settle_s=float(self.get_parameter("grip_settle_s").value),
            poll_interval_s=float(self.get_parameter("poll_interval_s").value),
            motion_start_grace_s=float(
                self.get_parameter("motion_start_grace_s").value
            ),
            motion_timeout_s=float(self.get_parameter("motion_timeout_s").value),
            motion_position_tolerance_m=float(
                self.get_parameter("motion_position_tolerance_m").value
            ),
            motion_orientation_tolerance_deg=float(
                self.get_parameter("motion_orientation_tolerance_deg").value
            ),
            motion_joint_tolerance_deg=float(
                self.get_parameter("motion_joint_tolerance_deg").value
            ),
            tool_name=str(self.get_parameter("tool_name").value),
            tcp_name=str(self.get_parameter("tcp_name").value),
            observe_height_m=float(self.get_parameter("observe_height_m").value),
            observe_ik_position_tolerance_m=float(
                self.get_parameter("observe_ik_position_tolerance_m").value
            ),
            observe_ik_orientation_tolerance_deg=float(
                self.get_parameter("observe_ik_orientation_tolerance_deg").value
            ),
            observe_max_camera_tilt_deg=float(
                self.get_parameter("observe_max_camera_tilt_deg").value
            ),
            observe_transition_z_m=float(
                self.get_parameter("observe_transition_z_m").value
            ),
            observe_camera_inset_max_m=float(
                self.get_parameter("observe_camera_inset_max_m").value
            ),
            observe_camera_inset_step_m=float(
                self.get_parameter("observe_camera_inset_step_m").value
            ),
        )
        self.tcp_name = profile.tcp_name
        self.use_wrist_grasp = bool(self.get_parameter("use_wrist_grasp").value)
        self.wrist_grasp_timeout_s = float(
            self.get_parameter("wrist_grasp_timeout_s").value
        )
        self.wrist_grasp_fallback = bool(
            self.get_parameter("wrist_grasp_fallback").value
        )
        self.tcp_pose_sample_period_s = float(
            self.get_parameter("tcp_pose_sample_period_s").value
        )
        self.tcp_pose_service_timeout_s = float(
            self.get_parameter("tcp_pose_service_timeout_s").value
        )
        if (
            self.tcp_pose_sample_period_s <= 0.0
            or self.tcp_pose_service_timeout_s <= 0.0
        ):
            raise ValueError("TCP pose sample period and timeout must be positive")

        # ---------------------------------------------------------- state

        self.scene: SceneSnapshot | None = None
        self.scene_monotonic = 0.0
        self.scene_lock = threading.Lock()

        self.holding: Holding | None = None
        self.status = "idle"
        self.current: RobotAction | None = None
        self.last_action_id = ""
        self.last_action = ""
        self.last_result = ""
        self.details = ""

        self.cancel = threading.Event()
        self.shutdown = threading.Event()
        # When the last stop was received. Actions decided before that instant
        # are refused however late they arrive -- a stop invalidates the world
        # view they were built on, and the agent may already have put one on
        # the wire before it heard about the stop.
        self.stop_time_ns = 0
        self.condition = threading.Condition()

        # Wrist grasp planning is a request/response over topics because it takes
        # about 1.6 s: the motion worker blocks on this event while the wrist node
        # runs GraspGenX on the GPU.
        self.grasp_plan_event = threading.Event()
        self.grasp_plan = None
        self.grasp_request_id = ""
        self.grasp_lock = threading.Lock()
        self.tcp_pose_future = None
        self.tcp_pose_future_started = 0.0
        self.tcp_pose_service_warned = False
        self.pending: RobotAction | None = None
        self.busy = False

        # ------------------------------------------------------- hardware

        self.arm = self._build_arm(bounds, profile)
        self.stop_client = self._build_stop_client()
        self.tcp_pose_client = self._build_tcp_pose_client()

        # ----------------------------------------------------------- ROS

        stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.state_publisher = self.create_publisher(
            RobotState, str(self.get_parameter("state_topic").value), latched_qos
        )
        self.grasp_request_publisher = self.create_publisher(
            GraspRequest,
            str(self.get_parameter("grasp_request_topic").value),
            command_qos,
        )
        self.tcp_pose_publisher = self.create_publisher(
            TcpPose,
            str(self.get_parameter("tcp_pose_topic").value),
            command_qos,
        )
        self.create_subscription(
            GraspPlan,
            str(self.get_parameter("grasp_plan_topic").value),
            self.grasp_plan_callback,
            command_qos,
        )
        self.create_subscription(
            SceneSnapshot,
            str(self.get_parameter("scene_topic").value),
            self.scene_callback,
            stream_qos,
        )
        self.create_subscription(
            RobotAction,
            str(self.get_parameter("action_topic").value),
            self.action_callback,
            command_qos,
        )
        # Two publishers, one handler. /vla/estop is the GUI's hardcoded
        # keyword path that never touches the LLM; /vla/robot/stop is the
        # agent's cancel_current_action. They are separate topics so the agent
        # can subscribe to the GUI's stop -- to learn the world changed --
        # without also hearing its own cancels echo back.
        for parameter in ("stop_topic", "estop_topic"):
            self.create_subscription(
                String,
                str(self.get_parameter(parameter).value),
                self.stop_callback,
                command_qos,
            )

        # This timer talks to a client owned by RobotNode; it never calls
        # ``self.arm`` or DSR_ROBOT2.  That distinction is important because
        # the latter spins a separate global node internally and cannot be
        # entered concurrently with the motion worker.
        if self.tcp_pose_client is not None:
            self.create_timer(
                self.tcp_pose_sample_period_s, self.publish_tcp_pose_sample
            )

        self.worker = threading.Thread(
            target=self.run_worker, name="vla-motion-worker", daemon=True
        )
        self.worker.start()
        self.publish_state()

        if self.motion_enabled:
            self.get_logger().warning("ROBOT MOTION ENABLED")
        else:
            self.get_logger().warning(
                "dry-run executor: actions are validated and timed, the arm does not move"
            )

    # ------------------------------------------------------------ hardware

    def _build_arm(self, bounds, profile):
        if not self.motion_enabled:
            return DryRunArm(
                bounds,
                profile,
                self.get_logger(),
                simulated_seconds=float(self.get_parameter("dry_run_motion_s").value),
            )

        from vla_system.robot.gripper import OnRobotRG

        gripper = OnRobotRG(
            str(self.get_parameter("gripper_model").value),
            str(self.get_parameter("gripper_ip").value),
            int(self.get_parameter("gripper_port").value),
        )
        from ament_index_python.packages import get_package_share_directory
        from vla_system.perception.wrist_geometry import load_camera_to_gripper

        calibration_path = str(self.get_parameter("handeye_calibration").value).strip()
        if not calibration_path:
            calibration_path = str(
                Path(get_package_share_directory("vla_system"))
                / "config"
                / "T_gripper2camera.npy"
            )
        camera_to_gripper = load_camera_to_gripper(calibration_path)
        return DoosanArm(
            robot_id=self.robot_id,
            robot_model=str(self.get_parameter("robot_model").value),
            bounds=bounds,
            profile=profile,
            gripper=gripper,
            logger=self.get_logger(),
            camera_offset_gripper_mm=camera_to_gripper[:3, 3],
            camera_axis_gripper=camera_to_gripper[:3, 2],
        )

    def _build_stop_client(self):
        """The hardware interrupt, on this node's executor rather than Doosan's.

        DSR_ROBOT2's helpers all spin ``DR_init.__dsr__node`` internally, so
        calling one from the ROS callback thread while the motion worker is
        mid-move is the documented route to "generator already executing".
        Owning the MoveStop client here means the stop is a plain non-blocking
        ``call_async`` on an executor that is already spinning.
        """
        if not self.motion_enabled:
            return None
        try:
            from dsr_msgs2.srv import MoveStop
        except ImportError as exc:
            self.get_logger().error(
                f"dsr_msgs2 unavailable, hardware stop disabled: {exc}"
            )
            return None
        self.move_stop_type = MoveStop
        return self.create_client(MoveStop, f"/{self.robot_id}/motion/move_stop")

    def _build_tcp_pose_client(self):
        """Create a non-blocking TCP service client on this node's executor.

        The controller does not publish the selected TCP as a stamped topic.
        Calling DSR_ROBOT2's synchronous ``get_current_posx`` from a ROS timer
        would race its private executor against the motion worker, so this node
        owns an ordinary async client instead.  At most one request is in
        flight, and samples are produced only while a wrist target is active.
        """

        if not self.motion_enabled:
            return None
        try:
            from dsr_msgs2.srv import GetCurrentPosx
        except ImportError as exc:
            self.get_logger().error(
                f"dsr_msgs2 unavailable, moving TCP samples disabled: {exc}"
            )
            return None
        self.get_current_posx_type = GetCurrentPosx
        return self.create_client(
            GetCurrentPosx, f"/{self.robot_id}/aux_control/get_current_posx"
        )

    def publish_tcp_pose_sample(self) -> None:
        """Start one async pose read for the currently tracked wrist request."""

        with self.grasp_lock:
            request_id = self.grasp_request_id
        if not request_id or self.tcp_pose_client is None:
            return
        if self.tcp_pose_future is not None:
            age = time.monotonic() - self.tcp_pose_future_started
            if age <= self.tcp_pose_service_timeout_s:
                return
            stale = self.tcp_pose_future
            self.tcp_pose_future = None
            self.tcp_pose_future_started = 0.0
            stale.cancel()
            self.get_logger().warning(
                f"TCP pose service exceeded {self.tcp_pose_service_timeout_s:.2f}s; "
                "discarding the stale request and retrying"
            )
        if not self.tcp_pose_client.service_is_ready():
            if not self.tcp_pose_service_warned:
                self.tcp_pose_service_warned = True
                self.get_logger().warning(
                    "TCP pose service is not ready; wrist tracking is waiting"
                )
            return

        self.tcp_pose_service_warned = False
        request = self.get_current_posx_type.Request()
        request.ref = 0  # DR_BASE
        sent_ros_ns = self.get_clock().now().nanoseconds
        sent_monotonic = time.monotonic()
        try:
            future = self.tcp_pose_client.call_async(request)
        except Exception as exc:
            self.get_logger().warning(f"could not submit TCP pose sample: {exc}")
            return
        self.tcp_pose_future = future
        self.tcp_pose_future_started = sent_monotonic
        future.add_done_callback(
            lambda completed, rid=request_id, sent_ns=sent_ros_ns, sent_mono=sent_monotonic: (
                self._tcp_pose_response(completed, rid, sent_ns, sent_mono)
            )
        )

    def _tcp_pose_response(
        self,
        future,
        request_id: str,
        sent_ros_ns: int,
        sent_monotonic: float,
    ) -> None:
        # A response to a request already timed out (or superseded) must never
        # clear the newer future or enter the pose stream.
        if self.tcp_pose_future is not future:
            return
        self.tcp_pose_future = None
        self.tcp_pose_future_started = 0.0
        if future.cancelled():
            return
        received_monotonic = time.monotonic()
        latency = received_monotonic - sent_monotonic
        if latency > self.tcp_pose_service_timeout_s:
            self.get_logger().warning(
                f"discarded a TCP pose delayed by {latency * 1000:.0f}ms"
            )
            return
        try:
            response = future.result()
            values = list(response.task_pos_info[0].data[:6])
            valid = (
                bool(response.success)
                and len(values) == 6
                and any(float(value) != 0.0 for value in values)
                and all(math.isfinite(float(value)) for value in values)
            )
        except Exception as exc:
            self.get_logger().warning(f"could not sample moving TCP pose: {exc}")
            return

        with self.grasp_lock:
            if request_id != self.grasp_request_id:
                return  # late service result from a completed/cancelled request
        if not valid:
            self.get_logger().warning("controller returned an invalid TCP pose sample")
            return

        # The service has no acquisition stamp.  The controller reads the pose
        # between request and response, so their midpoint is a tighter estimate
        # than pretending the response time itself was the measurement time.
        received_ros_ns = self.get_clock().now().nanoseconds
        midpoint = Time(
            nanoseconds=(int(sent_ros_ns) + int(received_ros_ns)) // 2
        ).to_msg()
        self._publish_tcp_pose(request_id, values, midpoint)

    def _publish_tcp_pose(self, request_id: str, values, stamp=None) -> None:
        message = TcpPose()
        message.header.stamp = stamp or self.get_clock().now().to_msg()
        message.header.frame_id = "base"
        message.request_id = request_id
        message.posx = [float(value) for value in values[:6]]
        message.tcp_name = self.tcp_name
        message.valid = len(message.posx) == 6
        self.tcp_pose_publisher.publish(message)

    # ------------------------------------------------------------ callbacks

    def scene_callback(self, message: SceneSnapshot) -> None:
        with self.scene_lock:
            self.scene = message
            self.scene_monotonic = time.monotonic()

    def action_callback(self, message: RobotAction) -> None:
        name = message.name.strip()
        if name not in KNOWN_ACTIONS:
            self.reject(message, f"알 수 없는 동작입니다: {name}")
            return

        # RobotAction.header.stamp is when the agent read the world, not when
        # it finished thinking. An LLM round-trip is seconds long, so a stop
        # can easily land inside it.
        decided_ns = Time.from_msg(message.header.stamp).nanoseconds
        if decided_ns and decided_ns <= self.stop_time_ns:
            self.reject(message, "정지 이전에 결정된 동작이라 실행하지 않습니다")
            return

        with self.condition:
            if self.busy or self.pending is not None:
                self.reject(message, "이미 다른 동작을 수행하는 중입니다")
                return
            self.cancel.clear()
            self.pending = message
            self.condition.notify()

    def stop_callback(self, message: String) -> None:
        reason = message.data.strip() or "정지"
        self.stop_time_ns = self.get_clock().now().nanoseconds
        with self.condition:
            dropped = self.pending is not None
            self.pending = None
            self.cancel.set()
            was_busy = self.busy
        self.get_logger().warning(f"STOP: {reason}")

        # Fired unconditionally. If the arm is already idle this is a no-op on
        # the controller, and guessing wrong about "already idle" is the one
        # mistake a stop path is not allowed to make.
        if self.stop_client is not None:
            if self.stop_client.service_is_ready():
                request = self.move_stop_type.Request()
                request.stop_mode = self.stop_mode
                self.stop_client.call_async(request)
            else:
                self.get_logger().error(
                    f"/{self.robot_id}/motion/move_stop is not available; "
                    "only the software cancel took effect"
                )

        if not was_busy:
            self.status = "holding" if self.holding else "idle"
            self.last_result = "cancelled" if dropped else self.last_result
            self.details = reason
            self.publish_state()

    def reject(self, action: RobotAction, reason: str) -> None:
        self.get_logger().warning(f"action rejected ({action.name}): {reason}")
        self.last_action_id = action.action_id
        self.last_action = action.name
        self.last_result = "rejected"
        self.details = reason
        self.publish_state()

    # ---------------------------------------------------------------- state

    def publish_state(self) -> None:
        message = RobotState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base"
        message.status = self.status
        message.holding_object_id = self.holding.object_id if self.holding else ""
        message.holding_class_name = self.holding.class_name if self.holding else ""
        message.current_action_id = self.current.action_id if self.current else ""
        message.current_action = self.current.name if self.current else ""
        message.last_action_id = self.last_action_id
        message.last_action = self.last_action
        message.last_result = self.last_result
        message.details = self.details
        message.motion_enabled = self.motion_enabled
        self.state_publisher.publish(message)

    def resolve(self, object_id: str):
        """Read the live position, not the one the LLM saw a round-trip ago."""
        object_id = object_id.strip()
        if not object_id:
            raise MotionError("동작에 대상 물체가 지정되지 않았습니다")
        with self.scene_lock:
            scene = self.scene
            age = time.monotonic() - self.scene_monotonic
        if scene is None:
            raise MotionError("아직 카메라 장면을 받지 못했습니다")
        if age > self.max_scene_age_s:
            raise MotionError(f"카메라 장면이 {age:.1f}초 전 것이라 사용할 수 없습니다")
        for scene_object in scene.objects:
            if scene_object.id != object_id:
                continue
            if not scene_object.position_valid:
                raise MotionError(f"{object_id}의 3D 위치를 아직 확정하지 못했습니다")
            position = scene_object.position_base
            return (position.x, position.y, position.z), scene_object.class_name
        raise MotionError(f"{object_id}가 지금 화면에 보이지 않습니다")

    # --------------------------------------------------------------- worker

    def run_worker(self) -> None:
        while not self.shutdown.is_set():
            with self.condition:
                while self.pending is None and not self.shutdown.is_set():
                    self.condition.wait(timeout=0.2)
                if self.shutdown.is_set():
                    return
                action = self.pending
                self.pending = None
                self.busy = True
                self.current = action
            try:
                self.execute(action)
            finally:
                with self.condition:
                    self.busy = False
                    self.current = None

    def execute(self, action: RobotAction) -> None:
        self.status = "moving"
        self.last_action_id = action.action_id
        self.last_action = action.name
        self.last_result = ""
        self.details = action.reason
        self.publish_state()

        try:
            self.run_action(action)
            result, details = "succeeded", ""
        except MotionCancelled as exc:
            result, details = "cancelled", str(exc)
        except MotionError as exc:
            result, details = "failed", str(exc)
            self.get_logger().warning(f"action {action.name} failed: {exc}")
        except Exception as exc:  # hardware boundary
            result, details = "failed", f"{type(exc).__name__}: {exc}"
            self.get_logger().error(f"action {action.name} raised: {exc}")

        # Cleared *before* the final publish, not in run_worker's finally after
        # it. A RobotState that still names a current_action reads as "still
        # running" to the agent, which filters it out -- and that filter is the
        # only thing that turns a finished action into the next decision point.
        # Leaving it set here silently disables every follow-on decision: the
        # arm picks one apple and then waits forever.
        with self.condition:
            self.current = None

        # All four fields are re-asserted together, not just the result. A
        # rejection arriving while this action was in flight will have
        # overwritten last_action_id/last_action with the *rejected* action's
        # identity; leaving that in place would publish "pick_and_place
        # succeeded" for an action that was actually refused, while the one
        # that really succeeded went unreported.
        self.status = "holding" if self.holding else "idle"
        self.last_action_id = action.action_id
        self.last_action = action.name
        self.last_result = result
        self.details = details
        self.publish_state()
        self.get_logger().info(
            f"action {action.name}({action.object_id}) -> {result} {details}".strip()
        )

    # ------------------------------------------------------- wrist grasping

    def grasp_plan_callback(self, message: GraspPlan) -> None:
        with self.grasp_lock:
            if message.request_id != self.grasp_request_id:
                return  # a reply to a request we already gave up on
            self.grasp_plan = message
        self.grasp_plan_event.set()

    def begin_wrist_grasp(self, action: RobotAction, position, class_name):
        """Register a target before the observation move starts.

        This method runs on the motion worker.  Its one synchronous TCP read is
        safe here and gives the wrist an immediate pose sample; subsequent
        moving samples come from RobotNode's independent async ROS client.
        """

        pose_sent_ns = self.get_clock().now().nanoseconds
        try:
            tcp_posx = [float(v) for v in self.arm.get_current_posx()[0][:6]]
        except Exception as exc:
            self.get_logger().warning(f"could not read the TCP pose: {exc}")
            return None
        pose_received_ns = self.get_clock().now().nanoseconds
        if (
            len(tcp_posx) != 6
            or not any(value != 0.0 for value in tcp_posx)
            or not all(math.isfinite(value) for value in tcp_posx)
        ):
            self.get_logger().warning("controller returned an invalid initial TCP pose")
            return None

        request = GraspRequest()
        pose_stamp = Time(
            nanoseconds=(pose_sent_ns + pose_received_ns) // 2
        ).to_msg()
        request.header.stamp = pose_stamp
        request.header.frame_id = "base"
        request.request_id = f"{action.action_id}:{pose_received_ns}"
        request.object_id = action.object_id
        request.class_name = class_name
        request.expected_base.x = float(position[0])
        request.expected_base.y = float(position[1])
        request.expected_base.z = float(position[2])
        request.tcp_posx = tcp_posx
        request.tcp_name = self.tcp_name
        request.cancel = False

        with self.grasp_lock:
            self.grasp_request_id = request.request_id
            self.grasp_plan = None
        self.grasp_plan_event.clear()
        try:
            self.grasp_request_publisher.publish(request)
        except Exception as exc:
            with self.grasp_lock:
                if self.grasp_request_id == request.request_id:
                    self.grasp_request_id = ""
            self.get_logger().warning(f"could not register wrist tracking: {exc}")
            return None
        try:
            self._publish_tcp_pose(request.request_id, tcp_posx, pose_stamp)
        except Exception as exc:
            # The periodic direct service stream remains available, so losing
            # only this optional first sample is not terminal.
            self.get_logger().warning(f"could not publish initial TCP pose: {exc}")
        self.get_logger().info(
            f"wrist tracking started for {action.object_id} ({class_name})"
        )
        return request.request_id

    def wait_wrist_grasp(self, request_id: str):
        """Wait for the continuously tracking wrist node's final plan."""

        # Poll rather than block outright so a stop during the roughly 1.6 s of
        # inference still lands inside poll_interval_s, matching the guarantee
        # every other motion in this file gives.
        deadline = time.monotonic() + self.wrist_grasp_timeout_s
        while time.monotonic() < deadline:
            if self.cancel.is_set():
                raise MotionCancelled("cancelled while planning the grasp")
            if self.grasp_plan_event.wait(self.arm.profile.poll_interval_s):
                break
        with self.grasp_lock:
            plan = self.grasp_plan if self.grasp_request_id == request_id else None
        if plan is None:
            self.get_logger().warning(
                f"손목 추적에서 {self.wrist_grasp_timeout_s:.0f}초 안에 안정된 "
                "YOLO-seg/좌표 일치가 확정되지 않았습니다. 요청을 취소합니다."
            )
            return None
        if not plan.success:
            self.get_logger().warning(f"wrist could not plan a grasp: {plan.reason}")
            return None
        return plan

    def cancel_wrist_grasp(self, request_id: str) -> None:
        """End one correlated tracking session and reject its late replies."""

        with self.grasp_lock:
            if self.grasp_request_id != request_id:
                return
            self.grasp_request_id = ""
            self.grasp_plan = None
        request = GraspRequest()
        request.header.stamp = self.get_clock().now().to_msg()
        request.request_id = request_id
        request.cancel = True
        try:
            self.grasp_request_publisher.publish(request)
        except Exception as exc:
            # Cleanup must not hide the MotionCancelled/MotionError that led
            # here; correlation is already cleared locally, so late plans are
            # still harmless.
            self.get_logger().warning(f"could not publish wrist cancellation: {exc}")

    def wrist_planner_available(self) -> bool:
        """Whether asking the wrist for a grasp can possibly work.

        Checked before the arm moves anywhere. Both of these used to be
        discovered only by waiting out the full timeout, which cost eight silent
        seconds on every grasp and left the log saying "timed out" when the real
        answer was "there is nothing to time out against".
        """

        # Warning, not info: the GUI only forwards warnings and errors to the
        # chat, and "why did it fall back" is exactly what the operator needs to
        # see there. An info-level line left the GUI showing a bare "falling
        # back" with no reason.
        if self.grasp_request_publisher.get_subscription_count() == 0:
            self.get_logger().warning(
                "손목 파지 노드(vla_wrist)가 없어 webcam 좌표로 집습니다. "
                "GUI의 '손목 파지 (GraspGenX)'를 체크하고 다시 시작하거나, "
                "use_wrist_grasp:=false로 두면 더 묻지 않습니다."
            )
            return False
        # The planner needs the arm's own pose to put its grasp in the base
        # frame, and a dry run has no controller to ask.
        if getattr(self.arm, "get_current_posx", None) is None:
            self.get_logger().warning(
                "dry run has no TCP pose, so the wrist camera cannot place its "
                "grasp in the base frame; using the webcam position. Run with "
                "motion_enabled:=true to exercise the wrist path."
            )
            return False
        if self.tcp_pose_client is None or not self.tcp_pose_client.service_is_ready():
            self.get_logger().warning(
                "로봇 TCP pose service가 없어 이동 중 손목 검출 좌표를 base로 "
                "변환할 수 없습니다; webcam 좌표로 집습니다."
            )
            return False
        return True

    def wrist_guided_pick(self, action: RobotAction, position, class_name, on_grasped):
        """Approach on the webcam position, then grasp on the wrist's plan.

        Returns True if the wrist path completed the grasp.
        """

        if not self.wrist_planner_available():
            return False

        decided_ns = self.get_clock().now().nanoseconds
        request_id = self.begin_wrist_grasp(action, position, class_name)
        if not request_id:
            return False
        try:
            # The wrist node is already armed here.  It runs YOLO-seg on every
            # timestamp-compatible RGB-D/TCP sample while this move progresses.
            self.arm.observe(position, self.cancel)
            plan = self.wait_wrist_grasp(request_id)
        finally:
            self.cancel_wrist_grasp(request_id)
        if plan is None:
            return False

        # The stop re-check that makes this safe. Planning takes seconds, so a
        # "정지" can arrive while the GPU is busy; without this the arm would
        # execute a grasp decided before the world was invalidated. Same reason
        # the agent re-checks its stop epoch before publishing an action.
        if self.cancel.is_set() or self.stop_time_ns > decided_ns:
            raise MotionCancelled("stopped while the grasp was being planned")

        self.get_logger().info(
            f"executing wrist grasp: confidence {plan.confidence:.3f}, "
            f"tilt {plan.approach_tilt_deg:.0f}°, {plan.cloud_points} points"
        )
        self.arm.pick_at_posx(
            list(plan.target_posx),
            list(plan.pregrasp_posx),
            self.cancel,
            on_grasped=on_grasped,
        )
        return True

    def run_action(self, action: RobotAction) -> None:
        name = action.name.strip()

        if name in PICK_ACTIONS:
            if self.holding is not None:
                raise MotionError(
                    f"이미 {self.holding.class_name}을(를) 들고 있어 새로 집을 수 없습니다"
                )
            position, class_name = self.resolve(action.object_id)

            def on_grasped() -> None:
                self.holding = Holding(action.object_id, class_name)
                self.status = "moving"
                self.publish_state()

            grasped = False
            if self.use_wrist_grasp:
                grasped = self.wrist_guided_pick(
                    action, position, class_name, on_grasped
                )
                if not grasped and not self.wrist_grasp_fallback:
                    raise MotionError(
                        "손목 카메라가 파지 자세를 만들지 못했습니다"
                    )
                if not grasped:
                    self.get_logger().warning(
                        "falling back to the webcam position for this grasp"
                    )
            if not grasped:
                # Observation plus planning can take many seconds. Never descend
                # on the scene coordinate captured before that delay; require a
                # fresh snapshot of the same object ID immediately before the
                # fallback motion.
                position, refreshed_class = self.resolve(action.object_id)
                if refreshed_class != class_name:
                    raise MotionError(
                        f"fallback target class changed from {class_name} "
                        f"to {refreshed_class}"
                    )
                self.arm.pick(position, self.cancel, on_grasped=on_grasped)
            if name == "pick_and_place":
                self.arm.place(self.cancel)
                self.holding = None
            return

        if name == "release":
            if self.holding is None:
                raise MotionError("지금 들고 있는 물체가 없습니다")
            self.arm.release(self.cancel)
            self.holding = None
            return

        if name == "go_home":
            if self.holding is not None:
                raise MotionError("물체를 든 채로 홈으로 복귀하지 않습니다")
            self.arm.go_home(self.cancel)
            return

        raise MotionError(f"알 수 없는 동작입니다: {name}")

    def close(self) -> None:
        self.shutdown.set()
        self.cancel.set()
        with self.condition:
            self.condition.notify_all()
        self.worker.join(timeout=3.0)
        try:
            self.arm.close()
        except Exception as exc:
            self.get_logger().error(f"arm shutdown failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    # A bare `rclpy.spin(node)` would add this node to rclpy's process-wide
    # *global* executor. With motion enabled, the worker thread's Doosan calls
    # internally do `rclpy.spin_until_future_complete(g_node, future)` with no
    # explicit executor -- so they default to that same global executor. Two
    # threads then drive one SingleThreadedExecutor's callback generator at
    # once, which raises "generator already executing" the moment an action
    # runs. Spinning on an executor we own keeps the global one free for
    # Doosan, so the two loops never collide.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
