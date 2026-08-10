#!/usr/bin/env python3
"""Continuous wrist tracking: RealSense -> YOLO-seg -> GraspGenX -> posx.

The fixed webcam says *which* object and roughly where.  Before the arm starts
its observation move, this node receives that target and continuously checks
the moving wrist view.  RGB, aligned depth and the selected robot TCP are paired
by timestamp.  Only a YOLO-seg instance whose base-frame coordinate agrees with
the webcam target for several frames is handed to GraspGenX.

Why it is a separate node from vla_perception
---------------------------------------------
Two cameras, two jobs, two rates. The webcam runs at 15 Hz because the scene has
to stay current for the agent. GraspGenX takes about 1.6 s per object, so it can
only ever run on request. Bolting that onto the scene loop would stall the scene.

Why it does not own the robot connection
----------------------------------------
`vla_robot` is the only node allowed to touch the Doosan driver -- two nodes
calling `DR_init` fight over one connection. So the TCP pose arrives on a topic
instead, and this node completes the hand-eye chain itself:

    p_base = T_gripper->base(TcpPose) @ T_camera->gripper(npy) @ p_camera

The controller does not publish the selected TCP as TF, so ``vla_robot`` emits
request-correlated ``TcpPose`` samples while tracking is active.  Using an
independent ``latest_*`` cache here would mix a moving camera frame with the
wrong robot pose, which is why every input goes through the timestamp matcher.
"""

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from time import monotonic, perf_counter
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from vla_interfaces.msg import GraspPlan, GraspRequest, TcpPose

from vla_system.grasp.poses import (
    GraspGeometryError,
    approach_tilt_degrees,
    build_candidates,
    posx_from_pose,
    pregrasp_pose,
    scale_pose_translation,
    select_grasp,
    transform_pose,
)
from vla_system.perception.detector import (
    YoloDetector,
    bgr_to_image_message,
    draw_tracks,
    image_message_to_bgr,
    object_id,
    result_to_detections,
)
from vla_system.perception.tracker import TrackedDetection
from vla_system.perception.object_matching import WristCandidate, match_target
from vla_system.perception.wrist_tracking import (
    MatchStability,
    TimedSample,
    result_matches_session,
    select_synced_observation,
    stamp_ns,
)
from vla_system.perception.wrist_geometry import (
    MM_PER_M,
    CameraIntrinsics,
    WristCalibrationError,
    base_point_m,
    camera_to_base_mm,
    load_camera_to_gripper,
    mask_to_camera_cloud,
    pose_matrix_from_posx,
)


def decode_depth(message: Image, integer_scale_mm: float):
    """Return (2-D array, scale that converts a sample to millimetres)."""

    if message.encoding in ("16UC1", "mono16"):
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        per_row = message.step // dtype.itemsize
        image = np.frombuffer(message.data, dtype=dtype).reshape(
            message.height, per_row
        )[:, : message.width]
        return image, integer_scale_mm
    if message.encoding == "32FC1":
        dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
        per_row = message.step // dtype.itemsize
        image = np.frombuffer(message.data, dtype=dtype).reshape(
            message.height, per_row
        )[:, : message.width]
        return image, MM_PER_M  # metres -> millimetres
    raise ValueError(f"unsupported depth encoding: {message.encoding}")


@dataclass(frozen=True)
class GraspComputation:
    """ROS-free result returned by the one-worker GraspGenX executor."""

    success: bool
    reason: str = ""
    target_posx: tuple = ()
    pregrasp_posx: tuple = ()
    confidence: float = 0.0
    approach_tilt_deg: float = 0.0
    candidate_count: int = 0
    cloud_points: int = 0


class WristGraspNode(Node):
    def __init__(self):
        super().__init__("vla_wrist")

        default_calibration = str(
            Path(get_package_share_directory("vla_system"))
            / "config"
            / "T_gripper2camera.npy"
        )

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/aligned_depth_to_color/camera_info"
        )
        self.declare_parameter("grasp_request_topic", "/vla/grasp/request")
        self.declare_parameter("grasp_plan_topic", "/vla/grasp/plan")
        self.declare_parameter("tcp_pose_topic", "/vla/robot/tcp_pose")
        self.declare_parameter("annotated_topic", "/vla/wrist/annotated_image")
        # A low-rate preview of what the wrist detector sees. YOLO is ~6ms, so
        # this is cheap; GraspGenX is the expensive part and still runs only on
        # request. Without it "물체가 보이지 않습니다" gives no way to tell an
        # empty frame from a detector that is looking at the right thing and
        # simply not firing.
        self.declare_parameter("annotate_rate_hz", 5.0)
        self.declare_parameter("tracking_rate_hz", 10.0)
        self.declare_parameter("tracking_timeout_s", 45.0)
        self.declare_parameter("frame_buffer_size", 12)
        self.declare_parameter("rgbd_sync_tolerance_s", 0.005)
        self.declare_parameter("tcp_sync_tolerance_s", 0.15)
        self.declare_parameter("stable_match_frames", 3)
        self.declare_parameter("stable_match_max_jump_m", 0.02)
        self.declare_parameter("stable_match_max_gap_s", 0.35)
        self.declare_parameter("tracking_report_interval_s", 1.0)

        self.declare_parameter("handeye_calibration", default_calibration)
        self.declare_parameter("expected_tcp_name", "GripperDA_v1")
        self.declare_parameter("integer_depth_scale_mm", 1.0)

        self.declare_parameter("backend", "pytorch")
        self.declare_parameter("model", "")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("max_detections", 30)
        self.declare_parameter(
            "target_classes", [], ParameterDescriptor(dynamic_typing=True)
        )
        self.declare_parameter("excluded_classes", ["person"])
        self.declare_parameter("torch_threads", 4)

        self.declare_parameter("match_tolerance_m", 0.06)
        self.declare_parameter("match_max_vertical_gap_m", 0.30)
        self.declare_parameter("refuse_ambiguous_match", True)

        self.declare_parameter("graspgen_repo", "")
        self.declare_parameter("gripper_name", "onrobot_RG2")
        self.declare_parameter("num_grasps", 200)
        self.declare_parameter("topk_num_grasps", 20)
        self.declare_parameter("min_grasp_confidence", 0.3)
        self.declare_parameter("max_approach_tilt_deg", 45.0)
        self.declare_parameter("standoff_m", 0.04)
        self.declare_parameter("min_cloud_points", 200)
        self.declare_parameter("max_cloud_points", 20000)
        self.declare_parameter("min_depth_mm", 100.0)
        self.declare_parameter("max_depth_mm", 1500.0)
        # Workspace bounds are duplicated from vla_robot deliberately: a plan
        # that leaves the safe box should never be published, and the executor
        # checks again before moving.
        self.declare_parameter("workspace_min_x_m", 0.2)
        self.declare_parameter("workspace_max_x_m", 0.9)
        self.declare_parameter("workspace_min_y_m", -0.6)
        self.declare_parameter("workspace_max_y_m", 0.6)
        self.declare_parameter("workspace_min_z_m", 0.02)
        self.declare_parameter("workspace_max_z_m", 0.8)

        self.expected_tcp_name = str(self.get_parameter("expected_tcp_name").value)
        self.integer_depth_scale_mm = float(
            self.get_parameter("integer_depth_scale_mm").value
        )
        self.match_tolerance_m = float(self.get_parameter("match_tolerance_m").value)
        self.match_max_vertical_gap_m = float(
            self.get_parameter("match_max_vertical_gap_m").value
        )
        self.refuse_ambiguous_match = bool(
            self.get_parameter("refuse_ambiguous_match").value
        )
        tracking_rate_hz = float(self.get_parameter("tracking_rate_hz").value)
        self.tracking_timeout_s = float(
            self.get_parameter("tracking_timeout_s").value
        )
        buffer_size = int(self.get_parameter("frame_buffer_size").value)
        if tracking_rate_hz <= 0.0 or self.tracking_timeout_s <= 0.0:
            raise ValueError("tracking rates and timeout must be positive")
        if buffer_size < 2:
            raise ValueError("frame_buffer_size must be at least two")
        self.tracking_period_s = 1.0 / tracking_rate_hz
        self.rgbd_tolerance_ns = int(
            float(self.get_parameter("rgbd_sync_tolerance_s").value) * 1e9
        )
        self.pose_tolerance_ns = int(
            float(self.get_parameter("tcp_sync_tolerance_s").value) * 1e9
        )
        self.tracking_report_interval_s = float(
            self.get_parameter("tracking_report_interval_s").value
        )
        if self.rgbd_tolerance_ns < 0 or self.pose_tolerance_ns < 0:
            raise ValueError("timestamp tolerances must be non-negative")
        if self.tracking_report_interval_s < 0.0:
            raise ValueError("tracking_report_interval_s must be non-negative")
        self.min_grasp_confidence = float(
            self.get_parameter("min_grasp_confidence").value
        )
        self.max_approach_tilt_deg = float(
            self.get_parameter("max_approach_tilt_deg").value
        )
        self.standoff_m = float(self.get_parameter("standoff_m").value)
        self.min_cloud_points = int(self.get_parameter("min_cloud_points").value)
        self.max_cloud_points = int(self.get_parameter("max_cloud_points").value)
        self.min_depth_mm = float(self.get_parameter("min_depth_mm").value)
        self.max_depth_mm = float(self.get_parameter("max_depth_mm").value)
        self.target_classes = {
            str(name) for name in (self.get_parameter("target_classes").value or [])
        }
        self.excluded_classes = {
            str(name) for name in self.get_parameter("excluded_classes").value
        }
        self.workspace = {
            "x": (
                float(self.get_parameter("workspace_min_x_m").value),
                float(self.get_parameter("workspace_max_x_m").value),
            ),
            "y": (
                float(self.get_parameter("workspace_min_y_m").value),
                float(self.get_parameter("workspace_max_y_m").value),
            ),
            "z": (
                float(self.get_parameter("workspace_min_z_m").value),
                float(self.get_parameter("workspace_max_z_m").value),
            ),
        }

        # An empty parameter means "the one shipped with the package". Without
        # this, the empty string in system.yaml became Path("") -> "." and
        # np.load tried to read the current directory.
        calibration_path = (
            str(self.get_parameter("handeye_calibration").value).strip()
            or default_calibration
        )
        self.camera_to_gripper = load_camera_to_gripper(calibration_path)
        self.get_logger().info(
            f"hand-eye calibration loaded from {calibration_path} "
            f"(camera -> gripper, mm); expects TCP "
            f"{self.expected_tcp_name!r} to be active"
        )

        self.detector = YoloDetector(
            backend=str(self.get_parameter("backend").value),
            model_override=str(self.get_parameter("model").value),
            device=str(self.get_parameter("device").value),
            imgsz=int(self.get_parameter("imgsz").value),
            confidence=float(self.get_parameter("confidence").value),
            max_detections=int(self.get_parameter("max_detections").value),
            torch_threads=int(self.get_parameter("torch_threads").value),
        )
        self.get_logger().info(f"wrist YOLO ready: {self.detector.model_path}")

        self.grasp_client = self._build_grasp_client()

        self.latest_color: Optional[Image] = None
        self.latest_depth: Optional[Image] = None
        self.intrinsics: Optional[CameraIntrinsics] = None
        self.tcp_name_warned = False
        self.durations = deque(maxlen=50)
        self.color_samples = deque(maxlen=buffer_size)
        self.depth_samples = deque(maxlen=buffer_size)
        self.pose_samples = deque(maxlen=buffer_size)
        self.closed_request_ids = deque(maxlen=64)
        self.pending_request: Optional[GraspRequest] = None
        self.request_after_ns = 0
        self.last_processed_color_ns = 0
        self.next_tracking_time = 0.0
        self.tracking_deadline = 0.0
        self.last_tracking_reason = "새 RGB-D/TCP 프레임을 기다리는 중입니다"
        self.last_tracking_report = 0.0
        self.tracking_busy = False
        self.session_generation = 0
        self.grasp_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="vla-graspgenx"
        )
        self.grasp_future = None
        self.planning_request: Optional[GraspRequest] = None
        self.planning_generation = 0
        self.planning_started = 0.0
        self.planning_source_stamp_ns = 0
        self.stability = MatchStability(
            required_frames=int(self.get_parameter("stable_match_frames").value),
            max_jump_m=float(
                self.get_parameter("stable_match_max_jump_m").value
            ),
            max_gap_ns=int(
                float(self.get_parameter("stable_match_max_gap_s").value) * 1e9
            ),
        )

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

        self.plan_publisher = self.create_publisher(
            GraspPlan, str(self.get_parameter("grasp_plan_topic").value), command_qos
        )
        self.annotated_publisher = self.create_publisher(
            Image, str(self.get_parameter("annotated_topic").value), stream_qos
        )
        self.create_subscription(
            Image, str(self.get_parameter("color_topic").value),
            self._color_callback, stream_qos,
        )
        self.create_subscription(
            Image, str(self.get_parameter("depth_topic").value),
            self._depth_callback, stream_qos,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback, stream_qos,
        )
        self.create_subscription(
            GraspRequest, str(self.get_parameter("grasp_request_topic").value),
            self._request_callback, command_qos,
        )
        self.create_subscription(
            TcpPose, str(self.get_parameter("tcp_pose_topic").value),
            self._tcp_pose_callback, command_qos,
        )
        annotate_rate = float(self.get_parameter("annotate_rate_hz").value)
        if annotate_rate > 0.0:
            self.create_timer(1.0 / annotate_rate, self.publish_annotated)
        # Selection is also attempted from each input callback.  This timer is
        # the timeout/fallback path and processes the newest complete triplet if
        # no callback happened exactly when the rate limiter opened.
        self.create_timer(min(0.05, self.tracking_period_s), self._tracking_tick)
        self.get_logger().info("wrist grasp planner ready")

    # ------------------------------------------------------------------ setup

    def _build_grasp_client(self):
        from vla_system.grasp.graspgen_client import (
            GraspGenClient,
            GraspGenUnavailable,
        )

        repo = str(self.get_parameter("graspgen_repo").value).strip() or None
        try:
            return GraspGenClient(
                gripper_name=str(self.get_parameter("gripper_name").value),
                repo_path=repo,
                num_grasps=int(self.get_parameter("num_grasps").value),
                topk_num_grasps=int(self.get_parameter("topk_num_grasps").value),
                logger=self.get_logger(),
            )
        except GraspGenUnavailable as exc:
            # Not fatal: every request then fails with a clear reason and the
            # executor falls back to the webcam-only grasp, which is better than
            # refusing to start the pipeline at all.
            self.get_logger().error(
                f"GraspGenX unavailable ({exc}); grasp planning will be refused"
            )
            return None

    # -------------------------------------------------------------- callbacks

    def _color_callback(self, message: Image) -> None:
        self.latest_color = message
        self.color_samples.append(TimedSample(stamp_ns(message), message))
        self._try_track()

    def _depth_callback(self, message: Image) -> None:
        self.latest_depth = message
        self.depth_samples.append(TimedSample(stamp_ns(message), message))
        self._try_track()

    def _tcp_pose_callback(self, message: TcpPose) -> None:
        request = self.pending_request
        if (
            request is None
            or message.request_id != request.request_id
            or not message.valid
        ):
            return
        pose = tuple(float(value) for value in message.posx)
        if (
            len(pose) != 6
            or not any(pose)
            or not all(np.isfinite(value) for value in pose)
        ):
            return
        self._check_tcp_name(message.tcp_name)
        if message.tcp_name != self.expected_tcp_name:
            self.last_tracking_reason = (
                f"TCP {message.tcp_name!r}가 hand-eye 기준 "
                f"{self.expected_tcp_name!r}와 다릅니다"
            )
            return
        if message.header.frame_id and message.header.frame_id != "base":
            self.last_tracking_reason = (
                f"TCP pose frame {message.header.frame_id!r}는 base가 아닙니다"
            )
            return
        self.pose_samples.append(TimedSample(stamp_ns(message), pose))
        self._try_track()

    def _camera_info_callback(self, message: CameraInfo) -> None:
        try:
            self.intrinsics = CameraIntrinsics.from_camera_info(message)
        except WristCalibrationError as exc:
            self.get_logger().warning(f"unusable wrist intrinsics: {exc}")

    def publish_annotated(self) -> None:
        """Draw what the wrist detector sees, so it can be watched in the GUI."""

        # Active tracking already publishes the exact synchronized frame it
        # used.  Running this preview detector too would duplicate GPU work and
        # show the operator a different image than the matching decision.
        if (
            self.pending_request is not None
            or self.tracking_busy
            or self.latest_color is None
        ):
            return
        if self.annotated_publisher.get_subscription_count() == 0:
            return
        try:
            frame = image_message_to_bgr(self.latest_color)
            detections = result_to_detections(
                self.detector.predict(frame),
                self.target_classes,
                self.excluded_classes,
            )
        except (ValueError, TypeError, RuntimeError):
            return
        self._publish_detections(frame, self.latest_color.header, detections)

    def _publish_detections(self, frame, header, detections) -> None:
        if self.annotated_publisher.get_subscription_count() == 0:
            return
        tracked = [
            TrackedDetection(
                track_id=index,
                class_id=detection.class_id,
                class_name=detection.class_name,
                confidence=detection.confidence,
                bbox=detection.bbox,
                velocity_x=0.0,
                velocity_y=0.0,
            )
            for index, detection in enumerate(detections)
        ]
        labels = {t.track_id: object_id(t.class_name, t.track_id) for t in tracked}
        self.annotated_publisher.publish(
            bgr_to_image_message(
                draw_tracks(frame, tracked, labels), header
            )
        )

    def _check_tcp_name(self, reported: str) -> None:
        """The calibration is only valid for the TCP it was measured with."""
        if reported and reported != self.expected_tcp_name and not self.tcp_name_warned:
            self.tcp_name_warned = True
            self.get_logger().error(
                f"robot reports TCP {reported!r} but the hand-eye calibration "
                f"was measured with {self.expected_tcp_name!r}. Every mapped "
                "coordinate will be in the wrong frame."
            )

    def inside_workspace(self, point_m) -> bool:
        for axis, value in zip("xyz", point_m):
            low, high = self.workspace[axis]
            if not low <= float(value) <= high:
                return False
        return True

    # ----------------------------------------------------------------- planning

    def _refuse(self, request: GraspRequest, reason: str) -> None:
        self.get_logger().warning(f"grasp refused for {request.object_id}: {reason}")
        plan = GraspPlan()
        plan.header.stamp = self.get_clock().now().to_msg()
        plan.request_id = request.request_id
        plan.object_id = request.object_id
        plan.success = False
        plan.reason = reason
        self.plan_publisher.publish(plan)

    def _request_callback(self, request: GraspRequest) -> None:
        if request.cancel:
            if (
                self.pending_request is not None
                and request.request_id == self.pending_request.request_id
            ):
                self.get_logger().info(
                    f"wrist tracking cancelled for {request.request_id}"
                )
                self._clear_tracking()
            return

        if request.request_id in self.closed_request_ids:
            return  # terminal requests are idempotent; never plan them twice

        if self.pending_request is not None:
            if request.request_id == self.pending_request.request_id:
                return  # idempotent duplicate registration
            return self._refuse(
                request,
                f"다른 객체({self.pending_request.object_id})를 이미 추적 중입니다",
            )
        if self.grasp_client is None:
            return self._refuse(request, "GraspGenX를 사용할 수 없습니다")

        tcp_pose = list(request.tcp_posx)
        self._check_tcp_name(request.tcp_name)
        if request.tcp_name != self.expected_tcp_name:
            return self._refuse(
                request,
                f"요청 TCP {request.tcp_name!r}가 hand-eye 기준 "
                f"{self.expected_tcp_name!r}와 다릅니다",
            )
        if request.header.frame_id and request.header.frame_id != "base":
            return self._refuse(
                request,
                f"요청 pose frame {request.header.frame_id!r}는 base가 아닙니다",
            )
        self.session_generation += 1
        cutoff_ns = stamp_ns(request)
        if cutoff_ns <= 0:
            cutoff_ns = self.get_clock().now().nanoseconds

        self.pending_request = request
        self.request_after_ns = cutoff_ns
        self.last_processed_color_ns = cutoff_ns
        self.next_tracking_time = 0.0
        self.tracking_deadline = monotonic() + self.tracking_timeout_s
        self.last_tracking_reason = "새 RGB-D/TCP 프레임을 기다리는 중입니다"
        self.last_tracking_report = 0.0
        self.stability.reset()
        self.pose_samples.clear()
        if (
            len(tcp_pose) == 6
            and any(tcp_pose)
            and all(np.isfinite(value) for value in tcp_pose)
        ):
            self.pose_samples.append(TimedSample(cutoff_ns, tuple(tcp_pose)))

        expected = request.expected_base
        self.get_logger().info(
            f"tracking {request.object_id} ({request.class_name}) while approaching; "
            f"webcam base=({expected.x:.3f}, {expected.y:.3f}, {expected.z:.3f})m"
        )
        self._try_track()

    def _clear_tracking(self) -> None:
        self.session_generation += 1  # invalidates a worker result already in flight
        if self.pending_request is not None:
            self.closed_request_ids.append(self.pending_request.request_id)
        self.pending_request = None
        self.request_after_ns = 0
        self.last_processed_color_ns = 0
        self.tracking_deadline = 0.0
        self.next_tracking_time = 0.0
        self.pose_samples.clear()
        self.stability.reset()

    def _tracking_tick(self) -> None:
        self._poll_grasp_future()
        request = self.pending_request
        if request is None:
            return
        if monotonic() >= self.tracking_deadline:
            reason = self.last_tracking_reason
            self._clear_tracking()
            self._refuse(
                request,
                f"{self.tracking_timeout_s:.0f}초 동안 목표를 확정하지 못했습니다: "
                f"{reason}",
            )
            return
        if self.tracking_busy:
            return
        self._try_track()

    def _tracking_status(self, reason: str, *, reset_stability: bool = True) -> None:
        if reset_stability:
            self.stability.reset()
        self.last_tracking_reason = reason
        now = monotonic()
        if now - self.last_tracking_report >= self.tracking_report_interval_s:
            self.last_tracking_report = now
            request = self.pending_request
            if request is not None:
                self.get_logger().info(
                    f"tracking {request.object_id}: {reason}"
                )

    def _try_track(self) -> None:
        request = self.pending_request
        if request is None or self.tracking_busy or self.intrinsics is None:
            if request is not None and self.intrinsics is None:
                self.last_tracking_reason = "손목 카메라 내부 파라미터가 없습니다"
            return
        now = monotonic()
        if now >= self.tracking_deadline:
            return
        if now < self.next_tracking_time:
            return

        observation = select_synced_observation(
            self.color_samples,
            self.depth_samples,
            self.pose_samples,
            after_ns=self.request_after_ns,
            last_color_ns=self.last_processed_color_ns,
            rgbd_tolerance_ns=self.rgbd_tolerance_ns,
            pose_tolerance_ns=self.pose_tolerance_ns,
        )
        if observation is None:
            missing = []
            if not self.color_samples:
                missing.append("RGB")
            if not self.depth_samples:
                missing.append("depth")
            if not self.pose_samples:
                missing.append("TCP pose")
            if missing:
                self._tracking_status(
                    f"입력 대기 중: {', '.join(missing)}",
                    reset_stability=False,
                )
            else:
                self._tracking_status(
                    "허용 오차 안의 새 RGB-depth-TCP 타임스탬프 조합이 없습니다",
                    reset_stability=False,
                )
            return

        self.last_processed_color_ns = observation.color.stamp_ns
        self.next_tracking_time = now + self.tracking_period_s
        try:
            frame = image_message_to_bgr(observation.color.value)
            depth_image, depth_scale_mm = decode_depth(
                observation.depth.value, self.integer_depth_scale_mm
            )
            detections = result_to_detections(
                self.detector.predict(frame),
                self.target_classes,
                self.excluded_classes,
            )
        except (ValueError, TypeError, RuntimeError) as exc:
            self._tracking_status(f"영상/YOLO 처리 실패: {exc}")
            return

        self._publish_detections(frame, observation.color.value.header, detections)
        if not detections:
            self._tracking_status("YOLO-seg 검출이 없습니다")
            return

        tcp_pose = list(observation.pose.value)
        candidates = []
        clouds = {}
        try:
            for index, detection in enumerate(detections):
                mask = getattr(detection, "mask", None)
                if mask is None or mask.shape != depth_image.shape:
                    continue
                cloud = mask_to_camera_cloud(
                    depth_image,
                    mask,
                    self.intrinsics,
                    depth_scale_mm=depth_scale_mm,
                    min_depth_mm=self.min_depth_mm,
                    max_depth_mm=self.max_depth_mm,
                    max_points=self.max_cloud_points,
                )
                if cloud is None:
                    continue
                handle = object_id(detection.class_name, index)
                base_mm = camera_to_base_mm(
                    cloud.centroid_mm, self.camera_to_gripper, tcp_pose
                )
                candidates.append(
                    WristCandidate(
                        handle=handle,
                        class_name=detection.class_name,
                        base_m=base_point_m(base_mm),
                    )
                )
                clouds[handle] = cloud
        except (ValueError, WristCalibrationError) as exc:
            self._tracking_status(f"depth/base 좌표 변환 실패: {exc}")
            return

        if not candidates:
            self._tracking_status("검출 마스크에 유효한 depth가 없습니다")
            return

        expected = (
            request.expected_base.x,
            request.expected_base.y,
            request.expected_base.z,
        )
        match = match_target(
            request.class_name,
            expected,
            candidates,
            tolerance_m=self.match_tolerance_m,
            max_vertical_gap_m=self.match_max_vertical_gap_m,
        )
        if match is None:
            seen = ", ".join(sorted({candidate.class_name for candidate in candidates}))
            self._tracking_status(
                f"webcam 목표 {request.class_name}과 좌표가 맞지 않습니다 "
                f"(보이는 것: {seen})"
            )
            return
        if match.ambiguous and self.refuse_ambiguous_match:
            self._tracking_status(
                "같은 종류 두 개가 가까워 목표를 확정할 수 없습니다 "
                f"({match.horizontal_distance_m * 1000:.0f}mm vs "
                f"{match.runner_up_distance_m * 1000:.0f}mm)"
            )
            return

        cloud = clouds[match.candidate.handle]
        if cloud.size < self.min_cloud_points:
            self._tracking_status(
                f"일치한 마스크 점군이 {cloud.size}개뿐입니다 "
                f"(최소 {self.min_cloud_points})"
            )
            return

        stable = self.stability.observe(
            match.candidate.class_name,
            match.candidate.base_m,
            stamp_ns=observation.color.stamp_ns,
        )
        if not stable:
            self._tracking_status(
                f"좌표 일치 {self.stability.count}/{self.stability.required_frames} "
                f"({match.horizontal_distance_m * 1000:.0f}mm)",
                reset_stability=False,
            )
            return

        self.get_logger().info(
            f"confirmed {request.object_id} -> wrist {match.candidate.handle} "
            f"for {self.stability.count} frames at "
            f"{match.horizontal_distance_m * 1000:.0f}mm, {cloud.size} points; "
            f"sync rgb-depth="
            f"{abs(observation.color.stamp_ns - observation.depth.stamp_ns) / 1e6:.1f}ms "
            f"rgb-tcp="
            f"{abs(observation.color.stamp_ns - observation.pose.stamp_ns) / 1e6:.1f}ms"
        )
        # State flips before submitting, so additional image callbacks cannot
        # enqueue a second GraspGenX job.  The GPU work is off the ROS executor;
        # cancel and timeout callbacks therefore remain live during inference.
        self.tracking_busy = True
        self.planning_request = request
        self.planning_generation = self.session_generation
        self.planning_started = perf_counter()
        self.planning_source_stamp_ns = observation.color.stamp_ns
        try:
            self.grasp_future = self.grasp_executor.submit(
                self._compute_grasp, cloud, tuple(tcp_pose)
            )
        except Exception as exc:
            self.tracking_busy = False
            self.grasp_future = None
            self.planning_request = None
            self.planning_generation = 0
            self.planning_started = 0.0
            self.planning_source_stamp_ns = 0
            self._finish_failure(request, f"GraspGenX 처리 실패: {exc}")

    def _finish_failure(self, request: GraspRequest, reason: str) -> None:
        if (
            self.pending_request is not None
            and self.pending_request.request_id == request.request_id
        ):
            self._clear_tracking()
            self._refuse(request, reason)

    def _compute_grasp(self, cloud, tcp_pose) -> GraspComputation:
        """Heavy, ROS-free calculation executed on the GraspGenX worker."""

        grasps = self.grasp_client.generate(cloud.points_mm)
        if not grasps:
            return GraspComputation(
                success=False,
                reason="GraspGenX가 파지 후보를 내지 못했습니다",
            )

        # Camera-frame grasps -> base frame. The chain is in millimetres, the
        # poses in metres, so the transform's translation is scaled down once.
        camera_to_base = pose_matrix_from_posx(tcp_pose) @ self.camera_to_gripper
        camera_to_base_m = scale_pose_translation(camera_to_base, 1.0 / MM_PER_M)
        # Rebuild rather than patch: the transform is rigid, so re-deriving the
        # contact point and approach axis from the moved pose gives the same
        # answer through the same tested code path.
        base_grasps = build_candidates(
            np.stack([transform_pose(camera_to_base_m, c.pose) for c in grasps]),
            [c.confidence for c in grasps],
            self.grasp_client.fingertip_m,
        )
        if not base_grasps:
            return GraspComputation(
                success=False,
                reason="GraspGenX 파지 후보를 좌표로 변환하지 못했습니다",
            )

        chosen = select_grasp(
            base_grasps,
            self.inside_workspace,
            max_tilt_degrees=self.max_approach_tilt_deg,
            min_confidence=self.min_grasp_confidence,
            standoff_m=self.standoff_m,
        )
        if chosen is None:
            best = max(g.confidence for g in base_grasps)
            tilts = [approach_tilt_degrees(g.pose) for g in base_grasps]
            return GraspComputation(
                success=False,
                reason=(
                    f"후보 {len(base_grasps)}개가 모두 걸러졌습니다 "
                    f"(최고 신뢰도 {best:.2f}, 최소 기울기 {min(tilts):.0f}°, "
                    f"허용 {self.max_approach_tilt_deg:.0f}°)"
                ),
            )

        # The commanded pose is the *contact* pose: the gripper-base pose moved
        # forward by the fingertip offset, because the Doosan TCP (GripperDA_v1)
        # sits at the fingers, not at the mounting flange.
        contact_pose = _translated(chosen.pose, chosen.contact_m)
        try:
            target_posx = posx_from_pose(contact_pose)
            pregrasp_posx = posx_from_pose(pregrasp_pose(contact_pose, self.standoff_m))
        except GraspGeometryError as exc:
            return GraspComputation(
                success=False,
                reason=f"파지 자세를 posx로 변환할 수 없습니다: {exc}",
            )

        return GraspComputation(
            success=True,
            target_posx=tuple(target_posx),
            pregrasp_posx=tuple(pregrasp_posx),
            confidence=float(chosen.confidence),
            approach_tilt_deg=float(approach_tilt_degrees(chosen.pose)),
            candidate_count=len(base_grasps),
            cloud_points=int(cloud.size),
        )

    def _poll_grasp_future(self) -> None:
        future = self.grasp_future
        if future is None or not future.done():
            return

        request = self.planning_request
        generation = self.planning_generation
        started = self.planning_started
        source_stamp_ns = self.planning_source_stamp_ns
        self.grasp_future = None
        self.planning_request = None
        self.planning_generation = 0
        self.planning_started = 0.0
        self.planning_source_stamp_ns = 0
        self.tracking_busy = False

        active = request is not None and result_matches_session(
            request.request_id,
            generation,
            self.pending_request.request_id if self.pending_request else None,
            self.session_generation,
        )
        try:
            result = future.result()
        except Exception as exc:
            if active:
                self.get_logger().error(
                    f"grasp planning raised for {request.object_id}: {exc}"
                )
                self._finish_failure(request, f"GraspGenX 처리 실패: {exc}")
            return

        # A stop/cancel may have arrived while CUDA was busy.  Generation and
        # request ID are both checked before any late result reaches the robot.
        if not active:
            if request is not None:
                self.get_logger().info(
                    f"discarded stale grasp result for {request.request_id}"
                )
            return
        if monotonic() >= self.tracking_deadline:
            self._finish_failure(request, "GraspGenX 결과가 tracking deadline을 넘었습니다")
            return
        if not result.success:
            self._finish_failure(request, result.reason)
            return

        plan = GraspPlan()
        plan.header.stamp = Time(nanoseconds=source_stamp_ns).to_msg()
        plan.header.frame_id = "base"
        plan.request_id = request.request_id
        plan.object_id = request.object_id
        plan.success = True
        plan.reason = ""
        plan.target_posx = list(result.target_posx)
        plan.pregrasp_posx = list(result.pregrasp_posx)
        plan.confidence = result.confidence
        plan.approach_tilt_deg = result.approach_tilt_deg
        plan.candidate_count = result.candidate_count
        plan.cloud_points = result.cloud_points
        self._clear_tracking()
        self.plan_publisher.publish(plan)

        elapsed = perf_counter() - started
        source_age = max(
            0.0,
            (self.get_clock().now().nanoseconds - source_stamp_ns) / 1e9,
        )
        self.durations.append(elapsed)
        self.get_logger().info(
            f"grasp plan for {request.object_id}: conf={plan.confidence:.3f} "
            f"tilt={plan.approach_tilt_deg:.0f}° "
            f"target=({result.target_posx[0]:.1f}, {result.target_posx[1]:.1f}, "
            f"{result.target_posx[2]:.1f})mm "
            f"in {elapsed:.2f}s (source age {source_age:.2f}s)"
        )

    def destroy_node(self):
        self.session_generation += 1
        self.grasp_executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def _translated(pose, translation) -> np.ndarray:
    """Same rotation, new origin."""

    matrix = np.asarray(pose, dtype=np.float64).copy()
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64)
    return matrix


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WristGraspNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
