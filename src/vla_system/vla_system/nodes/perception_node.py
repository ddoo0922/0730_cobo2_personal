#!/usr/bin/env python3
"""Fixed perception logic: shared camera topic -> YOLO-seg -> tracking -> robot base coords.

This node is deliberately the only place in the system that contains
deterministic rules. It answers "what is on the table and where is it in the
arm's own frame", and nothing else. Which of those objects to touch is not
decided here -- that is the agent's job.

The camera is the **fixed D435i that cobot2_ws's pick_fsm also uses** (shared
hardware, confirmed 2026-08-10 -- see docs/context/constraints.md "카메라
구성"). This node used to open `/dev/videoN` directly with `cv2.VideoCapture`,
which is wrong for a shared camera: two independent V4L2 opens of the same
RealSense device race and generally break one or both (2026-08-11 finding,
same doc). Instead this subscribes the `image_topic` that a `realsense2_camera`
driver publishes -- cobot2_ws's own launch when integrated, or this ws's own
`realsense2_camera` node when run standalone. Either way there is exactly one
process opening the device, and any number of subscribers.

Because a single color frame has no depth, positions come from the table
homography measured by `table_homography_test` (pixel -> base XY, plus a fitted
tabletop plane for Z). That carries one standing assumption: **objects lie on
the calibrated table**. A tall object's mask centre sits on its top face, so the
mapping reports where that face *would* touch the table -- off by a parallax
error that grows with height and with distance from the camera axis. Good enough
to send the arm to the right object; not good enough to close fingers blind,
which is what the wrist RealSense is for later.

Publishes one SceneSnapshot per processed frame. The agent samples the latest
one at each decision point; the executor samples it again the instant it starts
moving, because a position from one LLM round-trip ago is already stale.
"""

import threading
from collections import deque
from statistics import fmean
from time import perf_counter, monotonic
from typing import Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from vla_interfaces.msg import SceneObject, SceneSnapshot

from vla_system.perception.color_features import classify_detection_color
from vla_system.perception.detector import (
    YoloDetector,
    bgr_to_image_message,
    draw_tracks,
    image_message_to_bgr,
    mask_centroid,
    object_id,
    result_to_detections,
)
from vla_system.perception.table_homography import (
    CalibrationError,
    load_table_calibration,
    reprojection_errors_mm,
    table_point_from_pixel,
)
from vla_system.perception.tracker import IoUTracker

CAMERA_FRAME_ID = "shared_camera"


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("vla_perception")

        self.declare_parameter("scene_topic", "/vla/scene")
        self.declare_parameter("annotated_topic", "/vla/perception/annotated_image")

        # realsense2_camera's default color topic. Whoever launched the driver
        # (cobot2_ws in integrated mode, this ws's own realsense2_camera launch
        # in standalone mode) owns the device; this node only subscribes.
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("capture_rate_hz", 15.0)
        # A stale image is worse than no image: it is a real position for an
        # object that has since moved. Same guard shape as vla_pick_bridge's
        # max_scene_age_s.
        self.declare_parameter("max_image_age_s", 2.0)

        self.declare_parameter(
            "calibration_file", "~/.ros/vla_table_homography.json"
        )
        self.declare_parameter("grasp_height_offset_m", 0.02)
        self.declare_parameter("require_inside_table", True)

        self.declare_parameter("backend", "pytorch")
        self.declare_parameter("model", "")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("max_detections", 30)
        # An empty list default infers as BYTE_ARRAY and then rejects any
        # string list, so this parameter has to opt out of static typing.
        self.declare_parameter(
            "target_classes", [], ParameterDescriptor(dynamic_typing=True)
        )
        self.declare_parameter("excluded_classes", ["person"])
        self.declare_parameter("classify_color", True)
        self.declare_parameter("use_masks", True)
        self.declare_parameter("torch_threads", 4)
        self.declare_parameter("publish_annotated", True)

        self.declare_parameter("tracker_iou_threshold", 0.3)
        self.declare_parameter("tracker_max_missed_frames", 5)
        self.declare_parameter("velocity_smoothing", 0.5)
        self.declare_parameter("report_interval", 5.0)

        self.classify_color = bool(self.get_parameter("classify_color").value)
        self.use_masks = bool(self.get_parameter("use_masks").value)
        self.publish_annotated = bool(self.get_parameter("publish_annotated").value)
        self.target_classes = {
            str(name) for name in (self.get_parameter("target_classes").value or [])
        }
        self.excluded_classes = {
            str(name) for name in self.get_parameter("excluded_classes").value
        }
        self.grasp_height_offset_m = float(
            self.get_parameter("grasp_height_offset_m").value
        )
        self.require_inside_table = bool(
            self.get_parameter("require_inside_table").value
        )

        # Calibration needs the camera's resolution to reject a calibration
        # clicked at a different one, but that resolution is only known once
        # the first Image message actually arrives (no more owned
        # cv2.VideoCapture to query up front) -- loaded lazily in
        # image_callback() the first time a frame comes in.
        self.calibration = None
        self._calibration_checked = False
        self.frame_size: Optional[tuple[int, int]] = None
        self.image_lock = threading.Lock()
        self.latest_image: Optional[Image] = None
        self.latest_image_monotonic = 0.0
        self.max_image_age_s = float(self.get_parameter("max_image_age_s").value)

        self.detector = YoloDetector(
            backend=str(self.get_parameter("backend").value),
            model_override=str(self.get_parameter("model").value),
            device=str(self.get_parameter("device").value),
            imgsz=int(self.get_parameter("imgsz").value),
            confidence=float(self.get_parameter("confidence").value),
            max_detections=int(self.get_parameter("max_detections").value),
            torch_threads=int(self.get_parameter("torch_threads").value),
        )
        self.get_logger().info(
            f"YOLO ready in {self.detector.load_ms:.1f}ms: {self.detector.model_path} "
            f"| backend={self.detector.backend} device={self.detector.device}"
        )

        self.tracker = IoUTracker(
            iou_threshold=float(self.get_parameter("tracker_iou_threshold").value),
            max_missed_frames=int(
                self.get_parameter("tracker_max_missed_frames").value
            ),
            velocity_smoothing=float(self.get_parameter("velocity_smoothing").value),
        )

        self.total_frames = 0
        self.total_objects = 0
        self.total_positioned = 0
        self.missing_frames = 0
        self.stale_frames = 0
        self.inference_ms = deque(maxlen=300)
        self.processing_ms = deque(maxlen=300)

        stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.scene_publisher = self.create_publisher(
            SceneSnapshot, str(self.get_parameter("scene_topic").value), stream_qos
        )
        self.annotated_publisher = None
        if self.publish_annotated:
            self.annotated_publisher = self.create_publisher(
                Image, str(self.get_parameter("annotated_topic").value), stream_qos
            )
        # qos_profile_sensor_data (BEST_EFFORT) to match realsense2_camera's
        # own publisher QoS -- RELIABLE here would simply never match it and
        # the subscription would sit silent with no error (CLAUDE.md's QoS-
        # mismatch trap).
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self.image_callback,
            qos_profile_sensor_data,
        )

        capture_rate = float(self.get_parameter("capture_rate_hz").value)
        if capture_rate <= 0.0:
            raise ValueError("capture_rate_hz must be greater than zero")
        # One timer, one frame. Inference is synchronous, so a slow frame simply
        # delays the next tick instead of queueing work the scene will outgrow.
        self.create_timer(1.0 / capture_rate, self.capture_once)
        self.create_timer(
            float(self.get_parameter("report_interval").value), self.report
        )

    # ------------------------------------------------------------------ setup

    def _load_calibration(self, frame_size: tuple[int, int]):
        path = str(self.get_parameter("calibration_file").value)
        try:
            calibration = load_table_calibration(path, frame_size)
        except FileNotFoundError:
            self.get_logger().error(
                f"table calibration not found: {path}. Run "
                "'ros2 run vla_system table_homography_test' to measure it. "
                "Objects will be reported without base positions."
            )
            return None
        except (CalibrationError, ValueError, KeyError) as exc:
            self.get_logger().error(
                f"table calibration unusable ({exc}); positions will be withheld"
            )
            return None

        errors = reprojection_errors_mm(calibration)
        a, b, c = calibration.plane_z_coefficients
        self.get_logger().info(
            f"table calibration {path} | XY reprojection error [mm]: "
            + ", ".join(f"{value:.2f}" for value in errors)
        )
        self.get_logger().info(
            f"table plane: Z_mm = {a:.6f}*X + {b:.6f}*Y + {c:.2f} | "
            f"grasp Z = table + {self.grasp_height_offset_m * 1000.0:.1f}mm"
        )
        return calibration

    def image_callback(self, message: Image) -> None:
        with self.image_lock:
            self.latest_image = message
            self.latest_image_monotonic = monotonic()
        # Calibration needs a resolution, which only exists once one frame has
        # actually arrived -- checked once, not every callback, so a missing
        # calibration file does not retry (and re-log the same error) 15x/s.
        if not self._calibration_checked:
            self._calibration_checked = True
            self.frame_size = (message.width, message.height)
            self.calibration = self._load_calibration(self.frame_size)

    # ------------------------------------------------------------------- scene

    def capture_once(self) -> None:
        started = perf_counter()
        with self.image_lock:
            message = self.latest_image
            age_s = (
                monotonic() - self.latest_image_monotonic
                if message is not None
                else None
            )
        if message is None:
            self.missing_frames += 1
            return
        if age_s is not None and age_s > self.max_image_age_s:
            self.stale_frames += 1
            return
        try:
            frame = image_message_to_bgr(message)
        except ValueError as exc:
            self.get_logger().error(
                f"image_topic frame unusable ({exc})", throttle_duration_sec=10.0
            )
            self.missing_frames += 1
            return

        stamp = self.get_clock().now()
        result = self.detector.predict(frame)
        raw = result_to_detections(result, self.target_classes, self.excluded_classes)
        tracked = self.tracker.update(raw, stamp.nanoseconds / 1_000_000_000.0)

        snapshot = SceneSnapshot()
        snapshot.header.stamp = stamp.to_msg()
        snapshot.header.frame_id = "base"
        snapshot.camera_frame = CAMERA_FRAME_ID
        snapshot.calibration_ok = self.calibration is not None
        # frame_size is set from the first Image message (image_callback) --
        # always populated by the time capture_once can run at all, since
        # capture_once returns early above when latest_image is still None.
        snapshot.image_width, snapshot.image_height = self.frame_size

        labels: dict[int, str] = {}
        for detection in tracked:
            handle = object_id(detection.class_name, detection.track_id)
            labels[detection.track_id] = handle
            snapshot.objects.append(self.make_object(handle, detection, frame))

        self.scene_publisher.publish(snapshot)

        if self.annotated_publisher is not None:
            self.annotated_publisher.publish(
                bgr_to_image_message(
                    draw_tracks(frame, tracked, labels), snapshot.header
                )
            )

        self.total_frames += 1
        self.total_objects += len(snapshot.objects)
        self.total_positioned += sum(1 for o in snapshot.objects if o.position_valid)
        self.inference_ms.append(float(result.speed["inference"]))
        self.processing_ms.append((perf_counter() - started) * 1000.0)

    def make_object(self, handle, detection, frame) -> SceneObject:
        message = SceneObject()
        message.id = handle
        message.track_id = detection.track_id
        message.class_name = detection.class_name
        message.confidence = detection.confidence
        x_min, y_min, x_max, y_max = detection.bbox
        message.x_min = x_min
        message.y_min = y_min
        message.x_max = x_max
        message.y_max = y_max

        mask = getattr(detection, "mask", None) if self.use_masks else None
        name, confidence, _source = (
            classify_detection_color(frame, detection.bbox, mask)
            if self.classify_color
            else ("unknown", 0.0, "unknown")
        )
        message.color = name
        message.color_confidence = float(confidence)

        if self.calibration is None:
            return message

        pixel_x = (x_min + x_max) / 2.0
        pixel_y = (y_min + y_max) / 2.0
        if mask is not None:
            centroid = mask_centroid(mask)
            if centroid is not None:
                pixel_x, pixel_y = centroid

        point = table_point_from_pixel(
            self.calibration, pixel_x, pixel_y, self.grasp_height_offset_m
        )
        if point is None:
            return message
        if self.require_inside_table and not point.inside_table:
            # Outside the calibrated quadrilateral the homography is
            # extrapolating, and the fitted plane with it. Report the object so
            # the agent can talk about it, but never hand out that coordinate.
            return message

        message.position_valid = True
        message.position_base.x = point.x_m
        message.position_base.y = point.y_m
        message.position_base.z = point.z_m
        return message

    def report(self) -> None:
        if not self.processing_ms:
            self.get_logger().warning(
                f"no frames processed yet (missing={self.missing_frames} "
                f"stale={self.stale_frames}). Is something publishing "
                f"{self.get_parameter('image_topic').value}? "
                "(cobot2_ws's realsense2_camera launch, or this ws's own "
                "if run standalone)"
            )
            return
        positioned = (
            100.0 * self.total_positioned / self.total_objects
            if self.total_objects
            else 0.0
        )
        self.get_logger().info(
            f"frames={self.total_frames} objects={self.total_objects} "
            f"positioned={positioned:.1f}% "
            f"inference_avg={fmean(self.inference_ms):.1f}ms "
            f"processing_avg={fmean(self.processing_ms):.1f}ms"
            + (f" missing={self.missing_frames}" if self.missing_frames else "")
            + (f" stale={self.stale_frames}" if self.stale_frames else "")
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PerceptionNode()
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
