#!/usr/bin/env python3
"""Fixed perception logic: webcam -> YOLO-seg -> tracking -> robot base coords.

This node is deliberately the only place in the system that contains
deterministic rules. It answers "what is on the table and where is it in the
arm's own frame", and nothing else. Which of those objects to touch is not
decided here -- that is the agent's job.

The camera is a **fixed overhead webcam**, not the RealSense. The RealSense now
rides on the wrist, so it cannot survey the table: whatever it sees depends on
where the arm happens to be pointing, which is useless as a scene source. The
webcam stays still, so one calibration keeps holding.

Because a single webcam has no depth, positions come from the table homography
measured by `table_homography_test` (pixel -> base XY, plus a fitted tabletop
plane for Z). That carries one standing assumption: **objects lie on the
calibrated table**. A tall object's mask centre sits on its top face, so the
mapping reports where that face *would* touch the table -- off by a parallax
error that grows with height and with distance from the camera axis. Good enough
to send the arm to the right object; not good enough to close fingers blind,
which is what the wrist RealSense is for later.

Publishes one SceneSnapshot per captured frame. The agent samples the latest one
at each decision point; the executor samples it again the instant it starts
moving, because a position from one LLM round-trip ago is already stale.
"""

from collections import deque
from statistics import fmean
from time import perf_counter
from typing import Optional

import cv2
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vla_interfaces.msg import SceneObject, SceneSnapshot

from vla_system.perception.color_features import classify_detection_color
from vla_system.perception.detector import (
    YoloDetector,
    bgr_to_image_message,
    draw_tracks,
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

WEBCAM_FRAME_ID = "webcam"


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("vla_perception")

        self.declare_parameter("scene_topic", "/vla/scene")
        self.declare_parameter("annotated_topic", "/vla/perception/annotated_image")

        self.declare_parameter("webcam_device", "/dev/video8")
        self.declare_parameter("webcam_width", 1280)
        self.declare_parameter("webcam_height", 720)
        self.declare_parameter("webcam_fps", 30.0)
        self.declare_parameter("webcam_fourcc", "MJPG")
        self.declare_parameter("capture_rate_hz", 15.0)

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

        # Camera before calibration: the loader needs the opened resolution to
        # reject a calibration clicked at a different one.
        self.capture = self._open_camera()
        self.calibration = self._load_calibration()

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
        self.read_failures = 0
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

    def _load_calibration(self):
        path = str(self.get_parameter("calibration_file").value)
        try:
            calibration = load_table_calibration(path, self.frame_size)
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

    def _open_camera(self):
        raw = str(self.get_parameter("webcam_device").value).strip()
        source = int(raw) if raw.isdigit() else raw
        capture = cv2.VideoCapture(source)
        if not capture.isOpened():
            raise RuntimeError(f"cannot open webcam: {raw}")
        # Pixel format before size: uncompressed 720p is bandwidth-bound to about
        # 5 fps on USB 2, which measured as 196ms per blocking read and made every
        # published position a fifth of a second stale. MJPEG at the same size
        # measured 64ms. Compression does not move pixels, so the homography
        # stays valid; set this to "" if a camera dislikes the format.
        fourcc = str(self.get_parameter("webcam_fourcc").value).strip()
        if fourcc:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc[:4]))
        capture.set(
            cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("webcam_width").value)
        )
        capture.set(
            cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("webcam_height").value)
        )
        capture.set(cv2.CAP_PROP_FPS, float(self.get_parameter("webcam_fps").value))
        # Ask for the smallest driver buffer available: a queued frame is a stale
        # frame, and the arm is going to move based on it.
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # A driver may quietly substitute a size it prefers, so read back what we
        # actually got rather than trusting the request.
        self.frame_size = (width, height)
        self.get_logger().info(f"webcam {raw} opened at {width}x{height}")
        return capture

    # ------------------------------------------------------------------- scene

    def capture_once(self) -> None:
        started = perf_counter()
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.read_failures += 1
            return

        stamp = self.get_clock().now()
        result = self.detector.predict(frame)
        raw = result_to_detections(result, self.target_classes, self.excluded_classes)
        tracked = self.tracker.update(raw, stamp.nanoseconds / 1_000_000_000.0)

        snapshot = SceneSnapshot()
        snapshot.header.stamp = stamp.to_msg()
        snapshot.header.frame_id = "base"
        snapshot.camera_frame = WEBCAM_FRAME_ID
        snapshot.calibration_ok = self.calibration is not None

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
                f"no webcam frames captured ({self.read_failures} read failures). "
                "check webcam_device."
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
            + (f" read_failures={self.read_failures}" if self.read_failures else "")
        )

    def destroy_node(self) -> bool:
        capture = getattr(self, "capture", None)
        if capture is not None:
            try:
                capture.release()
            except Exception:
                pass
        return super().destroy_node()


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
