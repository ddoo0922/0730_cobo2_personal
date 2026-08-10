"""YOLO detection wrapper and ROS image helpers, kept free of node state."""

from array import array
from pathlib import Path
from time import perf_counter

import cv2
import numpy as np

from vla_system.perception.tracker import Detection, TrackedDetection


def image_message_to_bgr(message) -> np.ndarray:
    """Convert common ROS color encodings without depending on cv_bridge."""

    if message.encoding not in ("bgr8", "rgb8"):
        raise ValueError(
            f"Unsupported image encoding: {message.encoding}; expected bgr8 or rgb8"
        )
    row = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height,
        message.step,
    )
    image = row[:, : message.width * 3].reshape(message.height, message.width, 3)
    if message.encoding == "rgb8":
        return image[:, :, ::-1].copy()
    return image


def bgr_to_image_message(image: np.ndarray, header):
    from sensor_msgs.msg import Image

    message = Image()
    message.header = header
    message.height = image.shape[0]
    message.width = image.shape[1]
    message.encoding = "bgr8"
    message.is_bigendian = 0
    message.step = image.shape[1] * 3
    message.data = array("B", image.tobytes())
    return message


DEFAULT_MODEL_STEM = "yolo26s-seg"


def resolve_model_path(backend: str, override: str) -> str:
    from ament_index_python.packages import get_package_share_directory

    if override:
        path = Path(override).expanduser()
    else:
        model_directory = Path(get_package_share_directory("vla_system")) / "models"
        path = (
            model_directory / f"{DEFAULT_MODEL_STEM}_openvino_model"
            if backend == "openvino"
            else model_directory / f"{DEFAULT_MODEL_STEM}.pt"
        )
    if not path.exists():
        raise FileNotFoundError(f"YOLO model not found: {path}")
    if path.is_dir() and not any(path.glob("*.xml")):
        # setup.py's data_files entry leaves the directory behind even when no IR
        # has been exported, so existence alone is not enough to trust it.
        raise FileNotFoundError(
            f"no OpenVINO IR (*.xml) in {path}; export one with "
            f"`yolo export model={DEFAULT_MODEL_STEM}.pt format=openvino` "
            "or set backend: pytorch"
        )
    return str(path)


def result_masks(result) -> list[np.ndarray] | None:
    """Return one boolean mask per detection, in the same order as the boxes.

    ``None`` when the model is detect-only or nothing was segmented, which the
    callers treat as "fall back to the bounding box". Predictions are made with
    ``retina_masks=True`` so these are already at the color frame's resolution;
    the check below refuses anything else rather than silently rescaling, since a
    depth sample taken through a mis-scaled mask would read the background.
    """

    masks = getattr(result, "masks", None)
    if masks is None or masks.data is None or len(masks.data) == 0:
        return None
    data = masks.data.cpu().numpy().astype(bool)
    if data.shape[1:] != tuple(result.orig_shape):
        raise ValueError(
            f"mask resolution {data.shape[1:]} does not match the frame "
            f"{tuple(result.orig_shape)}; predict with retina_masks=True"
        )
    return list(data)


def result_to_detections(
    result,
    target_classes: set[str],
    excluded_classes: set[str] | None = None,
) -> list[Detection]:
    detections = []
    excluded_classes = excluded_classes or set()
    if result.boxes is None:
        return detections
    names = result.names
    masks = result_masks(result)
    for index, (box, confidence, class_id) in enumerate(
        zip(
            result.boxes.xyxy.cpu().tolist(),
            result.boxes.conf.cpu().tolist(),
            result.boxes.cls.cpu().tolist(),
        )
    ):
        class_index = int(class_id)
        if isinstance(names, dict):
            class_name = str(names.get(class_index, class_index))
        else:
            class_name = str(names[class_index])
        if class_name in excluded_classes:
            continue
        if target_classes and class_name not in target_classes:
            continue
        detections.append(
            Detection(
                class_id=class_index,
                class_name=class_name,
                confidence=float(confidence),
                bbox=tuple(float(value) for value in box),
                mask=masks[index] if masks is not None and index < len(masks) else None,
            )
        )
    return detections


def mask_centroid(mask: np.ndarray):
    """Return the pixel centre of mass of a boolean instance mask.

    Preferred over the bounding-box centre as the pixel to map onto the table:
    for a tilted banana or a partly occluded cup the box centre can land beside
    the object, which would aim the arm at empty table even when the mapping
    itself is correct.
    """

    if mask.ndim != 2:
        raise ValueError("mask must be a two-dimensional array")
    rows, columns = np.nonzero(mask)
    if rows.size == 0:
        return None
    return float(columns.mean()), float(rows.mean())


def object_id(class_name: str, track_id: int) -> str:
    """Build the handle the LLM uses to refer to one object.

    Spaces have to go: "wine glass" would otherwise produce "wine glass_4",
    which the model tends to re-emit with the space collapsed or quoted
    differently, and the executor's lookup is an exact string match.
    """
    slug = "_".join(str(class_name).strip().lower().split())
    return f"{slug or 'object'}_{int(track_id)}"


def draw_tracks(
    image: np.ndarray,
    detections: list[TrackedDetection],
    labels: dict[int, str] | None = None,
) -> np.ndarray:
    """Draw boxes labelled with the same handles the LLM sees."""

    annotated = image.copy()
    labels = labels or {}
    for detection in detections:
        x_min, y_min, x_max, y_max = (int(value) for value in detection.bbox)
        color = (
            (37 * detection.track_id) % 205 + 50,
            (17 * detection.track_id) % 205 + 50,
            (97 * detection.track_id) % 205 + 50,
        )
        label = labels.get(
            detection.track_id,
            object_id(detection.class_name, detection.track_id),
        )
        mask = getattr(detection, "mask", None)
        if mask is not None and mask.shape == annotated.shape[:2]:
            # Tint the segmented pixels so the clarification crops the GUI cuts
            # out of this image show what the depth sample actually covered.
            region = annotated[mask]
            annotated[mask] = (region * 0.6 + np.array(color) * 0.4).astype(
                annotated.dtype
            )
        cv2.rectangle(annotated, (x_min, y_min), (x_max, y_max), color, 2)
        cv2.putText(
            annotated,
            f"{label} {detection.confidence:.2f}",
            (x_min, max(18, y_min - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    return annotated


class YoloDetector:
    """Load YOLO once, warm it up, and run one frame at a time."""

    def __init__(
        self,
        backend: str,
        model_override: str,
        device: str,
        imgsz: int,
        confidence: float,
        max_detections: int,
        torch_threads: int,
    ):
        backend = backend.lower()
        if backend not in ("pytorch", "openvino"):
            raise ValueError("backend must be 'pytorch' or 'openvino'")
        if imgsz <= 0:
            raise ValueError("imgsz must be greater than zero")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be between zero and one")
        if max_detections <= 0:
            raise ValueError("max_detections must be greater than zero")
        if torch_threads <= 0:
            raise ValueError("torch_threads must be greater than zero")

        import torch
        from ultralytics import YOLO

        torch.set_num_threads(torch_threads)
        self.backend = backend
        self.device = device
        self.imgsz = imgsz
        self.confidence = confidence
        self.max_detections = max_detections
        self.model_path = resolve_model_path(backend, model_override)

        load_start = perf_counter()
        self.model = YOLO(self.model_path, task="segment")
        self.predict(np.zeros((480, 640, 3), dtype=np.uint8))
        self.load_ms = (perf_counter() - load_start) * 1000.0

    def predict(self, frame: np.ndarray):
        return self.model.predict(
            frame,
            imgsz=self.imgsz,
            conf=self.confidence,
            max_det=self.max_detections,
            device=self.device,
            # Masks come back at the frame's own resolution instead of the
            # letterboxed model input, so they index the aligned depth image
            # directly. At the RealSense default of 640x480 with imgsz=640 the
            # two are already identical, so this costs nothing there.
            retina_masks=True,
            verbose=False,
        )[0]
