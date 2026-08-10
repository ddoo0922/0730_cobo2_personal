"""Small dependency-free IoU tracker used by the YOLO perception node."""

from dataclasses import dataclass, field
from typing import Any, Iterable, Optional, Tuple


BBox = Tuple[float, float, float, float]


# The instance mask is a boolean array shaped like the color frame, or None when
# the model is detect-only. It is excluded from comparison and repr because a
# frozen dataclass would otherwise build __eq__/__hash__ over a NumPy array,
# where `==` yields an array and hashing raises. Identity never depends on it:
# the mask belongs to one frame, the track does not.
_mask_field = field(default=None, compare=False, repr=False)


@dataclass(frozen=True)
class Detection:
    """One detector result before identity assignment."""

    class_id: int
    class_name: str
    confidence: float
    bbox: BBox
    mask: Optional[Any] = _mask_field


@dataclass(frozen=True)
class TrackedDetection:
    """A detector result with a stable ID and image-plane velocity."""

    track_id: int
    class_id: int
    class_name: str
    confidence: float
    bbox: BBox
    velocity_x: float
    velocity_y: float
    mask: Optional[Any] = _mask_field


@dataclass
class _Track:
    track_id: int
    class_id: int
    bbox: BBox
    center_x: float
    center_y: float
    timestamp: Optional[float]
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    missed_frames: int = 0


def box_iou(first: BBox, second: BBox) -> float:
    """Return intersection-over-union for two ``xyxy`` boxes."""

    x_min = max(first[0], second[0])
    y_min = max(first[1], second[1])
    x_max = min(first[2], second[2])
    y_max = min(first[3], second[3])
    intersection = max(0.0, x_max - x_min) * max(0.0, y_max - y_min)
    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(
        0.0, second[3] - second[1]
    )
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def box_center(box: BBox) -> Tuple[float, float]:
    return (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0


class IoUTracker:
    """Associate same-class boxes across frames using greedy IoU matching.

    It intentionally has no ROS, OpenCV, or ML dependency. This keeps the
    identity logic unit-testable on a machine without a camera or GPU.
    """

    def __init__(
        self,
        iou_threshold: float = 0.3,
        max_missed_frames: int = 5,
        velocity_smoothing: float = 0.5,
    ):
        if not 0.0 <= iou_threshold <= 1.0:
            raise ValueError("iou_threshold must be between zero and one")
        if max_missed_frames < 0:
            raise ValueError("max_missed_frames must be non-negative")
        if not 0.0 <= velocity_smoothing <= 1.0:
            raise ValueError("velocity_smoothing must be between zero and one")

        self.iou_threshold = iou_threshold
        self.max_missed_frames = max_missed_frames
        self.velocity_smoothing = velocity_smoothing
        self._next_track_id = 1
        self._tracks = {}

    def reset(self) -> None:
        self._next_track_id = 1
        self._tracks.clear()

    def update(
        self,
        detections: Iterable[Detection],
        timestamp: Optional[float] = None,
    ) -> list[TrackedDetection]:
        detections = list(detections)
        candidates = []
        for track_id, track in self._tracks.items():
            for detection_index, detection in enumerate(detections):
                if track.class_id != detection.class_id:
                    continue
                overlap = box_iou(track.bbox, detection.bbox)
                if overlap >= self.iou_threshold:
                    candidates.append((overlap, track_id, detection_index))

        matched_tracks = set()
        matched_detections = set()
        assignments = {}
        for _, track_id, detection_index in sorted(candidates, reverse=True):
            if track_id in matched_tracks or detection_index in matched_detections:
                continue
            matched_tracks.add(track_id)
            matched_detections.add(detection_index)
            assignments[detection_index] = track_id

        for track_id, track in list(self._tracks.items()):
            if track_id not in matched_tracks:
                track.missed_frames += 1
                if track.missed_frames > self.max_missed_frames:
                    del self._tracks[track_id]

        output = []
        for detection_index, detection in enumerate(detections):
            center_x, center_y = box_center(detection.bbox)
            track_id = assignments.get(detection_index)
            if track_id is None:
                track_id = self._next_track_id
                self._next_track_id += 1
                track = _Track(
                    track_id=track_id,
                    class_id=detection.class_id,
                    bbox=detection.bbox,
                    center_x=center_x,
                    center_y=center_y,
                    timestamp=timestamp,
                )
                self._tracks[track_id] = track
            else:
                track = self._tracks[track_id]
                elapsed = None
                if timestamp is not None and track.timestamp is not None:
                    elapsed = timestamp - track.timestamp
                if elapsed is not None and elapsed > 0.0:
                    measured_x = (center_x - track.center_x) / elapsed
                    measured_y = (center_y - track.center_y) / elapsed
                    weight = self.velocity_smoothing
                    track.velocity_x = (
                        weight * measured_x + (1.0 - weight) * track.velocity_x
                    )
                    track.velocity_y = (
                        weight * measured_y + (1.0 - weight) * track.velocity_y
                    )
                track.class_id = detection.class_id
                track.bbox = detection.bbox
                track.center_x = center_x
                track.center_y = center_y
                track.timestamp = timestamp
                track.missed_frames = 0

            output.append(
                TrackedDetection(
                    track_id=track_id,
                    class_id=detection.class_id,
                    class_name=detection.class_name,
                    confidence=detection.confidence,
                    bbox=detection.bbox,
                    velocity_x=track.velocity_x,
                    velocity_y=track.velocity_y,
                    mask=detection.mask,
                )
            )

        return output
