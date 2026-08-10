"""Timestamp pairing and stability gates for continuous wrist tracking.

This module deliberately has no ROS imports.  The wrist node receives RGB,
aligned depth and TCP pose on three independent topics; these helpers make the
pairing rule explicit and testable instead of relying on callback arrival order.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Iterable, Optional, Sequence


@dataclass(frozen=True)
class TimedSample:
    stamp_ns: int
    value: Any


@dataclass(frozen=True)
class SyncedObservation:
    color: TimedSample
    depth: TimedSample
    pose: TimedSample


def stamp_ns(message) -> int:
    """Return a ROS message header stamp as integer nanoseconds."""

    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _closest(target_ns: int, samples: Iterable[TimedSample]) -> Optional[TimedSample]:
    return min(samples, key=lambda item: abs(item.stamp_ns - target_ns), default=None)


def select_synced_observation(
    colors: Sequence[TimedSample],
    depths: Sequence[TimedSample],
    poses: Sequence[TimedSample],
    *,
    after_ns: int,
    last_color_ns: int,
    rgbd_tolerance_ns: int,
    pose_tolerance_ns: int,
) -> Optional[SyncedObservation]:
    """Choose the newest unprocessed RGB frame with compatible depth and pose.

    Cross-topic callback ordering is not a synchronization contract.  Selection
    is therefore based only on sensor timestamps.  A pose may be just before or
    just after the image; the caller controls that approximation with
    ``pose_tolerance_ns`` and publishes samples frequently enough to keep it
    small.
    """

    if rgbd_tolerance_ns < 0 or pose_tolerance_ns < 0:
        raise ValueError("timestamp tolerances must be non-negative")

    for color in sorted(colors, key=lambda item: item.stamp_ns, reverse=True):
        if color.stamp_ns <= after_ns or color.stamp_ns <= last_color_ns:
            continue
        # The registration stamp is a freshness barrier for the whole
        # observation, not just RGB.  The initial pose is intentionally stamped
        # exactly at that barrier, so it is eligible for the first new frame.
        depth = _closest(
            color.stamp_ns,
            (sample for sample in depths if sample.stamp_ns > after_ns),
        )
        pose = _closest(
            color.stamp_ns,
            (sample for sample in poses if sample.stamp_ns >= after_ns),
        )
        if depth is None or pose is None:
            continue
        # Do not consume an image using only the last pose seen so far.  Waiting
        # for one pose at/after the exposure gives us a bracket, after which the
        # nearest sample really is the nearest rather than merely the newest
        # currently available sample.
        if not any(
            0 <= sample.stamp_ns - color.stamp_ns <= pose_tolerance_ns
            for sample in poses
        ):
            continue
        if abs(depth.stamp_ns - color.stamp_ns) > rgbd_tolerance_ns:
            continue
        if abs(pose.stamp_ns - color.stamp_ns) > pose_tolerance_ns:
            continue
        return SyncedObservation(color=color, depth=depth, pose=pose)
    return None


def result_matches_session(
    result_request_id: str,
    result_generation: int,
    active_request_id: Optional[str],
    active_generation: int,
) -> bool:
    """Whether an asynchronous result still belongs to the active session."""

    return bool(
        active_request_id
        and result_request_id == active_request_id
        and int(result_generation) == int(active_generation)
    )


class MatchStability:
    """Require one compact base-frame cluster in consecutive observations."""

    def __init__(
        self,
        required_frames: int,
        max_jump_m: float,
        max_gap_ns: Optional[int] = None,
    ):
        if required_frames < 1:
            raise ValueError("required_frames must be at least one")
        if max_jump_m < 0.0:
            raise ValueError("max_jump_m must be non-negative")
        if max_gap_ns is not None and max_gap_ns <= 0:
            raise ValueError("max_gap_ns must be positive when provided")
        self.required_frames = int(required_frames)
        self.max_jump_m = float(max_jump_m)
        self.max_gap_ns = int(max_gap_ns) if max_gap_ns is not None else None
        self.count = 0
        self.class_name = ""
        self.base_m: Optional[tuple[float, float, float]] = None
        self.last_stamp_ns: Optional[int] = None
        self.points: list[tuple[float, float, float]] = []

    def reset(self) -> None:
        self.count = 0
        self.class_name = ""
        self.base_m = None
        self.last_stamp_ns = None
        self.points = []

    def observe(
        self,
        class_name: str,
        base_m: Sequence[float],
        *,
        stamp_ns: Optional[int] = None,
    ) -> bool:
        point = tuple(float(value) for value in base_m[:3])
        if len(point) != 3:
            raise ValueError("base_m must contain three values")

        continuous = self.class_name == class_name and self.base_m is not None
        if continuous and self.max_gap_ns is not None and stamp_ns is not None:
            if self.last_stamp_ns is None:
                continuous = False
            else:
                gap_ns = int(stamp_ns) - self.last_stamp_ns
                continuous = 0 < gap_ns <= self.max_gap_ns
        if continuous:
            # Pairwise-only comparison permits a slow walk (0, 19, 38 mm) to
            # pass a 20 mm gate.  Every point in the window must instead fit
            # the same compact cluster.
            continuous = all(
                math.hypot(point[0] - previous[0], point[1] - previous[1])
                <= self.max_jump_m
                for previous in self.points
            )

        self.points = self.points + [point] if continuous else [point]
        self.points = self.points[-self.required_frames :]
        self.count = len(self.points)
        self.class_name = str(class_name)
        self.base_m = point
        self.last_stamp_ns = int(stamp_ns) if stamp_ns is not None else None
        return self.count >= self.required_frames
