"""Turning a GraspGenX pose into something the Doosan can be commanded to.

All of this is pure geometry, kept out of ``graspgen_client`` so it can be tested
without a GPU, model weights, or a robot.

What a GraspGenX pose actually is
--------------------------------
Measured against the released ``onrobot_RG2`` description, not assumed:

- The 4x4 is the **gripper base** pose, in the same frame and units (metres) as
  the point cloud handed in. Its origin sits a median 159 mm off the object
  surface; ``origin + R @ fingertip`` sits 4 mm off it.
- ``fingertip`` for RG2 is ``[0, 0, 0.18]``, so the **approach axis is local +Z**
  (the fingertip direction and the rotation's third column agree to 1.000000).

Both facts have to be applied or every grasp lands 18 cm short along the
approach axis, which on a robot reads as "it keeps missing" rather than as a
frame bug.

Units
-----
GraspGenX speaks metres. The hand-eye calibration and Doosan ``posx`` speak
millimetres. Poses here are carried in **metres** and converted once, in
``posx_from_pose``, because that is the only place a Doosan command is formed.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

MM_PER_M = 1000.0

# RG2's fingertip offset from its base link, straight out of the gripper's
# config.json. Kept as the default rather than read at runtime so the pure layer
# has no filesystem dependency; the client passes the real value through.
RG2_FINGERTIP_M = (0.0, 0.0, 0.18)


class GraspGeometryError(ValueError):
    """A grasp pose or its derived command is not usable."""


@dataclass(frozen=True)
class GraspCandidate:
    """One scored grasp, expressed in the robot base frame, metres.

    Attributes
    ----------
    pose:
        4x4 gripper-base pose. Column 2 of its rotation is the approach axis.
    confidence:
        GraspGenX discriminator score. Comparable within one inference call.
    contact_m:
        Where the fingers actually close: ``pose`` translated by the fingertip
        offset along its own +Z.
    approach:
        Unit vector the gripper travels along to reach ``contact_m``.
    """

    pose: np.ndarray
    confidence: float
    contact_m: np.ndarray
    approach: np.ndarray


def _as_pose(matrix) -> np.ndarray:
    pose = np.asarray(matrix, dtype=np.float64)
    if pose.shape != (4, 4):
        raise GraspGeometryError(f"expected a 4x4 pose, got {pose.shape}")
    if not np.isfinite(pose).all():
        raise GraspGeometryError("pose contains NaN or inf")
    return pose


def contact_point_m(pose, fingertip_m: Sequence[float] = RG2_FINGERTIP_M) -> np.ndarray:
    """Where the fingers close, given a gripper-base pose."""

    matrix = _as_pose(pose)
    return matrix[:3, 3] + matrix[:3, :3] @ np.asarray(fingertip_m, dtype=np.float64)


def approach_axis(pose) -> np.ndarray:
    """The direction the gripper advances along, i.e. its local +Z."""

    return _as_pose(pose)[:3, 2]


def pregrasp_pose(pose, standoff_m: float):
    """Back the pose off along its own approach axis by ``standoff_m``.

    The arm goes here first, then travels straight in. Retreating along the
    approach axis (rather than straight up) is what keeps the fingers clear of
    neighbouring objects on a crowded table.
    """

    if standoff_m < 0.0:
        raise GraspGeometryError("standoff_m must be non-negative")
    matrix = _as_pose(pose).copy()
    matrix[:3, 3] = matrix[:3, 3] - standoff_m * matrix[:3, 2]
    return matrix


def transform_pose(transform, pose) -> np.ndarray:
    """Re-express a pose in another frame. Both must share units."""

    return _as_pose(transform) @ _as_pose(pose)


def scale_pose_translation(pose, factor: float) -> np.ndarray:
    """Scale only the translation, leaving the rotation alone.

    Used at the metre/millimetre boundary: a pose whose rotation got scaled
    would no longer be a rotation.
    """

    matrix = _as_pose(pose).copy()
    matrix[:3, 3] = matrix[:3, 3] * float(factor)
    return matrix


def posx_from_pose(pose_m) -> list:
    """Convert a base-frame pose in metres to a Doosan ``posx`` list.

    ``posx`` is [x, y, z, rx, ry, rz] with translation in **mm** and orientation
    as **ZYZ** Euler **degrees**. Both conversions happen here and nowhere else.
    """

    from scipy.spatial.transform import Rotation

    matrix = _as_pose(pose_m)
    rotation = matrix[:3, :3]
    error = np.abs(rotation @ rotation.T - np.eye(3)).max()
    if error > 1e-4:
        raise GraspGeometryError(
            f"pose rotation is not orthonormal (max error {error:.3e}); "
            "a scaled or skewed rotation cannot be sent to the robot"
        )
    angles = Rotation.from_matrix(rotation).as_euler("ZYZ", degrees=True)
    return [
        float(matrix[0, 3] * MM_PER_M),
        float(matrix[1, 3] * MM_PER_M),
        float(matrix[2, 3] * MM_PER_M),
        float(angles[0]),
        float(angles[1]),
        float(angles[2]),
    ]


def approach_tilt_degrees(pose) -> float:
    """Angle between the approach axis and straight down, in degrees.

    Zero means the gripper comes vertically down onto the table. This is the
    cheap stand-in for collision checking: without a motion planner, a grasp
    that reaches in sideways is the one most likely to drag the wrist through
    something on the way.
    """

    axis = approach_axis(pose)
    norm = float(np.linalg.norm(axis))
    if norm <= 0.0:
        raise GraspGeometryError("approach axis has zero length")
    downward = float(np.dot(axis / norm, np.array([0.0, 0.0, -1.0])))
    return float(np.degrees(np.arccos(np.clip(downward, -1.0, 1.0))))


def build_candidates(
    poses,
    confidences,
    fingertip_m: Sequence[float] = RG2_FINGERTIP_M,
) -> list:
    """Wrap raw GraspGenX output as candidates, highest confidence first."""

    pose_array = np.asarray(poses, dtype=np.float64)
    scores = np.asarray(confidences, dtype=np.float64).reshape(-1)
    if pose_array.ndim != 3 or pose_array.shape[1:] != (4, 4):
        raise GraspGeometryError(f"expected (N, 4, 4) poses, got {pose_array.shape}")
    if len(pose_array) != len(scores):
        raise GraspGeometryError(
            f"{len(pose_array)} poses but {len(scores)} confidences"
        )

    candidates = []
    for pose, score in zip(pose_array, scores):
        axis = approach_axis(pose)
        norm = float(np.linalg.norm(axis))
        if norm <= 0.0:
            continue
        candidates.append(
            GraspCandidate(
                pose=pose,
                confidence=float(score),
                contact_m=contact_point_m(pose, fingertip_m),
                approach=axis / norm,
            )
        )
    candidates.sort(key=lambda item: item.confidence, reverse=True)
    return candidates


def select_grasp(
    candidates,
    inside_workspace,
    max_tilt_degrees: float = 45.0,
    min_confidence: float = 0.0,
    standoff_m: float = 0.0,
) -> Optional[GraspCandidate]:
    """Pick the best grasp that is actually safe to attempt.

    ``inside_workspace`` is a predicate over a base-frame point in metres, so the
    robot's own bounds stay the single source of truth rather than being
    duplicated here.

    Both the contact point and the pregrasp point are checked. Checking only the
    contact would accept a grasp whose approach starts outside the safe box and
    flies in through it.
    """

    if max_tilt_degrees < 0.0:
        raise GraspGeometryError("max_tilt_degrees must be non-negative")

    for candidate in candidates:
        if candidate.confidence < min_confidence:
            continue
        if approach_tilt_degrees(candidate.pose) > max_tilt_degrees:
            continue
        if not inside_workspace(candidate.contact_m):
            continue
        if standoff_m > 0.0:
            start = candidate.contact_m - standoff_m * candidate.approach
            if not inside_workspace(start):
                continue
        return candidate
    return None
