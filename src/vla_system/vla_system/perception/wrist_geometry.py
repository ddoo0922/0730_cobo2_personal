"""Wrist-camera geometry: pixels and depth to robot-base coordinates.

The RealSense is mounted on the wrist, so unlike the fixed webcam it has no
single standing calibration to the table. What it has instead is a hand-eye
calibration -- the rigid transform from the camera frame to the gripper frame --
which stays true no matter where the arm is. The arm's own reported pose supplies
the rest of the chain:

    p_base = T_gripper->base(current TCP pose) @ T_camera->gripper @ p_camera

``T_camera->gripper`` is what ``cv2.calibrateHandEye`` returns as
``R/t_cam2gripper``; it is stored as ``config/T_gripper2camera.npy``. The file's
name reads the other way round, but its contents map camera coordinates into the
gripper frame, which is what ``verify.py`` in the calibration tutorial relies on
when it computes ``base2gripper @ gripper2cam``.

Units
-----
Everything in this module is **millimetres and degrees**, matching the
calibration file, the checkerboard that produced it, and the Doosan ``posx``
API. The scene contract is metres, so ``base_point_m`` is the single conversion
boundary -- the same arrangement ``table_homography`` uses.

Orientation
-----------
Doosan ``posx`` carries orientation as **ZYZ** Euler angles in degrees. Reading
them as anything else (XYZ, or radians) produces a transform that looks
plausible and is wrong everywhere except near zero, so the convention is applied
in exactly one function here.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

MM_PER_M = 1000.0

# How far a rotation block may stray from orthonormal before the calibration is
# treated as corrupt. cv2.calibrateHandEye output lands within ~1e-15.
ROTATION_TOLERANCE = 1e-6


class WristCalibrationError(ValueError):
    """The hand-eye calibration is missing, malformed, or not a rigid motion."""


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics for the stream being sampled, in pixels."""

    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_camera_info(cls, message) -> "CameraIntrinsics":
        k = message.k
        intrinsics = cls(float(k[0]), float(k[4]), float(k[2]), float(k[5]))
        intrinsics.validate()
        return intrinsics

    def validate(self) -> None:
        if not (self.fx > 0.0 and self.fy > 0.0):
            raise WristCalibrationError("fx and fy must be greater than zero")


def validate_rigid_transform(matrix: np.ndarray) -> np.ndarray:
    """Check a 4x4 is a rotation plus a translation, and return it as float64.

    A hand-eye file is the one input that can be silently wrong in a way no
    later stage can detect: a scaled or skewed rotation still produces finite
    coordinates, just ones that put the gripper somewhere else. Reject it here.
    """

    array = np.asarray(matrix, dtype=np.float64)
    if array.shape != (4, 4):
        raise WristCalibrationError(f"expected a 4x4 transform, got {array.shape}")
    if not np.isfinite(array).all():
        raise WristCalibrationError("transform contains NaN or inf")

    rotation = array[:3, :3]
    error = np.abs(rotation @ rotation.T - np.eye(3)).max()
    if error > ROTATION_TOLERANCE:
        raise WristCalibrationError(
            f"rotation block is not orthonormal (max error {error:.3e})"
        )
    determinant = float(np.linalg.det(rotation))
    if abs(determinant - 1.0) > ROTATION_TOLERANCE:
        raise WristCalibrationError(
            f"rotation determinant is {determinant:.6f}, expected 1 "
            "(a reflection, not a rotation)"
        )
    if not np.allclose(array[3], [0.0, 0.0, 0.0, 1.0], atol=ROTATION_TOLERANCE):
        raise WristCalibrationError(f"bottom row is {array[3]}, expected [0 0 0 1]")
    return array


def load_camera_to_gripper(path) -> np.ndarray:
    """Load the hand-eye transform that maps camera coordinates into the gripper."""

    resolved = Path(path).expanduser()
    matrix = np.load(str(resolved))
    try:
        return validate_rigid_transform(matrix)
    except WristCalibrationError as exc:
        raise WristCalibrationError(f"{resolved}: {exc}") from exc


def pose_matrix_from_posx(pose: Sequence[float]) -> np.ndarray:
    """Build T_gripper->base from a Doosan ``posx`` [x, y, z, rx, ry, rz].

    Translation in mm, orientation as ZYZ Euler degrees.
    """

    values = [float(v) for v in pose]
    if len(values) < 6:
        raise WristCalibrationError("posx must have six components")
    if not np.isfinite(values).all():
        raise WristCalibrationError("posx contains NaN or inf")

    from scipy.spatial.transform import Rotation

    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_euler(
        "ZYZ", values[3:6], degrees=True
    ).as_matrix()
    matrix[:3, 3] = values[0:3]
    return matrix


def back_project(
    pixel_x: float,
    pixel_y: float,
    depth_mm: float,
    intrinsics: CameraIntrinsics,
) -> np.ndarray:
    """Project one pixel with known depth into the camera frame, in mm."""

    if not depth_mm > 0.0:
        raise WristCalibrationError("depth must be greater than zero")
    intrinsics.validate()
    return np.array(
        [
            (float(pixel_x) - intrinsics.cx) * float(depth_mm) / intrinsics.fx,
            (float(pixel_y) - intrinsics.cy) * float(depth_mm) / intrinsics.fy,
            float(depth_mm),
        ],
        dtype=np.float64,
    )


def camera_to_base_mm(
    camera_mm, camera_to_gripper: np.ndarray, tcp_pose: Sequence[float]
) -> np.ndarray:
    """Map camera-frame points to base-frame points, both in mm.

    Accepts a single (3,) point or an (N, 3) array and returns the same shape.
    """

    points = np.asarray(camera_mm, dtype=np.float64)
    single = points.ndim == 1
    if single:
        points = points.reshape(1, 3)
    if points.ndim != 2 or points.shape[1] != 3:
        raise WristCalibrationError("camera points must be (3,) or (N, 3)")

    transform = pose_matrix_from_posx(tcp_pose) @ validate_rigid_transform(
        camera_to_gripper
    )
    homogeneous = np.hstack([points, np.ones((len(points), 1))])
    mapped = (transform @ homogeneous.T).T[:, :3]
    return mapped[0] if single else mapped


def base_point_m(base_mm) -> tuple:
    """The one millimetre-to-metre boundary, matching the scene contract."""

    point = np.asarray(base_mm, dtype=np.float64).reshape(3)
    return tuple(float(v) / MM_PER_M for v in point)


@dataclass(frozen=True)
class MaskedCloud:
    """An object's own pixels, lifted into 3-D.

    Attributes
    ----------
    points_mm:
        (N, 3) in the camera frame. GraspGenX wants a partial point cloud of the
        segmented target, which is exactly this.
    centroid_mm:
        Mean of ``points_mm``, used to decide object identity against the fixed
        webcam's position for the same object.
    """

    points_mm: np.ndarray
    centroid_mm: np.ndarray

    @property
    def size(self) -> int:
        return int(len(self.points_mm))


def mask_to_camera_cloud(
    depth_image: np.ndarray,
    mask: np.ndarray,
    intrinsics: CameraIntrinsics,
    depth_scale_mm: float = 1.0,
    min_depth_mm: float = 100.0,
    max_depth_mm: float = 3000.0,
    max_points: int = 20000,
) -> Optional[MaskedCloud]:
    """Lift the pixels inside an instance mask into a camera-frame point cloud.

    Only masked pixels contribute, so the tabletop behind and beside the object
    never enters the cloud. That matters more here than it did for the median
    depth: a grasp model handed a cloud with the table in it will happily propose
    grasps on the table.

    ``max_points`` subsamples deterministically (an even stride, not a random
    draw) so repeated calls on the same frame give the same cloud and a failing
    grasp can be reproduced.
    """

    if depth_image.ndim != 2:
        raise WristCalibrationError("depth_image must be two-dimensional")
    if mask.shape != depth_image.shape:
        raise WristCalibrationError(
            f"mask shape {mask.shape} does not match depth shape {depth_image.shape}"
        )
    if not depth_scale_mm > 0.0:
        raise WristCalibrationError("depth_scale_mm must be greater than zero")
    if not 0.0 <= min_depth_mm < max_depth_mm:
        raise WristCalibrationError("invalid depth range")
    if max_points < 1:
        raise WristCalibrationError("max_points must be at least one")
    intrinsics.validate()

    rows, columns = np.nonzero(mask)
    if rows.size == 0:
        return None

    depths = depth_image[rows, columns].astype(np.float64) * float(depth_scale_mm)
    keep = np.isfinite(depths) & (depths >= min_depth_mm) & (depths <= max_depth_mm)
    if not keep.any():
        return None
    rows, columns, depths = rows[keep], columns[keep], depths[keep]

    if rows.size > max_points:
        stride = int(np.ceil(rows.size / max_points))
        rows, columns, depths = rows[::stride], columns[::stride], depths[::stride]

    points = np.column_stack(
        [
            (columns.astype(np.float64) - intrinsics.cx) * depths / intrinsics.fx,
            (rows.astype(np.float64) - intrinsics.cy) * depths / intrinsics.fy,
            depths,
        ]
    )
    return MaskedCloud(points_mm=points, centroid_mm=points.mean(axis=0))
