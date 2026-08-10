"""Planar webcam-to-robot calibration helpers.

The webcam sees a planar work surface. Four corresponding points are enough to
estimate a projective transform from image pixels (u, v) to robot-base table
coordinates (X, Y). The robot-recorded TCP points are also used to fit the table
height as Z = aX + bY + c, so a click can be turned into a 3-D approach target.

All public geometry functions in this module use **millimetres** for robot
coordinates and pixels for image coordinates. Keeping the robot side in mm
matches the Doosan Python API and avoids silent metre/mm mistakes.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

import cv2
import numpy as np

MM_PER_M = 1000.0


class CalibrationError(ValueError):
    """Raised when the four calibration correspondences are degenerate."""


@dataclass(frozen=True)
class TableCalibration:
    """Pixel-to-base planar calibration.

    Attributes
    ----------
    homography:
        3x3 matrix mapping [u, v, 1] to robot-base [X_mm, Y_mm, 1].
    plane_z_coefficients:
        [a, b, c] in Z_mm = a*X_mm + b*Y_mm + c.
    pixel_points:
        Four webcam points in the same P1..P4 order used on the robot.
    robot_points_mm:
        Four robot TCP contact points [X, Y, Z] in base coordinates.
    """

    homography: np.ndarray
    plane_z_coefficients: np.ndarray
    pixel_points: np.ndarray
    robot_points_mm: np.ndarray

    def pixel_to_base_xy_mm(self, u: float, v: float) -> tuple[float, float]:
        point = np.asarray([[[float(u), float(v)]]], dtype=np.float64)
        mapped = cv2.perspectiveTransform(point, self.homography)[0, 0]
        x_mm, y_mm = float(mapped[0]), float(mapped[1])
        if not np.isfinite([x_mm, y_mm]).all():
            raise CalibrationError("homography produced a non-finite XY coordinate")
        return x_mm, y_mm

    def table_z_mm(self, x_mm: float, y_mm: float) -> float:
        a, b, c = (float(v) for v in self.plane_z_coefficients)
        z_mm = a * float(x_mm) + b * float(y_mm) + c
        if not np.isfinite(z_mm):
            raise CalibrationError("table plane produced a non-finite Z coordinate")
        return float(z_mm)

    def approach_target_mm(
        self, u: float, v: float, z_offset_mm: float
    ) -> tuple[float, float, float]:
        x_mm, y_mm = self.pixel_to_base_xy_mm(u, v)
        z_mm = self.table_z_mm(x_mm, y_mm) + float(z_offset_mm)
        return x_mm, y_mm, z_mm


def _as_points(values: Iterable[Sequence[float]], columns: int, name: str) -> np.ndarray:
    array = np.asarray(list(values), dtype=np.float64)
    if array.shape != (4, columns):
        raise CalibrationError(f"{name} must contain exactly four {columns}D points")
    if not np.isfinite(array).all():
        raise CalibrationError(f"{name} contains NaN or inf")
    return array


def _polygon_area(points_xy: np.ndarray) -> float:
    x = points_xy[:, 0]
    y = points_xy[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def build_table_calibration(
    pixel_points: Iterable[Sequence[float]],
    robot_points_mm: Iterable[Sequence[float]],
) -> TableCalibration:
    """Build calibration from four P1..P4 point correspondences.

    P1..P4 should be entered around the table perimeter in the same clockwise or
    counter-clockwise order for both modalities. This makes the calibrated
    polygon useful as a safety boundary in addition to defining the homography.
    """

    pixels = _as_points(pixel_points, 2, "pixel_points")
    robot = _as_points(robot_points_mm, 3, "robot_points_mm")

    if _polygon_area(pixels) < 100.0:
        raise CalibrationError("webcam points are degenerate or cover too little image area")
    if _polygon_area(robot[:, :2]) < 100.0:
        raise CalibrationError("robot XY points are degenerate or cover too little table area")

    # Exactly four correspondences: getPerspectiveTransform gives the unique
    # planar projective transform without RANSAC silently dropping a point.
    homography = cv2.getPerspectiveTransform(
        pixels.astype(np.float32), robot[:, :2].astype(np.float32)
    ).astype(np.float64)

    if not np.isfinite(homography).all() or abs(float(np.linalg.det(homography))) < 1e-12:
        raise CalibrationError("homography is singular")

    # Fit a mildly tilted tabletop as Z = aX + bY + c. Four points give one
    # redundant observation, so least squares smooths small TCP-touch errors.
    design = np.column_stack([robot[:, 0], robot[:, 1], np.ones(4)])
    coeffs, _, rank, _ = np.linalg.lstsq(design, robot[:, 2], rcond=None)
    if rank < 3:
        raise CalibrationError("robot points cannot define a table plane")

    return TableCalibration(
        homography=homography,
        plane_z_coefficients=coeffs.astype(np.float64),
        pixel_points=pixels,
        robot_points_mm=robot,
    )


def reprojection_errors_mm(calibration: TableCalibration) -> np.ndarray:
    """Return XY reprojection error at each of the four calibration points."""

    src = calibration.pixel_points.reshape(-1, 1, 2).astype(np.float64)
    mapped = cv2.perspectiveTransform(src, calibration.homography).reshape(-1, 2)
    return np.linalg.norm(mapped - calibration.robot_points_mm[:, :2], axis=1)


def point_inside_calibrated_polygon(
    calibration: TableCalibration, u: float, v: float
) -> bool:
    contour = calibration.pixel_points.astype(np.float32).reshape(-1, 1, 2)
    return cv2.pointPolygonTest(contour, (float(u), float(v)), False) >= 0


def load_table_calibration(path, expected_image_size=None) -> TableCalibration:
    """Rebuild a calibration from the JSON the click tool writes.

    Only the four point correspondences are read back, and the homography is
    recomputed from them rather than trusted from the file. The stored matrix is
    derived data: recomputing keeps one code path for it and makes a
    hand-edited or truncated file fail loudly here instead of quietly aiming the
    arm somewhere else.

    ``expected_image_size`` is the (width, height) the *consumer* is capturing
    at. The correspondences are raw pixel coordinates, so they are only
    meaningful at the resolution they were clicked at -- open the same webcam at
    640x480 instead of 1280x720 and every mapped coordinate silently moves. When
    the file records its resolution this refuses the mismatch outright; older
    files without it cannot be checked, hence ``calibration_image_size``.
    """

    payload = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    try:
        pixel_points = payload["pixel_points"]
        robot_points_mm = payload["robot_points_mm"]
    except (KeyError, TypeError) as exc:
        raise CalibrationError(f"calibration file is missing {exc}") from exc

    stored = calibration_image_size(payload)
    if expected_image_size is not None and stored is not None:
        if tuple(int(v) for v in expected_image_size) != stored:
            raise CalibrationError(
                f"calibration was measured at {stored[0]}x{stored[1]} but the "
                f"camera is open at {expected_image_size[0]}x"
                f"{expected_image_size[1]}; pixel coordinates would not line up"
            )
    return build_table_calibration(pixel_points, robot_points_mm)


def calibration_image_size(payload) -> Optional[tuple]:
    """The (width, height) a calibration was clicked at, if it recorded one."""

    size = payload.get("image_size") if isinstance(payload, dict) else None
    if not isinstance(size, (list, tuple)) or len(size) != 2:
        return None
    try:
        return int(size[0]), int(size[1])
    except (TypeError, ValueError):
        return None


@dataclass(frozen=True)
class TablePoint:
    """One image point resolved onto the calibrated table, in metres.

    Metres, not millimetres: this is the boundary where the calibration's mm
    convention meets the scene contract, which is metres everywhere
    (`SceneObject.position_base`, the robot's workspace bounds). Converting here
    means exactly one place can get the factor wrong.
    """

    x_m: float
    y_m: float
    z_m: float
    inside_table: bool


def table_point_from_pixel(
    calibration: TableCalibration,
    u: float,
    v: float,
    grasp_height_offset_m: float = 0.0,
) -> Optional[TablePoint]:
    """Map an image pixel to a base-frame point in metres.

    ``grasp_height_offset_m`` is added to the fitted tabletop height. It exists
    because the homography can only ever report where the *table* is under a
    pixel, while the robot closes its gripper exactly at the Z it is handed --
    so a bare table Z would drive the fingers into the tabletop. The offset is
    a blunt stand-in for object height and cannot be derived from a single
    overhead view.
    """

    try:
        x_mm, y_mm = calibration.pixel_to_base_xy_mm(u, v)
        z_mm = calibration.table_z_mm(x_mm, y_mm)
    except CalibrationError:
        return None
    return TablePoint(
        x_m=x_mm / MM_PER_M,
        y_m=y_mm / MM_PER_M,
        z_m=z_mm / MM_PER_M + float(grasp_height_offset_m),
        inside_table=point_inside_calibrated_polygon(calibration, u, v),
    )
