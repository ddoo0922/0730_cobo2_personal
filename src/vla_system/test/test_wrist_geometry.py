"""The hand-eye chain that turns wrist pixels into base coordinates.

This is the arithmetic the arm acts on when it grasps, and every failure mode
here is silent: a wrong Euler convention, a transposed transform, or a
millimetre/metre slip all produce finite coordinates that simply point somewhere
else. So the tests pin the chain against independently computed values rather
than against the implementation's own output.
"""

import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from vla_system.perception.wrist_geometry import (
    CameraIntrinsics,
    MM_PER_M,
    WristCalibrationError,
    back_project,
    base_point_m,
    camera_to_base_mm,
    mask_to_camera_cloud,
    pose_matrix_from_posx,
    validate_rigid_transform,
)

# The real calibration on this robot, so the tests exercise the actual numbers.
REAL_CAMERA_TO_GRIPPER = np.array(
    [
        [-0.999824, 0.018784, 0.000342, 32.088573],
        [-0.018783, -0.999819, 0.003138, 77.766139],
        [0.000400, 0.003131, 0.999995, -221.771864],
        [0.0, 0.0, 0.0, 1.0],
    ]
)

INTRINSICS = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)


class RigidTransformValidationTest(unittest.TestCase):
    """A bad calibration must fail loudly; nothing downstream can catch it."""

    def test_the_real_calibration_is_accepted(self):
        matrix = validate_rigid_transform(REAL_CAMERA_TO_GRIPPER)
        self.assertEqual(matrix.shape, (4, 4))

    def test_identity_is_accepted(self):
        validate_rigid_transform(np.eye(4))

    def test_a_scaled_rotation_is_rejected(self):
        """The dangerous case: still finite, just wrong by a factor."""
        matrix = np.eye(4)
        matrix[:3, :3] *= 1.05
        with self.assertRaises(WristCalibrationError):
            validate_rigid_transform(matrix)

    def test_a_reflection_is_rejected(self):
        matrix = np.eye(4)
        matrix[0, 0] = -1.0  # det = -1, mirrors the scene
        with self.assertRaises(WristCalibrationError):
            validate_rigid_transform(matrix)

    def test_a_broken_bottom_row_is_rejected(self):
        matrix = np.eye(4)
        matrix[3] = [0.1, 0.0, 0.0, 1.0]
        with self.assertRaises(WristCalibrationError):
            validate_rigid_transform(matrix)

    def test_a_wrong_shape_is_rejected(self):
        with self.assertRaises(WristCalibrationError):
            validate_rigid_transform(np.eye(3))

    def test_nan_is_rejected(self):
        matrix = np.eye(4)
        matrix[0, 3] = np.nan
        with self.assertRaises(WristCalibrationError):
            validate_rigid_transform(matrix)


class PoseConventionTest(unittest.TestCase):
    """Doosan posx is ZYZ Euler degrees. Reading it as anything else is wrong
    everywhere except near zero, which is exactly where a smoke test looks."""

    def test_translation_lands_in_the_last_column(self):
        matrix = pose_matrix_from_posx([100.0, -200.0, 300.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(matrix[:3, 3], [100.0, -200.0, 300.0])

    def test_zero_orientation_is_the_identity_rotation(self):
        matrix = pose_matrix_from_posx([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(matrix[:3, :3], np.eye(3), atol=1e-12)

    def test_the_rotation_matches_scipy_zyz(self):
        angles = [30.0, 45.0, -60.0]
        matrix = pose_matrix_from_posx([0.0, 0.0, 0.0, *angles])
        expected = Rotation.from_euler("ZYZ", angles, degrees=True).as_matrix()
        np.testing.assert_allclose(matrix[:3, :3], expected, atol=1e-12)

    def test_zyz_is_distinguishable_from_xyz(self):
        """Guards the specific mistake of reading posx as XYZ."""
        angles = [30.0, 45.0, -60.0]
        zyz = pose_matrix_from_posx([0.0, 0.0, 0.0, *angles])[:3, :3]
        xyz = Rotation.from_euler("XYZ", angles, degrees=True).as_matrix()
        self.assertGreater(np.abs(zyz - xyz).max(), 0.1)

    def test_a_short_pose_is_rejected(self):
        with self.assertRaises(WristCalibrationError):
            pose_matrix_from_posx([1.0, 2.0, 3.0])

    def test_the_result_is_a_rigid_transform(self):
        validate_rigid_transform(
            pose_matrix_from_posx([10.0, 20.0, 30.0, 15.0, 25.0, 35.0])
        )


class BackProjectionTest(unittest.TestCase):
    def test_the_principal_point_projects_straight_out(self):
        point = back_project(320.0, 240.0, 500.0, INTRINSICS)
        np.testing.assert_allclose(point, [0.0, 0.0, 500.0])

    def test_an_offset_pixel_scales_with_depth(self):
        near = back_project(420.0, 240.0, 500.0, INTRINSICS)
        far = back_project(420.0, 240.0, 1000.0, INTRINSICS)
        # Same ray: doubling depth doubles the lateral offset.
        self.assertAlmostEqual(near[0], 100.0 * 500.0 / 600.0)
        self.assertAlmostEqual(far[0], 2.0 * near[0])

    def test_zero_depth_is_rejected(self):
        with self.assertRaises(WristCalibrationError):
            back_project(320.0, 240.0, 0.0, INTRINSICS)

    def test_degenerate_intrinsics_are_rejected(self):
        with self.assertRaises(WristCalibrationError):
            back_project(0.0, 0.0, 100.0, CameraIntrinsics(0.0, 600.0, 320.0, 240.0))


class HandEyeChainTest(unittest.TestCase):
    """p_base = T_gripper->base @ T_camera->gripper @ p_camera, per verify.py."""

    def test_the_chain_matches_an_independent_computation(self):
        tcp = [400.0, 100.0, 300.0, 20.0, 30.0, 40.0]
        camera_point = np.array([12.0, -34.0, 560.0])

        result = camera_to_base_mm(camera_point, REAL_CAMERA_TO_GRIPPER, tcp)

        base2gripper = pose_matrix_from_posx(tcp)
        expected = (base2gripper @ REAL_CAMERA_TO_GRIPPER @ np.append(camera_point, 1))[:3]
        np.testing.assert_allclose(result, expected, atol=1e-9)

    def test_the_order_of_multiplication_is_not_reversible(self):
        """Swapping the two transforms is the easiest way to get this wrong."""
        tcp = [400.0, 100.0, 300.0, 20.0, 30.0, 40.0]
        camera_point = np.array([12.0, -34.0, 560.0])
        correct = camera_to_base_mm(camera_point, REAL_CAMERA_TO_GRIPPER, tcp)
        swapped = (
            REAL_CAMERA_TO_GRIPPER
            @ pose_matrix_from_posx(tcp)
            @ np.append(camera_point, 1)
        )[:3]
        self.assertGreater(np.linalg.norm(correct - swapped), 1.0)

    def test_with_an_identity_calibration_the_camera_sits_at_the_tcp(self):
        tcp = [500.0, 0.0, 200.0, 0.0, 0.0, 0.0]
        result = camera_to_base_mm(np.zeros(3), np.eye(4), tcp)
        np.testing.assert_allclose(result, [500.0, 0.0, 200.0])

    def test_a_batch_of_points_keeps_its_shape(self):
        tcp = [400.0, 100.0, 300.0, 10.0, 20.0, 30.0]
        points = np.array([[0.0, 0.0, 500.0], [10.0, 10.0, 520.0], [-5.0, 3.0, 480.0]])
        batch = camera_to_base_mm(points, REAL_CAMERA_TO_GRIPPER, tcp)
        self.assertEqual(batch.shape, (3, 3))
        for index, point in enumerate(points):
            single = camera_to_base_mm(point, REAL_CAMERA_TO_GRIPPER, tcp)
            np.testing.assert_allclose(batch[index], single, atol=1e-9)

    def test_rigid_motion_preserves_distances(self):
        """A scale slip in the transform would show up here.

        Tolerance is a micron, not float precision: the constant above is the
        calibration printed to six decimals, so its rotation is orthonormal only
        to ~1e-6. That is still four orders of magnitude tighter than any scale
        error that would matter -- 1% on this 50 mm span would be 500 micron.
        """
        tcp = [350.0, -120.0, 410.0, 33.0, 44.0, 55.0]
        a = np.array([0.0, 0.0, 500.0])
        b = np.array([40.0, 30.0, 500.0])
        mapped = camera_to_base_mm(np.vstack([a, b]), REAL_CAMERA_TO_GRIPPER, tcp)
        self.assertAlmostEqual(
            float(np.linalg.norm(mapped[0] - mapped[1])),
            float(np.linalg.norm(a - b)),
            places=3,
        )

    def test_a_bad_calibration_is_rejected_in_the_chain(self):
        with self.assertRaises(WristCalibrationError):
            camera_to_base_mm(np.zeros(3), np.eye(4) * 2.0, [0.0] * 6)

    def test_a_wrongly_shaped_point_is_rejected(self):
        with self.assertRaises(WristCalibrationError):
            camera_to_base_mm(np.zeros((3, 2)), np.eye(4), [0.0] * 6)


class UnitBoundaryTest(unittest.TestCase):
    """mm inside, metres at the scene contract."""

    def test_millimetres_become_metres(self):
        self.assertEqual(base_point_m([500.0, -120.0, 35.0]), (0.5, -0.12, 0.035))

    def test_a_thousandfold_slip_would_be_caught(self):
        x, _y, _z = base_point_m([495.1, 16.0, 19.5])
        self.assertLess(abs(x), 2.0)
        self.assertGreater(abs(x), 0.05)


class MaskedCloudTest(unittest.TestCase):
    """GraspGenX is handed this cloud. Table pixels in it become table grasps."""

    def setUp(self):
        # 600 mm table with a 4x4 object 100 mm above it.
        self.depth = np.full((40, 40), 600, dtype=np.uint16)
        self.depth[10:14, 20:24] = 500
        self.mask = np.zeros((40, 40), dtype=bool)
        self.mask[10:14, 20:24] = True

    def test_only_masked_pixels_enter_the_cloud(self):
        cloud = mask_to_camera_cloud(self.depth, self.mask, INTRINSICS)
        self.assertEqual(cloud.size, 16)
        np.testing.assert_allclose(cloud.points_mm[:, 2], 500.0)

    def test_the_centroid_sits_on_the_object(self):
        cloud = mask_to_camera_cloud(self.depth, self.mask, INTRINSICS)
        self.assertAlmostEqual(float(cloud.centroid_mm[2]), 500.0)
        # Pixel centre of the 20..23 x 10..13 block, back-projected.
        expected_x = (21.5 - INTRINSICS.cx) * 500.0 / INTRINSICS.fx
        self.assertAlmostEqual(float(cloud.centroid_mm[0]), expected_x)

    def test_an_empty_mask_yields_nothing(self):
        self.assertIsNone(
            mask_to_camera_cloud(self.depth, np.zeros_like(self.mask), INTRINSICS)
        )

    def test_depth_holes_are_dropped_not_projected_as_zero(self):
        depth = self.depth.copy()
        depth[10:12, 20:24] = 0  # half the object has no depth return
        cloud = mask_to_camera_cloud(depth, self.mask, INTRINSICS)
        self.assertEqual(cloud.size, 8)
        np.testing.assert_allclose(cloud.points_mm[:, 2], 500.0)

    def test_an_all_hole_mask_yields_nothing(self):
        depth = np.zeros((40, 40), dtype=np.uint16)
        self.assertIsNone(mask_to_camera_cloud(depth, self.mask, INTRINSICS))

    def test_out_of_range_depth_is_excluded(self):
        depth = np.full((40, 40), 5000, dtype=np.uint16)
        self.assertIsNone(
            mask_to_camera_cloud(depth, self.mask, INTRINSICS, max_depth_mm=3000.0)
        )

    def test_a_metre_scaled_depth_image_is_handled(self):
        """32FC1 streams arrive in metres; the scale is the only difference."""
        depth = np.full((40, 40), 0.5, dtype=np.float32)
        cloud = mask_to_camera_cloud(
            depth, self.mask, INTRINSICS, depth_scale_mm=MM_PER_M
        )
        np.testing.assert_allclose(cloud.points_mm[:, 2], 500.0)

    def test_subsampling_is_deterministic_and_bounded(self):
        mask = np.ones((40, 40), dtype=bool)
        first = mask_to_camera_cloud(self.depth, mask, INTRINSICS, max_points=100)
        second = mask_to_camera_cloud(self.depth, mask, INTRINSICS, max_points=100)
        self.assertLessEqual(first.size, 100)
        np.testing.assert_array_equal(first.points_mm, second.points_mm)

    def test_a_mismatched_mask_is_rejected_not_broadcast(self):
        with self.assertRaises(WristCalibrationError):
            mask_to_camera_cloud(self.depth, np.ones((10, 10), dtype=bool), INTRINSICS)


if __name__ == "__main__":
    unittest.main()
