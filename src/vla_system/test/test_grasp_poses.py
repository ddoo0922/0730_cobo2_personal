"""GraspGenX pose conventions, pinned by measurement rather than assumption.

The two facts encoded here were established by running the released
``onrobot_RG2`` model and checking distances to the object point cloud:

- the returned pose is the gripper **base**, 159 mm off the surface, while
  ``origin + R @ fingertip`` sits 4 mm off it;
- the approach axis is the rotation's **third column** (local +Z), agreeing with
  the fingertip direction to 1.000000.

If either is applied wrongly the arm misses by the 180 mm fingertip offset, and
nothing downstream can tell that from a bad calibration. Hence these tests.
"""

import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from vla_system.grasp.poses import (
    MM_PER_M,
    RG2_FINGERTIP_M,
    GraspGeometryError,
    approach_axis,
    approach_tilt_degrees,
    build_candidates,
    contact_point_m,
    posx_from_pose,
    pregrasp_pose,
    scale_pose_translation,
    select_grasp,
    transform_pose,
)


def pose_from(rotation: np.ndarray, translation) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def straight_down_pose(translation=(0.5, 0.0, 0.3)) -> np.ndarray:
    """Gripper +Z pointing at the table, i.e. a top-down grasp."""
    rotation = Rotation.from_euler("Y", 180.0, degrees=True).as_matrix()
    return pose_from(rotation, translation)


class ContactPointTest(unittest.TestCase):
    def test_the_fingertip_offset_is_applied_along_local_z(self):
        pose = straight_down_pose((0.5, 0.0, 0.30))
        contact = contact_point_m(pose, RG2_FINGERTIP_M)
        # +Z points down, so the contact is 180 mm *below* the pose origin.
        np.testing.assert_allclose(contact, [0.5, 0.0, 0.30 - 0.18], atol=1e-9)

    def test_the_offset_rotates_with_the_pose(self):
        """A sideways grasp must offset sideways, not downward."""
        rotation = Rotation.from_euler("Y", 90.0, degrees=True).as_matrix()
        pose = pose_from(rotation, (0.4, 0.0, 0.2))
        contact = contact_point_m(pose, RG2_FINGERTIP_M)
        np.testing.assert_allclose(contact, [0.4 + 0.18, 0.0, 0.2], atol=1e-9)

    def test_a_zero_offset_leaves_the_origin(self):
        pose = straight_down_pose()
        np.testing.assert_allclose(contact_point_m(pose, (0, 0, 0)), pose[:3, 3])

    def test_the_offset_magnitude_is_the_rg2_value(self):
        pose = straight_down_pose()
        moved = np.linalg.norm(contact_point_m(pose) - pose[:3, 3])
        self.assertAlmostEqual(float(moved), 0.18, places=9)

    def test_a_bad_shape_is_rejected(self):
        with self.assertRaises(GraspGeometryError):
            contact_point_m(np.eye(3))

    def test_nan_is_rejected(self):
        pose = straight_down_pose()
        pose[0, 3] = np.nan
        with self.assertRaises(GraspGeometryError):
            contact_point_m(pose)


class ApproachAxisTest(unittest.TestCase):
    def test_the_axis_is_the_third_column(self):
        pose = straight_down_pose()
        np.testing.assert_allclose(approach_axis(pose), pose[:3, 2])

    def test_a_top_down_grasp_has_zero_tilt(self):
        self.assertAlmostEqual(approach_tilt_degrees(straight_down_pose()), 0.0, places=6)

    def test_a_horizontal_grasp_is_ninety_degrees(self):
        rotation = Rotation.from_euler("Y", 90.0, degrees=True).as_matrix()
        self.assertAlmostEqual(
            approach_tilt_degrees(pose_from(rotation, (0.4, 0, 0.2))), 90.0, places=6
        )

    def test_an_upward_grasp_is_beyond_ninety(self):
        """Reaching up from underneath: must be rejectable by the tilt gate."""
        self.assertAlmostEqual(approach_tilt_degrees(np.eye(4)), 180.0, places=6)

    def test_a_forty_five_degree_grasp_measures_forty_five(self):
        rotation = Rotation.from_euler("Y", 135.0, degrees=True).as_matrix()
        self.assertAlmostEqual(
            approach_tilt_degrees(pose_from(rotation, (0.4, 0, 0.2))), 45.0, places=6
        )


class PregraspTest(unittest.TestCase):
    def test_the_pregrasp_backs_off_along_the_approach_axis(self):
        pose = straight_down_pose((0.5, 0.0, 0.30))
        backed = pregrasp_pose(pose, 0.04)
        # +Z is down, so backing off raises the gripper.
        np.testing.assert_allclose(backed[:3, 3], [0.5, 0.0, 0.34], atol=1e-9)

    def test_the_rotation_is_untouched(self):
        pose = straight_down_pose()
        np.testing.assert_allclose(pregrasp_pose(pose, 0.04)[:3, :3], pose[:3, :3])

    def test_a_sideways_grasp_backs_off_sideways(self):
        rotation = Rotation.from_euler("Y", 90.0, degrees=True).as_matrix()
        backed = pregrasp_pose(pose_from(rotation, (0.4, 0.0, 0.2)), 0.05)
        np.testing.assert_allclose(backed[:3, 3], [0.35, 0.0, 0.2], atol=1e-9)

    def test_zero_standoff_is_a_no_op(self):
        pose = straight_down_pose()
        np.testing.assert_allclose(pregrasp_pose(pose, 0.0), pose)

    def test_a_negative_standoff_is_rejected(self):
        with self.assertRaises(GraspGeometryError):
            pregrasp_pose(straight_down_pose(), -0.01)


class UnitBoundaryTest(unittest.TestCase):
    """GraspGenX is metres; posx is millimetres and ZYZ degrees."""

    def test_translation_is_converted_to_millimetres(self):
        posx = posx_from_pose(straight_down_pose((0.4237, -0.0587, 0.1052)))
        self.assertAlmostEqual(posx[0], 423.7, places=6)
        self.assertAlmostEqual(posx[1], -58.7, places=6)
        self.assertAlmostEqual(posx[2], 105.2, places=6)

    def test_orientation_round_trips_through_zyz(self):
        angles = [37.0, 52.0, -71.0]
        rotation = Rotation.from_euler("ZYZ", angles, degrees=True).as_matrix()
        posx = posx_from_pose(pose_from(rotation, (0.5, 0.0, 0.2)))
        recovered = Rotation.from_euler("ZYZ", posx[3:6], degrees=True).as_matrix()
        np.testing.assert_allclose(recovered, rotation, atol=1e-9)

    def test_a_top_down_grasp_survives_zyz_gimbal_lock(self):
        """The common case sits exactly on the ZYZ singularity.

        A top-down grasp has a middle angle of 180 degrees, where ZYZ cannot
        separate the first and third angles -- scipy warns and folds them
        together. That makes the decomposition non-unique, but not wrong: the
        angles still rebuild the same rotation, so the orientation commanded to
        the robot is correct. Pinned because "the most common grasp orientation
        is degenerate" is alarming enough to be worth an explicit answer.
        """
        pose = straight_down_pose()
        with np.errstate(all="ignore"):
            posx = posx_from_pose(pose)
        rebuilt = Rotation.from_euler("ZYZ", posx[3:6], degrees=True).as_matrix()
        np.testing.assert_allclose(rebuilt, pose[:3, :3], atol=1e-9)
        # And the approach axis, which is what actually aims the gripper.
        np.testing.assert_allclose(rebuilt[:, 2], [0.0, 0.0, -1.0], atol=1e-9)

    def test_a_near_top_down_grasp_is_unambiguous(self):
        """A few degrees off vertical leaves the singularity behind entirely."""
        rotation = Rotation.from_euler("ZYZ", [10.0, 175.0, 20.0], degrees=True).as_matrix()
        posx = posx_from_pose(pose_from(rotation, (0.5, 0.0, 0.3)))
        rebuilt = Rotation.from_euler("ZYZ", posx[3:6], degrees=True).as_matrix()
        np.testing.assert_allclose(rebuilt, rotation, atol=1e-9)

    def test_a_scaled_rotation_is_refused_rather_than_commanded(self):
        pose = straight_down_pose()
        pose[:3, :3] *= 1.02
        with self.assertRaises(GraspGeometryError):
            posx_from_pose(pose)

    def test_only_the_translation_is_scaled(self):
        pose = straight_down_pose((0.5, 0.1, 0.2))
        scaled = scale_pose_translation(pose, MM_PER_M)
        np.testing.assert_allclose(scaled[:3, 3], [500.0, 100.0, 200.0])
        np.testing.assert_allclose(scaled[:3, :3], pose[:3, :3])

    def test_a_scaled_pose_is_still_a_valid_rotation(self):
        posx_from_pose(scale_pose_translation(straight_down_pose(), 1.0))


class TransformPoseTest(unittest.TestCase):
    def test_a_translation_moves_the_pose(self):
        transform = np.eye(4)
        transform[:3, 3] = [1.0, 2.0, 3.0]
        moved = transform_pose(transform, straight_down_pose((0.5, 0.0, 0.3)))
        np.testing.assert_allclose(moved[:3, 3], [1.5, 2.0, 3.3])

    def test_the_composition_order_is_frame_then_pose(self):
        rotation = Rotation.from_euler("Z", 90.0, degrees=True).as_matrix()
        transform = pose_from(rotation, (0.0, 0.0, 0.0))
        moved = transform_pose(transform, pose_from(np.eye(3), (1.0, 0.0, 0.0)))
        np.testing.assert_allclose(moved[:3, 3], [0.0, 1.0, 0.0], atol=1e-9)


class CandidateBuildingTest(unittest.TestCase):
    def test_candidates_come_back_sorted_by_confidence(self):
        poses = np.stack([straight_down_pose(), straight_down_pose(), straight_down_pose()])
        candidates = build_candidates(poses, [0.3, 0.9, 0.6])
        self.assertEqual([round(c.confidence, 1) for c in candidates], [0.9, 0.6, 0.3])

    def test_the_approach_is_normalised(self):
        candidates = build_candidates(np.stack([straight_down_pose()]), [0.5])
        self.assertAlmostEqual(float(np.linalg.norm(candidates[0].approach)), 1.0)

    def test_the_contact_point_is_precomputed(self):
        pose = straight_down_pose((0.5, 0.0, 0.30))
        candidate = build_candidates(np.stack([pose]), [0.5])[0]
        np.testing.assert_allclose(candidate.contact_m, [0.5, 0.0, 0.12], atol=1e-9)

    def test_mismatched_lengths_are_rejected(self):
        with self.assertRaises(GraspGeometryError):
            build_candidates(np.stack([straight_down_pose()]), [0.1, 0.2])

    def test_a_wrong_shape_is_rejected(self):
        with self.assertRaises(GraspGeometryError):
            build_candidates(np.zeros((3, 3, 3)), [0.1, 0.2, 0.3])

    def test_an_empty_batch_gives_no_candidates(self):
        self.assertEqual(build_candidates(np.zeros((0, 4, 4)), []), [])


class SelectionTest(unittest.TestCase):
    """Without a motion planner these gates are the only thing between a
    generated pose and the arm attempting it."""

    def setUp(self):
        self.inside_all = lambda point: True
        self.inside_none = lambda point: False

    def test_the_highest_confidence_safe_grasp_wins(self):
        poses = np.stack([straight_down_pose((0.5, 0, 0.3)), straight_down_pose((0.4, 0, 0.3))])
        chosen = select_grasp(build_candidates(poses, [0.4, 0.8]), self.inside_all)
        np.testing.assert_allclose(chosen.pose[:3, 3], [0.4, 0, 0.3])

    def test_a_sideways_grasp_is_rejected_by_the_tilt_gate(self):
        rotation = Rotation.from_euler("Y", 90.0, degrees=True).as_matrix()
        candidates = build_candidates(np.stack([pose_from(rotation, (0.4, 0, 0.2))]), [0.9])
        self.assertIsNone(select_grasp(candidates, self.inside_all, max_tilt_degrees=45.0))

    def test_a_tilted_grasp_within_the_gate_is_accepted(self):
        rotation = Rotation.from_euler("Y", 160.0, degrees=True).as_matrix()
        candidates = build_candidates(np.stack([pose_from(rotation, (0.4, 0, 0.3))]), [0.9])
        self.assertIsNotNone(
            select_grasp(candidates, self.inside_all, max_tilt_degrees=45.0)
        )

    def test_a_contact_outside_the_workspace_is_rejected(self):
        candidates = build_candidates(np.stack([straight_down_pose()]), [0.9])
        self.assertIsNone(select_grasp(candidates, self.inside_none))

    def test_a_pregrasp_outside_the_workspace_is_rejected(self):
        """Checking only the contact point would accept a grasp that flies in
        from outside the safe box."""
        candidates = build_candidates(np.stack([straight_down_pose((0.5, 0.0, 0.30))]), [0.9])

        def contact_only(point):
            # Contact at z=0.12 is fine; the pregrasp at z=0.32 is not.
            return float(point[2]) < 0.2

        self.assertIsNotNone(select_grasp(candidates, contact_only, standoff_m=0.0))
        self.assertIsNone(select_grasp(candidates, contact_only, standoff_m=0.2))

    def test_low_confidence_grasps_are_skipped(self):
        candidates = build_candidates(np.stack([straight_down_pose()]), [0.2])
        self.assertIsNone(
            select_grasp(candidates, self.inside_all, min_confidence=0.5)
        )

    def test_selection_falls_through_to_a_lower_scored_but_safe_grasp(self):
        sideways = Rotation.from_euler("Y", 90.0, degrees=True).as_matrix()
        poses = np.stack(
            [pose_from(sideways, (0.4, 0, 0.2)), straight_down_pose((0.45, 0, 0.3))]
        )
        chosen = select_grasp(
            build_candidates(poses, [0.95, 0.55]), self.inside_all, max_tilt_degrees=45.0
        )
        self.assertAlmostEqual(chosen.confidence, 0.55)

    def test_no_candidates_gives_none_rather_than_raising(self):
        self.assertIsNone(select_grasp([], self.inside_all))


if __name__ == "__main__":
    unittest.main()
