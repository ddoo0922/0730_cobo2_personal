"""Confirming the wrist is looking at the object the agent actually chose.

Getting this wrong grasps the wrong object, which in this system is the specific
failure the user guards against by saying "그 사과는 집지마". So the tests pin the
two decisions that matter: what counts as the same object, and when the answer is
too close to call.
"""

import unittest

from vla_system.perception.object_matching import (
    WristCandidate,
    horizontal_distance_m,
    match_target,
)


def candidate(handle, class_name, x, y, z=0.03):
    return WristCandidate(handle=handle, class_name=class_name, base_m=(x, y, z))


class HorizontalDistanceTest(unittest.TestCase):
    def test_height_is_ignored(self):
        self.assertAlmostEqual(
            horizontal_distance_m((0.5, 0.0, 0.0), (0.5, 0.0, 5.0)), 0.0
        )

    def test_it_is_the_table_plane_hypotenuse(self):
        self.assertAlmostEqual(
            horizontal_distance_m((0.0, 0.0, 0.0), (0.03, 0.04, 0.0)), 0.05
        )


class MatchingTest(unittest.TestCase):
    def test_the_same_object_seen_by_both_cameras_matches(self):
        result = match_target(
            "apple", (0.50, -0.10, 0.17), [candidate("apple_3", "apple", 0.505, -0.098)]
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.candidate.handle, "apple_3")
        self.assertLess(result.horizontal_distance_m, 0.01)
        self.assertFalse(result.ambiguous)

    def test_the_grasp_height_offset_does_not_break_the_match(self):
        """The regression this design exists to avoid: the webcam's Z is the
        table plane plus a 150 mm offset while the wrist measures the real
        surface, so a 3-D distance test would reject every correct match."""
        webcam_z = 0.02 + 0.15  # fitted table + grasp_height_offset_m
        wrist_z = 0.05  # actual object surface
        result = match_target(
            "banana", (0.42, 0.08, webcam_z), [candidate("banana_1", "banana", 0.421, 0.079, wrist_z)]
        )
        self.assertIsNotNone(result)
        self.assertGreater(result.vertical_gap_m, 0.10)

    def test_a_different_class_never_matches(self):
        self.assertIsNone(
            match_target("apple", (0.5, 0.0, 0.1), [candidate("cup_2", "cup", 0.5, 0.0)])
        )

    def test_something_too_far_away_does_not_match(self):
        self.assertIsNone(
            match_target(
                "apple",
                (0.50, 0.00, 0.1),
                [candidate("apple_9", "apple", 0.65, 0.00)],
                tolerance_m=0.06,
            )
        )

    def test_an_empty_view_does_not_match(self):
        self.assertIsNone(match_target("apple", (0.5, 0.0, 0.1), []))

    def test_the_nearest_of_several_same_class_objects_wins(self):
        result = match_target(
            "apple",
            (0.50, 0.00, 0.1),
            [
                candidate("apple_far", "apple", 0.545, 0.00),
                candidate("apple_near", "apple", 0.505, 0.00),
            ],
        )
        self.assertEqual(result.candidate.handle, "apple_near")
        self.assertAlmostEqual(result.runner_up_distance_m, 0.045, places=6)
        self.assertFalse(result.ambiguous)

    def test_two_nearly_equidistant_objects_are_flagged_ambiguous(self):
        """Choosing by a millimetre is a coin flip; the caller must be able to
        refuse rather than grasp the wrong apple."""
        result = match_target(
            "apple",
            (0.50, 0.00, 0.1),
            [
                candidate("apple_a", "apple", 0.512, 0.00),
                candidate("apple_b", "apple", 0.488, 0.00),
            ],
        )
        self.assertIsNotNone(result)
        self.assertTrue(result.ambiguous)

    def test_a_clear_winner_is_not_flagged_ambiguous(self):
        result = match_target(
            "apple",
            (0.50, 0.00, 0.1),
            [
                candidate("apple_a", "apple", 0.501, 0.00),
                candidate("apple_b", "apple", 0.550, 0.00),
            ],
            ambiguity_margin_m=0.02,
        )
        self.assertFalse(result.ambiguous)

    def test_a_single_match_has_no_runner_up(self):
        result = match_target(
            "apple", (0.5, 0.0, 0.1), [candidate("apple_1", "apple", 0.5, 0.0)]
        )
        self.assertIsNone(result.runner_up_distance_m)
        self.assertFalse(result.ambiguous)

    def test_something_on_a_different_shelf_is_excluded(self):
        """The loose vertical gate's only job."""
        self.assertIsNone(
            match_target(
                "cup",
                (0.5, 0.0, 0.05),
                [candidate("cup_1", "cup", 0.5, 0.0, 0.90)],
                max_vertical_gap_m=0.30,
            )
        )

    def test_only_matching_candidates_are_considered_for_ambiguity(self):
        """A nearby object of another class must not make the answer ambiguous."""
        result = match_target(
            "apple",
            (0.50, 0.00, 0.1),
            [
                candidate("apple_1", "apple", 0.505, 0.00),
                candidate("cup_1", "cup", 0.506, 0.00),
            ],
        )
        self.assertEqual(result.candidate.handle, "apple_1")
        self.assertIsNone(result.runner_up_distance_m)
        self.assertFalse(result.ambiguous)


class ParameterValidationTest(unittest.TestCase):
    def test_a_non_positive_tolerance_is_rejected(self):
        with self.assertRaises(ValueError):
            match_target("apple", (0.5, 0.0, 0.1), [], tolerance_m=0.0)

    def test_a_non_positive_vertical_gate_is_rejected(self):
        with self.assertRaises(ValueError):
            match_target("apple", (0.5, 0.0, 0.1), [], max_vertical_gap_m=-1.0)

    def test_a_negative_ambiguity_margin_is_rejected(self):
        with self.assertRaises(ValueError):
            match_target("apple", (0.5, 0.0, 0.1), [], ambiguity_margin_m=-0.01)


if __name__ == "__main__":
    unittest.main()
