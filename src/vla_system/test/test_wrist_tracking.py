"""Continuous wrist-search synchronization without ROS or a GPU."""

import unittest

from vla_system.perception.wrist_tracking import (
    MatchStability,
    TimedSample,
    result_matches_session,
    select_synced_observation,
)


def sample(milliseconds, value):
    return TimedSample(int(milliseconds * 1_000_000), value)


class SyncedObservationTest(unittest.TestCase):
    def select(self, colors, depths, poses, **overrides):
        options = dict(
            after_ns=0,
            last_color_ns=0,
            rgbd_tolerance_ns=20_000_000,
            pose_tolerance_ns=120_000_000,
        )
        options.update(overrides)
        return select_synced_observation(colors, depths, poses, **options)

    def test_selects_the_newest_compatible_triplet(self):
        result = self.select(
            [sample(100, "old rgb"), sample(200, "new rgb")],
            [sample(101, "old depth"), sample(203, "new depth")],
            [sample(95, "old pose"), sample(210, "new pose")],
        )
        self.assertEqual(result.color.value, "new rgb")
        self.assertEqual(result.depth.value, "new depth")
        self.assertEqual(result.pose.value, "new pose")

    def test_never_reuses_a_frame_from_before_the_request(self):
        result = self.select(
            [sample(100, "rgb")],
            [sample(100, "depth")],
            [sample(100, "pose")],
            after_ns=100_000_000,
        )
        self.assertIsNone(result)

    def test_every_member_respects_the_request_freshness_barrier(self):
        result = self.select(
            [sample(110, "rgb")],
            [sample(99, "pre-request depth")],
            [sample(100, "initial pose"), sample(120, "future pose")],
            after_ns=100_000_000,
        )
        self.assertIsNone(result)

    def test_depth_must_be_new_but_initial_pose_at_barrier_is_allowed(self):
        result = self.select(
            [sample(110, "rgb")],
            [sample(100, "cutoff depth"), sample(101, "fresh depth")],
            [sample(100, "initial pose"), sample(120, "future pose")],
            after_ns=100_000_000,
        )
        self.assertEqual(result.depth.value, "fresh depth")
        self.assertEqual(result.pose.value, "initial pose")

    def test_never_processes_the_same_color_twice(self):
        result = self.select(
            [sample(100, "rgb")],
            [sample(100, "depth")],
            [sample(100, "pose")],
            last_color_ns=100_000_000,
        )
        self.assertIsNone(result)

    def test_rejects_unsynchronised_depth(self):
        self.assertIsNone(
            self.select(
                [sample(100, "rgb")],
                [sample(150, "depth")],
                [sample(100, "pose")],
            )
        )

    def test_a_color_frame_never_uses_the_previous_30hz_depth(self):
        self.assertIsNone(
            self.select(
                [sample(133.3, "current rgb")],
                [sample(100.0, "previous depth")],
                [sample(140.0, "future pose")],
                rgbd_tolerance_ns=5_000_000,
            )
        )

    def test_rejects_a_pose_too_far_from_the_image(self):
        self.assertIsNone(
            self.select(
                [sample(200, "rgb")],
                [sample(200, "depth")],
                [sample(10, "pose")],
            )
        )

    def test_waits_for_a_pose_that_brackets_the_image(self):
        self.assertIsNone(
            self.select(
                [sample(200, "rgb")],
                [sample(200, "depth")],
                [sample(190, "only a past pose")],
            )
        )

    def test_a_far_future_pose_does_not_validate_a_stale_past_pose(self):
        self.assertIsNone(
            self.select(
                [sample(200, "rgb")],
                [sample(200, "depth")],
                [sample(190, "past pose"), sample(400, "far future pose")],
            )
        )


class MatchStabilityTest(unittest.TestCase):
    def test_requires_consecutive_consistent_matches(self):
        gate = MatchStability(required_frames=3, max_jump_m=0.01)
        self.assertFalse(gate.observe("apple", (0.50, 0.00, 0.05)))
        self.assertFalse(gate.observe("apple", (0.505, 0.00, 0.05)))
        self.assertTrue(gate.observe("apple", (0.507, 0.00, 0.05)))

    def test_a_large_jump_restarts_the_count(self):
        gate = MatchStability(required_frames=2, max_jump_m=0.01)
        self.assertFalse(gate.observe("apple", (0.50, 0.00, 0.05)))
        self.assertFalse(gate.observe("apple", (0.55, 0.00, 0.05)))
        self.assertTrue(gate.observe("apple", (0.552, 0.00, 0.05)))

    def test_a_different_class_restarts_the_count(self):
        gate = MatchStability(required_frames=2, max_jump_m=0.01)
        self.assertFalse(gate.observe("apple", (0.50, 0.00, 0.05)))
        self.assertFalse(gate.observe("orange", (0.50, 0.00, 0.05)))

    def test_pairwise_drift_does_not_form_a_stable_cluster(self):
        gate = MatchStability(required_frames=3, max_jump_m=0.02)
        self.assertFalse(gate.observe("apple", (0.500, 0.00, 0.05)))
        self.assertFalse(gate.observe("apple", (0.519, 0.00, 0.05)))
        self.assertFalse(gate.observe("apple", (0.538, 0.00, 0.05)))

    def test_a_long_frame_gap_restarts_the_window(self):
        gate = MatchStability(
            required_frames=2,
            max_jump_m=0.01,
            max_gap_ns=200_000_000,
        )
        self.assertFalse(
            gate.observe("apple", (0.50, 0.00, 0.05), stamp_ns=100_000_000)
        )
        self.assertFalse(
            gate.observe("apple", (0.50, 0.00, 0.05), stamp_ns=500_000_000)
        )
        self.assertTrue(
            gate.observe("apple", (0.50, 0.00, 0.05), stamp_ns=600_000_000)
        )


class AsyncResultGateTest(unittest.TestCase):
    def test_only_the_same_request_and_generation_is_active(self):
        self.assertTrue(result_matches_session("apple:1", 7, "apple:1", 7))

    def test_cancelled_session_rejects_a_late_result(self):
        self.assertFalse(result_matches_session("apple:1", 7, None, 8))

    def test_a_previous_generation_cannot_enter_a_new_session(self):
        self.assertFalse(result_matches_session("apple:1", 7, "apple:2", 9))
        self.assertFalse(result_matches_session("apple:1", 7, "apple:1", 9))


if __name__ == "__main__":
    unittest.main()
